#pragma once
// Depuracao por Bluetooth LOW ENERGY (o ESP32-S3 NAO tem Bluetooth classico/SPP, so BLE): um "serial" por BLE no padrao Nordic UART Service (NUS), que o celular
// (nRF Connect, "Serial Bluetooth Terminal" no modo BLE, LightBlue) e o computador (tools/console_ble.py) entendem. So para ESP32: inclua com
//   #include <seguidor/depuracao_ble.h>      (ele NAO entra em SeguidorRC.h para nao exigir BLE em outras placas)
//
//   DepuracaoBLE ble;                 // e um Print: ble.printf(...), ble.println(...)
//   Tee dbg(Serial, ble);             // escreve nos dois: Monitor Serial (USB) E Bluetooth
//   ble.iniciar("SeguidorRC", aoReceberLinha);   // no setup
//   ble.atualizar();                  // no loop: manda o que esta na fila e entrega as linhas recebidas a aoReceberLinha (no loop, nao na tarefa do BLE)
//
// Sem ninguem conectado o que se escreve e descartado (nao enche a memoria). A fila de saida tem 4 KB; se encher (conexao lenta) as linhas MAIS NOVAS sao descartadas
// e a proxima mensagem avisa quantos bytes se perderam.
#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

namespace seguidor {

constexpr const char* kUuidServico = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";   // Nordic UART Service
constexpr const char* kUuidRx = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";        // o aplicativo ESCREVE aqui (comandos para o robo)
constexpr const char* kUuidTx = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";        // o robo NOTIFICA aqui (texto de depuracao)

class DepuracaoBLE : public Print, public BLEServerCallbacks, public BLECharacteristicCallbacks {
 public:
  using AoReceber = void (*)(const String& linha);

  void iniciar(const char* nome, AoReceber aoReceber) {
    aoReceber_ = aoReceber;
    BLEDevice::init(nome);
    BLEDevice::setMTU(185);   // pede um pacote maior; o aplicativo pode aceitar menos (o valor negociado e lido em atualizar(), pois onMtuChanged muda de assinatura entre as pilhas BLE)
    servidor_ = BLEDevice::createServer();
    servidor_->setCallbacks(this);
    BLEService* servico = servidor_->createService(kUuidServico);
    tx_ = servico->createCharacteristic(kUuidTx, BLECharacteristic::PROPERTY_NOTIFY);
    tx_->addDescriptor(new BLE2902());
    BLECharacteristic* rx = servico->createCharacteristic(kUuidRx, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    rx->setCallbacks(this);
    servico->start();
    BLEAdvertising* anuncio = BLEDevice::getAdvertising();
    anuncio->addServiceUUID(kUuidServico);
    anuncio->setScanResponse(true);
    BLEDevice::startAdvertising();
  }

  bool conectado() const { return conectado_; }

  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t* dados, size_t n) override {
    if (!conectado_) return n;   // ninguem ouvindo: descarta
    for (size_t i = 0; i < n; i++) {
      const size_t prox = (cabeca_ + 1) % kTam;
      if (prox == cauda_) { perdidos_++; continue; }   // fila cheia: perde o mais novo
      fila_[cabeca_] = dados[i];
      cabeca_ = prox;
    }
    return n;
  }

  // Chame no loop(): manda um pacote por chamada (nao bloqueia) e entrega as linhas recebidas.
  void atualizar() {
    if (conectado_ && cauda_ != cabeca_) {
      const uint16_t mtu = BLEDevice::getMTU();   // negociado com o aplicativo (muda depois de conectar)
      pacoteMax_ = mtu > 23 ? mtu - 3 : 20;
      if (perdidos_ > 0) {   // avisa o que se perdeu, antes do proximo texto
        char aviso[48];
        snprintf(aviso, sizeof aviso, "\n[BLE: %u bytes perdidos]\n", static_cast<unsigned>(perdidos_));
        perdidos_ = 0;
        for (const char* p = aviso; *p; p++) { const size_t prox = (cabeca_ + 1) % kTam; if (prox != cauda_) { fila_[cabeca_] = static_cast<uint8_t>(*p); cabeca_ = prox; } }
      }
      uint8_t pacote[180];
      size_t n = 0;
      const size_t maximo = pacoteMax_ < sizeof pacote ? pacoteMax_ : sizeof pacote;
      while (cauda_ != cabeca_ && n < maximo) { pacote[n++] = fila_[cauda_]; cauda_ = (cauda_ + 1) % kTam; }
      tx_->setValue(pacote, n);
      tx_->notify();
    }
    if (pendente_ && aoReceber_) {   // linhas completas recebidas na tarefa do BLE: entregues aqui, no loop
      String linha;
      portENTER_CRITICAL(&trava_);
      linha = recebido_;
      recebido_ = "";
      pendente_ = false;
      portEXIT_CRITICAL(&trava_);
      int ini = 0;
      for (int i = 0; i < static_cast<int>(linha.length()); i++) {
        if (linha[i] == '\n' || linha[i] == '\r') {
          if (i > ini) aoReceber_(linha.substring(ini, i));
          ini = i + 1;
        }
      }
      if (ini < static_cast<int>(linha.length())) {   // resto sem fim de linha: volta para esperar o resto
        portENTER_CRITICAL(&trava_);
        recebido_ = linha.substring(ini) + recebido_;
        portEXIT_CRITICAL(&trava_);
      }
    }
  }

  // BLEServerCallbacks
  void onConnect(BLEServer*) override {   // assinatura sem parametro: existe nas duas pilhas (Bluedroid e NimBLE)
    conectado_ = true;
    cabeca_ = cauda_ = 0;
    perdidos_ = 0;
  }
  void onDisconnect(BLEServer*) override {
    conectado_ = false;
    BLEDevice::startAdvertising();   // volta a ser achado
  }

  // BLECharacteristicCallbacks (escrita do aplicativo)
  void onWrite(BLECharacteristic* c) override {
    const String v = c->getValue();
    portENTER_CRITICAL(&trava_);
    recebido_ += v;
    if (recebido_.length() > 400) recebido_ = recebido_.substring(recebido_.length() - 400);
    pendente_ = true;
    portEXIT_CRITICAL(&trava_);
  }

 private:
  static constexpr size_t kTam = 4096;
  BLEServer* servidor_ = nullptr;
  BLECharacteristic* tx_ = nullptr;
  AoReceber aoReceber_ = nullptr;
  uint8_t fila_[kTam];
  volatile size_t cabeca_ = 0, cauda_ = 0;
  size_t pacoteMax_ = 20, perdidos_ = 0;
  volatile bool conectado_ = false, pendente_ = false;
  String recebido_;
  portMUX_TYPE trava_ = portMUX_INITIALIZER_UNLOCKED;
};

// Escreve em dois lugares ao mesmo tempo (ex.: Serial USB e Bluetooth).
class Tee : public Print {
 public:
  Tee(Print& a, Print& b) : a_(a), b_(b) {}
  size_t write(uint8_t c) override { a_.write(c); b_.write(c); return 1; }
  size_t write(const uint8_t* d, size_t n) override { a_.write(d, n); b_.write(d, n); return n; }

 private:
  Print& a_;
  Print& b_;
};

}  // namespace seguidor
