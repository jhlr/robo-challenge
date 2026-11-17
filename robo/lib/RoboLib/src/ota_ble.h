#pragma once
// Atualizacao do firmware por Bluetooth LE (OTA). Servico proprio no MESMO servidor BLE da depuracao (depuracao_ble.h e dona do BLEDevice):
//   DepuracaoBLE ble;  OtaBLE ota;
//   ble.iniciar("SeguidorRC", aoReceberComando);   // cria o servidor e o console
//   ota.iniciar(ble.servidor(), OTA_SEGREDO);      // acrescenta o servico de OTA
//   ble.anunciar();                                // so agora comeca a anunciar (com os dois servicos)
//   no loop():  ota.atualizar();  if (ota.ativo()) { parar(); return; }
// Mantenha esse trecho como a PRIMEIRA coisa do setup(): um firmware que chega ao setup() ainda aceita outra atualizacao (sem rollback, o resto do
// setup que travar so se resolve por USB). O computador envia com tools/ota_ble.py.
//
// Protocolo (tudo em little-endian). Controle (escrita COM resposta + notificacao) e dados (escrita SEM resposta):
//   controle <- INICIAR  0x01 | tamanho u32 | sha256 da imagem (32) | HMAC-SHA256(segredo, tamanho u32 || sha256) (32)
//   controle <- FIM      0x02          (confere o sha256 recebido, fecha a imagem, reinicia)
//   controle <- ABORTAR  0x03
//   dados    <- bytes da imagem, em ordem, em pacotes de ate MTU-3
//   controle -> PRONTO 0x10 | janela u32     o computador manda ate `janela` bytes e espera o ACK
//   controle -> ACK    0x11 | recebidos u32  bytes ja gravados (cumulativo)
//   controle -> OK     0x12 | 0              imagem conferida, reiniciando
//   controle -> ERRO   0x1F | motivo u32     (OtaErro)
// O HMAC impede que um aparelho qualquer ao alcance grave um firmware; o sha256 confere que a imagem chegou inteira. Sem ninguem enviando por
// kSemDadosMs a atualizacao e abortada e o robo volta a andar.
#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Update.h>
#include <mbedtls/md.h>
#include <mbedtls/sha256.h>

namespace seguidor {

constexpr const char* kUuidOta = "9B2A0001-6F1D-4C6E-8B3A-5D0F2C7E1A00";
constexpr const char* kUuidOtaControle = "9B2A0002-6F1D-4C6E-8B3A-5D0F2C7E1A00";
constexpr const char* kUuidOtaDados = "9B2A0003-6F1D-4C6E-8B3A-5D0F2C7E1A00";

enum class OtaErro : uint32_t { Protocolo = 1, Autenticacao = 2, Tamanho = 3, Gravacao = 4, Hash = 5, Fechamento = 6 };

class OtaBLE : public BLECharacteristicCallbacks {
 public:
  static constexpr uint32_t kJanela = 4096;          // bytes entre ACKs
  static constexpr uint32_t kSemDadosMs = 10000;     // sem nada recebido: aborta
  static constexpr size_t kHash = 32;

  void iniciar(BLEServer* servidor, const char* segredo) {
    segredo_ = segredo;
    BLEService* servico = servidor->createService(kUuidOta);
    controle_ = servico->createCharacteristic(kUuidOtaControle, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
    controle_->addDescriptor(new BLE2902());
    controle_->setCallbacks(this);
    BLECharacteristic* dados = servico->createCharacteristic(kUuidOtaDados, BLECharacteristic::PROPERTY_WRITE_NR);
    dados->setCallbacks(this);
    dados_ = dados;
    servico->start();
  }

  bool ativo() const { return ativo_; }

  // Chame no loop(): reinicia depois do OK e aborta a atualizacao que parou de receber.
  void atualizar() {
    if (reiniciarEm_ && millis() >= reiniciarEm_) ESP.restart();
    if (ativo_ && millis() - ultimoMs_ > kSemDadosMs) abortar();
  }

  void onWrite(BLECharacteristic* c) override {
    const uint8_t* d = c->getData();
    const size_t n = c->getLength();
    if (c == dados_) gravar(d, n);
    else comando(d, n);
  }

 private:
  void notificar(uint8_t codigo, uint32_t arg) {
    uint8_t m[5] = {codigo, static_cast<uint8_t>(arg), static_cast<uint8_t>(arg >> 8), static_cast<uint8_t>(arg >> 16), static_cast<uint8_t>(arg >> 24)};
    controle_->setValue(m, sizeof m);
    controle_->notify();
  }

  void erro(OtaErro e) {
    abortar();
    notificar(0x1F, static_cast<uint32_t>(e));
  }

  void abortar() {
    if (ativo_) {
      Update.abort();
      mbedtls_sha256_free(&sha_);
    }
    ativo_ = false;
  }

  static uint32_t u32(const uint8_t* p) { return p[0] | (p[1] << 8) | (p[2] << 16) | (static_cast<uint32_t>(p[3]) << 24); }

  bool autentico(const uint8_t* tamanho, const uint8_t* hash, const uint8_t* mac) const {
    uint8_t msg[4 + kHash], esperado[kHash];
    memcpy(msg, tamanho, 4);
    memcpy(msg + 4, hash, kHash);
    const mbedtls_md_info_t* info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (mbedtls_md_hmac(info, reinterpret_cast<const uint8_t*>(segredo_), strlen(segredo_), msg, sizeof msg, esperado) != 0) return false;
    uint8_t dif = 0;
    for (size_t i = 0; i < kHash; i++) dif |= esperado[i] ^ mac[i];   // tempo constante
    return dif == 0;
  }

  void comando(const uint8_t* d, size_t n) {
    if (n < 1) return;
    switch (d[0]) {
      case 0x01: {
        if (n != 1 + 4 + kHash + kHash) return erro(OtaErro::Protocolo);
        if (!autentico(d + 1, d + 5, d + 5 + kHash)) return erro(OtaErro::Autenticacao);
        abortar();
        tamanho_ = u32(d + 1);
        if (tamanho_ == 0 || !Update.begin(tamanho_)) return erro(OtaErro::Tamanho);   // begin falha se nao cabe na particao
        memcpy(hashEsperado_, d + 5, kHash);
        mbedtls_sha256_init(&sha_);
        mbedtls_sha256_starts(&sha_, 0);
        recebido_ = ultimoAck_ = 0;
        ultimoMs_ = millis();
        ativo_ = true;
        notificar(0x10, kJanela);
        break;
      }
      case 0x02: {
        if (!ativo_) return erro(OtaErro::Protocolo);
        if (recebido_ != tamanho_) return erro(OtaErro::Protocolo);
        uint8_t hash[kHash];
        mbedtls_sha256_finish(&sha_, hash);
        uint8_t dif = 0;
        for (size_t i = 0; i < kHash; i++) dif |= hash[i] ^ hashEsperado_[i];
        if (dif != 0) return erro(OtaErro::Hash);
        if (!Update.end(true)) return erro(OtaErro::Fechamento);   // confere o cabecalho da imagem e troca a particao de boot
        mbedtls_sha256_free(&sha_);
        ativo_ = false;
        notificar(0x12, 0);
        reiniciarEm_ = millis() + 600;   // da tempo do OK sair antes
        break;
      }
      case 0x03:
        abortar();
        break;
      default:
        break;
    }
  }

  void gravar(const uint8_t* d, size_t n) {
    if (!ativo_) return;
    ultimoMs_ = millis();
    if (recebido_ + n > tamanho_) return erro(OtaErro::Tamanho);
    if (Update.write(const_cast<uint8_t*>(d), n) != n) return erro(OtaErro::Gravacao);
    mbedtls_sha256_update(&sha_, d, n);
    recebido_ += n;
    if (recebido_ - ultimoAck_ >= kJanela || recebido_ == tamanho_) {
      ultimoAck_ = recebido_;
      notificar(0x11, recebido_);
    }
  }

  const char* segredo_ = "";
  BLECharacteristic* controle_ = nullptr;
  BLECharacteristic* dados_ = nullptr;
  mbedtls_sha256_context sha_;
  uint8_t hashEsperado_[kHash] = {};
  uint32_t tamanho_ = 0, recebido_ = 0, ultimoAck_ = 0;
  volatile bool ativo_ = false;
  volatile unsigned long ultimoMs_ = 0, reiniciarEm_ = 0;
};

}  // namespace seguidor
