// A missao completa no carro STS: a MESMA logica que o simulador testa (seguidor/robo.h) sobre o carro de verdade (carro_real.h).
// Sem MPU: o rumo vem dos encoders das rodas com params::kTrackMm (a bitola EFETIVA, medida com o env `bitola`; sem ela os giros erram). O ombro se mexe de verdade ao pegar e largar; a garra e STUB (hal/claw.h).
//
// Inicio: o carro NAO anda ao ligar. A partida comeca quando o chao le VERDE (zona de largada) por ~0,5 s seguidos, ou pelo comando g. Depois de x ou do fim do tempo
// ele fica DESARMADO (nao reinicia sozinho sobre o verde): so g, ou desligar e ligar de novo, arma o gatilho do verde. Console por USB (115200) E por Bluetooth LE ("RoboChallenge", Nordic UART: tools/console_ble.py ou nRF Connect):
//   g [segundos]  anda (padrao 60 s, para sozinho)   x  para e reinicia   s  estado e o que o boot achou   t  teste de reta (1,5 s a 30%, mostra o quanto cada roda andou)   b  tempo cru dos 8 sensores da barra (us)   h  ajuda
// Atualizacao de firmware por BLE (carro parado e sem console conectado):
//   python3 SeguidorRC/tools/ota_ble.py firmware/.pio/build/robo/firmware.bin --nome RoboChallenge --segredo robo-challenge-2026
#include <Arduino.h>
#include <SCServo.h>
#include <Wire.h>
#include <seguidor/depuracao_ble.h>
#include <seguidor/ota_ble.h>

#include "carro_real.h"
#include "core/arm.h"
#include "core/starttrigger.h"
#include <esp_system.h>
#include "hal/claw.h"
#include "hal/drive.h"
#include "hal/sensors.h"
#include "hal/shoulder.h"
#include "params.h"
#include "pins.h"

namespace {
SMS_STS st;
hal::StsDrive drive(st);
seguidor::BarraRC barra;
hal::Ultrasonic sonar;
hal::ColorSensor corChao, corFrente;
hal::Claw garra;
uint8_t pinosBarra[seguidor::kSensores];

seguidor::DepuracaoBLE ble;
seguidor::OtaBLE ota;
constexpr const char* kOtaSegredo = "robo-challenge-2026";   // o mesmo em ota_ble.py (--segredo); sem ele ninguem grava firmware por BLE
seguidor::Tee dbg(Serial, ble);   // tudo que o carro escreve vai ao USB E ao Bluetooth

constexpr float kDtMs = 5.0f;   // um ciclo de leitura + decisao: o mesmo do simulador

robo::CarroReal carro(drive, barra, sonar, corChao, corFrente, garra, kDtMs);
seguidor::Robo<robo::CarroReal>* fw = nullptr;
bool andando = false;
unsigned long fimMs = 0;
String boot;            // o que o boot achou: ninguem estava conectado por BLE, entao `s` repete
String linhaSerial;     // comando digitado no USB, ate o fim da linha
constexpr uint32_t kMagicParado = 0xC0FFEE01;
RTC_NOINIT_ATTR uint32_t paradoPorComando;   // sobrevive ao ESP.restart() (nao ao desligar): x ou fim do tempo desarmam o gatilho do verde
core::StartTrigger gatilhoVerde;
unsigned long ultimoGatilhoMs = 0;
String linhaBle;        // ultimo comando recebido por BLE (entregue pelo ble.atualizar())

void registrar(const String& texto) { boot += texto + "\n"; dbg.println(texto); }

void reiniciarDesarmado() { paradoPorComando = kMagicParado; delay(200); ESP.restart(); }

void iniciar(int segundos, const char* motivo) {
  fimMs = millis() + 1000UL * segundos;
  andando = true;
  dbg.printf("ANDANDO por %d s (%s; x para parar)\n", segundos, motivo);
}

void parouTudo() {
  drive.setSpeeds(0.0f, 0.0f);
  carro.parar();
}

// So vale com a garra levantada: abaixada, ela fica na frente do sonar. Le o ombro a cada 60 ms; posicao ilegivel conta como abaixada.
bool garraLevantada() {
  static unsigned long ultima = 0;
  static bool levantada = false;
  if (millis() - ultima >= 60) {
    ultima = millis();
    levantada = core::shoulderIsUp(shoulderTicks(), params::kShoulderTicksUp, params::kShoulderUpTolTicks);
  }
  return levantada;
}

void atender();   // trata os comandos pendentes (USB e BLE)

// Algo muito perto do sonar: da re; se o obstaculo veio junto, e uma mao segurando o carro no ar. Ai para as rodas e espera soltar (nao adianta girar motor nem ler sensor no ar).
// O tempo parado nao conta no limite da missao.
void guardaDaMao() {
  if (!garraLevantada()) { carro.soltarMao(); return; }
  core::HandStep passo = carro.passoMao();
  if (passo == core::HandStep::None) return;
  if (passo == core::HandStep::BackUp) {
    dbg.println("obstaculo muito perto: da re para ver se ele vem junto");
    carro.recuar(core::kHandBackUpMm);
    passo = carro.depoisDoRecuo();
    if (passo == core::HandStep::None) { dbg.println("ficou para tras: obstaculo fixo, segue"); return; }
  }
  parouTudo();
  dbg.println("MAO: o obstaculo veio junto na re, o carro esta no ar. Parado ate soltar");
  const unsigned long desde = millis();
  while (carro.passoMao() == core::HandStep::Hold) {
    ble.atualizar();
    atender();   // 'x' ainda para e reinicia
    delay(20);
  }
  fimMs += millis() - desde;
  dbg.println("solto: segue");
}

// Isola o hardware da logica: as duas rodas para frente, iguais, por 1,5 s. Se uma andar diferente da outra, ou para tras, o problema esta no sinal ou na asa da roda, nao na missao.
void testeReto() {
  parouTudo();
  const float e0 = drive.leftMm(), d0 = drive.rightMm();
  drive.setSpeeds(0.3f, 0.3f);
  const unsigned long t0 = millis();
  while (millis() - t0 < 1500) { delay(20); atender(); }
  drive.setSpeeds(0.0f, 0.0f);
  delay(300);
  dbg.printf("TESTE RETO 1,5 s a 30%%: esquerda %.0f mm, direita %.0f mm (esperado: iguais e positivos)\n", drive.leftMm() - e0, drive.rightMm() - d0);
}

// Tempo cru (us) de cada sensor da barra, com o tempo maximo cheio (nao cortado no limiar): serve para ver o branco e a linha e escolher o limiar. Coloque a barra sobre o branco e depois sobre a linha.
void lerBarraCrua() {
  barra.qtr.setTimeout(barra.tempoMaxUs);
  for (int k = 0; k < 3; k++) {
    seguidor::ler(barra);
    String linha = "barra us (limiar " + String(barra.limiarUs) + "):";
    for (int i = 0; i < seguidor::kSensores; i++) linha += " S" + String(i + 1) + "=" + String(barra.leituras[i]) + (barra.ok[i] ? "" : "(morto)");
    dbg.println(linha);
    delay(100);
  }
  barra.qtr.setTimeout(barra.tempoLeituraUs);
}

void tratar(String linha) {
  linha.trim();
  linha.toLowerCase();
  if (linha.length() == 0) return;
  const char cmd = linha[0];
  if (cmd == 'x') { parouTudo(); dbg.println("PARADO (x): reiniciando desarmado"); reiniciarDesarmado(); }
  if (cmd == 's') {
    dbg.print(boot);
    dbg.printf("t=%lu ms | %s | gatilho do verde %s | ombro %d ticks (topo %d) | guarda da mao %s\n", millis(), andando ? "ANDANDO" : "PARADO", paradoPorComando == kMagicParado ? "DESARMADO" : "armado", shoulderTicks(), params::kShoulderTicksUp,
               garraLevantada() ? "ligada" : "desligada (garra abaixada)");
    dbg.printf("%s\n", carro.chaoTexto().c_str());
    if (fw) dbg.printf("entregues %d, coletados %d, pontos %d, cruzamentos %d, rumo %.1f\n", fw->nav.entregues, fw->nav.coletados, fw->nav.pontos, fw->cruzamentos, fw->rumo);
  } else if (cmd == 'b') {
    if (andando) { dbg.println("ja esta andando (x para parar)"); return; }
    lerBarraCrua();
  } else if (cmd == 't') {
    if (andando) { dbg.println("ja esta andando (x para parar)"); return; }
    testeReto();
  } else if (cmd == 'g') {
    if (andando) { dbg.println("ja esta andando (x para parar)"); return; }
    const int s = linha.length() > 1 ? linha.substring(1).toInt() : 60;
    paradoPorComando = 0;
    iniciar(s > 0 ? s : 60, "comando g");
  } else {
    dbg.println("g [s] anda | x para e reinicia | s estado | t teste de reta | b barra crua");
  }
}

// Comandos pendentes: USB (x vale na hora, sem Enter; o resto por linha) e BLE (linha inteira).
void atender() {
  while (Serial.available()) {
    const char ch = Serial.read();
    if (ch == 'x') tratar("x");
    else if (ch == '\n' || ch == '\r') { tratar(linhaSerial); linhaSerial = ""; }
    else linhaSerial += ch;
  }
  if (linhaBle.length()) { const String l = linhaBle; linhaBle = ""; tratar(l); }
}

// Uma linha a cada 500 ms enquanto anda: a barra (# = linha), o quanto cada roda andou, o rumo e o que foi mandado a cada roda.
void telemetria() {
  static unsigned long ultima = 0;
  if (millis() - ultima < 500) return;
  ultima = millis();
  bool preto[seguidor::kSensores];
  carro.ler(preto);
  char barraTxt[seguidor::kSensores + 1];
  for (int i = 0; i < seguidor::kSensores; i++) barraTxt[i] = preto[i] ? '#' : '.';
  barraTxt[seguidor::kSensores] = 0;
  dbg.printf("[%lu] barra %s | odo E %.0f D %.0f mm | rumo %.1f | alvo E %.0f D %.0f | cruz %d perdido %d busca %d | %s\n", millis(), barraTxt, carro.odoEMm(), carro.odoDMm(),
             fw ? fw->rumo : 0.0f, carro.roda(true), carro.roda(false), fw ? fw->cruzamentos : 0, fw ? fw->ticksPerdido : 0, fw ? fw->ticksBuscando : 0, fw ? fw->nav.ultimoPlano.c_str() : "");
}

// Chamado a cada ciclo (tambem dentro dos giros e avancos que bloqueiam): manda o BLE, atende comandos, guarda da mao e limite de tempo.
void vigiar() {
  if (!andando && paradoPorComando != kMagicParado && millis() - ultimoGatilhoMs >= 100) {   // gatilho: chao verde
    ultimoGatilhoMs = millis();
    const seguidor::Piso piso = carro.piso();
    static seguidor::Piso anterior = seguidor::kPisoBranco;
    if (piso != anterior) { anterior = piso; dbg.printf("[parado] %s\n", carro.chaoTexto().c_str()); }   // quem olha pelo BLE ve o chao mudar
    if (gatilhoVerde.update(piso == seguidor::kPisoVerde)) iniciar(core::kMatchSeconds, "chao verde");
  }
  ble.atualizar();
  ota.atualizar();
  if (ota.ativo()) { parouTudo(); return; }   // atualizando o firmware: carro parado
  atender();
  if (andando) guardaDaMao();
  if (andando) telemetria();
  if (andando && millis() > fimMs) { parouTudo(); dbg.println("TEMPO ACABOU: reiniciando desarmado"); reiniciarDesarmado(); }
}

void preparar() {
  seguidor::ConfigRobo cfg;
  cfg.dtMs = kDtMs;
  cfg.tauMs = 50.0;
  cfg.encoder = true;
  cfg.rumoPeloEncoder = true;
  cfg.ajustes.bitolaMm = params::kTrackMm;   // a bitola que o firmware supoe (a EFETIVA, ja com a derrapagem)
  cfg.ajustes = seguidor::normalizar(cfg.ajustes);
  fw = new seguidor::Robo<robo::CarroReal>(cfg);
  fw->ceder = vigiar;
  fw->ligarImu(carro);   // sem MPU: so zera o rumo e liga o ciclo por encoder
  fw->nav.iniciar();
  fw->nav.coleta.capacidade = 1;
  fw->portaoPendente = true;
}
}  // namespace

void setup() {
  Serial.begin(115200);
  if (esp_reset_reason() != ESP_RST_SW) paradoPorComando = 0;   // ligou de verdade (nao foi o nosso restart): arma o gatilho do verde
  ble.iniciar("RoboChallenge", [](const String& l) { linhaBle = l; });
  ota.iniciar(ble.servidor(), kOtaSegredo);   // antes de qualquer coisa que possa travar: um firmware que chega aqui ainda pode ser atualizado
  ble.anunciar();
  carro.saida(dbg);
  delay(300);
  hal::beginStsBus(st);
  if (!shoulderBegin(st)) registrar("ombro: NAO RESPONDE");
  else {   // sempre ao ligar: o topo e a posicao de partida conhecida
    const int hoje = shoulderTicks();
    registrar("ombro: " + String(hoje) + " ticks -> topo " + String(params::kShoulderTicksUp) + ": " +
              (shoulderGoTo(ShoulderLevel::Up) ? "no topo" : "NAO CHEGOU (travado, longe demais ou sem leitura): confira o braco antes de andar"));
  }
  drive.begin();
  garra.begin(pins::kClaw);
  parouTudo();
  hal::beginBuses();
  sonar.begin();
  registrar(String("TCS chao (Wire): ") + (corChao.begin(Wire) ? "ok" : "FALHOU") + " | TCS frente (Wire1): " + (corFrente.begin(Wire1) ? "ok" : "FALHOU"));
  for (int i = 0; i < seguidor::kSensores; i++) pinosBarra[i] = pins::kLine[i];
  seguidor::iniciar(barra, pinosBarra);
  if (!seguidor::aplicarLimiar(barra, dbg)) {
    registrar("Poucos sensores da barra vivos: nao anda.");
    for (;;) { ble.atualizar(); ota.atualizar(); delay(100); }   // sem barra nao anda, mas ainda aceita OTA
  }
  preparar();
  registrar("pronto. bitola supoe " + String(params::kTrackMm, 0) + " mm. " + (paradoPorComando == kMagicParado ? "DESARMADO (g ou desligar e ligar arma o verde)" : "ARMADO: chao verde inicia") + ". g [s] anda, x para, s estado. O carro esta PARADO.");
}

void loop() {
  if (!andando) {
    vigiar();
    return;
  }
  fw->passo(carro);
}
