// A missao completa no carro STS: a MESMA logica que o simulador testa (seguidor/robo.h) sobre o carro de verdade (carro_real.h).
// Sem MPU: o rumo vem dos encoders das rodas com params::kTrackMm (a bitola EFETIVA, medida com o env `bitola`; sem ela os giros erram). O braco/garra ainda nao sao acionados.
//
// Seguranca: o carro NAO anda ao ligar. Console serial (115200):  g [segundos]  anda (padrao 60 s, para sozinho)   x  para e reinicia   s  imprime o estado
#include <Arduino.h>
#include <SCServo.h>
#include <Wire.h>

#include "carro_real.h"
#include "hal/drive.h"
#include "hal/sensors.h"
#include "params.h"
#include "pins.h"

namespace {
SMS_STS st;
hal::StsDrive drive(st);
seguidor::BarraRC barra;
hal::Ultrasonic sonar;
hal::ColorSensor corChao, corFrente;
uint8_t pinosBarra[seguidor::kSensores];

constexpr float kDtMs = 5.0f;   // um ciclo de leitura + decisao: o mesmo do simulador

robo::CarroReal carro(drive, barra, sonar, corChao, corFrente, kDtMs);
seguidor::Robo<robo::CarroReal>* fw = nullptr;
bool andando = false;
unsigned long fimMs = 0;

void parouTudo() {
  drive.setSpeeds(0.0f, 0.0f);
  carro.parar();
}

// Chamado a cada ciclo (tambem dentro dos giros e avancos que bloqueiam): 'x' para as rodas e reinicia, o tempo maximo tambem.
void vigiar() {
  while (Serial.available()) {
    const char ch = Serial.read();
    if (ch == 'x') { parouTudo(); Serial.println("PARADO (x): reiniciando"); delay(100); ESP.restart(); }
    if (ch == 's' && fw) Serial.printf("estado: entregues %d, coletados %d, pontos %d, cruzamentos %d, rumo %.1f\n", fw->nav.entregues, fw->nav.coletados, fw->nav.pontos, fw->cruzamentos, fw->rumo);
  }
  if (andando && millis() > fimMs) { parouTudo(); Serial.println("TEMPO ACABOU: reiniciando"); delay(100); ESP.restart(); }
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
  delay(300);
  hal::beginStsBus(st);
  drive.begin();
  parouTudo();
  hal::beginBuses();
  sonar.begin();
  Serial.printf("TCS chao (Wire): %s | TCS frente (Wire1): %s\n", corChao.begin(Wire) ? "ok" : "FALHOU", corFrente.begin(Wire1) ? "ok" : "FALHOU");
  for (int i = 0; i < seguidor::kSensores; i++) pinosBarra[i] = pins::kLine[i];
  seguidor::iniciar(barra, pinosBarra);
  if (!seguidor::aplicarLimiar(barra, Serial)) {
    Serial.println("Poucos sensores da barra vivos: nao anda.");
    for (;;) delay(1000);
  }
  preparar();
  Serial.printf("pronto. bitola supoe %.0f mm. g [s] anda, x para, s estado. O carro esta PARADO.\n", params::kTrackMm);
}

void loop() {
  if (!andando) {
    if (Serial.available()) {
      const char ch = Serial.peek();
      if (ch == 'g') {
        const String linha = Serial.readStringUntil('\n');
        const int s = linha.length() > 1 ? linha.substring(1).toInt() : 60;
        fimMs = millis() + 1000UL * (s > 0 ? s : 60);
        andando = true;
        Serial.printf("ANDANDO por %d s (x para parar)\n", s > 0 ? s : 60);
      } else {
        vigiar();
      }
    }
    return;
  }
  fw->passo(carro);
}
