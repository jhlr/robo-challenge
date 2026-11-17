#include "sensores.h"

#include <Arduino.h>
#include <Adafruit_TCS34725.h>
#include <Wire.h>

#include "config.h"

namespace {
// ---- sensor de cor TCS34725: um por barramento I2C ----
struct SensorCor {
  TwoWire* bus;
  Adafruit_TCS34725 chip{TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X};
  bool ok = false;

  void iniciar(TwoWire& b) { bus = &b; ok = chip.begin(TCS34725_ADDRESS, bus); }

  // Le os 8 registradores de uma vez (c, r, g, b). A leitura da biblioteca espera 25 ms a cada chamada; esta e instantanea.
  bool ler(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c) {
    if (!ok) ok = chip.begin(TCS34725_ADDRESS, bus);   // fio solto no boot se cura sozinho
    if (!ok) return false;
    bus->beginTransmission(TCS34725_ADDRESS);
    bus->write(TCS34725_COMMAND_BIT | 0x20 | TCS34725_CDATAL);
    uint8_t buf[8] = {};
    if (bus->endTransmission() != 0 || bus->requestFrom(static_cast<uint8_t>(TCS34725_ADDRESS), static_cast<uint8_t>(8)) != 8) { ok = false; return false; }
    for (uint8_t& x : buf) x = bus->read();
    c = buf[0] | (buf[1] << 8);
    r = buf[2] | (buf[3] << 8);
    g = buf[4] | (buf[5] << 8);
    b = buf[6] | (buf[7] << 8);
    if (r == 0 && g == 0 && b == 0 && c == 0) { ok = false; return false; }   // tudo zero = o barramento nao respondeu
    return true;
  }

  Cor cor() {
    uint16_t r, g, b, c;
    return ler(r, g, b, c) ? classificarCor(r, g, b, c) : DESCONHECIDA;
  }
};

SensorCor chao, frente;
}  // namespace

void sensoresIniciar() {
  for (const int pino : pinos::kLinha) pinMode(pino, INPUT);
  pinMode(pinos::kTrig, OUTPUT);
  digitalWrite(pinos::kTrig, LOW);
  pinMode(pinos::kEcho, INPUT);
  pinMode(pinos::kBuzzer, OUTPUT);
  pinMode(pinos::kLed, OUTPUT);
  Wire.begin(pinos::kChaoSda, pinos::kChaoScl);
  Wire1.begin(pinos::kFrenteSda, pinos::kFrenteScl);
  chao.iniciar(Wire);
  frente.iniciar(Wire1);
}

uint8_t linhaLer() {
  uint8_t mascara = 0;
  for (int i = 0; i < 8; i++)
    if (digitalRead(pinos::kLinha[i]) == LOW) mascara |= 1 << i;   // o QRE8D vai a LOW sobre o preto
  return mascara;
}

int distanciaMm() {
  digitalWrite(pinos::kTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinos::kTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinos::kTrig, LOW);
  const unsigned long ecoUs = pulseIn(pinos::kEcho, HIGH, 10000);   // 0 = sem eco em 10 ms (~1,7 m); sem eco nao trava o seguidor por 25 ms
  if (ecoUs == 0) return -1;
  return static_cast<int>(ecoUs * cfg::kSomMmPorUs / 2.0f + 0.5f);   // ida e volta: metade do caminho
}

Cor corChao() { return chao.cor(); }
Cor corFrente() { return frente.cor(); }
bool corChaoCrua(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c) { return chao.ler(r, g, b, c); }
bool corFrenteCrua(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c) { return frente.ler(r, g, b, c); }

void bipar(uint16_t ms) {
  digitalWrite(pinos::kBuzzer, HIGH);
  delay(ms);
  digitalWrite(pinos::kBuzzer, LOW);
}

void led(bool ligado) { digitalWrite(pinos::kLed, ligado ? HIGH : LOW); }
