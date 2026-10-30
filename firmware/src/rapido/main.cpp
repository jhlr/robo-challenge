// Minimal line follower for the STS car: P+D on the line position, nothing else (no junctions, no map). Built to run TODAY on the bar as wired:
// S7/S8 (GPIO35/34) are input-only, so the RC driver marks them dead and the controller uses the working sensors only (S1..S6).
// Everything is tunable and reversible over Bluetooth (console_ble.py): wheel directions are still unmeasured, so `invesq` / `invdir` / `espelhar` exist.
// Updates go over BLE too (ota_ble.py), so this app keeps the OTA service first in setup().
#include <Arduino.h>
#include <seguidor/depuracao_ble.h>
#include <seguidor/ota_ble.h>
#include <seguidor/sensores_rc.h>

#include "core/line.h"
#include "core/pid.h"
#include "hal/drive.h"
#include "params.h"
#include "pins.h"

#ifndef OTA_SEGREDO
#define OTA_SEGREDO "troque-este-segredo"   // same as tools/ota_ble.py --segredo
#endif

namespace {
using seguidor::kSensores;

SMS_STS st;
hal::StsDrive drive(st);
seguidor::BarraRC barra;
seguidor::DepuracaoBLE ble;
seguidor::OtaBLE ota;
seguidor::Tee dbg(Serial, ble);

uint8_t barPins[kSensores];

// Tunables (BLE commands). Speeds are fractions of full STS speed (params::kStsMaxSteps).
float baseSpeed = 0.30f;      // straight-line speed
float searchSpeed = 0.25f;    // spin speed when the line is lost
float maxSpeed = 0.60f;       // hard cap per wheel
int signLeft = +1, signRight = +1;  // flip a wheel that turns the wrong way
bool mirror = false;          // flip left/right of the bar (S1 is assumed to be the LEFT end)
bool running = false;
core::Pid pid(1.0f, 0.0f, 0.12f, 2.0f);

int lastSide = +1;            // where the line was last seen: -1 left, +1 right
uint32_t lostSince = 0;
constexpr uint32_t kGiveUpMs = 3000;

float clampSpeed(float v) { return constrain(v, -maxSpeed, maxSpeed); }

void drive2(float left, float right) { drive.setSpeeds(clampSpeed(left) * signLeft, clampSpeed(right) * signRight); }

void stop() {
  running = false;
  drive.setSpeeds(0.0f, 0.0f);
}

// Does each wheel servo answer on the bus? (-1 = no reply: wiring, jumper, or servo power)
void pingWheels() {
  const int l = st.Ping(params::kIdLeft), r = st.Ping(params::kIdRight);
  dbg.printf("STS esquerda (id %u): %s | direita (id %u): %s\n", params::kIdLeft, l >= 0 ? "responde" : "SEM RESPOSTA", params::kIdRight, r >= 0 ? "responde" : "SEM RESPOSTA");
}

void show() {
  dbg.printf("%s | v=%.2f kp=%.2f kd=%.2f | invesq=%d invdir=%d espelhar=%d\n", running ? "ANDANDO" : "PARADO", baseSpeed, pid.kp, pid.kd, signLeft < 0, signRight < 0, mirror);
}

void onCommand(const String& line) {
  String l = line;
  l.trim();
  l.toLowerCase();
  const int sp = l.indexOf(' ');
  const String cmd = sp < 0 ? l : l.substring(0, sp);
  const float arg = sp < 0 ? 0.0f : l.substring(sp + 1).toFloat();
  if (cmd == "parar" || cmd == "p") { stop(); dbg.println("ok: parado"); }
  else if (cmd == "seguir" || cmd == "s") { pid.reset(); lostSince = 0; running = true; dbg.println("ok: andando"); }
  else if (cmd == "v") baseSpeed = constrain(arg, 0.0f, maxSpeed);
  else if (cmd == "kp") pid.kp = arg;
  else if (cmd == "kd") pid.kd = arg;
  else if (cmd == "invesq") signLeft = -signLeft;
  else if (cmd == "invdir") signRight = -signRight;
  else if (cmd == "espelhar") mirror = !mirror;
  else if (cmd == "ping") pingWheels();
  else if (cmd == "giro") {   // wheel test, blocking: both wheels forward 1.5 s, then stop (car in the air!)
    running = false;
    drive2(0.25f, 0.25f);
    delay(1500);
    stop();
    dbg.println("ok: giro de teste terminado");
  }
  else if (cmd != "estado") dbg.println("comandos: seguir | parar | ping | giro | v <0..0.6> | kp <x> | kd <x> | invesq | invdir | espelhar | estado");
  show();
}
}  // namespace

void setup() {
  Serial.begin(115200);
  ble.iniciar("SeguidorRC", onCommand);
  ota.iniciar(ble.servidor(), OTA_SEGREDO);   // FIRST in setup: any firmware that reaches this line can still be replaced over BLE
  ble.anunciar();

  hal::beginStsBus(st);
  drive.begin();
  pingWheels();
  for (int i = 0; i < kSensores; i++) barPins[i] = pins::kLine[i];
  seguidor::iniciar(barra, barPins);

  // Slide the car over the line by hand for 5 s (motors off). Needs 6 working sensors: S7/S8 are dead on this wiring, so S1..S6 must all react.
  while (!seguidor::calibrar(barra, dbg)) {
    dbg.println("Calibracao INVALIDA (menos de 6 sensores reagiram). De novo em 2 s...");
    const uint32_t until = millis() + 2000;
    while (millis() < until) { ble.atualizar(); ota.atualizar(); }
  }
  dbg.println("Calibrado. Coloque o carro na linha: parte em 3 s. `parar` pelo Bluetooth interrompe.");
  const uint32_t until = millis() + 3000;
  while (millis() < until) { ble.atualizar(); ota.atualizar(); }
  pid.reset();
  running = true;
  show();
}

void loop() {
  ble.atualizar();
  ota.atualizar();
  if (ota.ativo()) { stop(); return; }   // firmware update in progress

  static uint32_t lastUs = micros();
  const uint32_t nowUs = micros();
  const float dt = max((nowUs - lastUs) * 1e-6f, 1e-3f);
  lastUs = nowUs;

  seguidor::ler(barra);
  float v[kSensores];
  int n = 0;
  for (int i = 0; i < kSensores; i++)
    if (barra.ok[i]) v[n++] = barra.preto[i] ? 1.0f : 0.0f;   // dead sensors drop out: the working ones span -1..+1 on their own
  const core::LineState ln = core::readLine(v, n);
  const float pos = mirror ? -ln.pos : ln.pos;               // + = line to the right

  float left = 0.0f, right = 0.0f;
  if (running) {
    if (ln.lost) {
      if (lostSince == 0) lostSince = millis();
      if (millis() - lostSince > kGiveUpMs) {
        stop();
        dbg.println("!!! linha perdida ha 3 s: parado");
      } else {
        left = lastSide * searchSpeed;   // spin toward where the line was last seen
        right = -lastSide * searchSpeed;
      }
    } else {
      lostSince = 0;
      lastSide = pos >= 0.0f ? +1 : -1;
      const float corr = pid.step(pos, dt);   // line right => left wheel faster
      left = baseSpeed * (1.0f + corr);
      right = baseSpeed * (1.0f - corr);
    }
    if (running) drive2(left, right);
  }

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 250) {
    lastPrint = millis();
    char bar[kSensores + 1];
    for (int i = 0; i < kSensores; i++) bar[i] = !barra.ok[i] ? 'x' : (barra.preto[i] ? '#' : '.');
    bar[kSensores] = 0;
    dbg.printf("%s pos=%+.2f L=%+.2f R=%+.2f\n", bar, pos, left * signLeft, right * signRight);
  }
}
