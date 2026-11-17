// Bench: measures the EFFECTIVE track width (what turns a wheel travel difference into degrees, skid included) with the car's axle on the CENTRE of a crossing.
// It spins in place for more than one turn, records the wheel travel difference (STS encoders) and the QRE-8D centre sensors, and takes the encoder difference between the
// SAME street seen twice, one full turn apart: that is exactly 360 degrees, so a few mm of axle offset from the crossing centre cancel out. effective track = difference / 2 pi.
// Put the printed value in params::kTrackMm (the firmware turns encoder differences into heading with it). `pio run -e bitola -t upload`, then send any key over serial.
#include <Arduino.h>
#include <QTRSensors.h>
#include <SCServo.h>

#include <vector>

#include "core/odo.h"
#include "hal/drive.h"
#include "params.h"
#include "pins.h"

namespace {
SMS_STS st;
hal::StsDrive drive(st);
QTRSensors qtr;
constexpr float kSpeed = 0.2f;
constexpr float kMaxSpinDeg = 1400.0f;     // by the placeholder params::kTrackMm; enough for >1.5 real turns while the real track is 100..250 mm
constexpr unsigned long kMaxMs = 20000;
constexpr int kCentreA = 3, kCentreB = 4;  // S4 and S5

struct Sample { float diffMm; float rcUs; };  // wheel travel difference (mm, left minus right) and S4+S5 charge time (us)
std::vector<Sample> samples;

float diffMm() { return drive.spinDeg() * params::kTrackMm * core::kPi / 180.0f; }  // spinDeg() is (mmL - mmR) / kTrackMm

void measure() {
  samples.clear();
  samples.reserve(3000);
  drive.resetTravel();
  const unsigned long start = millis();
  uint16_t raw[8];
  while (millis() - start < kMaxMs && fabsf(drive.spinDeg()) < kMaxSpinDeg) {
    drive.setSpeeds(-kSpeed, kSpeed);   // left backwards, right forwards: counter-clockwise
    qtr.read(raw, QTRReadMode::Off);
    samples.push_back({diffMm(), static_cast<float>(raw[kCentreA]) + raw[kCentreB]});
  }
  drive.setSpeeds(0.0f, 0.0f);
  Serial.printf("%u amostras em %lu ms, giro contado (pela bitola provisoria %.0f mm): %.0f graus\n", static_cast<unsigned>(samples.size()), millis() - start, params::kTrackMm, drive.spinDeg());

  float lo = 1e9f, hi = -1e9f;
  for (const Sample& s : samples) { lo = min(lo, s.rcUs); hi = max(hi, s.rcUs); }
  const float threshold = 0.5f * (lo + hi);
  Serial.printf("S4+S5: minimo %.0f us, maximo %.0f us, limiar %.0f us\n", lo, hi, threshold);
  if (hi - lo < 500.0f) { Serial.println(F("SEM CONTRASTE: os sensores nao viram a linha (eixo longe do cruzamento, barra alta ou sem leitura)")); return; }

  std::vector<float> mids;   // middle of each street passing under the centre sensors, in encoder mm
  bool inside = false;
  float entry = 0.0f;
  for (const Sample& s : samples) {
    const bool black = s.rcUs > threshold;
    if (!inside && black) { inside = true; entry = s.diffMm; }
    else if (inside && !black) { inside = false; mids.push_back(0.5f * (entry + s.diffMm)); }
  }
  Serial.printf("ruas vistas: %u (esperado 4 por volta)\n", static_cast<unsigned>(mids.size()));
  for (size_t i = 0; i < mids.size(); ++i) Serial.printf("  rua %u: %.1f mm\n", static_cast<unsigned>(i), mids[i]);
  if (mids.size() < 5) { Serial.println(F("MENOS DE 5 RUAS: nao fechou uma volta; confira o eixo no centro do cruzamento")); return; }

  float sum = 0.0f;
  int n = 0;
  for (size_t i = 0; i + 4 < mids.size(); ++i) { sum += fabsf(mids[i + 4] - mids[i]); ++n; }   // the same street, one full turn later
  const float turnMm = sum / n;
  Serial.printf("diferenca por volta (360 graus): %.1f mm  (%d medida%s)\n", turnMm, n, n > 1 ? "s" : "");
  Serial.printf(">>> BITOLA EFETIVA = %.1f mm  -> params::kTrackMm\n", turnMm / (2.0f * core::kPi));
}
}  // namespace

void setup() {
  Serial.begin(115200);
  hal::beginStsBus(st);
  drive.begin();
  qtr.setTypeRC();
  uint8_t linePins[pins::kLineCount];   // pins::kLine is int[]; QTRSensors wants uint8_t[]
  for (int i = 0; i < pins::kLineCount; ++i) linePins[i] = static_cast<uint8_t>(pins::kLine[i]);
  qtr.setSensorPins(linePins, pins::kLineCount);
  qtr.setTimeout(2500);
  Serial.printf("ping esq=%d dir=%d\n", st.Ping(params::kIdLeft), st.Ping(params::kIdRight));
  Serial.println(F("Ponha o EIXO das rodas no centro de um cruzamento (barra a ~3 mm do chao) e mande qualquer tecla: o carro gira mais de uma volta sozinho."));
}

void loop() {
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    measure();
    Serial.println(F("Mande qualquer tecla para repetir (a media de 2 a 3 medidas e melhor que uma)."));
  }
  delay(50);
}
