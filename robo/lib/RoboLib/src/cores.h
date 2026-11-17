#pragma once
// Pure logic (no Arduino.h): floor/cube colour from TCS34725 raw r, g, b, clear, by distance to a table of reference readings.
// Rafael reads this: keep the colour ORDER and the comments next to the table.
#include <math.h>
#include <stdint.h>


enum Cor { BRANCO, PRETO, VERDE, AMARELO, AZUL, VERMELHO, NUM_CORES, DESCONHECIDA };
constexpr const char* kNomesCores[] = {"BRANCO", "PRETO", "VERDE", "AMARELO", "AZUL", "VERMELHO"};

struct RefCor {
  uint16_t r, g, b, c;
};

// Measured 2026-11-09 with the floor TCS34725 as mounted on the car, over the arena colors (bringup/robo `s`: chao rgbc). The earlier teammate table read brighter than this sensor and
// could not tell VERDE from PRETO: re-measure here whenever the sensor height, LED or lighting changes. VERDE is the mean of two readings in different spots.
constexpr RefCor kCoresRef[NUM_CORES] = {
    // r    g    b    c
    {112, 75, 51, 231},  // BRANCO
    {28, 18, 13, 57},    // PRETO
    {25, 21, 13, 56},    // VERDE
    {98, 57, 29, 177},   // AMARELO
    {25, 18, 15, 56},    // AZUL
    {58, 30, 19, 102},   // VERMELHO
};
constexpr float kDistanciaMaxima = 0.15f;  // acima disso a cor fica DESCONHECIDA (nada se parece com a leitura)
// VERDE starts the match and is masked on the bar, so it is held to a much tighter distance. Measured 2026-11-09: the real start zone reads 0.027-0.046 from the VERDE reference; the car held
// over a table or a hand reads 0.066-0.09 (same tone, twice as bright) and used to start the match by itself.
constexpr float kDistanciaMaximaVerde = 0.055f;
// Classification is by PROXIMITY and fully RELATIVE: the table holds raw readings, but only ratios are compared (r, g, b as fractions of their sum, clear as a fraction of the
// BRANCO reference), so the same table survives a change of overall light. The nearest reference wins; a repeat of two readings in a row (filtrarCor) is what guards against noise.

// Squared distance between a reading and one reference: chroma (r, g, b as fractions of their sum) weighs 4x, brightness is the LOG RATIO of the clear channels (a ratio, so a white
// that reads 20% dimmer in a shadow is as far from its reference as one that reads 20% brighter) weighing 0.05x. Chroma alone cannot tell BRANCO from PRETO (same tone), so brightness
// stays, but it must not outvote the tone: with a heavier weight a shaded white became AMARELO or VERMELHO (measured on the car, 2026-11-09).
inline float distanciaCor(uint16_t r, uint16_t g, uint16_t b, uint16_t c, const RefCor& ref) {
  const float soma = static_cast<float>(r) + g + b;
  const float somaRef = static_cast<float>(ref.r) + ref.g + ref.b;
  const float dr = r / soma - ref.r / somaRef;
  const float dg = g / soma - ref.g / somaRef;
  const float db = b / soma - ref.b / somaRef;
  const float dl = logf((c < 1 ? 1.0f : static_cast<float>(c)) / static_cast<float>(ref.c));
  return 4.0f * (dr * dr + dg * dg + db * db) + 0.05f * dl * dl;
}

// Nearest reference colour by relative distance, or DESCONHECIDA when nothing is within kDistanciaMaxima or the reading is empty.
inline Cor classificarCor(uint16_t r, uint16_t g, uint16_t b, uint16_t c, const RefCor* refs = kCoresRef,
                          float distanciaMaxima = kDistanciaMaxima) {
  if (static_cast<float>(r) + g + b < 1.0f) return DESCONHECIDA;
  Cor melhor = DESCONHECIDA;
  float menor = 1e9f;
  for (int i = 0; i < NUM_CORES; ++i) {
    const float d = distanciaCor(r, g, b, c, refs[i]);
    if (d < menor) {
      menor = d;
      melhor = static_cast<Cor>(i);
    }
  }
  const float limite = melhor == VERDE ? fminf(distanciaMaxima, kDistanciaMaximaVerde) : distanciaMaxima;
  return menor <= limite ? melhor : DESCONHECIDA;
}

inline const char* nomeCor(Cor cor) { return cor < NUM_CORES ? kNomesCores[cor] : "DESCONHECIDA"; }

// A colour has to repeat in two readings in a row before it is believed. Returns the accepted colour.
inline Cor filtrarCor(Cor nova, Cor& anterior, Cor aceita) {
  const Cor resultado = (nova == anterior) ? nova : aceita;
  anterior = nova;
  return resultado;
}

