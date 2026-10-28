#pragma once
// Pure logic (no Arduino.h): floor/cube colour from TCS34725 raw r, g, b, clear, by distance to a table of reference readings.
// Ported from the teammate's line-follower sketch. Rafael reads this: keep the colour ORDER and the comments next to the table.
#include <stdint.h>

namespace core {

enum Cor { BRANCO, PRETO, VERDE, AMARELO, AZUL, VERMELHO, NUM_CORES, DESCONHECIDA };
constexpr const char* kNomesCores[] = {"BRANCO", "PRETO", "VERDE", "AMARELO", "AZUL", "VERMELHO"};

struct RefCor {
  uint16_t r, g, b, c;
};

// Fill with the bring-up readout ({ r, g, b, c } over each colour); same numbers the teammate measured.
constexpr RefCor kCoresRef[NUM_CORES] = {
    // r    g    b    c
    {121, 93, 67, 243},  // BRANCO
    {36, 27, 20, 69},    // PRETO
    {38, 34, 24, 75},    // VERDE
    {127, 86, 48, 220},  // AMARELO
    {47, 40, 30, 89},    // AZUL
    {104, 44, 32, 150},  // VERMELHO
};
constexpr float kDistanciaMaxima = 0.15f;  // acima disso a cor fica DESCONHECIDA

// Squared distance between a reading and one reference: chroma (r, g, b as fractions of their sum) weighs 4x, brightness relative to BRANCO 0.5x.
inline float distanciaCor(uint16_t r, uint16_t g, uint16_t b, uint16_t c, const RefCor& ref, const RefCor& branco) {
  const float soma = static_cast<float>(r) + g + b;
  const float somaRef = static_cast<float>(ref.r) + ref.g + ref.b;
  const float dr = r / soma - ref.r / somaRef;
  const float dg = g / soma - ref.g / somaRef;
  const float db = b / soma - ref.b / somaRef;
  const float dl = static_cast<float>(c) / branco.c - static_cast<float>(ref.c) / branco.c;
  return 4.0f * (dr * dr + dg * dg + db * db) + 0.5f * dl * dl;
}

// Closest reference colour, or DESCONHECIDA when nothing is close enough or the reading is empty.
inline Cor classificarCor(uint16_t r, uint16_t g, uint16_t b, uint16_t c, const RefCor* refs = kCoresRef,
                          float distanciaMaxima = kDistanciaMaxima) {
  if (static_cast<float>(r) + g + b < 1.0f) return DESCONHECIDA;
  Cor melhor = DESCONHECIDA;
  float menor = 1e9f;
  for (int i = 0; i < NUM_CORES; ++i) {
    const float d = distanciaCor(r, g, b, c, refs[i], refs[BRANCO]);
    if (d < menor) {
      menor = d;
      melhor = static_cast<Cor>(i);
    }
  }
  return menor <= distanciaMaxima ? melhor : DESCONHECIDA;
}

inline const char* nomeCor(Cor cor) { return cor < NUM_CORES ? kNomesCores[cor] : "DESCONHECIDA"; }

// A colour has to repeat in two readings in a row before it is believed (the teammate's filter). Returns the accepted colour.
inline Cor filtrarCor(Cor nova, Cor& anterior, Cor aceita) {
  const Cor resultado = (nova == anterior) ? nova : aceita;
  anterior = nova;
  return resultado;
}

}  // namespace core
