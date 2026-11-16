// Perfil do carro real (params.h -> Ajustes) e preenchimento de sensor morto, no PC:
//   g++ -std=c++17 -Wall -Isrc -I../include test/teste_perfil_carro.cpp src/seguidor/decisao.cpp -o /tmp/teste_perfil && /tmp/teste_perfil
#include <cassert>
#include <cmath>
#include <cstdio>

#include "seguidor/perfil_carro.h"

using namespace seguidor;

static void barra(bool p[kSensores], const char* padrao) {
  for (int i = 0; i < kSensores; i++) p[i] = padrao[i] == '#';
}

int main() {
  { // sensor morto copia o vizinho vivo mais perto (S7 e S8 mortos: os dois copiam S6)
    bool p[kSensores];
    barra(p, ".....#..");
    preencherMortos(p, 0b11000000);
    assert(p[6] && p[7]);
    barra(p, ".....#.#");   // o valor do morto e descartado: o vizinho manda
    preencherMortos(p, 0b11000000);
    assert(p[7] && p[6]);
    barra(p, "......#.");   // so o S7 aceso (morto, mas lido como lixo): S6 apagado manda
    preencherMortos(p, 0b11000000);
    assert(!p[6] && !p[7]); }
  { // morto no meio: copia o mais perto, empate vai para a direita
    bool p[kSensores];
    barra(p, "..#.##..");
    preencherMortos(p, 0b00001000);   // S4 morto: vizinhos S3 (aceso) e S5 (aceso)
    assert(p[3]);
    barra(p, "..#..#..");
    preencherMortos(p, 0b00001000);   // S3 aceso, S5 apagado: o da direita (S5) ganha o empate
    assert(!p[3]); }
  { // sem mortos nada muda
    bool p[kSensores];
    barra(p, "#.#.#.#.");
    preencherMortos(p, 0);
    assert(p[0] && !p[1] && p[2] && !p[3] && p[4] && !p[5] && p[6] && !p[7]); }
  { // o perfil do carro sai de params.h
    const Ajustes a = ajustesDoCarro();
    assert(a.sensoresMortos == params::kLineDeadMask);
    assert(std::fabs(a.bitolaMm - params::kTrackMm) < 1e-3f);
    assert(std::fabs(a.barraAFrenteMm - params::kAxleToSensorMm) < 1e-3f);
    assert(a.zonaMortaPwm == 0); }
  { // a busca mede o giro pelo tempo: os graus por segundo vem da fisica (w = 2 v / bitola), nao do chute de 110
    const Ajustes a = normalizar(ajustesDoCarro());
    const float velRodaMmS = a.velBuscaExterna * kMmSPorPwm;
    const float esperado = 2.0f * velRodaMmS / params::kTrackMm * 57.29578f;
    assert(std::fabs(a.grausPorSegundoBusca - esperado) < 0.5f);
    assert(a.grausPorSegundoBusca < 110.0f);   // o carro real gira mais devagar que o chute antigo: 360 graus de busca sao mais que 3,3 s
    assert(std::fabs(buscaGiroMs(a) / 1000.0f * a.grausPorSegundoBusca - a.buscaGiroGraus) < 1.0f); }   // o tempo do giro da busca fecha o angulo pedido
  { // a lacuna e uma distancia: o carro, andando reto, atravessa kLacunaMm antes de comecar a girar
    const Ajustes a = normalizar(ajustesDoCarro());
    const float andaMm = a.velReto * kMmSPorPwm * a.lacunaMs / 1000.0f;
    assert(std::fabs(andaMm - kLacunaMm) < 2.0f); }
  { // o cruzamento supoe a velocidade de reta do carro
    const Ajustes a = normalizar(ajustesDoCarro());
    assert(std::fabs(cruzamentoDoCarro(a).velMmS - a.velReto * kMmSPorPwm) < 1e-3f); }
  std::puts("perfil do carro: ok");
}
