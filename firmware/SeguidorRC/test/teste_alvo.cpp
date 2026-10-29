// g++ -std=c++17 -Wall -Isrc test/teste_alvo.cpp -o /tmp/teste_alvo && /tmp/teste_alvo
#include <cassert>
#include <cstdio>

#include "seguidor/alvo.h"

using namespace seguidor;

int main() {
  assert(ruaTemCubo(180.0f) && !ruaTemCubo(500.0f) && !ruaTemCubo(-1.0f) && !ruaTemCubo(0.0f));
  // cubo parado: andando 100 mm/s para ele, em 500 ms a distancia cai 50 mm
  assert(!ecoMovel(300.0f, 250.0f, 500.0f, 100.0f));
  // oponente vindo ao nosso encontro: a distancia cai mais do que o nosso movimento explica
  assert(ecoMovel(300.0f, 190.0f, 500.0f, 100.0f));
  // oponente indo embora: a distancia cai menos (ou sobe)
  assert(ecoMovel(300.0f, 330.0f, 500.0f, 100.0f));
  // robo parado e o eco mudou 60 mm: se mexeu
  assert(ecoMovel(400.0f, 340.0f, 500.0f, 0.0f));
  assert(!ecoMovel(400.0f, 395.0f, 500.0f, 0.0f));   // ruido de poucos mm nao conta
  assert(!ecoMovel(-1.0f, 300.0f, 500.0f, 0.0f));    // sem eco antes: nao sabe
  AjustesColeta aj;
  assert(!podeColetar(CorCubo::Branco, aj) && !podeColetar(CorCubo::Nenhuma, aj) && podeColetar(CorCubo::Azul, aj) && podeColetar(CorCubo::Verde, aj));
  aj.coletavel[static_cast<int>(CorCubo::Branco)] = true;   // configuravel
  assert(podeColetar(CorCubo::Branco, aj));
  std::puts("teste_alvo: ok");
  return 0;
}
