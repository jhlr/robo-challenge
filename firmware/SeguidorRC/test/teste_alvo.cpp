// g++ -std=c++17 -Wall -Isrc test/teste_alvo.cpp -o /tmp/teste_alvo && /tmp/teste_alvo
#include <cassert>
#include <cstdio>

#include "seguidor/alvo.h"

using namespace seguidor;

int main() {
  assert(ruaTemCubo(100.0f) && ruaTemCubo(200.0f) && !ruaTemCubo(327.0f) && !ruaTemCubo(500.0f) && !ruaTemCubo(-1.0f) && !ruaTemCubo(0.0f));   // 327 mm = cubo da rua DEPOIS da esquina
  // o eco diz EM QUAL rua da linha reta esta o cubo (ate ~3 ruas a frente)
  assert(ruaDoEco(60.0f, 0.0f) == 0);                      // cubo na rua logo a frente
  assert(ruaDoEco(330.0f, 0.0f) == 1 && ruaDoEco(400.0f, 0.0f) == 1);   // alem do proximo cruzamento: a rua seguinte
  assert(ruaDoEco(640.0f, 0.0f) == 2 && ruaDoEco(700.0f, 0.0f) == 2);   // duas ruas a frente
  assert(ruaDoEco(250.0f, 0.0f) == -1);                    // perto do cruzamento: ambiguo, nao e cubo de rua
  assert(ruaDoEco(-1.0f, 0.0f) == -1);
  assert(ruaDoEco(330.0f - 150.0f, 150.0f) == 1);          // andando: a mesma rua 150 mm depois, o eco e 150 mm menor
  assert(ultimaRuaVistaVazia(0.0f, 900.0f) == 2);          // sem eco ate 900 mm: as 3 primeiras ruas estao vazias (0, 1 e 2)
  assert(ultimaRuaVistaVazia(0.0f, 150.0f) == -1 || ultimaRuaVistaVazia(0.0f, 150.0f) == 0);
  assert(primeiraRuaAindaAFrente(0.0f) == 0 && primeiraRuaAindaAFrente(250.0f) >= 1);
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
  assert(podeColetar(CorCubo::Preto, aj) && podeColetar(CorCubo::Marrom, aj) && aj.capacidade == 1);   // um cubo por viagem
  aj.coletavel[static_cast<int>(CorCubo::Verde)] = false;   // sem plataforma verde nesta partida: trata como branco
  assert(!podeColetar(CorCubo::Verde, aj));
  aj.coletavel[static_cast<int>(CorCubo::Branco)] = true;   // configuravel
  assert(podeColetar(CorCubo::Branco, aj));
  std::puts("teste_alvo: ok");
  return 0;
}
