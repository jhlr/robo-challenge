#pragma once
// Segurar o rumo numa lacuna da fita (a linha sumiu por pouco tempo): o rumo vem dos encoders das rodas (robo.h). Puro, so conta.
#include <math.h>

#include "decisao.h"

namespace seguidor {

// Corrige um comando de lacuna (a linha sumiu ha pouco) para manter o rumo em que ela foi perdida. rumo + = esquerda: se virou para a esquerda,
// gira no eixo para a direita (esquerda para frente, direita para tras): toda curva move as rodas em SENTIDOS OPOSTOS, nunca uma mais lenta arrastando.
// Dentro de `lacunaToleranciaGraus` anda reto na velocidade reta (nao repete o ultimo comando, que podia ser um giro).
inline void segurarRumoNaLacuna(Comando& c, float rumoGraus, float rumoAoPerderGraus, const Ajustes& aj) {
  const float erro = rumoGraus - rumoAoPerderGraus;
  if (fabsf(erro) <= aj.lacunaToleranciaGraus) { c.esq = c.dir = aj.velReto; return; }
  c.esq = erro > 0.0f ? aj.velCorrecaoLeve : -aj.velCorrecaoLeve;
  c.dir = -c.esq;
}

}  // namespace seguidor
