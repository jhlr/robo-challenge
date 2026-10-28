#pragma once
// SeguidorRC: seguidor de linha preta sem PID.
//   sensores_rc.h  le a barra de 8 sensores RC e calibra       (usa pinos)
//   decisao.h      escolhe a velocidade das rodas              (so conta, nao usa pinos nem millis())
//   giro_cruzamento.h confirma o cruzamento girando 90 graus para cada lado e vendo se ha rua dos dois lados (so conta)
//   borda.h        S1 e S8 no preto numa linha VERMELHA ou AZUL e a borda do mapa, nao cruzamento (so conta)
//   cruzamento.h   detecta o cruzamento (S1 e S8 no preto)     (so conta, nao usa pinos nem millis())
//   saude.h        marca sensor preto por tempo demais           (so conta, nao usa pinos nem millis())
//   ponte_h.h      manda a velocidade para a ponte TB6612FNG   (usa pinos)
// Cada peca funciona sozinha; o sketch liga uma na outra (veja examples/SeguidorBasico).
#include "seguidor/borda.h"
#include "seguidor/cruzamento.h"
#include "seguidor/decisao.h"
#include "seguidor/giro_cruzamento.h"
#include "seguidor/ponte_h.h"
#include "seguidor/saude.h"
#include "seguidor/sensores_rc.h"
