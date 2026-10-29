#pragma once
// SeguidorRC: seguidor de linha preta sem PID.
//   sensores_rc.h  le a barra de 8 sensores RC e calibra       (usa pinos)
//   decisao.h      escolhe a velocidade das rodas              (so conta, nao usa pinos nem millis())
//   giro_cruzamento.h confirma o cruzamento girando 90 graus para cada lado e vendo se ha rua dos dois lados (so conta)
//   exploracao.h   cruzamentos ja visitados (relativos) para virar para o lado menos explorado (so conta)
//   alvo.h         o que o eco do ultrassom e: cubo na rua, oponente (eco que se mexe), cor do cubo (so conta)
//   borda.h        S1 e S8 no preto numa linha VERMELHA ou AZUL e a borda do mapa, nao cruzamento (so conta)
//   cruzamento.h   detecta o cruzamento (S1 e S8 no preto)     (so conta, nao usa pinos nem millis())
//   saude.h        marca sensor preto por tempo demais           (so conta, nao usa pinos nem millis())
//   inercial.h     MPU6050: rumo ancorado na grade, levantado/reposicionado, travado, colisao, lacuna (so conta)
//   calibracao.h   mede zona morta e graus por segundo girando no eixo com o giroscopio (so conta)
//   ponte_h.h      manda a velocidade para a ponte TB6612FNG   (usa pinos)
// Cada peca funciona sozinha; o sketch liga uma na outra (veja examples/SeguidorBasico).
#include "seguidor/alvo.h"
#include "seguidor/borda.h"
#include "seguidor/calibracao.h"
#include "seguidor/cruzamento.h"
#include "seguidor/decisao.h"
#include "seguidor/exploracao.h"
#include "seguidor/giro_cruzamento.h"
#include "seguidor/inercial.h"
#include "seguidor/ponte_h.h"
#include "seguidor/saude.h"
#include "seguidor/sensores_rc.h"
