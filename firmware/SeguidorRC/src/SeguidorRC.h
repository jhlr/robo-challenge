#pragma once
// SeguidorRC: seguidor de linha preta sem PID.
//   sensores_rc.h  le a barra de 8 sensores RC e calibra       (usa pinos)
//   decisao.h      escolhe a velocidade das rodas              (so conta, nao usa pinos nem millis())
//   ponte_h.h      manda a velocidade para a ponte TB6612FNG   (usa pinos)
// Cada peca funciona sozinha; o sketch liga uma na outra (veja examples/SeguidorBasico).
#include "seguidor/decisao.h"
#include "seguidor/ponte_h.h"
#include "seguidor/sensores_rc.h"
