#pragma once
#include <Arduino.h>

#include "decisao.h"   // kSensores

namespace seguidor {

// Barra de 8 sensores tipo RC: carrega o pino e mede o tempo de descarga. Branco = tempo curto | Preto = tempo longo.
// Os 8 pinos precisam ser menores que 32: um unico registrador (GPIO_IN_REG) le todos de uma vez.
struct BarraRC {
  // Ajustes
  unsigned int tempoMaxUs = 2500;   // valor do preto
  unsigned int faixaMinUs = 300;    // preto - branco minimo para o sensor valer
  int sensoresMinimos = 6;          // abaixo disso a calibracao e recusada

  // Preenchido pela biblioteca
  const uint8_t* pinos = nullptr;            // S1 (ponta esquerda) ... S8 (ponta direita)
  unsigned int leituras[kSensores] = {};     // tempo de cada sensor em us (0 a tempoMaxUs)
  unsigned int limiar[kSensores] = {};       // acima disso = preto (calculado na calibracao)
  bool ok[kSensores] = {};                   // false = nao reagiu na calibracao, ignorado
  bool preto[kSensores] = {};                // true = esse sensor ve a linha
  unsigned int tempoLeituraUs = 2500;        // so espera ate o maior limiar (calculado na calibracao)
  uint32_t mascara[kSensores] = {};          // bit de cada sensor no registrador de entrada
  uint32_t mascaraTodos = 0;
};

void iniciar(BarraRC& barra, const uint8_t pinos[kSensores]);

// 5 s deslizando o robo sobre a linha (motores parados). Escreve a tabela em `saida` (ex.: Serial).
// false = poucos sensores reagiram: nao deve andar assim.
bool calibrar(BarraRC& barra, Print& saida);

// Le os 8 de uma vez e atualiza barra.preto[].
void ler(BarraRC& barra);

}  // namespace seguidor
