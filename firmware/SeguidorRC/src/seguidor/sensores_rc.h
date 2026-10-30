#pragma once
#include <Arduino.h>

#include "decisao.h"   // kSensores

namespace seguidor {

// Barra de 8 sensores tipo RC: carrega o pino e mede o tempo de descarga. Branco = tempo curto | Preto = tempo longo.
// Os pinos sao lidos de uma vez em dois registradores (GPIO0-31 e GPIO32-39). Pinos 34-39 sao SO DE ENTRADA: nao carregam o capacitor, o sensor nao reage.
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
  uint32_t mascaraBaixa[kSensores] = {};     // bit de cada sensor no registrador de GPIO0-31 (0 se o pino e 32+)
  uint32_t mascaraAlta[kSensores] = {};      // idem para GPIO32-39 (bit = pino - 32)
  uint32_t todosBaixa = 0, todosAlta = 0;
};

void iniciar(BarraRC& barra, const uint8_t pinos[kSensores]);

// 5 s deslizando o robo sobre a linha (motores parados). Escreve a tabela em `saida` (ex.: Serial).
// false = poucos sensores reagiram: nao deve andar assim.
bool calibrar(BarraRC& barra, Print& saida);

// Le os 8 de uma vez e atualiza barra.preto[].
void ler(BarraRC& barra);

}  // namespace seguidor
