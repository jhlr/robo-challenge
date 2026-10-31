#pragma once
#include <Arduino.h>
#include <QTRSensors.h>

#include "decisao.h"   // kSensores

namespace seguidor {

// Barra de 8 sensores QRE-8D (tipo RC): leitura e calibracao feitas pela biblioteca oficial Pololu QTRSensors; esta camada so mapeia S1..S8, ignora os mortos e decide preto/branco.
// GPIO34-39 sao SO DE ENTRADA: o modo RC precisa carregar o pino, entao esses sensores nao reagem e ficam de fora da QTRSensors.
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
  QTRSensors qtr;                            // so com os pinos que podem carregar
  uint8_t indice[kSensores] = {};            // posicao de cada sensor vivo na QTRSensors -> S1..S8
  uint8_t quantos = 0;
};

void iniciar(BarraRC& barra, const uint8_t pinos[kSensores]);

// 5 s deslizando o robo sobre a linha (motores parados). Escreve a tabela em `saida` (ex.: Serial).
// false = poucos sensores reagiram: nao deve andar assim.
bool calibrar(BarraRC& barra, Print& saida);

// Le os 8 de uma vez e atualiza barra.preto[].
void ler(BarraRC& barra);

}  // namespace seguidor
