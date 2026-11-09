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
  unsigned int limiarUs = 1000;     // acima disso = preto. PROVISORIO: conferir com a barra a ~3 mm do chao (branco bem abaixo, preto bem acima)
  int sensoresMinimos = 6;          // abaixo disso o robo nao anda
  uint8_t mortos = 0;               // bit i = S(i+1) sem sinal (ex.: 0b11 = S1 e S2 em 2500 us em qualquer chao): nao e lido, e a leitura dele copia o vizinho VIVO mais perto, para a logica
                                    // (que usa as pontas S1/S8 no cruzamento e na varredura) continuar valendo; a posicao da linha na ponta fica menos precisa (~1 passo de sensor)

  // Preenchido pela biblioteca
  const uint8_t* pinos = nullptr;            // S1 (ponta esquerda) ... S8 (ponta direita)
  unsigned int leituras[kSensores] = {};     // tempo de cada sensor em us (0 a tempoMaxUs)
  unsigned int limiar[kSensores] = {};       // acima disso = preto (= limiarUs; so os vivos contam)
  bool ok[kSensores] = {};                   // false = pino so de entrada (GPIO34-39), ignorado
  bool preto[kSensores] = {};                // true = esse sensor ve a linha
  unsigned int tempoLeituraUs = 2500;        // so espera ate o limiar (+100 us)
  QTRSensors qtr;                            // so com os pinos que podem carregar
  uint8_t indice[kSensores] = {};            // posicao de cada sensor vivo na QTRSensors -> S1..S8
  uint8_t quantos = 0;
};

void iniciar(BarraRC& barra, const uint8_t pinos[kSensores]);

// Sem calibracao: todos os sensores vivos usam barra.limiarUs. Imprime uma leitura em `saida` para conferir o limiar.
// false = poucos sensores vivos: nao deve andar assim.
bool aplicarLimiar(BarraRC& barra, Print& saida);

// Limiar por sensor a partir do VERDE do inicio da partida (o carro comeca sobre ele): o verde e escuro no infravermelho, entao o limiar de cada sensor vira `fracao` do tempo que ele
// leu sobre o verde (media de `amostras` leituras), nunca abaixo de limiarUs. Segue o nivel de iluminacao e a altura da barra daquele dia. So chame com o TCS do chao lendo VERDE.
// false = leitura implausivel (nenhum sensor vivo acima de limiarUs): mantem o limiar fixo.
bool calibrarPeloVerde(BarraRC& barra, Print& saida, float fracao = 0.8f, int amostras = 20);

// Le os 8 de uma vez e atualiza barra.preto[].
void ler(BarraRC& barra);

}  // namespace seguidor
