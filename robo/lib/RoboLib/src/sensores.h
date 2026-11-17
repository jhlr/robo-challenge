#pragma once
// Sensores: barra de linha, sonar e os dois sensores de cor. Tudo devolve valores simples.
#include <stdint.h>

#include "cores.h"  // enum Cor: BRANCO, PRETO, VERDE, AMARELO, AZUL, VERMELHO, DESCONHECIDA

void sensoresIniciar();

// Barra de linha: bit i ligado = sensor S(i+1) esta em cima da linha preta. S1 = esquerda (bit 0), S8 = direita (bit 7).
uint8_t linhaLer();

// Sonar: distancia ao obstaculo da frente em mm, ou -1 se nao houve eco.
int distanciaMm();

// Cor lida pelo sensor do chao / da frente. DESCONHECIDA se nada se parece ou o sensor nao respondeu.
Cor corChao();
Cor corFrente();

// Leitura crua (r, g, b, clear) para calibrar a tabela de cores em cores.h. false se o sensor nao respondeu.
bool corChaoCrua(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c);
bool corFrenteCrua(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c);

void bipar(uint16_t ms);   // buzzer: serve para avisar "cubo branco identificado"
void led(bool ligado);
