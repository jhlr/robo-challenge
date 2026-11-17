#pragma once
// Seguir a linha preta (as "ruas" da cidade), reconhecer cruzamentos e virar neles.
#include <stdint.h>

// Por que o robo parou de seguir a linha.
enum class Parada { Cruzamento, PerdeuLinha, Vermelho, Obstaculo, Tempo };
const char* nomeParada(Parada p);

// Onde a linha esta embaixo da barra: -3,5 (esquerda) ... 0 (centro) ... +3,5 (direita). Sem linha: devolve 999.
float linhaPosicao(uint8_t mascara);
constexpr float kSemLinha = 999.0f;

// Cruzamento = linha larga: 6 ou mais sensores acesos (ou as duas pontas S1 e S8 juntas).
bool eCruzamento(uint8_t mascara);

// Um passo do seguidor: ajusta as rodas pela leitura `mascara` da barra (linhaLer()). Chame num laco. Devolve false se nao ha linha embaixo.
bool seguirLinhaPasso(uint8_t mascara, float vel);

// Segue a linha ate algo acontecer. `obstaculoMm`: para se o sonar ver algo mais perto que isso (0 = ignora o sonar).
// Parar em VERMELHO e regra do campeonato (o vermelho e o limite da cidade).
Parada seguirAteParar(float vel, int obstaculoMm, uint32_t tempoMaxMs);

// No cruzamento: anda ate o eixo das rodas ficar no centro dele (a barra fica a frente do eixo). Chame logo depois de seguirAteParar() devolver Cruzamento.
void centrarNoCruzamento();

// Vira no lugar em cima do cruzamento: +90 direita, -90 esquerda, 180 meia-volta. Termina com a linha de novo embaixo do centro da barra.
// Se nao houver linha no destino, desiste da busca depois de ~40 graus extras (nao fica rodando).
void virar(int graus);

// Rumo: para onde o robo esta virado. 0 = para a cidade (como ele e posicionado na partida), 90 = direita, -90 = esquerda, 180 = para casa (ecoponto).
// virar() atualiza sozinho. Quem girar por outro meio (girarGraus) e quiser manter o rumo certo chama rumoDefinir().
int rumoGraus();
void rumoDefinir(int graus);
int giroParaRumo(int alvoGraus);   // quanto virar (-90, 0, +90 ou 180) para ficar olhando para `alvoGraus`
