#pragma once
// Rodas: dois STS3215 (um por lado, cada um move 2 rodas). Velocidades vao de -1 (re) a +1 (frente).
#include <SCServo.h>

void rodasIniciar();                          // abre o barramento STS e poe as rodas em modo velocidade (chame ANTES de bracoIniciar)
void rodasVelocidade(float esq, float dir);   // comando direto, nao espera nada
void rodasParar();

// Bloqueantes: andam/giram medindo pelos encoders e param no fim (ou no tempo limite).
void andarMm(float mm, float vel = 0.35f);    // mm negativo = re
void girarGraus(float graus, float vel = 0.35f);  // + = direita, - = esquerda

// Distancia andada medindo pelos encoders: zere, ande (rodasVelocidade) e leia. Serve para medir algo andando, ex.: a largura de uma linha.
void rodasZerarDistancia();
float rodasDistanciaMm();   // media das duas rodas desde rodasZerarDistancia(), frente positivo
float rodasGiroGraus();     // quanto o robo girou desde rodasZerarDistancia(): + = direita. Numa reta com rodas iguais deve dar ~0

SMS_STS& barramentoSts();                     // o barramento compartilhado (o braco usa o mesmo)
