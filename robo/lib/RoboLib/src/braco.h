#pragma once
// Braco: ombro (STS3215 na torre) e garra (SG90).
// Regra de seguranca: o ombro SEMPRE anda em passos pequenos a partir da posicao atual, conferindo posicao e corrente a cada passo.

bool bracoIniciar();   // true se o ombro respondeu. NAO move nada. Chame depois de rodasIniciar().

// Cada uma devolve true se CHEGOU na altura. Se algo der errado (nao segue, corrente alta), o ombro segura onde parou e devolve false.
bool bracoSubir();        // garra levantada: posicao de andar (o sonar enxerga a frente livre)
bool bracoPlataforma();   // cubo na altura da plataforma
bool bracoPegar();        // garra no chao, no cubo

void garraAbrir();
void garraFechar();
