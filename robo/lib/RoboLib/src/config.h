#pragma once
// TODOS os numeros do robo ficam aqui: pinos e medidas. Mexeu no robo? Muda so este arquivo.
// Placa: ESP32 DevKit classico (CP2102). Logica de 3,3 V em tudo.
#include <stdint.h>

namespace pinos {
// Servos STS3215 (rodas e ombro) no mesmo barramento, UART2.
constexpr int kStsRx = 16;
constexpr int kStsTx = 17;

// Dois barramentos I2C, um por sensor de cor (os dois TCS34725 tem o mesmo endereco 0x29).
constexpr int kChaoSda = 32, kChaoScl = 4;      // Wire  : cor do CHAO
constexpr int kFrenteSda = 25, kFrenteScl = 33;  // Wire1 : cor da FRENTE (cubo / plataforma)

// Sonar HC-SR04 (o ECHO precisa de divisor 5 V -> 3,3 V).
// ATENCAO: GPIO 12 (ECHO) e GPIO 5 (buzzer) sao pinos de boot do ESP32. Se o ECHO estiver em nivel ALTO ao ligar, a placa nao liga: mantenha o sonar sem energia ate o ESP32 subir ou o ECHO em LOW.
constexpr int kTrig = 14;
constexpr int kEcho = 12;

// Barra de linha QRE8D: S1 (esquerda) ... S8 (direita).
constexpr int kLinha[8] = {26, 27, 18, 19, 21, 22, 23, 34};

constexpr int kGarra = 13;   // servo SG90
constexpr int kBuzzer = 5;
constexpr int kLed = 2;
}  // namespace pinos

namespace cfg {
// ---- ids dos STS3215 no barramento (de fabrica todos sao 1: ajuste um de cada vez) ----
constexpr uint8_t kIdEsquerda = 1;
constexpr uint8_t kIdDireita = 2;
constexpr uint8_t kIdOmbro = 3;

// Sinal de cada lado (montagem + engrenagem invertida). Se o robo andar de re, inverta os dois.
constexpr int kSinalEsquerda = +1;
constexpr int kSinalDireita = -1;

// ---- geometria (mm) ----
constexpr float kDiametroRodaMm = 66.85f;
constexpr float kBitolaMm = 185.0f;          // distancia entre as rodas, medida com regua
constexpr float kDerrapagem = 0.8f;          // com 4 rodas derrapando o giro real e ~80% do ideal
constexpr float kEixoAteBarraMm = 15.0f;     // da linha do eixo das rodas ate a barra de linha
constexpr float kLarguraLinhaMm = 20.0f;     // largura da linha preta da pista

// ---- rodas ----
constexpr int kVelMaximaSts = 3400;          // passos/s com comando 1.0
constexpr float kVelPlenaMmS = kVelMaximaSts / 4096.0f * 3.14159265f * kDiametroRodaMm;   // ~174 mm/s com comando 1.0
constexpr uint8_t kAceleracaoRodas = 50;     // rampa de aceleracao do STS (0 = aceleracao maxima: o robo arranca e freia de supetao)
constexpr uint8_t kTensaoMax = 140;        // janela de tensao gravada no STS (x0,1 V); de fabrica 8 V e a de 12 V dá erro
constexpr uint8_t kTensaoMin = 40;

// ---- seguir linha ----
constexpr float kVelBase = 0.45f;            // fracao do comando maximo
constexpr float kGanhoLinha = 0.18f;         // quanto virar por unidade de erro (erro vai de -3,5 a +3,5)
constexpr uint32_t kLinhaPerdidaMs = 400;    // sem ver a linha por este tempo = perdida
constexpr float kFracaoGiroPorEncoder = 0.85f;  // virar(): gira esta parte do angulo pelos encoders e termina procurando a linha
constexpr uint32_t kBuscaLinhaMs = 1500;     // virar(): quanto tempo gira devagar procurando a linha (~40 graus)
constexpr int kVermelhoLeituras = 2;         // leituras seguidas de VERMELHO para parar (evita parar por um reflexo)

// ---- ombro (STS3215 direto na torre, 4096 passos = 360 graus) ----
// Medido na bancada: posicao crescente DESCE a garra.
constexpr int kOmbroTopo = 1981;             // garra levantada
constexpr int kOmbroPlataforma = 2567;       // cubo na altura da plataforma
constexpr int kOmbroChao = 2927;             // garra no cubo do chao (limite de baixo)
constexpr int kOmbroPasso = 64;              // anda de 64 em 64 passos (~5,6 graus), conferindo a cada um
constexpr int kOmbroVelocidade = 300;        // passos/s
constexpr int kOmbroAceleracao = 20;
constexpr int kOmbroCorrenteMax = 400;       // acima disso: bateu em algo, para

// ---- garra (SG90, graus 0..180) ----
constexpr float kGarraAbertaGraus = 0.0f;
constexpr float kGarraFechadaGraus = 90.0f;
constexpr uint16_t kGarraTempoMs = 600;

// ---- sonar ----
constexpr float kSomMmPorUs = 0.343f;        // velocidade do som a ~20 C
}  // namespace cfg
