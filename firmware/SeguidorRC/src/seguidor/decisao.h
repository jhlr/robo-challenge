#pragma once
// Decisao: dado quais sensores veem preto, escolhe a velocidade de cada roda.
// Nao toca em pino e nao chama millis(): o tempo entra como argumento, entao da para testar no PC.

namespace seguidor {

constexpr int kSensores = 8;   // S1 = ponta esquerda ... S8 = ponta direita

// Numeros para ajustar na pista. Os valores de partida sao chutes razoaveis.
struct Ajustes {
  // Velocidades (0 a 255)
  int velReto = 110;     // indo reto
  int velRapida = 120;   // roda de fora na curva
  int velLenta = 50;     // roda de dentro na curva leve
  int velGiro = 100;     // giro no lugar (curva fechada)
  int confirmaPonta = 2; // leituras seguidas para aceitar S1/S8 (filtra ruido)

  // Linha perdida: segue o ultimo comando, gira no eixo, abre em espiral, e repete ate achar a linha
  unsigned long lacunaMs = 120;         // falha curta na fita ou cruzamento: segue o ultimo comando
  unsigned long buscaGiroMs = 600;      // gira no proprio eixo por esse tempo
  unsigned long buscaEspiralMs = 4000;  // depois abre em espiral por esse tempo; ai recomeca o giro
  int velBuscaExterna = 70;             // roda de fora na busca (constante)
  int velBuscaInternaFim = 63;          // roda de dentro no fim da espiral; comeca em -velBuscaExterna e sobe ate aqui
};

// O que a decisao lembra de um ciclo para o outro. Comeca zerada.
struct Memoria {
  int ultimoLado = 0;            // onde a linha foi vista por ultimo: -1 esquerda, 1 direita
  unsigned long perdeuEm = 0;    // quando a linha sumiu (0 = linha visivel)
  int ultimoEsq = 0, ultimoDir = 0;
  int contPontaEsq = 0, contPontaDir = 0;
};

struct Comando {
  int esq;            // velocidade da roda esquerda, -255 a 255 (positivo = frente)
  int dir;
  bool parar;         // true = ponte H em repouso (nao e o mesmo que frente com PWM 0)
  const char* acao;   // texto curto para o debug
};

// preto[0] = S1 ... preto[7] = S8. agoraMs = millis().
Comando decidir(const bool preto[kSensores], unsigned long agoraMs, const Ajustes& ajustes, Memoria& memoria);

// Use quando voce segurar os motores parados por fora (modo teste): a lacuna nao deve repetir um comando que nao rodou.
void esquecerUltimoComando(Memoria& memoria);

}  // namespace seguidor
