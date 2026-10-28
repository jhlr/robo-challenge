#pragma once
// Decisao: dado quais sensores veem preto, escolhe a velocidade de cada roda.
// Nao toca em pino e nao chama millis(): o tempo entra como argumento, entao da para testar no PC.

namespace seguidor {

constexpr int kSensores = 8;   // S1 = ponta esquerda ... S8 = ponta direita

// Numeros para ajustar na pista. Os valores de partida sao chutes razoaveis.
struct Ajustes {
  // Velocidades (0 a 255)
  int velReto = 110;     // indo reto
  // Toda curva (correcao, busca, cruzamento) move as rodas em SENTIDOS OPOSTOS, nunca uma parada ou lenta arrastando: gira no eixo.
  // Os TT amarelos nao andam abaixo de uns 25% de PWM (~65): as duas velocidades ficam acima disso.
  int velCorrecaoLeve = 70;    // giro de correcao quando S3 ou S6 acende junto com o centro
  int velCorrecaoForte = 100;  // giro de correcao quando S2 ou S7 acende

  // Giro intercalado com avanco: so girar no eixo nao sai do lugar (gira, passa do ponto, gira de volta e fica travado).
  // Entao cada correcao alterna, dentro de periodoCorrecaoMs, um trecho de GIRO (rodas em sentidos opostos) e um de AVANCO reto.
  unsigned long periodoCorrecaoMs = 80;   // tamanho de um ciclo giro + avanco
  float fracaoGiroLeve = 0.5f;            // parte do ciclo girando na correcao leve (o resto avanca)
  float fracaoGiroForte = 0.8f;           // idem na forte; entre as duas a fracao sobe em rampa junto com a velocidade

  // Correcao proporcional: posicao da linha = media dos sensores S2..S7 acesos, de -3,5 (so S1) a +3,5 (so S8) sem as pontas: S2 = -2,5 ... S4 = -0,5, S5 = +0,5 ... S7 = +2,5
  float posReto = 0.75f;       // |posicao| ate aqui: em cima da linha (S4 e/ou S5), reto
  float posLeve = 1.5f;        // ate aqui a correcao e a leve (S3, S6 ou S3+S4...)
  float posForte = 2.5f;       // daqui para fora a correcao e a forte (S2 ou S7); entre posLeve e posForte sobe em rampa

  // Padrao largo (3 ou mais acesos, sem ser cruzamento): chegando num cruzamento, ou torto sobre a linha. Reduz a velocidade
  int larguraAviso = 4;        // sensores acesos que contam como "largo"
  float fatorLargura = 0.7f;   // velReto x isto

  // Oscilacao: correcoes que trocam de lado varias vezes seguidas = ganho alto. Reduz a velocidade
  int janelaOscilacao = 6;     // ultimas decisoes olhadas (maximo kJanelaOsc)
  int trocasOscilacao = 4;     // trocas de lado nessa janela que contam como oscilando
  float fatorOscilacao = 0.8f; // velReto x isto
  int velMinima = 65;          // nenhuma velocidade de reto fica abaixo disso (zona morta dos TT)

  // Linha perdida: segue o ultimo comando, gira no eixo, abre em espiral, e repete ate achar a linha
  unsigned long lacunaMs = 120;         // falha curta na fita ou cruzamento: segue o ultimo comando
  float buscaGiroGraus = 360.0f;        // ANTES de espiralar, sempre gira no proprio eixo uma volta inteira (a linha pode estar de qualquer lado)
  float grausPorSegundoBusca = 110.0f;  // CHUTE: velocidade desse giro com velBuscaExterna; cronometre uma volta e corrija
  unsigned long buscaEspiralMs = 4000;  // depois abre em espiral por esse tempo; ai recomeca o giro
  int velBuscaExterna = 70;             // roda de fora na busca (constante)
  float bitolaMm = 140.0f;              // CHUTE: distancia entre as rodas esquerda e direita; meca e corrija
  float raioMaxEspiralMm = 125.0f;      // a espiral NUNCA passa desse raio de curva (folga entre a rua e os cubos marrons dos quarteiroes)
  int velBuscaContrariaMin = 10;        // e a roda de dentro NUNCA chega a zero nem passa: fica sempre pelo menos assim para tras (sentidos opostos)
};

// Roda de dentro no fim da espiral: a que da exatamente raioMaxEspiralMm. Com a de fora em vo e a de dentro em vi, o raio da curva do
// centro do robo e R = (bitola/2) * (vo + vi) / (vo - vi); isolando vi: vi = vo * (k - 1) / (k + 1), com k = R / (bitola/2).
// A de dentro comeca em -vo (giro no eixo, R = 0) e sobe ate aqui, entao o raio cresce de 0 ate raioMaxEspiralMm e nunca passa.
// Mas as rodas giram sempre em sentidos opostos (nao arrasta): a de dentro fica pelo menos velBuscaContrariaMin para tras, o que
// limita o raio ainda mais (com bitola 140 e vo 70, ~52 mm).
inline int velBuscaInternaFim(const Ajustes& aj) {
  const float k = aj.raioMaxEspiralMm / (aj.bitolaMm / 2.0f);
  const int porRaio = static_cast<int>(aj.velBuscaExterna * (k - 1.0f) / (k + 1.0f));
  const int porSentido = -aj.velBuscaContrariaMin;
  return porRaio < porSentido ? porRaio : porSentido;
}

// Quanto tempo dura o giro no eixo da busca (uma volta inteira, antes da espiral).
inline unsigned long buscaGiroMs(const Ajustes& aj) {
  return static_cast<unsigned long>(aj.buscaGiroGraus / aj.grausPorSegundoBusca * 1000.0f + 0.5f);
}

// O que a decisao lembra de um ciclo para o outro. Comeca zerada.
constexpr int kJanelaOsc = 8;

struct Memoria {
  int ultimoLado = 0;            // onde a linha foi vista por ultimo: -1 esquerda, 1 direita
  unsigned long perdeuEm = 0;    // quando a linha sumiu (0 = linha visivel)
  int ultimoEsq = 0, ultimoDir = 0;
  int sinais[kJanelaOsc] = {};   // ultimas correcoes: -1 esquerda, 0 reto, +1 direita (anel)
  int proximoSinal = 0;
};

struct Comando {
  int esq;            // velocidade da roda esquerda, -255 a 255 (positivo = frente)
  int dir;
  bool parar;         // true = ponte H em repouso (nao e o mesmo que frente com PWM 0)
  const char* acao;   // texto curto para o debug
  bool largo = false;       // padrao largo: provavel cruzamento chegando, ou torto sobre a linha (anda mais devagar)
  bool oscilando = false;   // as correcoes estao trocando de lado: anda mais devagar
  bool estranho = false;   // S1 ou S8 sozinho: as duas pontas devem acender juntas (cruzamento); uma so e leitura suspeita
};

// Esta procurando a linha (ja passou da lacuna e esta girando/espiralando)? Quem chama usa isto para NAO tratar S1 e S8 acesos como cruzamento:
// girando no eixo, a barra acaba paralela a linha e acende inteira, mas isso e o robo de lado, nao um cruzamento.
inline bool estaBuscando(const Memoria& m, unsigned long agoraMs, const Ajustes& aj) {
  return m.perdeuEm != 0 && agoraMs - m.perdeuEm >= aj.lacunaMs;
}

// preto[0] = S1 ... preto[7] = S8. agoraMs = millis().
Comando decidir(const bool preto[kSensores], unsigned long agoraMs, const Ajustes& ajustes, Memoria& memoria);

// Use quando voce segurar os motores parados por fora (modo teste): a lacuna nao deve repetir um comando que nao rodou.
void esquecerUltimoComando(Memoria& memoria);

}  // namespace seguidor
