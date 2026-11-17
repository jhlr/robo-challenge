// Missao do robo, lida de cima para baixo. Cada passo e uma funcao da RoboLib (veja lib/RoboLib/src/RoboLib.h).
// ESTE CODIGO AINDA NAO FOI TESTADO NA PISTA: os numeros marcados com (AJUSTAR) sao chutes.
#include <Arduino.h>
#include <RoboLib.h>

// ======================= AJUSTES DA MISSAO =======================
// Em cada cruzamento o robo faz a proxima letra (e repete do comeco): S = segue reto, L = esquerda, R = direita, U = meia-volta.
const char ROTA[] = "SLSRSLSR";                // (AJUSTAR) cada letra e um cruzamento
constexpr float VELOCIDADE = 0.45f;              // fracao do maximo ao seguir linha
constexpr int DIST_CUBO_MM = 90;                 // sonar mais perto que isso = tem algo na frente (cubo, plataforma ou outro robo)
constexpr uint32_t TEMPO_PARTIDA_MS = 12UL * 60 * 1000;   // regra: 12 minutos
constexpr float DIST_ATE_ECOPONTO_MM = 600;      // (AJUSTAR) do fim da linha preta ate a area amarela (verde 280 mm + folga)
constexpr float PASSO_LATERAL_MM = 110;          // (AJUSTAR) de uma plataforma para a vizinha: 50 mm de largura + 100 mm de vao, menos o que ja andou

uint32_t inicioPartida = 0;
bool temTempo() { return millis() - inicioPartida < TEMPO_PARTIDA_MS; }

// ======================= PARTIDA =======================
// O robo comeca parado na area verde. Parte quando o chao le VERDE por 5 leituras seguidas (um verde so, de reflexo, nao conta). Ou digite 'g' no monitor serial.
void esperarPartida() {
  Serial.println("Pronto. Esperando o verde (ou 'g')...");
  int verdes = 0;
  while (verdes < 5) {
    if (Serial.available() && Serial.read() == 'g') break;
    verdes = (corChao() == VERDE) ? verdes + 1 : 0;
    delay(100);
  }
  inicioPartida = millis();
  Serial.println("PARTIDA!");
}

// ======================= PASSOS DA MISSAO =======================
// Cores de cubo que valem ponto: tem plataforma para elas. BRANCO e qualquer cor sem plataforma: so avisa e nao encosta.
bool eCuboColetavel(Cor c) { return c == AZUL || c == VERMELHO || c == VERDE || c == AMARELO || c == PRETO; }

// Anda ate a rua e entra na cidade: segue reto pela area verde ate a linha azul e depois ate a primeira linha preta.
void entrarNaCidade() {
  rodasVelocidade(VELOCIDADE, VELOCIDADE);
  while (temTempo() && corChao() != AZUL) delay(2);          // cruza a linha azul (inicio da cidade)
  while (temTempo() && linhaLer() == 0) delay(2);            // ate uma linha preta aparecer
  rodasParar();
}

// Pega o cubo que esta na frente, a ~DIST_CUBO_MM.
void pegarCubo() {
  garraAbrir();
  bracoPegar();
  andarMm(DIST_CUBO_MM - 30, 0.25f);   // (AJUSTAR) chega o cubo dentro da garra
  garraFechar();
  bracoSubir();
}

// Deixa o cubo na plataforma que esta na frente.
void soltarNaPlataforma() {
  bracoPlataforma();
  garraAbrir();
  andarMm(-60, 0.3f);
  bracoSubir();
}

// Uma viagem de coleta: anda pelas ruas seguindo a ROTA ate carregar 1 ou 2 cubos (regra: no maximo 2 por viagem).
int coletar(Cor carga[2]) {
  int cubos = 0, cruzamento = 0;
  while (cubos < 2 && temTempo()) {
    const Parada p = seguirAteParar(VELOCIDADE, DIST_CUBO_MM, 20000);
    Serial.printf("parou: %s\n", nomeParada(p));
    if (p == Parada::Cruzamento) {
      centrarNoCruzamento();
      const char letra = ROTA[cruzamento++ % (sizeof(ROTA) - 1)];
      if (letra == 'L') virar(-90);
      else if (letra == 'R') virar(90);
      else if (letra == 'U') virar(180);
    } else if (p == Parada::Obstaculo) {
      const Cor cor = corFrente();
      Serial.printf("achei: %s\n", nomeCor(cor));
      if (eCuboColetavel(cor)) {
        carga[cubos++] = cor;
        pegarCubo();
      } else if (cor == BRANCO) {
        bipar(300);   // cubo branco identificado: avisa e NAO encosta
      }
      andarMm(-80, 0.3f);   // desvia: o cubo (ou o obstaculo) fica para tras
      virar(180);
    } else if (p == Parada::Vermelho) {
      andarMm(-60, 0.3f);   // passou do limite da cidade: volta ja
      virar(180);
    } else if (p == Parada::PerdeuLinha) {
      virar(180);   // fim da rua: volta
    }
  }
  return cubos;
}

// Volta ao ecoponto: segue a rua ate o fim e atravessa a area verde ate a faixa amarela.
void irParaEcoponto() {
  virar(180);   // (AJUSTAR) supoe que a rua atual leva para casa; o robo ainda nao sabe o rumo (veja a revisao)
  Parada p;
  do {
    p = seguirAteParar(VELOCIDADE, 0, 30000);
    if (p == Parada::Cruzamento) centrarNoCruzamento();   // cruzamento no caminho: segue reto
  } while (p == Parada::Cruzamento && temTempo());        // termina quando a linha acaba (borda da cidade)
  andarMm(DIST_ATE_ECOPONTO_MM, 0.4f);
}

// Procura a plataforma da cor do cubo andando ao longo da faixa amarela e entrega. Devolve true se entregou.
bool entregar(Cor cor) {
  for (int tentativa = 0; tentativa < 5 && temTempo(); tentativa++) {
    const int d = distanciaMm();
    if (d > 0 && d < DIST_CUBO_MM && corFrente() == cor) {
      soltarNaPlataforma();
      return true;
    }
    // nao e esta: desloca para a proxima plataforma
    girarGraus(90);   // fora da cidade nao ha linha: gira so pelos encoders (virar() procura linha)
    andarMm(PASSO_LATERAL_MM, 0.3f);
    girarGraus(-90);
    rodasVelocidade(0.25f, 0.25f);   // anda ate o sonar ver a plataforma (limite de 1,5 s)
    const uint32_t t0 = millis();
    while (millis() - t0 < 1500) {
      const int dist = distanciaMm();
      if (dist > 0 && dist < DIST_CUBO_MM) break;
    }
    rodasParar();
  }
  return false;
}

void voltarParaCidade() {
  girarGraus(180);   // no ecoponto nao ha linha
  andarMm(DIST_ATE_ECOPONTO_MM, 0.4f);
  entrarNaCidade();
}

void missao() {
  entrarNaCidade();
  while (temTempo()) {
    Cor carga[2] = {DESCONHECIDA, DESCONHECIDA};
    const int cubos = coletar(carga);
    if (cubos == 0) break;   // acabou o tempo sem cubo
    irParaEcoponto();
    for (int i = 0; i < cubos; i++) entregar(carga[i]);   // regra: entrega TODOS antes de entrar na cidade de novo
    if (temTempo()) voltarParaCidade();
  }
  rodasParar();
}

// ======================= BANCADA =======================
// Compile com `pio run -e bancada -t upload` e use o monitor serial para testar cada parte isolada, sem a missao.
#ifdef BANCADA
void bancada() {
  Serial.println("\nBANCADA: s=sensores  l=linha  f=seguir linha  v=rodas  g=garra  b=braco  c=cor(crua)");
  while (true) {
    while (!Serial.available()) delay(10);
    const char c = Serial.read();
    if (c == 's') {
      for (int i = 0; i < 10; i++) {
        Serial.printf("linha=%02X sonar=%d chao=%s frente=%s\n", linhaLer(), distanciaMm(), nomeCor(corChao()), nomeCor(corFrente()));
        delay(300);
      }
    } else if (c == 'c') {
      uint16_t r, g, b, cl;
      if (corChaoCrua(r, g, b, cl)) Serial.printf("chao   r=%u g=%u b=%u c=%u\n", r, g, b, cl);
      if (corFrenteCrua(r, g, b, cl)) Serial.printf("frente r=%u g=%u b=%u c=%u\n", r, g, b, cl);
    } else if (c == 'l') {
      const uint8_t m = linhaLer();
      Serial.printf("barra=%02X posicao=%.2f cruzamento=%d\n", m, linhaPosicao(m), eCruzamento(m));
    } else if (c == 'f') {
      Serial.printf("seguindo ate parar: %s\n", nomeParada(seguirAteParar(VELOCIDADE, DIST_CUBO_MM, 15000)));
    } else if (c == 'v') {
      andarMm(100); delay(500); andarMm(-100); delay(500); girarGraus(90); delay(500); girarGraus(-90);
    } else if (c == 'g') {
      garraAbrir(); delay(500); garraFechar();
    } else if (c == 'b') {
      Serial.println(bracoSubir() ? "topo ok" : "topo FALHOU");
      Serial.println(bracoPlataforma() ? "plataforma ok" : "plataforma FALHOU");
      Serial.println(bracoSubir() ? "topo ok" : "topo FALHOU");
    }
  }
}
#endif

// ======================= LIGAR =======================
void setup() {
  Serial.begin(115200);
  sensoresIniciar();
  rodasIniciar();
  if (!bracoIniciar()) Serial.println("AVISO: o ombro (STS id 3) nao respondeu");
#ifdef BANCADA
  bancada();
#endif
  esperarPartida();
  bracoSubir();   // so mexe o ombro depois da partida
  missao();
  Serial.println("Fim da missao.");
}

void loop() { delay(1000); }
