// Missao do robo, lida de cima para baixo. Cada passo e uma funcao da RoboLib (veja lib/RoboLib/src/RoboLib.h).
// ESTE CODIGO AINDA NAO FOI TESTADO NA PISTA: os numeros marcados com (AJUSTAR) sao chutes.
// Limites conhecidos: o robo nao sabe para que lado fica "casa" (nao guarda o rumo) e nao ha tabela de cor MARROM (cubo marrom e obstaculo).
#include <Arduino.h>
#include <RoboLib.h>

// ======================= AJUSTES DA MISSAO =======================
// Em cada cruzamento o robo faz a proxima letra (e repete do comeco): S = segue reto, L = esquerda, R = direita, U = meia-volta.
const char ROTA[] = "SLSRSLSR";                // (AJUSTAR) cada letra e um cruzamento
constexpr float VELOCIDADE = cfg::kVelBase;      // fracao do maximo ao seguir linha
constexpr int DIST_CUBO_MM = 90;                 // sonar mais perto que isso = tem algo na frente (cubo, plataforma ou outro robo)
constexpr int DIST_LER_COR_MM = 40;              // o sensor de cor da frente so le bem a poucos cm: chega perto antes de ler
constexpr int DIST_PEGAR_MM = 25;                // (AJUSTAR) distancia do cubo/plataforma com a garra no lugar certo
constexpr uint32_t TEMPO_PARTIDA_MS = 12UL * 60 * 1000;   // regra: 12 minutos
constexpr uint32_t TEMPO_FASE_MS = 15000;        // nenhuma fase de "andar ate ver X" passa disso (evita sair da arena)
constexpr float DIST_ATE_ECOPONTO_MM = 600;      // (AJUSTAR) do fim da linha preta ate a area amarela (verde 280 mm + folga)
constexpr float PASSO_PLATAFORMA_MM = 260;       // plataformas de 160 mm de comprimento com 100 mm de vao, em fila ao longo da faixa
constexpr int CUBOS_POR_VIAGEM = 1;              // a garra segura 1 cubo (a regra permite ate 2)

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

// ======================= PEQUENAS AJUDAS =======================
// Cores de cubo que valem ponto: tem plataforma para elas. BRANCO e qualquer cor sem plataforma: so avisa e nao encosta.
bool eCuboColetavel(Cor c) { return c == AZUL || c == VERMELHO || c == VERDE || c == AMARELO || c == PRETO; }

// Anda para a frente ate o sonar ver algo mais perto que `mm` (ou ate `tempoMs`). Devolve true se chegou.
bool aproximarAte(int mm, uint32_t tempoMs) {
  rodasVelocidade(0.25f, 0.25f);
  const uint32_t t0 = millis();
  bool chegou = false;
  while (millis() - t0 < tempoMs && !chegou) {
    const int d = distanciaMm();
    chegou = d > 0 && d <= mm;
  }
  rodasParar();
  return chegou;
}

// ======================= PASSOS DA MISSAO =======================
// Anda ate a rua e entra na cidade: segue reto pela area verde ate a linha azul e depois ate a primeira linha preta. Cada fase tem limite de tempo.
void entrarNaCidade() {
  rodasVelocidade(VELOCIDADE, VELOCIDADE);
  uint32_t t0 = millis();
  while (temTempo() && millis() - t0 < TEMPO_FASE_MS && corChao() != AZUL) delay(2);   // cruza a linha azul (inicio da cidade)
  t0 = millis();
  while (temTempo() && millis() - t0 < TEMPO_FASE_MS && linhaLer() == 0 && corChao() != VERMELHO) delay(2);   // ate uma linha preta aparecer
  rodasParar();
}

// Pega o cubo que esta na frente. Devolve false se o ombro nao chegou (nao fecha a garra no ar).
bool pegarCubo() {
  garraAbrir();
  if (!bracoPegar()) { bipar(500); bracoSubir(); return false; }
  const int d = distanciaMm();
  if (d > DIST_PEGAR_MM) andarMm(d - DIST_PEGAR_MM, 0.25f);   // chega o cubo dentro da garra
  garraFechar();
  bracoSubir();
  return true;
}

// Deixa o cubo na plataforma que esta na frente.
void soltarNaPlataforma() {
  const int d = distanciaMm();
  if (d > DIST_PEGAR_MM) andarMm(d - DIST_PEGAR_MM, 0.25f);   // encosta na plataforma
  bracoPlataforma();
  garraAbrir();
  andarMm(-60, 0.3f);
  bracoSubir();
}

// Uma viagem de coleta: anda pelas ruas seguindo a ROTA ate carregar CUBOS_POR_VIAGEM cubos.
int coletar(Cor carga[]) {
  int cubos = 0, cruzamento = 0;
  while (cubos < CUBOS_POR_VIAGEM && temTempo()) {
    const Parada p = seguirAteParar(VELOCIDADE, DIST_CUBO_MM, 20000);
    Serial.printf("parou: %s\n", nomeParada(p));
    if (p == Parada::Cruzamento) {
      centrarNoCruzamento();
      const char letra = ROTA[cruzamento++ % (sizeof(ROTA) - 1)];
      if (letra == 'L') virar(-90);
      else if (letra == 'R') virar(90);
      else if (letra == 'U') virar(180);
    } else if (p == Parada::Obstaculo) {
      aproximarAte(DIST_LER_COR_MM, 1500);   // chega perto: o sensor de cor so le bem de perto
      const Cor cor = corFrente();
      Serial.printf("achei: %s\n", nomeCor(cor));
      if (eCuboColetavel(cor) && pegarCubo()) {
        carga[cubos++] = cor;
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
  virar(180);   // (AJUSTAR) supoe que a rua atual leva para casa; o robo ainda nao sabe o rumo
  Parada p;
  do {
    p = seguirAteParar(VELOCIDADE, 0, 30000);
    if (p == Parada::Cruzamento) centrarNoCruzamento();   // cruzamento no caminho: segue reto
  } while (p == Parada::Cruzamento && temTempo());        // termina quando a linha acaba (borda da cidade)
  andarMm(DIST_ATE_ECOPONTO_MM, 0.4f);
}

// Procura a plataforma da cor do cubo andando ao longo da faixa amarela e entrega. Devolve true se entregou.
bool entregar(Cor cor) {
  for (int plataforma = 0; plataforma < 5 && temTempo(); plataforma++) {   // ate 5 plataformas na faixa
    if (aproximarAte(DIST_LER_COR_MM, 3000) && corFrente() == cor) {
      soltarNaPlataforma();
      return true;
    }
    // nao e esta (ou nao ha nada na frente): recua e vai para a proxima, ao longo da faixa
    andarMm(-100, 0.3f);
    girarGraus(90);   // fora da cidade nao ha linha: gira so pelos encoders (virar() procura linha)
    andarMm(PASSO_PLATAFORMA_MM, 0.3f);
    girarGraus(-90);
  }
  return false;
}

// Sem plataforma para o cubo: a regra permite deixar na area livre (verde). Recua para o verde e solta.
void descartarNaAreaVerde() {
  andarMm(-300, 0.3f);   // (AJUSTAR) do amarelo de volta para o meio do verde
  bracoPlataforma();
  garraAbrir();
  andarMm(-60, 0.3f);
  bracoSubir();
}

void voltarParaCidade() {
  girarGraus(180);   // no ecoponto nao ha linha
  andarMm(DIST_ATE_ECOPONTO_MM, 0.4f);
  entrarNaCidade();
}

void missao() {
  entrarNaCidade();
  while (temTempo()) {
    Cor carga[CUBOS_POR_VIAGEM];
    const int cubos = coletar(carga);
    if (cubos == 0) break;   // acabou o tempo sem cubo
    irParaEcoponto();
    for (int i = 0; i < cubos; i++)
      if (!entregar(carga[i])) descartarNaAreaVerde();   // regra: entrega ou descarta TODOS antes de entrar na cidade de novo
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
