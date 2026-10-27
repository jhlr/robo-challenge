// =====================================================
//  Seguidor de linha preta SIMPLES (sem PID)
//  ESP32-S3 + TB6612FNG + barra de 8 sensores tipo RC
//  Placa na Arduino IDE: "ESP32S3 Dev Module"
//
//  Leitura RC: carrega o pino e mede o tempo de descarga.
//  Branco = tempo curto | Preto = tempo longo
//
//  Versao corrigida (ver README.md): o robo nao gira mais sem fim quando perde a linha.
// =====================================================

#include <Arduino.h>
#include <soc/gpio_reg.h>

// --- Pinos da ponte H TB6612FNG ---
// Motor A = roda ESQUERDA | Motor B = roda DIREITA
constexpr int AIN1 = 11;
constexpr int AIN2 = 12;
constexpr int PWMA = 13;

constexpr int BIN1 = 14;
constexpr int BIN2 = 21;
constexpr int PWMB = 47;

constexpr int STBY = 38;

// --- Sensores (S1 = ponta esquerda ... S8 = ponta direita) ---
// Todos os pinos abaixo de 32: um unico registrador (GPIO_IN_REG) le os 8 de uma vez
constexpr int NUM_SENSORES = 8;
constexpr int pinosSensores[NUM_SENSORES] = {8, 7, 6, 5, 4, 3, 2, 1};
constexpr unsigned int TEMPO_MAX = 2500;   // microssegundos (valor do preto)
constexpr unsigned int FAIXA_MIN = 300;    // preto - branco minimo (us) para o sensor valer
constexpr int SENSORES_MIN_OK = 6;         // abaixo disso a calibracao e recusada

unsigned int leituras[NUM_SENSORES];   // tempo de cada sensor (0 a 2500)
unsigned int limiar[NUM_SENSORES];     // separa branco de preto (calculado na calibração)
bool sensorOk[NUM_SENSORES];           // false = sensor não reagiu na calibração (ignorado)
bool preto[NUM_SENSORES];              // true = sensor vendo a linha
unsigned int tempoLeitura = TEMPO_MAX; // quanto esperar na leitura: so ate o maior limiar (calculado na calibracao)
uint32_t mascaraSensor[NUM_SENSORES];  // bit de cada sensor no registrador de entrada
uint32_t mascaraTodos = 0;

// --- Velocidades (0 a 255) ---
constexpr int VEL_RETO   = 110;    // indo reto
constexpr int VEL_RAPIDA = 120;    // roda de fora na curva
constexpr int VEL_LENTA  = 50;     // roda de dentro na curva leve
constexpr int VEL_GIRO   = 100;    // giro no lugar (curva fechada)
constexpr int VEL_BUSCA  = 70;     // giro no lugar ao procurar a linha (mais lento que o giro de curva)

// --- Linha perdida ---
constexpr unsigned long LACUNA_MS = 120;   // sem linha por menos que isso: segue o ultimo comando (falha na fita, cruzamento)
constexpr unsigned long BUSCA_MS  = 1500;  // depois da lacuna, procura no ultimo lado por ate isso; depois PARA
constexpr int CONFIRMA_PONTA = 2;          // leituras seguidas para aceitar S1/S8 (filtra ruido que fazia girar)

// --- Debug ---
constexpr bool DEBUG = true;        // mostra os sensores e a decisão no Monitor Serial (115200)
constexpr bool MODO_TESTE = false;  // true = motores desligados, só mostra no Monitor Serial

// Lado onde a linha foi vista por último: -1 esquerda, 1 direita
int ultimoLado = 0;
unsigned long ultimoPrint = 0;
unsigned long perdeuEm = 0;         // quando a linha sumiu (0 = linha visivel)
int ultimoEsq = 0, ultimoDir = 0;   // ultimo comando de motores (mantido durante a lacuna)
int contPontaEsq = 0, contPontaDir = 0;

// =====================================================

void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  analogWriteFrequency(PWMA, 20000);   // 20 kHz: fora da faixa audivel, a ponte H aguenta
  analogWriteFrequency(PWMB, 20000);
#endif

  for (int i = 0; i < NUM_SENSORES; i++) {
    mascaraSensor[i] = 1UL << pinosSensores[i];
    mascaraTodos |= mascaraSensor[i];
  }

  digitalWrite(STBY, HIGH);
  parar();

  delay(1000);
  // Nao anda com calibracao ruim: sensor morto ou robo parado fazia o carro rodopiar
  while (!calibrar()) {
    Serial.println("Calibracao INVALIDA (poucos sensores reagiram). Tentando de novo em 2 s...");
    delay(2000);
  }
  delay(2000);                 // tempo para colocar o robô na linha
}

void loop() {
  lerSensores();

  // Agrupa os sensores
  contPontaEsq = preto[0] ? contPontaEsq + 1 : 0;
  contPontaDir = preto[7] ? contPontaDir + 1 : 0;
  bool pontaEsq  = contPontaEsq >= CONFIRMA_PONTA;   // S1 (confirmado)
  bool esq       = preto[1] || preto[2];             // S2, S3
  bool centro    = preto[3] || preto[4];             // S4, S5
  bool dir       = preto[5] || preto[6];             // S6, S7
  bool pontaDir  = contPontaDir >= CONFIRMA_PONTA;   // S8 (confirmado)
  bool temLinha  = preto[0] || esq || centro || dir || preto[7];

  const char* acao;

  if (!temLinha) {
    acao = linhaPerdida();
  }
  else {
    perdeuEm = 0;

    if (pontaEsq && !pontaDir) {
      girarEsquerda(VEL_GIRO);
      ultimoLado = -1;
      acao = "<<< GIRO ESQUERDA";
    }
    else if (pontaDir && !pontaEsq) {
      girarDireita(VEL_GIRO);
      ultimoLado = 1;
      acao = "GIRO DIREITA >>>";
    }
    else if (esq && !dir) {
      mover(VEL_LENTA, VEL_RAPIDA);
      ultimoLado = -1;
      acao = "<< esquerda";
    }
    else if (dir && !esq) {
      mover(VEL_RAPIDA, VEL_LENTA);
      ultimoLado = 1;
      acao = "direita >>";
    }
    else {
      // centro, ou esq e dir juntos, ou so uma ponta ainda nao confirmada
      mover(VEL_RETO, VEL_RETO);
      acao = "^ RETO ^";
    }
  }

  if (MODO_TESTE) parar();

  if (DEBUG && millis() - ultimoPrint > 150) {
    ultimoPrint = millis();
    mostrarDebug(acao);
  }
}

// =====================================================
//  Linha perdida: lacuna -> busca limitada -> para
//  (antes: girava no lugar para sempre a PWM 100)
// =====================================================
const char* linhaPerdida() {
  const unsigned long agora = millis();
  if (perdeuEm == 0) perdeuEm = agora;
  const unsigned long perdido = agora - perdeuEm;

  if (perdido < LACUNA_MS) {
    mover(ultimoEsq, ultimoDir);                 // mantem o que estava fazendo
    return "PERDEU -> lacuna (mantem)";
  }
  if (ultimoLado != 0 && perdido < LACUNA_MS + BUSCA_MS) {
    if (ultimoLado < 0) girarEsquerda(VEL_BUSCA);
    else girarDireita(VEL_BUSCA);
    return ultimoLado < 0 ? "PERDEU -> procura ESQ" : "PERDEU -> procura DIR";
  }
  parar();                                       // nao achou: para e espera a linha voltar
  return "PERDEU -> PARADO";
}

// =====================================================
//  Leitura RC dos 8 sensores ao mesmo tempo
// =====================================================
// Espera ate 'limite' us. Os 8 pinos sao lidos de uma vez no registrador de entrada
// (nada de digitalRead em laco) e a medida para assim que todos descarregam.
void lerTempos(unsigned int limite) {
  // 1) Carrega os capacitores dos sensores
  for (int i = 0; i < NUM_SENSORES; i++) {
    pinMode(pinosSensores[i], OUTPUT);
    digitalWrite(pinosSensores[i], HIGH);
    leituras[i] = TEMPO_MAX;
  }
  delayMicroseconds(10);

  // 2) Solta os pinos e mede quanto tempo cada um demora para cair
  for (int i = 0; i < NUM_SENSORES; i++) pinMode(pinosSensores[i], INPUT);

  uint32_t pendentes = mascaraTodos;               // pinos que ainda estao em HIGH
  const uint32_t inicio = micros();
  while (pendentes) {
    const uint32_t t = micros() - inicio;
    if (t >= limite) break;
    const uint32_t caiu = ~REG_READ(GPIO_IN_REG) & pendentes;
    if (caiu) {
      for (int i = 0; i < NUM_SENSORES; i++) {
        if (caiu & mascaraSensor[i]) leituras[i] = t;
      }
      pendentes &= ~caiu;
    }
  }
}

void lerSensores() {
  // Sem MODO_TESTE basta esperar ate o maior limiar: quem ainda esta em HIGH la e preto
  lerTempos(MODO_TESTE ? TEMPO_MAX : tempoLeitura);
  for (int i = 0; i < NUM_SENSORES; i++) {
    preto[i] = sensorOk[i] && leituras[i] > limiar[i];
  }
}

// =====================================================
//  Calibração: 5 s deslizando o robô sobre a linha (motores parados)
//  Retorna false se poucos sensores reagiram (nao deve andar assim)
// =====================================================
bool calibrar() {
  unsigned int minimo[NUM_SENSORES];
  unsigned int maximo[NUM_SENSORES];
  for (int i = 0; i < NUM_SENSORES; i++) {
    minimo[i] = TEMPO_MAX;
    maximo[i] = 0;
  }

  Serial.println("Calibrando por 5 s: deslize o robo sobre a linha preta...");
  unsigned long inicio = millis();
  while (millis() - inicio < 5000) {
    lerTempos(TEMPO_MAX);
    for (int i = 0; i < NUM_SENSORES; i++) {
      if (leituras[i] < minimo[i]) minimo[i] = leituras[i];
      if (leituras[i] > maximo[i]) maximo[i] = leituras[i];
    }
  }

  int ok = 0;
  unsigned int maiorLimiar = 0;
  Serial.println("Sensor  Branco  Preto  Limiar");
  for (int i = 0; i < NUM_SENSORES; i++) {
    limiar[i] = (minimo[i] + maximo[i]) / 2;
    sensorOk[i] = (maximo[i] - minimo[i]) >= FAIXA_MIN;
    if (sensorOk[i]) {
      ok++;
      if (limiar[i] > maiorLimiar) maiorLimiar = limiar[i];
    }

    char linha[64];
    snprintf(linha, sizeof(linha), "  S%d     %4u   %4u    %4u", i + 1, minimo[i], maximo[i], limiar[i]);
    Serial.print(linha);
    if (!sensorOk[i]) Serial.print("  <-- NAO REAGIU! Sensor ignorado");
    Serial.println();
  }

  if (ok < SENSORES_MIN_OK) return false;

  tempoLeitura = min(TEMPO_MAX, maiorLimiar + 100);   // +100 us de margem sobre o maior limiar
  Serial.print("Calibracao concluida (");
  Serial.print(ok);
  Serial.print(" sensores, leitura de ");
  Serial.print(tempoLeitura);
  Serial.println(" us). Coloque o robo na linha.");
  return true;
}

// Mostra algo como:
// S1: 542 S2: 574 S3:2500 S4:2500 S5: 644 S6: 773 S7: 751 S8: 600 | [..##....]  << esquerda
// (fora do MODO_TESTE a leitura para no maior limiar: tempos acima dele aparecem como 2500)
void mostrarDebug(const char* acao) {
  char buf[96];
  int n = 0;
  for (int i = 0; i < NUM_SENSORES; i++) {
    n += snprintf(buf + n, sizeof(buf) - n, "S%d:%4u ", i + 1, leituras[i]);
  }
  Serial.print(buf);
  Serial.print("| [");
  for (int i = 0; i < NUM_SENSORES; i++) {
    if (!sensorOk[i]) Serial.print('x');
    else Serial.print(preto[i] ? '#' : '.');
  }
  Serial.print("]  ");
  Serial.println(acao);
}

// =====================================================
//  Movimentos
// =====================================================
void mover(int velEsq, int velDir) {
  ultimoEsq = velEsq;
  ultimoDir = velDir;
  motorA(velEsq);
  motorB(velDir);
}

void girarEsquerda(int vel) {
  mover(-vel, vel);   // roda esquerda para trás, direita para frente
}

void girarDireita(int vel) {
  mover(vel, -vel);   // roda esquerda para frente, direita para trás
}

// Velocidade positiva = frente, negativa = trás
void motorA(int vel) {
  vel = constrain(vel, -255, 255);
  if (vel >= 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    vel = -vel;
  }
  analogWrite(PWMA, vel);
}

void motorB(int vel) {
  vel = constrain(vel, -255, 255);
  if (vel >= 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    vel = -vel;
  }
  analogWrite(PWMB, vel);
}

void parar() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 0);
  ultimoEsq = 0;
  ultimoDir = 0;
}
