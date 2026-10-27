// =====================================================
//  Seguidor de linha preta SIMPLES (sem PID)
//  ESP32-S3 + TB6612FNG + barra de 8 sensores tipo RC
//  Placa na Arduino IDE: "ESP32S3 Dev Module"
//
//  Leitura RC: carrega o pino e mede o tempo de descarga.
//  Branco = tempo curto | Preto = tempo longo
// =====================================================

// --- Pinos da ponte H TB6612FNG ---
// Motor A = roda ESQUERDA | Motor B = roda DIREITA
const int AIN1 = 11;
const int AIN2 = 12;
const int PWMA = 13;

const int BIN1 = 14;
const int BIN2 = 21;
const int PWMB = 47;

const int STBY = 38;

// --- Sensores (S1 = ponta esquerda ... S8 = ponta direita) ---
const int NUM_SENSORES = 8;
const int pinosSensores[NUM_SENSORES] = {8, 7, 6, 5, 4, 3, 2, 1};
const unsigned int TEMPO_MAX = 2500;   // microssegundos (valor do preto)

unsigned int leituras[NUM_SENSORES];   // tempo de cada sensor (0 a 2500)
unsigned int limiar[NUM_SENSORES];     // separa branco de preto (calculado na calibração)
bool sensorOk[NUM_SENSORES];           // false = sensor não reagiu na calibração (ignorado)
bool preto[NUM_SENSORES];              // true = sensor vendo a linha

// --- Velocidades (0 a 255) ---
const int VEL_RETO   = 110;    // indo reto
const int VEL_RAPIDA = 120;    // roda de fora na curva
const int VEL_LENTA  = 50;     // roda de dentro na curva leve
const int VEL_GIRO   = 100;    // giro no lugar (curva fechada / linha perdida)

// --- Debug ---
const bool DEBUG = true;       // mostra os sensores e a decisão no Monitor Serial (115200)
const bool MODO_TESTE = false;  // true = motores desligados, só mostra no Monitor Serial

// Lado onde a linha foi vista por último: -1 esquerda, 1 direita
int ultimoLado = 0;
unsigned long ultimoPrint = 0;

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

  digitalWrite(STBY, HIGH);
  parar();

  delay(1000);
  calibrar();
  delay(2000);                 // tempo para colocar o robô na linha
}

void loop() {
  lerSensores();

  // Agrupa os sensores
  bool pontaEsq  = preto[0];                 // S1
  bool esq       = preto[1] || preto[2];     // S2, S3
  bool centro    = preto[3] || preto[4];     // S4, S5
  bool dir       = preto[5] || preto[6];     // S6, S7
  bool pontaDir  = preto[7];                 // S8

  String acao;

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
  else if (centro || (esq && dir)) {
    mover(VEL_RETO, VEL_RETO);
    acao = "^ RETO ^";
  }
  else {
    // Nenhum sensor vê a linha: gira para o último lado onde ela estava
    if (ultimoLado < 0) {
      girarEsquerda(VEL_GIRO);
      acao = "PERDEU -> procura ESQ";
    } else if (ultimoLado > 0) {
      girarDireita(VEL_GIRO);
      acao = "PERDEU -> procura DIR";
    } else {
      mover(VEL_RETO, VEL_RETO);
      acao = "PERDEU -> reto";
    }
  }

  if (MODO_TESTE) parar();

  if (DEBUG && millis() - ultimoPrint > 150) {
    ultimoPrint = millis();
    mostrarDebug(acao);
  }
}

// =====================================================
//  Leitura RC dos 8 sensores ao mesmo tempo
// =====================================================
void lerTempos() {
  // 1) Carrega os capacitores dos sensores
  for (int i = 0; i < NUM_SENSORES; i++) {
    pinMode(pinosSensores[i], OUTPUT);
    digitalWrite(pinosSensores[i], HIGH);
  }
  delayMicroseconds(10);

  // 2) Solta os pinos e mede quanto tempo cada um demora para cair
  for (int i = 0; i < NUM_SENSORES; i++) {
    pinMode(pinosSensores[i], INPUT);
    leituras[i] = TEMPO_MAX;
  }

  unsigned long inicio = micros();
  while (micros() - inicio < TEMPO_MAX) {
    unsigned int t = micros() - inicio;
    for (int i = 0; i < NUM_SENSORES; i++) {
      if (t < leituras[i] && digitalRead(pinosSensores[i]) == LOW) {
        leituras[i] = t;
      }
    }
  }
}

void lerSensores() {
  lerTempos();
  for (int i = 0; i < NUM_SENSORES; i++) {
    preto[i] = sensorOk[i] && leituras[i] > limiar[i];
  }
}

// =====================================================
//  Calibração: 5 s deslizando o robô sobre a linha (motores parados)
// =====================================================
void calibrar() {
  unsigned int minimo[NUM_SENSORES];
  unsigned int maximo[NUM_SENSORES];
  for (int i = 0; i < NUM_SENSORES; i++) {
    minimo[i] = TEMPO_MAX;
    maximo[i] = 0;
  }

  Serial.println("Calibrando por 5 s: deslize o robo sobre a linha preta...");
  unsigned long inicio = millis();
  while (millis() - inicio < 5000) {
    lerTempos();
    for (int i = 0; i < NUM_SENSORES; i++) {
      if (leituras[i] < minimo[i]) minimo[i] = leituras[i];
      if (leituras[i] > maximo[i]) maximo[i] = leituras[i];
    }
  }

  Serial.println("Sensor  Branco  Preto  Limiar");
  for (int i = 0; i < NUM_SENSORES; i++) {
    limiar[i] = (minimo[i] + maximo[i]) / 2;
    sensorOk[i] = (maximo[i] - minimo[i]) >= 300;

    char linha[64];
    snprintf(linha, sizeof(linha), "  S%d     %4u   %4u    %4u", i + 1, minimo[i], maximo[i], limiar[i]);
    Serial.print(linha);
    if (!sensorOk[i]) Serial.print("  <-- NAO REAGIU! Sensor ignorado");
    Serial.println();
  }
  Serial.println("Calibracao concluida. Coloque o robo na linha.");
}

// Mostra algo como:
// S1: 542 S2: 574 S3:2500 S4:2500 S5: 644 S6: 773 S7: 751 S8: 600 | [..##....]  << esquerda
void mostrarDebug(String acao) {
  char buf[16];
  for (int i = 0; i < NUM_SENSORES; i++) {
    snprintf(buf, sizeof(buf), "S%d:%4u ", i + 1, leituras[i]);
    Serial.print(buf);
  }
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
  motorA(velEsq);
  motorB(velDir);
}

void girarEsquerda(int vel) {
  motorA(-vel);   // roda esquerda para trás
  motorB(vel);    // roda direita para frente
}

void girarDireita(int vel) {
  motorA(vel);    // roda esquerda para frente
  motorB(-vel);   // roda direita para trás
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
}
