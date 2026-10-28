// Seguidor de linha preta sem PID. ESP32-S3 + TB6612FNG + barra de 8 sensores RC.
// Placa na Arduino IDE: "ESP32S3 Dev Module". Monitor Serial a 115200.
//
// O loop e sempre: ler sensores -> decidir -> mexer motores.

#include <SeguidorRC.h>

using namespace seguidor;

// --- Ligacao (S1 = ponta esquerda ... S8 = ponta direita; os pinos dos sensores precisam ser < 32) ---
const uint8_t pinosSensores[kSensores] = {8, 7, 6, 5, 4, 3, 2, 1};
const PinosPonteH ponte = {.ain1 = 11, .ain2 = 12, .pwma = 13, .bin1 = 14, .bin2 = 21, .pwmb = 47, .stby = 38};

constexpr bool kDebug = true;        // mostra sensores e decisao no Monitor Serial
constexpr bool kModoTeste = false;   // true = motores desligados, so mostra a decisao

BarraRC barra;
Ajustes ajustes;     // velocidades e tempos: mude aqui, ex.: ajustes.velReto = 90;
Memoria memoria;

void mostrarDebug(const char* acao) {
  for (int i = 0; i < kSensores; i++) Serial.printf("S%d:%4u ", i + 1, barra.leituras[i]);
  Serial.print("| [");
  for (int i = 0; i < kSensores; i++) Serial.print(!barra.ok[i] ? 'x' : (barra.preto[i] ? '#' : '.'));
  Serial.printf("]  %s\n", acao);
}

void setup() {
  Serial.begin(115200);
  iniciar(ponte);
  iniciar(barra, pinosSensores);

  delay(1000);
  // Nao anda com calibracao ruim: sensor morto ou robo parado fazia o carro rodopiar
  while (!calibrar(barra, Serial)) {
    Serial.println("Calibracao INVALIDA (poucos sensores reagiram). Tentando de novo em 2 s...");
    delay(2000);
  }
  delay(2000);   // tempo para colocar o robo na linha
}

void loop() {
  ler(barra);
  const Comando c = decidir(barra.preto, millis(), ajustes, memoria);
  aplicar(ponte, c);

  if (kModoTeste) {
    parar(ponte);
    esquecerUltimoComando(memoria);
  }

  static unsigned long ultimoPrint = 0;
  if (kDebug && millis() - ultimoPrint > 150) {
    ultimoPrint = millis();
    mostrarDebug(c.acao);
  }
}
