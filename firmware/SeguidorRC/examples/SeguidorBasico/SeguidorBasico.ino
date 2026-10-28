// Seguidor de linha preta sem PID. ESP32-S3 + TB6612FNG + barra de 8 sensores RC.
// Placa na Arduino IDE: "ESP32S3 Dev Module". Monitor Serial a 115200.
//
// O loop e sempre: ler sensores -> (cruzamento?) -> decidir -> mexer motores.

#include <SeguidorRC.h>

using namespace seguidor;

// --- Ligacao (S1 = ponta esquerda ... S8 = ponta direita; os pinos dos sensores precisam ser < 32) ---
const uint8_t pinosSensores[kSensores] = {8, 7, 6, 5, 4, 3, 2, 1};
const PinosPonteH ponte = {.ain1 = 12, .ain2 = 11, .pwma = 13, .bin1 = 14, .bin2 = 21, .pwmb = 47, .stby = 38};

constexpr bool kDebug = true;        // mostra sensores e decisao no Monitor Serial
constexpr bool kModoTeste = false;   // true = motores desligados, so mostra a decisao
constexpr bool kParaNoCruzamento = true;   // 8 sensores no preto: para, avanca ate o eixo, gira no eixo e segue

// Giro no cruzamento. Os dois numeros sao CHUTES: medir na pista (cronometrar um giro de 360 graus e o avanco ate as rodas ficarem sobre o cruzamento).
constexpr unsigned long kAvancoAteEixoMs = 0;     // anda reto antes de girar. 0 = gira onde parou. So aumente se a barra fica longe do eixo e voce quer o eixo sobre o cruzamento
constexpr float kGrausPorSegundoGiro = 180.0f;    // velocidade do giro no lugar com velGiro
constexpr float kGrausNoCruzamento = 90.0f;
int proximoLado = -1;                             // -1 esquerda, +1 direita; alterna a cada cruzamento

BarraRC barra;
Ajustes ajustes;     // velocidades e tempos: mude aqui, ex.: ajustes.velReto = 90;
Memoria memoria;
AjustesCruzamento ajustesCruzamento;   // confirma, saidaMs, paradaMs
MemoriaCruzamento cruzamento;
int cruzamentos = 0;

void mostrarDebug(const char* acao) {
  for (int i = 0; i < kSensores; i++) Serial.printf("S%d:%4u ", i + 1, barra.leituras[i]);
  Serial.print("| [");
  for (int i = 0; i < kSensores; i++) Serial.print(!barra.ok[i] ? 'x' : (barra.preto[i] ? '#' : '.'));
  Serial.printf("]  %s\n", acao);
}

// Gira no lugar, uma roda para frente e a outra para tras (lado -1 = esquerda, +1 = direita), por tempo, e para.
void girarNoEixo(int lado, float graus) {
  mover(ponte, lado * ajustes.velGiro, -lado * ajustes.velGiro);
  delay(static_cast<unsigned long>(graus / kGrausPorSegundoGiro * 1000.0f + 0.5f));
  parar(ponte);
}

// Os 8 sensores no preto, em 2 leituras seguidas, depois do giro.
bool todosNoPreto() {
  for (int tentativa = 0; tentativa < 2; tentativa++) {
    ler(barra);
    if (contarPretos(barra.preto) != kSensores) return false;
    delay(5);
  }
  return true;
}

// Achou "cruzamento" (8 no preto): gira 90 graus no eixo e olha de novo. Num cruzamento de verdade a barra cai sobre a outra rua
// e os 8 acendem outra vez; se nao acenderem era falso alarme, e ele desfaz o giro.
void pararNoCruzamento() {
  parar(ponte);
  if (kModoTeste) { Serial.println(">>> 8 no preto (modo teste: nao gira)"); comecarASair(cruzamento, millis()); return; }

  if (kAvancoAteEixoMs > 0) { mover(ponte, ajustes.velReto, ajustes.velReto); delay(kAvancoAteEixoMs); parar(ponte); }
  const int lado = proximoLado;
  girarNoEixo(lado, kGrausNoCruzamento);
  delay(100);   // assenta antes de ler

  if (todosNoPreto()) {
    cruzamentos++;
    proximoLado = -proximoLado;
    Serial.printf(">>> CRUZAMENTO %d CONFIRMADO (girou %s, 8 no preto de novo) - parado por %lu ms\n", cruzamentos,
                  lado < 0 ? "esquerda" : "direita", ajustesCruzamento.paradaMs);
    delay(ajustesCruzamento.paradaMs);
  } else {
    Serial.printf("    falso alarme: apos girar %s nao acenderam os 8; desfazendo o giro\n", lado < 0 ? "esquerda" : "direita");
    girarNoEixo(-lado, kGrausNoCruzamento);
    parar(ponte);
  }
  esquecerUltimoComando(memoria);
  comecarASair(cruzamento, millis());   // o relogio de saida comeca agora, quando volta a andar
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
  if (kParaNoCruzamento && cruzamentoConfirmado(barra.preto, millis(), ajustesCruzamento, cruzamento)) {
    pararNoCruzamento();
    return;
  }
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
