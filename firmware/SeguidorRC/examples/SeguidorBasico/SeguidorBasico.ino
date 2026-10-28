// Seguidor de linha preta sem PID. ESP32-S3 + TB6612FNG + barra de 8 sensores RC.
// Placa na Arduino IDE: "ESP32S3 Dev Module". Monitor Serial a 115200.
//
// O loop e sempre: ler sensores -> (cruzamento?) -> decidir -> mexer motores.

#include <SeguidorRC.h>

using namespace seguidor;

// --- Ligacao (S1 = ponta esquerda ... S8 = ponta direita; os pinos dos sensores precisam ser < 32) ---
// Barra montada ESPELHADA em relacao ao codigo do colega (confirmado pelo usuario em 2026-10-28): S1 (esquerda) e o pino 1 ... S8 (direita) e o pino 8.
const uint8_t pinosSensores[kSensores] = {1, 2, 3, 4, 5, 6, 7, 8};
const PinosPonteH ponte = {.ain1 = 12, .ain2 = 11, .pwma = 13, .bin1 = 14, .bin2 = 21, .pwmb = 47, .stby = 38};

constexpr bool kDebug = true;        // mostra sensores e decisao no Monitor Serial
constexpr bool kModoTeste = false;   // true = motores desligados, so mostra a decisao
constexpr bool kParaNoCruzamento = true;   // S1 e S8 no preto juntos: para, gira 90 graus para cada lado lendo S1, S8 e o centro, e confirma se e cruzamento
SaudeSensores saude;                              // sensor preto por mais de 30 s e marcado como ruim
unsigned long tUltimoCruzamento = 0;              // quando o ultimo cruzamento foi confirmado (para saber se a linha sumida e fim de beco)
bool fimDeBecoAvisado = false;

BarraRC barra;
Ajustes ajustes;     // velocidades e tempos: mude aqui, ex.: ajustes.velReto = 90;
Memoria memoria;
AjustesCruzamento ajustesCruzamento;   // confirma, saidaMs
AjustesGiroCruzamento ajustesGiro;     // giro de 90 graus para cada lado
MemoriaCruzamento cruzamento;
int cruzamentos = 0;

void mostrarDebug(const Comando& c) {
  for (int i = 0; i < kSensores; i++) Serial.printf("S%d:%4u ", i + 1, barra.leituras[i]);
  Serial.print("| [");
  for (int i = 0; i < kSensores; i++) Serial.print(!barra.ok[i] ? 'x' : (barra.preto[i] ? '#' : '.'));
  Serial.printf("]  %s%s%s\n", c.acao, c.estranho ? "  [ESTRANHO: S1 e S8 deviam acender juntos]" : "",
                c.oscilando ? "  [OSCILANDO]" : "");
}

// Giro de confirmacao e de olhar as ruas: 90 graus para a esquerda, 90 para a direita, e volta. A velocidade e a da busca (velBuscaExterna) e o
// angulo e ESTIMADO por tempo (grausPorSegundoBusca): nao ha referencia na pista, entao a janela de "90 graus" e larga (giro_cruzamento.h).
// A 90 graus a barra fica ao longo da rua do lado: algum sensor aceso = a rua existe. Nao depende da largura da linha.
constexpr int kMaxAmostras = 1800;
AmostraGiro amostras[kMaxAmostras];

int acesos() {
  int n = 0;
  for (int i = 0; i < kSensores; i++) n += barra.preto[i] ? 1 : 0;
  return n;
}

// Gira no eixo ate o angulo estimado chegar em `alvo` (+ = esquerda), lendo a barra a cada passo. Devolve o angulo estimado em que parou.
float girarAte(float agora, float alvo, int& n) {
  const bool paraEsquerda = alvo > agora;
  const int v = ajustes.velBuscaExterna;
  mover(ponte, paraEsquerda ? -v : v, paraEsquerda ? v : -v);   // rodas em sentidos opostos
  unsigned long t0 = micros();
  const unsigned long inicio = millis();
  while ((paraEsquerda ? agora < alvo : agora > alvo) && millis() - inicio < 3000) {
    ler(barra);
    const unsigned long t1 = micros();
    const float passo = ajustes.grausPorSegundoBusca * (t1 - t0) * 1e-6f;
    agora += paraEsquerda ? passo : -passo;
    t0 = t1;
    if (n < kMaxAmostras) amostras[n++] = {agora, barra.preto[0], barra.preto[kSensores - 1], acesos()};
  }
  parar(ponte);
  return agora;
}

// Gira 90 graus para a esquerda, 90 para a direita e volta. Cruzamento = ha rua dos dois lados.
bool confirmarGirando(ResultadoGiro& r) {
  int n = 0;
  amostras[n++] = {0.0f, barra.preto[0], barra.preto[kSensores - 1], acesos()};   // como chegou
  float g = girarAte(0.0f, ajustesGiro.giroGraus, n);
  g = girarAte(g, -ajustesGiro.giroGraus, n);
  girarAte(g, 0.0f, n);   // volta ao centro

  r = avaliarGiro(amostras, n, ajustesGiro);
  auto rua = [](int v) { return v < 0 ? "?" : (v ? "SIM" : "nao"); };
  Serial.printf("    rua a esquerda: %s | rua a direita: %s | %d amostras\n", rua(r.ruaEsquerda), rua(r.ruaDireita), n);
  return r.cruzamento;
}

// Achou "cruzamento" (S1 e S8 no preto): para e confirma girando 90 graus para cada lado. A decisao de qual rua seguir sera do navegador (ainda nao ligado
// aqui): por enquanto, depois de confirmar, segue reto.
void pararNoCruzamento() {
  parar(ponte);
  if (kModoTeste) { Serial.println(">>> S1 e S8 no preto (modo teste: nao gira)"); comecarASair(cruzamento, millis()); return; }

  ResultadoGiro giro;
  if (confirmarGirando(giro)) {
    cruzamentos++;
    Serial.printf(">>> CRUZAMENTO %d CONFIRMADO\n", cruzamentos);
  } else {
    Serial.println("    falso alarme: nao ha rua dos dois lados");
  }
  esquecerUltimoComando(memoria);
  tUltimoCruzamento = millis();
  fimDeBecoAvisado = false;
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
  const int travados = atualizarSaude(barra.preto, millis(), saude, barra.ok);
  if (travados > 0) Serial.printf("!!! %d sensor(es) no preto ha mais de 30 s: marcado(s) como ruim(ns) e ignorado(s)\n", travados);
  // Procurando a linha, S1 e S8 acesos nao sao cruzamento: a barra acende inteira ao ficar paralela a linha (robo de lado)
  if (kParaNoCruzamento && !estaBuscando(memoria, millis(), ajustes) && cruzamentoConfirmado(barra.preto, millis(), ajustesCruzamento, cruzamento)) {
    pararNoCruzamento();
    return;
  }
  const Comando c = decidir(barra.preto, millis(), ajustes, memoria);

  // A linha sumiu: falha curta, fim de beco (a rua acabou no limite do mapa) ou perdida? O fim de beco e evidencia de borda.
  const unsigned long perdidoHa = memoria.perdeuEm ? millis() - memoria.perdeuEm : 0;
  if (!fimDeBecoAvisado && tUltimoCruzamento &&
      classificarPerda(perdidoHa, millis() - tUltimoCruzamento, ajustes.lacunaMs, ajustesCruzamento) == Perda::FimDeBeco) {
    fimDeBecoAvisado = true;
    Serial.println(">>> FIM DE BECO: a linha acabou onde o beco termina (borda do mapa)");
  }
  aplicar(ponte, c);

  if (kModoTeste) {
    parar(ponte);
    esquecerUltimoComando(memoria);
  }

  static unsigned long ultimoPrint = 0;
  if (kDebug && millis() - ultimoPrint > 150) {
    ultimoPrint = millis();
    mostrarDebug(c);
  }
}
