// Seguidor de linha preta sem PID. ESP32-S3 + TB6612FNG + barra de 8 sensores RC.
// Placa na Arduino IDE: "ESP32S3 Dev Module". Monitor Serial a 115200.
//
// O loop e sempre: ler sensores -> (cruzamento?) -> decidir -> mexer motores.

#include <SeguidorRC.h>

using namespace seguidor;

// --- Cor da linha (TCS34725 do chao): as ruas terminam numa linha VERMELHA (cima/baixo) ou AZUL (laterais). Ela acende S1 e S8 como um cruzamento,
// mas e a borda do mapa: nao se confirma girando, o robo da meia-volta e volta para dentro do mapa (ou, com cubo na mao, espera o navegador).
// Desligado ate ligar o sensor: troque para 1, instale a biblioteca "Adafruit TCS34725" e defina os pinos do I2C do TCS (livres: este sketch
// ja usa 11-14, 21, 38 e 47 na ponte H). A tabela de cores e a do firmware principal (core/color_table.h): uma so, nao copie.
#define USA_COR 0
#if USA_COR
#include <Adafruit_TCS34725.h>
#include <Wire.h>

#include "../../../src/core/color_table.h"
constexpr int tcsSda = -1, tcsScl = -1;   // DEFINA
static_assert(tcsSda >= 0 && tcsScl >= 0, "defina os pinos do TCS34725 (tcsSda, tcsScl)");
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);
core::Cor corAnterior = core::DESCONHECIDA, corAceita = core::DESCONHECIDA;
#endif
MemoriaCor memCor;

// --- MPU6050 (inercial.h): rumo ancorado na grade, levantado/reposicionado, travado, colisao, lacuna segurando o rumo, calibracao dos motores.
// Desligado ate ligar o sensor: -DUSA_IMU=1 -DIMU_SDA=<pino> -DIMU_SCL=<pino> (livres: a ponte H ja usa 11-14, 21, 38 e 47). Bitola/eixos: confira o sinal de
// kSinalGz (+ = anti-horario com o MPU de cabeca para cima) girando o robo a mao para a esquerda e olhando o rumo no Monitor Serial.
#ifndef USA_IMU
#define USA_IMU 0
#endif
#if USA_IMU
#include <Adafruit_MPU6050.h>
#include <Wire.h>

#include "../../../src/core/yaw.h"   // dono da integracao do giroscopio (viés + dt)
#ifndef IMU_SDA
#define IMU_SDA -1
#define IMU_SCL -1
#endif
static_assert(IMU_SDA >= 0 && IMU_SCL >= 0, "defina os pinos do MPU6050 (-DIMU_SDA=.. -DIMU_SCL=..)");
constexpr float kSinalGz = 1.0f;
constexpr bool kCalibrarMotores = true;   // no setup, gira no eixo sozinho (~2 s) e mede zona morta e graus por segundo
Adafruit_MPU6050 mpu;
AjustesInercial ajustesInercial;
Imu imu = {0.0f, 0.0f, 0.0f, 1.0f};
float rumo = 0.0f;                 // graus, + = esquerda, 0 = de frente para dentro da cidade (SEMPRE assim depois de levantado)
float viesGz = 0.0f;               // rad/s
float rumoAoPerder = 0.0f;
unsigned long ultimoImuMs = 0;
Ancora ancora;
Pickup pickup;
Tracao tracao;
Colisao colisao;
int reposicionamentos = 0;
#endif
constexpr unsigned long kJanelaCorMs = 400;   // a cor da borda vale por este tempo (o TCS pode estar a frente ou atras da barra)
bool temCubo = false;   // carregando um cubo para guardar: na borda nao da meia-volta, deixa para o navegador (ainda nao ligado)
bool parado = false;
bool portaoPendente = true;   // ainda nao passou pelo nosso portao azul (a primeira azul lida e a nossa)

// A cada ~60 ms (a leitura do TCS leva 24 ms: nao ler a cada volta do loop, a correcao precisa de ciclos curtos).
void lerCor() {
#if USA_COR
  static unsigned long ultima = 0;
  if (millis() - ultima < 60) return;
  ultima = millis();
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  corAceita = core::filtrarCor(core::classificarCor(r, g, b, c), corAnterior, corAceita);   // so acredita se repetir em duas leituras
  anotarCor(memCor, corAceita == core::VERMELHO ? CorLinha::Vermelho : (corAceita == core::AZUL ? CorLinha::Azul : CorLinha::Outra), millis());
#endif
}

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

#if USA_IMU
// Le o MPU e integra o rumo (com o vies medido parado, e corrigido nas retas).
void lerImu() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  imu = {kSinalGz * g.gyro.z, a.acceleration.x / 9.80665f, a.acceleration.y / 9.80665f, a.acceleration.z / 9.80665f};
  const unsigned long agora = millis();
  rumo = core::integrateYaw(rumo, imu.gzRadS, viesGz, (agora - ultimoImuMs) / 1000.0f);
  ultimoImuMs = agora;
}
#endif

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
#if USA_IMU
  const float rumoRef = rumo - agora;   // angulo pelo giroscopio, a partir de onde comecou
#endif
  const int v = ajustes.velBuscaExterna;
  mover(ponte, paraEsquerda ? -v : v, paraEsquerda ? v : -v);   // rodas em sentidos opostos
  unsigned long t0 = micros();
  const unsigned long inicio = millis();
  while ((paraEsquerda ? agora < alvo : agora > alvo) && millis() - inicio < 3000) {
    ler(barra);
    const unsigned long t1 = micros();
#if USA_IMU
    lerImu();
    agora = rumo - rumoRef;
#else
    const float passo = ajustes.grausPorSegundoBusca * (t1 - t0) * 1e-6f;
    agora += paraEsquerda ? passo : -passo;
#endif
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

// S1 e S8 no preto numa linha vermelha ou azul: borda do mapa, nao cruzamento (nada para confirmar girando).
void bordaDoMapa(AcaoCruzamento acao) {
  if (acao == AcaoCruzamento::Portao) {
    Serial.println(">>> PORTAO azul (nosso): segue em frente");
    portaoPendente = false;
    esquecerCor(memCor);
    esquecerUltimoComando(memoria);
    tUltimoCruzamento = millis();
    comecarASair(cruzamento, millis());
    return;
  }
  if (acao == AcaoCruzamento::Navegador) {
    Serial.println(">>> BORDA com cubo: parado, a rota e do navegador (ainda nao ligado)");
    parado = true;
    return;
  }
  Serial.println(">>> BORDA do mapa (linha vermelha/azul): meia-volta para voltar ao mapa");
  int n = 0;
  girarAte(0.0f, 180.0f, n);
  esquecerCor(memCor);   // a linha que ficou atras nao conta para o proximo cruzamento
  esquecerUltimoComando(memoria);
  tUltimoCruzamento = millis();
  fimDeBecoAvisado = false;
  comecarASair(cruzamento, millis());
}

// Achou "cruzamento" (S1 e S8 no preto): para e confirma girando 90 graus para cada lado. A decisao de qual rua seguir sera do navegador (ainda nao ligado
// aqui): por enquanto, depois de confirmar, segue reto.
void pararNoCruzamento() {
  parar(ponte);
  const AcaoCruzamento acao = acaoNaPonta(corRecente(memCor, millis(), kJanelaCorMs), temCubo, portaoPendente);
  if (acao != AcaoCruzamento::Cruzamento) { bordaDoMapa(acao); return; }
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
#if USA_COR
  Wire.begin(tcsSda, tcsScl);
  if (!tcs.begin(0x29, &Wire)) Serial.println("!!! TCS34725 nao respondeu: sem leitura de cor (a borda sera tratada como cruzamento)");
#endif

#if USA_IMU
  Wire.begin(IMU_SDA, IMU_SCL);
  while (!mpu.begin()) { Serial.println("!!! MPU6050 nao respondeu, tentando de novo..."); delay(1000); }
  Serial.println("Medindo o vies do giroscopio: nao mexa no robo (1 s)");
  core::GyroBias vies;
  for (int i = 0; i < 200; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    core::addStillSample(vies, kSinalGz * g.gyro.z);
    delay(5);
  }
  viesGz = vies.bias();
  ultimoImuMs = millis();
  if (kCalibrarMotores) {   // gira no eixo ate o giroscopio acusar: zona morta e graus por segundo de verdade
    Calibracao cal;
    AjustesCalibracao ac;
    ac.velMedida = ajustes.velBuscaExterna;   // mede os graus por segundo NA velocidade em que vai procurar
    ComandoCalibracao k;
    do {
      lerImu();
      k = passoCalibracao(cal, imu.gzRadS - viesGz, millis(), ac);
      mover(ponte, k.esq, k.dir);
      delay(5);
    } while (!k.terminou);
    parar(ponte);
    if (cal.fase == Calibracao::Pronto) {
      ajustes.zonaMortaPwm = cal.zonaMortaPwm;
      ajustes.velBuscaExterna = cal.velMedida;   // a velocidade em que os graus por segundo foram medidos
      ajustes.grausPorSegundoBusca = cal.grausPorSegundo;
      Serial.printf("Calibracao dos motores: zona morta %d, giro %.0f graus/s a PWM %d\n", cal.zonaMortaPwm, cal.grausPorSegundo, cal.velMedida);
    } else {
      Serial.println("!!! Calibracao dos motores falhou (o robo nao girou): usando os valores padrao");
    }
  }
  ajustes.lacunaMs = 450;   // a lacuna agora segura o rumo pelo giroscopio, pode ser maior
  rumo = 0.0f;              // posto de frente, apontando para dentro da cidade
#endif
  ajustes = normalizar(ajustes);
  delay(1000);
  // Nao anda com calibracao ruim: sensor morto ou robo parado fazia o carro rodopiar
  while (!calibrar(barra, Serial)) {
    Serial.println("Calibracao INVALIDA (poucos sensores reagiram). Tentando de novo em 2 s...");
    delay(2000);
  }
  delay(2000);   // tempo para colocar o robo na linha
}

void loop() {
  if (parado) { parar(ponte); return; }
  ler(barra);
  lerCor();
#if USA_IMU
  lerImu();
  const EventoPostura evento = atualizarPickup(pickup, imu, millis(), ajustesInercial);
  if (evento == EventoPostura::FoiLevantado) { parar(ponte); Serial.println(">>> LEVANTADO: motores parados"); }
  if (evento == EventoPostura::Reposicionado) {
    // Depois de levantado ele e SEMPRE posto em algum lugar do verde, apontando para frente: rumo 0, pose desconhecida (o mapa de cubos fica).
    // O localizador (core/localizer.h, initInGreen) tem de receber o mesmo reset quando for ligado aqui.
    reposicionamentos++;
    rumo = 0.0f;
    memoria = Memoria{};
    cruzamento = MemoriaCruzamento{};
    ancora = Ancora{};
    tracao = Tracao{};
    esquecerCor(memCor);
    tUltimoCruzamento = 0;
    portaoPendente = true;   // vai entrar de novo pelo nosso portao
    Serial.printf(">>> REPOSICIONADO (%d): rumo 0, de frente\n", reposicionamentos);
  }
  if (!motoresLiberados(pickup)) { parar(ponte); return; }
  if (colisaoDetectada(colisao, imu, true, millis(), ajustesInercial)) Serial.printf(">>> COLISAO forte (pico %.1f g)\n", colisao.picoG);
#endif
  const int travados = atualizarSaude(barra.preto, millis(), saude, barra.ok);
  if (travados > 0) Serial.printf("!!! %d sensor(es) no preto ha mais de 30 s: marcado(s) como ruim(ns) e ignorado(s)\n", travados);
  // Procurando a linha, S1 e S8 acesos nao sao cruzamento: a barra acende inteira ao ficar paralela a linha (robo de lado)
  if (kParaNoCruzamento && !estaBuscando(memoria, millis(), ajustes) && cruzamentoConfirmado(barra.preto, millis(), ajustesCruzamento, cruzamento)) {
    pararNoCruzamento();
    return;
  }
  const unsigned long perdeuAntes = memoria.perdeuEm;
  Comando c = decidir(barra.preto, millis(), ajustes, memoria);
#if USA_IMU
  if (memoria.perdeuEm != 0 && perdeuAntes == 0) rumoAoPerder = rumo;
  if ((memoria.ultimoEsq != 0 || memoria.ultimoDir != 0) && memoria.perdeuEm != 0 && millis() - memoria.perdeuEm < ajustes.lacunaMs) segurarRumoNaLacuna(c, rumo, rumoAoPerder, ajustes, ajustesInercial);
  if (ancorarNaGrade(rumo, viesGz, imu.gzRadS, ehReto(c), millis(), ancora, ajustesInercial)) Serial.println("!!! rumo suspeito: longe de um multiplo de 90 numa reta");
  const bool comandando = abs(c.esq) >= ajustes.zonaMortaPwm || abs(c.dir) >= ajustes.zonaMortaPwm;
  const AcaoTracao tr = atualizarTracao(tracao, imu, comandando && !c.parar, millis(), ajustesInercial);
  if (tr == AcaoTracao::Reforcar) Serial.printf(">>> andando menos que o esperado: reforco de PWM %d\n", tracao.reforco);
  if (tr == AcaoTracao::Recuar) {
    Serial.println(">>> TRAVADO mesmo com reforco: recua um pouco");
    mover(ponte, -ajustes.velReto, -ajustes.velReto);
    delay(300);
    parar(ponte);
    tracao.reforco = 0;
    return;
  }
  aplicarReforco(c, tracao.reforco);
#endif

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
