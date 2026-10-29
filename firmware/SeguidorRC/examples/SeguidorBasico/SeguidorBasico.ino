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
core::Cor corAnterior = core::DESCONHECIDA, corAceita = core::DESCONHECIDA, corBruta = core::DESCONHECIDA;
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
unsigned long semReacaoAte = 0;   // depois de reagir a uma linha proibida (ou de passar o portao) espera a barra sair da faixa

// A cada ~60 ms (a leitura do TCS leva 24 ms: nao ler a cada volta do loop, a correcao precisa de ciclos curtos).
void lerCor() {
#if USA_COR
  static unsigned long ultima = 0;
  if (millis() - ultima < 60) return;
  ultima = millis();
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  corBruta = core::classificarCor(r, g, b, c);
  corAceita = core::filtrarCor(corBruta, corAnterior, corAceita);   // so acredita se repetir em duas leituras
  anotarCor(memCor, corAceita == core::VERMELHO ? CorLinha::Vermelho : (corAceita == core::AZUL ? CorLinha::Azul : CorLinha::Outra), millis());
#endif
}

// --- Ligacao (S1 = ponta esquerda ... S8 = ponta direita; os pinos dos sensores precisam ser < 32) ---
// Barra montada ESPELHADA em relacao ao codigo do colega (confirmado pelo usuario em 2026-10-28): S1 (esquerda) e o pino 1 ... S8 (direita) e o pino 8.
const uint8_t pinosSensores[kSensores] = {1, 2, 3, 4, 5, 6, 7, 8};
const PinosPonteH ponte = {.ain1 = 12, .ain2 = 11, .pwma = 13, .bin1 = 14, .bin2 = 21, .pwmb = 47, .stby = 38};

#ifndef USA_BLE
#define USA_BLE 1   // depuracao por Bluetooth LE (Nordic UART): console_ble.py, nRF Connect ou Serial Bluetooth Terminal (modo BLE)
#endif
#if USA_BLE
#include <seguidor/depuracao_ble.h>
DepuracaoBLE ble;
Tee dbg(Serial, ble);   // tudo que o robo escreve vai ao Monitor Serial (USB) E ao Bluetooth
#else
Print& dbg = Serial;
#endif

bool kDebug = true;        // mostra sensores e decisao (comando "debug on/off")
bool kModoTeste = false;   // true = motores desligados, so mostra a decisao (comando "teste on/off")
bool telemetria = false;   // CSV ~10 Hz: tempo_ms,rumo,S1..S8,esq,dir (comando "tele on/off")
constexpr bool kParaNoCruzamento = true;   // S1 e S8 no preto juntos (acima das outras regras, ate perdido): para e varre 90 graus para cada lado
SaudeSensores saude;                              // sensor preto por mais de 30 s e marcado como ruim
unsigned long tUltimoCruzamento = 0;              // quando o ultimo cruzamento foi confirmado (para saber se a linha sumida e fim de beco)
bool fimDeBecoAvisado = false;

BarraRC barra;
Ajustes ajustes;     // velocidades e tempos: mude aqui, ex.: ajustes.velReto = 90;
Memoria memoria;
AjustesCruzamento ajustesCruzamento;   // confirma, saidaMs
AjustesGiroCruzamento ajustesGiro;     // giro de 90 graus para cada lado
MemoriaCruzamento cruzamento;
Exploracao exploracao;   // cruzamentos ja visitados (relativos a partida): vira para o lado menos explorado
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

// --- Ultrassom HC-SR04 (alvo.h): pronto e DESLIGADO ate o sensor existir. Na varredura das duas pontas ele le o eco ao longo de cada rua (cubo na rua = eco menor que ~330 mm).
// Ligue com -DUSA_ULTRASSOM=1 -DUS_TRIG=<pino> -DUS_ECHO=<pino> (o ECHO do HC-SR04 e 5 V: use divisor para 3,3 V). TCS34725 da frente (cor do cubo): -DUSA_TCS_FRENTE=1,
// fica no mesmo I2C do MPU (core/color_table.h classifica); a logica de coleta e do navegador (ainda so no simulador: test/simulador.cpp).
#ifndef USA_ULTRASSOM
#define USA_ULTRASSOM 0
#endif
#if USA_ULTRASSOM
#ifndef US_TRIG
#define US_TRIG -1
#define US_ECHO -1
#endif
static_assert(US_TRIG >= 0 && US_ECHO >= 0, "defina os pinos do ultrassom (-DUS_TRIG=.. -DUS_ECHO=..)");
// Distancia em mm ate o eco a frente (-1 = sem eco ate ~1,5 m). Bloqueia ate 9 ms no pior caso.
float lerUltrassomMm() {
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);
  const unsigned long us = pulseIn(US_ECHO, HIGH, 9000);
  return us == 0 ? -1.0f : us * 0.343f / 2.0f;
}
#else
float lerUltrassomMm() { return -1.0f; }
#endif

void mostrarDebug(const Comando& c) {
  for (int i = 0; i < kSensores; i++) dbg.printf("S%d:%4u ", i + 1, barra.leituras[i]);
  dbg.print("| [");
  for (int i = 0; i < kSensores; i++) dbg.print(!barra.ok[i] ? 'x' : (barra.preto[i] ? '#' : '.'));
  dbg.printf("]  %s%s%s\n", c.acao, c.estranho ? "  [ESTRANHO: S1 e S8 deviam acender juntos]" : "",
                c.oscilando ? "  [OSCILANDO]" : "");
}

// Varredura das duas pontas acesas: 90 graus para um lado, 90 para o outro, e volta. A velocidade e a da busca (velBuscaExterna) e o
// angulo e ESTIMADO por tempo (grausPorSegundoBusca): nao ha referencia na pista, entao a janela de "90 graus" e larga (giro_cruzamento.h).
// A 90 graus a barra fica ao longo da rua do lado: algum sensor aceso = a rua existe. Nao depende da largura da linha.
constexpr int kMaxAmostras = 2400;
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

// Rege a varredura de 90 graus (giro_cruzamento.h, a maquina de estados e da biblioteca; aqui so se alimentam leituras e motores).
// Devolve o que as duas pontas acesas eram: Cruzamento, Endireitou (estava perdido de lado em cima da linha) ou FalsoAlarme.
Oito varrer(bool vinhaSeguindo, int lado) {
  Varredura v;
  iniciarVarredura(v, lado, vinhaSeguindo);
  float estimado = 0.0f;
#if USA_IMU
  const float rumo0 = rumo;
#endif
  AmostraGiro leitura = {0.0f, barra.preto[0], barra.preto[kSensores - 1], acesos(), lerUltrassomMm()};   // como chegou
  unsigned long t0 = micros();
  const unsigned long inicio = millis();
  while (millis() - inicio < 20000) {
    const ComandoVarredura k = passoVarredura(v, leitura, millis(), ajustes.velBuscaExterna, ajustesGiro, amostras, kMaxAmostras);
    if (k.terminou) break;
    mover(ponte, k.esq, k.dir);
    ler(barra);
    const unsigned long t1 = micros();
#if USA_IMU
    lerImu();
    estimado = rumo - rumo0;   // angulo pelo giroscopio
#else
    estimado += (k.dir > k.esq ? 1 : -1) * ajustes.grausPorSegundoBusca * (t1 - t0) * 1e-6f;   // angulo estimado por tempo
#endif
    t0 = t1;
    leitura = {estimado, barra.preto[0], barra.preto[kSensores - 1], acesos(), lerUltrassomMm()};
  }
  parar(ponte);
  auto rua = [](int x) { return x < 0 ? "?" : (x ? "SIM" : "nao"); };
  dbg.printf("    1o lado: %s | rua a esquerda: %s | rua a direita: %s | %d amostras\n", lado > 0 ? "esquerda" : "direita", rua(v.ruas.ruaEsquerda), rua(v.ruas.ruaDireita), v.n);
  if (v.ruas.usEsq > 0 || v.ruas.usDir > 0 || v.ruas.usFrente > 0) dbg.printf("    ultrassom: esquerda %.0f mm | frente %.0f mm | direita %.0f mm (-1 = nada)\n", v.ruas.usEsq, v.ruas.usFrente, v.ruas.usDir);
  return v.resultado;
}

// S1 e S8 no preto numa linha vermelha ou azul: borda do mapa, nao cruzamento (nada para confirmar girando).
void bordaDoMapa(AcaoCruzamento acao) {
  if (acao == AcaoCruzamento::Portao) {
    dbg.println(">>> PORTAO azul (nosso): segue em frente");
    portaoPendente = false;
    semReacaoAte = millis() + 1500;   // ainda em cima da faixa azul do portao
    esquecerCor(memCor);
    esquecerUltimoComando(memoria);
    tUltimoCruzamento = millis();
    comecarASair(cruzamento, millis());
    return;
  }
  if (acao == AcaoCruzamento::Navegador) {
    dbg.println(">>> BORDA com cubo: parado, a rota e do navegador (ainda nao ligado)");
    parado = true;
    return;
  }
  dbg.println(">>> BORDA do mapa (linha vermelha/azul): meia-volta para voltar ao mapa");
  int n = 0;
  girarAte(0.0f, 180.0f, n);
  esquecerCor(memCor);   // a linha que ficou atras nao conta para o proximo cruzamento
  esquecerUltimoComando(memoria);
  tUltimoCruzamento = millis();
  fimDeBecoAvisado = false;
  cruzamento = MemoriaCruzamento{};   // depois dos 180 graus a barra nao esta mais na faixa da borda: o cruzamento do beco (a 127 mm) precisa ser detectado, o debounce de 300 ms o engolia
}

// As duas pontas acesas (S1 e S8), ACIMA DE TODAS AS OUTRAS REGRAS, em qualquer situacao (inclusive perdido/buscando): para e varre 90 graus, comecando pelo
// lado MENOS EXPLORADO do mapa (rumo do MPU + cruzamentos ja visitados). Seguindo a linha: cruzamento se ha rua dos dois lados. Perdido: era ele de lado em cima da
// linha; se ficou estreito depois de virar, endireitou e segue por ela. A decisao de qual rua seguir sera do navegador (ainda nao ligado aqui).
void pararNoCruzamento() {
  parar(ponte);
  const AcaoCruzamento acao = acaoNaPonta(corRecente(memCor, millis(), kJanelaCorMs), temCubo, portaoPendente);
  if (acao != AcaoCruzamento::Cruzamento) { bordaDoMapa(acao); return; }
  if (kModoTeste) { dbg.println(">>> S1 e S8 no preto (modo teste: nao gira)"); comecarASair(cruzamento, millis()); return; }

  const bool vinhaSeguindo = vinhaSeguindoLinha(memoria, millis(), ajustes);
#if USA_IMU
  const int lado = ladoMenosExplorado(exploracao, rumo, true, memoria.ultimoLado);
#else
  const int lado = ladoMenosExplorado(exploracao, 0.0f, false, memoria.ultimoLado);   // sem MPU nao ha bussola: lado da ultima correcao
#endif
  const Oito oito = varrer(vinhaSeguindo, lado);
  if (oito == Oito::Cruzamento) {
    cruzamentos++;
#if USA_IMU
    registrarCruzamento(exploracao, rumo);
#endif
    dbg.printf(">>> CRUZAMENTO %d CONFIRMADO\n", cruzamentos);
  } else if (oito == Oito::Endireitou) {
    dbg.println(">>> ENDIREITOU: estava de lado em cima da linha, agora segue por ela");
    memoria.perdeuEm = 0;
  } else {
    dbg.println("    falso alarme");
  }
  esquecerUltimoComando(memoria);
  if (oito == Oito::Endireitou) memoria.perdeuEm = 0;
  tUltimoCruzamento = millis();
  fimDeBecoAvisado = false;
  comecarASair(cruzamento, millis());   // o relogio de saida comeca agora, quando volta a andar
}

// --- Comandos recebidos por Bluetooth (uma linha cada). "ajuda" lista. ---

void mostrarAjustes() {
  dbg.printf("velReto=%d velCorrecaoLeve=%d velCorrecaoForte=%d zonaMortaPwm=%d periodoCorrecaoMs=%lu\n", ajustes.velReto, ajustes.velCorrecaoLeve, ajustes.velCorrecaoForte, ajustes.zonaMortaPwm, ajustes.periodoCorrecaoMs);
}

void aoReceberComando(const String& linha) {
  String l = linha;
  l.trim();
  l.toLowerCase();
  const int sp = l.indexOf(' ');
  const String cmd = sp < 0 ? l : l.substring(0, sp);
  const String arg = sp < 0 ? "" : l.substring(sp + 1);
  if (cmd == "ajuda" || cmd == "h" || cmd == "?") {
    dbg.println("comandos: estado | parar | seguir | teste on/off | debug on/off | tele on/off | ajustes | ajuste <nome> <valor>");
    dbg.println("          zerar (rumo=0) | nomes: velReto velCorrecaoLeve velCorrecaoForte zonaMortaPwm periodoCorrecaoMs");
  } else if (cmd == "estado") {
    dbg.printf("t=%lu ms | %s | teste=%d debug=%d tele=%d | cruzamentos=%d | portaoPendente=%d\n", millis(), parado ? "PARADO" : "andando", kModoTeste, kDebug, telemetria, cruzamentos, portaoPendente);
#if USA_IMU
    dbg.printf("rumo=%.1f graus | vies gz=%.4f rad/s\n", rumo, viesGz);
#endif
    mostrarAjustes();
  } else if (cmd == "parar") { parado = true; parar(ponte); dbg.println("ok: parado");
  } else if (cmd == "seguir") { parado = false; dbg.println("ok: andando");
  } else if (cmd == "teste") { kModoTeste = arg == "on"; dbg.printf("ok: modo teste %s\n", kModoTeste ? "ligado (motores desligados)" : "desligado");
  } else if (cmd == "debug") { kDebug = arg == "on"; dbg.printf("ok: debug %s\n", kDebug ? "ligado" : "desligado");
  } else if (cmd == "tele") { telemetria = arg == "on"; if (telemetria) dbg.println("t_ms,rumo,S1,S2,S3,S4,S5,S6,S7,S8,esq,dir"); dbg.printf("ok: telemetria %s\n", telemetria ? "ligada" : "desligada");
  } else if (cmd == "ajustes") { mostrarAjustes();
  } else if (cmd == "zerar") {
#if USA_IMU
    rumo = 0.0f; dbg.println("ok: rumo = 0");
#else
    dbg.println("sem IMU neste build");
#endif
  } else if (cmd == "ajuste") {
    const int sp2 = arg.indexOf(' ');
    if (sp2 < 0) { dbg.println("uso: ajuste <nome> <valor>"); return; }
    const String nome = arg.substring(0, sp2);
    const long v = arg.substring(sp2 + 1).toInt();
    if (nome == "velreto") ajustes.velReto = v;
    else if (nome == "velcorrecaoleve") ajustes.velCorrecaoLeve = v;
    else if (nome == "velcorrecaoforte") ajustes.velCorrecaoForte = v;
    else if (nome == "zonamortapwm") ajustes.zonaMortaPwm = v;
    else if (nome == "periodocorrecaoms") ajustes.periodoCorrecaoMs = v;
    else { dbg.println("nome desconhecido (veja: ajuda)"); return; }
    dbg.print("ok: "); mostrarAjustes();
  } else {
    dbg.printf("comando desconhecido: '%s' (ajuda)\n", cmd.c_str());
  }
}

void setup() {
  Serial.begin(115200);
#if USA_BLE
  ble.iniciar("SeguidorRC", aoReceberComando);
#endif
  iniciar(ponte);
  iniciar(barra, pinosSensores);
#if USA_ULTRASSOM
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);
#endif
#if USA_COR
  Wire.begin(tcsSda, tcsScl);
  if (!tcs.begin(0x29, &Wire)) dbg.println("!!! TCS34725 nao respondeu: sem leitura de cor (a borda sera tratada como cruzamento)");
#endif

#if USA_IMU
  Wire.begin(IMU_SDA, IMU_SCL);
  while (!mpu.begin()) { dbg.println("!!! MPU6050 nao respondeu, tentando de novo..."); delay(1000); }
  dbg.println("Medindo o vies do giroscopio: nao mexa no robo (1 s)");
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
    // mede os graus por segundo numa velocidade RAPIDA (ac.velMedida, 140): e a que passa a ser a da busca, e procurar mais rapido acha mais e antes
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
      dbg.printf("Calibracao dos motores: zona morta %d, giro %.0f graus/s a PWM %d\n", cal.zonaMortaPwm, cal.grausPorSegundo, cal.velMedida);
    } else {
      dbg.println("!!! Calibracao dos motores falhou (o robo nao girou): usando os valores padrao");
    }
  }
  ajustes.lacunaMs = 450;   // a lacuna agora segura o rumo pelo giroscopio, pode ser maior
  rumo = 0.0f;              // posto de frente, apontando para dentro da cidade
#endif
  ajustes = normalizar(ajustes);
  delay(1000);
  // Nao anda com calibracao ruim: sensor morto ou robo parado fazia o carro rodopiar
  while (!calibrar(barra, Serial)) {
    dbg.println("Calibracao INVALIDA (poucos sensores reagiram). Tentando de novo em 2 s...");
    delay(2000);
  }
  delay(2000);   // tempo para colocar o robo na linha
}

void loop() {
#if USA_BLE
  ble.atualizar();
#endif
  if (parado) { parar(ponte); return; }
  ler(barra);
  lerCor();
#if USA_COR
  // Regras 7: "responder de forma imediata sempre que QUALQUER sensor detectar vermelho no chao". Vermelho (limite da cidade) e azul fora do portao (area do outro lado) NUNCA sao
  // rota: os sensores RC leem essas linhas como preto e o carro as seguia como uma rua. Uma leitura crua vale se alguma ponta da barra ve preto; sozinha, a leitura filtrada.
  {
    bool algum = false;
    for (int i = 0; i < kSensores; i++) algum = algum || barra.preto[i];
    const core::Cor viu = algum ? corBruta : corAceita;
    if (millis() >= semReacaoAte && (viu == core::VERMELHO || (viu == core::AZUL && !portaoPendente))) {
      parar(ponte);
      bordaDoMapa(temCubo ? AcaoCruzamento::Navegador : AcaoCruzamento::MeiaVolta);
      semReacaoAte = millis() + 1500;
      return;
    }
  }
#endif
#if USA_IMU
  lerImu();
  const EventoPostura evento = atualizarPickup(pickup, imu, millis(), ajustesInercial);
  if (evento == EventoPostura::FoiLevantado) { parar(ponte); dbg.println(">>> LEVANTADO: motores parados"); }
  if (evento == EventoPostura::Reposicionado) {
    // Depois de levantado ele e SEMPRE posto em algum lugar do verde, apontando para frente: rumo 0, pose desconhecida (o mapa de cubos fica).
    // O localizador (core/localizer.h, initInGreen) tem de receber o mesmo reset quando for ligado aqui.
    reposicionamentos++;
    rumo = 0.0f;
    memoria = Memoria{};
    cruzamento = MemoriaCruzamento{};
    exploracao = Exploracao{};
    ancora = Ancora{};
    tracao = Tracao{};
    esquecerCor(memCor);
    tUltimoCruzamento = 0;
    portaoPendente = true;   // vai entrar de novo pelo nosso portao
    dbg.printf(">>> REPOSICIONADO (%d): rumo 0, de frente\n", reposicionamentos);
  }
  if (!motoresLiberados(pickup)) { parar(ponte); return; }
  if (colisaoDetectada(colisao, imu, true, millis(), ajustesInercial)) dbg.printf(">>> COLISAO forte (pico %.1f g)\n", colisao.picoG);
#endif
  const int travados = atualizarSaude(barra.preto, millis(), saude, barra.ok);
  if (travados > 0) dbg.printf("!!! %d sensor(es) no preto ha mais de 30 s: marcado(s) como ruim(ns) e ignorado(s)\n", travados);
  // Procurando a linha, S1 e S8 acesos nao sao cruzamento: a barra acende inteira ao ficar paralela a linha (robo de lado)
  // Acima de todas as outras regras: as duas pontas acesas valem ate perdido/buscando
  if (kParaNoCruzamento && cruzamentoConfirmado(barra.preto, millis(), ajustesCruzamento, cruzamento)) {
    pararNoCruzamento();
    return;
  }
  const unsigned long perdeuAntes = memoria.perdeuEm;
  Comando c = decidir(barra.preto, millis(), ajustes, memoria);
#if USA_IMU
  if (memoria.perdeuEm != 0 && perdeuAntes == 0) rumoAoPerder = rumo;
  if ((memoria.ultimoEsq != 0 || memoria.ultimoDir != 0) && memoria.perdeuEm != 0 && millis() - memoria.perdeuEm < ajustes.lacunaMs) segurarRumoNaLacuna(c, rumo, rumoAoPerder, ajustes, ajustesInercial);
  if (ancorarNaGrade(rumo, viesGz, imu.gzRadS, ehReto(c), millis(), ancora, ajustesInercial)) dbg.println("!!! rumo suspeito: longe de um multiplo de 90 numa reta");
  const bool comandando = abs(c.esq) >= ajustes.zonaMortaPwm || abs(c.dir) >= ajustes.zonaMortaPwm;
  const AcaoTracao tr = atualizarTracao(tracao, imu, comandando && !c.parar, millis(), ajustesInercial);
  if (tr == AcaoTracao::Reforcar) dbg.printf(">>> andando menos que o esperado: reforco de PWM %d\n", tracao.reforco);
  if (tr == AcaoTracao::Recuar) {
    dbg.println(">>> TRAVADO mesmo com reforco: recua um pouco");
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
    dbg.println(">>> FIM DE BECO: a linha acabou onde o beco termina (borda do mapa)");
  }
  aplicar(ponte, c);

  if (kModoTeste) {
    parar(ponte);
    esquecerUltimoComando(memoria);
  }

  static unsigned long ultimaTele = 0;
  if (telemetria && millis() - ultimaTele >= 100) {
    ultimaTele = millis();
#if USA_IMU
    const float r = rumo;
#else
    const float r = 0.0f;
#endif
    dbg.printf("%lu,%.1f,", millis(), r);
    for (int i = 0; i < kSensores; i++) dbg.printf("%u,", barra.leituras[i]);
    dbg.printf("%d,%d\n", c.esq, c.dir);
  }
  static unsigned long ultimoPrint = 0;
  if (kDebug && millis() - ultimoPrint > 150) {
    ultimoPrint = millis();
    mostrarDebug(c);
  }
}
