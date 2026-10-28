// Simulador do carrinho seguidor no PC: cinematica diferencial + barra de 8 sensores sobre uma pista de linhas, rodando a logica REAL da biblioteca
// (decidir, cruzamentoConfirmado, giro de 90 graus, classificarPerda). Nao testa eletronica: testa se as REGRAS levam o carro onde deviam.
//
//   g++ -std=c++17 -Wall -Isrc test/simulador.cpp src/seguidor/decisao.cpp -o /tmp/simulador && /tmp/simulador [pasta_para_svg]
//
// Modelo (tudo CHUTE ate medir na pista; os numeros ficam em ParamsFisicos):
//   - carro 170 x 200 mm, eixo no meio, barra de 75 mm a 30 mm a frente do eixo, linha de 20 mm, sensor = ponto;
//   - 4 TT sem encoder: abaixo de ~65 de PWM a roda nao anda; acima, velocidade proporcional ao PWM; atraso de primeira ordem (tau);
//   - girar com rodas opostas derrapa: a velocidade angular real e `derrapagem` x a ideal;
//   - o laco espelha o loop() do SeguidorBasico.ino (ler -> cruzamento? -> decidir -> aplicar; giro de confirmacao com o angulo estimado por tempo).
//     Se o .ino mudar, mude aqui junto (a copia do giro e a unica duplicacao).
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <string>
#include <vector>

#include "seguidor/borda.h"
#include "seguidor/calibracao.h"
#include "seguidor/inercial.h"
#include "seguidor/cruzamento.h"
#include "seguidor/decisao.h"
#include "seguidor/exploracao.h"
#include "seguidor/giro_cruzamento.h"

using namespace seguidor;

// ---------------------------------------------------------------- pista

enum CorSeg { kPreta = 0, kVermelha = 1, kAzul = 2 };   // vermelho e azul valem como PRETO para os sensores RC (suposto: confirme na pista)

struct Seg { double x0, y0, x1, y1; int cor = kPreta; };

struct Ret { double x0, y0, x1, y1; int cor; };   // so desenho (zonas e quadrados dos cubos): nao e sentido pelos sensores RC

struct Pista {
  std::vector<Seg> segs;
  std::vector<Ret> decor;
  double largura = 20.0;

  void reta(double x0, double y0, double x1, double y1, int cor = kPreta) { segs.push_back({x0, y0, x1, y1, cor}); }
  // Arco de centro (cx,cy) e raio r de a0 a a1 (graus), em segmentos de ~5 mm.
  void arco(double cx, double cy, double r, double a0, double a1) {
    const int n = std::max(2, static_cast<int>(fabs(a1 - a0) * M_PI / 180.0 * r / 5.0));
    for (int i = 0; i < n; i++) {
      const double p = (a0 + (a1 - a0) * i / n) * M_PI / 180.0, q = (a0 + (a1 - a0) * (i + 1) / n) * M_PI / 180.0;
      reta(cx + r * cos(p), cy + r * sin(p), cx + r * cos(q), cy + r * sin(q));
    }
  }
  static double distSeg(const Seg& s, double x, double y) {
    const double dx = s.x1 - s.x0, dy = s.y1 - s.y0, l2 = dx * dx + dy * dy;
    double t = l2 > 0 ? ((x - s.x0) * dx + (y - s.y0) * dy) / l2 : 0.0;
    t = std::max(0.0, std::min(1.0, t));
    return hypot(x - (s.x0 + t * dx), y - (s.y0 + t * dy));
  }
  double distancia(double x, double y) const {
    double d = 1e9;
    for (const Seg& s : segs) d = std::min(d, distSeg(s, x, y));
    return d;
  }
  bool preto(double x, double y) const { return distancia(x, y) < largura / 2.0; }
  // Cor da linha sob o ponto (a mais proxima dentro da largura); kPreta tambem vale para fora da linha: so interessa vermelho e azul.
  int cor(double x, double y) const {
    int c = kPreta;
    for (const Seg& s : segs) {
      if (s.cor != kPreta && distSeg(s, x, y) < largura / 2.0) c = s.cor;   // a mancha colorida sob o sensor vence a linha preta
    }
    return c;
  }
};

// ---------------------------------------------------------------- carro

struct ParamsFisicos {
  double bitolaMm = 150.0;        // distancia real entre as rodas (a decisao supoe 140: erro de proposito)
  double mmSPorPwm = 2.31;        // velocidade de cada roda = isto x PWM (110 -> ~254 mm/s, igual ao chute velMmS)
  double zonaMortaPwm = 65.0;     // abaixo disto a roda nao anda
  double derrapagem = 0.8;        // fracao da velocidade angular ideal que sobra com as 4 rodas derrapando
  double tauMs = 50.0;            // atraso de primeira ordem das rodas
  double barraAFrenteMm = 30.0;   // barra a frente do eixo
  double barraMm = 75.0;
  double dtMs = 5.0;              // um ciclo de leitura+decisao
  double viesGzGrauS = 1.2;       // vies REAL do giroscopio (o firmware mede um estimado parado no inicio)
  double derivaViesGrauS = 0.0;   // quanto o vies anda depois da calibracao inicial (temperatura)
};

Ajustes gAjustes;   // os cenarios partem destes (mude na linha de comando para varrer parametros da decisao)
bool gSo_Placar = false;
ParamsFisicos gFis;   // os cenarios usam este; da para mudar na linha de comando (veja main)

struct Estado { double x, y, th; };   // eixo (mm), rumo (rad, 0 = +x, anti-horario)

class Carro {
 public:
  Carro(const Pista& p, Estado e, const ParamsFisicos& f = gFis) : pista_(p), e_(e), f_(f) { grava(); }

  Estado estado() const { return e_; }

  // ---- MPU simulado (gz em rad/s COM vies, aceleracao em g). Ruido deterministico (LCG): o mesmo resultado a cada execucao.
  Imu imu() {
    const double vies = (f_.viesGzGrauS + f_.derivaViesGrauS * (levantado_ ? 0.0 : t_ / 1000.0)) / 57.29578;
    Imu m;
    if (levantado_) {   // nas maos: inclinado ~35 graus, balancando
      m.gzRadS = static_cast<float>(vies + 0.3 * ruido());
      m.ax = static_cast<float>(0.57 + 0.15 * ruido());
      m.ay = static_cast<float>(0.15 * ruido());
      m.az = static_cast<float>(0.82 + 0.15 * ruido());
      return m;
    }
    const bool andando = fabs(vMmS_) > 5.0 || fabs(wRadS_) > 0.05;
    const double sigma = andando ? 0.04 : 0.004;
    m.gzRadS = static_cast<float>(wRadS_ + vies + 0.003 * ruido());
    m.ax = static_cast<float>(aMmS2_ / 9810.0 + sigma * ruido() + (impulsoTicks_ > 0 ? impulsoG_ : 0.0));
    m.ay = static_cast<float>(sigma * ruido());
    m.az = static_cast<float>(1.0 + sigma * ruido());
    if (impulsoTicks_ > 0) impulsoTicks_--;
    return m;
  }
  void levantar(bool sim) { levantado_ = sim; if (sim) { vE_ = vD_ = 0; vMmS_ = wRadS_ = aMmS2_ = 0; } }
  void limparRastro() { rastro_.clear(); bits_.clear(); grava(); }   // o que vem depois e o que interessa mostrar
  void colocarEm(Estado e) { e_ = e; vE_ = vD_ = 0; grava(); }   // solto no chao em outro lugar
  void bater(double g, int ticks) { impulsoG_ = g; impulsoTicks_ = ticks; }   // pancada horizontal de g por `ticks` ciclos
  bool levantado() const { return levantado_; }
  double roda(bool esquerda) const { return esquerda ? alvoE_ : alvoD_; }   // alvo da roda (mm/s) para conferir que parou
  unsigned long agoraMs() const { return t_; }
  const std::vector<Estado>& rastro() const { return rastro_; }
  const std::vector<uint8_t>& bits() const { return bits_; }
  double erroMax() const { return erroMax_; }
  double erroRms() const { return n_ ? sqrt(soma2_ / n_) : 0.0; }   // erro medio quadratico da barra ate a linha, a cada ciclo

  void ponto(int i, double& x, double& y) const {
    const double lat = f_.barraMm / 2.0 - i * f_.barraMm / (kSensores - 1);   // S1 = esquerda = +lado
    x = e_.x + f_.barraAFrenteMm * cos(e_.th) - lat * sin(e_.th);
    y = e_.y + f_.barraAFrenteMm * sin(e_.th) + lat * cos(e_.th);
  }
  void ler(bool preto[kSensores]) const {
    for (int i = 0; i < kSensores; i++) {
      double x, y;
      ponto(i, x, y);
      preto[i] = pista_.preto(x, y);
    }
  }
  // Cor do chao no TCS: suposto no centro da barra (a posicao real e pergunta aberta nos docs).
  int corChao() const {
    double x, y, x2, y2;
    ponto(3, x, y);
    ponto(4, x2, y2);
    return pista_.cor((x + x2) / 2, (y + y2) / 2);
  }
  double erroBarra() const {   // distancia do centro da barra ate a linha mais proxima
    double x, y;
    ponto(3, x, y);
    double x2, y2;
    ponto(4, x2, y2);
    return pista_.distancia((x + x2) / 2, (y + y2) / 2);
  }

  void comandar(int esq, int dir) { alvoE_ = alvo(esq); alvoD_ = alvo(dir); }
  void parar() { alvoE_ = alvoD_ = 0; }

  // Passa um ciclo (dtMs) com o comando atual.
  void avancar() {
    if (levantado_) { t_ += static_cast<unsigned long>(f_.dtMs); return; }   // as rodas giram no ar: nao sai do lugar
    const double dt = f_.dtMs / 1000.0, k = 1.0 - exp(-f_.dtMs / f_.tauMs);
    vE_ += (alvoE_ - vE_) * k;
    vD_ += (alvoD_ - vD_) * k;
    const double v = (vE_ + vD_) / 2.0, w = (vD_ - vE_) / f_.bitolaMm * f_.derrapagem;   // w > 0 = anti-horario (esquerda)
    aMmS2_ = (v - vMmS_) / dt;
    vMmS_ = v;
    wRadS_ = w;
    // roda direita mais rapida -> gira para a esquerda; passo medio do rumo
    const double thMeio = e_.th + w * dt / 2.0;
    e_.x += v * cos(thMeio) * dt;
    e_.y += v * sin(thMeio) * dt;
    e_.th += w * dt;
    t_ += static_cast<unsigned long>(f_.dtMs);
    const double eb = erroBarra();
    erroMax_ = std::max(erroMax_, eb);
    soma2_ += eb * eb;
    n_++;
    if (t_ % 20 == 0) grava();
  }

 private:
  double alvo(int pwm) const { return abs(pwm) < f_.zonaMortaPwm ? 0.0 : (pwm > 0 ? 1.0 : -1.0) * abs(pwm) * f_.mmSPorPwm; }
  void grava() {
    rastro_.push_back(e_);
    bool p[kSensores];
    ler(p);
    uint8_t b = 0;
    for (int i = 0; i < kSensores; i++) b |= p[i] ? (1 << i) : 0;
    bits_.push_back(b);
  }

  const Pista& pista_;
  Estado e_;
  ParamsFisicos f_;
  double ruido() { seed_ = seed_ * 6364136223846793005ULL + 1442695040888963407ULL; return ((seed_ >> 33) & 0xFFFF) / 32768.0 - 1.0; }   // -1..1
  double vE_ = 0, vD_ = 0, alvoE_ = 0, alvoD_ = 0, erroMax_ = 0, soma2_ = 0;
  double vMmS_ = 0, wRadS_ = 0, aMmS2_ = 0, impulsoG_ = 0;
  int impulsoTicks_ = 0;
  bool levantado_ = false;
  unsigned long long seed_ = 12345;
  long n_ = 0;
  unsigned long t_ = 0;
  std::vector<Estado> rastro_;
  std::vector<uint8_t> bits_;
};

// ---------------------------------------------------------------- firmware simulado (espelha SeguidorBasico.ino)

struct Firmware {
  Ajustes ajustes = normalizar(gAjustes);
  AjustesCruzamento ajC;
  AjustesGiroCruzamento ajG;
  Memoria memoria;
  MemoriaCruzamento cruz;
  int cruzamentos = 0, falsosAlarmes = 0, ticksBuscando = 0, ticksPerdido = 0;
  bool fimDeBeco = false, fimDeBecoAvisado = false;
  unsigned long tUltimoCruzamento = 0;
  // relatorio do ultimo giro
  float giroDriftGraus = 0;
  int ruaEsq = -1, ruaDir = -1, ruaEsqVerdade = -1, ruaDirVerdade = -1;
  bool girou = false, parado = false;
  Estado paradaEm = {0, 0, 0};   // onde o eixo estava quando o cruzamento foi confirmado
  bool paraNoCruzamento = true;
  bool temCubo = false;
  MemoriaCor memCor;
  static constexpr unsigned long kJanelaCorMs = 400;
  int bordas = 0, meiasVoltas = 0, paradasNavegador = 0, portoes = 0, endireitou = 0;
  Exploracao expl;
  Varredura varr;
  static constexpr int kBufVarr = 2400;
  AmostraGiro bufVarr[kBufVarr];
  Oito ultimoOito = Oito::FalsoAlarme;
  bool portaoPendente = false;   // o cenario do tabuleiro liga: a primeira azul e o nosso portao
  std::string rota;              // o que fazer em cada cruzamento confirmado: S (reto), L, R (virar para a rua do lado). O navegador de verdade ainda nao esta ligado.
  size_t rotaI = 0;

  // ---- MPU (inercial.h). Desligado por padrao: os cenarios antigos rodam como antes.
  bool usaImu = false, usaAncora = true;
  AjustesInercial ai;
  float rumo = 0.0f, vies = 0.0f;   // rumo em graus (+ esquerda, 0 = de frente); vies em rad/s (estimado parado no inicio)
  Imu ultimoImu = {0, 0, 0, 1};
  unsigned long ultimoMsImu = 0;
  Ancora ancora;
  Pickup pickup;
  Tracao tracao;
  Colisao colisao;
  float rumoAoPerder = 0.0f;
  bool rumoSuspeito = false;
  int levantados = 0, reposicionados = 0, colisoes = 0, reforcos = 0, recuos = 0, ticksMotorLevantado = 0;

  // Liga o MPU: mede o vies com o robo parado (1 s) e, como a lacuna agora segura o rumo, estica a lacuna.
  void ligarImu(Carro& c) {
    usaImu = true;
    float soma = 0.0f;
    for (int i = 0; i < 200; i++) { c.avancar(); soma += c.imu().gzRadS; }
    vies = soma / 200.0f;
    ultimoMsImu = c.agoraMs();
    ajustes.lacunaMs = 450;
  }

  // Avanca um ciclo e le o MPU (integra o rumo com o vies estimado).
  void tick(Carro& c) {
    c.avancar();
    if (!usaImu) return;
    ultimoImu = c.imu();
    const unsigned long agora = c.agoraMs();
    rumo += (ultimoImu.gzRadS - vies) * kRadParaGrau * static_cast<float>(agora - ultimoMsImu) / 1000.0f;
    ultimoMsImu = agora;
  }

  static void apaga(int& v, float x) { v = static_cast<int>(x); }

  static int acesos(const bool p[kSensores]) {
    int n = 0;
    for (int i = 0; i < kSensores; i++) n += p[i] ? 1 : 0;
    return n;
  }

  // Gira no eixo ate o angulo ESTIMADO (por tempo) chegar em alvo (espelho de girarAte do .ino). Guarda tambem o angulo VERDADEIRO de cada amostra.
  float girarAte(Carro& c, float agora, float alvo, std::vector<AmostraGiro>& est, std::vector<AmostraGiro>& ver, double thRef, float rumoRef = 0.0f) {
    const bool esq = alvo > agora;
    const int v = ajustes.velBuscaExterna;
    c.comandar(esq ? -v : v, esq ? v : -v);
    const unsigned long inicio = c.agoraMs();
    while ((esq ? agora < alvo : agora > alvo) && c.agoraMs() - inicio < 3000) {
      tick(c);
      bool p[kSensores];
      c.ler(p);
      if (usaImu) agora = rumo - rumoRef;   // angulo pelo giroscopio
      else agora += (esq ? 1 : -1) * ajustes.grausPorSegundoBusca * 0.005f;   // angulo estimado por tempo
      const float verdade = static_cast<float>((c.estado().th - thRef) * 180.0 / M_PI);
      est.push_back({agora, p[0], p[7], acesos(p)});
      ver.push_back({verdade, p[0], p[7], acesos(p)});
    }
    c.parar();
    return agora;
  }

  void pararNoCruzamento(Carro& c, bool preto[kSensores]) {
    c.parar();
    paradaEm = c.estado();
    const AcaoCruzamento acao = acaoNaPonta(corRecente(memCor, c.agoraMs(), kJanelaCorMs), temCubo, portaoPendente);
    if (acao == AcaoCruzamento::Portao) {   // nosso portao azul: segue em frente
      portoes++;
      portaoPendente = false;
      esquecerCor(memCor);
      esquecerUltimoComando(memoria);
      tUltimoCruzamento = c.agoraMs();
      comecarASair(cruz, c.agoraMs());
      return;
    }
    if (acao != AcaoCruzamento::Cruzamento) {   // borda do mapa: nao gira para confirmar
      bordas++;
      if (acao == AcaoCruzamento::MeiaVolta) {
        std::vector<AmostraGiro> est, ver;
        girarAte(c, 0.0f, 180.0f, est, ver, c.estado().th, rumo);
        meiasVoltas++;
        esquecerUltimoComando(memoria);
        esquecerCor(memCor);
        comecarASair(cruz, c.agoraMs());
      } else {
        paradasNavegador++;
        parado = true;
      }
      tUltimoCruzamento = c.agoraMs();
      return;
    }
    // As duas pontas acesas, em QUALQUER situacao (inclusive perdido/buscando): varredura de 90 graus a partir do lado menos explorado
    const bool vinhaSeguindo = !estaBuscando(memoria, c.agoraMs(), ajustes);
    const int lado = ladoMenosExplorado(expl, rumo, usaImu, memoria.ultimoLado);
    const double th0 = c.estado().th;   // angulo verdadeiro medido a partir de como chegou
    const float rumo0 = rumo;
    iniciarVarredura(varr, lado, vinhaSeguindo);
    float estimado = 0.0f;   // sem MPU: angulo estimado por tempo girando
    AmostraGiro leitura{0.0f, preto[0], preto[7], acesos(preto)};
    for (int i = 0; i < 4000; i++) {   // 20 s no maximo
      const ComandoVarredura k = passoVarredura(varr, leitura, c.agoraMs(), ajustes.velBuscaExterna, ajG, bufVarr, kBufVarr);
      c.comandar(k.esq, k.dir);
      if (k.terminou) break;
      tick(c);
      bool p[kSensores];
      c.ler(p);
      if (usaImu) estimado = rumo - rumo0;
      else estimado += (k.dir > k.esq ? 1 : -1) * ajustes.grausPorSegundoBusca * 0.005f;
      leitura = {estimado, p[0], p[7], acesos(p)};
    }
    c.parar();
    giroDriftGraus = static_cast<float>(remainder(c.estado().th - th0, 2 * M_PI) * 180.0 / M_PI);
    const ResultadoGiro& r = varr.ruas;
    ruaEsq = r.ruaEsquerda;
    ruaDir = r.ruaDireita;
    ruaEsqVerdade = ruaDirVerdade = -1;
    girou = true;
    ultimoOito = varr.resultado;
    if (getenv("SIM_DEBUG")) fprintf(stderr, "  [t=%.1fs] parou em (%.0f, %.0f) rumo %.0f, %s, 1o lado %s: ruas esq %d dir %d -> %s\n", c.agoraMs() / 1000.0, paradaEm.x, paradaEm.y, paradaEm.th * 57.29578, vinhaSeguindo ? "seguindo" : "PERDIDO", lado > 0 ? "esq" : "dir", r.ruaEsquerda, r.ruaDireita,
                                       varr.resultado == Oito::Cruzamento ? "CRUZAMENTO" : (varr.resultado == Oito::Endireitou ? "ENDIREITOU" : "falso alarme"));
    if (varr.resultado == Oito::Cruzamento) { cruzamentos++; registrarCruzamento(expl, rumo); }
    else if (varr.resultado == Oito::Endireitou) endireitou++;
    else falsosAlarmes++;
    esquecerUltimoComando(memoria);
    if (varr.resultado == Oito::Endireitou) memoria.perdeuEm = 0;   // estava de lado em cima da linha e agora esta alinhado: volta a seguir
    tUltimoCruzamento = c.agoraMs();
    fimDeBecoAvisado = false;
    if (varr.resultado == Oito::Cruzamento) {
      const char a = rotaI < rota.size() ? rota[rotaI++] : 'S';
      if (a == 'L' || a == 'R') virarParaRua(c, a == 'L');
    }
    comecarASair(cruz, c.agoraMs());
  }

  // Vira para a rua do lado. A barra para na BORDA da rua que cruza e o eixo ainda esta ~40 mm antes do centro: girar dali poe a rua a 40 mm do eixo,
  // fora da barra. Entao avanca ate as duas pontas apagarem (a barra atravessou a rua, sem depender da largura da linha) e so entao gira 90 graus.
  void virarParaRua(Carro& c, bool esquerda) {
    c.comandar(ajustes.velReto, ajustes.velReto);
    for (int i = 0; i < 120; i++) {   // ate 600 ms
      tick(c);
      bool p[kSensores];
      c.ler(p);
      if (!p[0] && !p[7]) break;
    }
    c.parar();
    std::vector<AmostraGiro> est, ver;
    girarAte(c, 0.0f, esquerda ? 90.0f : -90.0f, est, ver, c.estado().th, rumo);
    esquecerUltimoComando(memoria);
    memoria.perdeuEm = 0;
  }

  // Um ciclo do loop().
  void passo(Carro& c) {
    if (parado) { c.parar(); tick(c); return; }
    if (usaImu) {
      const EventoPostura ev = atualizarPickup(pickup, ultimoImu, c.agoraMs(), ai);
      if (ev == EventoPostura::FoiLevantado) levantados++;
      if (ev == EventoPostura::Reposicionado) {   // sempre de frente, em algum lugar do verde: rumo 0, pose desconhecida, mapa de cubos fica
        reposicionados++;
        rumo = 0.0f;
        memoria = Memoria{};
        cruz = MemoriaCruzamento{};
        ancora = Ancora{};
        tracao = Tracao{};
        esquecerCor(memCor);
        tUltimoCruzamento = 0;
        portaoPendente = true;
      }
      if (!motoresLiberados(pickup)) {   // carregado: motores parados ate estabilizar
        c.parar();
        if (c.roda(true) != 0.0 || c.roda(false) != 0.0) ticksMotorLevantado++;
        tick(c);
        return;
      }
      if (colisaoDetectada(colisao, ultimoImu, true, c.agoraMs(), ai)) colisoes++;
    }
    bool p[kSensores];
    c.ler(p);
    const unsigned long agora = c.agoraMs();
    const int corChao = c.corChao();
    anotarCor(memCor, corChao == kVermelha ? CorLinha::Vermelho : (corChao == kAzul ? CorLinha::Azul : CorLinha::Outra), agora);
    if (paraNoCruzamento && cruzamentoConfirmado(p, agora, ajC, cruz)) {   // acima de todas as outras regras: vale ate perdido/buscando
      pararNoCruzamento(c, p);
      return;
    }
    const unsigned long perdeuAntes = memoria.perdeuEm;
    const bool vinhaSeguindo = memoria.ultimoEsq != 0 || memoria.ultimoDir != 0;   // so segura o rumo se vinha andando: perdido sem nunca ter seguido, nao anda cego
    Comando cmd = decidir(p, agora, ajustes, memoria);
    const unsigned long perdidoHa = memoria.perdeuEm ? agora - memoria.perdeuEm : 0;
    if (memoria.perdeuEm) ticksPerdido++;
    if (estaBuscando(memoria, agora, ajustes)) ticksBuscando++;
    if (!fimDeBecoAvisado && tUltimoCruzamento &&
        classificarPerda(perdidoHa, agora - tUltimoCruzamento, ajustes.lacunaMs, ajC) == Perda::FimDeBeco) {
      fimDeBecoAvisado = true;
      fimDeBeco = true;
    }
    if (usaImu) {
      if (memoria.perdeuEm != 0 && perdeuAntes == 0) rumoAoPerder = rumo;
      if (!getenv("SIM_SEM_LACUNA") && vinhaSeguindo && memoria.perdeuEm != 0 && agora - memoria.perdeuEm < ajustes.lacunaMs) segurarRumoNaLacuna(cmd, rumo, rumoAoPerder, ajustes, ai);
      if (usaAncora) rumoSuspeito |= ancorarNaGrade(rumo, vies, ultimoImu.gzRadS, ehReto(cmd), agora, ancora, ai);
      const bool comandando = abs(cmd.esq) >= ajustes.zonaMortaPwm || abs(cmd.dir) >= ajustes.zonaMortaPwm;
      const AcaoTracao acao = atualizarTracao(tracao, ultimoImu, comandando && !cmd.parar, agora, ai);
      if (acao == AcaoTracao::Reforcar) reforcos++;
      if (acao == AcaoTracao::Recuar) {   // nao adiantou reforcar: recua um pouco e tenta de novo
        recuos++;
        c.comandar(-ajustes.velReto, -ajustes.velReto);
        for (int i = 0; i < 60; i++) tick(c);
        c.parar();
        tracao.reforco = 0;
        return;
      }
      if (!getenv("SIM_SEM_TRACAO")) aplicarReforco(cmd, tracao.reforco);
    }
    c.comandar(cmd.parar ? 0 : cmd.esq, cmd.parar ? 0 : cmd.dir);
    tick(c);
  }
};

// ---------------------------------------------------------------- cenarios

struct Resultado {
  std::string nome;
  bool ok;
  std::string info;
  bool informativo = false;   // limite conhecido do projeto: mostra, mas nao derruba o exit code
};

static std::string fmt(const char* f, ...) __attribute__((format(printf, 1, 2)));
static std::string fmt(const char* f, ...) {
  char b[512];
  va_list a;
  va_start(a, f);
  vsnprintf(b, sizeof b, f, a);
  va_end(a);
  return b;
}

static const char* gSvgDir = nullptr;

// Placar de qualidade do seguimento (reta, cantos, arcos): para comparar uma mudanca na decisao antes e depois.
static struct { int n = 0; double tempoS = 0, rms = 0, pior = 0; } gPlacar;
static void placar(const Carro& c) {
  gPlacar.n++;
  gPlacar.tempoS += c.agoraMs() / 1000.0;
  gPlacar.rms += c.erroRms();
  gPlacar.pior = std::max(gPlacar.pior, c.erroMax());
}

// Tudo o que cada cenario percorreu, para a pagina de visualizacao (--html=arquivo).
struct Viz { std::string nome; int achou; std::vector<Seg> segs; std::vector<Ret> decor; double largura; std::vector<Estado> rastro; std::vector<uint8_t> bits; };
static std::vector<Viz> gViz;
static const char* gHtmlPath = nullptr;

static void svg(const std::string& nome, const Pista& p, const Carro& c, int achou = -1) {
  if (gHtmlPath) gViz.push_back({nome, achou, p.segs, p.decor, p.largura, c.rastro(), c.bits()});
  if (!gSvgDir) return;
  double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
  auto acha = [&](double x, double y) { x0 = std::min(x0, x); y0 = std::min(y0, y); x1 = std::max(x1, x); y1 = std::max(y1, y); };
  for (const Seg& s : p.segs) { acha(s.x0, s.y0); acha(s.x1, s.y1); }
  for (const Estado& e : c.rastro()) acha(e.x, e.y);
  x0 -= 60; y0 -= 60; x1 += 60; y1 += 60;
  std::string path = std::string(gSvgDir) + "/" + nome + ".svg";
  FILE* f = fopen(path.c_str(), "w");
  if (!f) return;
  fprintf(f, "<svg xmlns='http://www.w3.org/2000/svg' viewBox='%.0f %.0f %.0f %.0f' width='%.0f' height='%.0f'><rect x='%.0f' y='%.0f' width='%.0f' height='%.0f' fill='white'/>\n",
          x0, -y1, x1 - x0, y1 - y0, (x1 - x0) * 0.8, (y1 - y0) * 0.8, x0, -y1, x1 - x0, y1 - y0);
  for (const Seg& s : p.segs)
    fprintf(f, "<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f' stroke='#222' stroke-width='%.0f' stroke-linecap='butt'/>\n", s.x0, -s.y0, s.x1, -s.y1, p.largura);
  fprintf(f, "<polyline fill='none' stroke='#e33' stroke-width='2' points='");
  for (const Estado& e : c.rastro()) fprintf(f, "%.1f,%.1f ", e.x, -e.y);
  fprintf(f, "'/></svg>\n");
  fclose(f);
}

// Roda o firmware ate `fim` ser verdadeiro. SEM limite de tempo por cenario: o unico teto e o de seguranca contra laco infinito (10 min simulados),
// que da "nao chegou" como qualquer outro fracasso.
constexpr unsigned long kTetoSegurancaMs = 600000;

template <class Fim>
static bool rodar(Firmware& fw, Carro& c, Fim fim) {
  const unsigned long inicio = c.agoraMs();
  while (c.agoraMs() - inicio < kTetoSegurancaMs) {
    fw.passo(c);
    if (fim(c)) return true;
  }
  return false;
}

// Deixa o tempo correr ate `ateMs` (instante absoluto da simulacao): para montar situacoes (esperar, levantar, bater), nao e limite de teste.
template <class Fim>
static void avancarAte(Firmware& fw, Carro& c, unsigned long ateMs, Fim fim) {
  while (c.agoraMs() < ateMs) {
    fw.passo(c);
    if (fim(c)) return;
  }
}

static Estado norte(double x, double y, double desvioGraus = 0) { return {x, y, M_PI / 2 + desvioGraus * M_PI / 180.0}; }

// 1. Reta: varia o desvio lateral e o angulo de partida; tem de chegar ao fim sem perder a linha.
static Resultado cenarioReta(const char* nome, double off, double ang, bool salva) {
  Pista p;
  p.reta(0, 0, 0, 1000);
  Carro c(p, norte(off, 100, ang));
  Firmware fw;
  const bool chegou = rodar(fw, c, [](const Carro& k) { return k.estado().y > 900; });
  placar(c);
  const bool ok = chegou && c.erroBarra() < 6.0 && fw.ticksBuscando == 0;
  if (salva) svg(nome, p, c);
  return {nome, ok, fmt("off %+.0f mm, ang %+.0f: %s em %lu ms, erro final %.1f mm, erro max %.1f mm, rms %.1f, buscando %d ciclos", off, ang, chegou ? "chegou" : "NAO chegou", c.agoraMs(),
                       c.erroBarra(), c.erroMax(), c.erroRms(), fw.ticksBuscando)};
}

// 2. Curva (canto vivo de 90 graus, ou arco): tem de seguir ate a perna de saida.
static Resultado cenarioCanto(bool paraEsquerda) {
  Pista p;
  const double s = paraEsquerda ? -1 : 1;
  p.reta(0, 0, 0, 400);
  p.reta(0, 400, s * 600, 400);
  Carro c(p, norte(0, 50));
  Firmware fw;
  fw.paraNoCruzamento = true;
  const bool chegou = rodar(fw, c, [&](const Carro& k) { return fabs(k.estado().x) > 450; });
  placar(c);
  const bool ok = chegou && c.erroBarra() < 6.0;
  svg(paraEsquerda ? "canto_esquerda" : "canto_direita", p, c);
  return {paraEsquerda ? "canto vivo 90 (esquerda)" : "canto vivo 90 (direita)", ok,
          fmt("%s em %lu ms, erro final %.1f mm, erro max %.1f mm, buscando %d ciclos, cruzamentos %d, falsos alarmes %d", chegou ? "chegou" : "NAO chegou", c.agoraMs(), c.erroBarra(),
              c.erroMax(), fw.ticksBuscando, fw.cruzamentos, fw.falsosAlarmes)};
}

static Resultado cenarioArco(double raio) {
  Pista p;
  p.reta(0, 0, 0, 200);
  p.arco(raio, 200, raio, 180, 90);   // sobe e vira a direita
  p.reta(raio, 200 + raio, raio + 400, 200 + raio);
  Carro c(p, norte(0, 50));
  Firmware fw;
  const bool chegou = rodar(fw, c, [&](const Carro& k) { return k.estado().x > raio + 300; });
  placar(c);
  const bool ok = chegou && c.erroBarra() < 6.0;
  svg(fmt("arco_%.0f", raio), p, c);
  return {fmt("arco raio %.0f mm", raio), ok, fmt("%s em %lu ms, erro final %.1f mm, erro max %.1f mm, buscando %d ciclos", chegou ? "chegou" : "NAO chegou", c.agoraMs(), c.erroBarra(), c.erroMax(), fw.ticksBuscando)};
}

// 3. Cruzamento em + (rua que cruza a 500 mm) seguido de beco ate 500+127: para, gira, confirma, atravessa, e acusa o fim do beco.
static Resultado cenarioCruzamento(double off, double ang) {
  Pista p;
  p.reta(0, 0, 0, 627);          // vertical: beco de 127 mm depois da rua
  p.reta(-400, 500, 400, 500);   // rua que cruza
  Carro c(p, norte(off, 100, ang));
  Firmware fw;
  rodar(fw, c, [&](const Carro&) { return fw.fimDeBeco; });
  const bool ok = fw.cruzamentos == 1 && fw.falsosAlarmes == 0 && fw.fimDeBeco && fw.ruaEsq == 1 && fw.ruaDir == 1;
  svg(fmt("cruzamento_off%+.0f_ang%+.0f", off, ang), p, c);
  return {fmt("cruzamento (off %+.0f, ang %+.0f)", off, ang), ok,
          fmt("confirmados %d, falsos %d, fim de beco %s | ruas esq %d dir %d (com o angulo verdadeiro: %d %d) | giro deixou o rumo %+.1f graus fora | eixo parou %.0f mm antes do centro da rua",
              fw.cruzamentos, fw.falsosAlarmes, fw.fimDeBeco ? "SIM" : "nao", fw.ruaEsq, fw.ruaDir, fw.ruaEsqVerdade, fw.ruaDirVerdade, fw.giroDriftGraus, 500.0 - fw.paradaEm.y)};
}

// 3a. Beco que termina na borda (linha vermelha ou azul): le a cor, da meia-volta e volta; com cubo, para e deixa para o navegador.
static Resultado cenarioBorda(int cor, bool cubo) {
  Pista p;
  p.reta(0, 0, 0, 627);               // beco de 127 mm depois da rua
  p.reta(-400, 500, 400, 500);        // ultima rua
  p.reta(-200, 627, 200, 627, cor);   // borda
  Carro c(p, norte(0, 100));
  Firmware fw;
  fw.temCubo = cubo;
  bool voltou = false;
  rodar(fw, c, [&](const Carro& k) {
    if (cubo) return fw.parado;
    voltou = k.estado().y < 300 && fw.meiasVoltas > 0;
    return voltou;
  });
  const bool ok = cubo ? (fw.bordas == 1 && fw.paradasNavegador == 1 && fw.meiasVoltas == 0)
                       : (voltou && fw.bordas == 1 && fw.meiasVoltas == 1 && fw.cruzamentos == 2 && fw.falsosAlarmes == 0 && c.erroBarra() < 8.0);
  const char* nome = cor == kVermelha ? "vermelha" : "azul";
  svg(fmt("borda_%s%s", nome, cubo ? "_cubo" : ""), p, c);
  return {fmt("borda %s%s", nome, cubo ? " com cubo" : ""), ok,
          fmt("bordas %d, meias-voltas %d, parou p/ navegador %d, cruzamentos %d, falsos %d, %s, y final %.0f, erro %.1f mm", fw.bordas, fw.meiasVoltas, fw.paradasNavegador,
              fw.cruzamentos, fw.falsosAlarmes, voltou ? "voltou ao mapa" : "NAO voltou", c.estado().y, c.erroBarra())};
}

// ---- MPU: giro por giroscopio, ancora na grade, levantado, travado, colisao, calibracao, lacuna segurando o rumo

static double rumoVerdadeGraus(const Carro& c, double base) { return remainder((c.estado().th - base) * 180.0 / M_PI, 360.0); }

// Cruzamento com o angulo do giro vindo do giroscopio (e nao de contar tempo): o carro tem de voltar ao rumo em que chegou.
static Resultado cenarioCruzamentoImu() {
  Pista p;
  p.reta(0, 0, 0, 627);
  p.reta(-400, 500, 400, 500);
  Carro c(p, norte(0, 100));
  Firmware fw;
  fw.ligarImu(c);
  rodar(fw, c, [&](const Carro&) { return fw.fimDeBeco; });
  const bool ok = fw.cruzamentos == 1 && fw.falsosAlarmes == 0 && fabs(fw.giroDriftGraus) < 2.0;
  svg("cruzamento_giroscopio", p, c);
  return {"cruzamento com giro pelo giroscopio", ok, fmt("confirmados %d, falsos %d, ruas %d %d | o giro deixou o rumo %+.1f graus fora (por tempo, sem giroscopio, passava de 4)", fw.cruzamentos, fw.falsosAlarmes, fw.ruaEsq, fw.ruaDir, fw.giroDriftGraus)};
}

// Reta longa com o vies do giroscopio andando 3 graus/s depois da calibracao (temperatura): sem ancora o rumo estimado foge; com ela, fica na grade.
static Resultado cenarioAncora() {
  Pista p;
  p.reta(0, 0, 0, 2400);
  ParamsFisicos f = gFis;
  f.derivaViesGrauS = 3.0;
  double erro[2] = {0, 0};
  for (int com = 0; com < 2; com++) {
    Carro c(p, norte(0, 100), f);
    Firmware fw;
    fw.usaAncora = com == 1;
    fw.ligarImu(c);
    rodar(fw, c, [](const Carro& k) { return k.estado().y > 2200; });
    erro[com] = fabs(fw.rumo - rumoVerdadeGraus(c, M_PI / 2));   // rumo estimado contra o real
    if (com == 1) svg("ancora_grade", p, c);
  }
  return {"rumo ancorado na grade (vies derivando 3 graus/s)", erro[1] < 6.0 && erro[0] > 5.0 * erro[1], fmt("erro do rumo estimado no fim da reta: sem ancora %.1f graus, com ancora %.1f graus", erro[0], erro[1])};
}

// Levantado no meio do caminho: para os motores, espera estabilizar, e volta de frente (rumo 0) e anda.
static Resultado cenarioLevantado() {
  Pista p;
  p.reta(0, 0, 0, 1400);
  Carro c(p, norte(15, 100, 10));
  Firmware fw;
  fw.ligarImu(c);
  avancarAte(fw, c, 1500, [](const Carro&) { return false; });
  c.levantar(true);
  avancarAte(fw, c, 3500, [](const Carro&) { return false; });   // 2 s no ar
  const int motoresNoAr = fw.ticksMotorLevantado;
  c.colocarEm(norte(-12, 700, -10));                         // solto em outro lugar, apontando para a frente
  c.levantar(false);
  const unsigned long solto = c.agoraMs();
  bool andouAntes = false;
  avancarAte(fw, c, solto + 1300, [&](const Carro& k) { andouAntes |= k.estado().y > 700.5; return false; });   // ainda estabilizando: nao anda
  const bool chegou = rodar(fw, c, [](const Carro& k) { return k.estado().y > 1300; });
  const bool ok = fw.levantados == 1 && fw.reposicionados == 1 && motoresNoAr == 0 && !andouAntes && chegou;
  svg("levantado", p, c);
  return {"levantado, solto de frente em outro lugar", ok, fmt("levantou %d, reposicionou %d, ciclos com motor ligado no ar %d, andou antes de estabilizar: %s, %s", fw.levantados, fw.reposicionados, motoresNoAr, andouAntes ? "SIM" : "nao", chegou ? "chegou ao fim" : "NAO chegou")};
}

// Zona morta real 95 e a decisao achando 65: as correcoes (90) nao giram. Sem o MPU o robo sai da linha; com ele, o reforco sobe ate girar.
static Resultado cenarioTravado() {
  Pista p;
  p.reta(0, 0, 0, 1000);
  ParamsFisicos f = gFis;
  f.zonaMortaPwm = 95;
  bool chegou[2] = {false, false};
  int reforcos = 0;
  for (int com = 0; com < 2; com++) {
    Carro c(p, norte(14, 100, 8), f);
    Firmware fw;
    fw.ajustes.zonaMortaPwm = 65;   // a decisao nao sabe da zona real
    fw.ajustes = normalizar(fw.ajustes);
    if (com) fw.ligarImu(c);
    chegou[com] = rodar(fw, c, [](const Carro& k) { return k.estado().y > 900; }) && c.erroBarra() < 8.0;
    if (com) { reforcos = fw.reforcos; svg("travado_reforco", p, c); }
  }
  return {"andando menos que o esperado: reforca o PWM", chegou[1], fmt("zona real 95, decisao 65: sem MPU %s; com MPU %s (%d reforcos)", chegou[0] ? "chegou" : "NAO chegou", chegou[1] ? "chegou" : "NAO chegou", reforcos)};
}

// Pancada forte (oponente) e leve (cubo): so a forte conta.
static Resultado cenarioColisao() {
  Pista p;
  p.reta(0, 0, 0, 1000);
  Carro c(p, norte(0, 100));
  Firmware fw;
  fw.ligarImu(c);
  avancarAte(fw, c, 1000, [](const Carro&) { return false; });
  c.bater(2.5, 4);
  avancarAte(fw, c, 2000, [](const Carro&) { return false; });
  c.bater(0.4, 4);
  avancarAte(fw, c, 3000, [](const Carro&) { return false; });
  return {"colisao: oponente conta, cubo nao", fw.colisoes == 1, fmt("colisoes detectadas %d (pancada forte 2,5 g e leve 0,4 g); pico visto %.2f g", fw.colisoes, fw.colisao.picoG)};
}

// Calibracao: o robo gira no eixo e acha a zona morta e os graus por segundo.
static Resultado cenarioCalibracao(double zonaReal) {
  Pista vazia;
  ParamsFisicos f = gFis;
  f.zonaMortaPwm = zonaReal;
  Carro c(vazia, norte(0, 0), f);
  Firmware fw;
  fw.ligarImu(c);
  Calibracao cal;
  const AjustesCalibracao aj;
  bool fim = false;
  for (int i = 0; i < 6000 && !fim; i++) {
    const ComandoCalibracao k = passoCalibracao(cal, fw.ultimoImu.gzRadS - fw.vies, c.agoraMs(), aj);
    fim = k.terminou;
    c.comandar(k.esq, k.dir);
    fw.tick(c);
  }
  // giro verdadeiro em regime a velMedida: duas rodas opostas
  const double verdade = 2.0 * (cal.velMedida * f.mmSPorPwm) / f.bitolaMm * f.derrapagem * 57.29578;
  const bool ok = fim && cal.fase == Calibracao::Pronto && fabs(cal.zonaMortaPwm - zonaReal) <= 6 && fabs(cal.grausPorSegundo - verdade) < 0.1 * verdade;
  return {fmt("calibracao de motores (zona real %.0f)", zonaReal), ok, fmt("achou zona %d (real %.0f), %.0f graus/s a PWM %d (real %.0f; o chute era %.0f)", cal.zonaMortaPwm, zonaReal, cal.grausPorSegundo, cal.velMedida, verdade, Ajustes{}.grausPorSegundoBusca)};
}

// Lacuna segurando o rumo pelo giroscopio: a lacuna pode ser maior do que o ultimo comando levava.
static Resultado cenarioLacunaImu(double lacunaMm) {
  Pista p;
  p.reta(0, 0, 0, 300);
  p.reta(0, 300 + lacunaMm, 0, 1000);
  Carro c(p, norte(0, 100, 6));
  Firmware fw;
  fw.ligarImu(c);
  const bool chegou = rodar(fw, c, [](const Carro& k) { return k.estado().y > 900; });
  svg(fmt("lacuna_giroscopio_%.0f", lacunaMm), p, c);
  return {fmt("lacuna de %.0f mm segurando o rumo", lacunaMm), chegou && c.erroBarra() < 6.0, fmt("%s em %lu ms, erro final %.1f mm, buscando %d ciclos", chegou ? "chegou" : "NAO chegou", c.agoraMs(), c.erroBarra(), fw.ticksBuscando)};
}

// ---- O tabuleiro inteiro (docs/2026-10-28_mapa-comparacao-e-localizacao.md): 6 colunas x 5 linhas de cruzamentos a 302 mm, becos de 127 mm ate a borda,
// linha azul nos lados (x = 0 e 1764), vermelha em cima e embaixo (y = 0 e 1462) e em volta, verde (280 mm) e amarelo (220 mm) nas pontas, cubos nos quarteiroes.
constexpr double kPasso = 302.0, kBeco = 127.0, kLarg = 2 * kBeco + 5 * kPasso, kAlt = 2 * kBeco + 4 * kPasso;   // 1764 x 1462

static Pista tabuleiro() {
  Pista p;
  p.decor.push_back({-500, 0, -280, kAlt, 1});             // amarelo (ecoponto)
  p.decor.push_back({-280, 0, 0, kAlt, 0});                // verde
  p.decor.push_back({kLarg, 0, kLarg + 280, kAlt, 0});
  p.decor.push_back({kLarg + 280, 0, kLarg + 500, kAlt, 1});
  for (int c = 0; c < 5; c++)
    for (int r = 0; r < 4; r++) {
      const double cx = kBeco + c * kPasso + kPasso / 2, cy = kBeco + r * kPasso + kPasso / 2;
      p.decor.push_back({cx - 15, cy - 15, cx + 15, cy + 15, 2});   // marrom: ponto de cubo
    }
  for (int r = 0; r < 5; r++) p.reta(0, kBeco + r * kPasso, kLarg, kBeco + r * kPasso);          // ruas horizontais: de azul a azul
  for (int c = 0; c < 6; c++) p.reta(kBeco + c * kPasso, 0, kBeco + c * kPasso, kAlt);          // ruas verticais: de vermelho a vermelho
  p.reta(0, 0, 0, kAlt, kAzul);
  p.reta(kLarg, 0, kLarg, kAlt, kAzul);
  p.reta(-500, 0, kLarg + 500, 0, kVermelha);
  p.reta(-500, kAlt, kLarg + 500, kAlt, kVermelha);
  p.reta(-500, 0, -500, kAlt, kVermelha);
  p.reta(kLarg + 500, 0, kLarg + 500, kAlt, kVermelha);
  return p;
}

// Percurso: sai do verde, passa o portao azul, atravessa o cruzamento da coluna 0, vira para o norte na coluna 1, sobe ate a borda vermelha, da meia-volta e desce.
static Resultado cenarioTabuleiro() {
  Pista p = tabuleiro();
  const double yRua = kBeco + 2 * kPasso;
  Carro c(p, {-140, yRua, 0.0});   // no verde, de frente para a cidade
  Firmware fw;
  fw.ligarImu(c);
  fw.portaoPendente = true;
  fw.rota = "SLSSSS";
  const bool desceu = rodar(fw, c, [&](const Carro& k) { return fw.meiasVoltas > 0 && k.estado().y < kBeco + 2 * kPasso + 50; });
  const bool ok = desceu && fw.portoes == 1 && fw.meiasVoltas == 1 && fw.bordas == 1 && fw.falsosAlarmes == 0 && c.erroBarra() < 8.0;
  svg("tabuleiro_inteiro", p, c);
  return {"tabuleiro inteiro: portao, cruzamentos, curva para o norte, borda", ok,
          fmt("%s em %.1f s | portao %d, cruzamentos confirmados %d, falsos %d, bordas %d (meia-volta %d) | parou em (%.0f, %.0f) mm, erro %.1f mm", desceu ? "completou o percurso" : "NAO completou", c.agoraMs() / 1000.0,
              fw.portoes, fw.cruzamentos, fw.falsosAlarmes, fw.bordas, fw.meiasVoltas, c.estado().x, c.estado().y, c.erroBarra())};
}

// Perdido e de lado em cima da linha (barra paralela a ela: os 8 sensores acesos): vira 90 graus, ve que ficou estreito e centrado, e segue por ela.
// Tambem vale quando o MPU nao esta ligado (angulo por tempo).
static Resultado cenarioEndireitar(bool imu) {
  Pista p;
  p.reta(0, 0, 0, 1200);
  Carro c(p, {-150, 300, 0.0});   // longe da linha, perdido
  Firmware fw;
  if (imu) fw.ligarImu(c);
  avancarAte(fw, c, c.agoraMs() + 700, [](const Carro&) { return false; });   // procura um pouco: perdeuEm e a lacuna ja passaram
  c.colocarEm({-30, 500, 0.0});   // a busca o deixou de lado em cima da linha: a barra inteira sobre ela
  const bool chegou = rodar(fw, c, [](const Carro& k) { return k.estado().y > 1000; });
  svg(imu ? "endireitar_na_linha" : "endireitar_na_linha_sem_mpu", p, c);
  return {imu ? "perdido de lado na linha: vira 90 e se endireita (com MPU)" : "perdido de lado na linha: vira 90 e se endireita (sem MPU)", chegou && fw.endireitou >= 1 && fw.cruzamentos == 0,
          fmt("%s | endireitou %d vez(es), cruzamentos %d, falsos alarmes %d | fim em (%.0f, %.0f), erro %.1f mm", chegou ? "chegou ao fim da linha" : "NAO chegou", fw.endireitou, fw.cruzamentos, fw.falsosAlarmes, c.estado().x, c.estado().y, c.erroBarra())};
}

// 3b. Depois do cruzamento o carro tem de seguir a linha do outro lado (rua e beco longos).
static Resultado cenarioAtravessa() {
  Pista p;
  p.reta(0, 0, 0, 1000);
  p.reta(-400, 500, 400, 500);
  Carro c(p, norte(0, 100));
  Firmware fw;
  const bool chegou = rodar(fw, c, [](const Carro& k) { return k.estado().y > 900; });
  const bool ok = chegou && fw.cruzamentos == 1 && c.erroBarra() < 6.0;
  svg("atravessa_cruzamento", p, c);
  return {"atravessa o cruzamento e segue", ok, fmt("%s em %lu ms, cruzamentos %d, falsos %d, erro final %.1f mm, giro deixou rumo %+.1f graus, buscando %d ciclos", chegou ? "chegou" : "NAO chegou",
                                                    c.agoraMs(), fw.cruzamentos, fw.falsosAlarmes, c.erroBarra(), fw.giroDriftGraus, fw.ticksBuscando)};
}

// 4. Lacuna na fita: nao pode virar busca.
static Resultado cenarioLacuna(double lacunaMm) {
  Pista p;
  p.reta(0, 0, 0, 300);
  p.reta(0, 300 + lacunaMm, 0, 1000);
  Carro c(p, norte(0, 100));
  Firmware fw;
  const bool chegou = rodar(fw, c, [](const Carro& k) { return k.estado().y > 900; });
  const bool ok = chegou && c.erroBarra() < 6.0;
  svg(fmt("lacuna_%.0f", lacunaMm), p, c);
  return {fmt("lacuna de %.0f mm", lacunaMm), ok, fmt("%s em %lu ms, erro final %.1f mm, buscando %d ciclos", chegou ? "chegou" : "NAO chegou", c.agoraMs(), c.erroBarra(), fw.ticksBuscando)};
}

// 5. Busca: carro solto fora da linha (distancia lateral e rumo variados). Informativo + criterio: achar a linha em ate 15 s.
static bool buscar(double dist, double rumoGraus, bool comMpu, unsigned long& ms, int& ciclos, const Pista& p, Carro** saida = nullptr) {
  // `dist` = distancia do CENTRO DA BARRA (nao do eixo) ate a linha: assim a barra nunca comeca em cima dela, qualquer que seja o rumo
  const double th = rumoGraus * M_PI / 180.0;
  const Estado origem = {dist - 30.0 * cos(th), 500 - 30.0 * sin(th), th};
  static Carro* ultimo = nullptr;
  delete ultimo;
  Carro* c = ultimo = new Carro(p, origem);
  Firmware fw;
  if (comMpu) {   // com o MPU o robo mede a zona morta e os graus por segundo ANTES de sair procurando (e depois volta onde estava)
    fw.ligarImu(*c);
    Calibracao cal;
    AjustesCalibracao ac;
    bool fim = false;
    for (int i = 0; i < 6000 && !fim; i++) {
      const ComandoCalibracao k = passoCalibracao(cal, fw.ultimoImu.gzRadS - fw.vies, c->agoraMs(), ac);
      fim = k.terminou;
      c->comandar(k.esq, k.dir);
      fw.tick(*c);
    }
    if (getenv("SIM_DEBUG")) fprintf(stderr, "  calibracao busca: fase %d zona %d vel %d graus/s %.1f\n", static_cast<int>(cal.fase), cal.zonaMortaPwm, cal.velMedida, cal.grausPorSegundo);
    if (cal.fase == Calibracao::Pronto) {
      fw.ajustes.zonaMortaPwm = cal.zonaMortaPwm;
      fw.ajustes.velBuscaExterna = cal.velMedida;
      fw.ajustes.grausPorSegundoBusca = cal.grausPorSegundo;
      fw.ajustes = normalizar(fw.ajustes);
    }
    c->colocarEm(origem);
    c->limparRastro();   // a animacao mostra so a busca, nao a calibracao
    if (getenv("SEM_IMU_NA_BUSCA")) fw.usaImu = false;
  }
  const unsigned long t0 = c->agoraMs();
  const bool achou = rodar(fw, *c, [&](const Carro& k) {
    return k.erroBarra() < 8.0 && fw.memoria.perdeuEm == 0;   // de novo em cima da linha (ao largar, a barra esta a >= 30 mm dela)
  });
  ms = c->agoraMs() - t0;
  ciclos = fw.ticksBuscando;
  if (achou) avancarAte(fw, *c, c->agoraMs() + 8000, [](const Carro&) { return false; });   // continua andando na linha depois de achar: e o que a animacao mostra
  if (getenv("SIM_DEBUG") && comMpu) fprintf(stderr, "  busca d%.0f r%.0f com MPU: achou=%d recuos %d reforcos %d levantados %d colisoes %d fim (%.0f,%.0f)\n", dist, rumoGraus, achou, fw.recuos, fw.reforcos, fw.levantados, fw.colisoes, c->estado().x, c->estado().y);
  if (getenv("SIM_DEBUG2") && comMpu && dist == 30 && rumoGraus == 90) { for (size_t i = c->rastro().size() > 800 ? c->rastro().size() - 800 : 0; i < c->rastro().size() && i < 800 + 100000; i += 20) fprintf(stderr, "   (%.0f,%.0f,%.0f|%02x)", c->rastro()[i].x, c->rastro()[i].y, c->rastro()[i].th * 57.29578, c->bits()[i]); fprintf(stderr, "\n   rumo[0..]\n"); }
  if (saida) *saida = c;
  return achou;
}

static Resultado cenarioBusca(double dist, double rumoGraus) {
  Pista p;
  p.reta(0, -400, 0, 1400);
  unsigned long msSem = 0, msCom = 0;
  int cSem = 0, cCom = 0;
  const bool sem = buscar(dist, rumoGraus, false, msSem, cSem, p);
  Carro* c = nullptr;
  const bool com = buscar(dist, rumoGraus, true, msCom, cCom, p, &c);
  svg(fmt("busca_d%.0f_r%.0f %s", dist, rumoGraus, com ? fmt("OK em %.1f s", msCom / 1000.0).c_str() : "NAO ACHOU"), p, *c, com ? 1 : 0);
  return {fmt("busca a %.0f mm, rumo %.0f", dist, rumoGraus), com,
          fmt("com MPU (calibra o giro antes): %s em %lu ms | sem MPU (chute de 110 graus/s): %s", com ? "achou" : "NAO achou", msCom, sem ? fmt("achou em %lu ms", msSem).c_str() : "NAO achou")};
}

// Pagina unica (sem dependencias) que anima cada cenario: pista, carro, barra de 8 sensores acendendo e rastro.
static void escreverHtml(const char* caminho) {
  std::stable_sort(gViz.begin(), gViz.end(), [](const Viz& a, const Viz& b) { return (a.nome == "tabuleiro_inteiro") > (b.nome == "tabuleiro_inteiro"); });   // o mapa completo vem primeiro
  FILE* f = fopen(caminho, "w");
  if (!f) return;
  fputs(R"HTML(<!doctype html><html lang="pt-br"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Simulador do seguidor</title>
<style>:root{--bg:#f6f6f4;--fg:#1b1b1b;--card:#fff;--bd:#d8d8d4;--ac:#2a6df4}@media(prefers-color-scheme:dark){:root{--bg:#161618;--fg:#ececec;--card:#202024;--bd:#34343a;--ac:#6ea0ff}}
body{margin:0;background:var(--bg);color:var(--fg);font:14px system-ui,sans-serif}main{max-width:1000px;margin:0 auto;padding:16px}
.bar{display:flex;flex-wrap:wrap;gap:10px;align-items:center;margin-bottom:10px}select,button,input{font:inherit;color:var(--fg);background:var(--card);border:1px solid var(--bd);border-radius:6px;padding:6px 10px}
button{cursor:pointer}canvas{width:100%;background:var(--card);border:1px solid var(--bd);border-radius:8px;display:block}
#sens{display:flex;gap:6px;align-items:center;margin:10px 0}#sens span{width:26px;height:26px;border-radius:50%;border:2px solid var(--bd);display:inline-flex;align-items:center;justify-content:center;font-size:11px}#sens span.on{background:#111;color:#fff;border-color:#111}
@media(prefers-color-scheme:dark){#sens span.on{background:#fff;color:#111;border-color:#fff}}small{opacity:.7}</style></head><body><main>
<div class="bar"><select id="sel"></select><button id="b1" title="1 s para tras">&#9664;&#9664; 1 s</button><button id="b01" title="0,1 s para tras">&#9664; 0,1 s</button><button id="pp">Pausar</button><button id="f01" title="0,1 s para frente">0,1 s &#9654;</button><button id="f1" title="1 s para frente">1 s &#9654;&#9654;</button><button id="r0" title="voltar ao inicio">&#8634; inicio</button><label>velocidade <select id="vel"><option value="0.25">0,25x</option><option value="0.5">0,5x</option><option value="1" selected>1x</option><option value="2">2x</option><option value="4">4x</option></select></label><span id="tt"></span></div>
<canvas id="c" width="1000" height="620"></canvas>
<div id="sens"></div><small>Sensores: S1 (esquerda) a S8 (direita), aceso = preto. Linha 20 mm, barra de 75 mm a 30 mm do eixo, carro 170 x 200 mm. Vermelho/azul = borda do mapa. Traco vermelho = caminho do eixo. Teclas: setas esquerda/direita (0,1 s), shift + seta (1 s), espaco (pausa).</small>
</main><script>
const D=)HTML",f);
  fputs("[", f);
  for (size_t v = 0; v < gViz.size(); v++) {
    const Viz& z = gViz[v];
    fprintf(f, "%s{n:\"%s\",a:%d,w:%.0f,s:[", v ? "," : "", z.nome.c_str(), z.achou, z.largura);
    for (size_t i = 0; i < z.segs.size(); i++) fprintf(f, "%s[%.1f,%.1f,%.1f,%.1f,%d]", i ? "," : "", z.segs[i].x0, z.segs[i].y0, z.segs[i].x1, z.segs[i].y1, z.segs[i].cor);
    fputs("],r:[", f);
    for (size_t i = 0; i < z.decor.size(); i++) fprintf(f, "%s[%.0f,%.0f,%.0f,%.0f,%d]", i ? "," : "", z.decor[i].x0, z.decor[i].y0, z.decor[i].x1, z.decor[i].y1, z.decor[i].cor);
    fputs("],f:[", f);
    for (size_t i = 0; i < z.rastro.size(); i++) fprintf(f, "%s[%.1f,%.1f,%.2f,%d]", i ? "," : "", z.rastro[i].x, z.rastro[i].y, z.rastro[i].th * 180.0 / M_PI, z.bits[i]);
    fputs("]}", f);
  }
  fputs("];\n", f);
  fputs(R"HTML(const cv=document.getElementById('c'),cx=cv.getContext('2d'),sel=document.getElementById('sel'),sens=document.getElementById('sens');
const cores=['#111','#e02424','#1c64f2'],zonas=['#2e8b3d','#f2d024','#8b4a1f'];let cur=0,fr=0,play=true,last=0,vel=1;
D.forEach((d,i)=>{const o=document.createElement('option');o.value=i;o.textContent=d.n;sel.appendChild(o)});
for(let i=0;i<8;i++){const s=document.createElement('span');s.textContent='S'+(i+1);sens.appendChild(s)}
function dark(){return matchMedia('(prefers-color-scheme: dark)').matches}
function tela(){const d=D[cur];let x0=1e9,y0=1e9,x1=-1e9,y1=-1e9;const a=(x,y)=>{x0=Math.min(x0,x);x1=Math.max(x1,x);y0=Math.min(y0,y);y1=Math.max(y1,y)};d.s.forEach(s=>{a(s[0],s[1]);a(s[2],s[3])});d.r.forEach(r=>{a(r[0],r[1]);a(r[2],r[3])});d.f.forEach(f=>a(f[0],f[1]));x0-=120;x1+=120;y0-=120;y1+=120;
const k=Math.min(cv.width/(x1-x0),cv.height/(y1-y0));return{k,ox:(cv.width-(x1-x0)*k)/2-x0*k,oy:(cv.height-(y1-y0)*k)/2+y1*k}}
function P(t,x,y){return[t.ox+x*t.k,t.oy-y*t.k]}
function desenha(){const d=D[cur],t=tela();cx.clearRect(0,0,cv.width,cv.height);cx.lineCap='butt';
d.r.forEach(r=>{const a=P(t,r[0],r[3]),b=P(t,r[2],r[1]);cx.fillStyle=zonas[r[4]];cx.fillRect(a[0],a[1],b[0]-a[0],b[1]-a[1])});
d.s.forEach(s=>{cx.strokeStyle=dark()&&s[4]==0?'#cfcfcf':cores[s[4]];cx.lineWidth=d.w*t.k;const a=P(t,s[0],s[1]),b=P(t,s[2],s[3]);cx.beginPath();cx.moveTo(a[0],a[1]);cx.lineTo(b[0],b[1]);cx.stroke()});
cx.strokeStyle='#e02424';cx.lineWidth=1.5;cx.beginPath();for(let i=0;i<=fr;i++){const p=P(t,d.f[i][0],d.f[i][1]);i?cx.lineTo(p[0],p[1]):cx.moveTo(p[0],p[1])}cx.stroke();
const f=d.f[fr],th=f[2]*Math.PI/180,c=Math.cos(th),s=Math.sin(th);const W=(fx,fy)=>P(t,f[0]+fx*c-fy*s,f[1]+fx*s+fy*c);
cx.fillStyle=dark()?'rgba(110,160,255,.35)':'rgba(42,109,244,.25)';cx.strokeStyle='#2a6df4';cx.lineWidth=2;cx.beginPath();[[-100,-85],[100,-85],[100,85],[-100,85]].forEach((q,i)=>{const p=W(q[0],q[1]);i?cx.lineTo(p[0],p[1]):cx.moveTo(p[0],p[1])});cx.closePath();cx.fill();cx.stroke();
const fr0=W(100,0),to=W(60,0);cx.beginPath();cx.moveTo(to[0],to[1]);cx.lineTo(fr0[0],fr0[1]);cx.stroke();
const e=W(0,0);cx.fillStyle='#2a6df4';cx.beginPath();cx.arc(e[0],e[1],4,0,7);cx.fill();
for(let i=0;i<8;i++){const lat=37.5-i*75/7,p=W(30,lat),on=(f[3]>>i)&1;cx.fillStyle=on?'#ffb000':(dark()?'#555':'#ddd');cx.strokeStyle=on?'#a06000':'#888';cx.lineWidth=1;cx.beginPath();cx.arc(p[0],p[1],Math.max(3,5*t.k*2),0,7);cx.fill();cx.stroke()}
[...sens.children].forEach((s,i)=>s.classList.toggle('on',((f[3]>>i)&1)==1));document.getElementById('tt').textContent='t = '+(fr*0.02).toFixed(2)+' s'}
function carrega(i){cur=i;fr=0;play=true;document.getElementById('pp').textContent='Pausar';desenha()}
function pausa(){play=false;document.getElementById('pp').textContent='Continuar'}
function passo(n){pausa();fr=Math.max(0,Math.min(D[cur].f.length-1,fr+n));desenha()}
sel.onchange=()=>carrega(+sel.value);
b1.onclick=()=>passo(-50);b01.onclick=()=>passo(-5);f01.onclick=()=>passo(5);f1.onclick=()=>passo(50);r0.onclick=()=>{fr=0;play=true;document.getElementById('pp').textContent='Pausar';desenha()};
addEventListener('keydown',e=>{if(e.key==='ArrowRight'){passo(e.shiftKey?50:5)}else if(e.key==='ArrowLeft'){passo(e.shiftKey?-50:-5)}else if(e.key===' '){e.preventDefault();document.getElementById('pp').click()}});
document.getElementById('pp').onclick=()=>{play=!play;document.getElementById('pp').textContent=play?'Pausar':'Continuar';if(play&&fr>=D[cur].f.length-1)fr=0};
document.getElementById('vel').onchange=e=>vel=+e.target.value;
function loop(ts){if(play&&ts-last>=20/vel){last=ts;if(fr<D[cur].f.length-1){fr++}}desenha();requestAnimationFrame(loop)}
carrega(0);requestAnimationFrame(loop);
</script></body></html>)HTML",f);
  fclose(f);
}

int main(int argc, char** argv) {
  // Argumentos: uma pasta (grava um SVG por cenario) e/ou chave=valor para os parametros fisicos: tau, derrap, bitola, zona, ganho, barra
  for (int i = 1; i < argc; i++) {
    const char* eq = strchr(argv[i], '=');
    if (!eq) { if (!strcmp(argv[i], "-q")) gSo_Placar = true; else gSvgDir = argv[i]; continue; }
    const std::string k(argv[i], eq - argv[i]);
    if (k == "--html") { gHtmlPath = eq + 1; continue; }
    const double v = atof(eq + 1);
    if (k == "tau") gFis.tauMs = v;
    else if (k == "derrap") gFis.derrapagem = v;
    else if (k == "bitola") gFis.bitolaMm = v;
    else if (k == "zona") { gFis.zonaMortaPwm = v; gAjustes.zonaMortaPwm = static_cast<int>(v); }   // zona CALIBRADA: a decisao sabe
    else if (k == "zonaaj") gAjustes.zonaMortaPwm = static_cast<int>(v);   // so a decisao (calibracao errada)
    else if (k == "ganho") gFis.mmSPorPwm = v;
    else if (k == "barra") gFis.barraAFrenteMm = v;
    else if (k == "periodo") gAjustes.periodoCorrecaoMs = static_cast<unsigned long>(v);
    else if (k == "fl") gAjustes.fracaoGiroLeve = static_cast<float>(v);
    else if (k == "ff") gAjustes.fracaoGiroForte = static_cast<float>(v);
    else if (k == "vl") gAjustes.velCorrecaoLeve = static_cast<int>(v);
    else if (k == "vf") gAjustes.velCorrecaoForte = static_cast<int>(v);
    else if (k == "posreto") gAjustes.posReto = static_cast<float>(v);
    else if (k == "posleve") gAjustes.posLeve = static_cast<float>(v);
    else if (k == "posforte") gAjustes.posForte = static_cast<float>(v);
    else if (k == "fatoravanco") gAjustes.fatorAvancoCorrecao = static_cast<float>(v);
    else if (k == "curvafechada") gAjustes.curvaFechadaSensores = static_cast<int>(v);
    else if (k == "avancobusca") gAjustes.buscaAvancoMs = static_cast<unsigned long>(v);
    else if (k == "espiral") gAjustes.buscaEspiralMs = static_cast<unsigned long>(v);
    else if (k == "grausbusca") gAjustes.grausPorSegundoBusca = static_cast<float>(v);
    else if (k == "maxfator") gAjustes.buscaAvancoMaxFator = static_cast<int>(v);
    else if (k == "lacuna") gAjustes.lacunaMs = static_cast<unsigned long>(v);
    else if (k == "velbusca") gAjustes.velBuscaExterna = static_cast<int>(v);
    else if (k == "velreto") gAjustes.velReto = static_cast<int>(v);
    else { fprintf(stderr, "parametro desconhecido: %s\n", k.c_str()); return 2; }
  }
  std::vector<Resultado> rs;

  rs.push_back(cenarioReta("reta", 0, 0, true));
  for (double off : {-25.0, -12.0, 12.0, 25.0})
    for (double ang : {-20.0, 0.0, 20.0}) rs.push_back(cenarioReta(fmt("reta off%+.0f ang%+.0f", off, ang).c_str(), off, ang, false));
  rs.push_back(cenarioCanto(true));
  rs.push_back(cenarioCanto(false));
  for (double r : {250.0, 150.0, 100.0}) rs.push_back(cenarioArco(r));
  for (double l : {30.0, 60.0, 100.0}) {
    rs.push_back(cenarioLacuna(l));
    rs.back().informativo = l > 60.0;   // lacunaMs = 120 ms cobre ~30 mm; acima disso vira busca (e a busca pode virar o carro para tras)
  }
  rs.push_back(cenarioCruzamento(0, 0));
  rs.push_back(cenarioCruzamento(10, 0));
  rs.push_back(cenarioCruzamento(-10, 8));
  rs.push_back(cenarioAtravessa());
  rs.push_back(cenarioCruzamentoImu());
  rs.push_back(cenarioAncora());
  rs.push_back(cenarioLevantado());
  rs.push_back(cenarioTravado());
  rs.push_back(cenarioColisao());
  for (double z : {65.0, 80.0, 95.0}) rs.push_back(cenarioCalibracao(z));
  for (double l : {60.0, 100.0, 150.0}) {
    rs.push_back(cenarioLacunaImu(l));
    rs.back().informativo = l > 100.0;
  }
  rs.push_back(cenarioTabuleiro());
  rs.push_back(cenarioEndireitar(true));
  rs.push_back(cenarioEndireitar(false));
  rs.push_back(cenarioBorda(kVermelha, false));
  rs.push_back(cenarioBorda(kAzul, false));
  rs.push_back(cenarioBorda(kVermelha, true));
  for (double d : {30.0, 60.0, 100.0})
    for (double rumo : {90.0, 0.0, 180.0, -90.0}) {
      rs.push_back(cenarioBusca(d, rumo));
      rs.back().informativo = d > 30.0;   // a barra alcanca ~48 mm do eixo: mais longe que isso depende da espiral
    }

  int falhas = 0;
  for (const Resultado& r : rs) {
    if (!gSo_Placar) printf("[%s] %s\n       %s\n", r.ok ? "ok  " : (r.informativo ? "info" : "FALHA"), r.nome.c_str(), r.info.c_str());
    falhas += (r.ok || r.informativo) ? 0 : 1;
  }
  if (gSo_Placar) printf("%d falhas | ", falhas);
  printf("\nPLACAR seguimento (%d percursos): tempo medio %.2f s, erro rms medio %.2f mm, pior desvio %.1f mm\n", gPlacar.n, gPlacar.tempoS / gPlacar.n, gPlacar.rms / gPlacar.n, gPlacar.pior);
  if (gHtmlPath) { escreverHtml(gHtmlPath); printf("pagina de visualizacao: %s (%d cenarios)\n", gHtmlPath, static_cast<int>(gViz.size())); }
  printf("%d de %d cenarios passaram\n", static_cast<int>(rs.size()) - falhas, static_cast<int>(rs.size()));
  return falhas ? 1 : 0;
}
