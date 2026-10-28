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
#include "seguidor/cruzamento.h"
#include "seguidor/decisao.h"
#include "seguidor/giro_cruzamento.h"

using namespace seguidor;

// ---------------------------------------------------------------- pista

enum CorSeg { kPreta = 0, kVermelha = 1, kAzul = 2 };   // vermelho e azul valem como PRETO para os sensores RC (suposto: confirme na pista)

struct Seg { double x0, y0, x1, y1; int cor = kPreta; };

struct Pista {
  std::vector<Seg> segs;
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
};

ParamsFisicos gFis;   // os cenarios usam este; da para mudar na linha de comando (veja main)

struct Estado { double x, y, th; };   // eixo (mm), rumo (rad, 0 = +x, anti-horario)

class Carro {
 public:
  Carro(const Pista& p, Estado e, const ParamsFisicos& f = gFis) : pista_(p), e_(e), f_(f) { grava(); }

  Estado estado() const { return e_; }
  unsigned long agoraMs() const { return t_; }
  const std::vector<Estado>& rastro() const { return rastro_; }
  double erroMax() const { return erroMax_; }

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
    const double dt = f_.dtMs / 1000.0, k = 1.0 - exp(-f_.dtMs / f_.tauMs);
    vE_ += (alvoE_ - vE_) * k;
    vD_ += (alvoD_ - vD_) * k;
    const double v = (vE_ + vD_) / 2.0, w = (vD_ - vE_) / f_.bitolaMm * f_.derrapagem;   // w > 0 = anti-horario (esquerda)
    // roda direita mais rapida -> gira para a esquerda; passo medio do rumo
    const double thMeio = e_.th + w * dt / 2.0;
    e_.x += v * cos(thMeio) * dt;
    e_.y += v * sin(thMeio) * dt;
    e_.th += w * dt;
    t_ += static_cast<unsigned long>(f_.dtMs);
    erroMax_ = std::max(erroMax_, erroBarra());
    if (t_ % 20 == 0) grava();
  }

 private:
  double alvo(int pwm) const { return abs(pwm) < f_.zonaMortaPwm ? 0.0 : (pwm > 0 ? 1.0 : -1.0) * abs(pwm) * f_.mmSPorPwm; }
  void grava() { rastro_.push_back(e_); }

  const Pista& pista_;
  Estado e_;
  ParamsFisicos f_;
  double vE_ = 0, vD_ = 0, alvoE_ = 0, alvoD_ = 0, erroMax_ = 0;
  unsigned long t_ = 0;
  std::vector<Estado> rastro_;
};

// ---------------------------------------------------------------- firmware simulado (espelha SeguidorBasico.ino)

struct Firmware {
  Ajustes ajustes;
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
  int bordas = 0, meiasVoltas = 0, paradasNavegador = 0;

  static void apaga(int& v, float x) { v = static_cast<int>(x); }

  static int acesos(const bool p[kSensores]) {
    int n = 0;
    for (int i = 0; i < kSensores; i++) n += p[i] ? 1 : 0;
    return n;
  }

  // Gira no eixo ate o angulo ESTIMADO (por tempo) chegar em alvo (espelho de girarAte do .ino). Guarda tambem o angulo VERDADEIRO de cada amostra.
  float girarAte(Carro& c, float agora, float alvo, std::vector<AmostraGiro>& est, std::vector<AmostraGiro>& ver, double thRef) {
    const bool esq = alvo > agora;
    const int v = ajustes.velBuscaExterna;
    c.comandar(esq ? -v : v, esq ? v : -v);
    const unsigned long inicio = c.agoraMs();
    while ((esq ? agora < alvo : agora > alvo) && c.agoraMs() - inicio < 3000) {
      c.avancar();
      bool p[kSensores];
      c.ler(p);
      agora += (esq ? 1 : -1) * ajustes.grausPorSegundoBusca * 0.005f;
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
    const AcaoCruzamento acao = acaoNaPonta(corRecente(memCor, c.agoraMs(), kJanelaCorMs), temCubo);
    if (acao != AcaoCruzamento::Cruzamento) {   // borda do mapa: nao gira para confirmar
      bordas++;
      if (acao == AcaoCruzamento::MeiaVolta) {
        std::vector<AmostraGiro> est, ver;
        girarAte(c, 0.0f, 180.0f, est, ver, c.estado().th);
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
    std::vector<AmostraGiro> est, ver;
    const double th0 = c.estado().th;   // angulo verdadeiro medido a partir de como chegou
    est.push_back({0, preto[0], preto[7], acesos(preto)});
    ver.push_back({0, preto[0], preto[7], acesos(preto)});
    float g = girarAte(c, 0.0f, ajG.giroGraus, est, ver, th0);
    g = girarAte(c, g, -ajG.giroGraus, est, ver, th0);
    girarAte(c, g, 0.0f, est, ver, th0);
    const ResultadoGiro r = avaliarGiro(est.data(), static_cast<int>(est.size()), ajG);
    const ResultadoGiro rv = avaliarGiro(ver.data(), static_cast<int>(ver.size()), ajG);   // oracula: com o angulo verdadeiro
    giroDriftGraus = static_cast<float>(remainder(c.estado().th - th0, 2 * M_PI) * 180.0 / M_PI);
    ruaEsq = r.ruaEsquerda;
    ruaDir = r.ruaDireita;
    ruaEsqVerdade = rv.ruaEsquerda;
    ruaDirVerdade = rv.ruaDireita;
    girou = true;
    if (r.cruzamento) cruzamentos++; else falsosAlarmes++;
    esquecerUltimoComando(memoria);
    tUltimoCruzamento = c.agoraMs();
    fimDeBecoAvisado = false;
    comecarASair(cruz, c.agoraMs());
  }

  // Um ciclo do loop().
  void passo(Carro& c) {
    if (parado) { c.parar(); c.avancar(); return; }
    bool p[kSensores];
    c.ler(p);
    const unsigned long agora = c.agoraMs();
    const int corChao = c.corChao();
    anotarCor(memCor, corChao == kVermelha ? CorLinha::Vermelho : (corChao == kAzul ? CorLinha::Azul : CorLinha::Outra), agora);
    if (paraNoCruzamento && !estaBuscando(memoria, agora, ajustes) && cruzamentoConfirmado(p, agora, ajC, cruz)) {
      pararNoCruzamento(c, p);
      return;
    }
    const Comando cmd = decidir(p, agora, ajustes, memoria);
    const unsigned long perdidoHa = memoria.perdeuEm ? agora - memoria.perdeuEm : 0;
    if (memoria.perdeuEm) ticksPerdido++;
    if (estaBuscando(memoria, agora, ajustes)) ticksBuscando++;
    if (!fimDeBecoAvisado && tUltimoCruzamento &&
        classificarPerda(perdidoHa, agora - tUltimoCruzamento, ajustes.lacunaMs, ajC) == Perda::FimDeBeco) {
      fimDeBecoAvisado = true;
      fimDeBeco = true;
    }
    c.comandar(cmd.parar ? 0 : cmd.esq, cmd.parar ? 0 : cmd.dir);
    c.avancar();
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

static void svg(const std::string& nome, const Pista& p, const Carro& c) {
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

// Roda o firmware ate `fim` ser verdadeiro ou estourar maxMs.
template <class Fim>
static bool rodar(Firmware& fw, Carro& c, unsigned long maxMs, Fim fim) {
  while (c.agoraMs() < maxMs) {
    fw.passo(c);
    if (fim(c)) return true;
  }
  return false;
}

static Estado norte(double x, double y, double desvioGraus = 0) { return {x, y, M_PI / 2 + desvioGraus * M_PI / 180.0}; }

// 1. Reta: varia o desvio lateral e o angulo de partida; tem de chegar ao fim sem perder a linha.
static Resultado cenarioReta(const char* nome, double off, double ang, bool salva) {
  Pista p;
  p.reta(0, 0, 0, 1000);
  Carro c(p, norte(off, 100, ang));
  Firmware fw;
  const bool chegou = rodar(fw, c, 12000, [](const Carro& k) { return k.estado().y > 900; });
  const bool ok = chegou && c.erroBarra() < 6.0 && fw.ticksBuscando == 0;
  if (salva) svg(nome, p, c);
  return {nome, ok, fmt("off %+.0f mm, ang %+.0f: %s em %lu ms, erro final %.1f mm, erro max %.1f mm, buscando %d ciclos", off, ang, chegou ? "chegou" : "NAO chegou", c.agoraMs(),
                       c.erroBarra(), c.erroMax(), fw.ticksBuscando)};
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
  const bool chegou = rodar(fw, c, 15000, [&](const Carro& k) { return fabs(k.estado().x) > 450; });
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
  const bool chegou = rodar(fw, c, 15000, [&](const Carro& k) { return k.estado().x > raio + 300; });
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
  rodar(fw, c, 15000, [&](const Carro&) { return fw.fimDeBeco; });
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
  rodar(fw, c, 20000, [&](const Carro& k) {
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

// 3b. Depois do cruzamento o carro tem de seguir a linha do outro lado (rua e beco longos).
static Resultado cenarioAtravessa() {
  Pista p;
  p.reta(0, 0, 0, 1000);
  p.reta(-400, 500, 400, 500);
  Carro c(p, norte(0, 100));
  Firmware fw;
  const bool chegou = rodar(fw, c, 20000, [](const Carro& k) { return k.estado().y > 900; });
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
  const bool chegou = rodar(fw, c, 12000, [](const Carro& k) { return k.estado().y > 900; });
  const bool ok = chegou && c.erroBarra() < 6.0;
  svg(fmt("lacuna_%.0f", lacunaMm), p, c);
  return {fmt("lacuna de %.0f mm", lacunaMm), ok, fmt("%s em %lu ms, erro final %.1f mm, buscando %d ciclos", chegou ? "chegou" : "NAO chegou", c.agoraMs(), c.erroBarra(), fw.ticksBuscando)};
}

// 5. Busca: carro solto fora da linha (distancia lateral e rumo variados). Informativo + criterio: achar a linha em ate 15 s.
static Resultado cenarioBusca(double dist, double rumoGraus) {
  Pista p;
  p.reta(0, -400, 0, 1400);
  Carro c(p, {dist, 500, rumoGraus * M_PI / 180.0});
  Firmware fw;
  const bool achou = rodar(fw, c, 15000, [&](const Carro& k) {
    return fw.ticksBuscando > 0 && k.erroBarra() < 8.0 && fw.memoria.perdeuEm == 0;
  });
  svg(fmt("busca_d%.0f_r%.0f", dist, rumoGraus), p, c);
  return {fmt("busca a %.0f mm, rumo %.0f", dist, rumoGraus), achou, fmt("%s em %lu ms (buscando %d ciclos)", achou ? "achou a linha" : "NAO achou", c.agoraMs(), fw.ticksBuscando)};
}

int main(int argc, char** argv) {
  // Argumentos: uma pasta (grava um SVG por cenario) e/ou chave=valor para os parametros fisicos: tau, derrap, bitola, zona, ganho, barra
  for (int i = 1; i < argc; i++) {
    const char* eq = strchr(argv[i], '=');
    if (!eq) { gSvgDir = argv[i]; continue; }
    const std::string k(argv[i], eq - argv[i]);
    const double v = atof(eq + 1);
    if (k == "tau") gFis.tauMs = v;
    else if (k == "derrap") gFis.derrapagem = v;
    else if (k == "bitola") gFis.bitolaMm = v;
    else if (k == "zona") gFis.zonaMortaPwm = v;
    else if (k == "ganho") gFis.mmSPorPwm = v;
    else if (k == "barra") gFis.barraAFrenteMm = v;
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
    printf("[%s] %s\n       %s\n", r.ok ? "ok  " : (r.informativo ? "info" : "FALHA"), r.nome.c_str(), r.info.c_str());
    falhas += (r.ok || r.informativo) ? 0 : 1;
  }
  printf("\n%d de %d cenarios passaram\n", static_cast<int>(rs.size()) - falhas, static_cast<int>(rs.size()));
  return falhas ? 1 : 0;
}
