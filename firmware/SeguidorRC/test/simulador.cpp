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
#include <array>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <string>
#include <vector>

#define ROBO_SIMULADOR   // a logica do carro (seguidor/robo.h) imprime depuracao e le SIM_* so aqui
#include "seguidor/robo.h"

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
  // O que o TCS do chao ve: vermelho/azul (linha colorida), verde/amarelo (zonas), preto (linha), branco.
  using Piso = seguidor::Piso;   // enum em seguidor/tabuleiro.h (o robo tambem o usa)
  static constexpr Piso kPisoBranco = seguidor::kPisoBranco, kPisoPreto = seguidor::kPisoPreto, kPisoVermelho = seguidor::kPisoVermelho, kPisoAzul = seguidor::kPisoAzul, kPisoVerde = seguidor::kPisoVerde, kPisoAmarelo = seguidor::kPisoAmarelo;
  Piso piso(double x, double y) const {
    for (const Seg& s : segs)
      if (s.cor != kPreta && distSeg(s, x, y) < largura / 2.0) return s.cor == kVermelha ? kPisoVermelho : kPisoAzul;
    for (const Ret& r : decor)
      if (r.cor <= 1 && x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1) return r.cor == 0 ? kPisoVerde : kPisoAmarelo;
    return preto(x, y) ? kPisoPreto : kPisoBranco;
  }
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
  bool encoder = false;           // STS3215: encoder de cada roda + velocidade em malha fechada (sem zona morta nem assimetria fixa entre os lados); a distancia vem das rodas, nao de tempo x velocidade
  double escalaOdo = 1.0;         // erro de escala do odometro (diametro da roda / relacao de engrenagens): 1,01 = le 1% a mais do que anda
};

Ajustes gAjustes;   // os cenarios partem destes (mude na linha de comando para varrer parametros da decisao)
bool gSo_Placar = false;
double gGestoGraus = 45.0;   // modo encoder: no cruzamento esperado (odometro ~400 mm, pose segura) so da uma rodadinha deste angulo para UM lado e volta reto, em vez da varredura de 90 graus (0 = desliga e usa a varredura de antes, < 0 = passa direto sem girar)
static bool gAutoCalBitola = false;   // param autocal=1: cada giro na linha corrige a bitola efetiva (o meio da faixa tem ~5% de vies: piora o caso ja calibrado)
static bool gRumoPeloEncoder = false;   // param rumoenc=1: sem MPU, o rumo vem dos encoders das rodas
ParamsFisicos gFis;   // os cenarios usam este; da para mudar na linha de comando (veja main)


constexpr double kEscalaRuido = 0.5;   // intensidade do modo ruido (1,0 = original; 0,5 = metade da frequencia, da duracao e do desvio)

// ================================================================ O MUNDO alem da pista: cubos, oponente, sensores configuraveis e RUIDO
// Tudo aqui e CONFIGURAVEL e fica pronto antes dos sensores existirem no robo (ultrassom, TCS da frente). O RUIDO e aleatorio mas deterministico (semente) e
// SEMPRE SINALIZADO: cada evento entra em Mundo::log (a pagina mostra) e a mancha aparece desenhada. Desligado por padrao nos testes.
struct Cubo {
  double x, y;
  seguidor::CorCubo cor;
  bool vivo = true;
  unsigned long tocadoAte = 0;
  bool obstaculo = false;   // cubo num quadrado MARROM (centro do quarteirao): so obstaculo para os sensores de distancia, nunca se mexe
  size_t mortoEm = static_cast<size_t>(-1);   // indice do quadro em que saiu do mapa (coletado ou levado pelo oponente), para a animacao
  int c = 0, r = 0;
  bool leste = true;   // a rua e (c,r)->(c+1,r) se leste, senao (c,r)->(c,r+1)
};


struct Oponente {
  bool ativo = false;
  bool externo = false;   // movido por outro agente (outro robo igual ao nosso), nao pelo passeio aleatorio
  double x = 0, y = 0, th = 0, v = 150.0;
  int c = 0, r = 0, alvoC = 0, alvoR = 0;
  unsigned long paradoAte = 0;
};

struct Mancha {   // sujeira escura (parece linha) ou desgaste branco na fita (a linha some): os sensores RC leem isso
  double x, y, raio;
  bool branca = false;
  size_t desde, ate = static_cast<size_t>(-1);   // quadros em que existe
  unsigned long expiraMs;
};

struct ConfigSensoresSim {
  bool us = true;              // ultrassom (HC-SR04): ainda nao instalado, mas o simulador ja o modela
  double usAlcanceMm = 900.0, usConeGraus = 20.0, usRuidoMm = 3.0, usFrenteMm = 100.0;   // ponta da frente do carro (eixo + 100)
  bool tcsFrente = true;       // TCS34725 da frente: cor do cubo a ~5 cm
  double tcsFrenteAlcanceMm = 90.0, tcsFrenteConeGraus = 14.0;
  double tcsChaoErro = 0.0;    // probabilidade de errar a cor do chao numa leitura (so com ruido ligado)
  unsigned long tcsPeriodoMs = 30;   // leitura de ~24 ms: a faixa vermelha de 20 mm dura ~80 ms a 250 mm/s, entao precisa de 2 leituras seguidas dentro dela
};

struct Mundo {
  std::vector<Cubo> cubos;
  std::vector<Plataforma> plataformas;   // as do NOSSO lado (o juiz poe de 1 a 5, cores distintas)
  std::vector<std::array<int8_t, 2>> cargaQuadros;   // o que o carrinho leva em cada quadro (cor do cubo, 0 = nada)
  int8_t carga0 = 0, carga1 = 0;
  Oponente op;
  std::vector<Mancha> manchas;
  ConfigSensoresSim cfg;
  bool ruidoLigado = false;
  std::vector<std::string> log;
  std::vector<std::array<float, 3>> opQuadros;   // oponente em cada quadro gravado
  unsigned long long seed = 777;
  unsigned long prox[10] = {20000, 15000, 25000, 30000, 18000, 35000, 50000, 60000, 40000, 45000}, fimPatina = 0, fimAtraso = 0, fimMotor = 0;
  bool assimetriaSorteada = false;
  unsigned long cooldownColisao = 0;
  int pegosPeloOponente = 0, colisoesOponente = 0, toquesEmCubos = 0, toquesPendentes = 0;

  double rnd() { seed = seed * 6364136223846793005ULL + 1442695040888963407ULL; return ((seed >> 33) & 0xFFFFFF) / 16777216.0; }   // 0..1
  double gauss() { return (rnd() + rnd() + rnd() + rnd() - 2.0) * 1.7; }   // ~N(0,1)
  void evento(unsigned long t, const std::string& txt) {
    char b[64];
    snprintf(b, sizeof b, "[%.1fs] ", t / 1000.0);
    log.push_back(std::string(b) + txt);
    if (log.size() > 5000) log.erase(log.begin());
  }
  bool cuboEm(double x, double y) const {
    for (const Cubo& c : cubos)
      if (c.vivo && fabs(x - c.x) < 22 && fabs(y - c.y) < 22) return true;
    return false;
  }
  bool manchaEm(double x, double y, bool branca = false) const {
    for (const Mancha& m : manchas)
      if (m.branca == branca && m.ate == static_cast<size_t>(-1) && hypot(x - m.x, y - m.y) < m.raio) return true;
    return false;
  }
  // Espalha n cubos em ruas diferentes (nunca no cruzamento), no meio da rua +- 40 mm, com as cores dadas em ordem.
  void semearCubos(int n, const std::vector<seguidor::CorCubo>& cores) {
    std::vector<std::array<int, 3>> ruas;   // c, r, leste
    for (int c = 0; c < core::kMapCols; c++)
      for (int r = 0; r < core::kMapRows; r++) {
        if (c + 1 < core::kMapCols) ruas.push_back({c, r, 1});
        if (r + 1 < core::kMapRows) ruas.push_back({c, r, 0});
      }
    for (int i = 0; i < n && !ruas.empty(); i++) {
      const size_t k = static_cast<size_t>(rnd() * ruas.size()) % ruas.size();
      const auto rua = ruas[k];
      ruas.erase(ruas.begin() + static_cast<long>(k));
      Cubo cu;
      cu.c = rua[0]; cu.r = rua[1]; cu.leste = rua[2] == 1;
      const double meio = 0.5;   // os quadrados cinza ficam no MEIO da rua (a ~200 mm do cruzamento): ai o giro do carro no cruzamento nao encosta (no robo real nao encostava)
      cu.x = xDoCruzamento(cu.c) + (cu.leste ? meio * kPasso : 0.0);
      cu.y = yDoCruzamento(cu.r) + (cu.leste ? 0.0 : meio * kPasso);
      cu.cor = cores[static_cast<size_t>(i) % cores.size()];
      cubos.push_back(cu);
    }
  }
  // Distribuicao das regras (5.1): 5 cubos de cada uma das 6 cores e 6 brancos, espalhados; aqui `n` deles, nas ruas (quadrados cinza).
  void semearRegras(int n) {
    static const seguidor::CorCubo cores[7] = {seguidor::CorCubo::Azul, seguidor::CorCubo::Vermelho, seguidor::CorCubo::Verde, seguidor::CorCubo::Amarelo, seguidor::CorCubo::Preto, seguidor::CorCubo::Marrom, seguidor::CorCubo::Branco};
    std::vector<seguidor::CorCubo> lista;
    for (int i = 0; i < n; i++) {
      const double r = rnd() * 36.0;   // 6 cores x 5 + 6 brancos = 36
      lista.push_back(r < 30.0 ? cores[static_cast<int>(r / 5.0)] : seguidor::CorCubo::Branco);
    }
    semearCubos(n, lista);
  }
  // Um cubo numa rua escolhida, no meio dela (para montar cenarios).
  void porCubo(int c, int r, bool leste, seguidor::CorCubo cor) {
    Cubo cu;
    cu.c = c; cu.r = r; cu.leste = leste;
    cu.x = xDoCruzamento(c) + (leste ? kPasso / 2 : 0.0);
    cu.y = yDoCruzamento(r) + (leste ? 0.0 : kPasso / 2);
    cu.cor = cor;
    cubos.push_back(cu);
  }
  // Cubos nos quadrados MARROM (centro dos quarteiroes): obstaculos so para os sensores de distancia (nao se pegam, nao se mexem).
  void semearObstaculos(int k) {
    for (int i = 0; i < k; i++) {
      Cubo cu;
      cu.c = static_cast<int>(rnd() * 5) % 5; cu.r = static_cast<int>(rnd() * 4) % 4;
      cu.x = xDoCruzamento(cu.c) + kPasso / 2; cu.y = yDoCruzamento(cu.r) + kPasso / 2;
      cu.cor = seguidor::CorCubo::Marrom;
      cu.obstaculo = true;
      cubos.push_back(cu);
    }
  }
  // Plataformas no nosso ecoponto (x = kPlatX): ate 5 posicoes; coloca as cores dadas nos slots dados.
  void porPlataformas(const std::vector<std::pair<double, seguidor::CorCubo>>& slots) {   // slot 0..4 = as 5 marcas; 0,5 / 1,5 ... = centradas entre elas
    plataformas.clear();
    for (const auto& sl : slots) plataformas.push_back({kPlatX, yDaPlataforma(sl.first), sl.second});
  }
  const Plataforma* plataformaDaCor(seguidor::CorCubo c) const {
    for (const Plataforma& p : plataformas) if (p.cor == c) return &p;
    return nullptr;
  }
  void tick(class Carro& c);
  void gravaQuadro() {
    opQuadros.push_back({static_cast<float>(op.x), static_cast<float>(op.y), static_cast<float>(op.th)});
    cargaQuadros.push_back({carga0, carga1});
  }
};

class Carro {
 public:
  Carro(const Pista& p, Estado e, const ParamsFisicos& f = gFis) : pista_(p), e_(e), f_(f) { grava(); }

  void usarMundo(Mundo* m) { mundo_ = m; if (m) { m->opQuadros.clear(); m->cargaQuadros.clear(); m->gravaQuadro(); } }
  Mundo* mundo() const { return mundo_; }
  void fatorRodas(double f) { fatorRodas_ = f; }          // patinagem / ganho das rodas (ruido)
  void atrasoLeitura(int ticks) { atrasoTicks_ = ticks; }  // atraso dos sensores RC em ciclos (ruido)
  double gE_ = 1.0, gD_ = 1.0;   // ganho da compensacao dos motores (firmware -> ponte)
  void assimetria(double esq, double dir) { aE_ = esq; aD_ = dir; }   // diferenca FIXA entre os dois motores (a partida inteira, ruido)
  void fatorLados(double esq, double dir) { fE_ = esq; fD_ = dir; }   // motor de um lado mais fraco (ruido)
  void prenderSensor(int i, bool preto, unsigned long ateMs) { presoAte_[i] = ateMs; presoValor_[i] = preto ? 1 : 0; }   // sensor RC preso (ruido)
  void ultrassomFalso(unsigned long ateMs, double mm) { usFalsoAte_ = ateMs; usFalsoMm_ = mm; }   // eco falso (mm > 0) ou perda de eco (-1) por um tempo (ruido)
  void saltarVies(double grausS) { extraVies_ += grausS; }   // o viés do giroscopio anda (temperatura)
  void chutar(double graus) { e_.th += graus * M_PI / 180.0; }   // solavanco: o carro gira sem comando (ruido)
  double velNominalMmS(int pwm) const { return pwm * f_.mmSPorPwm; }

  Estado estado() const { return e_; }

  // ---- MPU simulado (gz em rad/s COM vies, aceleracao em g). Ruido deterministico (LCG): o mesmo resultado a cada execucao.
  Imu imu() {
    const double vies = (f_.viesGzGrauS + extraVies_ + f_.derivaViesGrauS * (levantado_ ? 0.0 : t_ / 1000.0)) / 57.29578;
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
  void limparRastro() { rastro_.clear(); bits_.clear(); if (mundo_) { mundo_->opQuadros.clear(); mundo_->cargaQuadros.clear(); } grava(); }   // o que vem depois e o que interessa mostrar
  void colocarEm(Estado e) { e_ = e; vE_ = vD_ = 0; grava(); }   // solto no chao em outro lugar
  void bater(double g, int ticks) { impulsoG_ = g; impulsoTicks_ = ticks; }   // pancada horizontal de g por `ticks` ciclos
  bool levantado() const { return levantado_; }
  double roda(bool esquerda) const { return esquerda ? alvoE_ : alvoD_; }   // alvo da roda (mm/s) para conferir que parou
  unsigned long agoraMs() const { return t_; }
  double velMmS() const { return vMmS_; }
  const std::vector<Estado>& rastro() const { return rastro_; }
  const std::vector<uint8_t>& bits() const { return bits_; }
  double erroMax() const { return erroMax_; }
  double erroRms() const { return n_ ? sqrt(soma2_ / n_) : 0.0; }   // erro medio quadratico da barra ate a linha, a cada ciclo

  void ponto(int i, double& x, double& y) const {
    const double lat = f_.barraMm / 2.0 - i * f_.barraMm / (kSensores - 1);   // S1 = esquerda = +lado
    x = e_.x + f_.barraAFrenteMm * cos(e_.th) - lat * sin(e_.th);
    y = e_.y + f_.barraAFrenteMm * sin(e_.th) + lat * cos(e_.th);
  }
  void lerReal(bool preto[kSensores]) const {
    for (int i = 0; i < kSensores; i++) {
      double x, y;
      ponto(i, x, y);
      bool p = pista_.preto(x, y);
      if (mundo_) p = (p && !mundo_->cuboEm(x, y) && !mundo_->manchaEm(x, y, true)) || mundo_->manchaEm(x, y, false);   // cubo e desgaste cobrem a linha; sujeira parece linha
      if (presoAte_[i] > t_) p = presoValor_[i] != 0;                                                              // sensor RC preso (ruido)
      preto[i] = p;
    }
  }
  void ler(bool preto[kSensores]) const {
    if (atrasoTicks_ > 0 && ringN_ > atrasoTicks_) {   // sensores atrasados (ruido): devolve a leitura de alguns ciclos atras
      const uint8_t b = ring_[(ringI_ - 1 - atrasoTicks_ + 64) & 31];
      for (int i = 0; i < kSensores; i++) preto[i] = (b >> i) & 1;
      return;
    }
    lerReal(preto);
  }

  // ---- sensores novos (configuraveis em Mundo::cfg): TCS do chao, ultrassom, TCS da frente
  Pista::Piso piso() const {
    double x, y, x2, y2;
    ponto(3, x, y);
    ponto(4, x2, y2);
    return pista_.piso((x + x2) / 2, (y + y2) / 2);
  }
  // Cor lida pelo TCS do chao (centro da barra). Com ruido ligado, as vezes erra (probabilidade cfg.tcsChaoErro).
  seguidor::CorLinha tcsChao() {
    const Pista::Piso pz = piso();
    seguidor::CorLinha c = pz == Pista::kPisoVermelho ? seguidor::CorLinha::Vermelho : pz == Pista::kPisoAzul ? seguidor::CorLinha::Azul : pz == Pista::kPisoVerde ? seguidor::CorLinha::Verde : seguidor::CorLinha::Outra;
    if (mundo_ && mundo_->ruidoLigado && mundo_->rnd() < mundo_->cfg.tcsChaoErro * kEscalaRuido) {
      c = static_cast<seguidor::CorLinha>(static_cast<int>(mundo_->rnd() * 4) % 4);
      mundo_->evento(t_, "RUIDO: o TCS do chao leu uma cor errada");
    }
    return c;
  }
  // Ultrassom a frente: distancia (mm) ate o alvo mais proximo dentro do cone (cubos vivos r = 25, oponente r = 95), ou -1.
  double us() {
    if (!mundo_ || !mundo_->cfg.us) return -1.0;
    if (usFalsoAte_ > t_) return usFalsoMm_;   // eco falso ou perda de eco (ruido)
    const Mundo& m = *mundo_;
    const double sx = e_.x + m.cfg.usFrenteMm * cos(e_.th), sy = e_.y + m.cfg.usFrenteMm * sin(e_.th);
    double melhor = -1.0;
    auto alvo = [&](double x, double y, double raio) {
      const double d = hypot(x - sx, y - sy);
      if (d > m.cfg.usAlcanceMm + raio || d < 1e-6) return;
      const double dif = fabs(remainder(atan2(y - sy, x - sx) - e_.th, 2 * M_PI)) * 180.0 / M_PI;
      if (dif > m.cfg.usConeGraus / 2.0) return;
      const double dist = std::max(0.0, d - raio);
      if (melhor < 0 || dist < melhor) melhor = dist;
    };
    for (const Plataforma& pl : m.plataformas) {   // plataforma (50 x 160 mm): tres raios dentro do cone, o mais curto que bate no retangulo
      for (const double off : {-0.5, 0.0, 0.5}) {
        const double ang = e_.th + off * m.cfg.usConeGraus * M_PI / 180.0;
        for (double t = 0; t <= m.cfg.usAlcanceMm; t += 5.0) {
          if (fabs(sx + t * cos(ang) - pl.x) <= kPlatX_Tam / 2 && fabs(sy + t * sin(ang) - pl.y) <= kPlatY_Tam / 2) { if (melhor < 0 || t < melhor) { melhor = t; ecoTipo_ = "plataforma"; } break; }
        }
      }
    }
    for (const Cubo& c : m.cubos) if (c.vivo) { const double antes = melhor; alvo(c.x, c.y, 25.0); if (melhor != antes && melhor >= 0) ecoTipo_ = fmt("cubo cor %d em (%.0f,%.0f)%s", static_cast<int>(c.cor), c.x, c.y, c.obstaculo ? " OBSTACULO" : ""); }
    if (m.op.ativo) { const double antes = melhor; alvo(m.op.x, m.op.y, 95.0); if (melhor != antes && melhor >= 0) ecoTipo_ = fmt("OUTRO ROBO em (%.0f,%.0f)", m.op.x, m.op.y); }
    if (melhor < 0) return -1.0;
    return std::max(1.0, melhor + (mundo_->ruidoLigado ? m.cfg.usRuidoMm * mundo_->gauss() : 0.0));
  }
  // TCS da frente: cor do cubo que esta a frente a ate cfg.tcsFrenteAlcanceMm (o oponente nao tem cor de cubo: Nenhuma).
  seguidor::CorCubo tcsFrente() const {
    if (!mundo_ || !mundo_->cfg.tcsFrente) return seguidor::CorCubo::Nenhuma;
    const Mundo& m = *mundo_;
    const double sx = e_.x + m.cfg.usFrenteMm * cos(e_.th), sy = e_.y + m.cfg.usFrenteMm * sin(e_.th);
    for (const Cubo& c : m.cubos) {
      if (!c.vivo) continue;
      const double d = hypot(c.x - sx, c.y - sy) - 25.0;
      const double lat = fabs(-(c.x - sx) * sin(e_.th) + (c.y - sy) * cos(e_.th)), fre = (c.x - sx) * cos(e_.th) + (c.y - sy) * sin(e_.th);
      if (fre > -30.0 && d <= m.cfg.tcsFrenteAlcanceMm && lat <= 28.0) return c.cor;   // o campo do sensor perto do cubo e uma faixa estreita, nao um angulo
    }
    return seguidor::CorCubo::Nenhuma;
  }
  // Plataforma de cor X a frente (TCS da frente, como o cubo): so o campo estreito a frente, ate `alcanceMm` da ponta.
  seguidor::CorCubo plataformaNaFrente(double alcanceMm) const {
    if (!mundo_) return seguidor::CorCubo::Nenhuma;
    const double sx = e_.x + mundo_->cfg.usFrenteMm * cos(e_.th), sy = e_.y + mundo_->cfg.usFrenteMm * sin(e_.th);
    for (const Plataforma& p : mundo_->plataformas) {
      // retangulo 50 (x) x 160 (y) centrado em p.x, p.y (amostra a cada 5 mm ao longo do raio)
      for (double t = 0; t <= alcanceMm; t += 5.0) {
        const double px = sx + t * cos(e_.th), py = sy + t * sin(e_.th);
        if (fabs(px - p.x) <= kPlatX_Tam / 2 && fabs(py - p.y) <= kPlatY_Tam / 2) return p.cor;
      }
    }
    return seguidor::CorCubo::Nenhuma;
  }
  // O cubo vivo mais perto da frente do carro (para coletar): indice, ou -1.
  int cuboNaFrente(double alcanceMm) const {
    if (!mundo_) return -1;
    const Mundo& m = *mundo_;
    const double sx = e_.x + m.cfg.usFrenteMm * cos(e_.th), sy = e_.y + m.cfg.usFrenteMm * sin(e_.th);
    int melhor = -1;
    double dm = 1e9;
    for (size_t i = 0; i < m.cubos.size(); i++) {
      const Cubo& c = m.cubos[i];
      if (!c.vivo || c.obstaculo) continue;   // o cubo de quadrado marrom nao se mexe: nunca e alvo de coleta
      const double d = hypot(c.x - sx, c.y - sy) - 25.0;
      const double lat = fabs(-(c.x - sx) * sin(e_.th) + (c.y - sy) * cos(e_.th)), fre = (c.x - sx) * cos(e_.th) + (c.y - sy) * sin(e_.th);   // lateral e a frente, em relacao a ponta do carro
      if (fre > -30.0 && d <= alcanceMm && lat <= 30.0 && d < dm) { dm = d; melhor = static_cast<int>(i); }
    }
    return melhor;
  }
  // Pega o cubo i (some do mapa neste quadro).
  void coletar(int i) {
    if (!mundo_ || i < 0) return;
    mundo_->cubos[static_cast<size_t>(i)].vivo = false;
    mundo_->cubos[static_cast<size_t>(i)].mortoEm = rastro_.size();
  }
  // ---- interface da missao que o robo.h espera (no carro real: sem mundo, com ecoponto de verdade)
  bool missao() const { return mundo_ != nullptr; }
  bool usAtivo() const { return mundo_ && mundo_->cfg.us; }
  unsigned long tcsPeriodoMs() const { return mundo_ ? mundo_->cfg.tcsPeriodoMs : 0; }
  int consumirToques() { if (!mundo_) return 0; const int n = mundo_->toquesPendentes; mundo_->toquesPendentes = 0; return n; }   // cubos tocados sem coletar (sinalizados pelo juiz)
  void mostrarCarga(int slot, int cor) { if (!mundo_) return; (slot == 0 ? mundo_->carga0 : mundo_->carga1) = static_cast<int8_t>(cor); }
  void limparCarga() { if (mundo_) mundo_->carga0 = mundo_->carga1 = 0; }
  void evento(const std::string& texto) { if (mundo_) mundo_->evento(t_, texto); }
  size_t quadro() const { return rastro_.size(); }
  std::string ecoTipo_;   // so depuracao: o que deu o ultimo eco
  const std::string& ecoTipo() const { return ecoTipo_; }

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

  // Compensacao dos motores (aumenta a forca do lado fraco), como definirCompensacao() da ponte H real: multiplica o PWM e limita em 255.
  double odoEMm() const { return odoE_; }
  double odoDMm() const { return odoD_; }
  double odoMm() const { return (odoE_ + odoD_) / 2.0; }   // percurso medio das rodas desde o inicio (mm, com sinal; quem mede usa diferencas)
  void compensar(double esq, double dir) { gE_ = esq; gD_ = dir; }
  static int comGanho(int pwm, double g) { const double v = pwm * g; return static_cast<int>(v > 255 ? 255 : v < -255 ? -255 : v); }
  void comandar(int esq, int dir) { alvoE_ = alvo(comGanho(esq, gE_)); alvoD_ = alvo(comGanho(dir, gD_)); }
  void parar() { alvoE_ = alvoD_ = 0; }

  // Passa um ciclo (dtMs) com o comando atual.
  void avancar() {
    if (levantado_) { t_ += static_cast<unsigned long>(f_.dtMs); if (t_ % 20 == 0) grava(); return; }   // as rodas giram no ar: nao sai do lugar
    const double dt = f_.dtMs / 1000.0, k = 1.0 - exp(-f_.dtMs / f_.tauMs);
    vE_ += (alvoE_ - vE_) * k;
    vD_ += (alvoD_ - vD_) * k;
    odoE_ += vE_ * f_.escalaOdo * dt;   // o encoder le o eixo da roda: nao ve a patinagem no chao (fatorRodas_, derrapagem) nem o torque que falta
    odoD_ += vD_ * f_.escalaOdo * dt;
    const double ve = f_.encoder ? vE_ : vE_ * fE_ * aE_, vd = f_.encoder ? vD_ : vD_ * fD_ * aD_;   // malha fechada no servo: o lado fraco e compensado
    const double v = (ve + vd) / 2.0 * fatorRodas_, w = (vd - ve) / f_.bitolaMm * f_.derrapagem * fatorRodas_;   // w > 0 = anti-horario (esquerda)
    aMmS2_ = (v - vMmS_) / dt;
    vMmS_ = v;
    wRadS_ = w;
    // roda direita mais rapida -> gira para a esquerda; passo medio do rumo
    const double thMeio = e_.th + w * dt / 2.0;
    e_.x += v * cos(thMeio) * dt;
    e_.y += v * sin(thMeio) * dt;
    e_.th += w * dt;
    t_ += static_cast<unsigned long>(f_.dtMs);
    {
      bool p[kSensores];
      lerReal(p);
      uint8_t b = 0;
      for (int i = 0; i < kSensores; i++) b |= p[i] ? (1 << i) : 0;
      ring_[ringI_ & 31] = b;
      ringI_++;
      if (ringN_ < 64) ringN_++;
    }
    if (mundo_) mundo_->tick(*this);
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
    if (mundo_) mundo_->gravaQuadro();
    bool p[kSensores];
    lerReal(p);
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
  double odoE_ = 0.0, odoD_ = 0.0;   // percurso de cada roda lido pelo encoder (mm, com sinal)
  bool levantado_ = false;
  Mundo* mundo_ = nullptr;
  double fatorRodas_ = 1.0, fE_ = 1.0, fD_ = 1.0, aE_ = 1.0, aD_ = 1.0, extraVies_ = 0.0, usFalsoMm_ = 0.0;
  unsigned long presoAte_[8] = {};
  int presoValor_[8] = {};
  unsigned long usFalsoAte_ = 0;
  int atrasoTicks_ = 0;
  uint8_t ring_[32] = {};
  int ringI_ = 0, ringN_ = 0;
  unsigned long long seed_ = 12345;
  long n_ = 0;
  unsigned long t_ = 0;
  std::vector<Estado> rastro_;
  std::vector<uint8_t> bits_;
};

// ---------------------------------------------------------------- o mundo anda: oponente, cubos que somem, ruido SINALIZADO
inline void Mundo::tick(Carro& c) {
  const unsigned long t = c.agoraMs();
  const Estado e = c.estado();
  if (op.ativo && !op.externo) {
    if (t >= op.paradoAte) {
      const double tx = xDoCruzamento(op.alvoC), ty = yDoCruzamento(op.alvoR);
      const double d = hypot(tx - op.x, ty - op.y);
      if (d < 3.0) {   // chegou: escolhe outra rua (nao volta, a menos que seja beco)
        static const int dc[4] = {1, 0, -1, 0}, dr[4] = {0, 1, 0, -1};
        int cand[4], n = 0;
        for (int k = 0; k < 4; k++) {
          const int nc = op.alvoC + dc[k], nr = op.alvoR + dr[k];
          if (core::inMap(nc, nr) && !(nc == op.c && nr == op.r)) cand[n++] = k;
        }
        if (n == 0) cand[n++] = 0;
        const int k = cand[static_cast<int>(rnd() * n) % n];
        op.c = op.alvoC; op.r = op.alvoR;
        op.alvoC = std::max(0, std::min(core::kMapCols - 1, op.c + dc[k]));
        op.alvoR = std::max(0, std::min(core::kMapRows - 1, op.c == op.alvoC && dr[k] == 0 ? op.r : op.r + dr[k]));
      } else {
        const double passo = std::min(d, op.v * 0.005);
        op.x += (tx - op.x) / d * passo;
        op.y += (ty - op.y) / d * passo;
        op.th = atan2(ty - op.y, tx - op.x);
      }
    }
    for (Cubo& cu : cubos)   // o oponente leva cubos onde nos nao vemos
      if (cu.vivo && hypot(cu.x - op.x, cu.y - op.y) < 40.0 && rnd() < 0.004) {
        cu.vivo = false;
        cu.mortoEm = c.quadro();
        pegosPeloOponente++;
        evento(t, fmt("OPONENTE levou um cubo da rua (%d,%d)", cu.c, cu.r));
      }
    if (hypot(e.x - op.x, e.y - op.y) < 170.0 && t >= cooldownColisao) {   // bateu num robo
      c.bater(2.5, 4);
      op.paradoAte = t + 1500;
      cooldownColisao = t + 3000;
      colisoesOponente++;
      evento(t, "COLISAO com o oponente");
    }
  }
  // o carro encostou/passou por cima de um cubo? (regras 7: nao pode empurrar nem deslocar cubo: -5 e o juiz o recoloca)
  {
    const double sx = e.x + cfg.usFrenteMm * cos(e.th), sy = e.y + cfg.usFrenteMm * sin(e.th);
    for (Cubo& cu : cubos)
      if (cu.vivo && t >= cu.tocadoAte && hypot(cu.x - sx, cu.y - sy) < 38.0) {
        cu.tocadoAte = t + 4000;
        toquesEmCubos++;
        toquesPendentes++;
        evento(t, fmt("PASSOU POR CIMA/ENCOSTOU num cubo (cor %d) em (%.0f,%.0f): -5 (o juiz o recoloca)", static_cast<int>(cu.cor), cu.x, cu.y));
        if (getenv("SIM_DEBUG")) fprintf(stderr, "    TOQUE t=%.1f eixo (%.0f,%.0f) th %.0f ponta (%.0f,%.0f) cubo (%.0f,%.0f)\n", t / 1000.0, e.x, e.y, e.th * 57.3, sx, sy, cu.x, cu.y);
      }
  }
  if (!ruidoLigado) return;
  // --- ruido aleatorio: cada evento entra no log (a pagina mostra) e a mancha e desenhada. Cada tipo tem o seu relogio.
  for (Mancha& m : manchas)
    if (m.ate == static_cast<size_t>(-1) && t >= m.expiraMs) m.ate = c.quadro();
  // Dono unico da INTENSIDADE do ruido: 1,0 era o ruido original (muito forte); 0,5 = metade da frequencia, da duracao e do desvio de cada evento.
  constexpr double kEsc = kEscalaRuido;
  if (!assimetriaSorteada) {   // um motor sempre um pouco mais forte que o outro (3 a 10%): vale a partida inteira, e o carro puxa para um lado mesmo em reta
    assimetriaSorteada = true;
    const double a = (getenv("SIM_SEM_ASSIM") ? 0.0 : 1.0) * (rnd() < 0.5 ? -1.0 : 1.0) * (0.03 + 0.07 * rnd()) * kEsc;
    c.assimetria(1.0 - a / 2.0, 1.0 + a / 2.0);
    evento(t, fmt("RUIDO: ASSIMETRIA FIXA dos motores: o %s e %.0f%% mais forte (a partida inteira)", a > 0 ? "direito" : "esquerdo", fabs(a) * 100));
  }
  auto agenda = [&](int k, unsigned long de, unsigned long ate) { prox[k] = t + static_cast<unsigned long>((de + rnd() * (ate - de)) / kEsc); if (getenv("SIM_SEM_RUIDO") && (atoi(getenv("SIM_SEM_RUIDO")) >> k & 1)) prox[k] = ~0UL; };   // menos ruido = eventos mais raros
  if (t >= prox[0]) {   // mancha de sujeira: parece linha
    const double dist = 200 + rnd() * 150, lat = (rnd() - 0.5) * 80;
    Mancha m;
    m.x = e.x + dist * cos(e.th) - lat * sin(e.th);
    m.y = e.y + dist * sin(e.th) + lat * cos(e.th);
    m.raio = 7 + rnd() * 6;
    m.desde = c.quadro();
    m.expiraMs = t + 25000;
    manchas.push_back(m);
    evento(t, fmt("RUIDO: mancha de SUJEIRA de %.0f mm a %.0f mm a frente (parece linha para a barra)", m.raio * 2, dist));
    agenda(0, 15000, 35000);
  }
  if (t >= prox[1]) {   // patinagem
    const double f = 1.0 + (0.75 + 0.35 * rnd() - 1.0) * kEsc;
    c.fatorRodas(f);
    fimPatina = t + static_cast<unsigned long>(600 * kEsc);
    evento(t, fmt("RUIDO: PATINAGEM, rodas a %.0f%% por 0,6 s", f * 100));
    agenda(1, 12000, 24000);
  }
  if (fimPatina && t >= fimPatina) { c.fatorRodas(1.0); fimPatina = 0; }
  if (t >= prox[2]) {   // sensores RC atrasados
    const int ticks = std::max(1, static_cast<int>((2 + static_cast<int>(rnd() * 4)) * kEsc));
    c.atrasoLeitura(ticks);
    fimAtraso = t + 1000;
    evento(t, fmt("RUIDO: sensores ATRASADOS %d ms por 1 s", ticks * 5));
    agenda(2, 20000, 35000);
  }
  if (fimAtraso && t >= fimAtraso) { c.atrasoLeitura(0); fimAtraso = 0; }
  if (t >= prox[3]) {   // um sensor RC preso
    const int i = static_cast<int>(rnd() * 8) % 8;
    const bool preto = rnd() < 0.5;
    c.prenderSensor(i, preto, t + static_cast<unsigned long>(1500 * kEsc));
    evento(t, fmt("RUIDO: sensor S%d PRESO em %s por 1,5 s", i + 1, preto ? "preto" : "branco"));
    agenda(3, 25000, 45000);
  }
  if (t >= prox[4]) {   // ultrassom: eco falso ou perda de eco
    const bool perda = rnd() < 0.5;
    const double mm = perda ? -1.0 : 80 + rnd() * 220;
    c.ultrassomFalso(t + static_cast<unsigned long>(400 * kEsc), mm);
    evento(t, perda ? "RUIDO: ULTRASSOM perdeu o eco por 0,4 s" : fmt("RUIDO: ULTRASSOM deu um eco FALSO de %.0f mm por 0,4 s", mm));
    agenda(4, 18000, 32000);
  }
  if (t >= prox[5]) {   // um motor mais fraco
    const bool esq = rnd() < 0.5;
    const double f = 1.0 - (1.0 - (0.86 + 0.08 * rnd())) * kEsc;
    c.fatorLados(esq ? f : 1.0, esq ? 1.0 : f);
    fimMotor = t + static_cast<unsigned long>(2000 * kEsc);
    evento(t, fmt("RUIDO: MOTOR %s a %.0f%% por 2 s (o carro puxa para um lado)", esq ? "esquerdo" : "direito", f * 100));
    agenda(5, 25000, 40000);
  }
  if (fimMotor && t >= fimMotor) { c.fatorLados(1.0, 1.0); fimMotor = 0; }
  if (t >= prox[6]) {   // queda de tensao da bateria
    c.fatorRodas(1.0 - 0.15 * kEsc);
    fimPatina = t + static_cast<unsigned long>(4000 * kEsc);
    evento(t, "RUIDO: BATERIA caiu: todas as rodas a 85% por 4 s");
    agenda(6, 40000, 70000);
  }
  if (t >= prox[7]) {   // o vies do giroscopio anda
    const double d = (rnd() - 0.5) * 3.0 * kEsc;
    c.saltarVies(d);
    evento(t, fmt("RUIDO: o VIES do giroscopio andou %+.1f graus/s (o rumo vai derivar ate a ancora na grade corrigir)", d));
    agenda(7, 45000, 80000);
  }
  if (t >= prox[8]) {   // solavanco
    const double g = (rnd() - 0.5) * 14.0 * kEsc;
    c.chutar(g);
    evento(t, fmt("RUIDO: SOLAVANCO girou o carro %+.0f graus sem comando", g));
    agenda(8, 30000, 55000);
  }
  if (t >= prox[9]) {   // desgaste branco na fita: a linha some por um trecho
    const double dist = 180 + rnd() * 120;
    Mancha m;
    m.x = e.x + dist * cos(e.th);
    m.y = e.y + dist * sin(e.th);
    m.raio = 14 + rnd() * 6;
    m.branca = true;
    m.desde = c.quadro();
    m.expiraMs = t + 25000;
    manchas.push_back(m);
    evento(t, fmt("RUIDO: DESGASTE na fita de %.0f mm a %.0f mm a frente (a linha some)", m.raio * 2, dist));
    agenda(9, 30000, 55000);
  }
}

// ---------------------------------------------------------------- firmware simulado (espelha SeguidorBasico.ino)

// ---------------------------------------------------------------- o NAVEGADOR (usa o core: localizador, mapa de cubos, exploracao). Ainda so no simulador.
// Quem orquestra e o core (core/navigator.h); aqui so se ligam os eventos do robo (portao, cruzamento, giro, borda, reposicionado, cubo) ao core.
// ---------------------------------------------------------------- o firmware simulado: a logica de verdade (src/seguidor/robo.h) sobre o Carro simulado
inline ConfigRobo configDoSim() {
  ConfigRobo c;
  c.dtMs = gFis.dtMs;
  c.tauMs = gFis.tauMs;
  c.encoder = gFis.encoder;
  c.rumoPeloEncoder = gRumoPeloEncoder;
  c.autoCalBitola = gAutoCalBitola;
  c.gestoGraus = gGestoGraus;
  c.ajustes = gAjustes;
  return c;
}
struct Firmware : Robo<Carro> {
  Firmware() : Robo<Carro>(configDoSim()) {}
};

// ---------------------------------------------------------------- DOIS ROBOS IGUAIS na mesma arena (o oponente e outro robo com o MESMO firmware)
// Cada robo roda a sua logica numa thread e entrega a vez a cada ciclo de 5 ms (tick), entao os dois andam juntos sem mexer no firmware. O oponente usa a arena no
// referencial DELE (comeca no verde da esquerda, a mesma pista): o mundo real e o dele girado 180 graus em torno do centro. Os cubos sao os mesmos (copias espelhadas,
// sincronizadas a cada ciclo). Colisao (regras 7.1): quem estava avancando contra o outro leva -10 e volta ao ponto de partida (verde); o atingido e recolocado onde estava.
static Pista tabuleiro();
static void configurarMissao(Mundo& m, Firmware& fw, Carro& c, int ncubos, bool oponente, bool ruido);
static Estado espelho(const Estado& e) { return {kLarg - e.x, kAlt - e.y, e.th + M_PI}; }

struct Agente {
  std::string nome;
  Pista* pista = nullptr;
  Mundo mundo;
  std::unique_ptr<Carro> carro;
  std::unique_ptr<Firmware> fw;
  std::thread th;
  std::mutex mu;
  std::condition_variable cv;
  bool vez = false, morto = false;

  void iniciar() {
    fw->ceder = [this] {
      std::unique_lock<std::mutex> l(mu);
      vez = false;
      cv.notify_all();
      cv.wait(l, [this] { return vez || morto; });
    };
    th = std::thread([this] {
      { std::unique_lock<std::mutex> l(mu); cv.wait(l, [this] { return vez || morto; }); }
      while (!morto) fw->passo(*carro);
    });
  }
  void retomar() {   // roda ate o proximo tick
    std::unique_lock<std::mutex> l(mu);
    vez = true;
    cv.notify_all();
    cv.wait(l, [this] { return !vez || morto; });
  }
  ~Agente() {
    { std::lock_guard<std::mutex> l(mu); morto = true; vez = true; }
    cv.notify_all();
    if (th.joinable()) th.join();
  }
};

struct Cena {
  Pista pista = tabuleiro();
  Agente a, b;   // a = nos (lado esquerdo do mundo), b = o outro robo (lado direito, mas simulado no referencial dele)
  std::vector<char> vivoMundo;
  unsigned long cooldown = 0;
  int reinicios = 0;

  bool dois = true;
  void montar(int ncubos, bool ruido, bool comOponente = true) {
    dois = comOponente;
    a.nome = "nos"; b.nome = "oponente";
    a.pista = b.pista = &pista;
    a.mundo.seed = 4242;
    a.carro = std::make_unique<Carro>(pista, Estado{-270.0 + a.mundo.rnd() * 230.0, 60.0 + a.mundo.rnd() * (kAlt - 120.0), 0.0});   // posicao aleatoria do verde, olhando para as ruas
    b.carro = std::make_unique<Carro>(pista, Estado{-270.0 + a.mundo.rnd() * 230.0, 60.0 + a.mundo.rnd() * (kAlt - 120.0), 0.0});   // o outro: no referencial dele tambem
    a.fw = std::make_unique<Firmware>();
    configurarMissao(a.mundo, *a.fw, *a.carro, ncubos, false, ruido);
    if (!dois) { vivoMundo.assign(a.mundo.cubos.size(), 1); a.iniciar(); return; }
    b.fw = std::make_unique<Firmware>();
    // o oponente tem as proprias plataformas (cores diferentes) e os MESMOS cubos, espelhados
    using seguidor::CorCubo;
    b.mundo.porPlataformas({{0.5, CorCubo::Verde}, {1.5, CorCubo::Azul}, {2.5, CorCubo::Marrom}, {3.5, CorCubo::Vermelho}});
    for (const Cubo& cu : a.mundo.cubos) {
      Cubo e = cu;
      e.x = kLarg - cu.x; e.y = kAlt - cu.y;
      if (!cu.obstaculo) { if (cu.leste) { e.c = core::kMapCols - 2 - cu.c; e.r = core::kMapRows - 1 - cu.r; } else { e.c = core::kMapCols - 1 - cu.c; e.r = core::kMapRows - 2 - cu.r; } }
      else { e.c = 4 - cu.c; e.r = 3 - cu.r; }
      b.mundo.cubos.push_back(e);
    }
    b.mundo.ruidoLigado = ruido;
    b.mundo.seed = 9191;
    b.mundo.cfg.tcsChaoErro = ruido ? 0.02 : 0.0;
    b.carro->usarMundo(&b.mundo);
    b.fw->ligarImu(*b.carro);
    b.fw->nav.iniciar();
    b.fw->portaoPendente = true;
    a.mundo.op.ativo = b.mundo.op.ativo = true;
    a.mundo.op.externo = b.mundo.op.externo = true;
    vivoMundo.assign(a.mundo.cubos.size(), 1);
    a.iniciar();
    b.iniciar();
  }

  // Um ciclo de 5 ms para os dois.
  void ciclo() {
    a.retomar();
    if (dois) b.retomar();
    sincronizar();
  }

  // Regras 7 e 8: sair da arena, passar da linha vermelha ou da azul do OUTRO lado = robo perdido: -5 e o capitao o recoloca na area livre (verde).
  // Cada robo e testado no proprio referencial (a arena e a mesma): permitido x de -662 a kLarg (a azul do outro lado) e y de 0 a kAlt (as vermelhas).
  void conferirLimites(Agente& g) {
    const Estado e = g.carro->estado();
    if (e.x < -682.0 || e.x > kLarg + 20.0 || e.y < -20.0 || e.y > kAlt + 20.0) {
      g.mundo.evento(g.carro->agoraMs(), fmt("SAIU DA ARENA em (%.0f,%.0f) [fazendo: %s%s]", e.x, e.y, g.fw->nav.ultimoPlano.c_str(), g.fw->nav.entregando ? ", entregando" : ""));
      reiniciarNoVerde(g, "saiu da arena (robo perdido)", 5);
    }
  }

  static double fechamento(const Estado& de, double vDe, const Estado& para) {   // velocidade com que `de` se aproxima de `para` (mm/s)
    const double ang = atan2(para.y - de.y, para.x - de.x);
    return vDe * cos(remainder(ang - de.th, 2 * M_PI));
  }

  void reiniciarNoVerde(Agente& g, const char* motivo, int penalidade = 10) {
    g.carro->colocarEm({-270.0 + g.mundo.rnd() * 230.0, 60.0 + g.mundo.rnd() * (kAlt - 120.0), 0.0});   // posicao ALEATORIA do verde, olhando para as ruas
    g.carro->parar();
    g.fw->nav.reposicionado();
    g.fw->precisaAlinhar = true;
    g.fw->nav.pontos += 15;   // reposicionado() ja cobra o reinicio (-15); a colisao cobra os seus -10 (regras 7.1), nao os dois
    g.fw->nav.pontos -= penalidade;
    g.fw->reposicionados++;
    g.fw->rumo = 0.0f;
    g.fw->memoria = Memoria{};
    g.fw->cruz = MemoriaCruzamento{};
    esquecerCor(g.fw->memCor);
    g.fw->portaoPendente = true;
    g.mundo.carga0 = g.mundo.carga1 = 0;
    g.mundo.evento(g.carro->agoraMs(), fmt("%s: voltou ao ponto de partida (verde), -%d", motivo, penalidade));
  }

  void sincronizar() {
    conferirLimites(a);
    if (!dois) return;
    conferirLimites(b);
    // cubos
    for (size_t i = 0; i < vivoMundo.size(); i++) {
      if (!vivoMundo[i]) continue;
      if (!a.mundo.cubos[i].vivo) { vivoMundo[i] = 0; b.mundo.cubos[i].vivo = false; b.mundo.cubos[i].mortoEm = b.carro->quadro(); }
      else if (!b.mundo.cubos[i].vivo) {
        vivoMundo[i] = 0;
        a.mundo.cubos[i].vivo = false;
        a.mundo.cubos[i].mortoEm = a.carro->quadro();
        a.mundo.pegosPeloOponente++;
        a.mundo.evento(a.carro->agoraMs(), fmt("o OPONENTE coletou um cubo da rua (%d,%d)", a.mundo.cubos[i].c, a.mundo.cubos[i].r));
      }
    }
    // um enxerga o outro como obstaculo (ultrassom) e como alvo que se mexe
    const Estado ea = a.carro->estado(), eb = espelho(b.carro->estado());
    a.mundo.op.x = eb.x; a.mundo.op.y = eb.y; a.mundo.op.th = eb.th;
    b.mundo.op.x = kLarg - ea.x; b.mundo.op.y = kAlt - ea.y; b.mundo.op.th = ea.th + M_PI;
    // colisao
    const unsigned long t = a.carro->agoraMs();
    if (hypot(ea.x - eb.x, ea.y - eb.y) < 170.0 && t >= cooldown) {
      cooldown = t + 3000;
      const double fa = fechamento(ea, a.carro->velMmS(), eb), fb = fechamento(eb, b.carro->velMmS(), ea);
      a.carro->bater(2.5, 4);
      b.carro->bater(2.5, 4);
      reinicios++;
      Agente& culpado = fa >= fb ? a : b;
      Agente& vitima = fa >= fb ? b : a;
      a.mundo.evento(t, fmt("COLISAO entre os robos: %s vinha avancando mais (%.0f contra %.0f mm/s)", culpado.nome.c_str(), std::max(fa, fb), std::min(fa, fb)));
      reiniciarNoVerde(culpado, "colisao (regras 7.1)");
      reiniciarNoVerde(vitima, "colisao: o outro tambem volta ao verde", 0);   // colisao: OS DOIS voltam ao verde (so quem avancava mais perde pontos)
    }
  }
};

// ---------------------------------------------------------------- cenarios

struct Resultado {
  std::string nome;
  bool ok;
  std::string info;
  bool informativo = false;   // limite conhecido do projeto: mostra, mas nao derruba o exit code
};

static const char* gSvgDir = nullptr;
static unsigned long long gSemente = 0;   // somada a semente do mundo: --lote varia o sorteio de cubos/ruido
static int gCapacidade = 1;   // cubos por viagem (param cap=; o firmware escolheu 1, as regras deixam 2)
static struct { bool fim = false; int entregues = 0, pontos = 0, toques = 0; double tempoS = 0; } gUltimaMissao;   // resultado da ultima cenarioMissao (para --lote)

// Placar de qualidade do seguimento (reta, cantos, arcos): para comparar uma mudanca na decisao antes e depois.
static struct { int n = 0; double tempoS = 0, rms = 0, pior = 0; } gPlacar;
static void placar(const Carro& c) {
  gPlacar.n++;
  gPlacar.tempoS += c.agoraMs() / 1000.0;
  gPlacar.rms += c.erroRms();
  gPlacar.pior = std::max(gPlacar.pior, c.erroMax());
}

static void svg(const std::string& nome, const Pista& p, const Carro& c, int = -1) {
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

// Assimetria dos motores: com o MESMO PWM nos dois lados o carro puxa para um lado; o giroscopio em reta mede e o lado fraco ganha forca. Compara o desvio de rumo numa reta
// de 2 s sem e com a compensacao, e confere os ganhos (o lado fraco sobe quase o que faltava; o forte fica em 1).
static Resultado cenarioAssimetria(double assim) {   // assim > 0: direita mais forte (o carro puxa para a esquerda)
  Pista vazia;
  auto desvioReta = [&](bool compensar) {
    Carro c(vazia, norte(0, 0), gFis);
    c.assimetria(1.0 - assim / 2.0, 1.0 + assim / 2.0);
    Firmware fw;
    fw.ligarImu(c);
    if (compensar) fw.calibrarAssimetria(c);
    const double th0 = c.estado().th;
    for (int i = 0; i < 400; i++) { c.comandar(fw.ajustes.velReto, fw.ajustes.velReto); fw.tick(c); }   // 2 s
    c.parar();
    return std::make_pair(remainder(c.estado().th - th0, 2 * M_PI) * 57.29578, fw.assimMpu);
  };
  const auto sem = desvioReta(false), com = desvioReta(true);
  const double ganho = assim > 0 ? com.second.ganhoEsq : com.second.ganhoDir, outro = assim > 0 ? com.second.ganhoDir : com.second.ganhoEsq;
  const double razaoDir = (1.0 + assim / 2.0) / (1.0 - assim / 2.0);
  const double verdade = assim > 0 ? razaoDir : 1.0 / razaoDir;   // quanto o lado fraco precisa subir
  const bool ok = fabs(sem.first) > 5.0 && fabs(com.first) < 0.25 * fabs(sem.first) && fabs(ganho - verdade) < 0.15 * (verdade - 1.0) + 0.002 && outro == 1.0;
  return {fmt("assimetria dos motores (%+.0f%%)", assim * 100), ok, fmt("desvio em 2 s: %.1f graus sem, %.1f com; ganho do lado fraco x%.3f (real x%.3f), do forte x%.3f", sem.first, com.first, ganho, verdade, outro)};
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

// ---- O tabuleiro inteiro (docs/2026-10-28_mapa-comparacao-e-localizacao.md): 6 colunas x 5 linhas de cruzamentos a 400 mm, becos de 127 mm ate a borda,
// linha azul nos lados (x = 0 e kLarg), vermelha em cima e embaixo (y = 0 e kAlt) e em volta, verde (371 mm) e amarelo (291 mm) nas pontas, cubos nos quarteiroes.

static Pista tabuleiro() {
  Pista p;
  p.decor.push_back({-662, 0, -371, kAlt, 1});             // amarelo (ecoponto)
  p.decor.push_back({-371, 0, 0, kAlt, 0});                // verde
  p.decor.push_back({kLarg, 0, kLarg + 371, kAlt, 0});
  p.decor.push_back({kLarg + 371, 0, kLarg + 662, kAlt, 1});
  for (int c = 0; c < 5; c++)
    for (int r = 0; r < 4; r++) {
      const double cx = kBeco + c * kPasso + kPasso / 2, cy = kBeco + r * kPasso + kPasso / 2;
      p.decor.push_back({cx - 15, cy - 15, cx + 15, cy + 15, 2});   // marrom: ponto de cubo
    }
  for (int r = 0; r < 5; r++) p.reta(0, kBeco + r * kPasso, kLarg, kBeco + r * kPasso);          // ruas horizontais: de azul a azul
  for (int c = 0; c < 6; c++) p.reta(kBeco + c * kPasso, 0, kBeco + c * kPasso, kAlt);          // ruas verticais: de vermelho a vermelho
  p.reta(0, 0, 0, kAlt, kAzul);
  p.reta(kLarg, 0, kLarg, kAlt, kAzul);
  p.reta(-662, 0, kLarg + 662, 0, kVermelha);
  p.reta(-662, kAlt, kLarg + 662, kAlt, kVermelha);
  p.reta(-662, 0, -662, kAlt, kVermelha);
  p.reta(kLarg + 662, 0, kLarg + 662, kAlt, kVermelha);
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

// Quanto a memoria do carrinho bate com a realidade: para cada rua, ele acha que tem cubo (P > 0,5) e tem de fato?
struct PrecisaoMemoria { int certo = 0, falsoPositivo = 0, perdeu = 0, total = 0; };
static PrecisaoMemoria precisaoMemoria(const Mundo& m, const core::CubeBelief& b) {
  PrecisaoMemoria r;
  for (int c = 0; c < core::kMapCols; c++)
    for (int rr = 0; rr < core::kMapRows; rr++)
      for (int leste = 0; leste < 2; leste++) {
        if (leste ? c + 1 >= core::kMapCols : rr + 1 >= core::kMapRows) continue;
        bool tem = false;
        for (const Cubo& cu : m.cubos) tem |= cu.vivo && cu.c == c && cu.r == rr && cu.leste == (leste == 1);
        const float p = leste ? b.east[c][rr] : b.north[c][rr];
        r.total++;
        if (tem && p > 0.5f) r.certo++;
        else if (!tem && p > 0.5f) r.falsoPositivo++;
        else if (tem) r.perdeu++;
      }
  return r;
}

// Monta a missao no tabuleiro: cubos de varias cores nas ruas, oponente andando, ruido (sinalizado) opcional, e o navegador ligado.
static void configurarMissao(Mundo& m, Firmware& fw, Carro& c, int ncubos, bool oponente, bool ruido) {
  using seguidor::CorCubo;
  m.seed += gSemente * 7919ULL;
  // Plataformas do nosso ecoponto: o juiz poe de 1 a 5, cores distintas (aqui azul, vermelho, amarelo e preto, em marcas vizinhas); verde e marrom ficam SEM plataforma e valem como branco
  m.porPlataformas({{0.5, CorCubo::Azul}, {1.5, CorCubo::Vermelho}, {2.5, CorCubo::Amarelo}, {3.5, CorCubo::Preto}});   // 4 plataformas CENTRADAS entre as bordas, equidistantes (260 mm)
  m.semearRegras(ncubos);
  m.semearObstaculos(3);
  m.op.ativo = oponente;
  m.op.c = m.op.alvoC = core::kMapCols - 1;
  m.op.r = m.op.alvoR = core::kMapRows - 1;
  m.op.x = xDoCruzamento(m.op.c);
  m.op.y = yDoCruzamento(m.op.r);
  m.ruidoLigado = ruido;
  m.cfg.tcsChaoErro = ruido ? 0.02 : 0.0;
  c.usarMundo(&m);
  fw.ligarImu(c);
  fw.nav.iniciar();
  fw.nav.coleta.capacidade = gCapacidade;
  fw.portaoPendente = true;
}

// A missao inteira: sai do verde, acha o portao, localiza-se, explora e coleta cubos (so os coletaveis), entrega no verde, repete. Roda ate entregar `meta` cubos.
static Resultado cenarioMissao(bool ruido, int meta) {
  Pista p = tabuleiro();
  Mundo m;
  Carro c(p, {-140, yDoCruzamento(2), 0.0});
  Firmware fw;
  configurarMissao(m, fw, c, 14, false, ruido);
  fw.nav.verdade = &m;
  const bool fim = rodar(fw, c, [&](const Carro&) { return fw.nav.entregues >= meta; });
  svg(ruido ? "missao_com_ruido" : "missao", p, c);
  int visitados = 0;
  int visitadosLeste = 0;
  for (int cc = 0; cc < core::kMapCols; cc++) for (int rr = 0; rr < core::kMapRows; rr++) { visitados += fw.nav.visitas.n[cc][rr] > 0 ? 1 : 0; if (cc >= 3) visitadosLeste += fw.nav.visitas.n[cc][rr] > 0 ? 1 : 0; }
  if (getenv("SIM_DEBUG")) {
    for (const std::string& l : m.log) fprintf(stderr, "    %s\n", l.c_str());
    const Estado e = c.estado();
    fprintf(stderr, "    FIM: robo (%.0f,%.0f) th %.0f | oponente (%.0f,%.0f) | carregando %d | pose (%d,%d) conf %.2f | levantado %d parado %d buscando ciclos %d\n", e.x, e.y, e.th * 57.3, m.op.x, m.op.y, fw.nav.carregando, core::estimate(fw.nav.loc).pose.at.c, core::estimate(fw.nav.loc).pose.at.r, core::estimate(fw.nav.loc).confidence, c.levantado(), fw.parado, fw.ticksBuscando);
  }
  if (getenv("SIM_DEBUG")) {   // quanto tempo anda perto das molduras vermelhas (y ~ 0 ou kAlt, ou x < -440) e onde
    size_t perto = 0, topo = 0, base = 0, esq = 0;
    for (const Estado& e : c.rastro()) { const bool t = e.y > kAlt - 90, b = e.y < 90, l = e.x < -430; topo += t; base += b; esq += l; perto += t || b || l; }
    fprintf(stderr, "    VERMELHO: %.0f%% do tempo perto das molduras (topo %.0f%%, base %.0f%%, esquerda %.0f%%) de %.0f s\n", 100.0 * perto / c.rastro().size(), 100.0 * topo / c.rastro().size(), 100.0 * base / c.rastro().size(), 100.0 * esq / c.rastro().size(), c.agoraMs() / 1000.0);
  }
  gUltimaMissao = {fim, fw.nav.entregues, fw.nav.pontos, m.toquesEmCubos, c.agoraMs() / 1000.0};
  const PrecisaoMemoria pm = precisaoMemoria(m, fw.nav.cubos);
  if (getenv("SIM_DEBUG")) fprintf(stderr, "    MEMORIA: acertou %d, achou cubo onde nao ha %d, nao lembra de %d (de %d ruas)\n", pm.certo, pm.falsoPositivo, pm.perdeu, pm.total);
  return {ruido ? "missao completa com ruido: explora, coleta, entrega" : "missao completa: explora, coleta, entrega", fim,
          fmt("%s em %.0f s | PONTOS %d | coletou %d, entregou %d, brancos identificados %d, bloqueou %d rua(s) | cruzamentos %d (falsos %d), reposicoes %d | visitou %d de 30 cruzamentos (%d de 15 do lado do oponente, colunas 3 a 5) | velocidade medida pela distancia entre cruzamentos %.0f mm/s (%d amostras; real a 110 de PWM: %.0f) | varreduras puladas %d, cubos vistos ao longe %d | oponente levou %d cubo(s), colisoes %d",
              fim ? "entregou a meta" : "NAO entregou a meta", c.agoraMs() / 1000.0, fw.nav.pontos, fw.nav.coletados, fw.nav.entregues, fw.nav.brancosIdentificados, fw.nav.bloqueios, fw.cruzamentos, fw.falsosAlarmes, fw.nav.reposicoes, visitados, visitadosLeste, fw.ajC.velMmS, fw.amostrasVel, c.velNominalMmS(fw.ajustes.velReto), fw.semVarreduras, fw.nav.ecosLongeVistos, m.pegosPeloOponente, m.colisoesOponente)};
}

// Mede a bitola EFETIVA (a que converte diferenca de rodas em graus, ja com a derrapagem): com o eixo sobre o centro de um cruzamento, gira mais de uma volta e anota a diferenca de
// percurso das rodas (encoder) a cada vez que os sensores do centro veem uma rua (4 por volta). O MESMO braco visto duas vezes (5 ruas depois) e exatamente 360 graus, e o erro de o eixo
// nao estar no centro (cada braco desloca o meio da faixa) se cancela. bitola efetiva = diferenca / 2 pi. Devolve < 0 se nao viu 5 ruas.
static double medirBitolaEfetiva(Carro& c, int velPwm, double* graus360Mm = nullptr) {
  c.comandar(-velPwm, velPwm);   // anti-horario
  std::vector<double> meios;
  bool dentro = false;
  double entrada = 0.0;
  const unsigned long inicio = c.agoraMs();
  while (c.agoraMs() - inicio < 30000 && meios.size() < 6) {
    c.avancar();
    bool p[kSensores];
    c.ler(p);
    const double dif = c.odoDMm() - c.odoEMm();   // mm de giro acumulados (o encoder le o eixo da roda)
    const bool centro = p[3] && p[4];
    if (!dentro && centro) { dentro = true; entrada = dif; }
    else if (dentro && !centro) { dentro = false; meios.push_back((entrada + dif) / 2.0); }
  }
  c.parar();
  if (meios.size() < 5) return -1.0;
  const double volta = meios[4] - meios[0];   // mesmo braco, uma volta depois
  if (graus360Mm) *graus360Mm = volta;
  return volta / (2.0 * M_PI);
}

// Coleta de N cubos entregaveis (cores que tem plataforma: azul, vermelho, amarelo, preto), sem ruido e sem oponente, com o teto de 12 min da regra. Devolve o instante de cada
// entrega (s); entregas.size() < n se o tempo acabou antes.
static std::vector<double> coletaDeN(int n, unsigned long semente) {
  using seguidor::CorCubo;
  Pista p = tabuleiro();
  Mundo m;
  Carro c(p, {-140, yDoCruzamento(2), 0.0});
  Firmware fw;
  gSemente = semente;
  configurarMissao(m, fw, c, 0, false, false);
  fw.nav.verdade = &m;
  const CorCubo cores[4] = {CorCubo::Azul, CorCubo::Vermelho, CorCubo::Amarelo, CorCubo::Preto};
  std::vector<CorCubo> lista;
  for (int i = 0; i < n; i++) lista.push_back(cores[i % 4]);
  m.semearCubos(n, lista);
  std::vector<double> entregas;
  constexpr unsigned long kPartidaMs = 720000;   // 12 min (regras)
  while (c.agoraMs() < kPartidaMs && fw.nav.entregues < n) {
    fw.passo(c);
    while (static_cast<int>(entregas.size()) < fw.nav.entregues) entregas.push_back(c.agoraMs() / 1000.0);
  }
  if (getenv("SIM_DEBUG")) { for (const std::string& l : m.log) fprintf(stderr, "    %s\n", l.c_str()); const Estado e = c.estado(); fprintf(stderr, "    FIM coleta: robo (%.0f,%.0f) th %.0f carregando %d entregues %d coletados %d\n", e.x, e.y, e.th * 57.3, fw.nav.carregando, fw.nav.entregues, fw.nav.coletados); }
  return entregas;
}

// Dois robos IGUAIS na mesma arena (regras): cada um com o mesmo firmware; colisao leva -10 e volta ao ponto de partida (regras 7.1). Roda ate os dois terem entregue `meta`
// cubos (ou o teto de seguranca).
static Resultado cenarioDuasEquipes(bool ruido, int meta) {
  auto cena = std::make_unique<Cena>();
  cena->montar(18, ruido);
  const unsigned long inicio = cena->a.carro->agoraMs();
  bool fim = false;
  while (cena->a.carro->agoraMs() - inicio < kTetoSegurancaMs && !(fim = cena->a.fw->nav.entregues >= meta && cena->b.fw->nav.entregues >= meta)) cena->ciclo();
  if (getenv("SIM_DEBUG")) for (const std::string& l : cena->a.mundo.log) fprintf(stderr, "    NOS %s\n", l.c_str());
  const Firmware& fa = *cena->a.fw;
  const Firmware& fb = *cena->b.fw;
  return {ruido ? "dois robos iguais, com ruido" : "dois robos iguais na arena", fim,
          fmt("%s em %.0f s | NOS: %d pontos (coletou %d, entregou %d, brancos %d) | OPONENTE: %d pontos (coletou %d, entregou %d, brancos %d) | colisoes %d",
              fim ? "os dois entregaram a meta" : "meta nao atingida", cena->a.carro->agoraMs() / 1000.0, fa.nav.pontos, fa.nav.coletados, fa.nav.entregues, fa.nav.brancosIdentificados,
              fb.nav.pontos, fb.nav.coletados, fb.nav.entregues, fb.nav.brancosIdentificados, cena->reinicios)};
}

// A linha vermelha NUNCA e rota: a rua preta chega na moldura vermelha em angulo e, se a barra a lesse como preto e a seguisse, o carro correria pela moldura.
static Resultado cenarioVermelhoDeLado() {
  Pista p;
  p.reta(0, 0, 400, 400);                          // rua preta a 45 graus
  p.reta(400, 400, 1400, 400, kVermelha);          // moldura vermelha: a rua morre nela
  p.reta(0, 400, 400, 400);                         // (um pedaco preto antes, para a rua nao acabar em ponta)
  Carro c(p, {20, 20, 45.0 * M_PI / 180.0});
  Firmware fw;
  fw.ligarImu(c);
  double maxX = 0;
  rodar(fw, c, [&](const Carro& k) { maxX = std::max(maxX, k.estado().x); return fw.meiasVoltas >= 1 || k.estado().x > 900; });
  svg("vermelho_de_lado", p, c);
  const bool ok = fw.meiasVoltas >= 1 && maxX < 520.0;
  return {"linha vermelha a 45 graus: nao segue pela moldura", ok, fmt("%s | meias-voltas %d | foi ate x = %.0f mm (a rua acaba em 400; seguir a moldura passaria de 900)", ok ? "parou e deu meia-volta" : "SEGUIU a moldura vermelha", fw.meiasVoltas, maxX)};
}

// Acabaram os cubos do nosso lado: tem de EXPLORAR O OUTRO LADO (colunas 3 a 5, perto do oponente), onde ainda ha cubos que ele nao conhece.
static Resultado cenarioExplorarOutroLado() {
  using seguidor::CorCubo;
  auto cena = std::make_unique<Cena>();
  cena->montar(0, false, false);
  Mundo& m = cena->a.mundo;
  m.cubos.clear();
  m.porCubo(0, 1, true, CorCubo::Azul);       // dois cubos perto do portao...
  m.porCubo(1, 3, false, CorCubo::Vermelho);
  m.porCubo(4, 1, true, CorCubo::Azul);       // ...e dois do OUTRO lado, que ele nao conhece
  m.porCubo(4, 3, false, CorCubo::Vermelho);
  cena->vivoMundo.assign(m.cubos.size(), 1);
  cena->a.fw->nav.cubos = cena->a.fw->nav.cubos;   // memoria comeca no prior (0,4)
  Firmware& fw = *cena->a.fw;
  const unsigned long inicio = cena->a.carro->agoraMs();
  bool fim = false;
  while (cena->a.carro->agoraMs() - inicio < kTetoSegurancaMs && !(fim = fw.nav.entregues >= 3)) cena->ciclo();
  int leste = 0, total = 0;
  for (int cc = 0; cc < core::kMapCols; cc++) for (int rr = 0; rr < core::kMapRows; rr++) { total += fw.nav.visitas.n[cc][rr] > 0; if (cc >= 3) leste += fw.nav.visitas.n[cc][rr] > 0; }
  if (getenv("SIM_DEBUG")) {
    for (const std::string& l : m.log) fprintf(stderr, "    OUTRO %s\n", l.c_str());
    const auto& rs = cena->a.carro->rastro();
    for (size_t i = 5600; i < 6300 && i < rs.size(); i++) if (fabs(rs[i].x) < 40) fprintf(stderr, "    X0 t=%.2fs pos (%.0f,%.0f) th %.0f bits %02x\n", i * 0.02, rs[i].x, rs[i].y, rs[i].th * 57.3, cena->a.carro->bits()[i]);
  }
  return {"acabaram os cubos do nosso lado: explora o outro", fim,
          fmt("%s em %.0f s | entregou %d (precisa de 3: so dois cubos estao do nosso lado) | visitou %d cruzamentos, %d do lado do oponente | PONTOS %d", fim ? "foi ao outro lado e entregou" : "NAO foi ao outro lado", cena->a.carro->agoraMs() / 1000.0, fw.nav.entregues, total, leste, fw.nav.pontos)};
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
// Liga o MPU, mede viés e calibra os motores (zona morta e graus por segundo na velocidade RAPIDA de busca) e volta o robo onde estava, com o rastro limpo.
static void prepararMpu(Firmware& fw, Carro& c, const Estado& origem) {
  fw.ligarImu(c);
  Calibracao cal;
  AjustesCalibracao ac;
  bool fim = false;
  for (int i = 0; i < 6000 && !fim; i++) {
    const ComandoCalibracao k = passoCalibracao(cal, fw.ultimoImu.gzRadS - fw.vies, c.agoraMs(), ac);
    fim = k.terminou;
    c.comandar(k.esq, k.dir);
    fw.tick(c);
  }
  if (cal.fase == Calibracao::Pronto) {
    fw.ajustes.zonaMortaPwm = cal.zonaMortaPwm;
    fw.ajustes.velBuscaExterna = cal.velMedida;
    fw.ajustes.grausPorSegundoBusca = cal.grausPorSegundo;
    fw.ajustes = normalizar(fw.ajustes);
  }
  c.colocarEm(origem);
  c.limparRastro();   // a animacao mostra so o que vem depois da calibracao
}

static bool buscar(double dist, double rumoGraus, bool comMpu, unsigned long& ms, int& ciclos, const Pista& p, Carro** saida = nullptr) {
  // `dist` = distancia do CENTRO DA BARRA (nao do eixo) ate a linha: assim a barra nunca comeca em cima dela, qualquer que seja o rumo
  const double th = rumoGraus * M_PI / 180.0;
  const Estado origem = {dist - 30.0 * cos(th), 500 - 30.0 * sin(th), th};
  static Carro* ultimo = nullptr;
  delete ultimo;
  Carro* c = ultimo = new Carro(p, origem);
  Firmware fw;
  if (comMpu) prepararMpu(fw, *c, origem);
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

// ================================================================ SIMULACAO AO VIVO (--servir): sem duracao gravada
// O simulador vira um servidor HTTP local: a pagina pede quadros (20 ms de simulacao cada) e o servidor SIMULA SO O QUE FOI PEDIDO, sem fim. Da para
// clicar no mapa para soltar o robo em qualquer lugar (a busca roda ao vivo), levantar/soltar o robo e dar uma pancada. Usa a MESMA logica C++ dos testes.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <functional>
#include <map>
#include <memory>

struct Sessao {
  std::string nome;
  std::shared_ptr<Pista> pista;
  std::shared_ptr<Mundo> mundo;   // cubos, oponente, ruido (so nas sessoes de missao)
  std::shared_ptr<Carro> carro;
  std::shared_ptr<Firmware> fw;
  std::shared_ptr<Cena> cena;     // dois robos iguais: a = nos, b = o outro (mundo, carro e fw acima apontam para o `a`)
  double frenteGraus = 90.0;      // para onde e "de frente" depois de levantado e solto
  // Simula ate o robo ter `quadros` quadros gravados (cada um e 20 ms).
  void avancarAte(size_t quadros) {
    if (cena) { while (carro->rastro().size() < quadros) cena->ciclo(); }
    else { while (carro->rastro().size() < quadros) fw->passo(*carro); }
  }
};

struct Receita { std::string nome; std::function<std::shared_ptr<Sessao>()> cria; };

static std::shared_ptr<Sessao> sessaoSimples(const std::string& nome, Pista&& pista, Estado e, bool mpu, double frente = 90.0) {
  auto s = std::make_shared<Sessao>();
  s->nome = nome;
  s->pista = std::make_shared<Pista>(std::move(pista));
  s->carro = std::make_shared<Carro>(*s->pista, e);
  s->fw = std::make_shared<Firmware>();
  s->frenteGraus = frente;
  if (mpu) s->fw->ligarImu(*s->carro);
  return s;
}

static std::vector<Receita> receitas() {
  std::vector<Receita> r;
  for (const int variante : {0, 1, 2, 3}) {   // bit 0 = ruido aleatorio, bit 1 = sem oponente
    const bool ruido = (variante & 1) != 0, dois = (variante & 2) == 0;
    const std::string nome = std::string("MISSAO COMPLETA") + (ruido ? " com RUIDO aleatorio (sinalizado)" : "") + (dois ? ": dois robos iguais na arena" : ": so o nosso robo (sem oponente)") + (ruido ? "" : " (explora, coleta, entrega)");
    r.push_back({nome, [nome, ruido, dois] {
      auto s = std::make_shared<Sessao>();
      s->nome = nome;
      s->frenteGraus = 0.0;
      auto cena = std::make_shared<Cena>();
      cena->montar(dois ? 18 : 14, ruido, dois);
      s->pista = std::shared_ptr<Pista>(cena, &cena->pista);
      s->mundo = std::shared_ptr<Mundo>(cena, &cena->a.mundo);
      s->carro = std::shared_ptr<Carro>(cena, cena->a.carro.get());
      s->fw = std::shared_ptr<Firmware>(cena, cena->a.fw.get());
      s->cena = cena;
      return s;
    }});
  }
  r.push_back({"Mapa completo, rota fixa (so segue a linha e vira)", [] {
    auto s = sessaoSimples("Mapa completo, rota fixa (so segue a linha e vira)", tabuleiro(), {-140, kBeco + 2 * kPasso, 0.0}, true, 0.0);
    s->fw->portaoPendente = true;
    s->fw->rota = "SLSSSS";   // depois da rota, segue reto em todo cruzamento
    return s;
  }});
  for (double dist : {30.0, 60.0, 100.0})
    for (double rumo : {90.0, 0.0, 180.0, -90.0}) {
      const std::string nome = fmt("Busca: barra a %.0f mm da linha, apontando para %.0f graus", dist, rumo);
      r.push_back({nome, [nome, dist, rumo] {
        Pista p;
        p.reta(0, -400, 0, 1400);
        const double th = rumo * M_PI / 180.0;
        const Estado origem = {dist - 30.0 * cos(th), 500 - 30.0 * sin(th), th};
        auto s = sessaoSimples(nome, std::move(p), origem, false, rumo);
        prepararMpu(*s->fw, *s->carro, origem);
        return s;
      }});
    }
  r.push_back({"Perdido de lado na linha (barra inteira sobre ela)", [] {
    Pista p;
    p.reta(0, 0, 0, 1400);
    auto s = sessaoSimples("Perdido de lado na linha (barra inteira sobre ela)", std::move(p), {-150, 300, 0.0}, true, 90.0);
    for (int i = 0; i < 140; i++) s->fw->passo(*s->carro);   // procura um pouco
    s->carro->colocarEm({-30, 500, 0.0});
    return s;
  }});
  r.push_back({"Reta, comecando torto", [] {
    Pista p;
    p.reta(0, 0, 0, 3000);
    return sessaoSimples("Reta, comecando torto", std::move(p), norte(20, 100, 15), true);
  }});
  r.push_back({"Cruzamento com beco ate a borda vermelha", [] {
    Pista p;
    p.reta(0, 0, 0, 627);
    p.reta(-400, 500, 400, 500);
    p.reta(-200, 627, 200, 627, kVermelha);
    return sessaoSimples("Cruzamento com beco ate a borda vermelha", std::move(p), norte(0, 100), true);
  }});
  r.push_back({"Canto vivo de 90 graus (esquerda)", [] {
    Pista p;
    p.reta(0, 0, 0, 400);
    p.reta(0, 400, -600, 400);
    return sessaoSimples("Canto vivo de 90 graus (esquerda)", std::move(p), norte(0, 50), true);
  }});
  r.push_back({"Arco de 150 mm", [] {
    Pista p;
    p.reta(0, 0, 0, 200);
    p.arco(150, 200, 150, 180, 90);
    p.reta(150, 350, 550, 350);
    return sessaoSimples("Arco de 150 mm", std::move(p), norte(0, 50), true);
  }});
  r.push_back({"Lacuna de 100 mm na linha", [] {
    Pista p;
    p.reta(0, 0, 0, 300);
    p.reta(0, 400, 0, 1400);
    return sessaoSimples("Lacuna de 100 mm na linha", std::move(p), norte(0, 100, 6), true);
  }});
  return r;
}

static std::string jsonCubos(const Mundo* m) {
  std::string o = "[";
  if (m) for (size_t i = 0; i < m->cubos.size(); i++) o += fmt("%s[%.0f,%.0f,%d,%d]", i ? "," : "", m->cubos[i].x, m->cubos[i].y, static_cast<int>(m->cubos[i].cor), m->cubos[i].obstaculo ? 1 : 0);
  return o + "]";
}

static std::string jsonPlataformas(const Mundo* m) {
  std::string o = "[";
  if (m) for (size_t i = 0; i < m->plataformas.size(); i++) o += fmt("%s[%.0f,%.0f,%d]", i ? "," : "", m->plataformas[i].x, m->plataformas[i].y, static_cast<int>(m->plataformas[i].cor));
  return o + "]";
}

static std::string jsonSegs(const Pista& p) {
  std::string o = "{\"w\":" + fmt("%.0f", p.largura) + ",\"s\":[";
  for (size_t i = 0; i < p.segs.size(); i++) o += fmt("%s[%.1f,%.1f,%.1f,%.1f,%d]", i ? "," : "", p.segs[i].x0, p.segs[i].y0, p.segs[i].x1, p.segs[i].y1, p.segs[i].cor);
  o += "],\"r\":[";
  for (size_t i = 0; i < p.decor.size(); i++) o += fmt("%s[%.0f,%.0f,%.0f,%.0f,%d]", i ? "," : "", p.decor[i].x0, p.decor[i].y0, p.decor[i].x1, p.decor[i].y1, p.decor[i].cor);
  return o + "]}";
}

static std::string valor(const std::string& q, const std::string& chave, const std::string& padrao = "") {
  const std::string k = chave + "=";
  size_t i = 0;
  while ((i = q.find(k, i)) != std::string::npos) {
    if (i == 0 || q[i - 1] == '&') {
      const size_t f = q.find('&', i);
      return q.substr(i + k.size(), f == std::string::npos ? std::string::npos : f - i - k.size());
    }
    i++;
  }
  return padrao;
}

static void responder(int fd, const std::string& tipo, const std::string& corpo) {
  const std::string h = "HTTP/1.1 200 OK\r\nContent-Type: " + tipo + "; charset=utf-8\r\nContent-Length: " + std::to_string(corpo.size()) + "\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n";
  send(fd, h.data(), h.size(), 0);
  size_t env = 0;
  while (env < corpo.size()) {
    const ssize_t n = send(fd, corpo.data() + env, corpo.size() - env, 0);
    if (n <= 0) break;
    env += static_cast<size_t>(n);
  }
}

extern const char* kPaginaAoVivo;

static int servir(int porta) {
  const int srv = socket(AF_INET, SOCK_STREAM, 0);
  int um = 1;
  setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &um, sizeof um);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);   // so esta maquina
  addr.sin_port = htons(static_cast<uint16_t>(porta));
  if (bind(srv, reinterpret_cast<sockaddr*>(&addr), sizeof addr) != 0 || listen(srv, 8) != 0) { perror("bind/listen"); return 1; }
  const std::vector<Receita> lista = receitas();
  std::map<int, std::shared_ptr<Sessao>> sessoes;
  auto pega = [&](int i) -> Sessao& {
    if (!sessoes.count(i)) sessoes[i] = lista[static_cast<size_t>(i)].cria();
    return *sessoes[i];
  };
  printf("Simulador ao vivo em http://localhost:%d  (Ctrl+C para parar)\n", porta);
  fflush(stdout);
  for (;;) {
    const int fd = accept(srv, nullptr, nullptr);
    if (fd < 0) continue;
    char buf[4096];
    const ssize_t n = recv(fd, buf, sizeof buf - 1, 0);
    if (n <= 0) { close(fd); continue; }
    buf[n] = 0;
    std::string req(buf), alvo;
    const size_t a = req.find(' '), b = req.find(' ', a + 1);
    if (a != std::string::npos && b != std::string::npos) alvo = req.substr(a + 1, b - a - 1);
    const size_t qi = alvo.find('?');
    const std::string rota = alvo.substr(0, qi), q = qi == std::string::npos ? "" : alvo.substr(qi + 1);
    const int i = std::max(0, std::min(static_cast<int>(lista.size()) - 1, atoi(valor(q, "s", "0").c_str())));

    if (rota == "/") {
      responder(fd, "text/html", kPaginaAoVivo);
    } else if (rota == "/lista") {
      std::string o = "[";
      for (size_t k = 0; k < lista.size(); k++) o += fmt("%s\"%s\"", k ? "," : "", lista[k].nome.c_str());
      responder(fd, "application/json", o + "]");
    } else if (rota == "/sessao") {   // (re)cria a sessao e devolve a pista
      sessoes.erase(i);
      Sessao& s = pega(i);
      std::string js = jsonSegs(*s.pista);
      js.pop_back();   // reabre o objeto para acrescentar os cubos
      responder(fd, "application/json", js + ",\"cubos\":" + jsonCubos(s.mundo.get()) + ",\"plat\":" + jsonPlataformas(s.mundo.get()) + ",\"ruidoDisponivel\":" + (s.mundo ? "true" : "false") + "}");
    } else if (rota == "/frames") {   // quadros [desde, desde+n): simula so ate aqui
      Sessao& s = pega(i);
      const size_t desde = static_cast<size_t>(std::max(0, atoi(valor(q, "desde", "0").c_str())));
      const size_t n2 = static_cast<size_t>(std::max(1, std::min(500, atoi(valor(q, "n", "50").c_str()))));
      s.avancarAte(desde + n2);
      std::string o = "{\"f\":[";
      const Mundo* mu = s.mundo.get();
      for (size_t k = desde; k < desde + n2; k++) {
        const Estado& e = s.carro->rastro()[k];
        o += fmt("%s[%.1f,%.1f,%.2f,%d", k > desde ? "," : "", e.x, e.y, e.th * 180.0 / M_PI, s.carro->bits()[k]);
        if (s.cena && s.cena->dois && k < s.cena->b.carro->rastro().size()) {   // o OUTRO robo, no mundo real (referencial espelhado)
          const Estado ob = espelho(s.cena->b.carro->rastro()[k]);
          o += fmt(",%.1f,%.1f,%.1f", ob.x, ob.y, ob.th * 180.0 / M_PI);
        } else if (mu && mu->op.ativo && k < mu->opQuadros.size()) o += fmt(",%.1f,%.1f,%.1f", mu->opQuadros[k][0], mu->opQuadros[k][1], mu->opQuadros[k][2] * 180.0 / M_PI);
        else o += ",-9999,-9999,0";
        if (mu && k < mu->cargaQuadros.size()) o += fmt(",%d,%d", mu->cargaQuadros[k][0], mu->cargaQuadros[k][1]);
        o += "]";
      }
      const Firmware& f = *s.fw;
      std::string info = fmt("cruzamentos %d | falsos alarmes %d | endireitou %d | bordas %d (meias-voltas %d) | portao %d | levantado %s", f.cruzamentos, f.falsosAlarmes, f.endireitou, f.bordas, f.meiasVoltas, f.portoes, s.carro->levantado() ? "SIM" : "nao");
      if (f.nav.ativo) info += fmt(" | NOS: %d PONTOS (coletou %d, entregou %d, brancos identificados %d, bloqueou %d rua(s)) | plano: %s | %s | velocidade medida %.0f mm/s", f.nav.pontos, f.nav.coletados, f.nav.entregues, f.nav.brancosIdentificados, f.nav.bloqueios, f.nav.ultimoPlano.c_str(), f.nav.carregando ? fmt("levando %d cubo(s)", static_cast<int>(f.nav.carga.size())).c_str() : "sem cubo", f.ajC.velMmS);
      if (s.cena && s.cena->dois) { const Firmware& g = *s.cena->b.fw; info += fmt(" | OPONENTE: %d PONTOS (coletou %d, entregou %d, brancos %d) | colisoes entre os robos %d", g.nav.pontos, g.nav.coletados, g.nav.entregues, g.nav.brancosIdentificados, s.cena->reinicios); }
      o += "],\"i\":\"" + info + "\"";
      if (mu) {
        o += ",\"ruido\":" + std::string(mu->ruidoLigado ? "true" : "false") + ",\"cubos\":[";
        for (size_t k = 0; k < mu->cubos.size(); k++) o += fmt("%s%ld", k ? "," : "", mu->cubos[k].mortoEm == static_cast<size_t>(-1) ? -1L : static_cast<long>(mu->cubos[k].mortoEm));
        o += "],\"idents\":[";
        for (size_t k = 0; k < f.nav.identificados.size(); k++) o += fmt("%s%zu", k ? "," : "", f.nav.identificados[k]);
        o += "],\"manchas\":[";
        for (size_t k = 0; k < mu->manchas.size(); k++) o += fmt("%s[%.0f,%.0f,%.0f,%zu,%ld]", k ? "," : "", mu->manchas[k].x, mu->manchas[k].y, mu->manchas[k].raio, mu->manchas[k].desde, mu->manchas[k].ate == static_cast<size_t>(-1) ? -1L : static_cast<long>(mu->manchas[k].ate));
        o += fmt("],\"alvo\":[%.0f,%.0f],\"ev\":[", f.nav.alvoX, f.nav.alvoY);
        const size_t ini = mu->log.size() > 12 ? mu->log.size() - 12 : 0;
        for (size_t k = ini; k < mu->log.size(); k++) { std::string l = mu->log[k]; for (char& ch : l) if (ch == '"' || ch == '\\') ch = '\''; o += fmt("%s\"%s\"", k > ini ? "," : "", l.c_str()); }
        o += "]";
        if (f.nav.ativo) {   // o que o carrinho LEMBRA: probabilidade de cubo em cada rua e quantas vezes passou em cada cruzamento
          o += ",\"mem\":{\"e\":[";
          for (int c2 = 0; c2 < core::kMapCols; c2++) for (int r2 = 0; r2 < core::kMapRows; r2++) o += fmt("%s%.2f", (c2 || r2) ? "," : "", f.nav.cubos.east[c2][r2]);
          o += "],\"n\":[";
          for (int c2 = 0; c2 < core::kMapCols; c2++) for (int r2 = 0; r2 < core::kMapRows; r2++) o += fmt("%s%.2f", (c2 || r2) ? "," : "", f.nav.cubos.north[c2][r2]);
          o += "],\"v\":[";
          for (int c2 = 0; c2 < core::kMapCols; c2++) for (int r2 = 0; r2 < core::kMapRows; r2++) o += fmt("%s%d", (c2 || r2) ? "," : "", f.nav.visitas.n[c2][r2]);
          o += "],\"b\":[";
          bool primeiro = true;
          for (int c2 = 0; c2 < core::kMapCols; c2++) for (int r2 = 0; r2 < core::kMapRows; r2++) {
            if (f.nav.bloq.east[c2][r2]) { o += fmt("%s[%d,%d,1]", primeiro ? "" : ",", c2, r2); primeiro = false; }
            if (f.nav.bloq.north[c2][r2]) { o += fmt("%s[%d,%d,0]", primeiro ? "" : ",", c2, r2); primeiro = false; }
          }
          o += "]}";
        }
      }
      o += "}";
      responder(fd, "application/json", o);
    } else if (rota == "/colocar") {   // solta o robo em (x, y) mm, apontando para th graus (a memoria do firmware recomeca: foi pego e posto)
      Sessao& s = pega(i);
      const double th = atof(valor(q, "th", "90").c_str()) * M_PI / 180.0;
      s.carro->colocarEm({atof(valor(q, "x", "0").c_str()), atof(valor(q, "y", "0").c_str()), th});
      s.carro->levantar(false);
      s.fw->memoria = Memoria{};
      s.fw->cruz = MemoriaCruzamento{};
      responder(fd, "text/plain", "ok");
    } else if (rota == "/acao") {
      Sessao& s = pega(i);
      const std::string ac = valor(q, "a");
      if (ac == "levantar") s.carro->levantar(true);
      else if (ac == "soltar") {   // SEMPRE em posicao aleatoria do verde, olhando para as ruas (o MPU volta a rumo 0)
        Estado e = s.carro->estado();
        e.th = s.frenteGraus * M_PI / 180.0;
        if (s.cena) { e.x = -270.0 + s.mundo->rnd() * 230.0; e.y = 60.0 + s.mundo->rnd() * (kAlt - 120.0); e.th = 0.0; s.fw->rumo = 0.0f; s.fw->nav.reposicionado(); s.fw->precisaAlinhar = true; s.fw->portaoPendente = true; s.fw->memoria = Memoria{}; s.fw->cruz = MemoriaCruzamento{}; s.mundo->carga0 = s.mundo->carga1 = 0; }
        s.carro->colocarEm(e);
        s.carro->levantar(false);
      }   // SEMPRE de frente
      else if (ac == "bater") s.carro->bater(2.5, 4);
      else if (ac == "ruido" && s.mundo) { s.mundo->ruidoLigado = valor(q, "on") == "1"; s.mundo->evento(s.carro->agoraMs(), s.mundo->ruidoLigado ? "RUIDO LIGADO: manchas, patinagem e atraso de sensores aleatorios (cada evento e avisado aqui)" : "ruido desligado"); }
      responder(fd, "text/plain", "ok");
    } else {
      const std::string o = "nao encontrado";
      const std::string h = "HTTP/1.1 404 Not Found\r\nContent-Length: " + std::to_string(o.size()) + "\r\nConnection: close\r\n\r\n" + o;
      send(fd, h.data(), h.size(), 0);
    }
    close(fd);
  }
}

const char* kPaginaAoVivo = R"HTML(<!doctype html><html lang="pt-br"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Simulador do seguidor ao vivo</title>
<style>:root{--bg:#f6f6f4;--fg:#1b1b1b;--card:#fff;--bd:#d8d8d4;--ac:#2a6df4}@media(prefers-color-scheme:dark){:root{--bg:#161618;--fg:#ececec;--card:#202024;--bd:#34343a;--ac:#6ea0ff}}
body{margin:0;background:var(--bg);color:var(--fg);font:14px system-ui,sans-serif}main{max-width:1100px;margin:0 auto;padding:16px}
.bar{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin-bottom:10px}select,button,input{font:inherit;color:var(--fg);background:var(--card);border:1px solid var(--bd);border-radius:6px;padding:6px 10px}
button{cursor:pointer}canvas{width:100%;background:var(--card);border:1px solid var(--bd);border-radius:8px;display:block;cursor:crosshair}
#sens{display:flex;gap:6px;align-items:center;margin:10px 0}#sens span{width:26px;height:26px;border-radius:50%;border:2px solid var(--bd);display:inline-flex;align-items:center;justify-content:center;font-size:11px}#sens span.on{background:#111;color:#fff;border-color:#111}
@media(prefers-color-scheme:dark){#sens span.on{background:#fff;color:#111;border-color:#fff}}small{opacity:.7;display:block;margin-top:6px}#info{font-variant-numeric:tabular-nums;margin-top:6px}</style></head><body><main>
<div class="bar"><select id="sel" style="max-width:100%"></select><button id="rein" title="recria a simulacao do zero">&#8634; recomecar</button></div>
<div class="bar"><button id="b1">&#9664;&#9664; 1 s</button><button id="b01">&#9664; 0,1 s</button><button id="pp">Pausar</button><button id="f01">0,1 s &#9654;</button><button id="f1">1 s &#9654;&#9654;</button>
<label>velocidade <select id="vel"><option value="0.25">0,25x</option><option value="0.5">0,5x</option><option value="1" selected>1x</option><option value="2">2x</option><option value="4">4x</option><option value="8">8x</option></select></label><span id="tt"></span></div>
<div class="bar"><label>soltar apontando para <select id="th"><option value="90">cima (norte)</option><option value="0">direita (leste)</option><option value="180">esquerda (oeste)</option><option value="-90">baixo (sul)</option></select></label>
<button id="lev">Levantar o robo</button><button id="sol">Soltar de frente</button><button id="bat">Pancada forte</button><label><input type="checkbox" id="seg" checked> camera segue o robo</label><label id="lbM"><input type="checkbox" id="memchk" checked> mostrar a memoria do carrinho</label><label id="lbR"><input type="checkbox" id="rui"> ruido aleatorio (sinalizado)</label></div>
<div id="alerta" style="display:none;background:#e02424;color:#fff;border-radius:6px;padding:6px 10px;margin-bottom:8px;font-weight:600">RUIDO LIGADO: manchas, patinagem e atraso de sensores acontecem de forma aleatoria. Cada evento aparece no registro abaixo.</div>
<canvas id="c" width="1100" height="640"></canvas><div id="sens"></div><div id="info"></div><pre id="ev" style="white-space:pre-wrap;font-size:12px;max-height:190px;overflow:auto;background:var(--card);border:1px solid var(--bd);border-radius:6px;padding:8px;margin:8px 0"></pre>
<small>Simulacao AO VIVO, sem fim: nada e gravado, o servidor C++ simula o que voce esta vendo. Clique no mapa para soltar o robo ali (apontando para o lado escolhido). Sensores: S1 (esquerda) a S8 (direita), aceso = preto. Linha 20 mm, barra de 75 mm a 30 mm do eixo, carro 170 x 200 mm. Setas: 0,1 s; shift + seta: 1 s; espaco: pausa. LEGENDA. Rosa na rua = a chance de cubo que o carrinho LEMBRA (esquece com o tempo); numero no cruzamento = quantas vezes passou ali; X vermelho + 'bloqueada' = rua com um cubo que NAO se pega (branco ou cor sem plataforma: pelas regras so se identifica, sinaliza e desvia); linha verde tracejada + 'destino' = para onde esta indo agora; quadradinhos sobre o carro = cubos que esta carregando (no maximo 2); retangulos coloridos no ecoponto amarelo = plataformas; quadrados com borda laranja tracejada = cubos de quadrado marrom (so obstaculo); retangulo cinza 'outro robo' = o oponente (outro robo igual ao nosso); circulos marrons = manchas de ruido (preto = sujeira que parece linha; o desgaste branco na fita some com a linha).</small></main><script>
const cv=document.getElementById('c'),cx=cv.getContext('2d'),sel=document.getElementById('sel'),sens=document.getElementById('sens');
const cores=['#111','#e02424','#1c64f2'],zonas=['#2e8b3d','#f2d024','#8b4a1f'];
let cur=0,pista=null,frames=[],fr=0,play=true,pedindo=false,last=0,vel=1,info='',trilha0=0,mortoEm=[],manchas=[],mem=null,ruido=false;
const coresCubo=['#999','#ffffff','#e02424','#1c64f2','#2e8b3d','#f2d024','#111111','#8b4a1f'],nomesCubo=['','branco','vermelho','azul','verde','amarelo','preto','marrom'];
let alvo=[-1,-1],idents=[];
const BE=168,PS=400;
for(let i=0;i<8;i++){const s=document.createElement('span');s.textContent='S'+(i+1);sens.appendChild(s)}
const dark=()=>matchMedia('(prefers-color-scheme: dark)').matches;
async function j(u){return (await fetch(u)).json()}
async function carrega(i){cur=i;frames=[];fr=0;trilha0=0;play=true;pp.textContent='Pausar';mortoEm=[];manchas=[];mem=null;pista=await j('/sessao?s='+i);document.getElementById('lbR').style.display=pista.ruidoDisponivel?'':'none';document.getElementById('lbM').style.display=pista.ruidoDisponivel?'':'none';rui.checked=false;await mais()}
async function mais(){if(pedindo)return;pedindo=true;try{const r=await j(`/frames?s=${cur}&desde=${frames.length}&n=25`);for(const f of r.f)frames.push(f);info=r.i;if(r.cubos)mortoEm=r.cubos;manchas=r.manchas||[];if(r.ev)document.getElementById('ev').textContent=r.ev.join('\n');mem=r.mem||null;idents=r.idents||[];alvo=r.alvo||[-1,-1];ruido=!!r.ruido;rui.checked=ruido;document.getElementById('alerta').style.display=ruido?'block':'none'}finally{pedindo=false}}
let vista=null;
function tela(){const f=frames[fr]||[0,0,0,0];
if(seg.checked){const k=cv.width/900;return{k,ox:cv.width/2-f[0]*k,oy:cv.height/2+f[1]*k}}
let x0=1e9,y0=1e9,x1=-1e9,y1=-1e9;const a=(x,y)=>{x0=Math.min(x0,x);x1=Math.max(x1,x);y0=Math.min(y0,y);y1=Math.max(y1,y)};
pista.s.forEach(s=>{a(s[0],s[1]);a(s[2],s[3])});pista.r.forEach(r=>{a(r[0],r[1]);a(r[2],r[3])});x0-=60;x1+=60;y0-=60;y1+=60;
const k=Math.min(cv.width/(x1-x0),cv.height/(y1-y0));return{k,ox:(cv.width-(x1-x0)*k)/2-x0*k,oy:(cv.height-(y1-y0)*k)/2+y1*k}}
const P=(t,x,y)=>[t.ox+x*t.k,t.oy-y*t.k];
function desenha(){if(!pista||!frames.length)return;const t=tela();vista=t;cx.clearRect(0,0,cv.width,cv.height);
pista.r.forEach(r=>{const a=P(t,r[0],r[3]),b=P(t,r[2],r[1]);cx.fillStyle=zonas[r[4]];cx.fillRect(a[0],a[1],b[0]-a[0],b[1]-a[1])});
pista.s.forEach(s=>{cx.strokeStyle=dark()&&s[4]==0?'#cfcfcf':cores[s[4]];cx.lineWidth=pista.w*t.k;const a=P(t,s[0],s[1]),b=P(t,s[2],s[3]);cx.beginPath();cx.moveTo(a[0],a[1]);cx.lineTo(b[0],b[1]);cx.stroke()});
if(mem&&memchk.checked){const R=5;const xc=c=>BE+c*PS,yr=r=>BE+r*PS;
for(let c=0;c<6;c++)for(let r=0;r<R;r++){const pe=mem.e[c*R+r],pn=mem.n[c*R+r];
if(c<5){const a=P(t,xc(c),yr(r)),b=P(t,xc(c+1),yr(r));cx.strokeStyle='rgba(214,0,170,'+(0.08+0.62*pe)+')';cx.lineWidth=34*t.k;cx.beginPath();cx.moveTo(a[0],a[1]);cx.lineTo(b[0],b[1]);cx.stroke()}
if(r<4){const a=P(t,xc(c),yr(r)),b=P(t,xc(c),yr(r+1));cx.strokeStyle='rgba(214,0,170,'+(0.08+0.62*pn)+')';cx.lineWidth=34*t.k;cx.beginPath();cx.moveTo(a[0],a[1]);cx.lineTo(b[0],b[1]);cx.stroke()}}
mem.b.forEach(q=>{const c=q[0],r=q[1],h=q[2];const a=P(t,xc(c)+(h?PS/2:0),yr(r)+(h?0:PS/2));cx.strokeStyle='#e02424';cx.lineWidth=4;cx.beginPath();cx.moveTo(a[0]-9,a[1]-9);cx.lineTo(a[0]+9,a[1]+9);cx.moveTo(a[0]+9,a[1]-9);cx.lineTo(a[0]-9,a[1]+9);cx.stroke();cx.fillStyle='#e02424';cx.font='bold 12px system-ui';cx.fillText('bloqueada',a[0]+14,a[1]+4)});
cx.fillStyle=dark()?'#fff':'#111';cx.font='bold 13px system-ui';for(let c=0;c<6;c++)for(let r=0;r<R;r++){const n=mem.v[c*R+r];if(n>0){const a=P(t,xc(c)+16,yr(r)-16);cx.fillText(n,a[0],a[1])}}}
(pista.plat||[]).forEach(q=>{const a=P(t,q[0]-25,q[1]+80),b=P(t,q[0]+25,q[1]-80);cx.fillStyle=coresCubo[q[2]];cx.fillRect(a[0],a[1],b[0]-a[0],b[1]-a[1]);cx.strokeStyle='#fff';cx.lineWidth=2;cx.strokeRect(a[0],a[1],b[0]-a[0],b[1]-a[1]);cx.save();cx.translate(a[0]+(b[0]-a[0])/2,a[1]+(b[1]-a[1])/2);cx.rotate(-Math.PI/2);cx.fillStyle=q[2]==1||q[2]==5?'#111':'#fff';cx.font='bold 11px system-ui';cx.textAlign='center';cx.textBaseline='middle';cx.fillText(nomesCubo[q[2]],0,0);cx.restore()});
pista.cubos&&pista.cubos.forEach((q,i)=>{if(mortoEm[i]>=0&&fr>=mortoEm[i])return;const a=P(t,q[0]-25,q[1]+25),b=P(t,q[0]+25,q[1]-25);cx.fillStyle=coresCubo[q[2]];cx.fillRect(a[0],a[1],b[0]-a[0],b[1]-a[1]);cx.strokeStyle=q[3]?'#ff7a00':'#111';cx.lineWidth=q[3]?3:2;if(q[3])cx.setLineDash([4,3]);cx.strokeRect(a[0],a[1],b[0]-a[0],b[1]-a[1]);cx.setLineDash([]);if(idents.includes(i)){cx.fillStyle='#00a86b';cx.font='bold 12px system-ui';cx.fillText('identificado',a[0]-8,a[1]-6)}});
if(alvo[0]>=0&&frames[fr]){const f0=frames[fr],p0=P(t,f0[0],f0[1]),p1=P(t,alvo[0],alvo[1]);cx.strokeStyle='#00a86b';cx.lineWidth=2.5;cx.setLineDash([8,6]);cx.beginPath();cx.moveTo(p0[0],p0[1]);cx.lineTo(p1[0],p1[1]);cx.stroke();cx.setLineDash([]);cx.beginPath();cx.arc(p1[0],p1[1],10,0,7);cx.stroke();cx.fillStyle='#00a86b';cx.font='bold 12px system-ui';cx.fillText('destino',p1[0]+14,p1[1]-8)}
manchas.forEach(m=>{if(fr>=m[3]&&(m[4]<0||fr<m[4])){const a=P(t,m[0],m[1]);cx.fillStyle='rgba(70,45,25,.85)';cx.beginPath();cx.arc(a[0],a[1],m[2]*t.k,0,7);cx.fill()}});
cx.strokeStyle='#e02424';cx.lineWidth=1.5;cx.beginPath();for(let i=Math.max(0,fr-1500);i<=fr;i++){const p=P(t,frames[i][0],frames[i][1]);i==Math.max(0,fr-1500)?cx.moveTo(p[0],p[1]):cx.lineTo(p[0],p[1])}cx.stroke();
if(frames[fr].length>6&&frames[fr][4]>-9000){const o=frames[fr],ot=o[6]*Math.PI/180,oc=Math.cos(ot),os=Math.sin(ot);const OW=(fx,fy)=>P(t,o[4]+fx*oc-fy*os,o[5]+fx*os+fy*oc);cx.fillStyle='rgba(120,120,120,.55)';cx.strokeStyle='#555';cx.lineWidth=2;cx.beginPath();[[-100,-85],[100,-85],[100,85],[-100,85]].forEach((q,i)=>{const p=OW(q[0],q[1]);i?cx.lineTo(p[0],p[1]):cx.moveTo(p[0],p[1])});cx.closePath();cx.fill();cx.stroke();const lp=OW(0,0);cx.fillStyle=dark()?'#fff':'#111';cx.font='12px system-ui';cx.fillText('outro robo',lp[0]-26,lp[1]+4)}
const f=frames[fr],th=f[2]*Math.PI/180,c=Math.cos(th),s=Math.sin(th);const W=(fx,fy)=>P(t,f[0]+fx*c-fy*s,f[1]+fx*s+fy*c);
cx.fillStyle=dark()?'rgba(110,160,255,.35)':'rgba(42,109,244,.25)';cx.strokeStyle='#2a6df4';cx.lineWidth=2;cx.beginPath();[[-100,-85],[100,-85],[100,85],[-100,85]].forEach((q,i)=>{const p=W(q[0],q[1]);i?cx.lineTo(p[0],p[1]):cx.moveTo(p[0],p[1])});cx.closePath();cx.fill();cx.stroke();
const fr0=W(100,0),to=W(60,0);cx.beginPath();cx.moveTo(to[0],to[1]);cx.lineTo(fr0[0],fr0[1]);cx.stroke();
const e=W(0,0);cx.fillStyle='#2a6df4';cx.beginPath();cx.arc(e[0],e[1],4,0,7);cx.fill();
[7,8].forEach((ix,k)=>{const cor=f[ix];if(cor>0){const a=W(-35,(k?1:-1)*28),sz=22*t.k;cx.fillStyle=coresCubo[cor];cx.fillRect(a[0]-sz/2,a[1]-sz/2,sz,sz);cx.strokeStyle='#111';cx.lineWidth=2;cx.strokeRect(a[0]-sz/2,a[1]-sz/2,sz,sz)}});
for(let i=0;i<8;i++){const lat=37.5-i*75/7,p=W(30,lat),on=(f[3]>>i)&1;cx.fillStyle=on?'#ffb000':(dark()?'#555':'#ddd');cx.strokeStyle=on?'#a06000':'#888';cx.lineWidth=1;cx.beginPath();cx.arc(p[0],p[1],Math.max(3,5*t.k*2),0,7);cx.fill();cx.stroke()}
[...sens.children].forEach((s,i)=>s.classList.toggle('on',((f[3]>>i)&1)==1));
tt.textContent='t = '+(fr*0.02).toFixed(2)+' s';document.getElementById('info').textContent=info}
function pausa(){play=false;pp.textContent='Continuar'}
function passo(n){pausa();fr=Math.max(0,Math.min(frames.length-1,fr+n));desenha();if(frames.length-fr<60)mais()}
sel.onchange=()=>carrega(+sel.value);rein.onclick=()=>carrega(cur);
b1.onclick=()=>passo(-50);b01.onclick=()=>passo(-5);f01.onclick=()=>passo(5);f1.onclick=()=>passo(50);
pp.onclick=()=>{play=!play;pp.textContent=play?'Pausar':'Continuar'};vel_=document.getElementById('vel');vel_.onchange=e=>vel=+e.target.value;
rui.onchange=()=>fetch('/acao?s='+cur+'&a=ruido&on='+(rui.checked?1:0));
lev.onclick=()=>fetch('/acao?s='+cur+'&a=levantar');sol.onclick=()=>fetch('/acao?s='+cur+'&a=soltar');bat.onclick=()=>fetch('/acao?s='+cur+'&a=bater');
cv.onclick=e=>{if(!vista)return;const r=cv.getBoundingClientRect(),px=(e.clientX-r.left)*cv.width/r.width,py=(e.clientY-r.top)*cv.height/r.height;const x=(px-vista.ox)/vista.k,y=(vista.oy-py)/vista.k;
fetch(`/colocar?s=${cur}&x=${x.toFixed(1)}&y=${y.toFixed(1)}&th=${document.getElementById('th').value}`);play=true;pp.textContent='Pausar'};
addEventListener('keydown',e=>{if(e.key==='ArrowRight')passo(e.shiftKey?50:5);else if(e.key==='ArrowLeft')passo(e.shiftKey?-50:-5);else if(e.key===' '){e.preventDefault();pp.click()}});
let acum=0,ultimoTs=0;
function loop(ts){const dt=ultimoTs?ts-ultimoTs:0;ultimoTs=ts;if(play&&frames.length){acum+=dt*vel/20;while(acum>=1){acum-=1;if(fr<frames.length-1)fr++}}
if(frames.length-fr<40)mais();desenha();requestAnimationFrame(loop)}
(async()=>{const nomes=await j('/lista');nomes.forEach((n,i)=>{const o=document.createElement('option');o.value=i;o.textContent=n;sel.appendChild(o)});await carrega(0);requestAnimationFrame(loop)})();
</script></body></html>)HTML";

int main(int argc, char** argv) {
  bool servirAoVivo = false;
  // Argumentos: uma pasta (grava um SVG por cenario) e/ou chave=valor para os parametros fisicos: tau, derrap, bitola, zona, ganho, barra
  for (int i = 1; i < argc; i++) {
    const char* eq = strchr(argv[i], '=');
    if (!eq) {
      if (!strcmp(argv[i], "-q")) gSo_Placar = true;
      else if (!strcmp(argv[i], "--servir")) servirAoVivo = true;
      else if (!strcmp(argv[i], "--bitola")) {   // mede a bitola efetiva no centro de um cruzamento, com o eixo deslocado de 0 a 8 mm (erro de posicionar o carro)
        for (const double off : {0.0, 3.0, 6.0, 8.0}) {
          Pista pista = tabuleiro();
          Carro c(pista, {xDoCruzamento(1) + off, yDoCruzamento(1) - off * 0.5, 0.3}, gFis);
          const double b = medirBitolaEfetiva(c, gAjustes.velBuscaExterna);
          printf("eixo %.0f mm fora do centro: bitola efetiva medida %.1f mm (verdadeira %.1f = %.0f / %.2f)\n", off, b, gFis.bitolaMm / gFis.derrapagem, gFis.bitolaMm, gFis.derrapagem);
        }
        return 0;
      }
      else if (!strcmp(argv[i], "--coleta")) {   // --coleta N [sementes]: tempo para entregar N cubos (sem ruido), uma linha por semente
        const int n = i + 1 < argc ? atoi(argv[i + 1]) : 18;
        const int sementes = i + 2 < argc ? atoi(argv[i + 2]) : 5;
        double soma = 0.0; int completas = 0;
        for (int k = 0; k < sementes; k++) {
          if (getenv("SIM_SO") && atoi(getenv("SIM_SO")) != k) continue;
          const std::vector<double> e = coletaDeN(n, static_cast<unsigned long>(k));
          const bool ok = static_cast<int>(e.size()) >= n;
          printf("semente %2d: %s | entregou %zu de %d | ultima em %.0f s | metade (%d) em %.0f s\n", k, ok ? "COMPLETA" : "tempo acabou", e.size(), n, e.empty() ? 0.0 : e.back(), n / 2, static_cast<int>(e.size()) >= n / 2 ? e[static_cast<size_t>(n / 2 - 1)] : 0.0);
          if (ok) { soma += e.back(); completas++; }
        }
        if (completas) printf("COLETA %d cubos: %d de %d completas em 12 min | %.0f s em media nas completas\n", n, completas, sementes, soma / completas);
        else printf("COLETA %d cubos: nenhuma completa em 12 min\n", n);
        return 0;
      }
      else if (!strcmp(argv[i], "--lote") || !strcmp(argv[i], "--lote-ruido")) {   // missao so com o nosso robo em N sorteios: mede em vez de olhar uma partida so (e caotica)
        const bool ruido = !strcmp(argv[i], "--lote-ruido");
        const int n = i + 1 < argc ? atoi(argv[i + 1]) : 10;
        int ok = 0, toques = 0, entregues = 0;
        double tempo = 0;
        for (int k = 0; k < n; k++) {
          gSemente = static_cast<unsigned long long>(k);
          if (getenv("SIM_SO") && atoi(getenv("SIM_SO")) != k) continue;
          cenarioMissao(ruido, 2);
          printf("semente %2d: %s | entregou %d | pontos %4d | toques em cubos %d | %.0f s\n", k, gUltimaMissao.fim ? "META" : "falhou", gUltimaMissao.entregues, gUltimaMissao.pontos, gUltimaMissao.toques, gUltimaMissao.tempoS);
          ok += gUltimaMissao.fim; toques += gUltimaMissao.toques; entregues += gUltimaMissao.entregues; tempo += gUltimaMissao.tempoS;
        }
        printf("LOTE %s: %d de %d chegaram a meta | %.1f entregas e %.1f toques em cubos por partida | %.0f s em media\n", ruido ? "com ruido" : "sem ruido", ok, n, double(entregues) / n, double(toques) / n, tempo / n);
        return 0;
      }
      else gSvgDir = argv[i];
      continue;
    }
    const std::string k(argv[i], eq - argv[i]);
    const double v = atof(eq + 1);
    if (k == "tau") gFis.tauMs = v;
    else if (k == "derrap") gFis.derrapagem = v;
    else if (k == "cap") gCapacidade = static_cast<int>(v);
    else if (k == "rumoenc") gRumoPeloEncoder = v != 0.0;
    else if (k == "autocal") gAutoCalBitola = v != 0.0;
    else if (k == "encoder") { gFis.encoder = v != 0.0; if (gFis.encoder) { gFis.zonaMortaPwm = 0; gAjustes.zonaMortaPwm = 0; } }   // STS3215: sem zona morta, malha fechada
    else if (k == "escalaodo") gFis.escalaOdo = v;
    else if (k == "gesto") gGestoGraus = v;
    else if (k == "bitola") gFis.bitolaMm = v;
    else if (k == "bitolaaj") gAjustes.bitolaMm = v;   // a bitola que o FIRMWARE supoe (padrao 140; a real do simulador e 150)
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
  if (servirAoVivo) return servir(8765);
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
  for (double a : {0.04, -0.07}) rs.push_back(cenarioAssimetria(a));
  for (double l : {60.0, 100.0, 150.0}) {
    rs.push_back(cenarioLacunaImu(l));
    rs.back().informativo = l > 100.0;
  }
  rs.push_back(cenarioVermelhoDeLado());
  rs.push_back(cenarioTabuleiro());
  rs.push_back(cenarioMissao(false, 2));
  rs.back().informativo = true;   // a localizacao ainda deriva depois de colisoes e curvas nao verificadas (veja o README): mostra, nao derruba
  rs.push_back(cenarioMissao(true, 2));
  rs.back().informativo = true;
  rs.push_back(cenarioExplorarOutroLado());
  rs.back().informativo = true;
  rs.push_back(cenarioDuasEquipes(false, 2));
  rs.back().informativo = true;
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
  printf("%d de %d cenarios passaram\n", static_cast<int>(rs.size()) - falhas, static_cast<int>(rs.size()));
  return falhas ? 1 : 0;
}
