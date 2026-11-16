// A logica de navegacao do carro (seguir a rua, cruzamentos, giro na linha, mapa de cubos, coleta e entrega): UM dono, usado pelo simulador e pelo carro real.
// `Carro` e o robo de verdade ou o simulado; precisa oferecer (duck typing):
//   tempo/ciclo:  agoraMs(), avancar() (espera/simula um ciclo de cfg.dtMs)
//   atuadores:    comandar(pwmEsq, pwmDir) (-255..255), parar(), roda(esquerda) (alvo da roda, 0 = parada),
//                 coletar(i) -> bool (pegou e guardou o cubo), soltar(cor) -> bool (largou o cubo na plataforma)
//   sensores:     ler(bool[8]) (true = linha), odoMm()/odoEMm()/odoDMm() (mm das rodas), us() (mm ou -1), tcsChao(), tcsFrente(), piso(),
//                 plataformaNaFrente(alcanceMm), cuboNaFrente(alcanceMm)
//   missao:       missao(), usAtivo(), tcsPeriodoMs(), consumirToques(), mostrarCarga(slot, cor), limparCarga(), sinal(ligado) (buzzer), evento(texto)
//   so depuracao: estado() (a verdade do simulador; o carro real devolve zeros), ecoTipo() (o que deu o ultimo eco; vazio no carro real)
#pragma once
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <array>
#include <functional>
#include <string>
#include <vector>

#include <core/belief.h>
#include <core/explore.h>
#include <core/localizer.h>
#include <core/map.h>
#include <core/navigator.h>
#include "alvo.h"
#include "borda.h"
#include "cruzamento.h"
#include "decisao.h"
#include "exploracao.h"
#include "giro_cruzamento.h"
#include "segurar_rumo.h"
#include "tabuleiro.h"

#if defined(ROBO_SIMULADOR)   // so no PC: o carro real nao imprime nem le variaveis de ambiente
#define ROBO_DEPURANDO() (getenv("SIM_DEBUG") != nullptr)
#define ROBO_TRACE(agora) (getenv("SIM_TRACE") && fabs((agora) / 1000.0 - atof(getenv("SIM_TRACE"))) < 2.0)
#define ROBO_SEM(nome) (getenv(nome) != nullptr)
#define ROBO_DBG(...) do { if (ROBO_DEPURANDO()) fprintf(stderr, __VA_ARGS__); } while (0)
#else
#define ROBO_DEPURANDO() false
#define ROBO_TRACE(agora) false
#define ROBO_SEM(nome) false
#define ROBO_DBG(...) do { if (false) fprintf(stderr, __VA_ARGS__); } while (0)   // nunca roda no carro, mas as variaveis so de depuracao contam como usadas
#endif

namespace seguidor {

inline std::string fmt(const char* f, ...) __attribute__((format(printf, 1, 2)));
inline std::string fmt(const char* f, ...) {
  char b[512];
  va_list a;
  va_start(a, f);
  vsnprintf(b, sizeof b, f, a);
  va_end(a);
  return b;
}

// O que a logica precisa saber do robo e dos ajustes (no simulador vem da linha de comando; no carro, de params.h).
struct ConfigRobo {
  double dtMs = 5.0;          // um ciclo de leitura + decisao
  double tauMs = 50.0;        // atraso de primeira ordem das rodas (o giro ainda anda isto depois do comando de parar)
  bool encoder = false;       // STS3215: odometria das rodas, malha fechada no servo
  bool autoCalBitola = false;     // cada giro na linha corrige a bitola efetiva (o meio da faixa tem ~5% de vies: piora o caso ja calibrado)
  double gestoGraus = 45.0;       // modo encoder: no cruzamento esperado (odometro ~400 mm, pose segura) so da uma rodadinha deste angulo para UM lado (0 = varredura de 90, < 0 = passa direto)
  Ajustes ajustes;
  AjustesCruzamento cruzamento;   // velMmS e o resto do que a logica supoe do carro (no carro e no simulador: perfil_carro.h)
};

struct Navegador {
  bool ativo = false;
  core::PoseBelief loc;
  core::CubeBelief cubos;
  core::BeliefModel bm;
  core::Visits visitas;
  core::Blocked bloq;
  seguidor::AjustesAlvo alvo;
  seguidor::AjustesColeta coleta;
  bool carregando = false, pularProximo = false, entregando = false, naRua = false;
  std::vector<seguidor::CorCubo> carga;    // o que leva agora (no maximo 2 por viagem, regras 7)
  int pontos = 0, perdidos = 0, brancosIdentificados = 0;
  std::vector<size_t> identificados;       // cubos brancos/sem plataforma ja sinalizados (so pontua uma vez por cubo)
  std::vector<std::pair<double, seguidor::CorCubo>> plataformasVistas;   // o que OBSERVOU no ecoponto no inicio: (y do slot, cor); so entrega onde viu
  bool plataformasObservadas = false;
  bool observando = false;      // viagem ao ecoponto SEM cubo para ver as plataformas antes de coletar (so uma vez): usa o fluxo de entrega, com a carga vazia
  bool tentouObservar = false;
  bool observarNoInicio = false;   // viagem de observacao das plataformas antes do 1o cubo. DESLIGADA: custa ~85 s e a reentrada depois dela falhou no simulador (10 entregas em 5 sementes sem ela, 4 com ela)
  double alvoX = -1, alvoY = -1;           // para onde esta indo (para mostrar a trajetoria)
  std::string log;
  core::Node ruaDe{0, 0}, ruaPara{0, 0};   // a rua por onde anda: do ultimo cruzamento ao proximo
  unsigned long ultimoForgetMs = 0;
  int coletados = 0, entregues = 0, bloqueios = 0, reposicoes = 0, cuboPerdidoNaReposicao = 0;
  std::string ultimoPlano;
  bool ruaUsada = false;   // ja saiu de um cruzamento por uma rua (a de ruaDe para ruaPara)
  const void* verdade = nullptr;   // so para depurar: compara o que observa com a realidade

  void iniciar() {
    ativo = true;
    core::initInGreen(loc);
    core::initBelief(cubos, bm);
    core::clearVisits(visitas);
    core::clearBlocked(bloq);
  }
  // Reiniciado pelo juiz (levantado e posto no verde) ou verde inesperado: a pose some, o mapa de cubos FICA, o cubo na garra volta ao juiz.
  void reposicionado() {
    core::initInGreen(loc);
    if (observando) { observando = false; carregando = false; }   // o juiz levou o carro de volta: a viagem de observar perdeu o sentido (tentouObservar fica: nao repete)
    naRua = false; pularProximo = false; entregando = false;
    if (carregando) { cuboPerdidoNaReposicao += static_cast<int>(carga.size()); pontos -= 10 * static_cast<int>(carga.size()); perdidos += static_cast<int>(carga.size()); }   // cubo na garra: o juiz o devolve ao lugar, conta como perdido
    carga.clear();
    carregando = false;
    linhaEntrada = -1;
    calibrando = 0;
    contados = 0;
    reposicoes++;
    pontos -= 15;   // cada reinicio custa 15
  }
  // Depois de cada reset o robo se ALINHA com uma rua pela referencia vermelha (sabe a linha de verdade): no portao a coluna e 0 e a linha e a escolhida, sem precisar achar a
  // borda vermelha para descobrir. E mede + varre os 5 primeiros cruzamentos em linha reta (calibracao): velocidade, rumo e as ruas dessa linha entram na memoria.
  void aoPortao() {
    // coluna 0; a linha: se entrou alinhado com uma rua conhecida (volta do ecoponto: as linhas ficam a 400 mm, entao um erro de dezenas de mm ainda escolhe a certa) ja sabe; senao
    // descobre indo ate a linha vermelha de um dos lados e contando os cruzamentos que viu
    if (linhaEntrada >= 0 && linhaEntrada < core::kMapRows) core::onOwnBlueAtRow(loc, linhaEntrada, 0.9f);
    else {
      core::onOwnBlue(loc);
      // Sem saber a linha, o 1o contato com a coluna 0 e quase sempre o MEIO de um quarteirao (ou o beco), nao um cruzamento: contar como cruzamento fazia a conta de linhas passar de 4
      // (partida a y<168: 5 "cruzamentos" para cima) e o localizador ficava seguro numa linha errada sem nunca ver a moldura. Pula a contagem dele: o proximo cruzamento de verdade e o 1o no (0, r).
      if (primeiroContatoNaoENo) { pularProximo = true; contatoNaoContado = true; }
    }
  }
  bool primeiroContatoNaoENo = true;
  bool contatoNaoContado = false;   // o 1o contato com a coluna 0 foi pulado (nao contou como no): ate o 1o cruzamento contado nao ha no 'de onde veio' para a meia-volta da borda voltar a
  int linhaEntrada = -1;   // a linha pela qual entrou (conhecida pela moldura vermelha); -1 = nao sabe
  double faseGrade = std::nan("");   // as plataformas ficam numa grade de 260 mm, mas a FASE dela (nas 5 marcas ou entre elas) so se sabe depois de ver uma: desvio de (y - meio) em relacao ao passo
  std::vector<std::pair<double, seguidor::CorCubo>> platConhecidas;   // (y, cor) das plataformas ja lidas de perto
  std::vector<double> yOlhados;   // alturas do ecoponto ja olhadas (ou sem eco): nao olha de novo
  std::vector<double> platPresentes;   // alturas onde o ultrassom parado no meio do mapa achou plataforma (ainda sem ler a cor)
  bool mapeado = false;   // a varredura parada ja foi feita (uma vez por jogo)
  int calibrando = 0;      // (nao usado: a calibracao pos-reset agora e ir ate a linha vermelha contando cruzamentos)
  int contados = 0;        // cruzamentos contados nesse trecho ate a linha vermelha
  void aoProibido(core::Forbidden k) { core::onForbidden(loc, k); core::onTurn(loc, 2); pularProximo = !contatoNaoContado; }   // meia-volta no fim da rua: volta ao cruzamento que deixou
  void aposVirar(char a) { core::onTurn(loc, a == 'L' ? 1 : a == 'R' ? -1 : a == 'U' ? 2 : 0); }
  bool ruaValida() const { return core::inMap(ruaDe.c, ruaDe.r) && core::inMap(ruaPara.c, ruaPara.r) && (abs(ruaDe.c - ruaPara.c) + abs(ruaDe.r - ruaPara.r)) == 1; }
  bool ancorando = false;   // o ultimo cruzamento foi decidido no modo ANCORAR (linha desconhecida)
  int ancorarBloq = 0;      // cubos que fecharam o caminho ate a vermelha sem a linha ter sido achada
  void aoBloquear() { if (ancorando) ancorarBloq++; if (ruaValida()) { core::setStreet(cubos, ruaDe, ruaPara, 0.0f); core::streetSlot(bloq, ruaDe, ruaPara) = true; } bloqueios++; core::onTurn(loc, 2); pularProximo = true; }
  bool ruaAtualNaoMedida() const { return !ruaValida() || fabsf(core::streetSlot(cubos, ruaDe, ruaPara) - bm.prior) < 0.08f; }   // a rua para onde virou ainda tem so a crenca inicial: vale olhar adiante
  void aoColetar(seguidor::CorCubo cor) { if (ruaValida()) core::setStreet(cubos, ruaDe, ruaPara, 0.0f); carga.push_back(cor); carregando = true; coletados++; }

  // Rua que ja foi vista VAZIA: la nunca mais aparece cubo (ninguem poe cubo de volta), entao qualquer eco ali so pode ser o OUTRO ROBO.
  bool ruaVaziaCerta(core::Node a, core::Node b) const {
    if (!core::inMap(a.c, a.r) || !core::inMap(b.c, b.r)) return false;
    core::CubeBelief copia = cubos;
    return core::streetSlot(copia, a, b) < core::kKnownEmpty;
  }
  // Um raio do ultrassom ao longo de uma linha de ruas a partir de `origem` na direcao (dc, dr). `andadoMm` = quanto o eixo ja andou desde a parada na origem (0 parado).
  // Com eco: as ruas ANTES dele estao vazias e a rua do eco tem cubo (a nao ser que ja fosse vazia de certeza: ali so pode ser o outro robo). Sem eco: as ruas inteiras ao alcance
  // estao vazias. Serve de INFORMACAO e de OBJETIVO: um cubo a duas ruas ja entra na memoria e o navegador vai ate la.
  void observarRaio(core::Node origem, int dc, int dr, float usMm, float andadoMm, bool comSemEco, bool deLado = false) {
    auto rua = [&](int j, core::Node* a, core::Node* b) {
      *a = core::Node{static_cast<int8_t>(origem.c + dc * j), static_cast<int8_t>(origem.r + dr * j)};
      *b = core::Node{static_cast<int8_t>(origem.c + dc * (j + 1)), static_cast<int8_t>(origem.r + dr * (j + 1))};
      return core::inMap(a->c, a->r) && core::inMap(b->c, b->r);
    };
    if (usMm > 0.0f) {
      const int k = seguidor::ruaDoEco(usMm, andadoMm);
      if (k < 0) return;   // perto de um cruzamento: ambiguo
      const int j0 = seguidor::primeiraRuaAindaAFrente(andadoMm);
      for (int j = j0; j < k; j++) { core::Node a, b; if (rua(j, &a, &b) && !ruaVaziaCerta(a, b)) core::observeStreet(cubos, a, b, false, bm); }
      core::Node a, b;
      if (rua(k, &a, &b)) {
        if (ruaVaziaCerta(a, b)) oponenteVisto++;   // rua que ja era vazia de certeza: o eco so pode ser o outro robo
        else { core::observeStreet(cubos, a, b, true, bm); ecosLongeVistos++; }
      }
    } else if (comSemEco) {
      // Leitura DE LADO na varredura: o carro gira em torno do eixo, que fica ~38 mm fora da linha da rua do lado, e o cone do ultrassom e estreito (+-10 graus): um cubo no meio da rua
      // vizinha (a ~150 mm do cruzamento) fica FORA do cone. Sem eco, a primeira rua do lado nao pode ser dada como vazia (so as de depois, que o cone alcanca).
      const int j0 = std::max(seguidor::primeiraRuaAindaAFrente(andadoMm), deLado ? 1 : 0), j1 = seguidor::ultimaRuaVistaVazia(andadoMm, 900.0f);
      for (int j = j0; j <= j1 && j < 3; j++) { core::Node a, b; if (rua(j, &a, &b)) core::observeStreet(cubos, a, b, false, bm); }
    }
  }
  int ecosLongeVistos = 0;
  int oponenteVisto = 0, esperasSeguidas = 0;
  std::vector<std::pair<core::Node, core::Node>> bloqueiosTemporarios;   // ruas com cubo que NAO da para pegar agora (carregando): voltam a valer depois da entrega
  bool desviar = false;   // acabou de voltar de um cubo no caminho: na proxima esquina vai por outra linha
  void aoBloquearTemporario() {
    desviar = true;
    if (ruaValida()) core::setStreet(cubos, ruaDe, ruaPara, 1.0f);   // sabe que tem cubo la (para pegar depois da entrega)
    if (ruaValida()) { core::streetSlot(bloq, ruaDe, ruaPara) = true; bloqueiosTemporarios.push_back({ruaDe, ruaPara}); }
    ROBO_DBG("    BLOQ-TEMP rua (%d,%d)-(%d,%d) valida=%d pose (%d,%d)\n", ruaDe.c, ruaDe.r, ruaPara.c, ruaPara.r, (int)ruaValida(), core::estimate(loc).pose.at.c, core::estimate(loc).pose.at.r);
    core::onTurn(loc, 2);
    pularProximo = true;
  }
  bool bloqTemporariosVazio() const { return bloqueiosTemporarios.empty(); }
  void liberarBloqueiosTemporarios() { for (auto& b : bloqueiosTemporarios) core::streetSlot(bloq, b.first, b.second) = false; bloqueiosTemporarios.clear(); }

  // Carregando e com o oeste bloqueado: 1 = sobe, 3 = desce, ate a linha MAIS PROXIMA (na coluna atual) cuja rua para o oeste nao tem cubo. Antes era sempre "sobe, ou desce
  // no topo", e com cubo em duas linhas vizinhas ele ficava indo e voltando entre elas (a de cima tinha cubo, a de baixo tambem). Empate: desce (o portao fica no meio da arena).
  int qDesvio(core::Node at) const {
    for (int d = 1; d < core::kMapRows; d++) {
      for (const int sinal : {-1, 1}) {
        const int r = at.r + sinal * d;
        if (r < 0 || r >= core::kMapRows || at.c < 1) continue;
        if (!core::streetSlot(bloq, core::Node{at.c, static_cast<int8_t>(r)}, core::Node{static_cast<int8_t>(at.c - 1), static_cast<int8_t>(r)})) return sinal > 0 ? 1 : 3;
      }
    }
    return at.r < core::kMapRows - 1 ? 1 : 3;
  }

  static char letra(core::Turn t) { return t == core::Turn::Left ? 'L' : t == core::Turn::Right ? 'R' : t == core::Turn::UTurn ? 'U' : 'S'; }

  // Cruzamento confirmado: atualiza a pose, olha as ruas (ultrassom na varredura), planeja. Devolve S/L/R/U e se vai sair pelo portao (entrega).
  char aoCruzamento(unsigned long agoraMs, const ResultadoGiro& v, bool* saindoPeloPortao, float rumoGraus) {
    *saindoPeloPortao = false;
    core::forget(cubos, static_cast<float>(agoraMs - ultimoForgetMs) / 1000.0f, bm);
    ultimoForgetMs = agoraMs;
    // O rumo dos encoders manda no sentido da pose: a busca (linha perdida, partida num y que nao e de rua) gira o carro sem passar por aposVirar, e a pose seguia achando que ele
    // olhava para onde entrou (a linha contada de cima virava a de baixo e o carro saia pelo portao errado).
    if (!loc.inGreen) {
      const int qRumo = ((static_cast<int>(lroundf(rumoGraus / 90.0f)) % 4) + 4) % 4;   // 0 = leste, 1 = norte, 2 = oeste, 3 = sul
      const core::Pose& pe = core::estimate(loc).pose;
      const int qPose = pe.dc == 1 ? 0 : pe.dr == 1 ? 1 : pe.dc == -1 ? 2 : 3;
      if (qRumo != qPose) core::onTurn(loc, (qRumo - qPose + 4) % 4);
    }
    const bool chegouNormal = !pularProximo && naRua && !loc.inGreen && !loc.atGate;
    if (pularProximo) pularProximo = false; else { core::onCrossing(loc); contatoNaoContado = false; }
    if (loc.inGreen) return 'S';
    const core::Estimate est = core::estimate(loc);
    const core::Pose& p = est.pose;
    if (ROBO_DEPURANDO()) { core::CubeBelief cp = cubos; fprintf(stderr, "    BELIEF t=%.1f pose (%d,%d) dir (%d,%d) massaLinha %.2f massaColuna %.2f rua(0,0)-(0,1)=%.2f (0,1)-(0,2)=%.2f\n", agoraMs / 1000.0, p.at.c, p.at.r, p.dc, p.dr, core::rowMass(loc, p.at.r), core::columnMass(loc, p.at.c), core::streetSlot(cp, core::Node{0,0}, core::Node{0,1}), core::streetSlot(cp, core::Node{0,1}, core::Node{0,2})); }
    const bool colSure = core::columnMass(loc, p.at.c) >= core::kMinCoordBelief, rowSure = core::rowMass(loc, p.at.r) >= core::kMinCoordBelief;
    // (so com a pose SEGURA: com a pose errada a rua andada e outra, e a rua do cubo ficava marcada como vazia de certeza)
    // Andou a rua inteira de ruaDe ate aqui com o ultrassom ligado e NAO parou por nenhum cubo: a rua esta vazia (a de tras da esquina nunca aparece na varredura,
    // so as da frente e dos lados, e sem isto ele voltava e voltava a mesma rua achando que talvez tivesse cubo).
    if (chegouNormal && colSure && rowSure && ruaUsada && core::inMap(ruaDe.c, ruaDe.r) && core::inMap(ruaPara.c, ruaPara.r)) {
      core::observeStreet(cubos, ruaDe, ruaPara, false, bm);
      core::observeStreet(cubos, ruaDe, ruaPara, false, bm);
    }
    if (colSure && rowSure) core::markVisited(visitas, p.at);   // so conta o que sabe onde esta
    // olha as ruas pelo ultrassom: frente, esquerda e direita. Cada leitura e um RAIO ao longo de uma linha reta de ruas (ate ~3 ruas: o eco ve cubos ALEM do proximo cruzamento).
    // -2 = nao mediu (nao girou). Os cubos so ficam em ruas, nunca no cruzamento.
    const int dirs[3][2] = {{p.dc, p.dr}, {-p.dr, p.dc}, {p.dr, -p.dc}};
    const float us[3] = {v.usFrente, v.usEsq, v.usDir};
    for (int k = 0; k < 3 && colSure && rowSure; k++) {   // sem saber onde esta, o que o ultrassom ve nao pode ir para a memoria (iria para a rua errada)
      if (us[k] == -2.0f) continue;
      observarRaio(p.at, dirs[k][0], dirs[k][1], us[k], 0.0f, true, k != 0);
    }
    core::Decision d;
    alvoX = alvoY = -1;
    ancorando = !carregando && !rowSure;
    if (!ancorando) ancorarBloq = 0;
    if (!carregando && !rowSure) {
      // Depois de um reset ele nao sabe a linha: vai ate a linha VERMELHA de um dos lados (a mais perto da linha de entrada) CONTANDO os cruzamentos que viu; ao chegar, o localizador
      // sabe a linha (cruzamentos contados + vermelho) e a velocidade sai de cada trecho de 400 mm. Nao varre: so anda reto.
      const int qAtual = ((static_cast<int>(lroundf(rumoGraus / 90.0f)) % 4) + 4) % 4;   // 0 = leste, 1 = norte, 2 = oeste, 3 = sul
      int qAlvo = (qAtual == 1 || qAtual == 3) ? qAtual : 1;                             // segue na vertical em que ja esta; se estava na horizontal, sobe
      // Cubos nos dois sentidos da coluna (sobe e desce voltavam ao mesmo cruzamento para sempre): anda UMA coluna para o leste e tenta a vertical de la.
      if (ancorarBloq >= 2) { qAlvo = 0; ancorarBloq = 0; }
      const int dq = ((qAlvo - qAtual) % 4 + 4) % 4;                                       // giros a esquerda
      ultimoPlano = fmt("ANCORAR: ate a linha vermelha de %s contando os cruzamentos (ja contou %d) e olhando longe pelo ultrassom", qAlvo == 0 ? "leste (cubos nos dois sentidos da coluna)" : qAlvo == 1 ? "cima" : "baixo", contados + 1);
      contados++;
      ruaDe = p.at;
      ruaPara = qAlvo == 0 ? core::Node{static_cast<int8_t>(p.at.c + 1), p.at.r} : core::Node{p.at.c, static_cast<int8_t>(p.at.r + (qAlvo == 1 ? 1 : -1))};
      naRua = true;
      ruaUsada = core::inMap(ruaPara.c, ruaPara.r);
      return dq == 0 ? 'S' : dq == 1 ? 'L' : dq == 3 ? 'R' : 'U';
    }
    // Antes do PRIMEIRO cubo, com a linha ja conhecida: vai ao ecoponto ver quais cores tem plataforma (so se pega cor com plataforma: coletar uma sem destino custa ~85 s de busca e -10).
    // Reusa a entrega (anda para o oeste, sai pelo portao): a diferenca e a carga vazia, e `entregarNaPlataforma` so observa.
    if (observarNoInicio && !carregando && !tentouObservar && !plataformasObservadas && colSure && rowSure) { tentouObservar = true; observando = true; carregando = true; }
    // Capacidade 2 (regras): com 1 cubo so continua coletando se ha cubo provavel; com 2 vai entregar. Entregar e obrigatorio antes de coletar de novo.
    core::Plan plano = core::planAtCrossing(loc, cubos, core::kMinCoordBelief, &visitas, &bloq);
    if (carregando) {
      // Carregando 1 cubo: ENTREGA JA, e sem depender de saber onde esta: o rumo dos encoders diz para onde aponta e o nosso portao fica a OESTE (toda rua horizontal leva a ele).
      // Em cada esquina vira para o oeste e segue por ela; a azul no fim e o nosso portao (portaoPendente fica ligado enquanto carrega).
      const int q = ((static_cast<int>(lroundf(rumoGraus / 90.0f)) % 4) + 4) % 4;   // 0 = leste, 1 = norte, 2 = oeste, 3 = sul
      int qAlvo = 2;
      if (colSure && rowSure && core::inMap(p.at.c - 1, p.at.r) && core::streetSlot(bloq, p.at, core::Node{static_cast<int8_t>(p.at.c - 1), p.at.r})) {
        qAlvo = qDesvio(p.at);   // a rua para o oeste esta bloqueada por um cubo: vai para a linha mais proxima com o oeste livre
      }
      if (colSure && rowSure && (!bloqTemporariosVazio() || desviar)) {
        // Pose segura e ruas bloqueadas por cubos: nao basta subir ou descer (a rua vertical tambem pode ter cubo, e ele ia e voltava ate o fim da partida). Menor caminho PELAS RUAS LIVRES ate
        // qualquer cruzamento da coluna 0, de onde a rua para o oeste leva ao portao (a propria rua do oeste da coluna 0 nunca esta bloqueada: e o beco).
        core::Routes rotas;
        core::routesFrom(p.at, nullptr, &bloq, rotas);
        int melhorR = -1;
        float melhorCusto = 1e9f;
        for (int r = 0; r < core::kMapRows; r++) {
          const core::Node alvo{0, static_cast<int8_t>(r)};
          if (!core::reachable(rotas, alvo)) continue;
          if (rotas.dist[0][r] < melhorCusto) { melhorCusto = rotas.dist[0][r]; melhorR = r; }
        }
        if (melhorR >= 0) {
          const core::Node alvo{0, static_cast<int8_t>(melhorR)};
          if (alvo == p.at) qAlvo = 2;
          else {
            const core::Node prox = core::firstStepTo(rotas, p.at, alvo);
            qAlvo = prox.c > p.at.c ? 0 : prox.c < p.at.c ? 2 : prox.r > p.at.r ? 1 : 3;
          }
          desviar = false;
        }
      }
      ROBO_DBG("    CARREGANDO em (%d,%d) rumo q=%d colSure=%d rowSure=%d oesteBloq=%d qAlvo=%d bloqTemp=%zu\n", p.at.c, p.at.r, q, colSure, rowSure, core::inMap(p.at.c - 1, p.at.r) ? (int)core::streetSlot(bloq, p.at, core::Node{static_cast<int8_t>(p.at.c - 1), p.at.r}) : -1, qAlvo, bloqueiosTemporarios.size());
      if (desviar) { qAlvo = qDesvio(p.at); desviar = false; }   // mesmo sem pose segura: na esquina seguinte ao cubo, sobe (ou desce) uma linha
      *saindoPeloPortao = true;
      ultimoPlano = "entregar (vai para o oeste, pela bussola)";
      alvoX = 0; alvoY = colSure && rowSure ? yDoCruzamento(p.at.r) : -1;
      ruaDe = p.at;
      ruaPara = core::Node{static_cast<int8_t>(p.at.c + (qAlvo == 0 ? 1 : qAlvo == 2 ? -1 : 0)), static_cast<int8_t>(p.at.r + (qAlvo == 1 ? 1 : qAlvo == 3 ? -1 : 0))};   // a rua que vai usar de fato (um cubo achado nela bloqueia ELA, nao a do oeste)
      naRua = true;
      ruaUsada = false;
      const int dq = ((qAlvo - q) % 4 + 4) % 4;   // giros a esquerda
      return dq == 0 ? 'S' : dq == 1 ? 'L' : dq == 3 ? 'R' : 'U';
    }
    {
      d = plano.move;
      ultimoPlano = plano.mode == core::Mode::Collect ? (d.toCube ? "coletar" : "explorar (regiao menos visitada)") : (plano.mode == core::Mode::Anchor ? "ancorar a linha" : "achar o portao");
      if (plano.mode == core::Mode::Collect && d.toCube) {
        core::Node tmp{}, ga{}, gz{};
        if (!core::nextTowardBestCube(p, cubos, &tmp, core::kKnownCube, &bloq, &ga, &gz)) core::nextTowardBestCube(p, cubos, &tmp, core::kMinBelief, &bloq, &ga, &gz);   // o mesmo alvo que decideAtCrossing escolheu (guloso: cubo visto primeiro)
        alvoX = (xDoCruzamento(ga.c) + xDoCruzamento(gz.c)) / 2; alvoY = (yDoCruzamento(ga.r) + yDoCruzamento(gz.r)) / 2;
      } else if (d.next.c >= 0 && core::inMap(d.next.c, d.next.r)) { alvoX = xDoCruzamento(d.next.c); alvoY = yDoCruzamento(d.next.r); }
    }
    ruaDe = p.at;
    ruaPara = d.next;
    naRua = true;
    ruaUsada = core::inMap(d.next.c, d.next.r);
    return letra(d.turn);
  }
};

template <class Carro>
struct Robo {
  ConfigRobo cfg;
  explicit Robo(const ConfigRobo& c = ConfigRobo{}) : cfg(c) {}
  Ajustes ajustes = normalizar(cfg.ajustes);
  AjustesCruzamento ajC = cfg.cruzamento;
  AjustesGiroCruzamento ajG;
  Memoria memoria;
  // Para onde vai o tempo: cada ciclo (tick) e cobrado da fase em curso (exclusivo: o tempo de uma fase aninhada nao conta na de fora). Mede, nao decide.
  enum class Fase : uint8_t { Seguir, Corrigir, Buscar, Cruzamento, Varre, Gesto, Vira, Cubo, Plataforma, Entrada, Borda, kNum };
  static constexpr const char* kNomesFase[static_cast<int>(Fase::kNum)] = {"reto", "corrige", "busca", "cruzamento", "varre", "gesto", "vira", "cubo", "plataforma", "entrada", "borda"};
  unsigned long msFase[static_cast<int>(Fase::kNum)] = {};
  Fase fase = Fase::Seguir;
  unsigned long marcaFase = 0;
  void cobrar(unsigned long agora) { if (marcaFase) msFase[static_cast<int>(fase)] += agora - marcaFase; marcaFase = agora; }
  struct Fatia {   // na entrada de uma rotina bloqueante: tudo o que ela gasta vai para `f`, e a fase de fora volta ao sair
    Robo& r; Carro& c; Fase ant;
    Fatia(Robo& r_, Carro& c_, Fase f) : r(r_), c(c_), ant(r_.fase) { r.cobrar(c.agoraMs()); r.fase = f; }
    ~Fatia() { r.cobrar(c.agoraMs()); r.fase = ant; }
  };
  std::string resumoFases() const {
    unsigned long total = 0;
    for (const unsigned long ms : msFase) total += ms;
    std::string t;
    for (int k = 0; k < static_cast<int>(Fase::kNum); k++) {
      char b[48];
      snprintf(b, sizeof b, "%s%s %.0f%%", k ? " | " : "", kNomesFase[k], total ? 100.0 * msFase[k] / total : 0.0);
      t += b;
    }
    return t;
  }
  // Pedir ajuda: pegar o carro e poe-lo de volta no verde e permitido (custa o reinicio), e vagar procurando custa a partida inteira (60% do tempo no simulador). Sem nenhuma META
  // (coletar, entregar, observar as plataformas, passar o portao) por kMaxSemMetaMs ele PARA e apita ate o verde aparecer sob o sensor (o `VERDE inesperado` de sempre o reposiciona).
  static constexpr unsigned long kMaxSemMetaMs = 150000;
  bool esperandoAjuda = false;
  int pedidosDeAjuda = 0, marcoMetas = 0;
  unsigned long ultimaMetaMs = 0;
  bool tentarLados = true;
  bool portaoAteAzulAcabar = true;   // a travessia do portao so termina quando o TCS deixou o azul (antes: branco numa ponta + qualquer preto da propria faixa ja bastava)
  bool zerarSaindoNoFim = true;
  bool ancorarSemVarrer = true;
  bool portaoSoMeio = true;
  bool portaoSoMeio2 = true;   // so os 2 sensores do centro: a rua 20-30 mm ao lado (entrada com erro de y) nao encerra a travessia e o seguidor nao gira no eixo
  bool ancorarSemRepetir = true;
  bool olharSoSeNaoMedida = true;   // depois de virar, o olhar de 35 graus (~1,4 s) so quando a rua nova ainda nao foi medida pelo ultrassom
  static constexpr double kRetoParaReancorarMm = 150.0;
  double retoDesdeMm = -1.0;   // odometro quando comecou a andar reto sobre a linha (-1 = nao esta nisso)
  static constexpr double kMaxVerdeSemLinhaMm = 700.0;
  double semLinhaNoVerdeMm = -1.0;   // odometro quando comecou a andar no verde sem ver linha (-1 = nao esta nisso)
  const char* ultimaAcao = "";   // o que a decisao mandou no ultimo ciclo (telemetria e rastro do simulador)
  bool ultimaBarra[kSensores] = {};
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
  Navegador nav;                 // missao completa (localizador + mapa de cubos + exploracao + coleta + entrega); desligado = os cenarios de teste de sempre
  unsigned long ultimoTcsMs = 0, ultimoUsMs = 0, esperaAte = 0;
  float usAnt = -1.0f, velMmSAtual = 0.0f;
  int verdesSeguidos = 0, proibidasSeguidas = 0, semVarreduras = 0, azulSeguidas = 0;
  bool cruzandoPortao = false;
  unsigned long cruzandoPortaoDesde = 0;
  bool varrerSoQuandoPrecisa = true;   // com a pose segura e o cruzamento esperado, so gira para o lado cuja rua ainda nao foi medida (as duas medidas: nao gira; so uma: gira uma vez para ela)
  bool pularVarreduraComPoseSegura = false;   // opcional: no robo real o giro nao esbarrava nos cubos (ficam no meio da rua), entao varre sempre
  unsigned long ultimaObsRaioMs = 0;
  bool ecoRealinhado = false;   // ja realinhou com a rua uma vez para reler o eco lembrado
  float ecoLembradoMm = -1.0f, ecoLembradoAndadoMm = 0.0f;   // ultimo eco de cubo perto (tratarCubo) e quanto da rua ja tinha andado quando o viu
  bool precisaAlinhar = false, velRetaMedida = false, inicioFeito = false;   // acabou de ser reposicionado no verde
  unsigned long semReacaoAte = 0;
  // Calibracao da velocidade pela DISTANCIA entre cruzamentos: o carro sem encoder nao sabe quanto andou; mas de um cruzamento ao proximo em linha reta sao 400 mm.
  unsigned long tSaidaMs = 0;
  float odoCruz(const Carro& c) const { return cfg.encoder ? static_cast<float>(c.odoMm()) : -1.0f; }   // odometro para o debounce do cruzamento (sem encoder: so o tempo)
  double minMovMm() const { return cfg.encoder ? 4.0 : 20.0; }   // menor deslocamento que vale a pena andar para corrigir o y (sem encoder a partida come ~20 mm)
  double odoSaida = 0.0, odoAnt = 0.0;   // odometro (mm) ao sair do cruzamento / no ciclo anterior
  // Quanto o carro andou desde que saiu do cruzamento. Com encoder e a leitura das rodas; sem, tempo x velocidade (que erra com partida, correcao e parada).
  double andadoDesdeSaida(const Carro& c, unsigned long agora) const {
    return cfg.encoder ? c.odoMm() - odoSaida : (agora - tSaidaMs) / 1000.0 * (velLivreMmS > 0 ? velLivreMmS : ajC.velMmS);
  }
  float andadoRuaMm = 0.0f;   // quanto ANDOU (velocidade x tempo so enquanto avanca, sem as paradas e giros) desde que saiu do cruzamento para a rua atual
  double trechoRestante0 = -1.0;   // quanto faltava para o proximo cruzamento ao sair (mm); -1 = nao sei (meia-volta no meio da rua)
  bool medindoVel = false;
  int amostrasVel = 0;
  float ultimaVelMedida = 0.0f, velLivreMmS = 0.0f;
  static constexpr int kVerdesParaReposicionar = 3;

  // ---- Rumo: o carro nao tem bussola; o giro vem da diferenca dos percursos das duas rodas (encoder do eixo, nao ve patinagem), + = esquerda
  double odoEAnt = 0.0, odoDAnt = 0.0;   // odometros das rodas no ciclo anterior
  bool rumoIniciado = false;
  float rumo = 0.0f;   // rumo em graus (+ esquerda, 0 = de frente para dentro da cidade), contado pelos encoders
  float rumoAoPerder = 0.0f;
  int reposicionados = 0;

  std::function<void()> ceder;   // quando ha outro robo na cena: devolve a vez a cada ciclo (os dois andam juntos)

  // Avanca um ciclo e atualiza o rumo pelos encoders (o primeiro ciclo so guarda os odometros de partida).
  void tick(Carro& c) {
    c.avancar();
    cobrar(c.agoraMs());
    if (!rumoIniciado) { odoEAnt = c.odoEMm(); odoDAnt = c.odoDMm(); rumoIniciado = true; }
    const double dE = c.odoEMm() - odoEAnt, dD = c.odoDMm() - odoDAnt;
    odoEAnt = c.odoEMm(); odoDAnt = c.odoDMm();
    rumo += static_cast<float>((dD - dE) / ajustes.bitolaMm * 180.0 / M_PI);   // a bitola que o FIRMWARE supoe (a real pode diferir)
    if (ceder) ceder();
  }

  static void apaga(int& v, float x) { v = static_cast<int>(x); }

  // A BARRA e a referencia principal; o TCS do chao so confirma cor. O verde (e o azul do portao) e ESCURO no infravermelho: sobre o verde quase todos os sensores leem preto, e isso e chao,
  // nao linha nem cruzamento: fica mascarado so ENQUANTO nada indica o portao. Assim que o TCS le a faixa azul a barra passa a valer crua (a faixa e vista como linha, que o fluxo do portao
  // espera), e o primeiro momento com branco na barra depois dela (`saiuDoVerde`) e o carro entrando no tabuleiro: vale na hora, sem esperar o TCS aceitar outra cor (duas leituras de 60 ms,
  // que faziam o carro passar direto da primeira rua). A mascara antes ficava no hardware, atrelada ao verde ACEITO pelo TCS.
  bool emVerdeAnt = false, viuAzulNoVerde = false, saiuDoVerde = false;
  void lerBarra(Carro& c, bool p[kSensores]) {
    c.ler(p);
    const bool noVerde = nav.ativo && (nav.loc.inGreen || nav.entregando);
    if (noVerde && !emVerdeAnt) { viuAzulNoVerde = false; saiuDoVerde = false; }
    emVerdeAnt = noVerde;
    if (!noVerde || saiuDoVerde) return;
    if (azulSeguidas > 0) viuAzulNoVerde = true;
    if (viuAzulNoVerde) {
      if (acesos(p) < kSensores - 2) saiuDoVerde = true;   // branco na barra depois da faixa azul: entrou no tabuleiro
      return;
    }
    if (acesos(p) >= kSensores - 2) for (int i = 0; i < kSensores; i++) p[i] = false;   // chao verde (escuro no IR)
  }
  static int acesos(const bool p[kSensores]) {
    int n = 0;
    for (int i = 0; i < kSensores; i++) n += p[i] ? 1 : 0;
    return n;
  }

  // Gira no eixo ate o angulo ESTIMADO (por tempo) chegar em alvo (espelho de girarAte do .ino). Guarda tambem o angulo VERDADEIRO de cada amostra.
  // pelaLinha: o giro de rua fecha na LINHA, nao no angulo (girarNaLinha).
  float girarAte(Carro& c, float agora, float alvo, std::vector<AmostraGiro>& est, std::vector<AmostraGiro>& ver, double thRef, float rumoRef = 0.0f, bool pelaLinha = false) {
    if (pelaLinha) return girarNaLinha(c, agora, alvo, est, ver, thRef, rumoRef);
    const bool esq = alvo > agora;
    const int v = ajustes.velBuscaExterna;
    c.comandar(esq ? -v : v, esq ? v : -v);
    const unsigned long inicio = c.agoraMs();
    float omega = 0.0f, rumoAnt = rumo;   // graus/s pelos encoders
    // com encoder (servo em malha fechada, sem zona morta) a roda leva ~tauMs para parar: o giro que ainda sobra na freada (omega x tau, ~4 graus) fazia o rumo real errar o alvo
    auto previsto = [&] { return cfg.encoder ? agora + omega * static_cast<float>(cfg.tauMs) / 1000.0f : agora; };
    while ((esq ? previsto() < alvo : previsto() > alvo) && c.agoraMs() - inicio < 3000) {
      tick(c);
      bool p[kSensores];
      lerBarra(c, p);
      omega = (rumo - rumoAnt) / static_cast<float>(cfg.dtMs / 1000.0);
      rumoAnt = rumo;
      agora = rumo - rumoRef;   // angulo pelos encoders
      const float verdade = static_cast<float>((c.estado().th - thRef) * 180.0 / M_PI);
      est.push_back({agora, p[0], p[7], acesos(p)});
      ver.push_back({verdade, p[0], p[7], acesos(p)});
    }
    c.parar();
    if (cfg.encoder) {   // espera parar de girar: o que sobra entra na estimativa
      for (int i = 0; i < 80 && fabsf(omega) > 2.0f; i++) { tick(c); omega = (rumo - rumoAnt) / static_cast<float>(cfg.dtMs / 1000.0); rumoAnt = rumo; agora = rumo - rumoRef; }
    }
    return agora;
  }

  // Giro de rua. A barra gira em torno do eixo a ~30 mm dele, entao os sensores do centro veem a rua nova numa FAIXA de +-24 graus em volta da direcao dela: parar na primeira vez
  // que a veem deixa o carro de ate 24 graus de lado. Gira DIRETO pela faixa (marca o angulo contado pelos encoders na entrada e na saida), para, volta ao MEIO dela e so ai o rumo vira
  // o multiplo de 90 certo da grade (o erro do angulo dos encoders so conta na meia faixa, ~2 graus, e nao se acumula de um giro para o outro).
  float girarNaLinha(Carro& c, float agora, float alvo, std::vector<AmostraGiro>& est, std::vector<AmostraGiro>& ver, double thRef, float rumoRef) {
    const bool esq = alvo > agora;
    const int v = ajustes.velBuscaExterna;
    c.comandar(esq ? -v : v, esq ? v : -v);
    const unsigned long inicio = c.agoraMs();
    float omega = 0.0f, rumoAnt = rumo, aEntrada = 0.0f, aSaida = 0.0f;
    bool dentro = false, saiu = false;
    while (c.agoraMs() - inicio < 4000) {
      tick(c);
      bool p[kSensores];
      lerBarra(c, p);
      omega = (rumo - rumoAnt) / static_cast<float>(cfg.dtMs / 1000.0);
      rumoAnt = rumo;
      agora = rumo - rumoRef;
      est.push_back({agora, p[0], p[7], acesos(p)});
      ver.push_back({static_cast<float>((c.estado().th - thRef) * 180.0 / M_PI), p[0], p[7], acesos(p)});
      if (fabsf(agora) < 0.7f * fabsf(alvo)) continue;   // ainda saindo da rua de onde veio (com a barra a 15 mm ela cobre a rua de origem ate ~45 graus: 0,5 fechava nela)
      const bool centro = p[3] && p[4];
      if (!dentro && centro) { dentro = true; aEntrada = agora; }
      else if (dentro && !centro) { aSaida = agora; saiu = true; break; }
      if (fabsf(agora) > fabsf(alvo) + 60.0f) break;     // passou do giro e nao viu rua: desiste
    }
    c.parar();
    for (int i = 0; i < 80 && fabsf(omega) > 2.0f; i++) { tick(c); omega = (rumo - rumoAnt) / static_cast<float>(cfg.dtMs / 1000.0); rumoAnt = rumo; agora = rumo - rumoRef; }
    if (!saiu) return agora;   // sem rua vista: fica com o angulo dos encoders
    const float meio = (aEntrada + aSaida) / 2.0f;
    std::vector<AmostraGiro> e2, v2;
    if (fabsf(meio - agora) >= 2.0f) girarAte(c, 0.0f, meio - agora, e2, v2, c.estado().th, rumo);   // volta ao meio da faixa (sem fechar na linha de novo)
    // Cada giro de rua e uma medida da bitola efetiva: a rua esta a |alvo| graus de verdade e os encoders contaram |meio|. A derrapagem (e a bitola real) mudam com o piso, entao a bitola
    // que converte diferenca de rodas em graus se corrige sozinha, giro a giro (media movel: o meio da faixa tem ~3 graus de ruido).
    if (cfg.autoCalBitola && fabsf(alvo) >= 80.0f) {
      const float razao = std::min(1.4f, std::max(0.7f, fabsf(meio) / fabsf(alvo)));
      ajustes.bitolaMm *= 1.0f + 0.5f * (razao - 1.0f);
      ROBO_DBG("    BITOLA-EFETIVA: contou %.1f para %.0f -> %.1f mm\n", meio, alvo, ajustes.bitolaMm);
    }
    const float depois = 90.0f * roundf(rumoRef / 90.0f) + 90.0f * roundf(alvo / 90.0f);
    ROBO_DBG("    GIRO-LINHA: faixa %.1f..%.1f (meio %.1f) pedido %.0f, real %.1f: rumo %.1f -> %.0f\n", aEntrada, aSaida, meio, alvo, (c.estado().th - thRef) * 180.0 / M_PI, rumo, depois);
    rumo = depois;
    return rumo - rumoRef;
  }

  void pararNoCruzamento(Carro& c, bool preto[kSensores]) {
    Fatia fatia(*this, c, Fase::Cruzamento);
    c.parar();
    paradaEm = c.estado();
    esquecerCor(memCor);
    CorLinha tres[3];   // o TCS leva ~24 ms por leitura: le 3 vezes PARADO em cima do que acendeu as pontas; so acredita se as 3 concordam (uma leitura errada vira cruzamento, que o giro desmente)
    for (int i = 0; i < 3; i++) { for (int k = 0; k < 6; k++) tick(c); tres[i] = c.tcsChao(); }
    anotarCor(memCor, corConfirmada(tres, 3), c.agoraMs());
    const CorLinha corAgora = corRecente(memCor, c.agoraMs(), kJanelaCorMs);
    AcaoCruzamento acao = acaoNaPonta(corAgora, nav.ativo ? nav.carregando : temCubo, portaoPendente || (corAgora == CorLinha::Azul && indoParaOeste()));
    if (acao == AcaoCruzamento::Portao) {   // nosso portao azul: segue em frente
      portoes++;
      portaoPendente = false;
      semReacaoAte = c.agoraMs() + 1500;   // ainda esta em cima da faixa azul do portao: nao e para reagir
      c.evento(fmt("PORTAO azul em (%.0f,%.0f) rumo %.0f%s", c.estado().x, c.estado().y, rumo, nav.carregando ? " saindo com cubo" : ""));
      esquecerCor(memCor);
      esquecerUltimoComando(memoria);
      tUltimoCruzamento = c.agoraMs();
      comecarASair(cruz, c.agoraMs(), odoCruz(c));
      if (nav.ativo) {
        if (nav.carregando) entregarNaPlataforma(c);   // saiu COM cubo pelo portao: leva ate a plataforma da cor
        else if (indoParaOeste()) { core::initInGreen(nav.loc); nav.naRua = false; portaoPendente = true; precisaAlinhar = true; }   // saiu pelo nosso portao sem querer: esta no verde de novo
        else {
          nav.aoPortao();
          trechoRestante0 = -1.0;
          // A barra para em cima da faixa azul e a barra/eixo ainda estao dentro dela: seguir a linha dali o fazia SEGUIR A AZUL como se fosse rua (subia ou descia por ela). Atravessa em
          // frente, segurando o rumo pelos encoders, ate a barra sair da faixa (20 mm + a barra a ~30 mm do eixo).
          cruz = MemoriaCruzamento{};
          comecarASair(cruz, c.agoraMs(), odoCruz(c));   // o debounce conta desde ANTES de andar: o 1o cruzamento fica a ~90 mm do portao e nao pode ser engolido
          // Em frente ate sair da faixa azul E achar a rua (algum sensor preto sem azul sob o TCS), no maximo ~1,3 s: se a entrada errou uns 40 mm de y a rua nao esta sob a barra, e
          // a busca achava a faixa azul ATRAS (a mais perto) e a seguia como rua.
          const unsigned long t0 = c.agoraMs();
          andarMm(c, 400.0, [&] {
            const unsigned long dt = c.agoraMs() - t0;
            bool pp[kSensores];
            lerBarra(c, pp);
            const bool algumPreto = pp[0] || pp[1] || pp[2] || pp[3] || pp[4] || pp[5] || pp[6] || pp[7];
            const bool noMeio = portaoSoMeio2 ? (pp[3] || pp[4]) : (pp[2] || pp[3] || pp[4] || pp[5]);   // so as pontas acesas logo depois da faixa azul e a sobra da faixa (ou a rua 30 mm ao lado): nao e a rua sob a barra
            return dt >= 130 && c.tcsChao() != CorLinha::Azul && ((portaoSoMeio ? noMeio : algumPreto) || dt >= 1300);
          });
          esquecerUltimoComando(memoria);
          memoria.perdeuEm = 0;
          // Se a travessia acabou justamente em cima de uma rua que cruza (as 2 pontas pretas), esse E o 1o cruzamento: o "saindo" armado antes de andar o engolia (ficava ligado
          // enquanto a barra estava no preto) e o carro passava direto pela coluna 0, contando a seguinte como a primeira (pose uma coluna adiante pelo resto da partida).
          bool fim[kSensores];
          lerBarra(c, fim);
          if (zerarSaindoNoFim && pontasJuntas(fim)) cruz = MemoriaCruzamento{};
        }
      }
      return;
    }
    ROBO_DBG("    PARADA[andado %.0f restante0 %.0f pularProx %d th %.0f] t=%.1fs pos (%.0f,%.0f) cor=%d acao=%d portaoPend=%d inGreen=%d\n", andadoDesdeSaida(c, c.agoraMs()), trechoRestante0, (int)nav.pularProximo, c.estado().th * 57.3, c.agoraMs() / 1000.0, c.estado().x, c.estado().y, static_cast<int>(corAgora), static_cast<int>(acao), portaoPendente, nav.loc.inGreen);
    // Com o odometro, uma faixa de cruzamento a mais de 150 mm do ponto onde o proximo DEVIA estar nao e a chegada: e o cruzamento de onde saiu (o carro recuou ou balancou por cima
    // dele, por exemplo ao pegar um cubo logo na saida da curva). Contar como chegada avancava a pose uma rua sem o carro sair do lugar e dai em diante tudo ficava deslocado.
    // Chegada CEDO = limite do tabuleiro (mapa oficial): entre dois cruzamentos sao 400 mm, mas a borda (vermelha em cima/embaixo, azul dos lados) fica a so ~168 mm do ultimo cruzamento. Uma faixa
    // que fecha a barra depois de ~168 mm numa rua de 400 mm e a borda, qualquer que seja a cor lida (o branco sombreado ja foi lido como vermelho e como amarelo). A cor so diz QUAL borda quando a
    // distancia nao decide; o eixo da pose diz (andando em +c a borda e a azul do outro lado, senao a vermelha). Abaixo de ~108 mm e o cruzamento de onde saiu (tratado logo abaixo).
    CorLinha corBorda = corAgora;
    if (cfg.encoder && nav.ativo && !nav.loc.inGreen && !nav.loc.atGate && !portaoPendente && !indoParaOeste() && acao == AcaoCruzamento::Cruzamento && trechoRestante0 >= 300.0) {
      const double andadoAteAqui = andadoDesdeSaida(c, c.agoraMs());
      if (andadoAteAqui >= kBeco - 60.0 && andadoAteAqui < trechoRestante0 - 150.0) {
        corBorda = core::estimate(nav.loc).pose.dc > 0 ? CorLinha::Azul : CorLinha::Vermelho;
        acao = acaoNaPonta(corBorda, nav.carregando, false);
        c.evento(fmt("chegou CEDO (%.0f mm de %.0f): limite do tabuleiro", andadoAteAqui, trechoRestante0));
      }
    }
    // Ancorando (trecho desconhecido) o limite e o menor trecho que nao e o mesmo cruzamento: ~60 mm (o 1o cruzamento depois do portao fica a ~130 mm); sem isso a faixa do cruzamento que acabou de deixar contava duas vezes.
    const double minAndado = trechoRestante0 > 0.0 ? trechoRestante0 - 150.0 : (ancorarSemRepetir && !nav.pularProximo ? 60.0 : -1.0);
    if (cfg.encoder && nav.ativo && !nav.loc.inGreen && acao == AcaoCruzamento::Cruzamento && andadoDesdeSaida(c, c.agoraMs()) < minAndado) {
      ROBO_DBG("    (faixa do cruzamento de onde saiu, andado %.0f de %.0f: nao e chegada)\n", andadoDesdeSaida(c, c.agoraMs()), trechoRestante0);
      comecarASair(cruz, c.agoraMs(), odoCruz(c));
      return;
    }
    if (nav.ativo && acao == AcaoCruzamento::Navegador) acao = AcaoCruzamento::MeiaVolta;   // com o navegador ligado ele mesmo decide a rota: na borda so da meia-volta
    if (acao != AcaoCruzamento::Cruzamento) {   // borda do mapa: nao gira para confirmar
      bordas++;
      c.evento(fmt("BORDA %s em (%.0f,%.0f): meia-volta", corBorda == CorLinha::Azul ? "azul" : "vermelha", c.estado().x, c.estado().y));
      if (acao == AcaoCruzamento::MeiaVolta) {
        std::vector<AmostraGiro> est, ver;
        if (nav.ativo) nav.aoProibido(corBorda == CorLinha::Azul ? core::Forbidden::OtherBlue : core::Forbidden::Red);
        girarAte(c, 0.0f, 180.0f, est, ver, c.estado().th, rumo);
        meiasVoltas++;
                esquecerUltimoComando(memoria);
        esquecerCor(memCor);
        cruz = MemoriaCruzamento{};   // depois dos 180 graus a barra ja nao esta sobre a faixa da borda: o cruzamento do beco (a 127 mm) tem de ser detectado, o debounce de 300 ms o engolia
      } else {
        paradasNavegador++;
        parado = true;
      }
      tUltimoCruzamento = c.agoraMs();
      return;
    }
    // Chegou a um cruzamento depois de sair do ANTERIOR em linha reta (sem virar): a distancia e de 400 mm (o eixo para sempre no mesmo ponto antes do centro), entao
    // 400 mm / tempo andando = velocidade REAL. Media movel; vira a velocidade usada para tempos de perda de linha, fim de beco e deslocamentos fora da cidade.
    if (medindoVel && !cfg.encoder && tSaidaMs != 0 && !nav.loc.inGreen) {   // com encoder a velocidade e lida das rodas: nao precisa medir por tempo entre cruzamentos
      const float dtS = static_cast<float>(c.agoraMs() - tSaidaMs) / 1000.0f;
      const float v = static_cast<float>(kPasso) / dtS;
      if (dtS > 0.3f && v > 80.0f && v < 600.0f) {
        ajC.velMmS = amostrasVel == 0 ? v : 0.7f * ajC.velMmS + 0.3f * v;
        ultimaVelMedida = v;
        if (!velRetaMedida) velLivreMmS = std::max(v, velLivreMmS * 0.98f);   // (se a velocidade reta foi medida entre as molduras vermelhas, ela vale e nao decai)   // a mais rapida das ultimas (decai devagar): as medicoes incluem partida, correcoes e parada; o deslocamento SEM linha anda reto, perto da maxima
        amostrasVel++;
        ROBO_DBG("    VEL amostra %.0f mm/s (media %.0f, livre %.0f)\n", v, ajC.velMmS, velLivreMmS);
      }
    }
    medindoVel = false;
    // As duas pontas acesas, em QUALQUER situacao (inclusive perdido/buscando): varredura de 90 graus a partir do lado menos explorado
    const bool vinhaSeguindo = vinhaSeguindoLinha(memoria, c.agoraMs(), ajustes);
    const int lado = ladoMenosExplorado(expl, rumo, memoria.ultimoLado);
    const double th0 = c.estado().th;   // angulo verdadeiro medido a partir de como chegou
    const float rumo0 = rumo;
    // Girar 90 graus no cruzamento VARRE o corpo do carro (~130 mm de raio) por cima das ruas do lado, e um cubo a 112-140 mm do centro seria atingido. Com a pose SEGURA e o
    // cruzamento ESPERADO (andou ~400 mm desde o anterior) nao se gira: S1+S8 e a cor bastam. As ruas do lado entram na memoria pelo eco ao longe e ao passar. Varre so
    // perdido, em duvida, quando o cruzamento nao era esperado, e na calibracao pos-reset.
    bool pularVarredura = false;
    int soLadoMedir = 0;   // +1 = so a esquerda, -1 = so a direita, 0 = os dois
    bool gesto = false;    // pose segura + odometro conferindo: so uma rodadinha para um lado (a rua do outro lado vem do mapa)
    if ((pularVarreduraComPoseSegura || varrerSoQuandoPrecisa) && nav.ativo && nav.calibrando == 0 && vinhaSeguindo && !nav.loc.inGreen && !nav.pularProximo) {
      const core::Estimate es = core::estimate(nav.loc);
      const bool segura = core::columnMass(nav.loc, es.pose.at.c) >= core::kMinCoordBelief && core::rowMass(nav.loc, es.pose.at.r) >= core::kMinCoordBelief;
      const double andado = andadoDesdeSaida(c, c.agoraMs());
      const bool esperado = segura && trechoRestante0 > 0.0 && fabs(andado - trechoRestante0) < (cfg.encoder ? 40.0 : 110.0);   // com encoder o cruzamento esperado e conferido pelo odometro
      if (esperado && cfg.encoder && cfg.gestoGraus != 0.0) { pularVarredura = true; gesto = cfg.gestoGraus > 0.0; }   // gesto < 0: passa direto, sem girar
      else if (esperado && pularVarreduraComPoseSegura) pularVarredura = true;
      else if (esperado) {
        // A rua do lado ja foi MEDIDA se a crenca saiu do prior (vista vazia ou com cubo) ou se nao existe (fora do mapa: o beco do portao). So essas dispensam o giro.
        const core::Node cur{static_cast<int8_t>(es.pose.at.c + es.pose.dc), static_cast<int8_t>(es.pose.at.r + es.pose.dr)};
        if (core::inMap(cur.c, cur.r)) {
          auto precisaMedir = [&](int dc2, int dr2) {
            const core::Node z{static_cast<int8_t>(cur.c + dc2), static_cast<int8_t>(cur.r + dr2)};
            if (!core::inMap(z.c, z.r)) return false;
            return fabsf(core::streetSlot(nav.cubos, cur, z) - nav.bm.prior) < 0.08f;
          };
          const bool esq = precisaMedir(-es.pose.dr, es.pose.dc), dir = precisaMedir(es.pose.dr, -es.pose.dc);
          if (!esq && !dir) pularVarredura = true;
          else if (esq != dir) soLadoMedir = esq ? 1 : -1;
        }
      }
    }
    // ANCORAR (linha desconhecida, indo reto pela coluna ate a moldura vermelha): a coluna e conhecida e o cruzamento chegou a ~400 mm do anterior, entao e o proximo no da coluna, nao uma
    // falsa faixa; o giro de varredura (~4 s por no) so olha as ruas laterais, e na entrada o que importa e chegar a moldura. Passa direto, como com a pose segura.
    if (!pularVarredura && ancorarSemVarrer && nav.ativo && nav.ancorando && vinhaSeguindo && !nav.loc.inGreen && !nav.pularProximo && !nav.loc.atGate) {
      const core::Estimate es = core::estimate(nav.loc);
      const double andado = andadoDesdeSaida(c, c.agoraMs());
      if (core::columnMass(nav.loc, es.pose.at.c) >= core::kMinCoordBelief && fabs(andado - kPasso) < 40.0) { pularVarredura = true; gesto = false; }
    }
    float estimado = 0.0f;   // angulo girado pelos encoders desde o inicio da varredura
    if (pularVarredura) {
      iniciarVarredura(varr, lado, vinhaSeguindo);
      varr.ruas = ResultadoGiro{true, 1, 1, false, false, -2.0f, -2.0f, static_cast<float>(c.us())};   // -2 = "nao medi" (sem giro): so a frente tem leitura
      varr.resultado = Oito::Cruzamento;
      varr.fase = Varredura::Pronta;
      semVarreduras++;
      if (gesto) { Fatia giroF(*this, c, Fase::Gesto); const float r0 = rumo; girarPara(c, r0 + lado * static_cast<float>(cfg.gestoGraus)); girarPara(c, r0); }   // rodadinha para o lado menos explorado e volta reto
    } else {
      // Entregando (carregando, indo para o oeste) ele so precisa CONFIRMAR o cruzamento e se virar para a rua do oeste: gira uma vez, para o lado do oeste (uma rua ali ja e cruzamento).
      // Completa (os dois lados) quando precisa VER as ruas (ultrassom) ou escolher uma.
      bool soUmLado = false;
      int ladoVarredura = lado;
      if (nav.ativo && nav.carregando && vinhaSeguindo && !nav.loc.inGreen && !nav.desviar && nav.bloqueiosTemporarios.empty()) {   // com desvio/rua bloqueada ele PRECISA ver: varredura completa
        const double sr = sin(rumo * M_PI / 180.0);
        if (fabs(sr) > 0.7) { soUmLado = true; ladoVarredura = sr > 0 ? 1 : -1; }   // indo para o norte o oeste fica a esquerda; para o sul, a direita
      }
      if (!soUmLado && soLadoMedir != 0) { soUmLado = true; ladoVarredura = soLadoMedir; }
      iniciarVarredura(varr, ladoVarredura, vinhaSeguindo, soUmLado);
      AmostraGiro leitura{0.0f, preto[0], preto[7], acesos(preto), static_cast<float>(c.us())};
      Fatia giroF(*this, c, Fase::Varre);   // a varredura gira nos dois lados: e tempo de giro, nao de parada
      for (int i = 0; i < 4000; i++) {   // 20 s no maximo
        const ComandoVarredura k = passoVarredura(varr, leitura, c.agoraMs(), ajustes.velBuscaExterna, ajG, bufVarr, kBufVarr);
        c.comandar(k.esq, k.dir);
        if (k.terminou) break;
        tick(c);
        bool p[kSensores];
        lerBarra(c, p);
        estimado = rumo - rumo0;
        leitura = {estimado, p[0], p[7], acesos(p), static_cast<float>(c.us())};
      }
    }
    c.parar();
    giroDriftGraus = static_cast<float>(remainder(c.estado().th - th0, 2 * M_PI) * 180.0 / M_PI);
    const ResultadoGiro& r = varr.ruas;
    ruaEsq = r.ruaEsquerda;
    ruaDir = r.ruaDireita;
    ruaEsqVerdade = ruaDirVerdade = -1;
    girou = true;
    ultimoOito = varr.resultado;
    ROBO_DBG("  [t=%.1fs] %s", c.agoraMs() / 1000.0, pularVarredura ? "(SEM GIRO) " : "(GIROU) ");
    ROBO_DBG("parou em (%.0f, %.0f) rumo %.0f, %s, 1o lado %s: ruas esq %d dir %d -> %s\n", paradaEm.x, paradaEm.y, paradaEm.th * 57.29578, vinhaSeguindo ? "seguindo" : "PERDIDO", lado > 0 ? "esq" : "dir", r.ruaEsquerda, r.ruaDireita,
                                       varr.resultado == Oito::Cruzamento ? "CRUZAMENTO" : (varr.resultado == Oito::Endireitou ? "ENDIREITOU" : "falso alarme"));
    if (varr.resultado == Oito::Cruzamento) { cruzamentos++; registrarCruzamento(expl, rumo); }
    else if (varr.resultado == Oito::Endireitou) { endireitou++; if (nav.ativo) nav.aposVirar(varr.ladoEndireitou > 0 ? 'L' : 'R'); trechoRestante0 = -1.0; tSaidaMs = c.agoraMs(); odoSaida = c.odoMm(); andadoRuaMm = 0.0f; }
    else falsosAlarmes++;
    esquecerUltimoComando(memoria);
    if (varr.resultado == Oito::Endireitou) memoria.perdeuEm = 0;   // estava de lado em cima da linha e agora esta alinhado: volta a seguir
    tUltimoCruzamento = c.agoraMs();
    fimDeBecoAvisado = false;
    if (varr.resultado == Oito::Cruzamento) {
      char a = rotaI < rota.size() ? rota[rotaI++] : 'S';
      if (nav.ativo) {
        bool saindo = false;
        a = nav.aoCruzamento(c.agoraMs(), varr.ruas, &saindo, rumo);
        if (ROBO_DEPURANDO() && !nav.loc.inGreen) {   // depuracao: a pose estimada bate com o cruzamento onde o carro realmente esta?
          const double rx = paradaEm.x + 38.0 * cos(paradaEm.th), ry = paradaEm.y + 38.0 * sin(paradaEm.th);
          const int rc = static_cast<int>(lround((rx - kBeco) / kPasso)), rr = static_cast<int>(lround((ry - kBeco) / kPasso));
          const core::Estimate es = core::estimate(nav.loc);
          const int th90 = static_cast<int>(lround(paradaEm.th * 2.0 / M_PI)) & 3, thEst = es.pose.dc == 1 ? 0 : es.pose.dr == 1 ? 1 : es.pose.dc == -1 ? 2 : 3;
          fprintf(stderr, "    POSE t=%.0fs estimada (%d,%d) rumo %d | real (%d,%d) rumo %d %s\n", c.agoraMs() / 1000.0, es.pose.at.c, es.pose.at.r, thEst, rc, rr, th90, (rc == es.pose.at.c && rr == es.pose.at.r && th90 == thEst) ? "" : "<<< ERRADA");
        }
        if (saindo) portaoPendente = true;   // vai sair pelo nosso portao de proposito: a azul nao e borda
        c.evento(fmt("plano: %s | seguir '%c' (pose %s, coluna %.0f%%, linha %.0f%%)", nav.ultimoPlano.c_str(), a, nav.loc.inGreen ? "no verde" : fmt("(%d,%d)", core::estimate(nav.loc).pose.at.c, core::estimate(nav.loc).pose.at.r).c_str(), nav.loc.inGreen ? 0.0 : 100.0 * core::columnMass(nav.loc, core::estimate(nav.loc).pose.at.c), nav.loc.inGreen ? 0.0 : 100.0 * core::rowMass(nav.loc, core::estimate(nav.loc).pose.at.r)));
      }
      medindoVel = a == 'S';   // saiu em linha reta: o proximo cruzamento esta a 400 mm
      {
        Fatia giroF(*this, c, Fase::Vira);
        if (a == 'L' || a == 'R') { virarParaRua(c, a == 'L'); if (nav.ativo) nav.aposVirar(a); }
        else if (a == 'U') { std::vector<AmostraGiro> e2, v2; girarAte(c, 0.0f, 180.0f, e2, v2, c.estado().th, rumo, true); memoria.perdeuEm = 0; if (nav.ativo) nav.aposVirar('U'); }
        if ((a == 'L' || a == 'R') && !nav.loc.inGreen && (!olharSoSeNaoMedida || nav.ruaAtualNaoMedida())) olharAdiante(c, a == 'L' ? 1 : -1);
      }
      usAnt = -1.0f; ecoLembradoMm = -1.0f; ecoRealinhado = false; ultimoUsMs = 0;                   // depois de virar o 1o eco e lido NA HORA (um cubo no meio da rua fica a ~60 mm da ponta) e sem leitura antiga para parecer que o eco se mexe
      tSaidaMs = c.agoraMs(); odoSaida = c.odoMm();                          // daqui em diante conta o caminho andado na rua nova
      andadoRuaMm = 0.0f;
      // Partindo de um cruzamento que NAO se sabe onde esta (ANCORAR: o carro largou num y qualquer e o 'cruzamento' pode ser so a rua vertical vista de lado), nao ha distancia esperada ate o
      // proximo: com 360 mm supostos, o primeiro cruzamento de verdade chegava 'cedo' e era lido como a borda do tabuleiro (pose invertida, entrega para o lado errado).
      trechoRestante0 = nav.ancorando ? -1.0 : ((a == 'L' || a == 'R') ? kPasso - 40.0 : kPasso);   // depois de virar para o lado o eixo ja andou ~40 mm
      if (a == 'S') comecarASair(cruz, c.agoraMs(), odoCruz(c));   // reto: a barra ainda esta em cima da faixa do cruzamento, o debounce evita contar de novo
      else cruz = MemoriaCruzamento{};                 // virou (L, R, U): a barra ja nao esta na faixa; o debounce de 300 ms (75 mm) engolia a BORDA a 49 mm do beco (o carro passou da linha vermelha)
    } else {
      comecarASair(cruz, c.agoraMs(), odoCruz(c));
    }
  }

  // Depois de virar para uma rua do lado o EIXO ainda esta ~38 mm fora da linha dela e o cone do ultrassom (+-10 graus) passa ao lado de um cubo no meio da rua (a ~150 mm do
  // cruzamento): ele so apareceria a poucos mm, tarde demais. Olha 35 graus para o lado da linha (so ele: o outro lado aponta para o quarteirao, onde ha obstaculos que nao sao da rua) e, se ha eco perto, FICA apontado para ele: o tratamento
  // de cubo (para, le a cor, coleta ou desvia) assume dali.
  void olharAdiante(Carro& c, int lado) {   // lado +1 = virou a esquerda (a linha da rua fica a esquerda do eixo), -1 = direita
    const float r0 = rumo;
    girarPara(c, r0 + 35.0 * lado);
    const double d = c.us();
    girarPara(c, r0 + (d > 0.0 && d < 130.0 ? 35.0 * lado : 0.0));
  }

  // Vira para a rua do lado. A barra para na BORDA da rua que cruza e o eixo ainda esta ~40 mm antes do centro: girar dali poe a rua a 40 mm do eixo,
  // fora da barra. Entao avanca ate as duas pontas apagarem (a barra atravessou a rua, sem depender da largura da linha) e so entao gira 90 graus.
  void virarParaRua(Carro& c, bool esquerda) {
    c.comandar(ajustes.velReto, ajustes.velReto);
    const double odoIni = c.odoMm();
    double odoApagou = odoIni;
    for (int i = 0; i < 120; i++) {   // ate 600 ms
      tick(c);
      bool p[kSensores];
      lerBarra(c, p);
      odoApagou = c.odoMm();
      if (padraoEstreito(acesos(p), ajG) || acesos(p) == 0) break;   // a rua que cruza saiu da barra (so sobra a linha por onde veio). NAO por S1/S8 apagados: com S8 morto ele copia o S6, e a linha por onde veio sob a direita da barra o mantinha aceso
    }
    c.parar();
    esperarParar(c);
    // O giro fecha na linha (girarNaLinha) e isso so da certo se o eixo gira no CENTRO do cruzamento: as pontas apagam com a barra ~12,5 mm alem do centro, ou seja, com o eixo
    // (barraAFrenteMm - 12,5) mm antes dele. A freada ja andou `frenou` alem do ponto onde apagaram (a roda leva ~tauMs para parar: ~8 mm a 160 mm/s), entao falta o resto; se a freada passou, volta
    // de re. Com a barra a 15 mm o braco do giro e curto (~16 mm): 7 mm fora do centro ja deslocam a faixa da rua em ~25 graus e o giro nao fecha.
    const double frenou = c.odoMm() - odoApagou;
    const double falta = ajustes.barraAFrenteMm - 12.5 - frenou;
    ROBO_DBG("    VIRAR-RUA: apagou apos %.1f mm (freada %.1f), falta %.1f mm, eixo (%.0f,%.0f) th %.0f\n", odoApagou - odoIni, frenou, falta, c.estado().x, c.estado().y, c.estado().th * 57.3);
    if (fabs(falta) > 1.0) andarMm(c, falta, [] { return false; });
    std::vector<AmostraGiro> est, ver;
    const float rumo0 = rumo;
    girarAte(c, 0.0f, esquerda ? 90.0f : -90.0f, est, ver, c.estado().th, rumo, false);   // pelos encoders: com a barra a 15 mm ela ainda cobre a rua de origem a 90 graus, e fechar na linha nao acha a rua nova
    rumo = 90.0f * roundf(rumo0 / 90.0f) + (esquerda ? 90.0f : -90.0f);   // a grade e de multiplos de 90: o seguimento corrige o resto
    esquecerUltimoComando(memoria);
    memoria.perdeuEm = 0;
  }

  // ---- movimentos simples fora da cidade (sem linha): giram pelos encoders e andam por tempo com a velocidade CALIBRADA (distancia entre cruzamentos)
  void girarPara(Carro& c, double rumoAlvoGraus) {   // gira pelo caminho mais curto ate o rumo (0 = leste, + = esquerda)
    const float delta = static_cast<float>(remainder(rumoAlvoGraus - rumo, 360.0));
    if (fabsf(delta) < 3.0f) return;
    std::vector<AmostraGiro> e2, v2;
    girarAte(c, 0.0f, delta, e2, v2, c.estado().th, rumo);
  }
  // Espera as rodas pararem de verdade depois de um c.parar(): a roda leva ~tauMs para frear e o que ainda anda conta no percurso (o odometro continua somando).
  void esperarParar(Carro& c) {
    double antes = c.odoMm(), vNow = 1e9;
    for (int i = 0; i < 80 && fabs(vNow) > 3.0; i++) { tick(c); vNow = (c.odoMm() - antes) / (cfg.dtMs / 1000.0); antes = c.odoMm(); }
  }

  // Anda `mm` na direcao do rumo atual (mm < 0: de re), segurando o rumo pelos encoders na ida; para antes se `parar()` ficar verdadeiro.
  template <class Parar>
  double andarMm(Carro& c, double mm, Parar parar) {   // devolve quantos mm andou, sempre positivo (encoder; sem encoder, tempo x velocidade): quem precisa voltar ao ponto de partida anda o mesmo
    const double sentido = mm < 0.0 ? -1.0 : 1.0;
    mm = fabs(mm);
    const double alvoRumo = rumo;
    const float vel = std::max(1.0f, velLivreMmS > 0.0f ? velLivreMmS : ajC.velMmS);
    const unsigned long ms = static_cast<unsigned long>(mm / vel * 1000.0);
    const unsigned long ini = c.agoraMs();
    const double odo0 = c.odoMm();
    double vNow = 0.0, antes = odo0;
    // com encoder: para pela distancia lida, e comeca a frear a tempo (a roda leva ~tauMs para parar: v x tau a mais); o tempo so entra como limite (rodas travadas: 3x o previsto)
    while (cfg.encoder ? (fabs(c.odoMm() - odo0) + fabs(vNow) * cfg.tauMs / 1000.0 < mm && c.agoraMs() - ini < 3 * ms + 500) : c.agoraMs() - ini < ms) {
      Comando cmd{};
      cmd.esq = cmd.dir = static_cast<int>(sentido) * ajustes.velReto;   // a MESMA velocidade em que se mediu a velocidade (distancia entre cruzamentos / tempo): senao o tempo nao vira distancia
      if (sentido > 0.0) segurarRumoNaLacuna(cmd, rumo, static_cast<float>(alvoRumo), ajustes);
      c.comandar(cmd.esq, cmd.dir);
      tick(c);
      vNow = (c.odoMm() - antes) / (cfg.dtMs / 1000.0);
      antes = c.odoMm();
      if (parar()) break;
    }
    c.parar();
    if (cfg.encoder) {   // espera as rodas pararem de verdade: o que ainda anda na freada conta no percurso
      esperarParar(c);
      return fabs(c.odoMm() - odo0);
    }
    return (c.agoraMs() - ini) / 1000.0 * vel;
  }

  // Indo para o OESTE (pelo rumo dos encoders) a azul que aparece e a do NOSSO lado (nosso portao: sair para o verde e permitido); indo para o LESTE seria a do adversario.
  bool indoParaOeste() const { return cosf(rumo * 3.14159265f / 180.0f) < -0.3f; }

  // ---- inicio da partida e depois de cada reposicionamento: ele esta em posicao ALEATORIA do verde, olhando para a cidade (de frente = rumo 0)
  // Anda ate achar o chao amarelo ou uma plataforma a ~6 cm olhando para o oeste e volta para x ~ -150 (a mesma distancia das duas formas): isso da o x. Devolve a cor da plataforma vista.
  seguidor::CorCubo olharSlotOeste(Carro& c, bool ficar = false) {   // ficar: se ha plataforma, fica parado de frente para ela (olhando oeste) em vez de voltar
    Fatia fatia(*this, c, Fase::Plataforma);
    girarPara(c, 180.0);
    bool amarelo = false;
    const double andou = andarMm(c, 330.0, [&] {
      if (c.piso() == kPisoAmarelo) amarelo = true;
      return c.plataformaNaFrente(60.0) != seguidor::CorCubo::Nenhuma || c.piso() == kPisoVermelho;   // a plataforma (160 mm de altura) ou a moldura vermelha da ponta
    });
    const seguidor::CorCubo vista = c.plataformaNaFrente(70.0);
    ROBO_DBG("    LOOK t=%.1f andou %.0f pos (%.0f,%.0f) th %.0f vista %d\n", c.agoraMs() / 1000.0, andou, c.estado().x, c.estado().y, c.estado().th * 57.3, static_cast<int>(vista));
    if (ficar && vista != seguidor::CorCubo::Nenhuma) return vista;
    girarPara(c, 0.0);
    (void)amarelo;
    andarMm(c, andou, [] { return false; });   // volta ao x de onde saiu (o mesmo caminho: assim os 5 slots sao olhados do mesmo x)
    girarPara(c, 180.0);
    return vista;
  }

  // Anda ate a linha VERMELHA na direcao atual (TCS do chao). Duas leituras seguidas (como no sketch): uma leitura errada isolada do TCS nao pode contar como a moldura,
  // senao o y e a velocidade medidos saem errados.
  void irAteVermelhoAndando(Carro& c) {
    Fatia fatia(*this, c, Fase::Entrada);
    int seguidas = 0;
    andarMm(c, 2000.0, [&] { seguidas = c.piso() == kPisoVermelho ? seguidas + 1 : 0; return seguidas >= 2; });
  }
  // Gira para o norte/sul e vai ate a linha VERMELHA; devolve o tempo andado (ms) (quem chama sabe a distancia).
  unsigned long irAteVermelho(Carro& c, bool norte) {
    girarPara(c, norte ? 90.0 : -90.0);
    const unsigned long t0 = c.agoraMs();
    irAteVermelhoAndando(c);
    return c.agoraMs() - t0;
  }

  // Encaixa y na grade de plataformas quando a fase ja e conhecida; senao devolve y como esta.
  double naGrade(double y) const {
    if (std::isnan(nav.faseGrade)) return y;
    const double meio = kAlt / 2.0 + nav.faseGrade, passo = kPlatY_Tam + kPlatEntre;
    return meio + std::round((y - meio) / passo) * passo;
  }
  void aprenderGrade(double yPlataforma) {   // uma plataforma lida de perto define a fase da grade (so a primeira)
    if (!std::isnan(nav.faseGrade)) return;
    const double passo = kPlatY_Tam + kPlatEntre;
    nav.faseGrade = std::remainder(yPlataforma - kAlt / 2.0, passo);
  }

  // PARADO no meio do mapa, de frente para o ecoponto: gira de -36 a +36 graus lendo o ultrassom. Cada eco e um ponto da face das plataformas (Pitagoras: distancia ate a face =
  // alcance x cos, deslocamento lateral = distancia x tan); os trechos de angulos COM eco sao as plataformas e o MEIO de cada trecho e o centro dela: o cone (+-10 graus) alarga
  // cada trecho dos dois lados por igual, entao o meio nao tem vies, desde que a abertura do cone na face (distancia x tan 10) fique abaixo de meio vao (50 mm): por isso primeiro
  // chega a ~170 mm da face (fase 1 acha a distancia, anda, fase 2 varre fino). So le a COR de perto depois. Guarda onde ha plataforma e esvazia as marcas cobertas sem eco.
  void mapearDeLonge(Carro& c, double yAtual) {
    Fatia fatia(*this, c, Fase::Plataforma);
    constexpr double kRad = M_PI / 180.0, kDistVarreduraMm = 170.0, kAlcanceMaxMm = 700.0;
    auto alcance = [&](double phi) {   // phi > 0 = para o norte, a partir do oeste; distancia do EIXO (a ponta fica 100 mm a frente)
      girarPara(c, 180.0 - phi);
      const double e = c.us();
      return e > 0.0 ? e + 100.0 : -1.0;
    };
    ROBO_DBG("    MAPEAR inicio yAtual %.0f real (%.0f,%.0f) th %.0f\n", yAtual, c.estado().x, c.estado().y, c.estado().th * 57.3);
    double D = -1.0;
    for (const double phi : {0.0, 20.0, -20.0, 40.0, -40.0}) {
      const double r = alcance(phi);
      if (r > 0.0 && r < kAlcanceMaxMm) { D = r * cos(phi * kRad); break; }
    }
    girarPara(c, 180.0);
    if (D < 0.0) { c.evento("PLATAFORMAS: nenhum eco do meio do mapa: busca de perto"); return; }
    if (D > kDistVarreduraMm + 25.0) andarMm(c, D - kDistVarreduraMm, [] { return false; });
    std::vector<std::pair<double, double>> eco;   // (phi, alcance); alcance < 0 = sem eco
    for (int g = -36; g <= 36; g += 4) eco.push_back({static_cast<double>(g), alcance(g)});
    girarPara(c, 180.0);
    ROBO_DBG("    MAPEAR fim da varredura real (%.0f,%.0f) th %.0f\n", c.estado().x, c.estado().y, c.estado().th * 57.3);
    if (ROBO_DEPURANDO()) { std::string l; for (const auto& e : eco) l += fmt(" %.0f:%.0f", e.first, e.second); ROBO_DBG("    MAPEAR eco (phi:alcance)%s\n", l.c_str()); }
    std::vector<double> perp;
    for (const auto& e : eco) if (e.second > 0.0 && fabs(e.first) <= 25.0) perp.push_back(e.second * cos(e.first * kRad));
    if (perp.empty()) { c.evento("PLATAFORMAS: varredura parada sem eco: busca de perto"); return; }
    std::sort(perp.begin(), perp.end());
    const double D2 = perp[perp.size() / 2];
    // trechos de eco consecutivos. Um trecho que encosta no limite da varredura nao tem o meio visivel: o centro sai da ponta que se ve (borda da plataforma = ponta
    // vista + abertura do cone, e o centro fica 80 mm adiante).
    std::vector<double> centros;
    const double aberturaCone = D2 * tan(10.0 * kRad);
    for (size_t i = 0; i < eco.size();) {
      if (eco[i].second <= 0.0) { i++; continue; }
      size_t j = i;
      while (j + 1 < eco.size() && eco[j + 1].second > 0.0) j++;
      if (j > i) {   // pelo menos 2 amostras seguidas: um eco isolado nao e plataforma
        const double y0 = yAtual + D2 * tan(eco[i].first * kRad), y1 = yAtual + D2 * tan(eco[j].first * kRad);
        if (i == 0 && j + 1 == eco.size()) centros.push_back((y0 + y1) / 2.0);   // enche a varredura toda: so o meio serve
        else if (i == 0) centros.push_back(y1 - aberturaCone - kPlatY_Tam / 2.0);   // continua alem do lado do sul: a ponta de cima e a borda
        else if (j + 1 == eco.size()) centros.push_back(y0 + aberturaCone + kPlatY_Tam / 2.0);
        else centros.push_back((y0 + y1) / 2.0);
      }
      i = j + 1;
    }
    const double meio = kAlt / 2.0, passo = kPlatY_Tam + kPlatEntre;   // as 5 marcas ficam numa grade de 260 mm centrada no mapa
    for (double y : centros) {
      const double grade = naGrade(y);
      if (!std::isnan(nav.faseGrade) && fabs(grade - y) < 45.0) y = grade;
      bool repetida = false;
      for (const double o : nav.platPresentes) if (fabs(o - y) < 40.0) repetida = true;
      if (!repetida) { nav.platPresentes.push_back(y); c.evento(fmt("PLATAFORMAS: eco de plataforma em y = %.0f mm (ainda sem cor)", y)); }
    }
    for (int i = 0; i < 9; i++) {   // as marcas ao alcance (45 graus) sem plataforma por perto: vazias
      const double y = meio + (i - 4) * passo;
      if (fabs(y - yAtual) > D2 * tan(25.0 * kRad)) continue;   // so as marcas bem dentro da varredura
      bool temPlat = false;
      for (const double o : nav.platPresentes) if (fabs(o - y) < 60.0) temPlat = true;
      if (!temPlat) { nav.yOlhados.push_back(y); c.evento(fmt("PLATAFORMAS: y = %.0f mm sem eco (marca vazia)", y)); }
    }
    nav.mapeado = true;
  }

  // PARADO de frente para uma plataforma (oeste, ponta a ~60 mm da face): gira de -36 a +36 graus lendo o ultrassom. Os angulos COM eco (distancia x cos = distancia da face) sao a plataforma,
  // o MEIO do trecho e o centro dela (o cone alarga as duas pontas por igual) e o cateto lateral = distancia x tan(angulo do meio). O centro da plataforma vem da grade (conhecido), entao
  // o y do eixo = centro - cateto, sem depender de partida, de tempo nem de a ponta ter parado na borda da plataforma (erro de ate ~25 mm que fazia reentrar fora da rua).
  double yPeloGiro(Carro& c, double yPlataforma) {
    constexpr double kRad = M_PI / 180.0;
    const float rumo0 = rumo;   // de frente para a plataforma (oeste)
    const double e0 = c.us();
    if (e0 < 0.0) return yPlataforma;   // sem eco de frente: nao corrige
    const double r0 = e0 + 100.0;       // distancia do EIXO a face
    // varredura CONTINUA (sem parar a cada angulo): vai a phi = -36, gira para +36 lendo o ultrassom a cada ~30 ms (o HC-SR04 nao da mais que isso) e rotula cada leitura pelo rumo dos encoders
    girarPara(c, rumo0 + 36.0f);
    const int v = ajustes.velBuscaExterna;
    c.comandar(v, -v);   // phi cresce = rumo diminui (vira a direita)
    std::vector<std::pair<double, double>> bruto;   // (phi, alcance do EIXO; < 0 = sem eco)
    float omega = 0.0f, rumoAnt = rumo;
    const unsigned long inicio = c.agoraMs();
    for (int t = 0; c.agoraMs() - inicio < 4000; t++) {
      tick(c);
      omega = (rumo - rumoAnt) / static_cast<float>(cfg.dtMs / 1000.0);
      rumoAnt = rumo;
      const double phi = -(rumo - rumo0);
      if (phi + omega * -1.0 * cfg.tauMs / 1000.0 >= 36.0) break;   // omega < 0 girando para a direita: -omega x tau = quanto ainda gira
      if (t % 6 == 0) { const double e = c.us(); bruto.push_back({phi, e > 0.0 ? e + 100.0 : -1.0}); }
    }
    c.parar();
    for (int i = 0; i < 80 && fabsf(omega) > 2.0f; i++) { tick(c); omega = (rumo - rumoAnt) / static_cast<float>(cfg.dtMs / 1000.0); rumoAnt = rumo; }
    // a face pode estar inclinada (o rumo estimado deriva): r(phi) = d / cos(phi - delta). Ajusta delta (inclinacao) e d (distancia ao longo da normal) com os ecos perto de r0
    // (o resto e parede de fundo ou plataforma vizinha); com delta o y sai certo e o rumo se reancora na face (que e sempre perpendicular a um dos eixos).
    auto normal = [&](double phi, double dl) { return cos((phi - dl) * kRad); };
    double delta = 0.0, d = r0, melhor = 1e18;
    for (double dl = -40.0; dl <= 40.0; dl += 0.5) {
      std::vector<double> ds;
      for (const auto& b : bruto) if (b.second > 0.0 && b.second < r0 * 1.6 && fabs(b.first - dl) < 50.0) ds.push_back(b.second * normal(b.first, dl));
      if (ds.size() < 5) continue;
      std::sort(ds.begin(), ds.end());
      const double dm = ds[ds.size() / 2];
      double custo = 0.0;   // erro limitado: ecos fora da face (vizinhas, fundo) custam o mesmo, nao dominam
      for (const auto& b : bruto) if (b.second > 0.0 && fabs(b.first - dl) < 50.0) custo += std::min(fabs(b.second * normal(b.first, dl) - dm), 30.0);
      if (custo < melhor) { melhor = custo; delta = dl; d = dm; }
    }
    std::vector<std::pair<double, bool>> amostras;   // (phi relativo a normal da face, ha eco da face)
    for (const auto& b : bruto) amostras.push_back({b.first - delta, b.second > 0.0 && fabs(b.second * normal(b.first, delta) - d) < 30.0});
    // O ajuste tem vies de ate ~8 graus quando o rumo esta certo (o cone e os 3 raios do ultrassom achatam o perfil perto da normal); o rumo que derivou de verdade passa disso
    // (22 graus numa partida com ruido). Abaixo da zona morta nao mexe: o y e o rumo seguem como antes.
    constexpr double kZonaMortaInclinacao = 10.0;
    if (fabs(delta) <= kZonaMortaInclinacao) { delta = 0.0; d = r0; }
    if (fabs(delta) > kZonaMortaInclinacao) {   // reancora o rumo: a normal da face esta em phi = delta, ou seja, no rumo estimado rumo0 - delta, que deveria ser o eixo mais proximo
      const float nominal = 90.0f * roundf((rumo0 - static_cast<float>(delta)) / 90.0f);
      ROBO_DBG("    GIRO-Y: face inclinada %.1f graus (d %.0f, antes r0 %.0f): rumo reancorado %.1f -> %.1f (real th %.1f)\n", delta, d, r0, rumo, rumo + (nominal - (rumo0 - static_cast<float>(delta))), c.estado().th * 57.29578);
      rumo += nominal - (rumo0 - static_cast<float>(delta));
      girarPara(c, nominal);
    } else girarPara(c, rumo0);
    const double r0f = d;   // distancia do eixo a face pela normal
    // o trecho CONTIGUO com eco que passa pelo 0 e a plataforma (as vizinhas ficam a 130 mm e aparecem como outros trechos)
    int i0 = -1;
    for (size_t i = 0; i < amostras.size(); i++) if (amostras[i].second && (i0 < 0 || fabs(amostras[i].first) < fabs(amostras[i0].first))) i0 = static_cast<int>(i);
    if (i0 < 0 || fabs(amostras[i0].first) > 12.0) return yPlataforma;
    int lo = i0, hi = i0;
    while (lo > 0 && amostras[lo - 1].second) lo--;
    while (hi + 1 < static_cast<int>(amostras.size()) && amostras[hi + 1].second) hi++;
    const bool bordaSul = lo > 0, bordaNorte = hi + 1 < static_cast<int>(amostras.size());
    const double phiI = amostras[lo].first, phiJ = amostras[hi].first;
    constexpr double kCone = 10.0, kSensorMm = 100.0;   // meio angulo do cone e distancia do sensor ao eixo (o cone sai da ponta, nao do eixo)
    // borda da plataforma (deslocamento lateral em relacao ao eixo) vista quando a ponta do cone, saindo do sensor, a toca
    auto borda = [&](double phi, double cone) { return kSensorMm * sin(phi * kRad) + (r0f - kSensorMm * cos(phi * kRad)) * tan((phi + cone) * kRad); };
    double lateral;   // deslocamento do centro da plataforma em relacao ao eixo (+ = norte)
    if (hi - lo < 1) return yPlataforma;
    if (bordaSul && bordaNorte) lateral = (borda(phiI, kCone) + borda(phiJ, -kCone)) / 2.0;   // as duas bordas a vista: o meio
    else if (bordaSul) lateral = borda(phiI, kCone) + kPlatY_Tam / 2.0;                       // so a borda do sul: o centro fica meia plataforma acima
    else if (bordaNorte) lateral = borda(phiJ, -kCone) - kPlatY_Tam / 2.0;                    // so a borda do norte
    else return yPlataforma;                                                                  // enche a varredura toda: sem borda para medir
    const double phiMeio = atan2(lateral, r0f) / kRad;
    ROBO_DBG("    GIRO-Y: n=%d phiMeio %.1f r0 %.0f -> y %.0f | y real %.0f (plataforma %.0f)\n", hi - lo + 1, phiMeio, r0, yPlataforma - lateral, c.estado().y, yPlataforma);
    return yPlataforma - lateral;
  }

  // Carregando, a prioridade e GUARDAR O CUBO: nada de medir velocidade nem de olhar todas as marcas antes. Se a plataforma da cor ainda nao foi vista (nos primeiros cruzamentos
  // ou antes), desce do alto do ecoponto olhando as marcas ate achar a da cor do cubo (o que vir no caminho fica guardado em platConhecidas). Quando as 5 foram olhadas, sabe
  // quais cores tem plataforma e so passa a coletar essas. `yAtual` e atualizado.
  bool procurarPlataforma(Carro& c, seguidor::CorCubo cor, double& yAtual) {
    Fatia fatia(*this, c, Fase::Plataforma);
    // De frente para uma plataforma o carro aponta para o oeste (a face e uma parede): reancora o rumo, que anda com tantas curvas de 90 graus.
    auto reancorarOeste = [&] { rumo = static_cast<float>(180.0 + 360.0 * lround((rumo - 180.0) / 360.0)); };
    constexpr double kFolga = 60.0;   // a ponta para a ~60 mm da face da plataforma ao ler a cor
    auto jaOlhou = [&](double y) { for (const double o : nav.yOlhados) if (fabs(o - y) < 40.0) return true; return false; };
    // Onde ainda pode haver plataforma: sem nenhuma conhecida, nas 5 marcas (260 mm); achada uma, o passo e o das plataformas (260 mm) a partir dela.
    auto candidatos = [&] {
      std::vector<double> v;
      auto poe = [&](double y) { if (fabs(y - kAlt / 2.0) <= 560.0 && !jaOlhou(y)) { for (const double o : v) if (fabs(o - y) < 40.0) return; v.push_back(y); } };
      for (const double y : nav.platPresentes) poe(y);   // o ultrassom parado ja viu eco ali: le a cor dessas primeiro
      if (!v.empty()) return v;
      std::vector<double> ancoras = nav.platPresentes;
      for (const auto& pc : nav.platConhecidas) ancoras.push_back(pc.first);
      if (ancoras.empty()) for (int i = 0; i < 5; i++) poe(kAlt / 2.0 + (i - 2) * (kPlatY_Tam + kPlatEntre));   // as 5 marcas do ecoponto (260 mm): 1 a 5 plataformas ficam nelas
      else for (const double y0 : ancoras) for (int k = -4; k <= 4; k++) poe(y0 + k * (kPlatY_Tam + kPlatEntre));
      return v;
    };
    bool deFrente = false;   // parado olhando oeste, a ~60 mm de uma plataforma
    for (int guarda = 0; guarda < 20; guarda++) {
      const std::vector<double> cand = candidatos();
      if (cand.empty()) break;
      double yt = cand[0];
      for (const double y : cand) if (fabs(y - yAtual) < fabs(yt - yAtual)) yt = y;
      const double dy = yt - yAtual;
      seguidor::CorCubo vista = seguidor::CorCubo::Nenhuma;
      if (deFrente && fabs(dy) >= 100.0) {
        // De frente para uma plataforma: 1) RODA para a diagonal que aponta para a proxima marca e IDENTIFICA pelo ultrassom (eco perto da distancia esperada = ha plataforma; sem eco
        // = marca vazia, nem anda); 2) REALINHA (vira para cima/baixo); 3) PITAGORAS: o eco e a hipotenusa e o cateto ate a face (eixo - face) e conhecido, entao o outro cateto e a
        // distancia a andar, sem depender da velocidade; 4) anda, roda para o oeste e le a cor. A distancia do eixo a face vem do proprio ultrassom de frente (a ponta fica a ~50 mm).
        const int sentido = dy > 0 ? 1 : -1;
        if (!nav.mapeado) {
        const double aFace = c.us();   // de frente: ponta -> face
        const double cateto = (aFace > 0 ? aFace : kFolga) + 100.0;   // eixo -> face (a ponta fica 100 mm a frente do eixo)
        const double phi = atan2(fabs(dy), cateto) * 180.0 / M_PI;
        girarPara(c, sentido > 0 ? 180.0 - phi : 180.0 + phi);
        const double esperado = hypot(cateto, fabs(dy)) - 100.0;
        const double eco = c.us();
        nav.yOlhados.push_back(yt);
        ROBO_DBG("    DIAG yt %.0f yAtual %.0f aFace %.0f cateto %.0f phi %.0f eco %.0f esperado %.0f pos (%.0f,%.0f) th %.0f\n", yt, yAtual, aFace, cateto, phi, eco, esperado, c.estado().x, c.estado().y, c.estado().th * 57.3);
        if (!(eco > 0.0 && eco > esperado - 120.0 && eco < esperado + 40.0)) { c.evento(fmt("PLATAFORMAS: y = %.0f mm sem eco (marca vazia)", yt)); continue; }
        }
        // Pitagoras com o eco: o cone (+-10 graus) devolve o canto MAIS PERTO da plataforma, nao o centro da face, e o cateto sai curto (ate ~60 mm); por isso o eco so
        // CONFIRMA (hipotenusa plausivel) e a distancia andada e o passo conhecido entre plataformas.
        const double leg = fabs(dy);
        girarPara(c, sentido > 0 ? 90.0 : -90.0);
        andarMm(c, leg, [] { return false; });
        yAtual += sentido * leg;
        girarPara(c, 180.0);
        andarMm(c, 70.0, [&] { return c.plataformaNaFrente(60.0) != seguidor::CorCubo::Nenhuma || c.piso() == kPisoVermelho; });   // a curva deixa o x alguns mm fora: encosta ate a ponta ficar a ~60 mm
        vista = c.plataformaNaFrente(70.0);
        deFrente = vista != seguidor::CorCubo::Nenhuma;
      } else {
        if (fabs(dy) > minMovMm()) { girarPara(c, dy > 0 ? 90.0 : -90.0); andarMm(c, fabs(dy), [] { return false; }); }
        yAtual = yt;
        nav.yOlhados.push_back(yt);
        vista = olharSlotOeste(c, true);
        // O eco parado ja viu plataforma por aqui, mas a posicao dele (e a conta de distancia) erra ate ~70 mm: antes de dar a marca por vazia, tenta 55 mm para cada lado.
        bool ecoAqui = false;
        for (const double o : nav.platPresentes) if (fabs(o - yt) < 80.0) ecoAqui = true;
        if (vista == seguidor::CorCubo::Nenhuma && ecoAqui && tentarLados) {
          for (const double passoLado : {-55.0, 110.0}) {
            girarPara(c, passoLado > 0 ? 90.0 : -90.0);
            andarMm(c, fabs(passoLado), [] { return false; });
            yAtual += passoLado;
            vista = olharSlotOeste(c, true);
            if (vista != seguidor::CorCubo::Nenhuma) break;
          }
        }
        deFrente = vista != seguidor::CorCubo::Nenhuma;
      }
      if (vista != seguidor::CorCubo::Nenhuma) {
        reancorarOeste();
        // De frente para uma plataforma o y do eixo e o centro dela, e as plataformas so ficam nas 5 marcas, na grade de 260 mm centrada no mapa: a conta por
        // tempo (meio do mapa -> plataforma -> plataforma) erra ate ~65 mm, e esse erro fazia a volta ao portao errar a rua. Encaixa na grade.
        yAtual = naGrade(yAtual);   // fase conhecida: encaixa; a 1a plataforma lida define a fase (nas marcas ou entre elas)
        aprenderGrade(yAtual);
        yt = yAtual;
        nav.platConhecidas.push_back({yt, vista});
        nav.plataformasVistas.push_back({yt, vista});
        c.evento(fmt("PLATAFORMAS: %s em y = %.0f mm", nomeCor(vista), yt));
        if (vista == cor) break;
      } else {
        c.evento(fmt("PLATAFORMAS: y = %.0f mm vazia", yt));
      }
    }
    if (candidatos().empty()) {   // tudo olhado: so se pega cor que tem plataforma
      for (int i = 0; i < seguidor::kNumCoresCubo; i++) nav.coleta.coletavel[i] = false;
      for (const auto& pv : nav.plataformasVistas) nav.coleta.coletavel[static_cast<int>(pv.second)] = true;
      nav.plataformasObservadas = true;
    }
    for (const auto& pv : nav.plataformasVistas) if (pv.second == cor) return true;
    return false;
  }

  static const char* nomeCor(seguidor::CorCubo c) {
    static const char* n[] = {"nenhuma", "branca", "vermelha", "azul", "verde", "amarela", "preta", "marrom"};
    return n[static_cast<int>(c)];
  }

  // Alinha com a rua de uma linha de cruzamentos (a menos visitada na coluna 0) e vira de frente para a cidade. A linha so serve para escolher o lado da moldura vermelha mais perto:
  // quem descobre a linha de verdade e CONTAR os cruzamentos ate a linha vermelha (Navegador::aoCruzamento). `usarAlinhamento = false` entra reto (primeira rua vertical, de lado).
  bool usarAlinhamento = true;
  void entrarNaCidade(Carro& c, double yAtual) {
    Fatia fatia(*this, c, Fase::Entrada);
    ROBO_DBG("    ENTRAR t=%.1f rumo %.0f real th %.0f pos (%.0f,%.0f) yAtual %.0f\n", c.agoraMs() / 1000.0, rumo, c.estado().th * 57.3, c.estado().x, c.estado().y, yAtual);
    // Entra por uma linha do MEIO (1 a 3): nas das pontas a rua fica a 127 mm da moldura vermelha e a faixa azul do portao encontra a vermelha no canto (ele subia pela azul como se fosse rua).
    int melhor = 1;
    for (int r = 2; r < core::kMapRows - 1; r++) if (nav.visitas.n[0][r] < nav.visitas.n[0][melhor]) melhor = r;
    const bool sabeY = usarAlinhamento && !std::isnan(yAtual);   // NaN = entrou direto, sem subir ate o vermelho: a linha so se descobre contando cruzamentos
    if (sabeY) {
      const double yAlvo = yDoCruzamento(melhor);
      // A partida (zona morta + inercia) come um pouco de um deslocamento CURTO: compensa so ate ~200 mm (20 mm no maximo). Eram 40 mm (medido com o mapa de 302 mm), mas com o y
      // da plataforma ja encaixado na grade e a velocidade medida na subida, 40 mm passava do ponto: ele entrava ~40 mm abaixo da rua (fora da barra de 75 mm) e seguia a faixa azul
      // como rua. 29/30 sementes na meta contra 27/30; 60 sementes: 57 contra 54.
      const double dyAbs = fabs(yAlvo - yAtual);
      if (dyAbs > minMovMm()) { girarPara(c, yAlvo > yAtual ? 90.0 : -90.0); andarMm(c, dyAbs + (cfg.encoder ? 0.0 : std::max(0.0, 20.0 - dyAbs / 10.0)), [] { return false; }); }   // com encoder nao ha partida a compensar
    }
    girarPara(c, 0.0);
    core::initInGreen(nav.loc);
    nav.naRua = false;
    nav.linhaEntrada = sabeY ? melhor : -1;
    nav.calibrando = 0;
    nav.contados = 0;
    portaoPendente = true;
    trechoRestante0 = -1.0;   // o trecho da ultima saida nao vale mais: o 1o cruzamento depois do portao fica a ~170 mm (beco), nao a 360
    semReacaoAte = 0;   // a azul que vem agora E o portao que ele espera: a janela de 1,5 s da saida anterior (ainda em cima da faixa) nao pode engoli-la (voltava e atravessava a cidade 'no verde')
    memoria = Memoria{};
    cruz = MemoriaCruzamento{};
    esquecerCor(memCor);
    precisaAlinhar = false;
  }

  // Depois de um reset (colocado em posicao aleatoria do verde, olhando para a cidade: o rumo volta a 0): entra direto (sem percurso no verde): o y vem de contar os cruzamentos ate o vermelho.
  void alinharNaRua(Carro& c) {   // depois de um reset (posicao aleatoria do verde, de frente para a cidade, rumo = 0): sobe ate o vermelho para saber o y e alinha com uma rua
    Fatia fatia(*this, c, Fase::Entrada);
    entrarNaCidade(c, NAN);
  }

  // Saiu pelo nosso portao levando 1 ou 2 cubos: vai ao ecoponto, acha a plataforma de cada cor (TCS da frente), deixa o cubo nela e volta de frente para a cidade.
  // Cubo na plataforma errada ou fora dela = cubo perdido (-10). (A garra ainda nao e simulada: o cubo "e solto" quando o carro para diante da plataforma.)
  void entregarNaPlataforma(Carro& c) {
    Fatia fatia(*this, c, Fase::Plataforma);
    nav.entregando = true;
    const unsigned long tIni = c.agoraMs();
    ROBO_DBG("    ENTREGAR t=%.0fs: rumo %.0f, real th %.0f, pos (%.0f,%.0f)\n", c.agoraMs() / 1000.0, rumo, c.estado().th * 57.3, c.estado().x, c.estado().y);
    // 1) entra no verde: continua para o oeste ate x ~ -150
    girarPara(c, 180.0);
    andarMm(c, 150.0, [] { return false; });
    // 2) vai ao MEIO da altura do mapa e fica la: nao precisa subir ate o topo para achar as plataformas (o ultrassom varre dali, ver mapearDeLonge). So a PRIMEIRA entrega sobe ate a
    // linha VERMELHA do alto (TCS do chao: vermelho = para na hora): e a unica medida de velocidade em reta com distancia conhecida (os tempos entre cruzamentos incluem partidas e
    // curvas, e os deslocamentos de ~260 mm entre plataformas saem errados em mais de 80 mm). Depois disso a altura vem da linha do portao (pose), e vai direto ao meio.
    const core::Estimate esPortao = core::estimate(nav.loc);
    const bool sabeLinha = core::rowMass(nav.loc, esPortao.pose.at.r) >= core::kMinCoordBelief && core::inMap(esPortao.pose.at.c, esPortao.pose.at.r);
    const double yPortao = yDoCruzamento(esPortao.pose.at.r);
    double yAtual = yPortao;
    if ((!velRetaMedida && !cfg.encoder) || !sabeLinha) {   // com encoder nao precisa subir ate a moldura para medir a velocidade
      girarPara(c, 90.0);
      const unsigned long tSubida0 = c.agoraMs();
      irAteVermelhoAndando(c);
      const unsigned long msSubida = c.agoraMs() - tSubida0;
      if (sabeLinha && msSubida > 800 && (kAlt - 40.0 - yPortao) > 250.0) {
        velLivreMmS = static_cast<float>((kAlt - 40.0 - yPortao) / (msSubida / 1000.0));
        velRetaMedida = true;
        c.evento(fmt("velocidade reta medida na subida ate a moldura de cima = %.0f mm/s", velLivreMmS));
      } else if (!velRetaMedida) {   // nao sabe a linha: mede entre as duas molduras (desce e volta a subir)
        const unsigned long ms = irAteVermelho(c, false);
        if (ms > 500) { velLivreMmS = static_cast<float>((kAlt - 80.0) / (ms / 1000.0)); velRetaMedida = true; }
        girarPara(c, 90.0);
        irAteVermelhoAndando(c);
      }
      yAtual = kAlt - 40.0;   // com a barra na borda da faixa vermelha, o eixo esta ~40 mm abaixo dela
    }
    if (fabs(kAlt / 2.0 - yAtual) > minMovMm()) {
      girarPara(c, kAlt / 2.0 > yAtual ? 90.0 : -90.0);
      andarMm(c, fabs(kAlt / 2.0 - yAtual), [] { return false; });
      yAtual = kAlt / 2.0;
    }
    nav.plataformasVistas = nav.platConhecidas;   // o que ja foi lido de perto (cruzamentos iniciais e entregas anteriores)
    bool falta = false;
    for (const seguidor::CorCubo cor : nav.carga) { bool sabe = false; for (const auto& pv : nav.plataformasVistas) if (pv.second == cor) sabe = true; if (!sabe) falta = true; }
    if (falta && !nav.mapeado && !nav.plataformasObservadas) mapearDeLonge(c, yAtual);
    if (nav.observando) {   // viagem so de olhar: le as cores de todas as plataformas (procurarPlataforma sem cor-alvo olha todas) e fixa o que se pode coletar
      if (!nav.mapeado && !nav.plataformasObservadas) mapearDeLonge(c, yAtual);
      if (!nav.plataformasObservadas) procurarPlataforma(c, seguidor::CorCubo::Nenhuma, yAtual);
      c.evento(fmt("PLATAFORMAS vistas no inicio: %zu, coletaveis atualizados", nav.plataformasVistas.size()));
      // A varredura acumulou dezenas de mm de erro de y (a barra de 75 mm perde a rua se entrar assim). As plataformas ficam numa grade conhecida: volta de frente para a MAIS PERTO e
      // mede o y por ela (yPeloGiro), como depois de uma entrega. Sem plataforma a vista, reentra sem y (como na partida).
      double yRef = std::nan("");
      for (const auto& pv : nav.plataformasVistas) if (std::isnan(yRef) || fabs(pv.first - yAtual) < fabs(yRef - yAtual)) yRef = pv.first;
      if (!std::isnan(yRef)) {
        if (fabs(yRef - yAtual) > minMovMm()) { girarPara(c, yRef > yAtual ? 90.0 : -90.0); andarMm(c, fabs(yRef - yAtual), [] { return false; }); }
        girarPara(c, 180.0);
        andarMm(c, 330.0, [&] { return c.plataformaNaFrente(60.0) != seguidor::CorCubo::Nenhuma || c.piso() == kPisoVermelho; });
        if (c.plataformaNaFrente(70.0) != seguidor::CorCubo::Nenhuma) { rumo = static_cast<float>(180.0 + 360.0 * lround((rumo - 180.0) / 360.0)); yAtual = yPeloGiro(c, yRef); }
        else yAtual = std::nan("");
      } else yAtual = std::nan("");
      girarPara(c, 0.0);                // de costas para a plataforma: volta para o verde
      andarMm(c, 120.0, [] { return false; });
      girarPara(c, 180.0);
      nav.observando = false;
    }
    for (const seguidor::CorCubo cor : std::vector<seguidor::CorCubo>(nav.carga)) {
      Plataforma plLocal{kPlatX, 0.0, cor};
      const Plataforma* pl = nullptr;
      for (const auto& pv : nav.plataformasVistas) if (pv.second == cor) { plLocal.y = pv.first; pl = &plLocal; }   // so onde VIU a plataforma no inicio
      if (!pl && !nav.plataformasObservadas && procurarPlataforma(c, cor, yAtual)) for (const auto& pv : nav.plataformasVistas) if (pv.second == cor) { plLocal.y = pv.first; pl = &plLocal; }
      if (!pl) { nav.pontos -= 10; nav.perdidos++; c.evento("cubo sem plataforma da cor: perdido"); continue; }
      nav.alvoX = pl->x; nav.alvoY = pl->y;
      // 3) vai ate a altura ESPERADA da plataforma (os slots sao retangulos fixos na arena; a cor e que muda), 4) vira para o oeste e olha; como a conta de distancia por tempo
      //    erra algumas dezenas de mm e a plataforma tem so 50 mm de altura, 5) varre para cima e para baixo de 60 em 60 mm ate ver a plataforma da cor do cubo.
      const double dy = pl->y - yAtual;
      ROBO_DBG("    ENTREGAR-ALVO cor %d plataforma y %.0f yAtual(estimado) %.0f real y %.0f pose do portao (%d,%d)\n", static_cast<int>(cor), pl->y, yAtual, c.estado().y, esPortao.pose.at.c, esPortao.pose.at.r);
      if (fabs(dy) > minMovMm()) { girarPara(c, dy > 0 ? 90.0 : -90.0); andarMm(c, fabs(dy), [] { return false; }); }
      yAtual = pl->y;
      const unsigned long tChegouY = c.agoraMs();
      constexpr double kAproximacaoMm = 260.0;
      girarPara(c, 180.0);
      const unsigned long tGirou = c.agoraMs();
      auto achou = [&] {
        const unsigned long tA = c.agoraMs();
        const double andouA = andarMm(c, kAproximacaoMm, [&] { return c.plataformaNaFrente(60.0) != seguidor::CorCubo::Nenhuma || c.piso() == kPisoVermelho; });   // a moldura vermelha do ecoponto tambem para   // para ao ver a plataforma: o limite so cobre o caso de partir do portao (x ~ -125, ponta a ~340 mm da face)
        ROBO_DBG("    ACHOU? giro %.1f s, andou %.0f mm em %.1f s | real (%.0f,%.0f) th %.0f | plataforma y %.0f vista %d\n", (tA - tGirou) / 1000.0, andouA, (c.agoraMs() - tA) / 1000.0, c.estado().x, c.estado().y, c.estado().th * 57.3, pl->y, static_cast<int>(c.plataformaNaFrente(70.0)));
        const bool ok = c.plataformaNaFrente(70.0) == cor;
        if (!ok) { girarPara(c, 0.0); andarMm(c, andouA, [] { return false; }); girarPara(c, 180.0); }   // volta EXATAMENTE o que andou: com 90 fixos ele derivava ~170 mm para o oeste a cada tentativa e saia da arena   // nao e ela: recua para o verde para varrer sem bater
        return ok;
      };
      bool encontrou = achou();
      static const double passos[8] = {60, -120, 180, -240, 300, -360, 420, -480};   // cada passo e relativo ao anterior: +60, -60, +120, -120, ...
      for (int k = 0; k < 8 && !encontrou; k++) {
        girarPara(c, passos[k] > 0 ? 90.0 : -90.0);
        andarMm(c, fabs(passos[k]), [] { return false; });
        yAtual += passos[k];
        girarPara(c, 180.0);
        encontrou = achou();
      }
      const seguidor::CorCubo vista = c.plataformaNaFrente(70.0);
      const bool largou = vista == cor && c.soltar(cor);
      if (largou) {
        nav.pontos += 100;
        nav.entregues++;
        c.evento(fmt("ENTREGOU cubo na plataforma da cor %d (+100, total %d pontos)", static_cast<int>(cor), nav.pontos));
        const unsigned long tAchou = c.agoraMs();
        if (cfg.encoder) yAtual = yPeloGiro(c, pl->y);   // reentra pelo y medido, nao pelo y 'vi a plataforma'
        ROBO_DBG("    TEMPO-ENTREGA ate o y %.1f s, achar a plataforma %.1f s, yPeloGiro %.1f s (inicio %.0f s)\n", (tChegouY - tIni) / 1000.0, (tAchou - tChegouY) / 1000.0, (c.agoraMs() - tAchou) / 1000.0, tIni / 1000.0);

      } else {
        nav.pontos -= 10;
        nav.perdidos++;
        c.evento(vista == cor ? fmt("plataforma certa (cor %d) mas nao largou o cubo: cubo perdido (-10)", static_cast<int>(cor))
                              : fmt("plataforma ERRADA (vi cor %d, o cubo e cor %d): cubo perdido (-10)", static_cast<int>(vista), static_cast<int>(cor)));
      }
      girarPara(c, 0.0);                // de costas para a plataforma: volta para o verde
      andarMm(c, 120.0, [] { return false; });
      girarPara(c, 180.0);
    }
    nav.carga.clear();
    nav.carregando = false;
    nav.liberarBloqueiosTemporarios();
    c.limparCarga();
    // 6) volta de frente para a cidade: alinha com uma rua a partir do y da plataforma
    c.limparCarga();
    entrarNaCidade(c, yAtual);   // a velocidade reta foi medida entre as molduras e o y vem da plataforma que acabou de ver: nao precisa subir ate o vermelho de novo
    nav.alvoX = nav.alvoY = -1;
    nav.entregando = false;
  }

  // Cubo (ou o outro robo) no caminho: da meia-volta e volta. Se o cubo estava perto do cruzamento de onde saiu (um cubo no meio da rua fica a ~150 mm dele), o carro ainda esta
  // EM CIMA do cruzamento e nao ha outro para detectar: sem replanejar aqui ele ficava girando entre dois cubos (um em cada rua) para sempre, esbarrando neles. Entao, perto do
  // cruzamento, trata a volta como CHEGADA nele: confirma, varre e planeja (o cubo ja entrou no mapa como rua bloqueada, e a rota sai por outra rua).
  void voltarDoCubo(Carro& c) {
    std::vector<AmostraGiro> e2, v2;
    girarAte(c, 0.0f, 180.0f, e2, v2, c.estado().th, rumo);
    memoria.perdeuEm = 0;
    usAnt = -1.0f; ecoLembradoMm = -1.0f; ecoRealinhado = false;
    if (nav.ativo && !nav.loc.inGreen && tSaidaMs != 0 && andadoRuaMm < 120.0f) {
      bool noCruzamento[kSensores] = {true, false, false, true, true, false, false, true};   // S1 e S8 acesos: a premissa da varredura
      medindoVel = false;
      pararNoCruzamento(c, noCruzamento);
    }
  }

  // Ultrassom na frente enquanto anda por uma rua: cubo a ~7 cm -> para, le a cor (TCS da frente) e decide pelas REGRAS:
  //   cor que tem plataforma e ainda ha vaga (maximo 2) -> coleta; branco ou cor sem plataforma -> identifica, SINALIZA (som/luz, +50 uma vez por cubo), nao mexe e bloqueia a rua.
  // Eco que se MEXE = oponente: espera. true = tomou o controle neste ciclo.
  bool tratarCubo(Carro& c) {
    Fatia fatia(*this, c, Fase::Cubo);
    if (!c.missao() || !c.usAtivo() || !nav.naRua || nav.loc.inGreen) return false;
    const bool cheio = static_cast<int>(nav.carga.size()) >= nav.coleta.capacidade;   // carregando: NAO pega outro, mas tambem NAO pode passar por cima: desvia
    const unsigned long agora = c.agoraMs();
    if (agora < esperaAte) { c.parar(); velMmSAtual = 0.0f; tick(c); return true; }   // parado: a velocidade propria e 0 (senao o eco parece se mexer)
    if (agora - ultimoUsMs < 60) return false;
    const float dt = static_cast<float>(agora - ultimoUsMs);
    ultimoUsMs = agora;
    float d = static_cast<float>(c.us());
    bool deMemoria = false;
    if (ROBO_TRACE(agora)) fprintf(stderr, "      US t=%.2f eixo (%.0f,%.0f) th %.0f d=%.0f andadoRua=%.0f vel=%.0f\n", agora / 1000.0, c.estado().x, c.estado().y, c.estado().th * 57.3, d, andadoRuaMm, velMmSAtual);
    // MEMORIA DO ECO: depois de uma curva o carro balanca de 5 a 10 graus ao realinhar na rua e o cone (+-10) pode perder um cubo no meio da rua justo quando ele chega na distancia
    // de parar (o eco de 82 mm numa leitura e sem eco na seguinte: ele passava por cima). Um eco perto que some quando ainda falta andar ate ele vale como o eco menos o que andou.
    if (d > 0.0f && d <= 200.0f) { ecoLembradoMm = d; ecoLembradoAndadoMm = andadoRuaMm; }
    else if (d <= 0.0f && ecoLembradoMm > 0.0f) {
      const float restante = ecoLembradoMm - (andadoRuaMm - ecoLembradoAndadoMm);
      if (andadoRuaMm >= ecoLembradoAndadoMm && restante > -20.0f && restante <= nav.alvo.cuboParaDePertoMm + 15.0f) { d = std::max(restante, 1.0f); deMemoria = true; }
      else if (andadoRuaMm < ecoLembradoAndadoMm || restante <= -20.0f) ecoLembradoMm = -1.0f;
    }
    const bool movel = ecoMovel(usAnt, d, dt, velMmSAtual, nav.alvo);
    usAnt = d;
    // O eco enquanto anda tambem e informacao (ate ~3 ruas a frente na mesma linha): atualiza a memoria a cada 300 ms.
    if (trechoRestante0 > 0.0 && nav.ruaUsada && agora - ultimaObsRaioMs >= 300 && !movel) {
      ultimaObsRaioMs = agora;
      const double andadoRaio = andadoDesdeSaida(c, agora);
      // Sem eco logo depois de uma curva nao prova nada: o carro ainda esta ~40 mm fora da linha da rua e o cone (+-10 graus) passa ao lado de um cubo no meio dela (e marcava a rua
      // com cubo como vazia de certeza: o eco virava "o outro robo" e ele passava por cima). So vale depois de realinhar na rua (~110 mm andados).
      if (d > 0.0f || andadoRaio >= 110.0)
        nav.observarRaio(nav.ruaDe, nav.ruaPara.c - nav.ruaDe.c, nav.ruaPara.r - nav.ruaDe.r, d, static_cast<float>(andadoRaio), d <= 0.0f);
    }
    // O eco esta NESTA rua se a superficie dele fica antes do proximo cruzamento: da ponta (eixo + 100) ate o cruzamento faltam (trecho restante + 38 - 100) e um cubo da rua fica a 112+ mm
    // antes dele. Conta-se o que ja andou pela velocidade MEDIDA; se nao da para contar (meia-volta no meio da rua), a regra fica desligada.
    const double andado = andadoDesdeSaida(c, agora);
    const double faltaDaPonta = trechoRestante0 - andado + 38.0 - 100.0 - 87.0;
    if (trechoRestante0 > 0.0 && d > 0.0f && d <= faltaDaPonta && nav.ruaVaziaCerta(nav.ruaDe, nav.ruaPara)) {
      // Rua que ja foi vista vazia: o que o ultrassom ve ali so pode ser o OUTRO ROBO (parado ou nao). Regras 7.1: identificar e voltar para buscar outra rota; nunca empurrar.
      nav.oponenteVisto++;
      if (++nav.esperasSeguidas <= 3) { esperaAte = agora + 800; velMmSAtual = 0.0f; c.evento(fmt("eco a %.0f mm numa rua VAZIA de certeza (%d,%d)->(%d,%d) crenca %.2f, andado %.0f falta %.0f: so pode ser o outro robo, espera [%s]", d, nav.ruaDe.c, nav.ruaDe.r, nav.ruaPara.c, nav.ruaPara.r, [&] { core::CubeBelief cp = nav.cubos; return core::streetSlot(cp, nav.ruaDe, nav.ruaPara); }(), andado, faltaDaPonta, c.ecoTipo().c_str())); return true; }
      nav.esperasSeguidas = 0;
      c.evento("o outro robo continua na rua: volta e busca outra rota");
      nav.aoBloquear();   // trata como rua bloqueada por enquanto (o mapa de cubos nao muda: p ja e 0)
      nav.bloqueios--;
      trechoRestante0 = -1.0;
      voltarDoCubo(c);
      return true;
    }
    nav.esperasSeguidas = 0;
    if (d <= 0.0f || d > nav.alvo.cuboParaDePertoMm + 15.0f) {
      if (movel && d > 0.0f && d < 450.0f) { esperaAte = agora + 700; c.evento(fmt("eco que SE MEXE a %.0f mm: e o outro robo, espera", d)); }
      return false;
    }
    // Eco PERTO (na distancia de parar): para e identifica de qualquer jeito. "Se mexe" nao vale aqui: no comeco da rua o cone varre de lado enquanto o carro corrige e o eco de um
    // CUBO parado muda de valor (o carro ficava "esperando o outro robo" e depois passava por cima do cubo). Se nao for cubo, a identificacao abaixo espera e tenta de novo.
    c.parar();
    velMmSAtual = 0.0f;
    for (int i = 0; i < 6; i++) tick(c);   // le a cor parado
    const seguidor::CorCubo cor = c.tcsFrente();
    const int idx = c.cuboNaFrente(100.0);
    ROBO_DBG("    CUBO? t=%.1fs d=%.0f idx=%d cor=%d carga=%zu robo (%.0f,%.0f) th %.0f\n", c.agoraMs() / 1000.0, d, idx, static_cast<int>(cor), nav.carga.size(), c.estado().x, c.estado().y, c.estado().th * 57.3);
    if (idx < 0 || cor == seguidor::CorCubo::Nenhuma) {
      if (deMemoria && !ecoRealinhado) {   // o eco veio da memoria: o carro esta torto (5 a 10 graus) e o cone nao pega o cubo: aponta para o eixo da rua e le de novo
        ecoRealinhado = true;
        girarPara(c, 90.0 * lround(rumo / 90.0));
        ultimoUsMs = 0; usAnt = -1.0f;
        return true;
      }
      ecoLembradoMm = -1.0f; ecoRealinhado = false; esperaAte = agora + 500; return true;   // eco que nao e cubo
    }   // eco que nao e cubo (outro robo parado?) ou cor ilegivel: espera e tenta de novo
    if (cheio) {   // cubo no caminho de quem ja esta carregando: para, lembra dele (para pegar depois da entrega) e volta por outro caminho
      nav.aoBloquearTemporario();
      trechoRestante0 = -1.0;
      c.evento(fmt("cubo cor %d no caminho de quem ja carrega: nao passa por cima, lembra dele e volta por outra rua", static_cast<int>(cor)));
      voltarDoCubo(c);
      return true;
    }
    if (podeColetar(cor, nav.coleta)) {
      if (!c.coletar(idx)) {   // nao conseguiu pegar: o cubo continua la, como cubo no caminho (lembra dele e volta por outra rua, sem ficar tentando para sempre)
        nav.aoBloquearTemporario();
        trechoRestante0 = -1.0;
        c.evento(fmt("nao conseguiu pegar o cubo cor %d: lembra dele e volta por outra rua", static_cast<int>(cor)));
        voltarDoCubo(c);
        return true;
      }
      nav.aoColetar(cor);
      portaoPendente = true;   // levando o cubo: a azul no fim da rua e o NOSSO portao
      c.mostrarCarga(nav.carga.size() == 1 ? 0 : 1, static_cast<int>(cor));
      c.evento(fmt("COLETOU cubo cor %d na rua (%d,%d)-(%d,%d) (leva %d de %d)", static_cast<int>(cor), nav.ruaDe.c, nav.ruaDe.r, nav.ruaPara.c, nav.ruaPara.r, static_cast<int>(nav.carga.size()), nav.coleta.capacidade));
      usAnt = -1.0f; ecoLembradoMm = -1.0f; ecoRealinhado = false;
      return true;
    }
    // branco ou cor sem plataforma: identifica, sinaliza e BLOQUEIA a rua (regras 7)
    if (std::find(nav.identificados.begin(), nav.identificados.end(), static_cast<size_t>(idx)) == nav.identificados.end()) {
      nav.identificados.push_back(static_cast<size_t>(idx));
      nav.pontos += 50;
      nav.brancosIdentificados++;
      c.evento(fmt("SINALIZOU (bipe + LED): cubo cor %d sem plataforma identificado, +50 (total %d pontos)", static_cast<int>(cor), nav.pontos));
    }
    nav.aoBloquear();
    trechoRestante0 = -1.0;
    c.evento(fmt("rua (%d,%d)-(%d,%d) BLOQUEADA pelo cubo (nao se mexe), volta e procura outra rota", nav.ruaDe.c, nav.ruaDe.r, nav.ruaPara.c, nav.ruaPara.r));
    voltarDoCubo(c);
    return true;
  }

  // Linha proibida (vermelha, ou azul fora do portao) sob o sensor: para na hora e da meia-volta para dentro da cidade (ou do verde). Nao segue como se fosse uma rua.
  void reagirAProibida(Carro& c) {
    Fatia fatia(*this, c, Fase::Borda);
    const bool azul = c.tcsChao() == CorLinha::Azul;
    c.parar();
    bordas++;
    if (!nav.ativo && temCubo) {   // levando cubo sem navegador: so para e deixa a rota para quem decide (como na borda por cruzamento)
      paradasNavegador++;
      parado = true;
      return;
    }
    c.evento(fmt("%s sob o sensor em (%.0f,%.0f): para e da meia-volta (nao e rua)", azul ? "AZUL do outro lado" : "VERMELHO", c.estado().x, c.estado().y));
    std::vector<AmostraGiro> est, ver;
    if (nav.ativo && !nav.loc.inGreen) nav.aoProibido(azul ? core::Forbidden::OtherBlue : core::Forbidden::Red);
    girarAte(c, 0.0f, 180.0f, est, ver, c.estado().th, rumo);
    meiasVoltas++;
    esquecerUltimoComando(memoria);
    memoria.perdeuEm = 0;
    esquecerCor(memCor);
    cruz = MemoriaCruzamento{};
    proibidasSeguidas = 0;
    semReacaoAte = c.agoraMs() + 1500;   // deixa a barra sair da faixa antes de reagir de novo
    tUltimoCruzamento = c.agoraMs();
  }

  // Um ciclo do loop().
  void passo(Carro& c) {
    if (const int toques = c.consumirToques()) nav.pontos -= 5 * toques;
    if (parado) { c.parar(); tick(c); return; }
    bool p[kSensores];
    lerBarra(c, p);
    const unsigned long agora = c.agoraMs();
    {   // observador de metas: so conta o que e objetivo da partida
      const int metas = nav.coletados + nav.entregues + portoes + (nav.plataformasObservadas ? 1 : 0);
      if (metas != marcoMetas || ultimaMetaMs == 0) { marcoMetas = metas; ultimaMetaMs = agora; }
      else if (nav.ativo && !esperandoAjuda && !cruzandoPortao && agora - ultimaMetaMs > kMaxSemMetaMs) {
        esperandoAjuda = true;
        pedidosDeAjuda++;
        c.parar();
        c.evento(fmt("PEDINDO AJUDA: %lu s sem nenhuma meta (pose %s). Parado e apitando: recoloque o carro no verde", (agora - ultimaMetaMs) / 1000, nav.loc.inGreen ? "no verde" : "na cidade"));
      }
    }
    if (esperandoAjuda) c.sinal((agora / 400) % 2 == 0);   // apita em pulsos
    if (agora - ultimoTcsMs >= c.tcsPeriodoMs()) {   // o TCS do chao nao le a cada ciclo
      ultimoTcsMs = agora;
      const CorLinha lida = c.tcsChao();
      const bool proibida = lida == CorLinha::Vermelho || (lida == CorLinha::Azul && !portaoPendente && !nav.entregando && !indoParaOeste());   // vermelho: limite da cidade; azul fora do portao: area do outro lado
      proibidasSeguidas = proibida ? proibidasSeguidas + 1 : 0;
      azulSeguidas = lida == CorLinha::Azul ? azulSeguidas + 1 : 0;
      verdesSeguidos = lida == CorLinha::Verde ? verdesSeguidos + 1 : 0;   // uma leitura isolada de verde pode ser erro do sensor: so vale com varias seguidas
      if (nav.ativo && verdesSeguidos >= kVerdesParaReposicionar && (esperandoAjuda || (!nav.loc.inGreen && reposicionadoPeloVerde(CorLinha::Verde, portaoPendente, nav.entregando)))) {   // pediu ajuda: o verde sob o sensor E o humano recolocando   // verde inesperado: o juiz pegou e pos de volta
        nav.reposicionado();
        precisaAlinhar = true;
        reposicionados++;
        rumo = 0.0f;   // sem MPU o rumo so vem dos encoders: quem recoloca o carro o poe de frente para a cidade (rumo 0); o rumo de antes nao vale mais
        memoria = Memoria{};
        cruz = MemoriaCruzamento{};
        esquecerCor(memCor);
        portaoPendente = true;
        c.evento(fmt("VERDE inesperado em (%.0f,%.0f): reposicionado (pose e rumo reiniciados, o mapa de cubos fica)", c.estado().x, c.estado().y));
        if (esperandoAjuda) { esperandoAjuda = false; c.sinal(false); ultimaMetaMs = agora; }
      }
    }
    if (esperandoAjuda) { c.parar(); tick(c); return; }   // parado ate alguem recolocar o carro no verde
    // Regras 7: "o robo deve responder de forma imediata sempre que QUALQUER sensor detectar vermelho no chao". Vermelho e azul (fora do portao) nunca sao rota: os sensores RC
    // leem essas linhas como preto, e antes disso o carro as seguia como se fossem uma rua (a moldura da arena). So uma leitura vale se alguma ponta da barra ve preto; sozinha, duas.
    if (agora >= semReacaoAte && proibidasSeguidas >= 2 && !nav.entregando) {   // duas leituras seguidas: uma so pode ser erro do sensor (e o carro dava meia-volta no meio da rua)
      reagirAProibida(c);
      return;
    }
    // O nosso portao e uma faixa azul: se a barra a pega de lado (o carro nao chegou de frente) os sensores a leem como uma linha preta e o carro a SEGUIRIA como rua (descendo por ela).
    // Com a azul sob o TCS e o portao esperado, ele so atravessa em frente, segurando o rumo, e conta o portao quando a azul acaba.
    if (nav.ativo && portaoPendente && !nav.carregando && !indoParaOeste() && nav.loc.inGreen && azulSeguidas >= 2 && !cruzandoPortao) { cruzandoPortao = true; cruzandoPortaoDesde = agora; }
    if (cruzandoPortao) {
      Comando cmd{};
      cmd.esq = cmd.dir = ajustes.velReto;
      segurarRumoNaLacuna(cmd, static_cast<float>(remainder(rumo, 360.0)), 0.0f, ajustes);
      c.comandar(cmd.esq, cmd.dir);
      velMmSAtual = ajC.velMmS;
      tick(c);
      // A azul acabou (150 ms depois): atravessou o portao. Mas a rua da linha pode ficar ao lado do eixo (a entrada erra uns 40 mm de y) e, sem linha na barra, a busca achava a faixa
      // azul ATRAS (a mais perto) e a seguia como rua. Por isso continua em frente, segurando o rumo, ate algum sensor ver preto de novo (a rua) ou 150 mm a mais.
      // O carro nao comeca numa rua reta: saindo do verde ele pode chegar de lado na primeira rua (as 2 pontas da barra pretas ao mesmo tempo). Seguir uma linha PARCIAL dali o fazia
      // girar no lugar atras dela (o rumo ia a 155 graus). Entao segura o rumo ate as pontas fecharem (a rua que cruza, ~168 mm depois do portao): essa e a 1a juncao, que o cruzamento
      // normal confirma, e o navegador decide para que lado virar. Sem as pontas, o fim e o de antes (algum preto, ou 1,3 s).
      const bool algumPreto = acesos(p) > 0;
      const bool pontas = p[0] && p[kSensores - 1];
      const bool achouRuaDeLado = pontas && saiuDoVerde;
      if ((saiuDoVerde && agora - cruzandoPortaoDesde > 60 && algumPreto && (azulSeguidas == 0 || !portaoAteAzulAcabar)) || agora - cruzandoPortaoDesde > 1300) {   // entrou no tabuleiro (barra no branco) e a rua apareceu em algum sensor; branco total = ainda no beco: segue em frente
        cruzandoPortao = false;
        portoes++;
        portaoPendente = false;
        semReacaoAte = agora + 1500;
        nav.aoPortao();
        trechoRestante0 = -1.0;
        c.evento(fmt("PORTAO azul atravessado em (%.0f,%.0f) (de frente ou de lado)", c.estado().x, c.estado().y));
        esquecerUltimoComando(memoria);
        cruz = MemoriaCruzamento{};
        if (!achouRuaDeLado) comecarASair(cruz, agora, odoCruz(c));   // com as pontas pretas a barra JA esta na 1a juncao: sem "saindo", o cruzamento normal a confirma no proximo ciclo
      }
      return;
    }
    if (nav.ativo && nav.loc.inGreen && c.missao() && !inicioFeito) { inicioFeito = true; entrarNaCidade(c, NAN); return; }
    if (nav.ativo && nav.loc.inGreen && precisaAlinhar) { alinharNaRua(c); return; }
    const bool semLinha = !(p[0] || p[1] || p[2] || p[3] || p[4] || p[5] || p[6] || p[7]);
    if (!(nav.ativo && nav.loc.inGreen && semLinha)) semLinhaNoVerdeMm = -1.0;
    else if (semLinhaNoVerdeMm < 0.0) semLinhaNoVerdeMm = c.odoMm();
    // O verde tem ~370 mm ate o portao: andar mais que kMaxVerdeSemLinhaMm sem ver NENHUMA linha e pose errada (o carro 'reposicionado' fora do nosso verde), nao verde. Sem este limite ele
    // seguia reto para sempre (15 km em 8 min no simulador); passado o limite cai na busca normal.
    if (nav.ativo && nav.loc.inGreen && semLinha && fabs(c.odoMm() - semLinhaNoVerdeMm) <= kMaxVerdeSemLinhaMm) {   // no verde: sem linha, anda reto para o portao
      Comando cmd{};
      cmd.parar = false;
      cmd.acao = "no verde: de frente para a cidade e reto ate o portao";
      const float erro = static_cast<float>(remainder(rumo, 360.0));   // + = apontando para a esquerda de "frente"
      if (fabsf(erro) > 15.0f) {   // fora de rumo: gira no lugar ate apontar para a cidade (rumo 0), nao anda torto
        const int v = ajustes.velBuscaExterna;
        cmd.esq = erro > 0 ? v : -v;   // apontando a esquerda: gira para a direita
        cmd.dir = -cmd.esq;
      } else {
        cmd.esq = cmd.dir = ajustes.velReto;
        segurarRumoNaLacuna(cmd, erro, 0.0f, ajustes);
      }
      c.comandar(cmd.esq, cmd.dir);
      velMmSAtual = ajC.velMmS;
      tick(c);
      return;
    }
    if (nav.ativo && tratarCubo(c)) return;
    if (paraNoCruzamento && cruzamentoConfirmado(p, agora, ajC, cruz, odoCruz(c))) {   // acima de todas as outras regras: vale ate perdido/buscando
      pararNoCruzamento(c, p);
      return;
    }
    const unsigned long perdeuAntes = memoria.perdeuEm;
    const bool vinhaSeguindo = memoria.ultimoEsq != 0 || memoria.ultimoDir != 0;   // so segura o rumo se vinha andando: perdido sem nunca ter seguido, nao anda cego
    Comando cmd = decidir(p, agora, ajustes, memoria);
    ultimaAcao = cmd.acao;
    // As ruas da arena sao eixos: seguindo reto (barra centrada na linha) por kRetoParaReancorarMm o carro esta alinhado com a rua, e o rumo dos encoders (que deriva com cada giro da
    // linha e da busca: 20 graus no portao no simulador) volta ao multiplo de 90 mais perto. Sem isto a entrega virava para o oeste 20 graus torta e perdia a marca da plataforma.
    if (cmd.esq == cmd.dir && cmd.esq > 0 && !cmd.parar && !estaBuscando(memoria, agora, ajustes)) {
      if (retoDesdeMm < 0.0) retoDesdeMm = c.odoMm();
      else if (c.odoMm() - retoDesdeMm >= kRetoParaReancorarMm) {
        const float e = remainderf(rumo, 90.0f);
        if (fabsf(e) < 30.0f) rumo -= e;
      }
    } else {
      retoDesdeMm = -1.0;
    }
    for (int i = 0; i < kSensores; i++) ultimaBarra[i] = p[i];
    const unsigned long perdidoHa = memoria.perdeuEm ? agora - memoria.perdeuEm : 0;
    if (memoria.perdeuEm) ticksPerdido++;
    if (estaBuscando(memoria, agora, ajustes)) ticksBuscando++;
    if (!fimDeBecoAvisado && tUltimoCruzamento &&
        classificarPerda(perdidoHa, agora - tUltimoCruzamento, ajustes.lacunaMs, ajC) == Perda::FimDeBeco) {
      fimDeBecoAvisado = true;
      fimDeBeco = true;
    }
    if (memoria.perdeuEm != 0 && perdeuAntes == 0) rumoAoPerder = rumo;
    if (!ROBO_SEM("SIM_SEM_LACUNA") && vinhaSeguindo && memoria.perdeuEm != 0 && agora - memoria.perdeuEm < ajustes.lacunaMs) segurarRumoNaLacuna(cmd, rumo, rumoAoPerder, ajustes);
    c.comandar(cmd.parar ? 0 : cmd.esq, cmd.parar ? 0 : cmd.dir);
    if (cfg.encoder) {   // velocidade e percurso lidos das rodas
      velMmSAtual = static_cast<float>((c.odoMm() - odoAnt) / 0.005);
      andadoRuaMm = static_cast<float>(c.odoMm() - odoSaida);
    } else {
      velMmSAtual = (cmd.esq > 0 && cmd.dir > 0 && !cmd.parar) ? ajC.velMmS * static_cast<float>(cmd.esq + cmd.dir) / (2.0f * static_cast<float>(ajustes.velReto)) : 0.0f;
      andadoRuaMm += velMmSAtual * 0.005f;
    }
    odoAnt = c.odoMm();
    fase = estaBuscando(memoria, agora, ajustes) ? Fase::Buscar : (cmd.esq == cmd.dir && cmd.esq > 0 ? Fase::Seguir : Fase::Corrigir);
    tick(c);
  }
};
}  // namespace seguidor
