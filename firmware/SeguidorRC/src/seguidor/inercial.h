#pragma once
// Inercial (MPU6050): o que a barra de sensores nao ve. Tudo PURO (so conta; o tempo e o MPU entram como argumento), entao testa no PC.
//   - rumo: o yaw vem de core/yaw.h (dono da integracao do giroscopio); aqui so se ancora esse rumo na grade do mapa
//   - levantado: carregado/tombado -> motores parados; solto e quieto por um tempo -> "reposicionado" (a pose volta para "no verde, de frente")
//   - travado: roda comandada mas o robo nao se mexe -> reforca o PWM e, se nao adiantar, recua
//   - colisao: pico de aceleracao forte (oponente; cubo e leve e fica abaixo do limiar)
//   - lacuna: segura o rumo pelo giroscopio enquanto a linha some por pouco tempo
// Unidades: giroscopio em rad/s (como core/yaw.h), aceleracao em g, rumo em graus (+ = esquerda, 0 = de frente para dentro da cidade).
#include <math.h>
#include <stdint.h>

#include "decisao.h"

namespace seguidor {

constexpr float kRadParaGrau = 57.29578f;

struct Imu {
  float gzRadS;   // giroscopio Z (sem tirar o vies)
  float ax, ay, az;   // g; az = para cima com o robo no chao
};

// Thresholds sao CHUTES: grave os picos reais na pista e corrija aqui.
struct AjustesInercial {
  // Ancora na grade (as ruas do mapa sao ortogonais: seguindo reta, o rumo tem de ser multiplo de 90 graus)
  unsigned long retoMinMs = 300;   // seguindo reta por tanto tempo antes de ancorar
  float tauAncoraMs = 600.0f;      // constante de tempo com que o rumo vai para o multiplo de 90
  float tauViesMs = 3000.0f;       // idem para o vies do giroscopio (numa reta o giro medio e o vies)
  unsigned long lacunaChamadasMs = 200;   // chamadas mais espacadas que isto (o robo ficou parado girando no cruzamento): a contagem de reta recomeca
  float viesPortaRadS = 0.15f;             // o vies so aprende quando o giro esta perto dele (reta de verdade); giro forte nao e vies
  float suspeitoGraus = 35.0f;     // mais longe que isto de um multiplo de 90 numa reta: o rumo estimado nao serve (rumoSuspeito)

  // Levantado
  float inclinacaoLevantadoGraus = 20.0f;   // gravidade inclinada mais que isto (ou |a| longe de 1 g) por levantadoMs
  float desvioNormaLevantadoG = 0.35f;
  unsigned long levantadoMs = 150;
  float quietoInclinacaoGraus = 8.0f;       // "estabilizou": no chao, parado, ha quietoMs seguidos
  float quietoGzGrauS = 4.0f;
  float quietoNormaG = 0.08f;
  unsigned long quietoMs = 1500;

  // Travado: comandando (alguma roda acima da zona) e sem vibracao nem giro por travadoMs
  float vibracaoParadoG = 0.02f;
  float gzParadoGrauS = 3.0f;
  unsigned long travadoMs = 500;
  int reforcoPasso = 10;     // PWM somado a cada travada
  int reforcoMax = 40;
  int travadasAteRecuar = 4;   // depois de tantos reforcos sem sair do lugar, recua

  // Colisao (oponente: bate de frente a ~250 mm/s e para em poucos ms; cubo leve fica bem abaixo)
  float colisaoG = 1.2f;
  unsigned long colisaoRefratarioMs = 500;

  // Lacuna: correcao do rumo em PWM por grau de erro, e o maximo
  float kpLacuna = 2.0f;
  int lacunaMaxPwm = 25;
};

inline float normaG(const Imu& m) { return sqrtf(m.ax * m.ax + m.ay * m.ay + m.az * m.az); }
inline float inclinacaoGraus(const Imu& m) {
  const float n = normaG(m);
  return n < 1e-3f ? 0.0f : acosf(fmaxf(-1.0f, fminf(1.0f, m.az / n))) * kRadParaGrau;
}

// ---------------------------------------------------------------- rumo ancorado na grade

struct Ancora {
  unsigned long retoDesde = 0;   // +1; 0 = nao esta em reta
  unsigned long ultimoMs = 0;
};

// Seguindo reta (a decisao mandou RETO) por retoMinMs, o rumo tem de ser multiplo de 90: puxa o rumo para o multiplo mais proximo e o vies do giroscopio
// para o giro medio da reta (que deveria ser zero). Devolve true se o rumo estava longe demais de qualquer multiplo (rumoSuspeito): nao corrige,
// o localizador deve confiar menos na direcao. `viesRadS` e o mesmo vies de core/yaw.h.
inline bool ancorarNaGrade(float& rumoGraus, float& viesRadS, float gzRadS, bool seguindoReto, unsigned long agoraMs, Ancora& a, const AjustesInercial& aj) {
  const float dt = a.ultimoMs ? static_cast<float>(agoraMs - a.ultimoMs) : 0.0f;
  const bool houvePausa = a.ultimoMs != 0 && agoraMs - a.ultimoMs > aj.lacunaChamadasMs;
  a.ultimoMs = agoraMs;
  if (!seguindoReto || houvePausa) { a.retoDesde = 0; if (!seguindoReto) return false; }
  if (a.retoDesde == 0) a.retoDesde = agoraMs + 1;
  if (agoraMs + 1 - a.retoDesde < aj.retoMinMs) return false;
  const float alvo = roundf(rumoGraus / 90.0f) * 90.0f;
  const float residuo = rumoGraus - alvo;
  if (fabsf(residuo) > aj.suspeitoGraus) return true;
  rumoGraus -= residuo * fminf(1.0f, dt / aj.tauAncoraMs);
  if (fabsf(gzRadS - viesRadS) < aj.viesPortaRadS) viesRadS += (gzRadS - viesRadS) * fminf(1.0f, dt / aj.tauViesMs);
  return false;
}

// A decisao mandou andar reto (as duas rodas iguais para a frente, sem perder a linha)?
inline bool ehReto(const Comando& c) { return !c.parar && c.esq == c.dir && c.esq > 0 && !c.largo; }

// ---------------------------------------------------------------- levantado / reposicionado

enum class Postura : uint8_t { Normal, Levantado, Estabilizando };
enum class EventoPostura : uint8_t { Nada, FoiLevantado, Reposicionado };

struct Pickup {
  Postura postura = Postura::Normal;
  unsigned long anomaliaDesde = 0;   // +1
  unsigned long quietoDesde = 0;     // +1
};

// Chame uma vez por leitura do MPU. Reposicionado devolve UMA vez, depois de levantado e solto no chao quieto por quietoMs:
// ai o rumo vai para 0 (SEMPRE de frente para dentro da cidade) e a pose volta para "em algum lugar do verde" (o mapa de cubos fica).
inline EventoPostura atualizarPickup(Pickup& p, const Imu& m, unsigned long agoraMs, const AjustesInercial& aj) {
  const bool anomalo = inclinacaoGraus(m) > aj.inclinacaoLevantadoGraus || fabsf(normaG(m) - 1.0f) > aj.desvioNormaLevantadoG;
  const bool quieto = inclinacaoGraus(m) < aj.quietoInclinacaoGraus && fabsf(m.gzRadS * kRadParaGrau) < aj.quietoGzGrauS && fabsf(normaG(m) - 1.0f) < aj.quietoNormaG;

  if (p.postura == Postura::Normal) {
    if (!anomalo) { p.anomaliaDesde = 0; return EventoPostura::Nada; }
    if (p.anomaliaDesde == 0) p.anomaliaDesde = agoraMs + 1;
    if (agoraMs + 1 - p.anomaliaDesde < aj.levantadoMs) return EventoPostura::Nada;
    p.postura = Postura::Levantado;
    p.quietoDesde = 0;
    return EventoPostura::FoiLevantado;
  }
  // Levantado ou estabilizando: so volta quando fica QUIETO no chao por quietoMs seguidos
  if (!quieto) { p.postura = Postura::Levantado; p.quietoDesde = 0; return EventoPostura::Nada; }
  p.postura = Postura::Estabilizando;
  if (p.quietoDesde == 0) p.quietoDesde = agoraMs + 1;
  if (agoraMs + 1 - p.quietoDesde < aj.quietoMs) return EventoPostura::Nada;
  p = Pickup{};
  return EventoPostura::Reposicionado;
}

inline bool motoresLiberados(const Pickup& p) { return p.postura == Postura::Normal; }

// ---------------------------------------------------------------- travado / andando menos que o esperado

enum class AcaoTracao : uint8_t { Nada, Reforcar, Recuar };

struct Tracao {
  float vibracao = 0.0f;           // media movel de | |a| - 1 |
  unsigned long travadoDesde = 0;  // +1
  unsigned long ultimoMs = 0;
  int reforco = 0;                 // PWM somado a toda velocidade comandada
  int travadas = 0;
};

// `comandando` = alguma roda esta acima da zona morta (o robo deveria estar se mexendo ou girando).
inline AcaoTracao atualizarTracao(Tracao& t, const Imu& m, bool comandando, unsigned long agoraMs, const AjustesInercial& aj) {
  const float dt = t.ultimoMs ? static_cast<float>(agoraMs - t.ultimoMs) : 0.0f;
  t.ultimoMs = agoraMs;
  t.vibracao += (fabsf(normaG(m) - 1.0f) - t.vibracao) * fminf(1.0f, dt / 100.0f);
  const bool parado = t.vibracao < aj.vibracaoParadoG && fabsf(m.gzRadS * kRadParaGrau) < aj.gzParadoGrauS;
  if (!comandando || !parado) { t.travadoDesde = 0; if (!comandando) t.travadas = 0; return AcaoTracao::Nada; }
  if (t.travadoDesde == 0) t.travadoDesde = agoraMs + 1;
  if (agoraMs + 1 - t.travadoDesde < aj.travadoMs) return AcaoTracao::Nada;
  t.travadoDesde = 0;
  if (++t.travadas > aj.travadasAteRecuar) { t.travadas = 0; return AcaoTracao::Recuar; }
  if (t.reforco >= aj.reforcoMax) return AcaoTracao::Recuar;
  t.reforco += aj.reforcoPasso;
  if (t.reforco > aj.reforcoMax) t.reforco = aj.reforcoMax;
  return AcaoTracao::Reforcar;
}

// Soma o reforco (em modulo) a toda roda que esta se mexendo.
inline void aplicarReforco(Comando& c, int reforco) {
  if (c.parar || reforco <= 0) return;
  if (c.esq > 0) c.esq = c.esq + reforco > 255 ? 255 : c.esq + reforco;
  else if (c.esq < 0) c.esq = c.esq - reforco < -255 ? -255 : c.esq - reforco;
  if (c.dir > 0) c.dir = c.dir + reforco > 255 ? 255 : c.dir + reforco;
  else if (c.dir < 0) c.dir = c.dir - reforco < -255 ? -255 : c.dir - reforco;
}

// Andando menos do que o esperado (distancia entre dois cruzamentos / tempo): sobe o reforco ate a velocidade real chegar perto da esperada.
inline int reforcoPelaVelocidade(int reforcoAtual, float velRealMmS, float velEsperadaMmS, const AjustesInercial& aj) {
  if (velEsperadaMmS <= 0.0f) return reforcoAtual;
  const float razao = velRealMmS / velEsperadaMmS;
  if (razao < 0.85f && reforcoAtual < aj.reforcoMax) return reforcoAtual + aj.reforcoPasso / 2;
  return reforcoAtual;
}

// ---------------------------------------------------------------- colisao

struct Colisao {
  unsigned long ultimaEm = 0;   // +1
  float picoG = 0.0f;           // maior pico horizontal visto (para ajustar colisaoG na pista)
};

// Pico de aceleracao horizontal acima de colisaoG (e nao levantado). true UMA vez por batida.
inline bool colisaoDetectada(Colisao& c, const Imu& m, bool normal, unsigned long agoraMs, const AjustesInercial& aj) {
  const float h = sqrtf(m.ax * m.ax + m.ay * m.ay);
  if (h > c.picoG) c.picoG = h;
  if (!normal || h < aj.colisaoG) return false;
  if (c.ultimaEm != 0 && agoraMs + 1 - c.ultimaEm < aj.colisaoRefratarioMs) return false;
  c.ultimaEm = agoraMs + 1;
  return true;
}

// ---------------------------------------------------------------- lacuna na fita: segura o rumo

// Corrige um comando de lacuna (a linha sumiu ha pouco) para manter o rumo em que ela foi perdida. rumo + = esquerda: se virou para a esquerda,
// a roda esquerda acelera e a direita freia, para voltar. A lacuna anda reto na velocidade reta (nao repete o ultimo comando, que podia ser um giro).
inline void segurarRumoNaLacuna(Comando& c, float rumoGraus, float rumoAoPerderGraus, const Ajustes& aj, const AjustesInercial& ai) {
  float d = ai.kpLacuna * (rumoGraus - rumoAoPerderGraus);
  d = fmaxf(-static_cast<float>(ai.lacunaMaxPwm), fminf(static_cast<float>(ai.lacunaMaxPwm), d));
  c.esq = aj.velReto + static_cast<int>(d);
  c.dir = aj.velReto - static_cast<int>(d);
}

}  // namespace seguidor
