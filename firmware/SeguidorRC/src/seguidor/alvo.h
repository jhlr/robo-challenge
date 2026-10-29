#pragma once
// Alvos do ultrassom (e do TCS da frente): o que o eco e. Puro, so conta. Tudo CONFIGURAVEL e desligado por padrao no robo real ate o sensor existir;
// o simulador ja modela os dois (alcance, cone, ruido) para testar a logica antes.
//   - ruaTemCubo: na varredura do cruzamento o ultrassom olha ao longo de cada rua; eco mais perto que o comprimento da rua = ha algo nela. Os cubos so ficam nas
//     ruas (nunca no cruzamento), entao um eco ali e um cubo (ou o oponente: veja ecoMovel).
//   - ecoMovel: DISTINGUE O OPONENTE DE UM CUBO DE LONGE. Um cubo e parado: andando a v mm/s para ele, a distancia cai v*dt. Se o eco muda de outro jeito (cai mais
//     rapido, sobe, ou some e volta), o alvo se mexe: e o oponente, e se o robo esta parado qualquer variacao e movimento.
//   - classificarCuboFrente: o TCS da frente (a ~5 cm) diz a cor do cubo; branco ou cor sem plataforma = nao mexer (rua bloqueada).
#include <math.h>
#include <stdint.h>

namespace seguidor {

struct AjustesAlvo {
  // O ultrassom fica na ponta da frente (eixo + 100 mm) e o eixo para ~38 mm ANTES do centro do cruzamento: o proximo cruzamento esta a ~240 mm da ponta, e os cubos da
  // rua do lado (a 112 a 190 mm do cruzamento) dao eco de 25 a 103 mm. Os cubos da rua SEGUINTE (alem do cruzamento) dao eco de 327 mm para cima: so vale como cubo NESTA rua
  // o eco ate aqui (senao ele 've' a rua de tras da esquina e a memoria enche de cubo onde nao ha).
  float ruaMm = 230.0f;
  float ecoMovelToleranciaMm = 25.0f;   // diferenca entre a variacao esperada (parado) e a medida que ja conta como alvo que se mexe
  float cuboParaDePertoMm = 60.0f;      // distancia em que o robo para e olha a cor do cubo
  float velMinimaMmS = 20.0f;           // abaixo disto o robo conta como parado
};

inline bool ruaTemCubo(float usMm, const AjustesAlvo& aj = {}) { return usMm > 0.0f && usMm < aj.ruaMm; }

// ---- O ultrassom NAO ve so a rua da frente: com alcance de ~900 mm, em linha reta ele alcanca ate 3 ruas (o cone e estreito: so cubos NO EIXO da rua). O eco vale como
// informacao e como objetivo: um cubo a 2 ruas de distancia ja entra na memoria. Geometria (mm): o eixo para 38 antes do centro do cruzamento, a ponta do sensor fica 100 a frente do
// eixo, o cubo tem 25 de meia largura e fica de 112 a 190 depois do cruzamento onde a rua comeca. `s` = distancia do CENTRO do cruzamento de onde a rua 0 comeca ate o centro do cubo.
constexpr float kPassoMm = 400.0f, kCuboMinMm = 112.0f, kCuboMaxMm = 190.0f, kPontaMaisMeioCuboMenosEixoMm = 100.0f + 25.0f - 38.0f;

// Em qual rua da linha reta a frente (0 = a primeira, que sai do cruzamento de partida) esta o cubo que deu este eco? `andadoMm` = quanto o eixo ja andou desde a parada no cruzamento
// (0 na varredura, parado). -1 se o eco cai perto de um cruzamento (ambiguo: nao e cubo de rua).
inline int ruaDoEco(float usMm, float andadoMm) {
  if (usMm <= 0.0f) return -1;
  const float s = andadoMm + usMm + kPontaMaisMeioCuboMenosEixoMm;
  const int k = static_cast<int>(s / kPassoMm);
  const float resto = s - k * kPassoMm;
  return (resto >= kCuboMinMm - 40.0f && resto <= kCuboMaxMm + 40.0f) ? k : -1;
}

// Sem eco ate `alcanceMm`: quais ruas (da primeira a ultima) ficaram TODAS vistas vazias? Devolve a ultima rua inteira dentro do alcance (-1 se nenhuma). Ruas que o carro ja passou
// (andadoMm grande) saem do comeco: so interessam as de k >= primeiraRuaAindaAFrente.
inline int ultimaRuaVistaVazia(float andadoMm, float alcanceMm) {
  const float sMax = andadoMm + alcanceMm + kPontaMaisMeioCuboMenosEixoMm;
  if (sMax < kCuboMaxMm) return -1;
  return static_cast<int>((sMax - kCuboMaxMm) / kPassoMm);
}
inline int primeiraRuaAindaAFrente(float andadoMm) {
  const float sMin = andadoMm + kPontaMaisMeioCuboMenosEixoMm;   // o que o sensor ja nao alcanca para tras
  const int k = static_cast<int>((sMin - kCuboMinMm + kPassoMm - 1.0f) / kPassoMm);
  return k < 0 ? 0 : k;
}

// d0 -> d1 em dtMs, enquanto o robo andava `velPropriaMmS` PARA o eco (negativo = se afastando). true = o alvo se mexeu por conta propria.
inline bool ecoMovel(float d0Mm, float d1Mm, float dtMs, float velPropriaMmS, const AjustesAlvo& aj = {}) {
  if (d0Mm <= 0.0f || d1Mm <= 0.0f || dtMs <= 0.0f) return false;
  const float esperado = -velPropriaMmS * dtMs / 1000.0f;   // variacao da distancia se o alvo estivesse parado
  const float medido = d1Mm - d0Mm;
  return fabsf(medido - esperado) > aj.ecoMovelToleranciaMm;
}

// As cores dos cubos (regras 5.1): azul, vermelho, verde, amarelo, preto e marrom (5 de cada) e branco (6). Branco nunca tem plataforma.
enum class CorCubo : uint8_t { Nenhuma, Branco, Vermelho, Azul, Verde, Amarelo, Preto, Marrom };
constexpr int kNumCoresCubo = 8;

// O que fazer com o cubo a frente (regras 7): so se pega a cor que TEM plataforma no nosso ecoponto (o juiz poe de 1 a 5 plataformas de cores distintas antes da partida).
// Branco, ou cor SEM plataforma, e tratado como branco: identificar, sinalizar (som/luz: 50 pontos, uma vez por cubo), NAO mexer e considerar a rua bloqueada.
// Configuravel: `coletavel` e preenchido com as cores das plataformas lidas no inicio.
struct AjustesColeta {
  bool coletavel[kNumCoresCubo] = {false, false, true, true, true, true, true, true};   // indexado por CorCubo: nenhum, branco, vermelho, azul, verde, amarelo, preto, marrom
  int capacidade = 1;   // as regras permitem ate dois cubos por viagem; a equipe escolheu UM (entrega logo, sem perambular carregando)
};

inline bool podeColetar(CorCubo c, const AjustesColeta& aj = {}) { return c != CorCubo::Nenhuma && aj.coletavel[static_cast<int>(c)]; }

}  // namespace seguidor
