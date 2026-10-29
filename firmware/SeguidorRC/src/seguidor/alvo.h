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
  float ruaMm = 330.0f;          // rua de 302 mm + folga: eco mais perto que isto esta NA rua
  float ecoMovelToleranciaMm = 25.0f;   // diferenca entre a variacao esperada (parado) e a medida que ja conta como alvo que se mexe
  float cuboParaDePertoMm = 60.0f;      // distancia em que o robo para e olha a cor do cubo
  float velMinimaMmS = 20.0f;           // abaixo disto o robo conta como parado
};

inline bool ruaTemCubo(float usMm, const AjustesAlvo& aj = {}) { return usMm > 0.0f && usMm < aj.ruaMm; }

// d0 -> d1 em dtMs, enquanto o robo andava `velPropriaMmS` PARA o eco (negativo = se afastando). true = o alvo se mexeu por conta propria.
inline bool ecoMovel(float d0Mm, float d1Mm, float dtMs, float velPropriaMmS, const AjustesAlvo& aj = {}) {
  if (d0Mm <= 0.0f || d1Mm <= 0.0f || dtMs <= 0.0f) return false;
  const float esperado = -velPropriaMmS * dtMs / 1000.0f;   // variacao da distancia se o alvo estivesse parado
  const float medido = d1Mm - d0Mm;
  return fabsf(medido - esperado) > aj.ecoMovelToleranciaMm;
}

enum class CorCubo : uint8_t { Nenhuma, Branco, Vermelho, Azul, Verde, Amarelo };

// O que fazer com o cubo a frente. Coletavel = cor que tem plataforma (configuravel); branco ou outra cor = bloqueia a rua.
struct AjustesColeta {
  bool coletavel[6] = {false, false, true, true, true, true};   // indexado por CorCubo: nenhum, branco, vermelho, azul, verde, amarelo
};

inline bool podeColetar(CorCubo c, const AjustesColeta& aj = {}) { return c != CorCubo::Nenhuma && aj.coletavel[static_cast<int>(c)]; }

}  // namespace seguidor
