// Geometria do tabuleiro e das plataformas do ecoponto: dono unico dos numeros, usado pelo simulador (mundo e desenho) e pelo robo.
#pragma once
#include <math.h>

#include "alvo.h"

namespace seguidor {

struct Estado { double x, y, th; };   // eixo (mm), rumo (rad, 0 = +x, anti-horario); so o simulador sabe a verdade, o carro real devolve zeros

// O que o TCS do chao ve: vermelho/azul (linha colorida), verde/amarelo (zonas), preto (linha), branco.
enum Piso { kPisoBranco, kPisoPreto, kPisoVermelho, kPisoAzul, kPisoVerde, kPisoAmarelo };

struct Plataforma { double x, y; seguidor::CorCubo cor; };   // 160 x 50 mm no ecoponto (zona amarela)

// O tabuleiro (docs/2026-10-28_mapa-comparacao-e-localizacao.md): 6 x 5 cruzamentos a 400 mm, becos de 168 mm (rua de 40 cm; becos e ecoponto: medidas da imagem na escala 400/302).
constexpr double kPasso = 400.0, kBeco = 168.0, kLarg = 2 * kBeco + 5 * kPasso, kAlt = 2 * kBeco + 4 * kPasso;   // 2336 x 1936 (rua de 40 cm; becos e ecoponto na mesma escala da imagem)
inline double xDoCruzamento(int c) { return kBeco + c * kPasso; }
inline double yDoCruzamento(int r) { return kBeco + r * kPasso; }
// Plataformas do ecoponto (regras 4): retangulos de 160 x 50 mm EM PE (50 mm em x, 160 mm em y), 100 mm entre eles (passo de 260 mm), CENTRADOS no ecoponto de 220 mm
// (equidistantes das duas bordas: 85 mm de cada lado) e na altura da arena (cinco = 1200 mm, 130 mm de folga em cima e embaixo). O robo assume essa geometria (so a COR muda
// de um jogo para outro): dono unico dos numeros, usado pelo mundo, pelo desenho e pelo robo.
constexpr double kEcopontoLarg = 291.0, kEcopontoBordaX = -371.0;   // borda do ecoponto voltada para o verde
constexpr double kPlatX_Tam = 50.0, kPlatY_Tam = 160.0, kPlatEntre = 100.0;
constexpr double kPlatX = kEcopontoBordaX - kEcopontoLarg / 2.0;   // -516,5: centro, 120 mm de cada borda
inline double yDaPlataforma(double slot) { return kAlt / 2.0 + (slot - 2) * (kPlatY_Tam + kPlatEntre); }   // slot 0..4, de baixo para cima

}  // namespace seguidor
