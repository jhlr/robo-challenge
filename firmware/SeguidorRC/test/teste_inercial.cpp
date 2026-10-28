// g++ -std=c++17 -Wall -Isrc test/teste_inercial.cpp -o /tmp/teste_inercial && /tmp/teste_inercial
#include <cassert>
#include <cmath>
#include <cstdio>

#include "seguidor/inercial.h"

using namespace seguidor;

static Imu parado() { return {0.0f, 0.0f, 0.0f, 1.0f}; }

int main() {
  const AjustesInercial ai;
  const Ajustes aj;

  { // ancora: seguindo reta com o rumo 12 graus fora de 0, puxa para 0; o vies tambem anda para o giro medio da reta
    float rumo = 12.0f, vies = 0.0f;
    Ancora a;
    for (unsigned long t = 0; t < 12000; t += 5) {   // varios tau do vies (3 s)
      assert(!ancorarNaGrade(rumo, vies, 0.02f, true, t, a, ai));   // giro medio 0,02 rad/s = vies
    }
    assert(std::fabs(rumo) < 0.5f);
    assert(std::fabs(vies - 0.02f) < 0.005f); }
  { // ancora so age depois de retoMinMs em reta; uma curva no meio zera a contagem
    float rumo = 20.0f, vies = 0.0f;
    Ancora a;
    for (unsigned long t = 0; t < ai.retoMinMs - 10; t += 5) ancorarNaGrade(rumo, vies, 0.0f, true, t, a, ai);
    assert(rumo == 20.0f);
    ancorarNaGrade(rumo, vies, 0.0f, false, ai.retoMinMs, a, ai);   // saiu da reta
    for (unsigned long t = ai.retoMinMs + 5; t < ai.retoMinMs + 100; t += 5) ancorarNaGrade(rumo, vies, 0.0f, true, t, a, ai);
    assert(rumo == 20.0f); }
  { // multiplo de 90 mais proximo: 95 -> 90; e perto de 180 -> 180
    float rumo = 95.0f, vies = 0.0f;
    Ancora a;
    for (unsigned long t = 0; t < 4000; t += 5) ancorarNaGrade(rumo, vies, 0.0f, true, t, a, ai);
    assert(std::fabs(rumo - 90.0f) < 0.5f);
    rumo = -176.0f; a = Ancora{};
    for (unsigned long t = 0; t < 4000; t += 5) ancorarNaGrade(rumo, vies, 0.0f, true, t, a, ai);
    assert(std::fabs(rumo + 180.0f) < 0.5f); }
  { // rumo a 45 graus de qualquer multiplo numa reta: suspeito, e NAO corrige
    float rumo = 45.0f, vies = 0.0f;
    Ancora a;
    bool suspeito = false;
    for (unsigned long t = 0; t < 1000; t += 5) suspeito |= ancorarNaGrade(rumo, vies, 0.0f, true, t, a, ai);
    assert(suspeito && rumo == 45.0f); }
  { // depois de uma pausa (giro no cruzamento) o primeiro dt e enorme: nao pode saltar o vies para o giro instantaneo, nem ancorar de imediato
    float rumo = 0.0f, vies = 0.01f;
    Ancora a;
    for (unsigned long t = 0; t < 1000; t += 5) ancorarNaGrade(rumo, vies, 0.01f, true, t, a, ai);
    rumo = 7.0f;
    ancorarNaGrade(rumo, vies, 1.2f, true, 5000, a, ai);   // 4 s depois, girando forte
    assert(std::fabs(vies - 0.01f) < 1e-4f);               // giro forte nao vira vies
    assert(rumo == 7.0f); }                                // e a contagem de reta recomecou
  { Comando c; c.esq = c.dir = 110; c.parar = false; assert(ehReto(c)); c.largo = true; assert(!ehReto(c)); }

  { // levantado: inclinacao sustentada -> FoiLevantado UMA vez; solto no chao, so reposiciona depois de quietoMs quieto
    Pickup p;
    unsigned long t = 0;
    int levantou = 0, repos = 0;
    auto passo = [&](const Imu& m) {
      const EventoPostura e = atualizarPickup(p, m, t, ai);
      levantou += e == EventoPostura::FoiLevantado;
      repos += e == EventoPostura::Reposicionado;
      t += 5;
    };
    for (int i = 0; i < 200; i++) passo(parado());
    assert(levantou == 0 && motoresLiberados(p));
    const Imu inclinado = {0.0f, 0.5f, 0.0f, 0.8f};   // ~32 graus
    for (int i = 0; i < 20; i++) passo(inclinado);     // 100 ms: ainda nao
    assert(levantou == 0);
    for (int i = 0; i < 100; i++) passo(inclinado);
    assert(levantou == 1 && !motoresLiberados(p));
    for (int i = 0; i < 100; i++) passo(parado());     // no chao, mas so 500 ms quieto
    assert(repos == 0 && !motoresLiberados(p));
    Imu tremendo = parado(); tremendo.gzRadS = 0.5f;   // mexeu: recomeca a contar
    for (int i = 0; i < 40; i++) passo(tremendo);
    for (int i = 0; i < 250; i++) passo(parado());     // 1250 ms
    assert(repos == 0);
    for (int i = 0; i < 100; i++) passo(parado());
    assert(repos == 1 && motoresLiberados(p)); }
  { // uma batida curta nao e levantar
    Pickup p;
    unsigned long t = 0;
    for (int i = 0; i < 200; i++) { assert(atualizarPickup(p, parado(), t, ai) == EventoPostura::Nada); t += 5; }
    assert(atualizarPickup(p, {0, 0, 0, 2.0f}, t, ai) == EventoPostura::Nada);   // 1 amostra de pico
    t += 5;
    for (int i = 0; i < 100; i++) assert(atualizarPickup(p, parado(), t += 5, ai) == EventoPostura::Nada);
    assert(motoresLiberados(p)); }

  { // travado: comandando e sem vibracao -> reforca, ate recuar
    Tracao t;
    unsigned long ms = 0;
    int reforcos = 0, recuos = 0;
    for (int i = 0; i < 2000; i++) {
      const AcaoTracao a = atualizarTracao(t, parado(), true, ms, ai);
      reforcos += a == AcaoTracao::Reforcar;
      recuos += a == AcaoTracao::Recuar;
      ms += 5;
    }
    assert(reforcos >= 1 && t.reforco <= ai.reforcoMax && recuos >= 1); }
  { // andando (vibracao) ou girando: nao e travado
    Tracao t;
    unsigned long ms = 0;
    Imu vibra = {0.0f, 0.0f, 0.0f, 1.08f};
    for (int i = 0; i < 1000; i++) { vibra.az = (i % 2) ? 1.08f : 0.92f; assert(atualizarTracao(t, vibra, true, ms, ai) == AcaoTracao::Nada); ms += 5; }
    Imu gira = parado(); gira.gzRadS = 1.0f;
    Tracao t2;
    for (int i = 0; i < 1000; i++) { assert(atualizarTracao(t2, gira, true, ms, ai) == AcaoTracao::Nada); ms += 5; }
    Tracao t3;   // parado de verdade mas SEM comandar: tambem nao
    for (int i = 0; i < 1000; i++) { assert(atualizarTracao(t3, parado(), false, ms, ai) == AcaoTracao::Nada); ms += 5; } }
  { Comando c; c.esq = -90; c.dir = 90; c.parar = false;
    aplicarReforco(c, 10);
    assert(c.esq == -100 && c.dir == 100);
    Comando z; z.esq = 0; z.dir = 250; z.parar = false;
    aplicarReforco(z, 10);
    assert(z.esq == 0 && z.dir == 255); }
  { assert(reforcoPelaVelocidade(0, 200.0f, 250.0f, ai) > 0);      // 80%: sobe
    assert(reforcoPelaVelocidade(0, 240.0f, 250.0f, ai) == 0); }   // 96%: deixa

  { // colisao: pico forte uma vez; fraco (cubo) nao; levantado nao conta
    Colisao c;
    assert(!colisaoDetectada(c, {0, 0.3f, 0.0f, 1.0f}, true, 0, ai));
    assert(colisaoDetectada(c, {0, 2.0f, 0.0f, 1.0f}, true, 100, ai));
    assert(!colisaoDetectada(c, {0, 2.0f, 0.0f, 1.0f}, true, 200, ai));   // refratario
    assert(colisaoDetectada(c, {0, 2.0f, 0.0f, 1.0f}, true, 700, ai));
    Colisao d;
    assert(!colisaoDetectada(d, {0, 2.0f, 0.0f, 1.0f}, false, 0, ai) && d.picoG > 1.9f); }

  { // lacuna: virou 5 graus para a esquerda -> roda esquerda mais rapida; para a direita -> a direita
    Comando c; c.parar = false;
    segurarRumoNaLacuna(c, 5.0f, 0.0f, aj, ai);
    assert(c.esq > aj.velReto && c.dir < aj.velReto && c.esq + c.dir == 2 * aj.velReto);
    segurarRumoNaLacuna(c, -5.0f, 0.0f, aj, ai);
    assert(c.esq < aj.velReto && c.dir > aj.velReto);
    segurarRumoNaLacuna(c, 90.0f, 0.0f, aj, ai);   // limitado
    assert(c.esq - aj.velReto == ai.lacunaMaxPwm);
    segurarRumoNaLacuna(c, 0.0f, 0.0f, aj, ai);
    assert(c.esq == aj.velReto && c.dir == aj.velReto); }
  std::puts("teste_inercial: ok");
  return 0;
}
