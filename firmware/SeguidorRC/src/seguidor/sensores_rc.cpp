#include "sensores_rc.h"

namespace seguidor {

namespace {
bool podeCarregar(int pino) { return pino < 34; }

void copiarLeituras(BarraRC& b, uint16_t crus[kSensores]) {
  for (int i = 0; i < kSensores; i++) b.leituras[i] = b.tempoMaxUs;   // morto: fica no maximo e a calibracao o marca
  for (int k = 0; k < b.quantos; k++) b.leituras[b.indice[k]] = crus[k];
}
}  // namespace

void iniciar(BarraRC& b, const uint8_t pinos[kSensores]) {
  b.pinos = pinos;
  uint8_t vivos[kSensores];
  b.quantos = 0;
  for (int i = 0; i < kSensores; i++) {
    if (!podeCarregar(pinos[i]) || (b.mortos >> i) & 1) continue;
    vivos[b.quantos] = pinos[i];
    b.indice[b.quantos++] = i;
  }
  b.qtr.setTypeRC();
  b.qtr.setSensorPins(vivos, b.quantos);
  b.qtr.setTimeout(b.tempoMaxUs);
}

void ler(BarraRC& b) {
  uint16_t crus[kSensores];
  b.qtr.read(crus, QTRReadMode::Off);   // sem pino de emissor: os LEDs ficam sempre acesos
  copiarLeituras(b, crus);
  if (b.adaptativo) {
    bool mudou = false;
    unsigned int maior = b.limiarUs;
    for (int i = 0; i < kSensores; i++) {
      if (!b.ok[i]) continue;
      if (b.leituras[i] < b.branco[i]) {
        b.branco[i] = max(b.leituras[i], 400u);   // abaixo de 400 us nao e leitura de chao
        b.limiar[i] = constrain(b.branco[i] + (b.tempoMaxUs - b.branco[i]) / 2, b.limiarUs, b.tempoMaxUs - 200);
        mudou = true;
      }
      maior = max(maior, b.limiar[i]);
    }
    if (mudou) { b.tempoLeituraUs = min(b.tempoMaxUs, maior + 100); b.qtr.setTimeout(b.tempoLeituraUs); }   // a espera cobre o maior limiar
  }
  for (int i = 0; i < kSensores; i++) b.preto[i] = b.ok[i] && b.leituras[i] > b.limiar[i];
  for (int i = 0; i < kSensores; i++) {   // sensor morto (b.mortos): copia o vizinho vivo mais perto
    if (!((b.mortos >> i) & 1)) continue;
    for (int d = 1; d < kSensores; d++) {
      if (i + d < kSensores && b.ok[i + d]) { b.preto[i] = b.preto[i + d]; break; }
      if (i - d >= 0 && b.ok[i - d]) { b.preto[i] = b.preto[i - d]; break; }
    }
  }
}

bool aplicarLimiar(BarraRC& b, Print& saida) {
  int validos = 0;
  for (int i = 0; i < kSensores; i++) {
    b.ok[i] = false;
    b.limiar[i] = b.limiarUs;
  }
  for (int k = 0; k < b.quantos; k++) {
    b.ok[b.indice[k]] = true;
    validos++;
  }
  b.tempoLeituraUs = min(b.tempoMaxUs, b.limiarUs + 100);
  b.qtr.setTimeout(b.tempoLeituraUs);   // so espera ate o limiar: quem ainda esta em HIGH la e preto

  ler(b);
  saida.printf("Limiar fixo %u us (%d sensores vivos). Leitura agora (us):", b.limiarUs, validos);
  for (int i = 0; i < kSensores; i++) saida.printf(" S%d=%u%s", i + 1, b.leituras[i], b.ok[i] ? "" : "(morto)");
  saida.println();
  return validos >= b.sensoresMinimos;
}

void reiniciarAdaptacao(BarraRC& b) {
  for (int i = 0; i < kSensores; i++) {
    b.branco[i] = b.tempoMaxUs;
    b.limiar[i] = b.limiarUs;
  }
}

}  // namespace seguidor
