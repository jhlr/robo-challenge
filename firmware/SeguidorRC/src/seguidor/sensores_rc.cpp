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
    if (!podeCarregar(pinos[i])) continue;
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
  for (int i = 0; i < kSensores; i++) b.preto[i] = b.ok[i] && b.leituras[i] > b.limiar[i];
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

}  // namespace seguidor
