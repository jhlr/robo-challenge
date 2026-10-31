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

bool calibrar(BarraRC& b, Print& saida) {
  b.qtr.resetCalibration();
  saida.println("Calibrando por 5 s: deslize o robo sobre a linha preta...");
  const unsigned long inicio = millis();
  while (millis() - inicio < 5000) b.qtr.calibrate(QTRReadMode::Off);

  int validos = 0;
  unsigned int maiorLimiar = 0;
  saida.println("Sensor  Branco  Preto  Limiar");
  for (int i = 0; i < kSensores; i++) {
    unsigned int minimo = b.tempoMaxUs, maximo = 0;
    for (int k = 0; k < b.quantos; k++) {
      if (b.indice[k] != i) continue;
      minimo = b.qtr.calibrationOn.minimum[k];
      maximo = b.qtr.calibrationOn.maximum[k];
    }
    b.limiar[i] = (minimo + maximo) / 2;
    b.ok[i] = maximo >= minimo && (maximo - minimo) >= b.faixaMinUs;
    if (b.ok[i]) {
      validos++;
      if (b.limiar[i] > maiorLimiar) maiorLimiar = b.limiar[i];
    }

    char linha[64];
    snprintf(linha, sizeof(linha), "  S%d     %4u   %4u    %4u", i + 1, minimo, maximo, b.limiar[i]);
    saida.print(linha);
    if (!b.ok[i]) saida.print(b.pinos[i] >= 34 ? "  <-- NAO REAGIU! GPIO so de entrada: nao carrega o RC. Sensor ignorado" : "  <-- NAO REAGIU! Sensor ignorado");
    saida.println();
  }

  if (validos < b.sensoresMinimos) return false;

  b.tempoLeituraUs = min(b.tempoMaxUs, maiorLimiar + 100);
  b.qtr.setTimeout(b.tempoLeituraUs);   // so espera ate o maior limiar: quem ainda esta em HIGH la e preto
  saida.printf("Calibracao concluida (%d sensores, leitura de %u us). Coloque o robo na linha.\n", validos, b.tempoLeituraUs);
  return true;
}

}  // namespace seguidor
