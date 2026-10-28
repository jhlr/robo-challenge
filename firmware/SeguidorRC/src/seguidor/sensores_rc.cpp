#include "sensores_rc.h"

#include <soc/gpio_reg.h>

namespace seguidor {

namespace {

// Espera ate `limite` us. Os 8 pinos sao lidos de uma vez no registrador de entrada
// (nada de digitalRead em laco) e a medida para assim que todos descarregam.
void lerTempos(BarraRC& b, unsigned int limite) {
  // 1) Carrega os capacitores dos sensores
  for (int i = 0; i < kSensores; i++) {
    pinMode(b.pinos[i], OUTPUT);
    digitalWrite(b.pinos[i], HIGH);
    b.leituras[i] = b.tempoMaxUs;
  }
  delayMicroseconds(10);

  // 2) Solta os pinos e mede quanto tempo cada um demora para cair
  for (int i = 0; i < kSensores; i++) pinMode(b.pinos[i], INPUT);

  uint32_t pendentes = b.mascaraTodos;   // pinos que ainda estao em HIGH
  const uint32_t inicio = micros();
  while (pendentes) {
    const uint32_t t = micros() - inicio;
    if (t >= limite) break;
    const uint32_t caiu = ~REG_READ(GPIO_IN_REG) & pendentes;
    if (caiu) {
      for (int i = 0; i < kSensores; i++) {
        if (caiu & b.mascara[i]) b.leituras[i] = t;
      }
      pendentes &= ~caiu;
    }
  }
}

}  // namespace

void iniciar(BarraRC& b, const uint8_t pinos[kSensores]) {
  b.pinos = pinos;
  b.mascaraTodos = 0;
  for (int i = 0; i < kSensores; i++) {
    b.mascara[i] = 1UL << pinos[i];
    b.mascaraTodos |= b.mascara[i];
  }
}

void ler(BarraRC& b) {
  // Basta esperar ate o maior limiar: quem ainda esta em HIGH la e preto
  lerTempos(b, b.tempoLeituraUs);
  for (int i = 0; i < kSensores; i++) b.preto[i] = b.ok[i] && b.leituras[i] > b.limiar[i];
}

bool calibrar(BarraRC& b, Print& saida) {
  unsigned int minimo[kSensores];
  unsigned int maximo[kSensores];
  for (int i = 0; i < kSensores; i++) {
    minimo[i] = b.tempoMaxUs;
    maximo[i] = 0;
  }

  saida.println("Calibrando por 5 s: deslize o robo sobre a linha preta...");
  const unsigned long inicio = millis();
  while (millis() - inicio < 5000) {
    lerTempos(b, b.tempoMaxUs);
    for (int i = 0; i < kSensores; i++) {
      if (b.leituras[i] < minimo[i]) minimo[i] = b.leituras[i];
      if (b.leituras[i] > maximo[i]) maximo[i] = b.leituras[i];
    }
  }

  int validos = 0;
  unsigned int maiorLimiar = 0;
  saida.println("Sensor  Branco  Preto  Limiar");
  for (int i = 0; i < kSensores; i++) {
    b.limiar[i] = (minimo[i] + maximo[i]) / 2;
    b.ok[i] = (maximo[i] - minimo[i]) >= b.faixaMinUs;
    if (b.ok[i]) {
      validos++;
      if (b.limiar[i] > maiorLimiar) maiorLimiar = b.limiar[i];
    }

    char linha[64];
    snprintf(linha, sizeof(linha), "  S%d     %4u   %4u    %4u", i + 1, minimo[i], maximo[i], b.limiar[i]);
    saida.print(linha);
    if (!b.ok[i]) saida.print("  <-- NAO REAGIU! Sensor ignorado");
    saida.println();
  }

  if (validos < b.sensoresMinimos) return false;

  b.tempoLeituraUs = min(b.tempoMaxUs, maiorLimiar + 100);   // +100 us de margem sobre o maior limiar
  saida.printf("Calibracao concluida (%d sensores, leitura de %u us). Coloque o robo na linha.\n", validos, b.tempoLeituraUs);
  return true;
}

}  // namespace seguidor
