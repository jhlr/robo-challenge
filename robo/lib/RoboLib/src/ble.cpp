#include "ble.h"

#include "depuracao_ble.h"
#include "ota_ble.h"

namespace {
constexpr const char* kNome = "RoboChallenge";
constexpr const char* kSegredo = "robo-challenge-2026";   // o mesmo do --segredo do ota_ble.py; sem ele ninguem grava firmware por BLE

seguidor::DepuracaoBLE ble;
seguidor::OtaBLE ota;
seguidor::Tee tee(Serial, ble);
volatile char ultimoComando = 0;
}  // namespace

void bleIniciar() {
  ble.iniciar(kNome, [](const String& linha) { if (linha.length()) ultimoComando = linha[0]; });
  ota.iniciar(ble.servidor(), kSegredo);
  ble.anunciar();
}

void bleAtualizar() {
  ble.atualizar();
  ota.atualizar();
}

bool bleAtualizando() { return ota.ativo(); }

char bleComando() {
  const char c = ultimoComando;
  ultimoComando = 0;
  return c;
}

Print& saida() { return tee; }
