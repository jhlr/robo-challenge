#!/bin/bash
# Compila e grava o carro por BLE (OTA). Rode numa aba do terminal do app (o macOS derruba o Bluetooth iniciado por agentes).
# uso: firmware/tools/gravar.sh [env=robo]
# Carro ligado, parado e sem outro aparelho conectado. Se a ponte (robo_bridge.py) estiver rodando, usa o /ota dela (so env robo);
# senao fala direto com o carro. Primeira gravacao (tabela de particoes) ou carro sem BLE: por USB, `pio run -e robo -t upload`.
set -e
cd "$(dirname "$0")/.."
env=${1:-robo}
pio run -e "$env"
if curl -s -m 2 localhost:8765/log >/dev/null 2>&1 && [ "$env" = robo ]; then
  curl -s -X POST localhost:8765/ota
else
  caffeinate -i ~/.venvs/seguidor/bin/python SeguidorRC/tools/ota_ble.py ".pio/build/$env/firmware.bin" --nome RoboChallenge --segredo robo-challenge-2026
fi
