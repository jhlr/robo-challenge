# Depuracao por Bluetooth (BLE) — 2026-10-28

O ESP32-S3 so tem BLE (sem Bluetooth classico/SPP). O sketch `SeguidorBasico` expoe um "serial" no padrao Nordic UART Service:
tudo que vai ao `dbg` sai no USB **e** no BLE; linhas recebidas viram comandos. Nome anunciado: `SeguidorRC`. Desligar: `-DUSA_BLE=0`.

Codigo: `firmware/SeguidorRC/src/seguidor/depuracao_ble.h` (DepuracaoBLE, Tee), comandos em `aoReceberComando()` no sketch.
Compila (ESP32 core 3.3.12, 649 KB de flash, 22% de RAM).

## Usar
- Computador: `pip3 install bleak` e `python3 firmware/SeguidorRC/tools/console_ble.py` (`--log arquivo.txt` grava).
- Celular: nRF Connect ou Serial Bluetooth Terminal (modo BLE), servico NUS.

## Comandos
`ajuda`, `estado`, `parar`, `seguir`, `teste on/off` (motores desligados), `debug on/off`, `tele on/off` (CSV ~10 Hz: t_ms,rumo,S1..S8,esq,dir),
`ajustes`, `ajuste <nome> <valor>` (velReto, velCorrecaoLeve, velCorrecaoForte, zonaMortaPwm, periodoCorrecaoMs), `zerar` (rumo = 0).

## Pendencias
- Nao testado no robo (so compilado). Fila de saida 4 KB: com conexao lenta as linhas mais novas sao descartadas e o aviso mostra quantos bytes.
- Navegador (core/) ainda nao esta ligado no sketch; so o simulador roda a missao completa.
