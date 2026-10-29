#!/usr/bin/env python3
"""Console Bluetooth LE para o SeguidorRC (Nordic UART Service): mostra o que o robo escreve e envia as linhas que voce digita.

    pip3 install bleak
    python3 tools/console_ble.py                 # procura por "SeguidorRC"
    python3 tools/console_ble.py --nome OutroNome
    python3 tools/console_ble.py --log sessao.txt   # guarda tudo (com hora) em arquivo

Digite `ajuda` (ou `h`) para a lista de comandos do robo. Ctrl+C sai.
"""
import argparse
import asyncio
import sys
import time

from bleak import BleakClient, BleakScanner

NUS_RX = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   # computador -> robo
NUS_TX = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   # robo -> computador


async def main(nome: str, log_path: str | None) -> None:
    print(f"Procurando '{nome}' por Bluetooth LE (10 s)...")
    dispositivo = await BleakScanner.find_device_by_name(nome, timeout=10.0)
    if dispositivo is None:
        sys.exit(f"Nao achei '{nome}'. O robo esta ligado e nao conectado a outro aparelho (o celular, por exemplo)?")
    log = open(log_path, "a", buffering=1) if log_path else None
    buffer = ""

    def ao_receber(_, dados: bytearray) -> None:
        nonlocal buffer
        texto = dados.decode(errors="replace")
        sys.stdout.write(texto)
        sys.stdout.flush()
        if log:
            buffer += texto
            while "\n" in buffer:
                linha, buffer = buffer.split("\n", 1)
                log.write(f"{time.strftime('%H:%M:%S')} {linha}\n")

    async with BleakClient(dispositivo) as cliente:
        await cliente.start_notify(NUS_TX, ao_receber)
        print(f"Conectado a {dispositivo.name} ({dispositivo.address}). Digite comandos; 'ajuda' lista os do robo.")
        loop = asyncio.get_running_loop()
        while cliente.is_connected:
            linha = await loop.run_in_executor(None, sys.stdin.readline)
            if not linha:
                break
            # o aplicativo escreve em pacotes pequenos; o robo junta ate o fim da linha
            dados = linha.encode()
            for i in range(0, len(dados), 100):
                await cliente.write_gatt_char(NUS_RX, dados[i : i + 100], response=False)
    print("Desconectado.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--nome", default="SeguidorRC")
    ap.add_argument("--log", default=None)
    args = ap.parse_args()
    try:
        asyncio.run(main(args.nome, args.log))
    except KeyboardInterrupt:
        pass
