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
import threading
import time

from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError

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

    # stdin numa thread so: o loop de conexao nao pode ficar preso numa leitura quando o robo cai (reset, brownout)
    loop = asyncio.get_running_loop()
    linhas: asyncio.Queue[str] = asyncio.Queue()

    def ler_stdin() -> None:
        for linha in sys.stdin:
            loop.call_soon_threadsafe(linhas.put_nowait, linha)

    threading.Thread(target=ler_stdin, daemon=True).start()

    while True:
        caiu = asyncio.Event()
        try:
            async with BleakClient(dispositivo, disconnected_callback=lambda _: loop.call_soon_threadsafe(caiu.set)) as cliente:
                await cliente.start_notify(NUS_TX, ao_receber)
                print(f"Conectado a {dispositivo.name} ({dispositivo.address}). Digite comandos; 'ajuda' lista os do robo.")
                while not caiu.is_set():
                    pega = asyncio.ensure_future(linhas.get())
                    queda = asyncio.ensure_future(caiu.wait())
                    feito, _ = await asyncio.wait({pega, queda}, return_when=asyncio.FIRST_COMPLETED)
                    if pega not in feito:
                        pega.cancel()
                        break
                    queda.cancel()
                    # o aplicativo escreve em pacotes pequenos; o robo junta ate o fim da linha
                    dados = pega.result().encode()
                    for i in range(0, len(dados), 100):
                        await cliente.write_gatt_char(NUS_RX, dados[i : i + 100], response=False)
        except BleakError as erro:
            print(f"\n[BLE: {erro}]")
        print("\n[Desconectado: o robo reiniciou ou saiu do alcance. Procurando de novo...]")
        dispositivo = None
        while dispositivo is None:
            dispositivo = await BleakScanner.find_device_by_name(nome, timeout=10.0)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--nome", default="SeguidorRC")
    ap.add_argument("--log", default=None)
    args = ap.parse_args()
    try:
        asyncio.run(main(args.nome, args.log))
    except KeyboardInterrupt:
        pass
