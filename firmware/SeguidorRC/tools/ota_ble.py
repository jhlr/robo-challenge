#!/usr/bin/env python3
"""Grava firmware no SeguidorRC por Bluetooth LE (protocolo em src/seguidor/ota_ble.h).

    python3 -m venv ~/.venvs/seguidor && ~/.venvs/seguidor/bin/pip install bleak   # uma vez (o pip3 do sistema recusa: PEP 668)
    cd firmware && pio run -e seguidor
    ~/.venvs/seguidor/bin/python SeguidorRC/tools/ota_ble.py                      # usa .pio/build/seguidor/firmware.bin
    ~/.venvs/seguidor/bin/python SeguidorRC/tools/ota_ble.py caminho/firmware.bin --segredo meu-segredo

O robo precisa estar PARADO e sem outro aparelho conectado (feche o console_ble.py). O segredo tem de ser o mesmo do OTA_SEGREDO do sketch.
O primeiro firmware com OTA (e a tabela de particoes min_spiffs) vai por USB: pio run -e seguidor -t upload.
"""
import argparse
import asyncio
import hashlib
import hmac
import struct
import sys
from pathlib import Path

from bleak import BleakClient, BleakScanner

OTA_CONTROLE = "9B2A0002-6F1D-4C6E-8B3A-5D0F2C7E1A00"
OTA_DADOS = "9B2A0003-6F1D-4C6E-8B3A-5D0F2C7E1A00"
PADRAO = Path(__file__).resolve().parents[2] / ".pio" / "build" / "seguidor" / "firmware.bin"
ERROS = {1: "protocolo", 2: "segredo errado", 3: "tamanho (nao cabe na particao)", 4: "gravacao na flash", 5: "sha256 nao confere", 6: "imagem invalida"}


def mensagem_iniciar(imagem: bytes, segredo: str) -> bytes:
    """0x01 | tamanho u32 | sha256 | HMAC-SHA256(segredo, tamanho u32 || sha256)."""
    tamanho = struct.pack("<I", len(imagem))
    sha = hashlib.sha256(imagem).digest()
    mac = hmac.new(segredo.encode(), tamanho + sha, hashlib.sha256).digest()
    return b"\x01" + tamanho + sha + mac


async def enviar(imagem: bytes, nome: str, segredo: str) -> None:
    print(f"Procurando '{nome}' por Bluetooth LE (10 s)...")
    dispositivo = await BleakScanner.find_device_by_name(nome, timeout=10.0)
    if dispositivo is None:
        sys.exit(f"Nao achei '{nome}'. Ligado e sem outro aparelho conectado?")

    respostas: asyncio.Queue[tuple[int, int]] = asyncio.Queue()

    def ao_notificar(_, dados: bytearray) -> None:
        respostas.put_nowait((dados[0], struct.unpack_from("<I", dados, 1)[0]))

    async def esperar(codigo: int, tempo: float = 10.0) -> int:
        while True:
            cod, arg = await asyncio.wait_for(respostas.get(), tempo)
            if cod == 0x1F:
                sys.exit(f"\nO robo recusou: {ERROS.get(arg, arg)}")
            if cod == codigo:
                return arg

    async with BleakClient(dispositivo) as cliente:
        await cliente.start_notify(OTA_CONTROLE, ao_notificar)
        pacote = max(20, min(cliente.mtu_size - 3, 244))
        print(f"{len(imagem)} bytes, pacotes de {pacote}")
        await cliente.write_gatt_char(OTA_CONTROLE, mensagem_iniciar(imagem, segredo), response=True)
        janela = await esperar(0x10)
        enviados = 0
        while enviados < len(imagem):
            fim_janela = min(enviados + janela, len(imagem))
            while enviados < fim_janela:
                parte = imagem[enviados:min(enviados + pacote, fim_janela)]
                await cliente.write_gatt_char(OTA_DADOS, parte, response=False)
                enviados += len(parte)
            recebidos = await esperar(0x11)
            if recebidos != enviados:
                sys.exit(f"\nPerdeu bytes: enviei {enviados}, o robo gravou {recebidos}")
            print(f"\r{enviados * 100 // len(imagem):3d}%  {enviados}/{len(imagem)}", end="", flush=True)
        print()
        await cliente.write_gatt_char(OTA_CONTROLE, b"\x02", response=True)
        await esperar(0x12)
        print("OK: imagem conferida, o robo esta reiniciando.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("firmware", nargs="?", type=Path, default=PADRAO)
    ap.add_argument("--nome", default="SeguidorRC")
    ap.add_argument("--segredo", default="troque-este-segredo")
    args = ap.parse_args()
    if not args.firmware.is_file():
        sys.exit(f"Nao achei {args.firmware} (rode: cd firmware && pio run -e seguidor)")
    asyncio.run(enviar(args.firmware.read_bytes(), args.nome, args.segredo))
