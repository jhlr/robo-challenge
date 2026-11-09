#!/usr/bin/env python3
"""Ponte BLE do carro: fica conectada ao "RoboChallenge", guarda tudo que ele escreve e aceita comandos por HTTP em localhost.
Existe para um agente (ou voce, com curl) observar e comandar o carro sem prender o Bluetooth numa aba de console. Rode no terminal do app (a permissao de Bluetooth e dele):

    ~/.venvs/seguidor/bin/python firmware/tools/robo_bridge.py            # porta 8765, log em ~/.cache/robo/ble.log

    curl -s localhost:8765/status
    curl -s -X POST localhost:8765/send -d '{"cmd": "s"}'
    curl -s 'localhost:8765/log?desde=0&espera=5'

So aceita os comandos do carro: g [s], x, s, h. A ponte segura a conexao BLE (o carro aceita um aparelho so): pare-a (Ctrl+C) antes de usar console_ble.py ou o OTA.
"""
import argparse
import asyncio
import json
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError

NUS_RX = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   # computador -> carro
NUS_TX = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   # carro -> computador
COMANDO_OK = re.compile(r"^(g( \d{1,4})?|x|s|h)$")

dados = {"texto": "", "conectado": False, "linhas": 0}
trava = threading.Lock()
loop_ble: asyncio.AbstractEventLoop | None = None
cliente: BleakClient | None = None
arquivo_log = None


def guardar(texto: str) -> None:
    with trava:
        dados["texto"] += texto
        dados["linhas"] += texto.count("\n")
        if len(dados["texto"]) > 200_000:   # janela: o log em arquivo guarda tudo
            cortar = len(dados["texto"]) - 200_000
            dados["texto"] = dados["texto"][cortar:]
            dados["base"] = dados.get("base", 0) + cortar
    if arquivo_log:
        arquivo_log.write(texto)


async def manter_conexao(nome: str) -> None:
    global cliente
    while True:
        dados["conectado"] = False
        dispositivo = await BleakScanner.find_device_by_name(nome, timeout=10.0)
        if dispositivo is None:
            continue
        try:
            async with BleakClient(dispositivo) as c:
                cliente = c
                await c.start_notify(NUS_TX, lambda _, b: guardar(b.decode(errors="replace")))
                dados["conectado"] = True
                guardar(f"\n[ponte {time.strftime('%H:%M:%S')}: conectada a {nome}]\n")
                while c.is_connected:
                    await asyncio.sleep(0.5)
        except (BleakError, OSError, asyncio.TimeoutError) as e:
            guardar(f"\n[ponte: erro BLE {e}]\n")
        guardar(f"\n[ponte {time.strftime('%H:%M:%S')}: desconectada, procurando de novo]\n")
        cliente = None
        await asyncio.sleep(1.0)


async def enviar(cmd: str) -> str:
    if cliente is None or not cliente.is_connected:
        return "sem conexao com o carro"
    await cliente.write_gatt_char(NUS_RX, (cmd + "\n").encode(), response=False)
    return "ok"


class Manipulador(BaseHTTPRequestHandler):
    def _json(self, codigo: int, corpo: dict) -> None:
        bruto = json.dumps(corpo).encode()
        self.send_response(codigo)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(bruto)))
        self.end_headers()
        self.wfile.write(bruto)

    def do_GET(self) -> None:
        url = urlparse(self.path)
        q = parse_qs(url.query)
        if url.path == "/status":
            return self._json(200, {"conectado": dados["conectado"], "bytes": dados.get("base", 0) + len(dados["texto"])})
        if url.path == "/log":
            desde = int(q.get("desde", ["0"])[0])
            espera = min(float(q.get("espera", ["0"])[0]), 60.0)
            fim = time.time() + espera
            while True:
                with trava:
                    base = dados.get("base", 0)
                    total = base + len(dados["texto"])
                    if total > desde or time.time() >= fim:
                        texto = dados["texto"][max(desde - base, 0):]
                        break
                time.sleep(0.1)
            return self._json(200, {"texto": texto, "proximo": total, "conectado": dados["conectado"]})
        self._json(404, {"erro": "use /status, /log ou POST /send"})

    def do_POST(self) -> None:
        if urlparse(self.path).path != "/send":
            return self._json(404, {"erro": "use POST /send"})
        corpo = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
        cmd = str(corpo.get("cmd", "")).strip().lower()
        if not COMANDO_OK.match(cmd):
            return self._json(400, {"erro": f"comando recusado: {cmd!r}. Aceitos: g [s], x, s, h"})
        futuro = asyncio.run_coroutine_threadsafe(enviar(cmd), loop_ble)
        guardar(f"\n[ponte >>> {cmd}]\n")
        self._json(200, {"resultado": futuro.result(timeout=5)})

    def log_message(self, *_: object) -> None:
        pass


def main() -> None:
    global loop_ble, arquivo_log
    ap = argparse.ArgumentParser()
    ap.add_argument("--nome", default="RoboChallenge")
    ap.add_argument("--porta", type=int, default=8765)
    ap.add_argument("--log", type=Path, default=Path.home() / ".cache" / "robo" / "ble.log")
    args = ap.parse_args()
    args.log.parent.mkdir(parents=True, exist_ok=True)
    arquivo_log = open(args.log, "a", buffering=1)
    servidor = ThreadingHTTPServer(("127.0.0.1", args.porta), Manipulador)
    threading.Thread(target=servidor.serve_forever, daemon=True).start()
    print(f"ponte em http://127.0.0.1:{args.porta}, log em {args.log}. Procurando '{args.nome}'... (Ctrl+C sai)")
    loop_ble = asyncio.new_event_loop()
    asyncio.set_event_loop(loop_ble)
    try:
        loop_ble.run_until_complete(manter_conexao(args.nome))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
