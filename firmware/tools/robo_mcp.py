#!/usr/bin/env python3
"""Servidor MCP (stdio, sem dependencias) para o agente comandar e ler o carro pela ponte BLE (robo_bridge.py, que precisa estar rodando).
Ferramentas: robo_enviar(cmd) com g [s], x, s ou h; robo_ler(espera_s, desde) devolve o que o carro escreveu.

    claude mcp add robo -- ~/.venvs/seguidor/bin/python /caminho/firmware/tools/robo_mcp.py
"""
import json
import sys
import urllib.error
import urllib.request

BASE = "http://127.0.0.1:8765"
FERRAMENTAS = [
    {"name": "robo_enviar", "description": "Manda um comando ao carro pela ponte BLE: 'g [segundos]' inicia a partida (padrao 60 s), 'x' para e reinicia desarmado, 's' estado e leitura do chao, 'h' ajuda. O carro se move com g.",
     "inputSchema": {"type": "object", "properties": {"cmd": {"type": "string"}}, "required": ["cmd"]}},
    {"name": "robo_ler", "description": "Le o que o carro escreveu pelo BLE. 'desde' e o 'proximo' devolvido na leitura anterior (0 = tudo que a ponte guarda); 'espera_s' espera ate esse tempo por texto novo.",
     "inputSchema": {"type": "object", "properties": {"desde": {"type": "integer"}, "espera_s": {"type": "number"}}}},
]


def chamar(metodo: str, caminho: str, corpo: dict | None = None) -> dict:
    req = urllib.request.Request(BASE + caminho, data=json.dumps(corpo).encode() if corpo is not None else None, method=metodo)
    try:
        with urllib.request.urlopen(req, timeout=70) as r:
            return json.load(r)
    except urllib.error.HTTPError as e:
        return json.load(e)
    except OSError as e:
        return {"erro": f"ponte fora do ar ({e}). Rode firmware/tools/robo_bridge.py no terminal do app."}


def ferramenta(nome: str, args: dict) -> str:
    if nome == "robo_enviar":
        return json.dumps(chamar("POST", "/send", {"cmd": args.get("cmd", "")}), ensure_ascii=False)
    if nome == "robo_ler":
        r = chamar("GET", f"/log?desde={int(args.get('desde', 0))}&espera={float(args.get('espera_s', 0))}")
        return r.get("erro") or f"{r['texto']}\n[proximo={r['proximo']} conectado={r['conectado']}]"
    return f"ferramenta desconhecida: {nome}"


def responder(id_: object, resultado: dict) -> None:
    sys.stdout.write(json.dumps({"jsonrpc": "2.0", "id": id_, "result": resultado}) + "\n")
    sys.stdout.flush()


def main() -> None:
    for linha in sys.stdin:
        if not linha.strip():
            continue
        msg = json.loads(linha)
        metodo, id_ = msg.get("method"), msg.get("id")
        if metodo == "initialize":
            responder(id_, {"protocolVersion": "2024-11-05", "capabilities": {"tools": {}}, "serverInfo": {"name": "robo", "version": "1.0"}})
        elif metodo == "tools/list":
            responder(id_, {"tools": FERRAMENTAS})
        elif metodo == "tools/call":
            p = msg["params"]
            responder(id_, {"content": [{"type": "text", "text": ferramenta(p["name"], p.get("arguments", {}))}]})
        elif id_ is not None:
            sys.stdout.write(json.dumps({"jsonrpc": "2.0", "id": id_, "error": {"code": -32601, "message": f"metodo desconhecido: {metodo}"}}) + "\n")
            sys.stdout.flush()


if __name__ == "__main__":
    main()
