#!/usr/bin/env python3
# uso: entrada.py "params" [N=20]: por semente, quando a linha ficou segura (massaLinha>=0.9) e se a pose estava certa nesse instante
import subprocess, sys, re, os
from concurrent.futures import ThreadPoolExecutor
params = sys.argv[1].split() if len(sys.argv) > 1 and sys.argv[1] else []
n = int(sys.argv[2]) if len(sys.argv) > 2 else 20
def run(k):
    env = dict(os.environ, SIM_SO=str(k), SIM_DEBUG="1")
    try:
        out = subprocess.run(["./simM"] + params + ["--coleta", "6", str(k + 1)], env=env, capture_output=True, text=True, timeout=30).stdout
    except subprocess.TimeoutExpired as e:
        out = (e.stderr or b"").decode() if isinstance(e.stderr, bytes) else (e.stderr or "")
    return k, out
def run2(k):
    env = dict(os.environ, SIM_SO=str(k), SIM_DEBUG="1")
    p = subprocess.Popen(["./simM"] + params + ["--coleta", "6", str(k + 1)], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    t = None; linhas = []
    try:
        for l in p.stderr:
            m = re.search(r"BELIEF t=([\d.]+) pose .* massaLinha ([\d.]+)", l)
            if m and t is None and float(m.group(2)) >= 0.9: t = float(m.group(1))
            m = re.search(r"POSE t=(\d+)s .*", l)
            if m and t is not None and int(m.group(1)) >= t:
                linhas.append("ERRADA" if "ERRADA" in l else "ok"); break
            if m and int(m.group(1)) > 150: break
    finally:
        p.kill()
    return k, t, (linhas[0] if linhas else "-")
with ThreadPoolExecutor(8) as ex: res = sorted(ex.map(run2, range(n)))
ts = [t for _, t, _ in res if t is not None]
for k, t, p in res: print(k, t if t is not None else "nunca", p)
print(f"media {sum(ts)/max(1,len(ts)):.1f} s | linha segura em {len(ts)} | nunca {n-len(ts)} | pose errada {sum(1 for _,t,p in res if p=='ERRADA')}")
