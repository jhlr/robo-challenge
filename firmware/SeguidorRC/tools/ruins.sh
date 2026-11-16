#!/bin/bash
# uso: ruins.sh "params" [alvo=5] [limite=2] [max=60]
# Varre sementes em ordem; guarda as RUINS (entregou <= limite de 6) ate juntar `alvo`; descarta as boas.
# Escreve a lista em ruins.txt (uma semente por linha) e imprime a entrega de cada ruim.
cd "$(dirname "$0")"
params="$1"; alvo=${2:-5}; lim=${3:-2}; max=${4:-60}
: > ruins.txt
seq 0 $((max-1)) | xargs -P 6 -I{} sh -c 'o=$(SIM_SO={} timeout 200 ./simM '"$params"' --coleta 6 $(({}+1)) 2>&1 | grep -E "^semente"); n=$(echo "$o" | sed "s/.*entregou \([0-9]*\) de.*/\1/"); echo "{} $n"' | sort -n | awk -v l=$lim '$2<=l' | head -n $alvo | tee ruins.txt
