#!/bin/bash
# uso: avalia.sh "params" : roda so as sementes ruins de ruins.txt e soma as entregas
cd "$(dirname "$0")"; t=0; l=""
for k in $(cut -d' ' -f1 ruins.txt); do n=$(SIM_SO=$k timeout 300 ./simM $1 --coleta 6 $((k+1)) 2>&1 | grep ^semente | sed 's/.*entregou \([0-9]*\) de.*/\1/'); t=$((t+n)); l="$l $k:$n"; done
echo "$1 => $t |$l"
