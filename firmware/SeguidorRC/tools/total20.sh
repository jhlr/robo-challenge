#!/bin/bash
# uso: total20.sh "params" [N=20]: soma das entregas em N sementes (6 cubos)
cd "$(dirname "$0")"; p="$1"; N=${2:-20}
n=$(seq 0 $((N-1)) | xargs -P 6 -I{} sh -c "SIM_SO={} timeout 250 ./simM $p --coleta 6 \$(({}+1)) 2>&1 | grep ^semente | sed 's/.*entregou \([0-9]*\) de.*/\1/'" | awk '{s+=$1} END{print s}')
echo "$p => $n entregas em $N sementes"
