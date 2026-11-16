#!/bin/bash
# uso: tempo20.sh "params" [N=20]: tempo medio para entregar 6 cubos (720 s se nao completou) e entregas, em N sementes
cd "$(dirname "$0")"; p="$1"; N=${2:-20}
seq 0 $((N-1)) | xargs -P 6 -I{} sh -c "SIM_SO={} timeout 250 ./simM $p --coleta 6 \$(({}+1)) 2>&1 | grep ^semente" | awk -v p="$p" '{ if ($0 ~ /COMPLETA/) { match($0,/ultima em [0-9]+/); t=substr($0,RSTART+10,RLENGTH-10)+0 } else t=720; s+=t; match($0,/entregou [0-9]+/); e+=substr($0,RSTART+9,RLENGTH-9); n++ } END{printf "%s => tempo medio %.0f s para 6 cubos | %d entregas em %d sementes\n", p, s/n, e, n}'
