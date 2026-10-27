# Chassi superior alargado (2026-10-21)

> Continua valendo para o historico do DXF; o resumo atual esta em `2026-10-24_braco-opcao2-estado-atual.md`.

Script: `cad/chassi/alargar_superior.py` (ezdxf + shapely; venv em `.scratch/venv`). Entrada: `~/Downloads/Esboço30.dxf` (referência) e `~/Downloads/chassi_superior.dxf`. Saída: `~/Downloads/chassi_superior_alargado.dxf` (cópia em `cad/chassi/`), conferência visual em `cad/chassi/compare_after.png`.

1. Largura do superior 94,01 -> 99,01 mm (igual ao Esboço30); altura 156,02 já era igual.
2. Alargamento: o que está à direita do centro anda +5,00 mm em x (entalhes laterais e furos da direita mantêm a distância até a borda direita); o rasgo central 25 x 12 mm mantém o tamanho e anda 2,5 mm.
3. Alinhamento: contorno coincide com o do Esboço30 (cantos); os 8 furos de poste (3,21 mm) foram levados ao centro dos 8 furos do Esboço30. Moveram -1,65 mm em y (padrão 38 / 60,74 / 38 estava deslocado) e, na coluna da direita, -0,80 mm em x (o Esboço30 tem furos a 10,4 mm da borda direita e 9,6 da esquerda).
4. Conferido por código: contorno válido, nenhum furo ou rasgo a menos de 1,5 mm da borda, entalhes laterais cobrem os ovais de 10 x 20 do Esboço30.
5. Não mexi: furos de 3,01 mm de componentes (3 unidades) e o furo duplicado em (-412,15, -304,70) do arquivo original, que tem duas entidades quase iguais (uma delas com canto quadrado). Remover uma antes de cortar a laser, senão corta duas vezes.

## Asas de 40 mm por lado (2026-10-22)

Pedido: +8 cm de largura. Regra: 300 x 300 mm no maximo, medido com tudo estendido; a largura total do robo ja e 182 mm (rodas externas a |x| = 91), entao as asas (superior 99 -> 179 mm, borda a |x| = 89,5) nao aumentam a largura total. Comprimento total do robo: 288 mm (so ~12 mm de sobra).
- `cad/chassi/alargar_superior.py` (constante `WING`) gera `chassi_superior_asas.dxf` (copia em `~/Desktop/corte_mdf/`): nas 3 faixas sem entalhe de cada lado o contorno cresce 40 mm para fora; os 4 entalhes viram abertos ate a nova borda; furos, rasgo central e postes nao se mexem.
- Modelo: `wing = 40` em `cad/modelo_robo/robot_completo.scad` (so o convés de cima). Convés de cima fica a z = 68, topo das rodas a 66: 2 mm de folga sobre as rodas (pneu e tolerancia). O modelo mostra a asa como caixa inteira, sem os entalhes.
- Nao verificado: rigidez das asas (MDF 2,3 mm, 40 mm em balanco, so presas pelos postes).

## Janelas fechadas nos motores (2026-10-24)

Os 4 entalhes laterais dos motores (22,7 x 23 mm, antes abertos até a borda nova) agora são **janelas fechadas**: a asa de 40 mm cobre a faixa entre a borda antiga e a nova, e o entalhe fica como furo interno (`alargar_superior.py`, `fills`). `chassi_superior_asas.dxf` tem 19 entidades (contorno + 4 janelas + furos), conferido por shapely. Copiado para `~/Desktop/corte_mdf/`.

## Porca espaçadora: mantida (2026-10-24)

Os parafusos dos postes ficam 10 mm abaixo do topo das rodas e o convés precisa de 2 mm de folga sobre elas, então o espaçador (~12 mm) continua. Testei um convés baixo com a largura extra por fora das rodas (largura total 260 mm, pontes no vão entre as rodas) e foi descartado pelo usuário; arquivos em `arquivo/chassi_baixo_descartado/`.
