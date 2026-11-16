# Sweep de gambiarra e consolidacao: flags de A/B

## O que era
O simulador tinha 14 flags (`gPortaoMeio`, `gSemRepetir`, `gDireto`, etc.) que ligavam caminhos alternativos dentro do firmware (`robo.h`). Todas com default `true`. No carro real o `.ino` nao muda nenhuma, entao o ramo OFF de cada uma nunca roda. Eram experimentos de A/B que ficaram no codigo, com duas fontes de verdade (default no simulador e default no struct).

## O que foi feito
- Removidos 12 flags de default `true` e seus ramos OFF: `primeiroContatoNaoENo`, `tentarLados`, `portaoAteAzulAcabar`, `zerarSaindoNoFim`, `ancorarSemVarrer`, `corNaSaidaParaTudo`, `sinteticoLimpaPular`, `irDiretoParaPlataforma`, `portaoSoMeio`, `portaoSoMeio2`, `ancorarSemRepetir`, `olharSoSeNaoMedida`. Fica so o ramo ON, que era o comportamento padrao.
- Removidos os globais `gX` correspondentes e as opcoes de linha de comando do simulador (`olhar=`, `portaomeio=`, `norepete=`, etc.).
- Mantidos `observarNoInicio` (`observar=`) e `autoCalBitola` (`autocal=`), default `false`. O cenario de observacao (`simulador.cpp`) ainda usa o primeiro. Decidir depois se removem.
- Removida a variavel `algumPreto` que ficou sem uso.

## Verificacao
- 20 sementes (6 cubos): saida do simulador identica byte a byte ao HEAD (19/20 completas, igual ao antes).
- 8 programas de teste (`teste_*`): compilam com `-Wall` e passam.
- Nenhuma referencia aos flags removidos sobrou em `firmware/` ou `examples/`.

## Ainda nao feito (proximo passo)
Numeros magicos que sao remendos de comportamento, sem dono nomeado (apenas listados, nao mudados):
- `robo.h`: gate de portao com 1300 ms, 130 ms e 60 ms em linhas separadas; `minAndado` com 150 mm e 60 mm.
- Giro lateral de busca com passos fixos `-55` e `110` mm.
- `250 mm` e `15 mm` no corte de cor na saida da rua.
- Ganho e limites do `autoCalBitola` (0.7/1.4, 0.5, 80 mm).

Essas constantes precisam de um dono (um lugar so, com nome) antes de virar consolidacao de fato. Mudar valores muda o resultado, entao vai junto com a validacao de sementes.
