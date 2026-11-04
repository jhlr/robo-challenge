# Navegacao do simulador no carro real (2026-11-04)

A logica que o simulador testa (seguir a rua, cruzamentos por odometro, giro fechado na linha, localizador, mapa de cubos, coleta, entrega) deixou de existir so em
`SeguidorRC/test/simulador.cpp`. Agora ha um dono unico:

- `SeguidorRC/src/seguidor/robo.h`: `Navegador` e `Robo<Carro>` (template sobre o carro). O simulador e o carro real usam este mesmo codigo.
- `SeguidorRC/src/seguidor/tabuleiro.h`: geometria do tabuleiro e das plataformas (antes duplicada no simulador).
- `firmware/src/robo/carro_real.h`: o `Carro` de verdade (rodas STS por `hal::StsDrive`, barra RC, sonar, dois TCS34725). `firmware/src/robo/main.cpp` + `[env:robo]`.

Verificado no simulador depois da extracao (parametros ANTES de `--lote`/`--coleta`, senao sao ignorados): saida dos 125 cenarios identica no modo padrao e, no modo do carro STS
(`encoder=1 rumoenc=1 derrap=1.0 bitola=185 bitolaaj=185`), binario antigo e novo iguais: lote sem ruido 30/30 em 178 s por 2 entregas; coleta de 18 cubos em 12 min: 12 entregues (cap=1) e 13 (cap=2), nao completa.
Rodas de borracha (sem derrapagem): bitola efetiva = fisica = 185 mm; o resultado nao muda nem com a bitola suposta 8% errada (os giros fecham na linha).

Sem MPU (`rumoPeloEncoder`), a logica nao usa pickup, colisao, ancora nem tracao (precisam do acelerometro); o resultado do simulador nao mudou com isso.

## O que roda no carro e o que nao
- Anda, cruza, gira na linha, localiza, mapeia e decide como no simulador. Nao anda ao ligar: `g [s]` no console (padrao 60 s e para sozinho), `x` para e reinicia, `s` estado.
- NAO aciona garra nem ombro: `coletar` so avisa (falta calibrar angulos). A logica segue como se tivesse pegado.
- Cores: usam `core/color_table.h` (tabela medida por outro carro/iluminacao); sem recalibrar, cubo e piso podem ser lidos errado.
- PWM do firmware -> comando do STS: linear, 160 PWM = 100% (`kPwmPlenoSts`). O simulador supoe 254 mm/s na reta; o STS entrega ~150 mm/s, entao os tempos do simulador sao otimistas.

## Antes de deixar andar
1. `params::kTrackMm` = 185 mm (regua entre os centros; rodas de borracha, sem derrapagem). Conferir com o env `bitola` se um giro de 90 graus sai de 90.
2. Conferir o sentido das rodas (`kLeftSign`/`kRightSign`) e a barra sobre a pista (limiar RC provisorio).
3. Primeiro teste com o carro levantado ou em pista curta, `g 10`.
