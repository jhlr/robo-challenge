# Navegacao do simulador no carro real (2026-11-04)

A logica que o simulador testa (seguir a rua, cruzamentos por odometro, giro fechado na linha, localizador, mapa de cubos, coleta, entrega) deixou de existir so em
`SeguidorRC/test/simulador.cpp`. Agora ha um dono unico:

- `SeguidorRC/src/seguidor/robo.h`: `Navegador` e `Robo<Carro>` (template sobre o carro). O simulador e o carro real usam este mesmo codigo.
- `SeguidorRC/src/seguidor/tabuleiro.h`: geometria do tabuleiro e das plataformas (antes duplicada no simulador).
- `firmware/src/robo/carro_real.h`: o `Carro` de verdade (rodas STS por `hal::StsDrive`, barra RC, sonar, dois TCS34725). `firmware/src/robo/main.cpp` + `[env:robo]`.

Verificado no simulador depois da extracao: saida dos 125 cenarios identica, lote sem ruido 28/30, coleta de 18 cubos 14 a 15 entregues em 12 min (nao completa).
Sem MPU (`rumoPeloEncoder`), a logica nao usa pickup, colisao, ancora nem tracao (precisam do acelerometro); o resultado do simulador nao mudou com isso.

## O que roda no carro e o que nao
- Anda, cruza, gira na linha, localiza, mapeia e decide como no simulador. Nao anda ao ligar: `g [s]` no console (padrao 60 s e para sozinho), `x` para e reinicia, `s` estado.
- NAO aciona garra nem ombro: `coletar` so avisa (falta calibrar angulos). A logica segue como se tivesse pegado.
- Cores: usam `core/color_table.h` (tabela medida por outro carro/iluminacao); sem recalibrar, cubo e piso podem ser lidos errado.
- PWM do firmware -> comando do STS: linear, 160 PWM = 100% (`kPwmPlenoSts`). O simulador supoe 254 mm/s na reta; o STS entrega ~150 mm/s, entao os tempos do simulador sao otimistas.

## Antes de deixar andar
1. Medir a bitola EFETIVA com `pio run -e bitola` e gravar em `params::kTrackMm` (hoje 100 mm, valor antigo): sem isso o rumo erra e os giros em rua falham.
2. Conferir o sentido das rodas (`kLeftSign`/`kRightSign`) e a barra sobre a pista (limiar RC provisorio).
3. Primeiro teste com o carro levantado ou em pista curta, `g 10`.
