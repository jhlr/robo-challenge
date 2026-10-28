# SeguidorRC

Seguidor de linha preta **sem PID** para ESP32-S3 + ponte H TB6612FNG + barra de 8 sensores RC. Placa na Arduino IDE: "ESP32S3 Dev Module". Monitor Serial a 115200.

## Tres pecas independentes
| Arquivo | Faz | Usa pinos? |
|---|---|---|
| `sensores_rc.h` | le os 8 sensores de uma vez e calibra (`iniciar`, `calibrar`, `ler`) | sim |
| `decisao.h` | escolhe a velocidade das rodas (`decidir`) | **nao**, nem `millis()`: testa no PC |
| `ponte_h.h` | manda a velocidade para a ponte (`iniciar`, `mover`, `parar`, `aplicar`) | sim |

Sem classes: so funcoes e `struct` de dados (`BarraRC`, `PinosPonteH`, `Ajustes`, `Memoria`, `Comando`). O sketch liga uma peca na outra; veja `examples/SeguidorBasico`.
Voce pode usar so a decisao com outros sensores, ou so a ponte H em outro robo.

```cpp
ler(barra);
Comando c = decidir(barra.preto, millis(), ajustes, memoria);
aplicar(ponte, c);
```

## Usar
- Arduino IDE: copie (ou faca link de) a pasta `SeguidorRC` para `~/Documents/Arduino/libraries/`.
- arduino-cli: `arduino-cli compile --fqbn esp32:esp32:esp32s3 --library SeguidorRC SeguidorRC/examples/SeguidorBasico`
- Teste da decisao no PC: `g++ -std=c++17 -Isrc test/teste_decisao.cpp src/seguidor/decisao.cpp -o /tmp/teste && /tmp/teste`

## Ajustar na pista
Tudo em `Ajustes` (valores de partida em `decisao.h`): `velReto`, `velGiro`, `confirmaPonta`, `lacunaMs`, `buscaGiroMs`, `buscaEspiralMs`, `velBuscaExterna`, `velBuscaInternaFim`. Os tempos e velocidades sao chutes razoaveis, **nao testados em hardware** (so compila para ESP32-S3, core 3.3.12). Com `kModoTeste = true` no exemplo os motores ficam desligados e so o Monitor Serial mostra a decisao.

## Linha perdida
Segue o ultimo comando por `lacunaMs` (falha na fita); depois gira no proprio eixo para o ultimo lado, abre em espiral (roda de fora constante, roda de dentro acelerando) e **repete ate achar a linha**. Se nunca viu a linha, comeca pela direita.

## O que estava fazendo o robo rodopiar (codigo original do colega)
1. Linha perdida = giro no lugar para sempre, a PWM 100, sem limite. Agora ha lacuna, giro e espiral controlados.
2. Ponta (S1/S8) dava giro com uma unica leitura: agora precisa de 2 seguidas.
3. Calibracao ruim passava batida: com menos de 6 sensores validos ela e recusada e repetida.
4. Os 8 pinos sao lidos de uma vez no registrador `GPIO_IN_REG` e a leitura espera so ate o maior limiar da calibracao.
