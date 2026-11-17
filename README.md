# Robo Challenge 2026

Robo autonomo da RCB Challenge (regras: `Regras_Challenge_2026_Portugues-V1.0.pdf`). Limite 300 x 300 mm, totalmente autonomo.

## Onde esta o codigo da competicao

```
robo/
  platformio.ini            gravar: pio run -e robo -t upload   |   testar pecas: pio run -e bancada -t upload
  src/main.cpp              A MISSAO, lida de cima para baixo (comece por aqui)
  lib/RoboLib/src/          a biblioteca: funcoes simples, uma por assunto
    config.h                pinos e medidas (o unico lugar para ajustar numeros)
    rodas.h                 andarMm, girarGraus, rodasVelocidade
    sensores.h              linhaLer, distanciaMm, corChao, corFrente
    braco.h                 bracoSubir/Pegar/Plataforma, garraAbrir/Fechar
    linha.h                 seguirAteParar, centrarNoCruzamento, virar
    cores.h                 tabela de cores (calibrar aqui)
  lib/SCServo/              driver dos servos STS3215 (de terceiros, nao mexer)
extras/                     tudo que nao e codigo da competicao (CAD, simulador, versoes antigas)
docs/                       decisoes, estado e checklists
```

Para mexer: ajuste numeros em `config.h`, a rota e as distancias no topo de `main.cpp`, ou escreva uma funcao nova na biblioteca.
A missao nao foi testada na pista: use `-e bancada` para validar cada peca antes.

## extras/
- `cad/` modelos e pecas (OpenSCAD, DXF, STL); `old/` versoes de CAD descartadas
- `sim/` simulador Python da pista
- `ferramentas/` scripts de console BLE, ponte, gravacao
- `legado/robo_antigo/` firmware anterior (mapa + localizador, 1700 linhas em `logica/robo.h`), mantido so para consulta

O plotter de desenho e outro projeto (`jhlr/plotter`).
