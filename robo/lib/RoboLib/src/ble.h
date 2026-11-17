#pragma once
// Bluetooth LE: console sem fio e atualizacao do firmware (OTA). O robo aparece como "RoboChallenge".
//   - comandos: o computador/celular manda uma letra pelo console (Nordic UART): 'g' = partir, 'x' = parar tudo
//   - gravar sem cabo:  ~/.venvs/seguidor/bin/python extras/ferramentas/ota_ble.py robo/.pio/build/robo/firmware.bin --nome RoboChallenge --segredo robo-challenge-2026
//     (a primeira gravacao com este firmware vai por USB; depois e por BLE. Feche robo_bridge.py/console_ble.py antes: so um aparelho conecta)
#include <Arduino.h>

void bleIniciar();       // chame como PRIMEIRA coisa do setup(): um firmware que chega ate aqui ainda aceita ser atualizado
void bleAtualizar();     // chame com frequencia (o laco da missao ja chama)
bool bleAtualizando();   // true enquanto chega um firmware novo: o robo tem de ficar parado
char bleComando();       // ultima letra recebida por BLE (0 se nenhuma); zera ao ler
Print& saida();          // escreve no monitor serial E no BLE: saida().printf("x=%d\n", x);
