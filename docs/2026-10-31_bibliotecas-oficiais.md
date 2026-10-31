# 2026-10-31 — Componentes em bibliotecas oficiais

Regra: nada de reinventar a roda. Cada periférico usa a biblioteca do fabricante/comunidade; `hal/` e `sensores_rc` só mapeiam pinos e decidem.

| Componente | Biblioteca | Antes |
|---|---|---|
| QRE-8D (RC) | pololu/QTRSensors 4.0.0 | leitura por registrador de GPIO feita à mão |
| HC-SR04 | teckel12/NewPing 1.9.7 | `pulseIn` direto |
| PCA9548 | robtillaart/TCA9548 0.3.2 | `Wire.write(1<<canal)` + cache próprio |
| Servo de garra | madhephaestus/ESP32Servo | LEDC à mão |
| STS3215 | SCServo oficial (Waveshare), já vendored | — |
| TCS34725 / MPU6050 | Adafruit, já em uso | — |

Notas:
- GPIO34-39 não têm saída: o modo RC precisa carregar o pino, então S7/S8 em 34/35 continuam mortos. A QTRSensors só recebe os pinos que carregam; os mortos ficam de fora e a calibração os marca.
- A QTRSensors desliga interrupções durante a leitura (até `tempoLeituraUs`); verificar se o BLE segura no carrinho.
- Compilam: rapido, seguidor, bringup, follow. Nada testado no hardware ainda.
- Fora do escopo (sem biblioteca oficial equivalente): console/OTA por BLE, PID (`core/pid.h`), `hal::LineArray` (digital, usado só em bringup/follow).
