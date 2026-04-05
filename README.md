# StratosBrain S3

Wearable cockpit, meteo station and field telemetry OS for the Waveshare `ESP32-S3 Touch AMOLED 1.64`.


## Firmware oficial

- `StratosBrain_LVGL9/StratosBrain_LVGL9.ino`

Este projeto agora trabalha com **um unico firmware** para:

- `PLANE`
- `METEO`
- `COMMS`
- `CONFIG`
- servidor web local via `Wi-Fi AP`
- acesso opcional via `LAN/STA`

## Hardware ativo no projeto

- `ESP32-S3R8`
- AMOLED `CO5300` `280x456`
- touch `FT3168`
- IMU `QMI8658`
- `BME688`
- `BMP581`
- `BMM350`
- `GPS AT6558R`
- `LoRa UART E220/E32`
- `microSD`

Itens removidos do escopo atual:

- `LTR390UV`
- `MAX17048`

## O que ja funciona

- display, touch e navegacao principal
- `Wi-Fi AP` no firmware principal
- `LAN/STA` no mesmo sketch
- web dashboard para `Meteo`, `GPS`, `SD`, `LoRa` e configuracao
- leitura basica do `BME688`
- logger `CSV` em `microSD`
- serial monitor com diagnostico de `BME688`, `GPS` e `LoRa`
- `BOOT` como `Home/Back`

## Estrutura do repositorio

- `StratosBrain_LVGL9/`
  firmware principal
- `README.md`
  visao geral do projeto
- `SESSION_HANDOFF.md`
  estado tecnico e proximos passos para agentes
- `WEB_STABILITY_NOTES_2026-04-02.md`
  notas tecnicas para humanos sobre a estabilidade do dashboard web

## Arduino IDE

Use:

- Board: `Waveshare ESP32-S3-Touch-AMOLED-1.64`
- USB CDC On Boot: `Enabled`
- Partition Scheme: `16M Flash (3MB APP/9.9MB FATFS)`
- PSRAM: `Enabled`
- Upload Mode: `UART0 / Hardware CDC`
- Upload Speed: `921600`

## Notas importantes

- O projeto foi consolidado para um unico sketch.
- O `LoRa` atual e **UART**, nao `SPI`.
- O console web do `LoRa` mostra trafego de payload/UART, nao espectro `SDR`.
- O `METEO` usa inferencia local simples para `ABERTO`, `NUBLADO` e `CHUVA PROVAVEL`.
- A classificacao avancada de gases do `BME688` ainda e futura via `BSEC2`.

## Estabilidade do dashboard web

Em 2026-04-02 foi tratada uma degradacao observada apos cerca de 15 minutos de operacao: o display local continuava recebendo GPS e meteo, mas o dashboard web passava a mostrar valores zerados ou deixava de atualizar.

Mitigacoes aplicadas no firmware:

- timeout HTTP do cliente aumentado para reduzir desconexoes prematuras
- atendimento do servidor web endurecido com `setTimeout()` e `setNoDelay()`
- reinicio do servidor com `stop()` antes de novo `begin()`
- `/api/status` dividido entre leitura leve e leitura detalhada
- dashboard V2 alterado para evitar requisicoes concorrentes e reduzir a frequencia das cargas pesadas
- senha do AP removida do JSON exposto pela API

Validacao recomendada:

- manter o dispositivo ligado por pelo menos `30 a 60 minutos`
- deixar o dashboard aberto durante todo o teste
- confirmar que `web_hits` continua subindo
- confirmar que GPS, umidade e demais leituras nao voltam para zero no navegador
- se possivel, repetir o teste em janela longa de `24 h`

## Licenca

MIT. Veja [LICENSE](LICENSE).
