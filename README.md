# StratosBrain S3

Wearable cockpit, meteo station and field telemetry OS for the Waveshare `ESP32-S3 Touch AMOLED 1.64`.

## Root oficial

- `C:\Esp32\CLAUDE\StratosBrain_S3`

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
- `ESQUEMA_LIGACOES.md`
  guia de ligacao dos modulos usados
- `ARQUITETURA_TECNICA.md`
  arquitetura atual do software
- `MENU_UI_IDEIAS.md`
  direcao de UX e telas
- `SESSION_HANDOFF.md`
  estado tecnico e proximos passos
- `LICENSE`
  licenca MIT

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

## Licenca

MIT. Veja [LICENSE](LICENSE).
