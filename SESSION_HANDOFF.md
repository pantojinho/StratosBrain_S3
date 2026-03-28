# StratosBrain S3 - Session Handoff

**Date:** 2026-03-27  
**Root oficial:** `C:\Esp32\CLAUDE\StratosBrain_S3`

## Firmware ativo

- `StratosBrain_LVGL9/StratosBrain_LVGL9.ino`

## Escopo atual

Projeto consolidado em um unico firmware para:

- `PLANE`
- `METEO`
- `COMMS`
- `CONFIG`
- `Wi-Fi AP`
- `LAN/STA`
- web dashboard
- `GPS`
- `LoRa UART`
- `microSD`

## Hardware em uso

| Modulo | Interface | Pinos |
|---|---|---|
| FT3168 | I2C | `47/48`, addr `0x38` |
| QMI8658 | I2C | `47/48`, addr `0x6B/0x6A` |
| BME688 | I2C | `47/48`, addr `0x76/0x77` |
| BMP581 | I2C | `47/48`, addr `0x46/0x47` |
| BMM350 | I2C | `47/48`, addr `0x14` |
| GPS AT6558R | UART | `RX=43`, `TX=44` |
| LoRa E220/E32 | UART | `TX=17`, `RX=18`, `AUX=6`, `M0=7`, `M1=8` |
| microSD | SPI | `CS=38`, `MOSI=39`, `MISO=40`, `SCK=41` |

Itens removidos do projeto:

- `LTR390UV`
- `MAX17048`

## O que esta funcionando

- display e touch estaveis
- menu principal e navegacao base
- `Wi-Fi AP` no firmware principal
- `LAN/STA` no mesmo sketch
- dashboard web com abas de:
  - `Lab`
  - `GPS`
  - `SD`
  - `Meteo`
  - `Rede`
  - `LoRa`
  - `Config`
  - `Overview`
- `BME688` lendo:
  - temperatura
  - umidade
  - pressao
  - altitude estimada
  - resistencia de gas
- `METEO` com classificacao local:
  - `ABERTO`
  - `NUBLADO`
  - `CHUVA PROVAVEL`
- `GPS` com parser basico de NMEA
- `GPS GP10` revisado contra a documentacao oficial:
  - `9600 8N1` por padrao
  - `1Hz` por padrao
  - comandos `PCAS` confirmados para `baud`, `update rate`, `restart` e constelacao
  - breakout `DX-PJ17` alimentado em `5V`
- aba `GPS` da web com diagnostico melhor de ligacao e controles de bancada:
  - `Cold Start`
  - `Hot Start`
  - `1Hz`
  - `5Hz`
  - `All GNSS`
  - `GPS+BDS`
  - `All NMEA ON`
- `LoRa UART` com console web de payload
- logger `CSV` em `microSD`
- serial monitor com diagnostico continuo de `BME688`, `GPS` e `LoRa`
- `BOOT` como `Home/Back`

## Limitacoes atuais

- `PLANE` ainda esta mais forte como painel leve do que como cockpit final
- `BMP581` e `BMM350` ainda nao estao entregando todos os dados finais na UI
- `GPS` depende de fix real e visao aberta do ceu
- o `LoRa` atual mostra payload/UART, nao `SDR`
- classificacao avancada de gases do `BME688` ainda nao foi integrada

## Proximos passos recomendados

1. fechar a leitura real de `BMP581` no `PLANE`
2. integrar `BMM350` para heading
3. melhorar a tela e a web de `GPS` com mais estatisticas
4. definir estrategia de gateway para `LoRa`
5. avaliar `BSEC2` para classificacao de gases no `BME688`

## Limpeza do repositorio

Repositorio enxugado para ficar so com:

- firmware principal
- documentacao ativa
- licenca

Diretorios de build, referencias e material legado podem ser removidos sem afetar o firmware oficial.
