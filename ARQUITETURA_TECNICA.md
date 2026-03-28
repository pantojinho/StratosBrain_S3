# StratosBrain S3 - Arquitetura Tecnica

## Principio central

O projeto agora trabalha com **um unico firmware**.

Nao ha mais separacao entre:

- sketch principal
- sketch web

A estrategia atual e:

- um so runtime
- `Wi-Fi AP` integrado
- `LAN/STA` opcional
- modo leve quando necessario

## Camadas do sistema

### 1. Hardware

- AMOLED `CO5300`
- touch `FT3168`
- IMU `QMI8658`
- `BME688`
- `BMP581`
- `BMM350`
- `GPS AT6558R`
- `LoRa UART`
- `microSD`

### 2. Drivers e leitura

- `I2C` para touch e sensores Bosch
- `UART` para `GPS`
- `UART` para `LoRa`
- `SPI` para `microSD`

### 3. Estado em memoria

Estados principais mantidos no firmware:

- `ImuState`
- `Bme688State`
- `GpsState`
- `LoraState`
- `BlackboxState`

## Telas locais

- `HOME`
- `PLANE`
- `METEO`
- `COMMS`
- `LORA`
- `CONFIG`

## Web

O dashboard web espelha o estado da bancada:

- `Lab`
- `GPS`
- `SD`
- `Meteo`
- `Rede`
- `LoRa`
- `Config`
- `Overview`

## Fluxo de dados

### Meteo

`BME688 -> poll -> state -> UI local + web + CSV`

### GPS

`UART1 -> parser NMEA -> state -> UI local + web + CSV`

### LoRa

`UART2 -> payload RX/TX -> state -> web console + serial`

### Blackbox

`states -> CSV no SD -> preview web`

## Regras atuais

- `portrait-first`
- sem rotacao agressiva
- atualizacao web suave
- cards grandes
- diagnostico visivel no serial

## Fora do escopo atual

- `LTR390UV`
- `MAX17048`
- `LoRa SDR`
- classificacao avancada de gases ainda nao finalizada

## Proxima arquitetura a fechar

1. `BMP581` como altimetro/vario principal
2. `BMM350` para heading
3. `GPS` com mais estatisticas
4. gateway para `LoRa`
5. `BSEC2` para gases no `BME688`
