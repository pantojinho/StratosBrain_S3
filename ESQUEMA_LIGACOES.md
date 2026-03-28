# StratosBrain S3 - Esquema de Ligacoes

Guia rapido para ligar sensores e modulos externos no projeto `StratosBrain S3`.

Raiz oficial:

- `C:\Esp32\CLAUDE\StratosBrain_S3`

---

## Regras importantes

- Toda a logica do `ESP32-S3` eh `3.3V`.
- Todos os modulos devem compartilhar o mesmo `GND`.
- Sensores `I2C` vao no barramento `GPIO47/GPIO48`.
- `GPS` usa `UART`.
- O `LoRa` escolhido agora tambem usa `UART`, nao `SPI`.
- O `microSD` serve para logs e caixa-preta, nao como RAM extra.

---

## Barramentos usados no projeto

### I2C principal

- `SDA = GPIO47`
- `SCL = GPIO48`

Uso:

- `BME688`
- `BMP581`
- `BMM350`

### UART do GPS

- `ESP RX = GPIO43`
- `ESP TX = GPIO44`

### UART do LoRa

- `ESP TX = GPIO17`
- `ESP RX = GPIO18`
- `AUX = GPIO6`
- `M0 = GPIO7`
- `M1 = GPIO8`

### SPI do cartao SD

- `CS = GPIO38`
- `MOSI = GPIO39`
- `MISO = GPIO40`
- `SCK = GPIO41`

---

## 1. BME688

Uso:

- temperatura
- umidade
- pressao
- gas
- altitude estimada pela pressao

Ligacao:

- `VIN/VCC -> 3V3`
- `GND -> GND`
- `SDA -> GPIO47`
- `SCL -> GPIO48`
- `CS -> 3V3` para forcar modo `I2C`
- `SDO -> GND` para endereco `0x76`
- ou `SDO -> 3V3` para endereco `0x77`

Endereco esperado:

- `0x76` ou `0x77`

Onde aparece no software:

- `CONFIG`
- `METEO`
- `Web > Meteo`

---

## 2. BMP581

Uso:

- altitude barometrica
- pressao
- vertical speed

Ligacao:

- `VIN/VCC -> 3V3`
- `GND -> GND`
- `SDA -> GPIO47`
- `SCL -> GPIO48`

Endereco esperado:

- `0x46` ou `0x47`

Onde aparece no software:

- `CONFIG`
- `PLANE`
- `METEO`

---

## 3. BMM350

Uso:

- bussola digital
- heading

Ligacao:

- `VIN/VCC -> 3V3`
- `GND -> GND`
- `SDA -> GPIO47`
- `SCL -> GPIO48`

Endereco esperado:

- `0x14`

Onde aparece no software:

- `CONFIG`
- `PLANE`

---

## 4. GPS AT6558R

Existem dois cenarios:

### GPS modulo nu

- `VCC -> 3V3`
- `GND -> GND`
- `TX do GPS -> GPIO43 (RX do ESP)`
- `RX do GPS -> GPIO44 (TX do ESP)`

### GPS breakout/adaptador 5V

Se for aquela placa adaptadora do anuncio com `VCC = 5V`:

- `VCC -> 5V`
- `GND -> GND`
- `TXD do GPS -> GPIO43 (TX do ESP)`
- `RXD do GPS -> GPIO44 (RX do ESP)`

Observacao:

- O mais importante no primeiro teste eh `TX do GPS -> RX do ESP`.
- Alguns modulos funcionam no primeiro teste mesmo sem ligar o `RX` do GPS.
- O `GP10` trabalha por padrao em `9600 8N1`.

Onde aparece no software:

- `COMMS`
- `PLANE`
- `Web > Comms`

---

## 5. LoRa UART E220 / E32 / similar

O modulo escolhido agora eh `UART`, com pinos:

- `M0`
- `M1`
- `RXD`
- `TXD`
- `AUX`
- `VCC`
- `GND`

Entao ele **nao usa SPI** e **nao usa SDA/SCL**.

### Ligacao final no StratosBrain S3

- `LoRa VCC -> 5V`
- `LoRa GND -> GND`
- `ESP GPIO17 (TX) -> RXD do LoRa`
- `ESP GPIO18 (RX) <- TXD do LoRa`
- `ESP GPIO6 <- AUX do LoRa`
- `ESP GPIO7 -> M0 do LoRa`
- `ESP GPIO8 -> M1 do LoRa`

### Modo inicial recomendado

- `M0 = LOW`
- `M1 = LOW`

Isso coloca o modulo em `modo normal`.

### Resumo rapido

| Sinal do modulo LoRa | Vai para |
|---|---|
| `VCC` | `5V` |
| `GND` | `GND` |
| `RXD` | `GPIO17` |
| `TXD` | `GPIO18` |
| `AUX` | `GPIO6` |
| `M0` | `GPIO7` |
| `M1` | `GPIO8` |

Onde aparece no software:

- `COMMS`
- `LORA`
- `Web > LoRa`

Observacao:

- O firmware ja foi ajustado para essa pinagem.
- A tela `LORA` ja mostra essa configuracao.
- O proximo passo eh implementar `TX/RX` real entre dois modulos.

---

## 6. microSD

Ligacao onboard:

- `CS = GPIO38`
- `MOSI = GPIO39`
- `MISO = GPIO40`
- `SCK = GPIO41`

Uso:

- logs
- caixa-preta

---

## Ligando varios sensores I2C juntos

Todos estes sensores podem compartilhar o mesmo barramento:

- `BME688`
- `BMP581`
- `BMM350`

Ligacao compartilhada:

- todos os `VCC -> 3V3`
- todos os `GND -> GND`
- todos os `SDA -> GPIO47`
- todos os `SCL -> GPIO48`

---

## Tabela rapida

| Modulo | Interface | Pinos na placa | Endereco |
|---|---|---|---|
| BME688 | I2C | `47/48` | `0x76` ou `0x77` |
| BMP581 | I2C | `47/48` | `0x46` ou `0x47` |
| BMM350 | I2C | `47/48` | `0x14` |
| GPS | UART | `43/44` | nao usa I2C |
| LoRa UART | UART | `17/18/6/7/8` | nao usa I2C |
| microSD | SPI | `38/39/40/41` | nao usa I2C |

---

## Ordem recomendada de testes

1. `BME688`
2. `GPS`
3. `LoRa UART`
4. `BMP581`
5. `BMM350`

---

## Estado atual do firmware

Ja preparado para:

- `BME688` com leitura basica
- `GPS` reservado em `43/44`
- `LoRa UART` reservado em `17/18/6/7/8`
- `LORA` menu dedicado

Ainda pendente:

- `TX/RX` real do `LoRa UART`
- `GPS` com leitura real
- `BMP581` e `BMM350` com driver real
