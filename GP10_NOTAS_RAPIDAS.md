# GP10 Notas Rapidas

Resumo pratico do material em:

- `C:\Esp32\CLAUDE\GP10 Development & User Information-20260328T000923Z-1-001`

Arquivos mais uteis encontrados:

- `03-GP10 Technical Documentation/DX-GP10-GPS Module Technical manual.pdf`
- `04-GP10 MODULE USER GUIDE/DX-GP10-GPSSerial Port application guide.pdf`
- `04-GP10 MODULE USER GUIDE/DX-GP10-NMEA0813 protocol specification.pdf`

## O que esses documentos confirmam

- O `GP10` modulo nu trabalha em `2.7V ~ 3.4V`, recomendado `3.3V`
- Interface principal: `UART`
- Baud padrao: `9600 8N1`
- Baud maximo configuravel: `115200`
- Taxa de atualizacao padrao: `1Hz`
- O guia serial mostra comandos para `2Hz`, `4Hz`, `5Hz` e ate `10Hz`
- Frases NMEA suportadas incluem `GGA`, `GLL`, `GSA`, `GSV`, `RMC`, `VTG`, `ZDA`, `TXT`, `GST`
- O pino `WAKE_UP` fica em alto por padrao
- O pino `1PPS` gera `1 pulso por segundo` depois do posicionamento

## Ligacao recomendada no projeto atual

Firmware oficial:

- `C:\Esp32\CLAUDE\StratosBrain_S3\StratosBrain_LVGL9\StratosBrain_LVGL9.ino`

Pinagem usada no firmware:

- `GPS RX do ESP = GPIO43`
- `GPS TX do ESP = GPIO44`
- `baud = 9600`

Ligacao correta:

- `TXD do GP10 -> GPIO43`
- `RXD do GP10 -> GPIO44`
- `GND -> GND`
- `VCC -> 3V3` para modulo nu
- `VCC -> 5V` apenas se for breakout com regulador que pede `5V`

## Comandos uteis do GP10

### Baud rate

- `9600`: `$PCAS01,1*1D`
- `115200`: `$PCAS01,5*19`

### Taxa de atualizacao

- `1Hz`: `$PCAS02,1000*2E`
- `2Hz`: `$PCAS02,500*1A`
- `4Hz`: `$PCAS02,250*18`
- `5Hz`: `$PCAS02,200*1D`
- `10Hz`: `$PCAS02,100*1E`

### Todas as frases NMEA

- tudo desligado: `$PCAS03,0,0,0,0,0,0,0,0,0,0,,,0,0*02`
- tudo ligado: `$PCAS03,1,1,1,1,1,1,1,1,1,1,,,1,1*02`

### Sistemas GNSS

- `GPS`: `$PCAS04,1*18`
- `BDS`: `$PCAS04,2*1B`
- `GPS+BDS`: `$PCAS04,3*1A`
- `GLONASS`: `$PCAS04,4*1D`
- `GPS+GLONASS`: `$PCAS04,5*1C`
- `BDS+GLONASS`: `$PCAS04,6*1F`
- `GPS+BDS+GLONASS`: `$PCAS04,7*1E`

### Reinicio do receptor

- `hot start`: `$PCAS10,0*1C`
- `warm start`: `$PCAS10,1*1D`
- `cold start`: `$PCAS10,2*1E`
- `factory start`: `$PCAS10,3*1F`

## Como isso ajuda no StratosBrain

- confirma que o firmware atual esta certo em `9600`
- confirma que o caminho de parse por `NMEA` faz sentido
- abre caminho para um menu web/local de configuracao do `GPS`
- permite reduzir o trafego NMEA no futuro ligando so as frases que interessam
- permite testar `cold start` e trocar `update rate` pelo proprio firmware

## Melhor uso imediato

Para a proxima rodada do GPS, o mais util eh:

1. manter `9600 8N1`
2. mostrar na web as ultimas frases `GGA` e `RMC`
3. adicionar botoes para:
   - `cold start`
   - `1Hz`
   - `5Hz`
   - `all NMEA on`
   - `NMEA essencial`
4. se continuar sem trafego, revisar a ligacao `TXD GP10 -> GPIO43`
