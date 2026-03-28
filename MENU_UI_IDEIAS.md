# StratosBrain S3 - Menu e UI

## Modos principais

- `PLANE`
- `METEO`
- `COMMS`
- `CONFIG`

## Direcao de UX

- telas simples
- leitura rapida
- botoes grandes
- hierarquia rasa
- `portrait-first`
- web como extensao de bancada e configuracao

## 1. PLANE

Objetivo:

- painel de voo leve
- altitude
- vertical speed
- heading
- GPS speed

Direcao visual:

- menos animacao pesada
- mais instrumentos e numeros grandes
- pronto para evoluir para `EFIS`, `NAV` e `SKYDIVE`

## 2. METEO

Objetivo:

- estacao meteorologica local
- temperatura
- umidade
- pressao
- altitude estimada
- gas bruto do `BME688`

Direcao visual:

- clima principal no topo
- cards grandes
- tema dinamico:
  - `ABERTO`
  - `NUBLADO`
  - `CHUVA PROVAVEL`

Observacao:

- a classificacao de gases/odores ainda e futura via `BSEC2`

## 3. COMMS

Objetivo:

- concentrar:
  - `Wi-Fi`
  - `LAN`
  - `LoRa`
  - `GPS`

Direcao visual:

- cards de status
- botoes de acao
- acesso rapido ao menu `LoRa`

## 4. CONFIG

Objetivo:

- diagnostico
- status de sensores
- logger SD
- brilho
- orientacao

Direcao visual:

- rolagem simples
- sem informacao espremida
- sempre com navegacao clara de volta

## Web

A web hoje deve servir para:

- validar hardware
- ver dados ao vivo
- testar `GPS`
- testar `LoRa`
- ver `SD / blackbox`
- entrar na rede local

Nao deve parecer uma pagina de debug solta.
Deve parecer um painel de bancada limpo e util.

## Estado atual

- `METEO` e web ja estao com cara melhor
- `LoRa` web virou console de payload
- `GPS` web ja tem area dedicada
- a UX do `PLANE` ainda precisa da proxima rodada forte
