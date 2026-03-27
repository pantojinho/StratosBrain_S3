# Consolidacao da Base de Referencia

**Data:** 2026-03-27  
**Escopo:** usar `C:\Esp32\ESP32-S3-High-Precision-Avionics---Advanced-Meteorological-HAT--GPS---LoRa` apenas como referencia para o projeto atual em `C:\Esp32\CLAUDE`

## O que vale aproveitar

- Arquitetura logica de estado central:
  - `FlightState`
  - `MeteoState`
  - `GnssState`
  - `LoraState`
  - `PowerState`
  - `HealthState`
  - `ConnectivityState`
  - `UiState`
- Separacao por servicos:
  - voo
  - meteo
  - GNSS
  - LoRa
  - energia
  - saude do sistema
- Ideia de barra de status compacta no topo:
  - tempo de operacao
  - links
  - bateria
- Ideia de usar subpaginas, nao modos demais no nivel principal:
  - o `mini_os` tem `Tracker`, `GPS` e `Black Box`
  - no projeto atual, isso faz mais sentido como subareas de `COMMS` ou `PLANE`, nao como menus principais
- Disciplina de documentacao do repo de referencia:
  - memoria do projeto
  - status de agentes
  - fases de implementacao

## O que NAO vale migrar

- O firmware `src/` inteiro como base principal.
  - Ele nao corresponde ao comportamento que esta sendo testado hoje na placa.
  - A UI dele e outra.
- O menu principal do `mini_os` como esta hoje:
  - `Home`
  - `Avionics`
  - `Meteo`
  - `Tracker`
  - `GPS`
  - `Black Box`
  - `Config`
  - Isso conflita com a direcao atual:
    - `PLANE`
    - `METEO`
    - `COMMS`
    - `CONFIG`
- Os placeholders de conectividade.
  - O repo de referencia tem estados e switches visuais de Wi-Fi/BLE/Web.
  - Nao tem implementacao real do servidor web que resolva o problema atual.
- A trilha PlatformIO como caminho principal.
  - O `build_log.txt` mostra incompatibilidade de toolchain/lib (`esp32-hal-periman.h`).
  - Isso hoje adiciona ruido, nao estabilidade.
- O sketch `examples/avionics_horizon` como base do produto.
  - Ele serve como referencia de horizonte artificial.
  - Nao serve como base do sistema completo com menus, SD, touch, configuracao e operacao.

## Riscos de manter duas bases ativas

- Divergencia de menus e nomes de modos.
- Duplicacao de drivers e logica de sensores.
- Duas arquiteturas de UI concorrendo ao mesmo tempo.
- GitHub ficar com uma base que nao bate com o firmware que voce realmente testa.
- Perda de tempo corrigindo bugs no firmware errado.
- Conflito de stack:
  - Arduino IDE numa base
  - PlatformIO noutra
  - bibliotecas e compatibilidade diferentes

## Recomendacao objetiva

- Tratar `StratosBrain_LVGL9/StratosBrain_LVGL9.ino` como a base ativa real do produto.
- Usar o repo de referencia apenas para puxar:
  - modelo de estados
  - separacao de servicos
  - organizacao documental
- Nao migrar UI, menus nem stack de build do repo de referencia para o firmware atual.
- Quando for subir para GitHub, consolidar primeiro uma unica verdade:
  - firmware ativo
  - docs do produto
  - backlog tecnico

## Proximo passo sugerido

- Se for consolidar no GitHub depois, o melhor caminho e:
  1. promover a base `StratosBrain_LVGL9.ino` para firmware oficial
  2. absorver apenas a arquitetura de estados/servicos do repo de referencia
  3. arquivar a outra base como `legacy/reference`, nao como firmware principal
