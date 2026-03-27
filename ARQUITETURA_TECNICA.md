# StratosBrain S3 - Arquitetura Tecnica

**Date:** 2026-03-27  
**Scope:** proposta pratica para menus definitivos, estrategia de firmware WiFi/Web e backend de sensores preservando RAM

---

## Objetivo

Fechar uma arquitetura que caiba no hardware atual sem continuar empilhando recursos em um unico firmware pesado.

Principios:

- interface rasa: no maximo `Home -> Modo -> Submodo`
- `portrait-first` ate a rotacao ficar estavel
- uma tela operacional por vez
- backend desacoplado da UI
- `WiFi/Web` como variante separada se apertar a DRAM

---

## Estrutura definitiva dos menus

### Home

Grid `2x2` fixo:

- `PLANE`
- `METEO`
- `COMMS`
- `CONFIG`

Regras:

- sem blocos de debug na home
- no maximo um micro status por card
- `BOOT` pode virar `Back/Home`

### 1. PLANE

Objetivo:

- uso de voo, navegacao e paraquedismo

Submenus enxutos:

- `EFIS`
  - horizonte artificial
  - pitch / roll
  - heading
  - altitude barometrica
  - vertical speed
- `NAV`
  - GPS speed
  - GPS altitude
  - satelites / fix
  - heading track
- `SKYDIVE`
  - altitude AGL estimada
  - vertical speed
  - alerta de pull altitude
  - perfil simples de queda

Observacao:

- `EFIS` e `NAV` podem compartilhar a mesma base visual e alternar por botao ou swipe curto
- `SKYDIVE` deve ser uma tela dedicada e mais simples, sem excesso de dados

### 2. METEO

Objetivo:

- estacao local e estacao movel

Submenus enxutos:

- `LIVE`
  - temperatura
  - umidade
  - pressao
  - UV / lux
  - bateria
- `AIR`
  - IAQ / gas
  - BME688
  - resumo ambiental
- `TX`
  - telemetria ativa
  - intervalo LoRa
  - ultimo pacote
  - SD logger

Observacao:

- o fundo visual dinamico entra em `LIVE`, nao em todas as telas

### 3. COMMS

Objetivo:

- radio, rastreio e depuracao de campo

Submenus enxutos:

- `LORA`
  - TX / RX
  - RSSI / SNR
  - ultimo pacote
- `GPS`
  - fix
  - satelites
  - coordenadas
  - velocidade
- `LINK`
  - status WiFi AP
  - IP
  - hits web
  - mesh futura

Observacao:

- evitar um console textual longo
- cada submodo deve caber em cards grandes

### 4. CONFIG

Objetivo:

- manutencao e setup

Submenus enxutos:

- `SYSTEM`
  - brilho
  - orientacao
  - versao
  - memoria
- `SENSORS`
  - scan I2C
  - sensores encontrados
  - refresh
- `STORAGE`
  - SD status
  - logger on/off
  - limpar logs
- `CAL`
  - nivel do IMU
  - bussola
  - touch futuramente
- `NET`
  - SSID/AP
  - IP
  - modo web
  - credenciais futuras

Observacao:

- `CONFIG` nao deve mais ser uma tela unica lotada

---

## Regra de navegacao

Arquitetura recomendada:

- `Home`
- `Mode screen`
- `Optional detail screen`

Limites:

- profundidade maxima de 2 niveis abaixo da home
- sempre existe `Back`
- `BOOT` curto: `Back`
- `BOOT` longo: `Home`

---

## Estrategia de firmware

## Problema atual

O sketch principal ja esta perto do limite de DRAM.
O proprio handoff registra overflow de `.dram0.bss` quando `WiFi` entra junto.

Conclusao pratica:

- nao insistir em um unico firmware com cockpit completo + web completa + BLE

### Variantes recomendadas

#### Variante 1: `FLIGHT`

Objetivo:

- cockpit principal
- EFIS
- meteo local
- comms basico
- SD logger

Recursos:

- `WiFi` desligado por padrao
- `BLE` desligado
- foco em UI e sensores

Estado desejado no sketch:

- `#define BUILD_PROFILE_FLIGHT 1`

#### Variante 2: `WEB_CONFIG`

Objetivo:

- setup de rede
- web dashboard
- diagnostico de sensores
- SD status/download depois

Recursos:

- UI local minima
- `WiFi SoftAP`
- `WiFiServer` simples
- sem EFIS canvas
- sem telas pesadas

Estado desejado no sketch:

- `#define BUILD_PROFILE_WEB_CONFIG 1`

#### Variante 3: `LAB`

Opcional depois:

- testes de sensores
- calibracao
- traz pouca pressao de layout sobre o firmware de voo

### Implementacao recomendada

Primeira fase:

- manter um unico `.ino`
- introduzir `BUILD_PROFILE_*`
- compilar caminhos diferentes com `#if`

Segunda fase:

- mover backend comum para arquivos separados
  - `app_state.h`
  - `sensor_backend.h`
  - `logger_backend.h`
  - `network_backend.h`
  - `ui_router.h`

Motivo:

- separa firmware por perfil sem duplicar logica

---

## Estrategia de WiFi / Web

### O que usar

- `WiFiServer` simples
- `SoftAP` como primeira fase
- pagina HTML pequena
- endpoint `/api/status`
- polling no cliente a cada `1s` ou `2s`

### O que evitar agora

- `AsyncWebServer`
- websockets
- espelhamento completo da UI LVGL
- BLE junto do web mode no mesmo build

### Fluxo recomendado

#### Fase 1

- AP local fixo
- `http://192.168.4.1`
- dashboard minimo
- JSON status

#### Fase 2

- editar configuracoes via formulario
- salvar em NVS
- STA opcional

#### Fase 3

- download de logs SD
- pagina de sensores
- controle de logger

### Armazenamento de configuracoes

Usar `Preferences` / NVS para:

- SSID do modo station
- senha
- nome da estacao
- intervalo de logger
- intervalo de telemetria
- calibracoes

---

## Backend de sensores e navegacao

### Regra principal

A UI nao deve ler sensor diretamente.
Tudo deve passar por snapshots leves de estado.

### Camadas recomendadas

#### 1. `SensorBackend`

Responsavel por:

- polling I2C/UART/SPI
- retry e health flags
- cadence por sensor

Sensores:

- QMI8658
- BMP581
- BMM350
- BME688
- LTR390
- MAX17048
- GPS
- LoRa

#### 2. `FusionBackend`

Responsavel por:

- complementary filter inicialmente
- Madgwick depois
- heading consolidado
- altitude e vertical speed filtrados

#### 3. `LoggerBackend`

Responsavel por:

- SD mount/unmount
- abrir arquivo
- flush
- CSV schema
- rotacao de logs

#### 4. `NetworkBackend`

Responsavel por:

- SoftAP
- servidor HTTP
- JSON status
- configuracao futura

#### 5. `UiRouter`

Responsavel por:

- navegar entre modos
- abrir uma tela por vez
- gerenciar overlays de confirmacao
- impedir rebuilds agressivos durante orientacao

---

## Modelo de estado recomendado

Substituir espalhamento de labels e caches por estados agregados:

```cpp
struct FlightState {
  bool imu_ok;
  float pitch_deg;
  float roll_deg;
  float heading_deg;
  float altitude_m;
  float vertical_speed_mps;
  float gps_speed_mps;
  float gps_altitude_m;
  uint8_t sats;
  bool gps_fix;
};

struct MeteoState {
  bool bme_ok;
  float temperature_c;
  float humidity_pct;
  float pressure_hpa;
  float uv_index;
  float lux;
  float iaq;
  float battery_pct;
};

struct CommsState {
  bool lora_ok;
  uint32_t tx_count;
  uint32_t rx_count;
  int16_t rssi;
  int8_t snr;
  bool wifi_ap_on;
  uint32_t web_hits;
};

struct AppState {
  FlightState flight;
  MeteoState meteo;
  CommsState comms;
  BlackboxState blackbox;
};
```

Regras:

- backend escreve
- UI le snapshot
- sem `String` dinamica para estado critico

---

## Estrategia para preservar RAM

### 1. Uma tela viva por vez

Hoje o sketch precria varias telas e mantem tudo residente.

Recomendacao:

- criar somente a tela ativa
- destruir a anterior ao trocar
- manter apenas dialogs/overlays pequenos alem da tela atual

### 2. Portrait first

Recomendacao:

- congelar `AUTO`
- priorizar `portrait`
- reintroduzir rotacao so depois do roteador de telas estabilizar

### 3. Menos `String`

Recomendacao:

- trocar caches dinamicos por `char[]`
- formatar texto no refresh local da tela

### 4. Menos widgets por tela

Recomendacao:

- uma tela operacional com 4 a 6 elementos grandes
- detalhe em outra tela

### 5. PSRAM para historico

Usar PSRAM apenas para:

- canvas do EFIS
- buffers de historico simples
- nunca para multiplicar telas inteiras

### 6. Rede em variante

Recomendacao:

- web pesada fora do build de voo

---

## Ordem de implementacao

### Prioridade 1

- introduzir `BUILD_PROFILE_FLIGHT` e `BUILD_PROFILE_WEB_CONFIG`
- parar de manter todas as telas vivas ao mesmo tempo
- reconstruir `CONFIG` em submenus

### Prioridade 2

- criar `AppState`
- mover leitura de sensores para snapshots padronizados
- limitar UI a ler apenas snapshots

### Prioridade 3

- finalizar `PLANE`
  - BMP581
  - BMM350
  - GPS

### Prioridade 4

- construir variante `WEB_CONFIG`
- SoftAP
- dashboard
- NVS

### Prioridade 5

- completar `METEO`
- depois `COMMS`

---

## Recomendacao final

Se for preciso escolher uma unica decisao arquitetural agora, ela e esta:

- separar `cockpit` e `web/config` em perfis de firmware

Se for preciso escolher a segunda:

- parar de precriar todas as telas e adotar roteamento com uma tela ativa por vez

Essas duas mudancas atacam diretamente:

- DRAM alta
- travamentos de UI
- rotacao quebrando layout
- dificuldade de fazer o web mode caber
