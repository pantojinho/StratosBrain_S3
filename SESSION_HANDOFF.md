# SESSION HANDOFF — StratosBrain-S3
**Data:** 2026-03-28 | **Última atualização:** 23:27 BRT  
**Arquivo principal:** `StratosBrain_LVGL9/StratosBrain_LVGL9.ino`

---

## 🎯 Objetivo do Projeto

Sistema embarcado em ESP32-S3 (Waveshare Touch AMOLED 1.64") que integra:
- Sensor ambiental BME688 (temperatura, pressão, umidade, gás)
- GPS CASIC GP10 via UART (NMEA 0183)
- IMU QMI8658 (acelerômetro + giroscópio)
- Módulo LoRa E220 (UART, pinos TX=17, RX=18, AUX=6, M0=7, M1=8)
- Display AMOLED CO5300 via QSPI + LVGL 9
- Cartão SD (blackbox/logger)
- Web Dashboard via Wi-Fi AP (SSID: `StratosBrain-S3`, senha: `stratos123`, IP: `192.168.4.1`)

---

## 📁 Estrutura do Projeto

```
c:\Esp32\CLAUDE\StratosBrain_S3\
├── StratosBrain_LVGL9\
│   └── StratosBrain_LVGL9.ino   ← ARQUIVO PRINCIPAL (6900+ linhas)
├── Tarefas\
│   └── [completed]_Plano_02_Refinamentos_Gerais.md
├── SESSION_HANDOFF.md            ← ESTE ARQUIVO
└── GEMINI.md
```

---

## 🔧 Configurações de Hardware (Pinagem)

| Função | Pino(s) |
|--------|---------|
| Display QSPI CS/CLK/D0-D3/RST | 9,10,11,12,13,14,21 |
| I2C SDA/SCL | 47, 48 |
| GPS UART TX/RX | 2 (TX→GPS), 1 (RX←GPS) |
| LoRa UART TX/RX | 17, 18 |
| LoRa AUX/M0/M1 | 6, 7, 8 |
| SD CS/MOSI/MISO/SCK | 38,39,40,41 |
| RGB LED | 45 |
| Boot Button | 0 |
| Touch FT3168 | I2C 0x38 |
| IMU QMI8658 | I2C 0x6B |
| BME688 | I2C 0x77 |

---

## ✅ O Que Foi Implementado Nesta Sessão

### 1. Dashboard Web (AJAX)
- Substituído texto estático por JSON via `/api/status` (polling a cada 1,8s)
- Abas: **Dashboard**, **GPS**, **Cartão SD**, **Meteorologia**, **Rede**, **LoRa**, **Configuração**, **Aviação**, **Visão Geral**
- Diretiva `translate="no"` + meta `google: notranslate` para evitar Google Tradutor quebrar a UI
- Endpoint novo: **`/api/scan`** — varre redes Wi-Fi e retorna JSON `{networks:[{ssid,rssi,enc}]}`
- Google Maps e OpenStreetMap: links atualizados dinamicamente com coordenadas GPS reais (corrigido de `href='#'` fixo)

### 2. Aba GPS — Skyplot
- Skyplot via Canvas (HTML5) com plotagem de satélites por constelação
- **Legenda de contagem por tipo** adicionada: GPS / GLONASS / BeiDou / Galileo (pills coloridas)
- Identificação de constelação pelos campos GSV do NMEA:
  - `$GPGSV` → G (verde) | `$GLGSV` → R (roxo) | `$BDGSV`/`$GBGSV` → B (laranja) | `$GAGSV` → E (ciano)
- **Bug corrigido:** parser GSV tinha `fields[0][0]` (que é sempre `$`) em vez de `fields[0][1]` e `fields[0][2]` para identificar a constelação

### 3. Aba Rede (Wi-Fi)
- Lista clicável de redes Wi-Fi disponíveis ao pressionar **"Varrer Redes Wi-Fi"**
- Clique na rede preenche automaticamente o campo SSID
- Mostra sinal (barras ▂▄▆█), dBm e ícone de cadeado 🔒
- Botões Conectar / Desconectar

### 4. Aba Aviação (Plane)
- Cards: Altitude média (m), Velocidade Vertical (m/s), Velocidade GPS (km/h)
- **Horizonte Artificial Web:** Canvas HTML5 com céu azul/terra marrom, linha de horizonte, marcas de ângulo de banco ±30°/60° e marcador central dourado
- Pitch e Roll são lidos do JSON `d.imu.pitch_deg` / `d.imu.roll_deg` em tempo real

### 5. Filtro EMA (Firmware)
- Variáveis globais: `g_plane_smooth_alt`, `g_plane_smooth_vs`, `g_plane_smooth_spd`, `g_plane_last_update_ms`
- Calculadas no `loop()` combinando altitude do BME688 + GPS
- Alpha EMA: altitude=0.2, velocidade=0.2, VS=0.15 (estabilidade boa para bancada)
- Expostas via `/api/status` em `d.plane.avg_altitude_m`, `d.plane.avg_vs_ms`, `d.plane.avg_speed_kmh`

### 6. Correções de Compilação
- Adicionado `#include <WiFi.h>` (necessário para `WiFi.scanNetworks()`)
- Adicionadas **forward declarations** para funções usadas antes de sua definição:
  - `ensureEfisCanvasBuffer()` — definida em ~linha 4906
  - `drawEfisHorizon(const ImuState&)` — definida em ~linha 5000
  - `refreshSensorCaches()` — definida em ~linha 3620
  - `efisSetLevelEventCb(lv_event_t*)` — definida em ~linha 5465
  - `loadScreenById(AppScreenId)` — declarada **após** o enum `AppScreenId` (~linha 213)

---

## 🔴 Estado Atual / Pendências

### Compilação
- A correção das forward declarations foi aplicada **nesta sessão**
- O agente deve compilar e verificar se ainda há erros antes de prosseguir
- Aviso persistente de lint do IDE (não é erro real): `Unable to handle compilation, expected exactly one compiler job`

### Funcionalidades Pendentes / A Validar
1. **Teste de campo** — EMA está configurado mas não testado em voo real
2. **Wi-Fi Scan** — `WiFi.scanNetworks()` bloqueia ~3s; monitorar impacto no sistema
3. **Satélites NMEA** — O reset do `sats_count` só ocorre na msg GSV nº1 do tipo `GP`; precisa testar com múltiplas constelações ativas simultaneamente
4. **Aba SD** — Cards HTML criados mas `d.blackbox.tail` e `d.blackbox.file_name` precisam ser validados no JSON de status
5. **Aba Visão Geral** — `d.activity_history` e `d.touch.*` precisam ser confirmados no JSON

---

## 📡 Arquitetura do Firmware

### Tarefas FreeRTOS
| Tarefa | Core | Função |
|--------|------|--------|
| `imuTask` | 0 | Leitura QMI8658 + filtro complementar |
| `blackboxTask` | 0 | Logger SD |
| `loop()` | 1 | GPS + BME688 + EMA + LV_Task + Wi-Fi |

### JSON `/api/status` — Campos Importantes
```json
{
  "plane": {"avg_altitude_m": 0.0, "avg_vs_ms": 0.0, "avg_speed_kmh": 0.0},
  "imu": {"connected": true, "pitch_deg": 0.0, "roll_deg": 0.0, "temperature_c": 0.0},
  "gps": {"fix": false, "sats": 7, "lat": -23.5, "lon": -47.5, "alt_m": 648.0, "has_location": true, "update_hz": 1},
  "sats_arr": [{"p": 27, "e": 45, "a": 180, "s": 32, "t": "G"}],
  "lora": {"enabled": true, "rx_bytes": 0, "tx_bytes": 0, "last_message": "", "history": ""},
  "meteo": {"connected": true, "temperature_c": 30.1, "humidity_pct": 51.7, "pressure_hpa": 948.2, "altitude_m": 648.0},
  "blackbox": {"logging_enabled": false, "file_name": "", "file_path": ""},
  "wifi": {"ap": true, "ip": "192.168.4.1", "clients": 0, "web_hits": 5, "sta_connected": false}
}
```

### Endpoints HTTP
| Endpoint | Descrição |
|----------|-----------|
| `GET /` | Dashboard HTML |
| `GET /api/status` | JSON completo do sistema |
| `GET /api/action?...` | Executa ações (lora=toggle, wifi=toggle, etc.) |
| `GET /api/scan` | Varre redes Wi-Fi, retorna JSON |

### Ações HTTP (`/api/action`)
- `wifi=toggle` — Liga/desliga AP
- `lora=toggle` — Liga/desliga LoRa sniffer
- `lora_tx=ping` / `lora_tx=status` — Envia pacote LoRa
- `lora_payload=TEXT` — Envia payload personalizado
- `logger=toggle` — Liga/desliga blackbox SD
- `sta_ssid=X&sta_pass=Y&sta=on` — Conecta no Wi-Fi residencial
- `sta=off` — Desconecta do Wi-Fi residencial
- `gps_restart=cold` / `gps_restart=hot` — Reinicia GPS
- `gps_rate=1` / `gps_rate=5` — Taxa de atualização GPS (Hz)
- `gps_constellation=all` / `gps_constellation=gps_bds` — Seleciona constelações
- `gps_nmea=all_on` — Habilita todos os tipos NMEA
- `screen=efis` / `screen=home` / etc. — Muda tela no display

---

## ⚠️ Cuidados Importantes

1. **Quando o módulo conecta no Wi-Fi residencial**, o AP (192.168.4.1) é desligado. O usuário deve usar o novo IP do roteador.
2. **GPS UART** usa lógica inversa de pinos: TX do ESP vai para RX do GPS e vice-versa. Pinos: GPIO1/GPIO2.
3. **O arquivo `.ino` tem 7000+ linhas** — edições devem ser cirúrgicas e pequenas para evitar erros de substituição.
4. **Lint do IDE** reporta erro de compilação mesmo sem erro real — é limitação do environment, não do código.
5. **EMA do horizonte artificial** usa dados do QMI8658 com filtro complementar α=0.985. Se a placa estiver parada, o giroscópio pode mostrar drift pequeno (~1°) — isso é normal com sensor sem calibração de zero.

---

## 🔗 Referências Rápidas

- **Biblioteca Display:** `Arduino_GFX_Library` + driver `CO5300`
- **LVGL versão:** 9.x
- **Framework:** Arduino (ESP32 Arduino Core 3.3.7)
- **Board:** `esp32s3` — Waveshare ESP32-S3 Touch AMOLED 1.64
- **Repositório:** Gerenciado manualmente pelo usuário (git local)
