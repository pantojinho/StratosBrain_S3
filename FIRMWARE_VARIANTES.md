# StratosBrain S3 - Variantes de Firmware

**Date:** 2026-03-27

---

## Quando usar cada firmware

### 1. Firmware principal

Arquivo:

- `StratosBrain_LVGL9/StratosBrain_LVGL9.ino`

Use quando o objetivo for:

- interface AMOLED principal
- `PLANE`, `METEO`, `COMMS`, `CONFIG`
- EFIS
- touch
- IMU
- logger SD
- evolucao da UX principal

Nao e o firmware certo para:

- testar `WiFi/Web` pesado
- tentar colocar servidor web junto do cockpit enquanto a DRAM estiver apertada

### 2. Firmware web/config leve

Arquivo:

- `StratosBrain_WebConfig/StratosBrain_WebConfig.ino`

Use quando o objetivo for:

- subir `SoftAP`
- ver IP no serial
- abrir pagina web simples no celular/notebook
- consultar `/api/status`
- validar I2C e SD sem carregar a UI principal

Recursos atuais:

- `SoftAP`
- pagina HTML simples
- endpoint JSON
- scan I2C
- status SD

Credenciais atuais:

- SSID: `StratosBrain-S3`
- senha: `stratos123`
- IP esperado: `192.168.4.1`

Rotas:

- `http://192.168.4.1`
- `http://192.168.4.1/api/status`
- `http://192.168.4.1/rescan`

---

## Motivo da separacao

O projeto principal esta com RAM alta.
Quando `WiFi/Web` entra junto do cockpit completo, a DRAM fica pressionada demais e o risco de travamento sobe.

Conclusao pratica:

- cockpit e web devem evoluir separados por enquanto

---

## Board settings

As configuracoes recomendadas continuam:

- Board: `Waveshare ESP32-S3-Touch-AMOLED-1.64`
- PSRAM: `Enabled`
- USB CDC On Boot: `Enabled`
- Partition Scheme: `16M Flash (3MB APP/9.9MB FATFS)`

---

## Proximo passo recomendado

Fluxo ideal de trabalho:

1. usar `StratosBrain_WebConfig.ino` para validar rede e web
2. usar `StratosBrain_LVGL9.ino` para validar UI, EFIS e sensores
3. so depois decidir quais configuracoes precisam ser compartilhadas via NVS entre os dois
