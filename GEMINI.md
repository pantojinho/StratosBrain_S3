# GEMINI Project Memory - StratosBrain S3

## 🎯 Visão Geral
Sistema de aviônica e telemetria baseado em ESP32-S3 com display AMOLED, sensores climáticos, GPS, LoRa e interface Web.

## 🛠️ Estado Atual
- **Hardware:** ESP32-S3, AMOLED 1.64", BME688, GPS GP10, LoRa E220, SD Card.
- **Firmware:** Baseado em Arduino Core 3.x, LVGL 9, Servidor Web AJAX.
- **Conectividade:** AP Wi-Fi funcional, Scanner de rede funcional, modo Station implementado mas sem persistência (TRABALHANDO NISSO AGORA).

## 📝 Histórico de Tarefas
1. **Pinos GPS:** Alterados para GPIO 1/2 para evitar conflito com USB/JTAG nativo. [CONCLUÍDO]
2. **Refinamentos Gerais:** Implementação de AJAX no Web UI, filtros EMA para sensores, Horizon Canvas. [CONCLUÍDO]
3. **Persistência Wi-Fi + Modos:** NVS via `Preferences` para salvar SSID/Senha e modo operacional (AP/STA) de forma persistente. [CONCLUÍDO]

## 📌 Links e Referências
- `StratosBrain_LVGL9/StratosBrain_LVGL9.ino` - Código principal.
- `SESSION_HANDOFF.md` - Contexto detalhado da última sessão.
