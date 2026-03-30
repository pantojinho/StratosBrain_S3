# Resumo da Tarefa [completed]: Persistência Wi-Fi + Modos (NVS)

**Status:** Concluído com Sucesso ✅

## 📋 O que foi aprimorado:

1. **Persistência de Estados (Modo AP/STA):**
   - O sistema agora não salva apenas SSID e senha, mas também os flags `g_wifi_ap_requested` e `g_wifi_sta_requested`.
   - Mudança no namespace NVS de `wifi_sta` para `wifi_cfg` (configuração Wi-Fi completa).

2. **Refatoração no Boot (setup):**
   - Removido o código "hardcoded" que forçava o modo AP em todo boot.
   - Adicionada lógica de fallback que ativa o modo AP apenas se nenhum modo (`AP` ou `STA`) estiver configurado como ativo na memória não-volátil.

3. **Gatilhos de Salvamento:**
   - Adicionadas chamadas para `saveWifiPrefs()` em:
     - `requestWifiStaCredentials` (Carga de novas redes)
     - `requestWifiMode` (Alternância do AP)
     - `requestWifiStaMode` (Alternância do Station)

4. **Estabilidade de Compilação:**
   - Adicionadas as declarações antecipadas (`forward declarations`) para `saveWifiPrefs` e `loadWifiPrefs`, já que são chamadas antes de suas definições completas no arquivo principal.

## 🚀 Resultados:

- O dispositivo agora "lembra" se o usuário preferia que o AP estivesse desligado ou o STA ligado, mantendo a configuração desejada mesmo após quedas de energia ou reboots manuais.

---
*Tarefa revisada e finalizada seguindo as instruções refinadas do usuário.*
