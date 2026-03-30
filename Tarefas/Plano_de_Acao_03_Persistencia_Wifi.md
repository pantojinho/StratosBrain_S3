# Plano de Ação 03: Persistência de Wi-Fi e Modos (NVS) - REVISADO

**Objetivo:** Garantir que além do SSID/Senha, o **modo de operação** (AP ou Station) também persista após um reboot, evitando que o sistema sempre volte para o modo AP por padrão.

## Etapas Planejadas:

1. **Alterações no Código (`StratosBrain_LVGL9.ino`):**
   - **Renomear e Expandir Funções de IO:**
     - Mudar de `saveWifiStaCredentials` para `saveWifiPrefs()`: Agora salva SSID, Password, `g_wifi_ap_requested` e `g_wifi_sta_requested`.
     - Mudar de `loadWifiStaCredentials` para `loadWifiPrefs()`: Restaura tanto as credenciais quanto os estados dos modos.
   - **Persistência de Mudança de Modo:**
     - Modificar `requestWifiMode` e `requestWifiStaMode` para chamar `saveWifiPrefs()` sempre que o modo for alterado via UI.
   - **Restauração Inteligente no Setup:**
     - Substituir o bloco fixo de inicialização Wi-Fi no `setup()` pela chamada `loadWifiPrefs()`.
     - Implementar lógica de fallback (subir AP se nada estiver configurado).

2. **Novos Recursos:**
   - [X] Biblioteca: `Preferences.h`
   - [X] Novas Variáveis na NVS: `ap_on`, `sta_on`

3. **Validação:**
   - [ ] Ligar modo Station, conectar e dar Reboot. O sistema deve voltar tentando conectar na LAN, sem subir o AP (se configurado assim).
   - [ ] Ligar ambos ou apenas AP, dar Reboot. O estado deve ser idêntico ao pré-reboot.

4. **Atualização de Memória:**
   - [ ] Atualizar `GEMINI.md` e criar o resumo final.

---
*Plano sequencial de ação:*
1. Refatoração das funções de NVS.
2. Inserção do `saveWifiPrefs()` nos disparadores de modo.
3. Ajuste fino do `setup()`.

Aguardando sua aprovação para processar essas alterações!
