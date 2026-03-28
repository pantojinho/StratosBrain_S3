# Plano de Ação 02: Refinamento Robusto do Sistema (Atualizado)

Este plano atende às solicitações originais e aos ajustes finais (áudio), garantindo que o Web UI pare de dar *refresh* incômodo, que o Wi-Fi funcione em Station com IP via DHCP perfeitamente e que o módulo LoRa possa atuar como um *sniffer*.

## Parte 1: Wi-Fi UI (Scanner) e Servidor Web em STA
- **Scanner UI (LVGL):** Modificar a aba de rede para primeiro rodar um `WiFi.scanNetworks()` (ou rotina equivalente de Idf/WiFi). Montar uma lista no display (LVGL List/Dropdown) com as redes próximas; ao clicar numa rede, o sistema abre o teclado apenas para digitar a senha.
- **Auto-Turnoff do AP:** Assim que a conexão do modo Station (STA) for confirmada (pegar IP via DHCP), o modo AP é desligado. O Servidor Web passará a responder exclusivamente no IP recebido na rede local.

## Parte 2: Redesign Moderno e Ajax do Servidor Web
- **Fim dos "Pulos" e Refresh constantes:** Remover completamente o `<meta http-equiv="refresh">` do código HTML gerado no `StratosBrain_LVGL9.ino`.
- **Ajax / Fetch:** Substituir por JavaScript moderno (`fetch('/api/status')`) rodando num `setInterval`. Assim, os dados na tela atualizam suavemente, sem piscar e sem perder a posição do scroll.
- **Visual Moderno (CSS):** Atualizar o CSS para ser robusto, escuro (Dark Mode) de verdade, botões bonitos e agradável.

## Parte 3: Estabilização de Sensores (Jitter e Drift)
- **GPS (Velocidade):** Implementar um *Deadband* (zona morta). Se a velocidade for menor que ~3 km/h, o painel assume `0 km/h` para evitar "jitter" na mesa.
- **IMU (QMI8658):** Adicionar um limiar dinâmico (*deadband*) no giroscópio. Pequenas flutuações não devem recalcular o pitch/roll do horizonte (evitando variação na bancada).

## Parte 4: LoRa Sniffer e GPS Mapas/Casic no Web UI
- **LoRa Sniffer:** Garantir a inicialização física do UART LoRa (`g_lora_enabled`). Modificar o front-end web para listar os pacotes interceptados claramente, tornando-o um receptor/escuta da rede.
- **GPS e Visão de Satélites/Mapas:** Parsear as sentenças `GSV` para enviar as constelações vistas à Web UI.
- **Comandos CASIC:** Documentar/explicar na Web o que cada botão ("Partida a frio", "5Hz", "GPS+BDS") realmente faz.

## Parte 5: Cartão SD
- Revisão técnica da função de montagem/timeout no barramento SPI (`38, 39, 40, 41`) e adicionar feedback visual para o usuário quando o SD falhar ou for removido.

---
Vou começar a bater cada ponto, começando pelos **Filtros dos Sensores (Parte 3)** (que aliviam a tela de pular muito na bancada local) e a **Lógica de Wi-Fi/Servidor (Parte 1 e 2)**!
