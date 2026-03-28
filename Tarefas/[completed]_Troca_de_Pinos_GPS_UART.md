# Resumo da Tarefa Concluída

**Ação Finalizada:** Troca dos pinos UART do GPS (de 43/44 para 1/2).
**Data de Conclusão:** 2026-03-27

## O que foi feito:
1. Identificação do conflito de Hardware: Os pinos 43 e 44 do ESP32-S3 são os pinos nativos do Serial (U0TXD e U0RXD) do console.
2. Criação do Plano de Ação em `Tarefas` detalhando os passos.
3. Alteração no código fonte (`StratosBrain_LVGL9.ino`): As constantes `GPS_UART_TX_PIN` e `GPS_UART_RX_PIN` foram reatribuídas para os pinos 2 e 1, respectivamente.
4. Alteração na documentação (`ESQUEMA_LIGACOES.md`): Atualização oficial da fiação para o GPS, refletindo a nova orientação de pinos (TX do GPS no pino 1, RX do GPS no pino 2 do ESP32).

O sistema agora está pronto e isolado do processo de comunicação USB principal.
