# Plano de Ação: Alteração dos Pinos UART do GPS

**Objetivo:** Evitar o conflito de hardware entre o GPS e a porta Serial Monitor/USB nativa do ESP32-S3 (pinos 43 e 44).

## Etapas Planejadas:

1. **Alterar Definições no Código (`StratosBrain_LVGL9.ino`):**
   - Modificar a constante `GPS_UART_TX_PIN` de `44` para `2`.
   - Modificar a constante `GPS_UART_RX_PIN` de `43` para `1`.

2. **Revisar a Fiação Física (Instruções para o Usuário):**
   - Conectar o cabo **TX** do módulo GPS no pino **1** (RX do ESP32).
   - Conectar o cabo **RX** do módulo GPS no pino **2** (TX do ESP32).
   - (Se não funcionar, tentar inverter os cabos entre pino 1 e 2, garantindo o padrão cruzado TX-RX e RX-TX).

3. **Atualização de Documentos:**
   - Adicionar nota técnica sobre a resolução do problema e por que as portas 43 e 44 eram problemáticas para esse uso na placa Waveshare.

*P.S.: Notei nas suas regras o uso de `GEMINI.md` e da pasta `templates IA/`. No entanto, como elas não estão presentes na estrutura atual deste projeto do ESP32, criei este plano de ação direto na pasta `Tarefas`. Por favor, revise e aprove para eu realizar a alteração no código!*
