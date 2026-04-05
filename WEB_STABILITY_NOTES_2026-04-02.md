# Estabilidade do Dashboard Web

Data: 2026-04-02

## Sintoma observado

Depois de aproximadamente 15 minutos de operacao, o display local do dispositivo continuava exibindo dados corretos de GPS e meteorologia, mas a interface web passava a parar de atualizar ou mostrava campos zerados. Isso indicava que a aquisicao dos sensores seguia funcionando e que a degradacao estava concentrada na camada web.

## Hipotese tecnica

A causa mais provavel, por inferencia do codigo, era a combinacao destes fatores:

- polling frequente demais no navegador
- resposta `/api/status` pesada demais para todas as consultas
- possibilidade de requests concorrentes se acumularem no cliente
- timeout HTTP curto no firmware
- reinicio de servidor sem limpeza previa do listener

## Ajustes aplicados no firmware

Arquivo alterado: `StratosBrain_LVGL9/StratosBrain_LVGL9.ino`

1. Timeout HTTP do cliente aumentado.
   - `WIFI_CLIENT_TIMEOUT_MS` passou de `350U` para `1200U`

2. Atendimento HTTP endurecido por conexao.
   - `client.setTimeout(WIFI_CLIENT_TIMEOUT_MS)`
   - `client.setNoDelay(true)`

3. Reinicio do servidor web mais robusto.
   - `g_web_server.stop()` antes de `g_web_server.begin()`

4. Endpoint `/api/status` dividido em leitura leve e detalhada.
   - modo leve para polling continuo
   - modo detalhado apenas quando chamado com `detail=1`

5. Endpoint `/api/action` passou a devolver resposta leve.

6. Dashboard V2 ajustado para reduzir carga.
   - status leve a cada `2.5 s`
   - detalhes pesados a cada `10 s`
   - bloqueio de requests concorrentes
   - timeout e cancelamento com `AbortController`

7. Endurecimento de seguranca simples.
   - senha do AP removida do JSON exposto pela API

## Campos pesados movidos para modo detalhado

Os seguintes blocos deixaram de trafegar em toda consulta leve:

- `blackbox.tail`
- `blackbox.files`
- `gps.history`
- `lora.history`
- `lora.export`
- `activity.recent`

## Resultado esperado

Com menos carga por request e sem empilhamento de fetches, o dashboard deve continuar recebendo dados por periodos longos sem congelar ou zerar os campos, mesmo com o dispositivo ligado continuamente.

## Validacao recomendada

1. Gravar o firmware atualizado no ESP32-S3.
2. Abrir o dashboard web e manter a pagina aberta por pelo menos `30 a 60 minutos`.
3. Verificar se `wifi.web_hits` continua aumentando durante todo o periodo.
4. Confirmar que GPS, umidade, pressao e demais leituras continuam variando normalmente.
5. Repetir um soak test de `24 h` quando a rodada curta estiver estavel.

## Observacoes

- Nao foi possivel compilar nesta maquina porque `arduino-cli` nao esta instalado.
- A causa raiz ainda precisa de confirmacao empirica com teste prolongado.
