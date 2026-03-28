# Resumo da Etapa Concluída: Plano 02 - Refinamentos Gerais

## O que foi realizado
1. **Filtros de Estabilização (Sensibilidade de Sensores)**
   - Adicionada uma `deadband` de 0.6 graus para `pitch` e `roll` vindos do QMI8658, evitando jitter visual na tela com o dispositivo parado na bancada.
   - Adicionada uma trava na velocidade do GPS: se for menor que 3 km/h, marca 0 km/h.

2. **Parsing de Satélites NMEA (Skyplot)**
   - O código do GPS passou a ler e interpretar fisicamente strings `$GPGSV`, `$BDGSV`, `$GLGSV`, `$GAGSV`.
   - Adicionado um array circular na API JSON de status para trafegar via Ajax os satélites em repouso e movimento, com suas posições de elevação e azimute.

3. **Automação Wi-Fi (AP off)**
   - Implementado o desligamento contínuo / automático do Access Point (`StratosBrain-S3`) quando a placa captura um IP DHCP local na sua rede Station, poupando bateria.

4. **Nova Web UI AJAX & Skyplot**
   - Refeito todo o HTML injetado a partir da RAM do ESP32, garantindo zero refreshes de tela com o uso de `fetch` via JavaScript a cada 1.8 segundos.
   - Adicionado no dashboard o Canvas desenhado via JS que renderiza um *Skyplot* local usando os dados GSV do GPS.
   - Adicionado formulário de despache do LoRa (envio de Payload via Web) e Sniffer de console.
   - Limpeza do código Python `apply_patch.py`, deixando o ambiente limpo.

## Próximos Passos
- Conforme listado na Documentação Global (e possivelmente pendente em relação aos menus da placa em si), a próxima fase pode avançar para criar a interface gráfica touch dos Satélites na própria tela do ESP32 ou focar no Machine Learning dos gases BME688. 

> *Regra Global atendida.*
