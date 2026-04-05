# 433

Espaco reservado para a frente `Tracker GPS LORA 433` do StratosBrain.

Objetivo inicial:

- monitorar pacotes recebidos pelo modem 433 em UART
- extrair `node`, `lat`, `lon`, `alt`, `speed` e `course` quando presentes
- mostrar o ultimo fix e um historico leve no dashboard web
- persistir as capturas no SD em `/logs/lora433_capture.csv`

Formatos de payload que o parser da web tenta reconhecer:

- `lat=-23.5505 lon=-46.6333`
- `latitude=-23.5505 longitude=-46.6333`
- `node=baloon1,lat=-23.5505,lon=-46.6333,alt=1234`
- `IOT;node=tracker-a;cmd=uplink;data=lat=-23.55,lon=-46.63`
- pares simples `-23.5505,-46.6333`

Proximos passos sugeridos:

1. padronizar um payload 433 unico para o seu modem
2. mover o parser principal do navegador para o firmware, se ficar estavel
3. adicionar lista de objetos ativos com timeout e ultimo horario visto
4. opcional: exportar trilha CSV ou GeoJSON
