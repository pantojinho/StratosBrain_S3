# StratosBrain S3 - Menu e UI Design

**Date:** 2026-03-27  
**Purpose:** consolidar ideias de menus, linguagem visual e o ponto atual do projeto

---

## Visao do produto

Complemento tecnico de arquitetura:

- `ARQUITETURA_TECNICA.md`

O objetivo do software e transformar a placa em um pequeno "cockpit digital" modular.
Nao e so uma tela de sensores.
A interface deve parecer um instrumento de bordo compacto:

- leitura rapida
- toque simples
- boa visibilidade ao ar livre
- navegacao curta
- telas focadas por contexto

A ideia central agora e dividir o sistema em 4 modos principais:

- `PLANE`
- `METEO`
- `COMMS`
- `CONFIG`

Cada modo deve existir tanto na tela AMOLED quanto, no futuro, numa visao web para celular.

---

## Estado atual

### O que ja esta feito

- display CO5300 funcionando
- touch FT3168 funcionando
- menu principal funcionando
- telas base `EFIS`, `METEO`, `GPS` e `CONFIG` criadas como primeira estrutura
- `QMI8658` integrado em tarefa FreeRTOS dedicada
- horizonte artificial inicial desenhado em LVGL canvas
- leitura de `pitch` e `roll` visivel na tela `EFIS`
- orientacao vertical, horizontal e auto funcionando
- `CONFIG / TESTE` com scan I2C e resumo dos sensores
- logger de caixa-preta em microSD criado como primeira base de telemetria
- acao segura para limpar os logs do SD e preparar uma nova missao
- base inicial de `WiFi AP + pagina web` adicionada para espelhar status no celular
  - mas ela nao cabe no firmware principal hoje
  - a direcao certa agora e uma variante separada para `web/config`

### Onde paramos

O projeto nao parou por falta de sensores.
Ele parou porque a interface atual ficou instavel e apertada demais para a tela pequena.

Resumo real do ponto atual:

- o `EFIS` agradou visualmente e o horizonte artificial foi visto como promissor
- o hardware base esta respondendo
- a navegacao e o layout ainda nao estao prontos para uso real
- antes de crescer o produto, a UI precisa ser simplificada
- a web agora deve evoluir em firmware separado, nao dentro do cockpit principal

Problemas observados pelo usuario:

1. textos ficam sobrepostos
2. alguns menus estao ruins de enxergar
3. `CONFIG` ficou apertado
4. `CONFIG` nao tem volta confiavel para a tela principal
5. mudanca entre vertical e horizontal quebra o layout
6. em algumas transicoes a tela trava ou falha ao trocar
7. o produto precisa parecer mais um relogio/wearable do que uma tela de debug

Conclusao:

- a proxima sessao deve ser um redesign de UX e navegacao
- adicionar mais features agora tende a piorar o estado atual

Primeiro passo ja aplicado no firmware principal:

- header mais curto
- home maior e com melhor aproveitamento da area util
- home com faixa de status compacta
- subtitulo inicial do launcher removido
- `PLANE` compactado em portrait
- `PLANE` com inversao de roll ajustada no software
- `PLANE` com amortecimento visual para o horizonte parecer menos nervoso
- `CONFIG` reduzido para blocos mais curtos
- `CONFIG` com bloco proprio de rede e links
- `COMMS` reorganizado em cards grandes para `WiFi/Web`, `BLE`, `LoRa` e `GPS`
- `BOOT` usado como `Home` em runtime
- `AUTO` congelado para reduzir quebra de UI
- botao de orientacao travado em `portrait fixo`
- smoke test visual do display desativado no boot
- a estrategia mais segura agora e:
  - cockpit principal leve
  - variante `WebConfig` separada para WiFi e browser

Diagnostico de rede:

- o cockpit principal nao sobe `WiFi/Web`
- a variante `WebConfig` e a referencia para testar `SSID`, senha, IP e acesso via celular
- o serial do `WebConfig` deve ser tratado como fonte principal de diagnostico de rede

Atualizacao mais recente:

- a estrategia mudou de `WebConfig` separado para um firmware unico com `WiFi AP` integrado
- `COMMS` virou a central de rede e telemetria
- `CONFIG` agora tambem mostra resumo de `WiFi`, `IP`, clientes e estado do `LoRa`
- quando o `WiFi AP` fica ativo, o sistema entra em modo leve para aliviar a interface
- o maior ganho tecnico veio de um `lv_conf.h` local que deixou o LVGL bem mais enxuto para este hardware
- `PLANE` deixou de depender do horizonte artificial animado como elemento principal
- a direcao atual do `PLANE` e ocupar a tela com dados grandes e legiveis:
  - altitude
  - vertical speed
  - heading
  - speed
  - pitch / roll / status do IMU
- o `PLANE` agora deve conviver com `WiFi AP` ativo sem derrubar o acesso

Decisao tecnica importante:

- o microSD sera usado como armazenamento, logger e caixa-preta
- ele nao substitui RAM ou PSRAM

Tambem ainda nao entraram:

- Madgwick
- magnetometro `BMM350`
- heading real
- altitude e variometro
- sensores meteo em tela
- GPS ao vivo
- LoRa
- parser NMEA do `AT6558R` preenchendo o CSV do logger
- BLE no firmware principal
- portal web mais rico sem estourar a DRAM

---

## Arquitetura de modos

### Visao oficial atual

O software deixa de ser pensado como um app unico de sensores e passa a ser um sistema modular com 4 modos:

1. `PLANE`
2. `METEO`
3. `COMMS`
4. `CONFIG`

Esses modos sao o mapa principal tanto para a interface na tela quanto para a futura interface web.

---

## Arquitetura de navegacao

### Nivel 1: Home

A `Home` deve continuar como uma tela curta de acesso rapido, mas agora apontando para os 4 modos oficiais.

Elementos:

- header com nome do sistema
- 4 botoes grandes para os modos
- status minimo
- nada de blocos longos de debug na home

Decisao atual:

- manter grade `2x2` com botoes grandes
- manter navegacao rasa
- evitar menu escondido ou navegacao profunda no comeco
- priorizar legibilidade acima de quantidade de informacao

### Nivel 2: Back/Home

Esse ponto virou obrigatorio.

Toda tela secundaria deve ter uma forma consistente de voltar.

Direcao recomendada:

- botao visual `Voltar`
- gesto ou toque simples apenas se ficar confiavel
- estudar uso do botao fisico `BOOT` como atalho de `Back/Home`

Observacao:

- usar o `BOOT` como acao de sistema pode combinar muito bem com a proposta wearable
- isso pode reduzir a dependencia de pequenos alvos de toque
- no estado atual do projeto, essa virou a estrategia preferida para `Home/Back`

### Regra nova: portrait-first

Enquanto a UI nao estiver estavel, a interface deve ser tratada como `portrait-first`.

Pragmaticamente:

- `portrait` e o layout principal
- `landscape` vira modo experimental
- `auto` deve ser desligado por padrao ou escondido ate a navegacao parar de quebrar

Isso reduz duas classes de bug ao mesmo tempo:

- sobreposicao de objetos por falta de espaco
- rebuilds agressivos de tela durante rotacao

---

## Estrutura de menus

### 1. PLANE

Objetivo:
ser o modo de voo e navegacao, com instrumentos basicos de aviacao e dados de posicao.

#### Blocos principais

- horizonte artificial
- pitch
- roll
- altitude barometrica
- variometro
- bussola digital
- heading
- velocidade GPS
- altitude GPS
- satelites e fix

#### Sensores e fontes

- `QMI8658` para atitude
- `BMM350` para heading/bussola
- `BMP581` para altitude e vario
- `AT6558R` para GPS

#### Regras de UX

- prioridade maxima para leitura instantanea
- texto grande
- referencias visuais de aviacao
- botao rapido para calibrar nivel
- evitar excesso de labels pequenos
- manter poucos instrumentos por tela se necessario

#### Submodos futuros

- `EFIS`
- `NAV`
- `LOGGER`

---

### 2. METEO

Objetivo:
ser a mini estacao meteorologica embarcada.

#### Dados esperados

- temperatura
- umidade
- pressao
- altitude
- UV
- lux
- qualidade do ar
- bateria

#### Sensores previstos

- `BME688`
- `BMP581`
- `LTR-390UV`
- `MAX17048`
- `BMM350` como apoio de contexto

#### Estrutura sugerida

- tela resumo com cards grandes
- telas de detalhe por sensor
- historico simples depois
- leitura clara para uso em campo
- visual que mude com o contexto:
  - dia
  - noite
  - chuva
  - nublado

#### Extensao direta

- esse modo precisa ter uma variante `METEO TX`
- `METEO TX` usa os mesmos dados do modo meteorologico
- diferenca principal: transmissao via `LoRa` em intervalos definidos
- foco em missao movel ou balao meteorologico

---

### 3. COMMS

Objetivo:
ser o modo de comunicacao, rastreio e debug de radio.

#### Funcoes principais

- status do `LoRa`
- pacotes enviados
- pacotes recebidos
- RSSI / SNR
- GPS resumido para rastreio
- visualizacao de dados recebidos
- debug de rede mesh futura
- debug de telemetria de campo

#### Estrutura sugerida

- aba `LoRa`
- aba `GPS`
- aba `Mesh`
- aba `Debug`

Nota de UX:

- essas abas talvez precisem virar paginas simples e grandes, nao mini-consoles densos
- o modo `COMMS` deve ser legivel com o dispositivo na mao, nao so como tela de engenharia

#### Papel no produto

- modo operacional para rede de campo
- modo de diagnostico de radio
- modo de recepcao de dados de outros nos
- modo de auditoria da caixa-preta/telemetria

---

### 4. CONFIG

Objetivo:
concentrar ajustes de sistema, calibracao e ferramentas de manutencao.

#### Itens previstos

- brilho da tela
- teste de touch
- calibracao do IMU
- calibracao da bussola
- calibracao do horizonte
- informacoes de memoria
- versao do firmware
- scan I2C
- status da PSRAM
- status SD
- limpeza de logs SD
- ajustes de telemetria
- ajustes de `LoRa`
- configuracao de rede
- pagina de testes rapidos
- configuracao web futura

#### Recomendacao

Nao misturar configuracao com dados de missao.
`CONFIG` deve ser util para ajuste e manutencao.

Direcao nova:

- `CONFIG` deve ser o modo mais simples visualmente
- menos informacao simultanea
- listas mais curtas
- grupos claros:
  - sistema
  - sensores
  - armazenamento
  - calibracao
  - rede

---

## Web espelhada

### Direcao correta

A interface web nao deve ser apenas uma pagina de setup.
Ela deve espelhar os 4 modos principais do sistema:

- `PLANE`
- `METEO`
- `COMMS`
- `CONFIG`

### Uso esperado no celular

- acompanhar dados da tela em tempo real
- fazer configuracao mais confortavel
- visualizar telemetria e status do logger
- depurar `LoRa` e GPS
- ajustar parametros sem depender da tela pequena

### Estrategia recomendada

- manter a UI principal da placa enxuta
- criar a web como variante separada ou modo leve
- evitar colocar servidor web pesado dentro do firmware completo se a DRAM continuar no limite

---

## Direcao visual

### Identidade geral

A interface deve seguir uma linguagem de "instrumento de bordo AMOLED":

- fundo escuro profundo
- contraste alto
- poucos acentos fortes
- elementos legiveis de longe
- pouca decoracao gratuita
- cara de wearable tecnico
- elementos que funcionem com dedo e glance rápido

### Paleta recomendada

- fundo principal: azul-preto quase preto
- EFIS/voo: azul tecnico + amarelo de referencia
- METEO: verde / ciano
- GPS: dourado / amber
- CONFIG: violeta escuro ou magenta tecnico

### Regras visuais

- preto real ou quase preto para economizar e valorizar a AMOLED
- branco puro apenas em referencias importantes
- amarelo para referencia de voo
- azul para "sky"
- marrom/terra para "ground"
- cinzas suaves para grades, bordas e dados secundarios

---

## Componentes de UI

### Home buttons

Os botoes da home sao a parte mais importante da proxima iteracao.
Melhorias futuras:

- icone simples
- titulo muito curto
- numero pequeno de status apenas se nao poluir
- espacamento generoso
- area de toque maxima

### Header

O header atual ja cria unidade visual.
Pode evoluir para:

- titulo do modulo
- subtitulo curto
- icone pequeno de status

Mas:

- evitar headers altos demais
- evitar duas linhas se isso comprimir o conteudo principal

Regra pratica:

- titulo grande em uma linha
- subtitulo opcional e curto
- se o subtitulo competir com o conteudo, remover

Para esta tela, o header ideal ocupa:

- `52` a `60 px` em portrait
- `44` a `52 px` em landscape

### Cards

Padrao ideal para `METEO`, `GPS` e `CONFIG`:

- borda fina colorida
- fundo escuro
- titulo curto
- valor grande
- unidade menor

Regra importante:

- se um card nao ficar legivel a primeira vista, ele esta complexo demais para essa tela

Regra objetiva para o firmware:

- card de `CONFIG` nao deve ter mais de `1 titulo + 2 linhas`
- card de operacao (`PLANE`, `METEO`, `COMMS`) nao deve ter mais de `1 valor principal + 1 linha secundaria`
- textos de debug devem sair da tela principal e ir para pagina separada ou web

### Horizonte artificial

Direcao visual inspirada em instrumento aeronautico:

- ceu azul
- solo marrom
- marcador central amarelo
- escala de bank branca
- linhas de pitch discretas

---

## UX e interacao

### Principios

- um toque deve sempre dar feedback
- menus devem exigir o minimo de leitura
- dados importantes devem estar no centro ou topo
- textos longos devem ser evitados na operacao normal
- uma transicao nunca pode quebrar a tela
- estabilidade vale mais que animacao

### Navegacao recomendada agora

- home com grid de modulos
- cada modulo com botao `Voltar ao menu`
- opcionalmente `BOOT` como `Back/Home`

Fluxo fechado recomendado:

1. `HOME` sempre como raiz
2. qualquer modulo secundario deve ter:
   - botao visual `Voltar`
   - suporte a `BOOT` como `Back/Home`
3. `CONFIG` nao deve ter scroll infinito de tudo ao mesmo tempo
4. acoes destrutivas ou sensiveis devem pedir confirmacao fora do rotulo do botao

### Navegacao recomendada depois

Quando o sistema estiver mais maduro:

- swipe horizontal entre telas principais
- barra inferior opcional
- pressao longa para abrir ferramentas de teste

Mas isso deve vir depois da fase de redesenho e estabilizacao.

---

## Prioridade de implementacao

### Fase 1: reset de UX

1. redesenhar `HOME`
2. criar fluxo de `Back/Home`
3. simplificar `CONFIG`
4. decidir estrategia de orientacao
5. remover sobreposicoes e travamentos
6. manter `portrait` como layout confiavel

Plano pratico da Fase 1:

#### 1. HOME

- manter 4 botoes grandes
- remover subtitulos longos
- usar so:
  - nome do modo
  - opcionalmente 1 status curto
- reservar a faixa inferior para:
  - bateria futura
  - rede futura
  - nunca para blocos longos

#### 2. CONFIG

Dividir em 3 blocos simples:

- `Tela`
  - orientacao
  - brilho depois
- `Armazenamento`
  - SD status
  - logger
  - limpar logs
- `Sensores`
  - resumo curto
  - `Atualizar`

Itens longos devem sair:

- raw I2C detalhado
- textos de nota muito extensos
- blocos de debug longos

#### 3. Navegacao

- um helper unico deve criar o botao `Voltar`
- `Voltar` sempre no mesmo lugar
- `BOOT` deve chamar o mesmo fluxo do botao visual

#### 4. Rotacao

- congelar `AUTO`
- deixar so `portrait` ativo por padrao
- so reativar `landscape/auto` depois que `rebuildUI()` estiver comprovadamente estavel

### Fase 2: validar o voo basico

1. confirmar `pitch` e `roll`
2. ajustar flags do IMU
3. melhorar suavidade do EFIS

### Fase 3: transformar em AHRS

1. integrar `BMM350`
2. adicionar Madgwick
3. obter heading estavel

### Fase 4: dados de voo

1. `BMP581`
2. altitude
3. variometro
4. velocidade GPS

### Fase 5: meteo

1. `BME688`
2. `LTR-390UV`
3. `MAX17048`
4. cards e graficos

### Fase 6: conectividade

1. GPS completo
2. LoRa
3. enriquecer o SD logger com GPS real
4. web config futuramente

---

## Decisoes de design para manter

- manter fundo escuro
- manter layout vertical otimizado para `280x456`
- evitar excesso de informacao em uma unica tela
- manter o `EFIS` como tela mais visual
- usar `METEO` e `COMMS` como telas de cards e indicadores
- deixar `CONFIG` mais utilitario
- priorizar portrait ate a rotacao ficar realmente estavel

---

## Riscos e limites atuais

- RAM continua alta, por volta de `87%`
- evitar multiplos canvases grandes
- evitar graficos pesados enquanto o EFIS nao estiver validado
- touch e IMU dividem I2C, entao a protecao por mutex deve permanecer

Hotspots diretos do `.ino` que explicam a situacao atual:

- `createHeader()`
  - header alto e com subtitulo em todas as telas
- `createNavButton()`
  - botoes ainda tentam acomodar titulo e subtitulo no mesmo bloco pequeno
- `createConfigScreen()`
  - mistura orientacao, SD, sensores, rede e acoes em pouco espaco
- `rebuildUI()`
  - recria todas as telas, o que aumenta risco de glitch
- `applyDisplayRotation()`
  - rotacao ainda depende de rebuild total
- `handleAutoRotation()`
  - pode reentrar em momentos ruins para uma UI ja apertada

---

## Arquivos relacionados

- sketch atual:
`StratosBrain_LVGL9/StratosBrain_LVGL9.ino`
- handoff tecnico:
  `C:\Esp32\CLAUDE\SESSION_HANDOFF.md`

---

## Proximo passo recomendado

A proxima sessao deve comecar por um redesign de interface, nao por novos sensores.

Sequencia ideal:

1. reconstruir `HOME` como launcher wearable
2. adicionar `Back/Home` consistente
3. testar se o `BOOT` pode virar retorno de sistema
4. simplificar `CONFIG`
5. congelar ou reduzir a rotacao automatica ate ficar confiavel
6. so depois continuar `PLANE`, `METEO` e `COMMS`

---

## Integracao do botao BOOT

Objetivo:

- usar o botao fisico como `Back/Home`
- sem atrapalhar bootloader, reset ou gravacao

Regras de integracao:

- usar `GPIO 0` apenas como entrada
- nao dirigir o pino
- habilitar leitura so depois do boot terminar
- ignorar pressionamentos nos primeiros `2s` a `3s`
- tratar apenas `short press`
- nao usar `long press` por enquanto

Fluxo recomendado:

- se estiver fora da `HOME`, `BOOT` volta para `HOME`
- se estiver na `HOME`, `BOOT` nao faz nada

Razao tecnica:

- o papel critico do `BOOT` acontece durante reset/power-on
- ler o pino em runtime e seguro, desde que o firmware nao mude o papel eletrico dele
- a protecao principal e evitar acao de UI cedo demais logo apos ligar
