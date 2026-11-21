# Como ler o log do carrinho

O S9 grava um arquivo por corrida: `explorar_<fonte>_<data>_<hora>.jsonl`.
Cada linha é um registro em JSON. O campo **`k`** diz o tipo; o campo **`w`** é a hora em que foi gravado
(milissegundos desde 1970; subtraia o primeiro `w` para ter "segundos desde o início").

| `k` | O que é |
|---|---|
| `f` | um quadro processado pela visão |
| `tx` | mensagem que o S9 **enviou** ao ESP32 (dentro de `m`) |
| `rx` | mensagem que o S9 **recebeu** do ESP32 (dentro de `m`) |

> O log baixado do visor (`carrinho_*.jsonl`) tem só as mensagens do ESP32, sem `k`/`w`; a hora fica em `ts`.

> **Logs gravados com o firmware 0.4.1 e 0.5.0** (21/11) têm `L` e `R` **trocados** e as curvas ao contrário
> (`dir` virava à esquerda): esses firmwares mandavam o `L` para o ENB, que é a roda **direita**. Até a 0.4.0 (20/11)
> e desde a 0.5.1, `L` = roda esquerda = ENA (OUT1/OUT2). Ver `docs/protocolo_ws.md` (fiação).

---

## `f`: quadro da visão

| Campo | Significado |
|---|---|
| `t`, `n` | segundos desde que a visão começou / número do quadro |
| `ph` | fase: `SETTLE` (1 s esperando a exposição da câmera) → `WARMUP` (aprendendo a cor do chão) → `RUN` (decidindo) |
| `vs` | estado da visão: `LIVRE`, `INCERTO` ou `OBSTACULO` |
| `r` | confiança: o chão à frente comparado com o chão aprendido. ~1 = igual; abaixo de 0,2 = não parece chão |
| `sec` | 5 números (0–100), um por direção: −30°, −15°, 0°, +15°, +30°. Quanto da imagem, subindo a partir da base, é chão livre. 100 = chão até o fim da área vista (~130 cm); 0 = nenhum chão |
| `ok` | 1 = a visão tem opinião; 0 = ainda calibrando ou erro |
| `e.h` | altura da primeira linha horizontal longa (rodapé): 0 = base da imagem (perto), 1 = topo (**1,0 significa "não achou"**) |
| `e.c` | em quantas faixas da imagem essa linha apareceu (0–1) |
| `e.n` | 1 = rodapé perto |
| `g`, `src` | motivação (`explorar`) e fonte escolhida (`sensor` ou `visao`) |
| `v`, `s` | velocidade (0–1) e curva que o celular pediu. Desde o firmware 0.5 o `s` é sempre 0 (quem corrige o rumo é o ESP32) |
| `ms` | tempo para processar o quadro (bom: < 33 ms) |

## `tx`: o que o S9 envia

| `m.t` | Campos |
|---|---|
| `drive` | `v` velocidade, `s` = 0 (só na fonte visão). `v` = 0,15 com `OBSTACULO` refutado pelo sensor |
| `vis` | `ok` (tem opinião?), `s` = os 5 setores de `sec` |
| `yaw` | giroscópio: `psi` = graus girados desde o início (+ = direita), `r` = graus/s agora |
| `avoid` | pede desvio; `deg` = direção que a visão sugere. **Não sai** se o sensor refuta a visão |
| `mode` | `auto` (ESP dirige) ou `phone` (celular dirige) |
| `think` | texto para o visor (`msg`, `state`, `conf`; `viz` = endereço do vídeo). `msg` = "visão refutada pelo sensor (86 cm)" quando a visão diz `OBSTACULO` e o sensor lê ≥ 60 cm válidos |

## `rx`: o que o ESP32 envia

**`state`** (a cada 0,5 s):

| Campo | Significado |
|---|---|
| `mode` / `src` | quem dirige: `auto`/`sensor` ou `phone`/`visao` |
| `armed` | motores liberados |
| `esp` | `true` = o ESP assumiu o comando no modo visão (desvio ou celular mudo) |
| `st` | estado do ESP: `FRENTE`, `VARREDURA` (devagar, algo no meio do caminho), `DESVIO`, `DIAG` (diagnóstico); `-` = o celular está dirigindo |
| `sub` | etapa do desvio: `PARADA`, `OLHA` (servo varrendo), `RE`, `CURVA`, `ESPERA` (pausa anti-repetição), `PRESO`. Em `DIAG`: o teste (`dir`, `esq`, `reto`, `reto_corr`) ou `parado` |
| `c` | distância à frente em cm (mediana); **−1 = sem eco** |
| `deg` | para onde o servo do sensor aponta (graus, + = direita) |
| `cls` | como o ESP leu o "sem eco": `valida` (há eco), `livre` (sem eco e nada indica obstáculo), `suspeito` (sem eco logo depois de algo perto) |
| `ang` | ângulo escolhido na última varredura |
| `vis`, `yaw` | 1 = está recebendo visão / giroscópio frescos |
| `L`, `R` | comando da roda **esquerda** / **direita** depois da rampa, do trim fixo e da correção de rumo. Na reta com correção a roda lenta pode passar de 1 (até `hold_pwm_max` de duty). No teste por roda: duty/255 |
| `hold` | correção de rumo: `0` = fora de reta (parado, curva, ré), `1` = só o trim aprendido (sem yaw fresco), `2` = corrigindo com o giroscópio |
| `hc` | correção aplicada agora (unidades de roda; > 0 = mais força na esquerda). Colada em ±2,0 = sem autoridade |
| `trim` | trim aprendido (persiste entre trechos e na NVS). Deve mudar pouco de uma corrida para a outra |
| `why` | motivo da decisão atual (ex.: "visao refutada pelo sensor (86 cm)") |
| `up` | segundos desde que o ESP ligou (se diminuir, ele reiniciou) |
| `rst` | motivo do último reinício: `LIGOU` normal, `WDT` travou e o watchdog reiniciou, `BROWNOUT` queda de tensão |
| `rest` | quantas vezes retomou sozinho após reinício |

**`range`**: cada leitura do sensor. `deg` = ângulo, `cm` = distância (`null` = sem eco).

**`scan`**: diário da varredura. `n` = sequência, `boot` = nº do boot, `ms` = relógio do ESP.
- `aim`: o servo vai para `deg`.
- `olha`: leitura em `deg`; `got` 1 = leu, 0 = tempo esgotado; `cm`.
- `decide`: `d` = ângulos olhados, `sc` = nota de cada um, `ang` = escolhido, `x` 1 = sem saída.
  Nota: distância do eco em cm; **200** = sem eco e a visão diz livre; **15** = sem eco e a visão diz bloqueado;
  **45** = sem eco e sem opinião da visão; **−1** = não olhou (sugestão aceita direto).

**`turn`** (300 ms depois de cada curva do desvio, contando o giro por inércia):

| Campo | Significado |
|---|---|
| `alvo` | ângulo pedido (graus, + = direita) |
| `girou` | quanto girou de fato pelo giroscópio; `null` = sem yaw (curva pelo tempo, `turn_dps`) |
| `erro` | `girou − alvo`. **Mesmo sinal do `alvo` = passou** (aumente `yaw_lead_ms`); sinal oposto = ficou curta (diminua). Bom: \|erro\| < 5 |
| `ms` | duração da curva |
| `yaw` | 1 = a curva foi feita pelo giroscópio |

**`diag`** (botões do diagnóstico no visor):

| Campo | Significado |
|---|---|
| `teste` | `dir` / `esq` (gira no lugar 1 s, `v = 0`, `s = ±1`), `reto` (2 s, sem correção nem trim), `reto_corr` (2 s, com correção), `roda_esq` / `roda_dir` (ver abaixo), `zerar_trim` |
| `L`, `R` | comando médio de cada roda na 2ª metade do teste |
| `psi` | graus girados no teste (+ = direita); `null` = sem yaw do celular |
| `rate_med` | giro mediano na 2ª metade (graus/s). No `reto` sem correção, é o quanto o carro puxa; no `dir`, ≈ giro no lugar |
| `n`, `ms` | amostras de yaw e duração |
| `trim` | trim aprendido no momento do teste |
| `abortado` | motivo, se parou antes (hard stop, desarmado, parar) |
| `erro` | o teste foi recusado (ex.: "arme antes") |

`roda_esq` / `roda_dir` (uma roda só, para frente, duty bruto em degraus **descendo**; o carro gira sobre a roda parada):

| Campo | Significado |
|---|---|
| `duty` | os degraus de PWM (0–255) aplicados à roda, de 220 a 45, 600 ms cada (o 1º com 1,2 s) |
| `rate` | giro mediano na 2ª metade de cada degrau (°/s). Mede a velocidade da roda: `roda_esq` deve dar > 0 (direita), `roda_dir` < 0 |
| `dr` | quanto o giro ainda mudava na janela de medida. Grande (> 15% do `rate`) = o degrau não assentou |
| `minimo` | último degrau ainda com \|giro\| ≥ 15°/s = menor duty que mantém a roda girando; `null` = não girou |

Até a 0.5.1 os degraus **subiam** (110..230, 300 ms) e o campo era `partida`. Esses degraus mediam a aceleração do carro:
em 21/11 a esquerda deu 59 → 143 → 228 → 352 e travou em ~395°/s a partir de 170, formato de aceleração até a velocidade
de topo. O visor também avisa quando o `rate` quase não cai do duty mais alto ao mais baixo: o PWM não controla a roda.

Com o `psi`/`rate_med`: até 21/11 o `psi` do diagnóstico vinha quase 0 (o brain zerava o integrador a cada quadro sem goal);
use o `rate_med` nesses logs.

**`log`** (`msg`): "ARMADO", "ESP32 assumiu…", "celular retomou…", **"avoid recusado: visao refutada pelo sensor (X cm)"**
(o ESP recusou um `avoid` porque o sensor lê o caminho livre), **"trim do rumo salvo: X"** (ao desarmar).
**`goal`** (`g`, `src`), **`calibrar`**, **`think`** (repassado).

---

## Leitura rápida em 4 perguntas
1. **O que a visão achou?** Siga `vs`, `r` e `sec` nos `f`.
2. **O que o ESP fez?** Siga `st`/`sub`/`why` nos `state` e os `decide`. Um `OBSTACULO` refutado aparece no `think` e no `log`.
3. **O carro andou reto?** Na reta (`hold` = 2), compare `yaw.r` com 0: deve ficar abaixo de ~10°/s depois de 1 s; `hc` estável e longe de ±2,0.
4. **As curvas acertaram?** Siga os `turn`: `erro` pequeno e sem viés de sinal.

Atalho: `python tools/log_resumo.py <arquivo> --giro` monta a linha do tempo.
