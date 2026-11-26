# Como ler o log do carrinho

O S9 grava um arquivo por corrida: `corrida_<data>_<hora>_<fonte>.jsonl` (até a 0.7.0: `explorar_<fonte>_<data>_<hora>.jsonl`).
Cada linha é um registro em JSON. O campo **`k`** diz o tipo; o campo **`w`** é a hora em que foi gravado
(milissegundos desde 1970; subtraia o primeiro `w` para ter "segundos desde o início").

| `k` | O que é |
|---|---|
| `f` | um quadro processado pela visão |
| `tx` | mensagem que o S9 **enviou** ao ESP32 (dentro de `m`) |
| `rx` | mensagem que o S9 **recebeu** do ESP32 (dentro de `m`) |

> O log baixado do visor (`carrinho_*.jsonl`) tem só as mensagens do ESP32, sem `k`/`w`; a hora fica em `ts`.

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
| `v`, `s` | velocidade (0–1) e curva (−1 esquerda … +1 direita) que o celular pediu |
| `ms` | tempo para processar o quadro (bom: < 33 ms) |

## `tx`: o que o S9 envia

| `m.t` | Campos |
|---|---|
| `drive` | `v` velocidade, `s` curva (só na fonte visão) |
| `vis` | `ok` (tem opinião?), `s` = os 5 setores de `sec` |
| `yaw` | giroscópio: `psi` = graus girados desde o início (+ = direita), `r` = graus/s agora |
| `avoid` | pede desvio; `deg` = direção que a visão sugere |
| `mode` | `auto` (ESP dirige) ou `phone` (celular dirige) |
| `think` | texto para o visor (`msg`, `state`, `conf`; `viz` = endereço do vídeo) |

## `rx`: o que o ESP32 envia

**`state`** (a cada 0,5 s):

| Campo | Significado |
|---|---|
| `mode` / `src` | quem dirige: `auto`/`sensor` ou `phone`/`visao` |
| `armed` | motores liberados |
| `esp` | `true` = o ESP assumiu o comando no modo visão (desvio ou celular mudo) |
| `st` | estado do ESP: `FRENTE`, `VARREDURA` (devagar, algo no meio do caminho), `DESVIO`; `-` = o celular está dirigindo |
| `sub` | etapa do desvio: `PARADA`, `OLHA` (servo varrendo), `RE`, `CURVA`, `ESPERA` (pausa anti-repetição), `PRESO` |
| `c` | distância à frente em cm (mediana); **−1 = sem eco** |
| `deg` | para onde o servo do sensor aponta (graus, + = direita) |
| `cls` | como o ESP leu o "sem eco": `valida` (há eco), `livre` (sem eco e nada indica obstáculo), `suspeito` (sem eco logo depois de algo perto) |
| `ang` | ângulo escolhido na última varredura |
| `vis`, `yaw` | 1 = está recebendo visão / giroscópio frescos |
| `L`, `R` | força pedida a cada motor (−1 a 1), depois da rampa e antes do trim |
| `why` | motivo da decisão atual |
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

**`log`** (`msg`: "ARMADO", "ESP32 assumiu…", "celular retomou…"), **`goal`** (`g`, `src`), **`calibrar`**, **`think`** (repassado).

---

## Leitura rápida em 3 perguntas
1. **O que a visão achou?** Siga `vs`, `r` e `sec` nos `f`.
2. **O que o ESP fez?** Siga `st`/`sub`/`why` nos `state` e os `decide`.
3. **O carro obedeceu?** Compare `L`/`R` com `yaw.r`: com `L = R` (reto), o giro deveria ficar perto de 0.

Atalho: `python tools/log_resumo.py <arquivo> --giro` monta essa linha do tempo.
