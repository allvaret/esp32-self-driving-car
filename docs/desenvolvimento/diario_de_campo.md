# Diário de campo

Registro dos testes no carro (20 a 26/11/2026): o que cada teste mostrou, o que foi corrigido e a checklist da próxima sessão.
Formato dos logs: [`../como_ler_o_log.md`](../como_ler_o_log.md). Fluxo de execução: [`fluxo_de_execucao.md`](fluxo_de_execucao.md).

## 6. Achados do mapa anterior e o que aconteceu

| Achado | Resolução |
|---|---|
| A. `why` do `Controller` fantasma vazava para o parser do brain | `_parse` lê só `vision_state` (igualdade); `Controller` fora do caminho ao vivo; teste de regressão |
| B. `conf` nunca chegava | `conf = r` em [0,1] |
| C. Explorar não esperava a calibração | `ARM_DELAY_MS=4000`; a opinião sai `ok:0` enquanto calibra e o ESP32 a ignora (em `sensor` a visão nem é usada) |
| D. `edge_near` só alimentava o fantasma | segue só informativo (log) |
| E. `architecture.md` descrevia o que não rodava | reescrito |
| F. Reconexão não reenviava o mode | reenvia o mode do src atual (`conn_seq`), **sem armar** |
| `forwardAllowed` usava o ping bruto | usa a classe pela mediana + histórico |
| ESPERA infinita com obstáculo parado | fim da ESPERA ⇒ manobra completa; guarda com `gone` |
| `MockDriveLink.log` crescia sem limite no S9 | o `Runner` com mocks saiu do caminho ao vivo |
| Log 20/11 (1): com L = R o carro gira 100–200°/s | correção de rumo no ESP32 com trim aprendido (0.5). A troca de fiação da 0.4.1 (ENA = direita) estava **errada** e foi desfeita na 0.5.1 (ver abaixo) |
| Diagnóstico 21/11 (0.5.0): `dir` gira à esquerda (−300°/s), reto puxa à direita (+175°/s) e a correção não muda nada | o giroscópio estava certo e a fiação do firmware trocada: ENA = roda **esquerda** (0.5.1). A correção subia a roda que já era a rápida |
| Diagnóstico 21/11: `psi` ≈ 0 com 300°/s | o brain zerava o integrador a cada quadro sem goal; agora só nas transições |
| Diagnóstico 21/11: em v = 0,35 (duty 137) a roda direita quase não anda | faixa de duty **por roda** medida pelo teste por roda (visor sugere e aplica, NVS) |
| Modelo do carro de 21/11: ganhos 0,01/0,05 oscilam | `hold_kp`/`hold_ki` = 0,0015/0,015 e PWM dos motores em 10 bits |
| Campo 21/11 (0.5.1): faixa sugerida 64..89 / 106..131 e o reto **piorou** (+230..260°/s); com correção, 197°/s | o ajuste usava os degraus do teto da esquerda (~395°/s) e os degraus subindo de 300 ms mediam a aceleração. 0.5.2: degraus descendo de 600 ms com `dr` e `minimo`, ajuste sem o teto, aviso de roda que não obedece o PWM. A esquerda anda bem mais que a direita com o mesmo duty: motor/caixa ou PWM, a separar no próximo teste |
| Campo 22/11 (0.5.2): degraus descendo — roda esquerda ~374°/s em **qualquer** duty (45..220), direita obedece; GPIO19 medido em ~3 V parado (fio solto), ENA do módulo 0 V | **erro de código**: no core 3.x o `ledcAttach` escolhe o canal sozinho; sem o servo de direção, o ENA caía no canal 1, que divide o timer 0 com o servo (50 Hz/16 bits) e não recebia o PWM. 0.5.3: `ledcAttachChannel` com canais explícitos (0 = servo, 2/3 = motores). Provavelmente também era o "defeito" dos pinos 25/26/27. Código do servo de direção (`STEER_SERVO`, nunca usado) removido: o carro é só diferencial |
| Campo 22/11 (0.5.3): mesmo com os canais explícitos, a esquerda segue em ~380–400°/s de 220 a 45; no Explorar visão começa reto e vira à direita (L 0,15 × R 0,9, +146°/s) | o canal não era a (única) causa: **GPIO19 com defeito**. 0.5.4: `PIN_ENA 13` (passar o fio do ENA para o GPIO13) |
| Campo 23/11 (0.5.4): ENA no GPIO13 resolveu (esquerda para em ~120 como a direita). No Explorar visão o carro ficou PRESO num canto: sem eco + visão bloqueada nas 4 direções, ré fixa de 0,75 s e só ±30° de varredura | 0.6.0: **crédito de ré** (fila de trechos retos `{psi, v·s}`; a ré cega só volta pelo caminho percorrido) e **fuga** de 90° para fora do cone, sempre para o mesmo lado (não 180°, que vira vai-e-vem); PRESO só depois de 4 fugas. Simulado: beco abre em 60° → sai; beco fechado → 4 fugas ao mesmo lado → PRESO. Também: a faixa por roda ainda não foi aplicada (trim foi a +0,37) |
| Campo 26/11 (0.6.3, à tarde): com o fio do servo no GPIO4 o sensor **continuou sem girar** (relato; sem log da 0.6.3 na pasta). O carro seguiu sem bater "ao mínimo", mas quando **encosta** em algo que o sonar não vê continua mandando força às rodas | Dois pinos diferentes com o mesmo sintoma: o defeito provavelmente é do **servo** (ou do 5 V/GND dele), não do GPIO. 0.7.0: **batida** — rodas empurrando em reta e nada se mexendo (câmera do celular `m`, giro e sonar) por 1,5 s ⇒ para, ré (≥ 0,5 s) e desvio de ≥ 45° (90° na 2ª seguida). E o **vídeo da corrida** no S9 (`.mjpg` + índice ao lado do JSONL, `/logs`) |
| Campo 26/11 (fim da tarde, firmware 0.7.0 + **versão anterior do app**): `carrinho_2026-11-26T15-38/15-39`, `explorar_sensor_20261126_133728`. O **servo girou** (varreduras ±15/±30 no log), sem mudança de código. O carro bateu duas vezes e **não reagiu**; nenhum vídeo apareceu | O app no celular não foi reinstalado: os registros `f` do celular não têm `mov`/`vf` e o ESP32 recebeu `mov: -1` o tempo todo (`stl` e `bat` sempre 0). Sem o `m` da câmera a batida fica desligada por projeto, e o versão anterior do app não grava vídeo: **as duas funções ainda não foram testadas**. Servo intermitente: provável mau contato no fio de sinal/GND (o servo tem alimentação própria: o GND dela precisa estar ligado ao GND do ESP32). Arquivos da corrida renomeados: `corrida_<data>_<hora>_<fonte>` |
| Limpeza 26/11 | 0.6.3: sai todo o código do servo de direção (`STEER_MODE`, `PIN_STEER_SERVO`, `steer_trim_deg`, `steer_max_deg`), que nunca foi usado: o carro é só diferencial. Sem mudança de comportamento (sim e testes iguais) |
| Campo 26/11 (0.6.1): teste do servo = 80,6 cm nos 5 ângulos com `hz` 50 (o PWM do servo está ligado no firmware); multímetro no GPIO18: 0 V e sem continuidade até o servo. O carro andou sem bater (rodas custaram a partir uma vez). `ENA/ENB 0 Hz` com o carro parado: o `ledcReadFreq` do core devolve 0 com duty 0 (não é falha) | 0.6.2: **servo do sensor no GPIO4** (livre, não é pino de boot; o servo de direção, nunca usado, passou para o 14). O log diz `parado` em vez de 0 Hz; o sim imita o core (duty 0 → 0 Hz) |
| Campo 26/11 (0.6.0): o crédito de ré funcionou (`cred` 1,25 s → ré → 0, depois gira direto), mas o carro quase só fazia "sem saída: fuga". **O sensor não gira desde 23/11**: em toda varredura de 23/11 e 26/11 as 4 direções leem a **mesma** distância (ex. 26,4 / 26,4 / 26,4 / 26,4); em 20–21/11 (0.5.2) variavam (46 / 47 / 65 / 67). O firmware manda o servo (`aim` −15/−30/+15/+30 no diário). No Explorar visão de 10:34 a câmera não calibrou: o celular estava desconectado no clique e o `calibrar` se perdeu (o ESP32 reenvia só o `goal`); na fonte sensor o visor nunca mandava `calibrar` | 0.6.1: o código do servo é o mesmo da 0.5.2, exceto o `ledcAttachChannel` da 0.5.3. Pelo fonte do core 3.3.12, o timer é escolhido por frequência/resolução nos dois casos (não era a causa da roda), mas por segurança **volta o `ledcAttach` da 0.5.2**. Novo: cada cliente recebe `pwm: servo GPIO18 50 Hz…` (frequência real) e o visor tem **Servo do sensor** (−60..+60, sem armar), que diz se o sensor girou. O sim agora lê o ângulo do **PWM** do pino do servo (`--servo-dead`, `--servo-nopwm`). Suspeita principal: o **fio do GPIO18**, vizinho do 19 que foi mexido em 22/11. Brain: goal `explorar` novo com o carro parado reaprende o chão (as duas fontes), sem depender do `calibrar` |
| Campo 21/11 (0.5.1): a correção sobe a roda certa, sem oscilar, e o trim aprende para o lado certo (−0,8) | ok; falta autoridade enquanto as rodas estiverem tão diferentes |
| Log 20/11 (2): correção de rumo não atuava | saiu do brain; roda no ESP32 nos dois modos, desde a arrancada, autoridade ±2,0 (até `hold_pwm_max`), trim persistente |
| Log 20/11 (3): ciclo avoid → ESPERA de 35 s com 86 cm válidos | sensor refuta a visão (brain e ESP32); fim da ESPERA com a condição persistente ⇒ manobra completa |
| Log 20/11 (4): curva passava do alvo (70° para 30°) | curva era espelhada (fiação) e sem desaceleração; agora desacelera perto do alvo e reporta `turn` |

---

## 7. Checklist de teste de campo (na ordem)

Antes de tudo: gravar o firmware **0.7.0** e **recompilar e instalar o app** pelo Android Studio (Run ▶ com o S9 no
cabo). Só regravar o ESP32 não basta: o Python do celular vai dentro do app, e é ele que manda o movimento da câmera `m`
(que detecta a batida) e grava o vídeo. No teste de 26/11 o app era o antigo. **Confira antes de andar**: o log do celular
novo chama `corrida_<data>_<hora>_<fonte>.jsonl` (não `explorar_...`), e o chip do visor mostra `camera: mov N`. Fio do servo no **GPIO4**, ENA no **GPIO13**. Se algo piorar: firmware 0.6.3
(commit 42c0751). App aberto e conectado (o yaw e o `m` vêm dele).
No visor, **Gravar log** ligado o tempo todo. Ao abrir o visor, o log mostra `car_firmware 0.7.0 | pwm: servo GPIO4 50 Hz, ENA GPIO13 parado, ENB GPIO32 parado` (andando: 1000 Hz); `ERRO sem PWM` = pare e mande o log.
O chip de fonte mostra `camera: mov N`: carro parado ~1, andando bem mais. `sem movimento medido` = o celular não está
mandando o `m` (versão anterior do app?) e **não há detecção de batida**.

**B. Batida (0.7.0, no chão, Explorar: sensor).** Deixe o carro ir contra algo que o sonar **não vê** (pé de cadeira,
caixa baixa, ou segure o para-choque com a mão). Em ~1,5 s com as rodas empurrando: log `batida: rodas empurrando e nada
se mexe ha 1500 ms (camera m=…)`, PARADA → **RE** → varredura → curva de ≥ 45° (`why: batida: desvia -45`). Batendo de
novo logo em seguida: 90°. Anote no log do celular o `mov` com o carro **parado empurrando** e **andando devagar**:
- não detectou e o `mov` parado ficou acima de 3 (tremor das rodas): suba `stall_cam_max` (cfg) para entre os dois valores;
- batida **falsa** andando (chão liso, pouca textura): `mov` andando abaixo de 3: baixe `stall_cam_max` ou suba `stall_ms`.
Repita no Explorar: visão (log `ESP32 assumiu: batida` e depois `celular retomou o controle`).

**V. Vídeo da corrida.** Depois de uma corrida, abra o link de logs do visor: a corrida aparece com **log**, **vídeo** e
**índice** (tamanhos) e **assistir** toca o vídeo no navegador; o stream do visor não pode travar enquanto baixa. No PC:
copie os arquivos para `docs/logs/` e rode `python -m tools.video_to_mp4 docs/logs` (converte todas as corridas novas em
`.mp4`, ao lado), depois `python -m replay.replay_main docs/logs/corrida_<...>.mp4`. Feche o app no meio de uma
corrida uma vez: o vídeo até ali tem que abrir (ver `docs/como_ler_o_log.md`, "Vídeo da corrida").

**00. Servo do sensor (desarmado, antes de tudo).** Mão (ou caixa) a ~30 cm só do lado ESQUERDO do carro, botão
**Servo do sensor**. Olhe o sensor: tem que ir à esquerda, centro, direita e voltar (~5 s). O visor mostra a distância
por ângulo: −60/−30 perto (a mão), 0/+30/+60 longe ou "sem eco". "Distâncias IGUAIS" = o sensor não girou: com o
carro desligado, confira o fio de **sinal** do servo no **GPIO4** (o GPIO18 morreu em 26/11), o **5 V** e o **GND** do servo
(servo sem 5 V não gira e às vezes treme). No GPIO4 ele falhou numa corrida e girou na seguinte (26/11), sem mudança
de código: mau contato. O servo tem fonte própria, então o **GND dessa fonte tem que estar ligado ao GND do ESP32**
(sem GND comum o sinal não tem referência e o servo falha "às vezes"). Refaça ou troque o fio de sinal e o de GND e
mexa neles com o teste rodando: se o sensor parar quando você mexe, é ali. Sem o servo, o carro ainda anda, mas só olha para a frente (toda varredura
vira "sem saída" → fuga de 90°).

**0. Diagnóstico pelo visor (carro no chão, ~1,5 m livre em volta).** Seção **Diagnóstico** do `http://192.168.4.1`:
   1. **Armar parado (4 s)**: o estado vira `DIAG / parado` e o carro **não** anda. Chip `giro ok` aceso.
   2. **Rodas: padrão** e **Zerar trim** (a faixa 64..89 / 106..131 e o trim −0,775 de 21/11 vieram de medidas erradas).
   3. **Roda esq. (degraus)**: só a esquerda gira, começando forte e **diminuindo** (~5,4 s); o carro gira sobre a roda
      parada. Responda **qual roda girou**. Olhe e escute: a roda tem que **desacelerar** a cada degrau e parar perto do
      fim. Anote o `minimo`. Se o visor disser que a roda **gira igual em qualquer duty**, o PWM não chega a ela (fio do
      EN/placa): pare aqui. Se a roda **parar** num duty bem mais alto que a outra, é atrito/motor (caixa de redução).
   4. **Roda dir. (degraus)**: o espelho (roda DIREITA, carro gira para a ESQUERDA). Se uma roda **não gira** nem no
      duty 220, é mecânico (caixa/roda): pare aqui. Linhas "(não assentou)" na tabela: repita o teste.
   5. Depois dos dois, o visor mostra a **faixa sugerida** de cada roda: **Aplicar faixa das rodas** (grava na NVS; o log
      mostra `rodas (NVS): esq A..B dir C..D`).
   6. **Girar à direita 1 s** e **Girar à esquerda 1 s**: responda para onde virou; veredito esperado "OK" (`rate_med`
      positivo no `dir`).
   7. **Reto 2 s (sem correção)**: com a faixa aplicada, `rate_med` deve ficar bem abaixo dos +175°/s de 21/11
      (idealmente < 30). É o carro cru com as rodas equalizadas.
   8. **Reto 2 s com correção** (3–4 vezes): `rate_med` < 10°/s, `hc` longe de ±2 e `trim` mudando pouco. Se balançar
      de um lado a outro (sinal do giro trocando a cada `state`), baixe `hold_kp`. Se a correção piorar, volte ao 3.
   9. **PARAR TUDO**: log `trim do rumo salvo: X`. Desligue e ligue o ESP32: o mesmo `trim` no `state` e a reta (passo 7)
      continua boa sem reaplicar a faixa (ela está na NVS).
   Se algo der errado com a faixa: **Rodas: padrão** volta à faixa única 130..150.

1. **Visor e estado (carro parado, rodas no ar).** Conectar no CARRINHO, abrir `http://192.168.4.1`. Observar: `state` aparece, botões **Explorar: sensor / Explorar: visão / PARAR TUDO**, link de logs fica "baixar logs gravados no celular" depois que o S9 conectar. A faixa do app mostra `conectado | goal=… fonte=…`.
2. **PARAR TUDO e reset.** Clicar Explorar: sensor, esperar armar (4 s, LED fixo), clicar PARAR TUDO. Observar: "desarmado", `fonte` volta a SENSOR, goal "parar", motores 0. Repetir desarmando pelo BOOT. (Opcional: reiniciar o ESP32 depois de parar e ver que **não** retoma.)
3. **Sensor puro, rodas no ar.** Explorar: sensor; passar a mão a 25 cm e depois a 45 cm à frente. Observar: `cls=valida`, PARADA → OLHA (4 posições, radar acende) → `decide` com notas por ângulo e `ângulo: N°` no chip; RE → CURVA. Conferir no log `[servo] notas -30°=… -15°=… 15°=… 30°=… -> curva N°`.
4. **Sem eco suspeito.** Aproximar uma superfície macia/inclinada (toalha, papelão em diagonal) a ~40 cm e afastar até sumir o eco. Observar: `cls=suspeito`, o carro **para e varre sem ré** (`why: sem eco suspeito`). Chão aberto sem nada deve dar `cls=livre`.
5. **Giroscópio.** Girar o carro à mão para a DIREITA: `giro ok`/`psi` positivos na faixa do app. Se vier negativo, inverter o sinal em `YawSensor.kt`.
6. **No chão, só sensor.** Corredor livre, `Explorar: sensor`. Observar: anda em FRENTE **reto** (`hold` = 2, giro < 10°/s), desvia de uma caixa, **vira para o ângulo escolhido** e o visor mostra `[curva] alvo … girou … erro …` com |erro| < 5°. Passou do alvo: suba `yaw_lead_ms`; empacou no fim: suba `turn_k_min`.
7. **Obstáculo parado.** Pôr uma caixa fixa na frente e deixar. Observar: 2 manobras, 1 ESPERA, e depois manobra completa (**não** fica só em ESPERA).
8. **No chão, visão.** `Explorar: visão` com o carro parado apontado para chão livre (o chão é reaprendido; o chip mostra `visao ok` só depois). Observar: anda por `drive` com `s = 0` e o ESP32 segura a reta (`hold` = 2), OBSTACULO da visão ⇒ `avoid` ⇒ "ESP32 no comando", sugestão de ângulo aceita ou vetada (`scan/decide`). Aproximar a mão a < 30 cm sem a visão ver: o ESP32 **veta**.
9. **Visão refutada.** Na fonte visão, um tapete/mancha de cor diferente no chão com nada a menos de 60 cm: a visão diz `OBSTACULO`, o visor mostra "visão refutada pelo sensor (X cm)", o carro segue devagar e **não** entra em DESVIO/ESPERA. Uma caixa a 45 cm com a visão vendo: manobra (e, se repetir, 1 ESPERA seguida de manobra, não ciclo).
10. **Logs.** Depois da corrida, abrir o link de logs no visor, baixar o `.jsonl` e conferir `f` (frames), `tx` (`vis`, `yaw`, `drive`, `avoid`) e `rx` (`state` com `hold/hc/trim`, `scan`, `range`, `turn`, `diag`). Ver `docs/como_ler_o_log.md`.

**Parâmetros a validar** (pontos de partida; ver [`../parametros.md`](../parametros.md)):
- **`hold_*` / `trim_*`**: pelo passo 0 (reto com e sem correção). Ver `docs/parametros_a_calibrar.md`.
- **`turn_dps`** (90): com `yaw` desligado (tampar o giroscópio/desabilitar o sensor), comandar uma curva e medir os graus/s; só vale sem yaw.
- **`vis_free_min`** (50): ler `sec[2]` no log em chão livre × parede × objeto baixo e escolher o corte.
- **`suspect_ms`** (1500): duração típica do "sem eco" depois de um eco válido (parede em diagonal, sofá).
- Depois: `yaw_lead_ms` (se passa do ângulo, aumentar), `vis_fresh_ms`/`yaw_fresh_ms` (conferir o atraso real no log), `reverse_ms`, `pwm_min/max`, `trim_*`.
