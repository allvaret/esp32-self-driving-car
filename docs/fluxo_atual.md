# Fluxo atual do projeto (Caminho 1: o ESP32 é o único executor de manobras)

Este documento substitui o mapa anterior (que descrevia o `Controller` do celular rodando "em sombra").
Tudo abaixo está implementado e testado em simulação; **nada foi gravado no ESP32 nem rodado no carro**.

---

## 1. Cópias Python do Android × raiz

- `android/app/src/main/python/` não está no git; a task `syncPython` (`android/app/build.gradle.kts`) copia da raiz a cada build:
  `config.py`, `vision/*.py`, `control/*.py`, `tools/{__init__,bench_s9,android_entry,ws_link,flight_log}.py`, `brain.py`, `viz.py`, `main.py`
  e `replay/{__init__,offline}.py` (**só** para o app de bancada, `MainActivity → android_entry.run_main`; o caminho ao vivo não importa `replay`).
- `replay_main.py`, `sim/`, `tests/` e as outras `tools/` ficam fora do app.
- `tests/test_live_path.py` copia exatamente o conjunto do `syncPython` para uma pasta temporária e roda ali o caminho ao vivo, a bancada
  (`run_main`/`bench`) e confere que `main`, `brain`, `viz` e `ws_link` **não importam `control.controller`, `control.mocks` nem `replay`**.

---

## 2. Caminho de execução no S9 (frame → ESP32)

Thread única `exec` da `LiveActivity` para câmera + Python.

| # | Onde | Entra | Sai |
|---|---|---|---|
| 0 | `onResume` → `BrainBridge.start(filesDir/logs)` → `brain.set_log_dir` + `brain.start` | — | `WsLink` (reconexão) + `viz` (MJPEG :8080, `/logs`) + `FlightLog` |
| 1 | CameraX → `LiveActivity.analyze` | `ImageProxy` RGBA 640×360 | bytes |
| 2 | `StreamRunner.process_rgba_frame` (`main.py`); **sem `log_path`** (o log é o JSONL do brain) | bytes, rotação | JSON string |
| 3 | `frame_from_rgba` → `VisionRunner.feed`: SETTLE (1 s) → WARMUP (11 frames) → RUN: `floor_mask` → `free_space_vector` → **5 setores** → `floor_prob`/`coverage_row` → `DecisionMaker` → `edge_probe` | BGR "em pé" | `{phase, ok, vision_state, r, sectors[5], edge_h_cen, edge_cov, edge_near, ...}` (`ok=False` em SETTLE/WARMUP/ERROR) |
| 4 | `BrainBridge.step(json)` → `brain.step` | o JSON | opinião ao ESP32 |
| 5 | `brain._parse`: lê **`vision_state` por igualdade exata** (nunca procura palavra em `why`); `conf = r` em [0,1]; setores | | |
| 6 | `brain.step` (≈10 Hz) → `{"t":"vis","ok":0\|1,"s":[5]}`; se `goal=explorar`: manda `{"t":"mode"}` (sensor→auto, visao→phone) **só quando goal ou src muda** (ou o WS reconecta); **nunca arma** | | |
| 7 | `YawSensor` → `brain.on_gyro` (~50 Hz) → `YawIntegrator` (psi **sem limite**, viés estimado parado) → `{"t":"yaw","psi","r"}` a ~20 Hz. **O rumo em reta é corrigido no ESP32** (`holdStep`); o `HeadingHold` saiu do caminho | | |
| 8 | só `src=visao`: `{"t":"drive","v","s":0}` a cada frame (v pela visão); `OBSTACULO` + armado → `{"t":"avoid","deg":<setor mais livre entre -30,-15,+15,+30>}` (máx. 1 a cada 2 s), **exceto** se a leitura central é válida, fresca e ≥ `d_clear` (60 cm): visão refutada, `v = V_SLOW`, sem avoid, `think` "visão refutada pelo sensor (X cm)" | | |
| 9 | `brain` envia `think` **só da visão** (estado, conf, setores). As decisões aparecem no `state` do ESP32 | | |
| 10 | ESP32: `handleJson` → FSM (`autoStep`/`phoneStep`) → `driveApply` | | PWM |
| 11 | Toda mensagem enviada e recebida e cada frame vão para o **log JSONL** (fila + thread, flush 1 s); nada é transmitido durante a corrida | | `filesDir/logs/explorar_<src>_<data>.jsonl` |

Quem envia o quê:
- `{"t":"goal"}`: só o visor (**Explorar: sensor**, **Explorar: visão**, **PARAR TUDO**). O ESP32 aplica (modo pela fonte, arma em 4 s) e repassa a todos.
- `{"t":"mode"}`: o brain (por goal/src) e o próprio ESP32 ao receber o goal (idempotente). Os botões de modo do visor foram removidos.
- `{"t":"calibrar"}`: o visor, ao clicar em **Explorar: visão**; o brain faz `recalibrate()` e a opinião fica `ok:0` (o ESP32 ignora) até o chão ser reaprendido. `ARM_DELAY_MS = 4000` cobre isso.
- `Controller`, mocks e `edge_near`: o `Controller` não roda mais ao vivo; `edge_near` ainda é calculado e vai para o log, mas nada o usa para decidir.

---

## 3. O que existe e não está no caminho ao vivo

| Módulo | Situação |
|---|---|
| `control/controller.py` | **Congelado** (referência v1). Usado por `replay/`, `tools/bench_s9.py` e testes. |
| `control/mocks.py` (`MockRangeHead`, `MockDriveLink`, `Scenario`) | Só teste/replay/bancada. |
| `control/hal.py` `LinkRangeHead`/`LinkDrive`/`SerialLink`/`MemoryLink` | Testes, `sim/check_firmware.py`, `tools/serial_check.py`. Poderiam alimentar um `Runner` com sensor real, mas o Caminho 1 faz o ESP32 ler o sensor. |
| `vision/ground.free_space_vector` | **Agora no caminho** (setores). |
| `vision/flow_probe.py` | Removido (cópia idêntica de `edge_probe.py`). |
| `vision/edge_probe` | No caminho, só informativo (log). Candidato a virar mais uma opinião (`edge_near` → setor 0 baixo) se o campo mostrar falha em parede de cor parecida. |

---

## 4. Quem decide, por fonte (goal `explorar`)

| Fonte (`src`) | Modo ESP32 | Quem decide o movimento | O que o celular manda | O que o visor mostra |
|---|---|---|---|---|
| **sensor** | `auto` | **ESP32** (`autoStep`): FRENTE → VARREDURA → DESVIO, só com o sensor (ignora `vis`) | `vis` e `yaw` (o `yaw` serve às curvas); sem `drive`/`avoid` | `state` do ESP32 (fonte SENSOR, FSM, ângulo escolhido, classe do eco) + `think` da visão |
| **visão** | `phone` | **celular** dirige em marcha normal (v pela visão, `s = 0`); **ESP32** corrige o rumo, veta e manobra: mediana < `d_stop_cm` (30), **sem eco suspeito**, `avoid` da visão (recusado se o sensor lê ≥ `d_clear` válido), ou celular mudo por 500 ms (watchdog → `autoStep`) | `drive` por frame, `vis`, `yaw`, `avoid` | `state` ("celular dirige" ou "ESP32 no comando: DESVIO/…"), ângulo, `vis ok`/`giro ok` + `think` da visão |
| **parar** (goal `parar`, `arm false`, BOOT) | volta ao padrão (`auto`) | **ninguém**: reset completo (desarmado, FSM zerada, RTC `armed/mode/goal` limpos) | o brain zera goal/src, rumo e integrador | "desarmado" |

Reflexos válidos em ambas as fontes: só avança com leitura central **fresca**, classe **não suspeita** e mediana ≥ `hard_stop_cm` (20); ré ≤ `max_rev_ms`; rampa; servo parado antes de partir.
**Rumo em reta nas duas fontes** (firmware 0.5, `holdStep`): em todo trecho com `v > 0` e `|s| < hold_s_eps` (FRENTE, VARREDURA, `drive` reto), `s_corr = −(hold_kp·r + hold_ki·psi_trecho) + trim`; trim aprendido desde o 1º ciclo, PI após ≤ 100 ms com yaw fresco (sem yaw: só o trim); sobe a roda lenta, desce a rápida só se a lenta saturar; trim persiste entre trechos e na NVS. Detalhes em `docs/protocolo_ws.md`.

---

## 5. Firmware: "sem eco" e a guarda

**Classe do centro** (`centerClass`, sempre pela **mediana** `centerCm`, nunca pelo ping bruto):

| Classe | Condição |
|---|---|
| `valida` | há eco (`centerCm ≥ 0`) |
| `livre` | sem eco **e** (modo phone com `vis` fresca e setor 0 ≥ `vis_free_min`) **ou** (sem visão e nenhuma leitura < `d_clear` nos últimos `suspect_ms`) |
| `suspeito` | sem eco **e** (modo phone com `vis` fresca e setor 0 < `vis_free_min`) **ou** (sem visão e leitura válida < `d_clear` há menos de `suspect_ms`) |

Com visão fresca ela decide (livre ou suspeito); sem visão decide o histórico. `forwardAllowed` usa essa classe (suspeito ⇒ não avança).
**Suspeito ⇒** `enterAvoid(sem ré)`: PARADA → varredura → curva direto (sem ré inicial).

**Varredura** (`scan_deg=30`, `scan_step=15`): posições −30, −15, +15, +30 (vizinho mais próximo; começa pela sugestão do celular, se houver).
Nota: eco válido = cm; sem eco = **visão fresca do setor** (livre → `max_range_cm`, bloqueado → `d_stop/2`), senão `none_score_cm` (45).
Escolhe o **melhor ângulo**; empate → menor |ângulo|, depois esquerda. `scan/decide` registra ângulo e notas.
A sugestão `avoid deg` é varrida primeiro e **aceita de imediato** se a nota ≥ `d_clear_cm` (sem veto do sensor); com eco válido < `d_stop` ela é vetada e a varredura segue.
Nenhum ângulo ≥ `d_stop`: "sem saída" → ré pelo **crédito** e olha de novo (até `avoid_retries`), depois **fuga** de `escape_deg` (90°) sempre para o mesmo lado; PRESO só depois de `escape_max` (4) fugas sem andar livre. Crédito de ré: `docs/protocolo_ws.md`.

**Curva até o ângulo**: com `yaw` fresco (psi extrapolado entre amostras), gira até `Δpsi ≥ |alvo| − r·yaw_lead_ms`; faltando menos de `turn_slow_deg` (25°), `v_turn` e `s` caem proporcionais ao erro (mínimo `turn_k_min` = 0,35). Timeout `1,5·|alvo|/turn_dps + 0,3 s + turn_slow_deg/(turn_dps·turn_k_min)`; sem yaw, `|alvo| / turn_dps`. 300 ms depois: `{"t":"turn","alvo","girou","erro"}`. Ré: `min(reverse_ms = 500, crédito)`; sem crédito (< `cred_min_ms`) gira direto, sem ré cega.

**ESPERA e guarda de reentrada**:
- 3º desvio em 20 s ⇒ ESPERA (1, 2, 4 s). **Ao fim da ESPERA, se a condição persiste, faz a manobra completa** (PARADA → varredura), não outra ESPERA. "Persiste" = mediana < `d_stop`, suspeito, ou (desvio pedido pelo celular) a visão segue com o centro bloqueado / chegou outro `avoid` durante a espera, sem o sensor refutar. Antes, no modo visão, o ciclo avoid → ESPERA → devolve ao celular → avoid se repetia (35 s sem manobra no log de 20/11; simulado: 7 ESPERA e 2 manobras em 40 s → agora 1 ESPERA e 19 manobras).
- `avoid` recusado com leitura central válida, fresca e ≥ `d_clear` (log `avoid recusado: visao refutada pelo sensor (X cm)`).
- Depois disso a guarda só volta a esperar se o obstáculo **sumiu e voltou** (`gone`); obstáculo parado ⇒ manobras repetidas, não laço de ESPERA (simulado: 1 ESPERA e 21 manobras em 45 s com obstáculo fixo a 25 cm).
- `avoidLog`/`guardLevel` **decaem** com o tempo em FRENTE (não só ao reentrar).

---

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
| Campo 26/11 (0.6.1): teste do servo = 80,6 cm nos 5 ângulos com `hz` 50 (o PWM do servo está ligado no firmware); multímetro no GPIO18: 0 V e sem continuidade até o servo. O carro andou sem bater (rodas custaram a partir uma vez). `ENA/ENB 0 Hz` com o carro parado: o `ledcReadFreq` do core devolve 0 com duty 0 (não é falha) | 0.6.2: **servo do sensor no GPIO4** (livre, não é pino de boot; o servo de direção, nunca usado, passou para o 14). O log diz `parado` em vez de 0 Hz; o sim imita o core (duty 0 → 0 Hz) |
| Campo 26/11 (0.6.0): o crédito de ré funcionou (`cred` 1,25 s → ré → 0, depois gira direto), mas o carro quase só fazia "sem saída: fuga". **O sensor não gira desde 23/11**: em toda varredura de 23/11 e 26/11 as 4 direções leem a **mesma** distância (ex. 26,4 / 26,4 / 26,4 / 26,4); em 20–21/11 (0.5.2) variavam (46 / 47 / 65 / 67). O firmware manda o servo (`aim` −15/−30/+15/+30 no diário). No Explorar visão de 10:34 a câmera não calibrou: o celular estava desconectado no clique e o `calibrar` se perdeu (o ESP32 reenvia só o `goal`); na fonte sensor o visor nunca mandava `calibrar` | 0.6.1: o código do servo é o mesmo da 0.5.2, exceto o `ledcAttachChannel` da 0.5.3. Pelo fonte do core 3.3.12, o timer é escolhido por frequência/resolução nos dois casos (não era a causa da roda), mas por segurança **volta o `ledcAttach` da 0.5.2**. Novo: cada cliente recebe `pwm: servo GPIO18 50 Hz…` (frequência real) e o visor tem **Servo do sensor** (−60..+60, sem armar), que diz se o sensor girou. O sim agora lê o ângulo do **PWM** do pino do servo (`--servo-dead`, `--servo-nopwm`). Suspeita principal: o **fio do GPIO18**, vizinho do 19 que foi mexido em 22/11. Brain: goal `explorar` novo com o carro parado reaprende o chão (as duas fontes), sem depender do `calibrar` |
| Campo 21/11 (0.5.1): a correção sobe a roda certa, sem oscilar, e o trim aprende para o lado certo (−0,8) | ok; falta autoridade enquanto as rodas estiverem tão diferentes |
| Log 20/11 (2): correção de rumo não atuava | saiu do brain; roda no ESP32 nos dois modos, desde a arrancada, autoridade ±2,0 (até `hold_pwm_max`), trim persistente |
| Log 20/11 (3): ciclo avoid → ESPERA de 35 s com 86 cm válidos | sensor refuta a visão (brain e ESP32); fim da ESPERA com a condição persistente ⇒ manobra completa |
| Log 20/11 (4): curva passava do alvo (70° para 30°) | curva era espelhada (fiação) e sem desaceleração; agora desacelera perto do alvo e reporta `turn` |

---

## 7. Checklist de teste de campo (na ordem)

Antes de tudo: passar o fio de **sinal do servo** do GPIO18 para o **GPIO4** e gravar o firmware **0.6.2** (fio do ENA no **GPIO13**; aplicar a faixa das rodas e zerar o trim no passo 0) e o app (o brain mudou: reaprende o chão em todo Explorar novo); app aberto e conectado (o yaw vem dele).
No visor, **Gravar log** ligado o tempo todo. Ao abrir o visor, o log mostra `car_firmware 0.6.2 | pwm: servo GPIO4 50 Hz, ENA GPIO13 parado, ENB GPIO32 parado` (andando: 1000 Hz); `ERRO sem PWM` = pare e mande o log.

**00. Servo do sensor (desarmado, antes de tudo).** Mão (ou caixa) a ~30 cm só do lado ESQUERDO do carro, botão
**Servo do sensor**. Olhe o sensor: tem que ir à esquerda, centro, direita e voltar (~5 s). O visor mostra a distância
por ângulo: −60/−30 perto (a mão), 0/+30/+60 longe ou "sem eco". "Distâncias IGUAIS" = o sensor não girou: com o
carro desligado, confira o fio de **sinal** do servo no **GPIO4** (o GPIO18 morreu em 26/11), o **5 V** e o **GND** do servo
(servo sem 5 V não gira e às vezes treme). Sem o servo, o carro ainda anda, mas só olha para a frente (toda varredura
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

**Parâmetros a calibrar** (valores [DEMO] hoje; ver `docs/parametros_a_calibrar.md`):
- **`hold_*` / `trim_*`**: pelo passo 0 (reto com e sem correção). Ver `docs/parametros_a_calibrar.md`.
- **`turn_dps`** (90): com `yaw` desligado (tampar o giroscópio/desabilitar o sensor), comandar uma curva e medir os graus/s; só vale sem yaw.
- **`vis_free_min`** (50): ler `sec[2]` no log em chão livre × parede × objeto baixo e escolher o corte.
- **`suspect_ms`** (1500): duração típica do "sem eco" depois de um eco válido (parede em diagonal, sofá).
- Depois: `yaw_lead_ms` (se passa do ângulo, aumentar), `vis_fresh_ms`/`yaw_fresh_ms` (conferir o atraso real no log), `reverse_ms`, `pwm_min/max`, `trim_*`.
