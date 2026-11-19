# Fluxo atual do projeto (mapa de leitura, sem alterações de código)

Estado analisado: branch `main`, com as alterações ainda não commitadas na árvore de trabalho
(`brain.py`, firmware, `config.py`, etc.). Tudo abaixo vem da leitura do código. Os itens marcados
**[verificado]** também foram executados. Os marcados **[hipótese]** são inferência de leitura, sem teste no carro.

---

## 1. Cópias Python do Android × raiz

- `android/app/src/main/python/` **não está no git** (`.gitignore` da raiz, linha `android/app/src/main/python/`).
  É gerada.
- A sincronização é **automática** e fica em `android/app/build.gradle.kts`:
  a task `syncPython` (tipo `Sync`) copia da raiz do repo para `src/main/python`:
  `config.py`, `vision/*.py`, `control/*.py`, `tools/{__init__,bench_s9,android_entry,ws_link}.py`,
  `brain.py`, `viz.py`, `main.py`.
  Ela é ligada a `preBuild` e a toda task cujo nome contém "Python" (`dependsOn(syncPython)`).
  Basta dar **Run** no Android Studio. Por ser `Sync`, **apaga** qualquer outro arquivo dessa pasta.
- **[verificado]** `diff` (ignorando CR) entre as 18 `.py` da pasta Android e as da raiz: **todas idênticas**
  (as de `brain.py`/`config.py` são da última build, 18/11 14:37).
  Só enquanto não houver build novo é que a cópia pode ficar velha.
  Há ainda cópias derivadas do Chaquopy em `android/app/build/python/sources/{debug,release}/` (artefato de build, ignorar).
- Ficam **fora** do app (de propósito): `replay/`, `sim/`, `tests/`, `tools/{label,make_demo,re_frame,serial_check,video_to_frames}.py`.
- Detalhe: o docstring de `tools/android_entry.py` cita `hardware/`, pasta que não existe (é `control/hal.py`).

Código Kotlin (`android/app/src/main/java/com/example/carbench/`):

| Arquivo | Papel |
|---|---|
| `LiveActivity.kt` | CameraX (640×360, RGBA_8888, `KEEP_ONLY_LATEST`), laço por frame, UI |
| `BrainBridge.kt` | ponte Kotlin → `brain.py` (`start/stop/step/popCalibrate/status`), faixa de status |
| `YawSensor.kt` | giroscópio projetado na gravidade → `brain.on_gyro` (~50 Hz, thread própria) |
| `MainActivity.kt` | botões de bancada: `smoke`, `bench`, `run_main` (usa o mock; **não** fala com o carro) |

---

## 2. Caminho de execução no S9 (frame → ESP32)

Thread única `exec` (`LiveActivity.exec`) para a câmera + Python. Tudo abaixo roda nela, exceto onde indicado.

| # | Onde | Entra | Sai |
|---|---|---|---|
| 0 | `LiveActivity.onResume` → `BrainBridge.start()` (executor `io`) → `brain.start()` | — | `WsLink(ws://192.168.4.1:81/)` em thread própria com reconexão; `viz.start()` (MJPEG :8080) |
| 1 | CameraX `analysis.setAnalyzer(exec)` → `LiveActivity.analyze(img)` | `ImageProxy` RGBA | bytes do plano 0 em `ByteArray` reutilizado |
| 2 | 1ª chamada: `Python.getModule("main").callAttr("StreamRunner", true, filesDir/live_log.csv)` | `use_edges=true`, **`log_path` = sim** | objeto `StreamRunner` (cacheado em `stream`) |
| 3 | `if (BrainBridge.popCalibrate() \|\| wantCal) stream.recalibrate()` | botão da tela ou comando "calibrar" do visor | novo `Runner` (zera chão aprendido + controller) |
| 4 | `StreamRunner.process_rgba_frame(arr, w, h, stride, rotation)` (`main.py`) | bytes | **string JSON** (`info`) |
| 4a | `frame_from_rgba` | RGBA com padding | BGR já rotacionado |
| 4b | fase **SETTLE** (1 s após `recalibrate`) | — | `info` sem `vision_state` |
| 4c | `Runner.feed(frame)` | BGR | `info` dict |
| 4d | `Runner.feed`: **WARMUP** (11 frames) → `learn_floor_hist` + `calibrate_thr`; depois **RUN**: `floor_mask` → `viz.push` → `floor_prob` → `coverage_row` → `DecisionMaker.update` → (se `use_edges`) `edge_features` + `edge_is_near` → `Controller.update(t, Vision(vstate, near))` → `link.send(t, drive)` | | `info` = `ctrl_state, sub, why, v, steer, sensor_m, angle, guard_level, phase, vision_state, r, edge_*, proc_ms, ...` |
| 5 | `BrainBridge.step(json)` → `brain.step(json)` | o mesmo JSON | `(v, s)` (descartado pelo Kotlin) |
| 6 | `brain.step`: `_norm_state` → `st`; `goal = link.goal`; troca de goal ⇒ `link.mode(...)`, `link.arm(True)`; calcula `v`; `_heading.update` → `s`; `link.drive(v, s)`; eventualmente `link.send({"t":"avoid"})`; `link.think(...)` | | mensagens WS |
| 7 | `WsLink.send` (JSON compacto por WebSocket) | | ESP32 |
| 8 | Firmware: `netLoop → ws.loop → wsEvent → handleJson` | `drive/mode/arm/avoid/think/goal/calibrar` | `cmdV/cmdS/lastDriveT`, `mode`, `armed`, `avoidReq`; `think/goal/calibrar` são **repassados** (`netRelay`) a todos os clientes |
| 9 | `loop()`: `buttonTask → servoTick → sensorTask → (autoStep \| phoneStep) → driveApply` | | PWM L298N |
| 10 | Kotlin `render(JSONObject(json))` na UI thread | o mesmo JSON | texto "ctrl=… sub … why …" |

Paralelos: `YawSensor` → `brain.on_gyro(rate)` → `HeadingHold.feed_rate`. `BrainBridge.resumeStatus` chama `brain.status()` a cada 500 ms para a faixa.

### Respostas diretas

- **O StreamRunner recebe `log_path`?** Sim. `LiveActivity.kt:86` passa `File(filesDir, "live_log.csv")`.
  O CSV grava só `LOG_FIELDS` (wall_s, t, n, phase, vision_state, r, edge_*, ctrl_state, sub, why, conv_ms, proc_ms, w, h, rot).
  **Não grava** `v`, `steer`, `sensor_m`, nem nada do que o brain enviou ao ESP32 nem do que o ESP32 respondeu.
- **O Runner usa `MockRangeHead` no S9?** Sim. `StreamRunner.recalibrate()` cria `Runner(cfg, use_edges, cv_threads=1)` sem `head` nem `link`.
  O `Runner` usa então `MockRangeHead(Scenario.constant())` e `MockDriveLink()`.
  O cenário constante é 99 m e o alcance máximo do mock é 2 m, então a leitura é **sempre "sem eco"** (`dist_m=None`, `settled=True` depois de 0,15 s).
  O `Controller` do celular roda, mas num mundo sem sensor, e o `Drive` que ele gera vai para `MockDriveLink.send`, que só faz `log.append` (lista que **cresce sem limite**, 1 entrada por frame).
  Nada disso chega ao ESP32.
- **Quem envia `{"t":"mode"}`?** Dois emissores:
  1. `brain.step`, **uma vez a cada troca de goal**: `explorar → mode "auto"`, `reto → mode "phone"`. Para "parar" não envia `mode`.
  2. O **visor web** (`web_ui.h`, botões "Modo auto" / "Modo celular"), a qualquer momento. O brain **não percebe** essa troca (só reage a mudança de goal).
  (Também a serial: `mode auto|phone`.)
- **Quem envia `{"t":"goal"}`?** **Só o visor web** (botões Explorar / Reto / Parar; "PARAR TUDO" manda `arm:false` + `goal:parar`).
  O ESP32 guarda (`netGoal`, `rtcGoal`), aplica efeito próprio (ver §4) e **repassa** a todos os clientes WS; é assim que o S9 recebe (`WsLink._on_message` → `link.goal`).
  O ESP32 também reenvia o goal atual a quem conecta. O S9 **nunca** envia goal.
  O botão Explorar manda antes `{"t":"calibrar"}`, que o ESP32 repassa ao S9 (`calib_seq++` → `popCalibrate()`).

### Achados do caminho (todos de leitura, a confirmar com você)

**A. O `Controller` do celular roda "em sombra", e o `why` dele vaza para a decisão do brain. [verificado em simulação]**
`brain._norm_state` procura a chave `state/estado/decision/decisao` no JSON. O `info` **não tem nenhuma delas**
(tem `vision_state`, `ctrl_state`, `why`).
Então ele cai no fallback `str(dict).upper()` e procura as substrings `OBSTACULO`, `INCERTO`, `LIVRE`, nessa ordem de prioridade.
Resultado: o texto de `why` do controller conta como estado de visão.
Simulei `Controller` + mock com 0,3 s de OBSTACULO seguidos de LIVRE, passando `brain._norm_state` sobre o JSON:

```
t=1.5 visao=OBSTACULO ctrl=DESVIO/PARADA  why='visao OBSTACULO sem refutacao'  brain->OBSTACULO
t=1.8 visao=LIVRE     ctrl=DESVIO/OLHA_E  why='visao OBSTACULO sem refutacao'  brain->OBSTACULO
t=2.2 visao=LIVRE     ctrl=DESVIO/RE      ...                                  brain->OBSTACULO
t=3.0 visao=LIVRE     ctrl=DESVIO/CURVA   ...                                  brain->OBSTACULO
t=4.0 visao=LIVRE     ctrl=VARREDURA      why='pos-desvio'                     brain->LIVRE
```

Como o sensor mock nunca refuta, **todo** OBSTACULO de visão vira um DESVIO simulado de ~2,5 s.
Durante ele o brain enxerga OBSTACULO mesmo com a visão já LIVRE: `v=0`, e pode pedir `avoid` de novo (a cada 2 s, se o ESP32 não estiver em DESVIO).
**[hipótese]** isso pode encadear dois desvios reais no ESP32 por um único obstáculo.
O `think` também usa `info["why"]` como texto (`msg = src.get("why")`), então o visor mostra o `why` do controller fantasma.
A UI Kotlin mostra `ctrl=DESVIO RE` mesmo com o carro parado ou andando por outro motivo.

**B. `conf` nunca chega.** `_KEYS_CONF = (conf, confidence, prob_mean)`; o `info` traz `r`. O chip "visão: X (NN%)" do visor aparece sem percentual.

**C. Explorar não espera a calibração da cor do chão.** Em `explorar`, o brain **não envia `drive`** (`sensor_only`).
O ESP32 arma sozinho 3 s depois do goal (e de novo 3 s depois do `arm` do brain) e anda por `autoStep`.
A calibração no S9 (1 s de SETTLE + 11 frames) é paralela e **não há handshake**: o `v=0` do brain durante a calibração só existe em `reto`.
Nada no movimento de `explorar` depende da visão.

**D. `edge_probe`/`edge_near` só alimentam o controller fantasma.** Não chegam ao ESP32 (a não ser pelo vazamento do item A, e `edge_near` não contém palavra-chave).

**E. `docs/architecture.md` descreve o desenho pretendido, não o que roda.**
O diagrama tem `controller → drive, aim → ESP32` e `ESP32 → range → controller`. No S9 hoje quem fala com o ESP32 é o `brain.py`, só com `drive/arm/mode/avoid/think`, e nunca há `aim` nem leitura de `range`.

**F. Reconexão.** Se o WS cair e voltar, `link.goal` não muda ⇒ `brain.step` não reenvia `mode`/`arm`.
Se o ESP32 reiniciou (voltando ao `DEFAULT_MODE` auto), o brain pode continuar "pensando" `reto` com o ESP32 em auto.

---

## 3. Visão/controle que existem e não estão no caminho do S9

| Módulo/função | Quem usa hoje | Onde poderia conectar |
|---|---|---|
| `vision/ground.free_space_vector(mask)` | só `tests/test_pipeline_smoke.py` | Em `Runner.feed`, logo após `floor_mask` (a máscara já está calculada). Colocar o vetor no `info` (ex.: `free_vec`, `free_best_dir`). No `brain.step`, vira `s_cmd` (dirigir para a faixa mais livre) em `_heading.update(t, v, s_cmd)`. Hoje `s_cmd` é sempre 0. Observação: `HeadingHold` só atua com `|s_cmd| < s_eps`, então é preciso decidir como os dois conviveriam. |
| `control/hal.LinkRangeHead` + `LinkDrive` | `tests/test_controller.py`, `sim/check_firmware.py`, `tools/serial_check.py` (via `SerialLink`/`MemoryLink`) | `Runner(head=LinkRangeHead(link), link=LinkDrive(link))`. Faltam: (i) `WsLink` **não tem `poll()`** (o `LinkRangeHead.read` precisa dele), só guarda `ranges{}` e `state{}`; seria um adaptador com fila alimentada por `on_msg`. (ii) o `WsLink` hoje pertence ao `brain.py`, então `Runner` e `brain` teriam de compartilhar a mesma conexão. (iii) o ESP32 só aceita `aim` em `mode phone` e sem `delegated`. (iv) hoje há dois "donos" do `drive` (Runner e brain). |
| `control/hal.SerialLink`, `MemoryLink` | testes/bancada | Não faz sentido no S9 (rede é WebSocket). |
| `Controller` (FRENTE/VARREDURA/DESVIO) | roda no S9 só com mock; é a base de `replay`, `bench_s9`, testes | Opção 1: alimentar com sensor real (item acima) e fazer o brain **enviar** o `Drive` dele em vez de recalcular `v` por conta própria. Opção 2: removê-lo do S9 e manter só a `DecisionMaker` (resolve o vazamento do achado A). |
| `WsLink.ranges` / `link.state["c"]` | só `brain.status()` (texto) | O brain já recebe a distância frontal pelo heartbeat (500 ms) e as leituras `range` (só em modo phone). Dá para usar como "refutação" da visão, como o `Controller` faz (sensor perto ⇒ OBSTACULO; sensor longe ⇒ visão refutada). |
| `vision/flow_probe.py` | ninguém | Descartado (conforme você). É **byte a byte igual a `edge_probe.py`** [verificado com `diff`], mas continua sendo copiada para o app (`vision/*.py`). Candidata a remover. |
| `vision/coverage_log.log_floor_coverage/plot_coverage`, `edge_probe.run/plot` | ferramentas offline (CLI) | Fora do laço ao vivo. |
| `tools/bench_s9.py`, `android_entry.run_main` | `MainActivity` (bancada) | Só mock. |

---

## 4. Quem decide o movimento: goal × mode

Legenda: **ESP** = `autoStep` (só sensor) · **brain** = `brain.step` (visão + rumo) · **WD** = watchdog (`phoneStep`: sem `drive` por `wd_ms`=500 ⇒ `delegated` + `resetAuto` + `autoStep`, `fallback_auto=1`).
Em todos os casos valem os reflexos do ESP32 (`driveApply`): só anda com leitura central fresca (≤400 ms) e `centerRaw` sem eco ou ≥ `hard_stop_cm` (20), limite de ré 1500 ms, rampa, sem partir com o servo em movimento.

| goal \ mode | **auto** | **phone** |
|---|---|---|
| **explorar** | **ESP** decide tudo (`autoStep`: FRENTE → VARREDURA → DESVIO). O brain não manda `drive` (`sensor_only`) e fixa `s=0`. | O brain **não manda `drive`** ⇒ após ~500 ms **WD** coloca o **ESP** no comando (`"sem celular: ESP32 segue so com o sensor"`). Com `fallback_auto=0`, o carro para. Na prática é igual a auto. |
| **reto** | **ESP** decide (`autoStep`). O `drive` do brain chega e é guardado (`cmdV/cmdS`), mas `autoStep` o ignora; `avoid` é ignorado (exige `mode==phone`). O brain continua "pensando" `-> v=0.35` sem efeito. (Só acontece se alguém clicar "Modo auto" no visor depois de `reto`.) | **brain** decide `v` pela visão (LIVRE 0,35 / INCERTO 0,15 / resto 0) e `s` pelo `HeadingHold`; `drive` a cada frame. Visão OBSTACULO + armado ⇒ `avoid` (no máx. a cada 2 s) ⇒ o **ESP** assume (`delegated`: PARADA → OLHA_E/D → RE → CURVA), depois "celular retomou o controle". Sem frames do S9 por 500 ms ⇒ **WD**. |
| **parar / nenhuma / None** | O ESP32 **desarma** ao receber `goal parar` (`setArmed(false)`): ninguém move. O brain só manda `drive(0,0)`. **Exceções:** `goal nenhuma` não desarma; e se alguém armar pelo botão BOOT ou por `go` na serial, em auto o **ESP** anda mesmo com goal "parar" (o brain não desarma de novo, só envia `drive 0`, que o auto ignora). | Desarmado: ninguém move. Armado à mão: o brain manda `v=0`. |

**O `controller.py` do celular não decide nada em nenhuma célula** (ver achado A). Em `reto/phone`, a avoidance real é do ESP32, disparada pela visão (`avoid`). Em `reto/phone` o ESP32 **não** compara a distância com `d_stop_cm` (30 cm). Só o reflexo de 20 cm e o `avoid` pedido pelo brain contam. Um obstáculo que a visão classifica como LIVRE só é parado pelo `hard_stop`.
Depois de um DESVIO em phone, o `phoneStep` devolve o comando ao celular assim que o `drive` estiver fresco.

### Quais `think`/mensagens chegam ao visor, por origem

| Origem | Mensagem | Quando |
|---|---|---|
| **brain** (`link.think`) | `{"t":"think","msg","state","conf"}` → log `[cerebro]` + chip "visão: ESTADO" | motivação nova ("motivacao 'X': ..."); mudança de `st` ou a cada 0,5 s. Texto = `info["why"]` do controller fantasma **ou** fallback `"LIVRE -> v=0.35"` / `"LIVRE -> sensor (ESP32 auto)"`. Em `calibrar`: "calibrar: aprendendo...". Em OBSTACULO+avoid: "obstaculo na visao: pedindo desvio ao ESP32". |
| **brain** (`viz`) | `{"t":"think","viz":url}` a cada 3 s | atualiza a imagem MJPEG |
| **ESP32** `state` (500 ms) | `Estado` ("FRENTE/..." em auto, "ESP32 no comando: ..." se `esp`, "modo celular" caso contrário), `why`, "frente: N cm", barras L/R, uptime/reset; log `[esp32] why` só quando muda | sempre |
| **ESP32** `log` | "armando em 3 s", "ARMADO", "desarmado", "ESP32 assumiu: desvio pedido", "celular retomou o controle", "sem celular: ESP32 segue so com o sensor", brownout/retomada | eventos |
| **ESP32** `scan` / `range` | log `[servo] ...` (aim/olha/decide) e radar | durante desvios; `range` em modo phone ou com cliente WS |
| **ESP32** `goal` | log "motivacao: X" + destaque do botão | eco do `goal` e ao conectar |

Por célula: **explorar/auto** mostra estado do ESP + think "→ sensor (ESP32 auto)"; **explorar/phone** mostra "ESP32 no comando" (por causa do WD) + o mesmo think. **reto/phone** mostra "modo celular" + think "→ v=…" e, na manobra, "ESP32 no comando: DESVIO/…" mais os `scan`. **reto/auto** mostra estado do ESP, mas o think continua "→ v=…" (enganoso). **parar** mostra "desarmado" + think "parado".

---

## 5. Firmware: tratamento de "sem eco" (`-1`)

Convenção: `-1` = sem eco = **livre**. Valores abaixo são do `car_firmware.ino` atual (v0.3.4).

- **`pingRawCm`** (l.316): `pulseIn(ECHO, HIGH, max_range_cm*58.3 µs)`; `us==0` ⇒ `-1`; `cm > max_range_cm (200)` ⇒ `-1`. Timeout de ~11,7 ms **bloqueante** a cada ping (período de 65 ms).
  O HC-SR04 também devolve "sem eco" para obstáculo muito perto (<~2 cm), macio ou inclinado, e isso vira "livre".
- **`medianKeep`** (l.328): sem eco vira `1e9` antes de ordenar.
  n=1: o valor. n=2: `v[0]`, o **menor** (um eco válido vence um sem eco). n=3: a mediana `v[1]`.
  Retorno `≥1e8` ⇒ `-1`. Assim, com 3 amostras, dois "sem eco" + um válido ⇒ `-1` (livre).
  `aimTo` zera o anel (`ringN=0`) e só entram amostras depois de o servo parar (`servoDeg==servoCur`) e de `settle_ms` (150 ms), então a 1ª leitura após mirar é n=1 (bruta).
- **`sensorTask`** guarda `meas.cm = med` e, se o servo está no centro (±10°), `centerCm = med` (mediana) e `centerRaw = raw` (**amostra bruta atual**), `centerT`, `haveC`.
- **`forwardAllowed`** (l.358): exige `haveC` e leitura central fresca (`fresh_ms` 400); depois `centerRaw < 0 || centerRaw >= hard_stop_cm`.
  **Usa a amostra bruta, não a mediana**: um único ping sem eco libera o reflexo naquela janela, mesmo com a mediana dizendo "perto".
  Em `auto` isso é mitigado porque `autoStep` já zera `v` pela mediana (`d<d_stop`). Em `phone` o reflexo é a única barreira (ver §4).
- **`autoStep`**:
  - FRENTE/VARREDURA: `d = centerCm`; `d>=0 && d<d_stop(30)` ⇒ `enterAvoid("sensor perto")`; `mid = d>=0 && d<d_clear(60)` ⇒ VARREDURA a `v_slow`. **`d<0` não dispara nada ⇒ `v_cruise`** (sem eco = pista livre; no auto não há visão para contradizer).
  - OLHA_E/OLHA_D: `sc = (leu && cm>=0) ? cm : none_score_cm (45)`. Sem eco **e** estouro de tempo valem 45 (acima de `d_stop`, abaixo de `d_clear`). `got=0` só aparece no log (`scan`). `decideSide`: ambos `< d_stop` ⇒ "sem saída" (re maior 1,5× e olha de novo, até `avoid_retries`=2, depois PRESO). Como 45 ≥ 30, um lado sem eco conta como passável. Empate ⇒ esquerda.
  - PRESO: `ok = centerCm<0 || centerCm>=d_clear` (sem eco = liberado), por `clear_ms`.
- **Laço ESPERA → VARREDURA → ESPERA com obstáculo parado (<30 cm)** — **[leitura do código, não rodado]**:
  1. Os 2 primeiros desvios na janela de 20 s fazem a manobra completa (PARADA → OLHA → RE → CURVA).
  2. Se logo depois o centro ainda vê `<30 cm`, `enterAvoid` encontra `avoidN >= guard_max (2)` ⇒ **ESPERA** com `backoff1/2/3 = 1000/2000/4000 ms` (`guardLevel` sobe a cada reentrada e satura em 4000).
  3. Fim da ESPERA: `autoStep` põe `cst=VARREDURA`, `v=v_slow` **por uma única passada do `loop()`**. Na passada seguinte `d<d_stop` ⇒ `enterAvoid` ⇒ ESPERA de novo (`v=0` na hora).
     O `v_slow` dessa passada praticamente não move o carro: a rampa dá ~`slew_per_s·dt` ≈ 0,02 e `motorWrite` trata `|v|<0,02` como parado. O carro fica parado.
  4. `avoidLog` é podado por janela de 20 s no `enterAvoid`, mas cada ciclo acrescenta uma entrada, então **nunca esvazia** e `guardLevel` **nunca zera**: o carro espera 4 s, re-espera 4 s, e assim por diante, **sem tentar nova manobra**, até o obstáculo sair (ou `stop`).
  5. No visor aparece só "ESPERA / reentrada: espera 4000 ms" (o log `[esp32] why` deduplica texto repetido, e o estado "VARREDURA" dura 1 passada, invisível no heartbeat de 500 ms).
  6. Em `reto/phone` esse laço **não existe**: terminada a ESPERA, `phoneStep` vê `cst != DESVIO` com `drive` fresco e **devolve o controle ao celular**; só o reflexo de 20 cm segura (obstáculo entre 20 e 30 cm com visão LIVRE ⇒ avança até <20 cm).

---

## Pontos para sua revisão (nada foi alterado)

1. Achado A (vazamento do `why` do controller fantasma no `_norm_state`) é o de maior impacto e **está verificado**; decidir se o `Controller` sai do S9, ganha sensor real, ou se o brain passa a ler `vision_state` explicitamente.
2. Achado C: a calibração da cor do chão não bloqueia o movimento em `explorar`.
3. §5.4: laço ESPERA infinito com obstáculo parado (sem retentativa de manobra).
4. `forwardAllowed` usa `centerRaw` (bruto) enquanto o resto usa a mediana.
5. `flow_probe.py` é cópia idêntica de `edge_probe.py` e segue sincronizado para o app.
6. `MockDriveLink.log` cresce sem limite no S9 (1 entrada/frame).
