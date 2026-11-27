# Fluxo de execução (firmware 0.7.x)

Arquitetura "ESP32 executor": o ESP32 é o único executor de manobras. O celular roda a visão e envia a opinião da visão e o rumo do giroscópio.
Este documento descreve o caminho de um quadro da câmera até o motor. O histórico de testes de campo está em [`diario_de_campo.md`](diario_de_campo.md).

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
| 11 | Toda mensagem enviada e recebida e cada frame vão para o **log JSONL** (fila + thread, flush 1 s); nada é transmitido durante a corrida | | `filesDir/logs/corrida_<data>_<hora>_<src>.jsonl` (+ `.mjpg` e `_indice.csv` do vídeo) |

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
| `control/hal.py` `LinkRangeHead`/`LinkDrive`/`SerialLink`/`MemoryLink` | Testes, `sim/check_firmware.py`, `tools/serial_check.py`. Poderiam alimentar um `Runner` com sensor real, mas o arquitetura "ESP32 executor" faz o ESP32 ler o sensor. |
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

