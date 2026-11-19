# Parâmetros a calibrar

Valores que hoje são **constantes por falta de informação real** (carro, sensor, S9).
Nada aqui está medido no hardware. Quando um valor for medido, ele entra em
`config.json` (carregado por `Config.load`), sem editar código.

Legenda de status:

- **DEMO**: valor arbitrário, só para a lógica funcionar.
- **HIPÓTESE**: derivado de 2–3 vídeos feitos à mão (não vale para o carro).
- **MEDIR**: só se obtém com o hardware (carro, HC-SR04, S9).
- **MIGRAR**: ainda é constante dentro de um módulo, fora do `config.py`.

Convenções: tempos em **segundos** (nunca ticks); `v` e `steer` são **normalizados**
(não são m/s nem graus).

---

## 1. Carro (`config.calib` + `control.v_*`)

| Parâmetro | Valor atual | Status | Como obter |
|---|---|---|---|
| `calib.speed_at_v_rev_mps` | `None` | MEDIR | Comandar `v = v_rev` em linha reta por 2 s sobre uma pista de 1 m marcada; velocidade = distância / tempo. Repetir 3x. |
| `calib.yaw_rate_dps` | `None` | MEDIR | Comandar `steer = 1`, `v = v_turn` por 2 s; medir o ângulo girado (transferidor no chão ou vídeo de cima) e dividir por 2. |
| `control.v_cruise / v_slow / v_rev / v_turn` | 0,35 / 0,15 / 0,3 / 0,3 (= `Params` do firmware; `tests/test_config_firmware.py` confere) | DEMO | Escolher velocidades que o carro realmente consiga manter; anotar a velocidade em m/s de cada uma. |
| **`control.turn_dps`** (firmware `turn_dps`) | 90 °/s | **MEDIR** | Graus/s com `v = v_turn` e `steer = 1` (= `calib.yaw_rate_dps`). **Só vale quando o celular não manda `yaw`** (a curva é feita pelo tempo `ângulo / turn_dps`). Medir com 360° completos, 3x, na bateria carregada. Com `yaw` fresco a curva gira até o ângulo e este valor só define o timeout. |
| Latência comando → movimento | desconhecida | MEDIR | Filmar o carro e marcar o instante do envio do comando (log no celular) e o início do movimento. |
| Distância de frenagem de `v_cruise` | desconhecida | MEDIR | Mandar `v = 0` em plena marcha e medir quanto ele anda até parar. |

## 2. Sensor HC-SR04 + servo (`control.*`, `hardware/hal.py`)

| Parâmetro | Valor atual | Status | Como obter |
|---|---|---|---|
| `control.d_stop_m` | 0,30 m | DEMO | `d_stop ≥ v_cruise[m/s] · (latência visão + comunicação + ciclo do sensor) + frenagem + margem`. |
| `control.d_clear_m` | 0,60 m | DEMO | Distância a partir da qual uma leitura válida "refuta" a visão. Escolher com base no alcance confiável medido. |
| `control.none_score_m` | 0,45 m | DEMO | Valor dado a um lado sem eco ao escolher a curva. Revisar com a taxa de "sem eco" medida por tipo de superfície. |
| `control.scan_deg` / `scan_step_deg` | 30° / 15° | DEMO | O ESP32 varre -30, -15, +15, +30 (o centro é o obstáculo) e **gira até o melhor ângulo**. O passo de 15° é a abertura do feixe do HC-SR04 e o passo do servo. Alinhado aos 5 setores da visão (FOV ~74°, ~14,8° por setor). |
| `control.look_timeout_s` | 1,0 s | DEMO | Maior que o tempo de assentamento do servo medido. |
| Tempo de assentamento do servo (`MockRangeHead.settle_s`) | 0,15 s | MEDIR | Cronometrar de `aim()` até a leitura estável, para -45°, 0° e +45°. |
| Alcance útil (`MockRangeHead.max_range_m`) | 2,0 m | MEDIR | Teste de bancada: o HC-SR04 especifica ~2–400 cm, mas o alcance útil em objetos pequenos costuma ser menor. |
| Ruído / taxa de "sem eco" por tipo de objeto | 1 cm / 0 % | MEDIR | Teste de bancada de 1 h: perna de cadeira, tecido do sofá, parede em ângulo, pessoa andando. Alimenta `faults` do mock. |
| Ciclo mínimo entre leituras | ≥ 60 ms (especificação) | MEDIR | Confirmar no firmware. Limita a taxa de varredura. |
| Posição do sensor em relação à câmera (paralaxe) | não modelada | MEDIR | Medir o deslocamento em cm; compensar o ângulo do servo em curta distância. |

### 2b. "Sem eco" em 3 estados e opinião do celular (firmware 0.4; `control.*`)

| Parâmetro | Valor atual | Status | Como obter |
|---|---|---|---|
| **`vis_free_min`** | 50 (setor 0, escala 0..100) | **MEDIR** | Valor do setor central (`sectors[2]`, percentil 20 da altura de chão livre) a partir do qual a visão diz "livre". Gravar o log (`sec`) andando em chão livre, em frente a uma parede e a um objeto baixo; escolher o limiar entre as duas populações. Abaixo dele: sem eco = **suspeito**, e a nota de um lado sem eco cai para `d_stop/2`. |
| **`suspect_ms`** | 1500 ms | **MEDIR** | Quanto tempo depois de uma leitura válida < `d_clear` um "sem eco" ainda é tratado como suspeito. Medir com uma parede em diagonal e com tecido do sofá: duração típica do "sem eco" depois de um eco válido. Longo demais = carro parado à toa; curto demais = volta a acelerar contra a parede. |
| `vis_fresh_ms` / `yaw_fresh_ms` | 300 / 200 ms | DEMO | `vis` sai a ~10 Hz e `yaw` a ~20 Hz; devem ser ≥ 2–3 períodos e menores que `wd_ms`. Ver no log o atraso real do S9. |
| `yaw_lead_ms` | 100 ms | DEMO | Antecipa o fim da curva em `giro × lead` (inércia). Se o carro passa do ângulo, aumente; se fica curto, diminua. |
| `none_score_cm` | 45 | DEMO | Nota de uma posição sem eco **quando não há visão fresca** (com visão: livre = `max_range_cm`, bloqueado = `d_stop/2`). |

## 3. Manobra do controller (`control.*`)

| Parâmetro | Valor atual | Status | Como obter |
|---|---|---|---|
| `control.stop_s` | 0,3 s | DEMO | Tempo para o carro parar de fato. Usar a frenagem medida. |
| `control.reverse_s` | 0,5 s (×1,5 sem saída) | DEMO | Derivar com `derive_maneuver_times(cfg, reverse_m, turn_deg)` a partir de `speed_at_v_rev_mps`. |
| `control.turn_s` | 1,0 s | DEMO | **Só o controller v1 congelado** (tempo fixo). O firmware não usa mais `turn_ms`: gira até o ângulo (yaw) ou `ângulo / turn_dps`. |
| `control.avoid_retries` | 2 | DEMO | Quantas vezes tentar de novo (ré maior + nova olhada) antes do PRESO. |
| `control.clear_s` | 0,2 s | HIPÓTESE | Tempo "limpo" para voltar a FRENTE (equivale a 3 ticks a 10 fps). Revisar com o fps real do S9. |
| `control.guard_window_s / guard_max_episodes / guard_backoff_s` | 20 s / 2 / (1, 2, 4) s | DEMO | Testar com uma pessoa atravessando à frente do carro; ajustar para o tempo típico de passagem. |
| `control.edge_near_h / edge_cov_min` | 0,20 / 0,50 | HIPÓTESE | Só validado no vídeo da parede 1 (rodapé de alto contraste). Parede 2 não detectada. |

## 4. Decisão da visão (`decision.*`)

| Parâmetro | Valor atual | Status | Como obter |
|---|---|---|---|
| `ref_s`, `ref_min` | 0,9 s, 0,05 | HIPÓTESE | Premissa: a janela inicial só mostra chão. Verificar no carro. |
| `r_free / r_obst / r_exit` | 0,5 / 0,2 / 0,4 | HIPÓTESE | Vieram de 2 vídeos (corredor: r mín. 0,28, folga de só 0,08 em relação a `r_obst`). |
| `u_unc / u_obst` | 0,30 / 0,15 | HIPÓTESE | Idem. |
| `enter_obst_s / exit_obst_s / enter_livre_s / timeout_s` | 0,1 / 0,4 / 0,2 / 0,9 s | HIPÓTESE | Equivalem a 2 / 5 / 3 / 10 frames a 10 fps. Reavaliar com o fps e a latência reais. |

## 5. Features de visão (constantes em módulos, **MIGRAR**)

| Parâmetro | Valor atual | Onde | Observação |
|---|---|---|---|
| `WARMUP` | **10 frames** | `vision/coverage_log.py` | Em frames, não em segundos. Define o buffer inicial do `thr` automático e a referência. Converter para segundos. |
| Percentil do `thr` automático | 5 | `vision/thr_auto.py` | Hipótese; no corredor (p5 = 2,0) a ROI parece heterogênea. |
| `STRIP_Y0`, `CEN_X0/CEN_X1` | 0,75 / 1/3–2/3 | `vision/coverage_log.py` | Faixa inferior e terço central; dependem da altura/inclinação da câmera. |
| `V_LOW`, `V_HIGH` | 40 / 235 | `vision/coverage_log.py` | Limites de "desconhecido" (V escuro/estourado). Não captura o chão claro iluminado (frame de 6,7 s no sofá). |

## 6. `ground.py`

| Parâmetro | Valor atual | Observação |
|---|---|---|
| `target_w` | 320 | Compromisso resolução × custo no S9. Reavaliar após medir fps. |
| ROI (x0..x3, y0..y3) | trapézio: base 5–95 % da largura a 95 % da altura; topo 25–75 % a 65 % da altura | Depende da altura e da inclinação da câmera no carro. |
| Bins do histograma H×S | 30 × 32 | |
| `strip_w`, `run`, `min_floor_frac` | 8, 5, 0,5 | Resolução do vetor de espaço livre e sensibilidade a ruído. |
| Kernel morfológico / `medianBlur` | 3×3 / 3 | |

## 7. `edge_probe.py`

| Parâmetro | Valor atual | Observação |
|---|---|---|
| `SCAN_TOP / SCAN_BOTTOM` | 0,25 / 0,97 | Faixa de linhas varrida. |
| `CANNY_SIGMA` | 0,33 (automático) | Canny fixo baixo pegou bordas de troca de cor; manter automático. |
| `HORIZ_RATIO`, `MIN_LEN`, `MIN_FRAC` | 4,0 / 20 px / 0,5 | Filtro de borda quase horizontal e longa. |

## 8. Câmera e S9

| Item | Valor atual | Status | Como obter |
|---|---|---|---|
| Altura e inclinação da câmera no carro | desconhecidas | MEDIR | Medir com régua e inclinômetro; definem ROI e proxy de distância pela borda. |
| HFOV (campo de visão horizontal) do frame **final** | desconhecido | MEDIR | Calibrar com alvo a distância conhecida. O `re_frame.py` recorta e **muda o FOV**. Necessário para `theta = atan((x − W/2)/f_px)`. |
| Exposição / balanço de branco | automáticos | MEDIR | Travar AE/AWB no S9 (parte do problema de luz vem do auto-exposure). |
| fps e latência de processamento no S9 | desconhecidos | MEDIR | Teste de fumaça (ver plano do S9). |

## 9. Comunicação e ESP32 (firmware 0.4)

| Item | Valor proposto | Observação |
|---|---|---|
| `hard_stop_cm` (parada reflexa no ESP32) | 20 | Deve ser menor que `d_stop_m`. |
| `wd_ms` (watchdog sem `drive`) | 500 | Maior que o período de envio de comandos. |
| Taxa de envio de `drive` | a definir | Pelo menos o dobro de 1/`wd_ms`. |
| Baud da serial | 115200 | Confirmar no firmware. |
| `ARM_DELAY_MS` | 4000 | Cobre a calibração do chão no celular (SETTLE 1 s + 11 frames); o ESP32 ignora `vis` com `ok:0`. |
| Taxas do celular | `vis` ~10 Hz, `yaw` ~20 Hz, `drive` a cada frame | Mensagens ≤ 160 bytes (testado). |

---

## Ordem sugerida de calibração

1. **Bancada do HC-SR04** (sem carro): alcance útil, ruído, "sem eco" por objeto, tempo de assentamento do servo. Preenche `faults` do mock e `d_clear_m`.
2. **Teste de fumaça no S9** (sem carro): fps e latência da visão. Reavalia `target_w` e as durações em segundos da seção 4.
3. **Carro parado/pista curta:** `speed_at_v_rev_mps`, `yaw_rate_dps`, latência e frenagem. Depois `derive_maneuver_times` preenche `reverse_s` e `turn_s`.
4. **Câmera montada no carro:** altura, inclinação, HFOV, ROI. Refazer os vídeos de referência.
5. **Persistência:** pessoa atravessando; ajustar `guard_*`.

## Pendências de código ligadas a estes parâmetros

- `vision/coverage_log.py`: `WARMUP` em frames (converter para segundos) e constantes da seção 5 (migrar para `Config`).
- `ground.py` e `edge_probe.py`: constantes fora do `Config`.
- `MockRangeHead`: valores padrão (`settle_s`, `max_range_m`, `noise_m`) devem vir da bancada.
- Detecção de obstáculo móvel pela taxa de variação da leitura: não implementada.
## Rumo em linha reta (`heading.*`, `control/heading.py`)

| Parâmetro | Padrão | Status | Como calibrar |
|---|---|---|---|
| `heading.kp_rate` | 0,005 | DEMO | Steer por °/s de giro. Se o carro balança de um lado a outro em reta, diminua. |
| `heading.kh` | 0,02 | DEMO | Steer por grau de rumo acumulado; é ele que cancela o desvio fixo. Se demora a endireitar, aumente aos poucos. |
| `heading.s_max` | 0,4 | DEMO | Teto da correção. Precisa ser maior que o desvio fixo do carro (em unidades de steer). |
| `heading.engage_delay_s` | 0,4 s | DEMO | Ignora a arrancada (kick + escalonamento dos motores). |
| `heading.stale_s` | 0,3 s | DEMO | Sem amostra do giroscópio por esse tempo, não corrige. |

O `psi` que o celular manda ao ESP32 (`{"t":"yaw"}`) vem de outro integrador (`YawIntegrator`, **sem o limite de ±20°**), com o mesmo filtro e a mesma estimativa de viés (feita com as rodas paradas, lida do `state` do ESP32).

Conferir no app antes de rodar: a faixa de status mostra `psi=...` e `rumo: giro=...`. Girar o carro
à mão para a DIREITA tem que dar giro positivo; se der negativo, inverta o sinal em `YawSensor.kt`.
