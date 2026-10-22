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
| `control.v_cruise / v_slow / v_rev / v_turn` | 0,5 / 0,2 / 0,3 / 0,3 | DEMO | Escolher velocidades que o carro realmente consiga manter; anotar a velocidade em m/s de cada uma. |
| Latência comando → movimento | desconhecida | MEDIR | Filmar o carro e marcar o instante do envio do comando (log no celular) e o início do movimento. |
| Distância de frenagem de `v_cruise` | desconhecida | MEDIR | Mandar `v = 0` em pleno cruzeiro e medir quanto ele anda até parar. |

## 2. Sensor HC-SR04 + servo (`control.*`, `hardware/hal.py`)

| Parâmetro | Valor atual | Status | Como obter |
|---|---|---|---|
| `control.d_stop_m` | 0,50 m | DEMO | `d_stop ≥ v_cruise[m/s] · (latência visão + comunicação + ciclo do sensor) + frenagem + margem`. |
| `control.d_clear_m` | 1,00 m | DEMO | Distância a partir da qual uma leitura válida "refuta" a visão. Escolher com base no alcance confiável medido. |
| `control.none_score_m` | 0,50 m | DEMO | Valor dado a um lado sem eco ao escolher a curva. Revisar com a taxa de "sem eco" medida por tipo de superfície. |
| `control.scan_deg` | 45° | DEMO | Ângulo da olhada lateral. Deve ficar próximo do ângulo girado na curva (`turn_s`). |
| `control.look_timeout_s` | 1,0 s | DEMO | Maior que o tempo de assentamento do servo medido. |
| Tempo de assentamento do servo (`MockRangeHead.settle_s`) | 0,15 s | MEDIR | Cronometrar de `aim()` até a leitura estável, para -45°, 0° e +45°. |
| Alcance útil (`MockRangeHead.max_range_m`) | 2,0 m | MEDIR | Teste de bancada: o HC-SR04 especifica ~2–400 cm, mas o alcance útil em objetos pequenos costuma ser menor. |
| Ruído / taxa de "sem eco" por tipo de objeto | 1 cm / 0 % | MEDIR | Teste de bancada de 1 h: perna de cadeira, tecido do sofá, parede em ângulo, pessoa andando. Alimenta `faults` do mock. |
| Ciclo mínimo entre leituras | ≥ 60 ms (especificação) | MEDIR | Confirmar no firmware. Limita a taxa de varredura. |
| Posição do sensor em relação à câmera (paralaxe) | não modelada | MEDIR | Medir o deslocamento em cm; compensar o ângulo do servo em curta distância. |

## 3. Manobra do controller (`control.*`)

| Parâmetro | Valor atual | Status | Como obter |
|---|---|---|---|
| `control.stop_s` | 0,3 s | DEMO | Tempo para o carro parar de fato. Usar a frenagem medida. |
| `control.reverse_s` | 0,8 s | DEMO | Derivar com `derive_maneuver_times(cfg, reverse_m, turn_deg)` a partir de `speed_at_v_rev_mps`. |
| `control.turn_s` | 1,0 s | DEMO | Idem, a partir de `yaw_rate_dps` (alvo: girar ≈ `scan_deg`). |
| `control.avoid_retries` | 2 | DEMO | Quantas vezes tentar de novo (ré maior + nova olhada) antes do HOLD. |
| `control.clear_s` | 0,2 s | HIPÓTESE | Tempo "limpo" para voltar a CRUISE (equivale a 3 ticks a 10 fps). Revisar com o fps real do S9. |
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

## 9. Comunicação e ESP32 (propostas, sem firmware ainda)

| Item | Valor proposto | Observação |
|---|---|---|
| `hard_stop_cm` (parada reflexa no ESP32) | 20 | Deve ser menor que `d_stop_m`. |
| `wd_ms` (watchdog sem `drive`) | 500 | Maior que o período de envio de comandos. |
| Taxa de envio de `drive` | a definir | Pelo menos o dobro de 1/`wd_ms`. |
| Baud da serial | 115200 | Confirmar no firmware. |

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