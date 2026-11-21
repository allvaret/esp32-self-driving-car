# Protocolo celular <-> ESP32 (firmware 0.5.1)

**Caminho 1:** o ESP32 e o UNICO executor de manobras. O celular (S9) nao tem maquina de estados: manda a
*opiniao da visao* (`vis`) e o *rumo do giroscopio* (`yaw`) e, na fonte "visao", o `drive` e o `avoid`.

## Rede
- ESP32 em softAP: SSID `CARRINHO`, senha definida em `car_firmware/secrets.h` (modelo: `secrets.example.h`; sem ele vale `troque-esta-senha`), IP `192.168.4.1`.
- `http://192.168.4.1` -> pagina do visor (pensamento + motivacao). Funciona em qualquer navegador.
- `ws://192.168.4.1:81/` -> WebSocket. Uma mensagem JSON por frame, mesmo formato da serial.
- `http://<ip-do-S9>:8080/stream` (MJPEG da visao), `/logs` (lista) e `/log/<nome>` (download dos logs JSONL gravados no celular). O link aparece no visor.
- Android: ao conectar no CARRINHO, aceitar "rede sem internet / manter conectado".

## Fiacao dos motores (L298N)
| Pino ESP32 | L298N | Saida | Roda (tracao so nas dianteiras; traseiras livres) | No firmware |
|---|---|---|---|---|
| 19 / 16 / 17 | ENA / IN1 / IN2 | OUT1 / OUT2 | dianteira **ESQUERDA** | `curL`, `L`, `trim_left`, `MOTOR_LEFT_INVERT` |
| 32 / 33 / 21 | ENB / IN3 / IN4 | OUT3 / OUT4 | dianteira **DIREITA** | `curR`, `R`, `trim_right`, `MOTOR_RIGHT_INVERT` |

Curva diferencial: `L = v + s*diff_gain`, `R = v - s*diff_gain`; `s > 0` = direita (roda esquerda mais rapida).
Fiacao **medida** pelo giroscopio (log de 20/11: ENA para frente e ENB para tras = +153 graus/s, direita; diagnostico de
21/11 com a 0.5.0: `dir` girou para a esquerda). As versoes 0.4.1 e 0.5.0 tinham ENA = direita: nelas `L`/`R` do
`state`/`diag` estao **trocados** e as curvas sairam ao contrario. Ate a 0.4.0 e desde a 0.5.1 o mapeamento esta certo.

## Papeis
| Quem | Conecta em | Faz |
|---|---|---|
| Visor (2o celular) | pagina web | **Explorar: sensor**, **Explorar: visao**, **PARAR TUDO**; ve estado (fonte e angulo escolhido), radar e pensamento; baixa os logs |
| Cerebro (S9, Python) | WebSocket | recebe goal/state/range/scan; manda `vis` (~10 Hz), `yaw` (~20 Hz), `mode`, `think`; na fonte visao, `drive` (a cada frame) e `avoid` |
| ESP32 | - | **decide e executa** tudo: FRENTE/VARREDURA/DESVIO, reflexos, veto do sensor, varredura por angulo, curva, guarda de reentrada |

**Fonte do goal** (`src`): `sensor` = modo `auto` (so o sensor; a visao e ignorada). `visao` = modo `phone` (o celular manda o `drive`; o ESP32 veta e manobra). Sem `src` vale `sensor`.

## Mensagens que o ESP32 recebe
Toda mensagem deve caber em **160 bytes** (lineBuf da serial); o brain.py garante isso.

| JSON | Efeito |
|---|---|
| `{"t":"goal","g":"explorar","src":"sensor"\|"visao"}` | motivacao do visor: define o modo pela fonte, **arma sozinho** (contagem de 4 s) e repassa a todos |
| `{"t":"goal","g":"parar"}` | **reset completo** (ver abaixo) e avisa todos |
| `{"t":"vis","ok":1,"s":[100,100,90,100,40]}` | opiniao da visao: 5 setores livres 0..100 em -30,-15,0,+15,+30 graus. `ok:0` (calibrando/erro) ou mais velha que `vis_fresh_ms` (300) = sem opiniao. **So o modo phone usa** |
| `{"t":"yaw","psi":-12.3,"r":-45.2}` | rumo acumulado (graus, + = direita, sem limite; so o delta importa) e giro (graus/s). Mais velho que `yaw_fresh_ms` (200) = sem yaw (curva pelo tempo, `turn_dps`) |
| `{"t":"drive","v":0.3,"s":0}` | so no modo phone; vale por `wd_ms` (500). `s` > 0 = direita |
| `{"t":"avoid","deg":-15}` | modo phone, armado: o ESP32 faz o desvio. `deg` (opcional) = angulo sugerido pela visao (setor mais livre): a varredura comeca por ele e ele so e aceito se o sensor nao vetar. **Recusado** se a leitura central e valida, fresca (`fresh_ms`) e `>= d_clear_cm` (visao refutada pelo sensor; log `avoid recusado: ...`). Se chega de novo durante a manobra/ESPERA, a ESPERA termina em manobra completa |
| `{"t":"aim","deg":45}` | so no modo phone (bancada) |
| `{"t":"mode","m":"auto"\|"phone"}` | troca o modo (idempotente: o mesmo modo nao reinicia nada) |
| `{"t":"arm","on":true\|false}` | arma (4 s) / desarma (`false` = reset completo) |
| `{"t":"cfg","v_cruise":0.35}` | muda parametros (na RAM). Excecao: `pwm_min_l`/`pwm_max_l`/`pwm_min_r`/`pwm_max_r` (faixa de duty por roda, -1 = `pwm_min`/`pwm_max`) sao gravados na NVS e lidos no boot |
| `{"t":"think","msg":"...","state":"LIVRE","conf":0.8}` | so repassa a todos (o brain narra so a VISAO) |
| `{"t":"calibrar"}` | so repassa; o cerebro reaprende a cor do chao (a opiniao fica `ok:0` enquanto isso). O visor manda ao clicar em "Explorar: visao" |
| `{"t":"ping"}` | responde `pong` |
| `{"t":"trim","v":0}` | define e grava na NVS o trim aprendido do rumo (unidades de roda) |
| `{"t":"diag","teste":"armar"\|"dir"\|"esq"\|"reto"\|"reto_corr"\|"roda_esq"\|"roda_dir"\|"parar"\|"zerar_trim"}` | diagnostico (secao **Diagnostico** do visor), ver abaixo |

### Diagnostico (visor, sem PC)
- `armar`: arma (4 s) e fica **parado** (`state.st = "DIAG"`, `sub = "parado"`), sem rodar a FSM.
- `dir` / `esq`: `v = 0`, `s = +1` / `-1` por 1 s (gira no lugar). `reto`: `v = v_cruise`, `s = 0` por 2 s **sem** correcao de
  rumo (nem PI nem trim: mede o carro cru). `reto_corr`: igual, com a correcao. `zerar_trim`: trim aprendido = 0 (NVS).
- So com o carro armado (senao responde `{"t":"diag","teste":..,"erro":..}`). Valem o hard stop e o `forwardAllowed`
  (um teste reto sem leitura livre e abortado). Durante e depois do teste a FSM fica suspensa e o carro parado, ate um
  novo goal `explorar` ou o desarme; um reset no meio do diagnostico **nao** retoma a exploracao.
- Ao fim: `{"t":"diag","teste":"dir","L":0.60,"R":-0.60,"psi":74.8,"rate_med":90.0,"n":10,"ms":1000,"trim":0.00}`:
  `L`/`R` = media dos comandos na 2a metade, `psi` = graus girados no teste pelo yaw recebido (+ = direita; `null` sem yaw),
  `rate_med` = mediana do giro na 2a metade, `n` = amostras de yaw, `abortado` (se houve).
- O visor pergunta para onde o carro virou de verdade e cruza com o sinal do `rate_med` (o `psi` so se nao houver giro):
  curva errada e giroscopio certo = fiacao/firmware; curva certa e giro com sinal trocado = `YawSensor.kt`; os dois
  trocados = corrigir os dois.
- `roda_esq` / `roda_dir`: **uma roda so**, para frente, com **duty bruto** em degraus (`d0` 110 ate `d1` 230, passo `dd` 15,
  `step_ms` 300 cada; opcionais na mensagem), a outra parada; sem rampa, trim nem correcao (valem o hard stop e o servo
  parado). O carro gira sobre a roda parada, entao o giro de cada degrau mede a velocidade daquela roda:
  `{"t":"diag","teste":"roda_esq","duty":[110,125,..],"rate":[0.0,8.3,..],"partida":140,"psi":..,"ms":2700}`.
  `rate` = mediana do giro nos ultimos 60% de cada degrau (`null` sem yaw); `partida` = 1o duty com |giro| >= 15 graus/s.
  Esperado: `roda_esq` gira para a direita (`rate` > 0), `roda_dir` para a esquerda. O visor pergunta qual roda girou.

### Reset completo
`goal parar`, `arm false` e o BOOT desarmando levam tudo ao estado inicial: `armed`/`delegated` falsos, modo padrao
(`auto`), FSM zerada (`resetAuto`), comandos zerados, e **limpam `rtcArmed`, `rtcMode` e `rtcGoal`** (um reset de
hardware depois disso nao retoma). O ESP32 emite `{"t":"goal","g":"parar"}` para o visor e o celular acompanharem.
O brain, ao ver o goal `parar` ou o `armed:false`, zera o goal/src lembrados e o integrador de yaw.
Ao desarmar, o ESP32 grava o **trim aprendido do rumo** na NVS (so se mudou; log `trim do rumo salvo: X`).

### Rumo em reta (no ESP32, modos auto e phone)
Trecho reto = `v > 0` e `|s| < hold_s_eps` (FRENTE, VARREDURA, `drive` reto do celular; o brain manda `s = 0`).
`s_corr = -(hold_kp*r + hold_ki*psi_trecho) + trim`, limitado a `+-hold_s_max`, em **unidades de roda**
(`s_corr > 0` = mais forca na roda esquerda). O trim vale desde o 1o ciclo; o PI entra `hold_delay_ms` (100) depois,
se o `yaw` estiver fresco (sem yaw: so o trim, malha aberta). A roda lenta **sobe** primeiro (ate `hold_pwm_max` de duty,
acima do `pwm_max`); a rapida so desce se a lenta saturar. A rampa (`slew_per_s`) vale para o comando base; a correcao
entra por cima dela. O trim integra devagar (`trim_rate`) o termo `hold_ki*psi_trecho`, **nao zera entre trechos**,
fica em `+-trim_max`, e e gravado na NVS ao desarmar e lido no boot. `{"t":"trim","v":0}` (ou `trim 0` na serial) zera.

## Mensagens que o ESP32 emite (serial + WebSocket)
`hello`, `state` (a cada 500 ms), `range` (cada leitura; no modo auto so com clientes WS),
`log`, `cfg_ok`, `pong`, `scan`, `turn` (fim de cada curva), `diag` (fim de cada teste) e os `goal`/`think` repassados.
Campos de `turn` e `diag`: `docs/como_ler_o_log.md`. Ao conectar, cada cliente recebe o `goal` atual (com `src`) e o diario do servo.

`state`: `mode`, `src` (`sensor`/`visao`), `armed`, `esp` (o ESP32 esta no comando), `st`/`sub` (FSM; `sub` = ESPERA, PARADA, OLHA, RE, CURVA, PRESO),
`c` (mediana central em cm, -1 = sem eco), `cls` (`valida`/`livre`/`suspeito`), `deg` (servo), `ang` (angulo escolhido na ultima varredura ou `null`),
`vis`/`yaw` (1 = opiniao/rumo frescos), `L`/`R` (roda esquerda/direita, com a correcao de rumo; podem passar de 1 na roda
lenta), `hold` (0 = fora de reta, 1 = so trim, 2 = corrigindo com yaw), `hc` (correcao aplicada), `trim` (trim aprendido),
`why`, `up`, `rst`, `rest`.

### "Sem eco" em 3 estados
Sempre pela **mediana** e pelo historico, nunca pelo ping bruto:
- **valida**: ha eco.
- **livre**: sem eco **e** (visao fresca com setor 0 >= `vis_free_min`, **ou** nenhuma leitura < `d_clear_cm` nos ultimos `suspect_ms`).
- **suspeito**: sem eco logo depois de leitura valida < `d_clear_cm` (ate `suspect_ms`), **ou** visao fresca com setor 0 < `vis_free_min`.
  Suspeito => para e varre (como DESVIO, sem re inicial). `forwardAllowed` bloqueia o avanco.

### `scan` (diario do servo)
Guardado na RTC e reenviado com `"replay":1` a **todo cliente novo** (se o ESP32 nao ficou sem energia):
`{"k":"aim","deg":-15}` (ordem ao servo), `{"k":"olha","deg":-15,"got":1,"cm":38.1}` (`cm:null` com `got:1` = sem eco; `got:0` = estourou o tempo) e
`{"k":"decide","ang":-15,"x":0,"d":[-30,-15,15,30],"sc":[45,45,38,200]}` (angulo escolhido, notas por posicao em cm, `-1` = nao varrida;
`x:1` = sem saida, `ang:null`). `n` = sequencia, `boot` = numero do boot, `ms` = uptime em ms.

## Logs no celular (nada e transmitido durante a corrida)
O brain grava JSONL em `filesDir/logs`, um arquivo por goal (`explorar_<src>_<data>.jsonl`): por frame (`k:"f"`: `vs`, `r`, `sec`, `e`, `v`, `s`),
tudo que enviou (`k:"tx"`) e toda mensagem recebida do ESP32 (`k:"rx"`: state, range, scan, log, goal). Baixe depois pelo link do visor (`/logs`).
