# Protocolo celular <-> ESP32 (firmware 0.7.0)

**Arquitetura "ESP32 executor":** o ESP32 e o UNICO executor de manobras. O celular (S9) nao tem maquina de estados: manda a
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
| 13 / 16 / 17 | ENA / IN1 / IN2 | OUT1 / OUT2 | dianteira **ESQUERDA** | `curL`, `L`, `trim_left`, `MOTOR_LEFT_INVERT` |
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
| `{"t":"vis","ok":1,"s":[100,100,90,100,40],"m":7.4}` | opiniao da visao: 5 setores livres 0..100 em -30,-15,0,+15,+30 graus. `ok:0` (calibrando/erro) ou mais velha que `vis_fresh_ms` (300) = sem opiniao. **So o modo phone usa os setores.** `m` (0.7.0, opcional) = movimento da camera (`vision/motion.py`: diferenca media da imagem em 0,3 s, 0..255; parado ~1, andando ~5..40); vai **mesmo com `ok:0`** e vale nos DOIS modos: e o que detecta a BATIDA |
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
| `{"t":"diag","teste":"armar"\|"dir"\|"esq"\|"reto"\|"reto_corr"\|"roda_esq"\|"roda_dir"\|"servo"\|"parar"\|"zerar_trim"}` | diagnostico (secao **Diagnostico** do visor), ver abaixo |

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
- `roda_esq` / `roda_dir`: **uma roda so**, para frente, com **duty bruto** em degraus **descendo** (de `d1` 220 ate `d0` 45,
  passo `dd` 25, `step_ms` 600 cada e o 1o com o dobro para a roda embalar; opcionais na mensagem), a outra parada; sem
  rampa, trim nem correcao (valem o hard stop e o servo parado). O carro gira sobre a roda parada, entao o giro de cada
  degrau mede a velocidade daquela roda:
  `{"t":"diag","teste":"roda_esq","duty":[220,195,..,45],"rate":[310.2,..],"dr":[-4,..],"minimo":95,"psi":..,"ms":5400}`.
  `rate` = mediana do giro na 2a metade de cada degrau (`null` sem yaw); `dr` = quanto o giro ainda mudava nessa janela
  (grande = nao assentou); `minimo` = ultimo degrau, descendo, ainda com |giro| >= 15 graus/s (menor duty que mantem a
  roda girando; `null` = nao girou). Ate a 0.5.1 os degraus subiam (110..230, 300 ms) e o campo era `partida`: degraus
  curtos a partir do zero mediam a aceleracao do carro, nao a velocidade da roda.
  Esperado: `roda_esq` gira para a direita (`rate` > 0), `roda_dir` para a esquerda. O visor pergunta qual roda girou e
  avisa se a roda gira igual em qualquer duty (o PWM nao a controla).

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
Campos de `turn` e `diag`: `docs/como_ler_o_log.md`. Ao conectar, cada cliente recebe o `goal` atual (com `src`), um `log` com a versao e a frequencia **real** de cada PWM
(`car_firmware 0.6.3 | pwm: servo GPIO4 50 Hz, ENA GPIO13 parado, ENB GPIO32 1000 Hz`; `parado` = duty 0 (o core le 0 Hz), `ERRO sem PWM: GPIOx` =
o pino nao recebe PWM) e o diario do servo.

### Teste do servo do sensor (0.6.1)
`{"t":"diag","teste":"servo"}`: **nao precisa armar** (recusado com o carro andando: a FSM tambem mexe no servo). Motores
parados; o servo aponta -60, -30, 0, +30, +60 (900 ms cada) e volta ao centro. Resposta:
`{"t":"diag","teste":"servo","deg":[-60,-30,0,30,60],"cm":[40.0,null,120.0,-1,90.0],"us":[2160,1830,1500,1170,840],"hz":50}`
(`cm`: `null` = sem eco, `-1` = sem leitura assentada; `us` = pulso pedido; `hz` = PWM real no pino do servo). Com algo
perto de UM lado, distancias iguais em todos os angulos = o sensor nao girou (fio de sinal/5 V/GND do servo).

`state`: `mode`, `src` (`sensor`/`visao`), `armed`, `esp` (o ESP32 esta no comando), `st`/`sub` (FSM; `sub` = ESPERA, PARADA, OLHA, RE, CURVA, PRESO),
`c` (mediana central em cm, -1 = sem eco), `cls` (`valida`/`livre`/`suspeito`), `deg` (servo), `ang` (angulo escolhido na ultima varredura ou `null`),
`vis`/`yaw` (1 = opiniao/rumo frescos), `L`/`R` (roda esquerda/direita, com a correcao de rumo; podem passar de 1 na roda
lenta), `hold` (0 = fora de reta, 1 = so trim, 2 = corrigindo com yaw), `hc` (correcao aplicada), `trim` (trim aprendido),
`cred` (credito de re em s a `v_rev`, ate `max_rev_ms` - 250), `esc` (fugas seguidas sem andar livre), `mov` (ultimo `m` da
camera, -1 = sem `m` fresco: sem deteccao de batida), `stl` (ms empurrando sem movimento na janela atual; batida em
`stall_ms`), `bat` (batidas seguidas), `why`, `up`, `rst`, `rest`.

### Batida (0.7.0)
O sonar nao ve tudo (perna fina, objeto baixo, canto): o carro encostava e seguia empurrando. Agora, com as duas rodas
para a frente em reta (comando base > 0,05, nas duas fontes e fora do DESVIO) e **nada mostrando movimento** por
`stall_ms` (1500), o ESP32 trata como obstaculo. "Movimento" = camera `m > stall_cam_max` (3,0), OU giro
`|r| >= stall_yaw_dps` (20), OU sonar central mudando `stall_sonar_cm` (8) desde o inicio da janela. Sem `m` fresco
(celular fora/sem camera) a janela nao abre: o sonar sozinho erraria (le o chao/um reflexo constante andando).
- Emite `{"t":"log","msg":"batida: rodas empurrando e nada se mexe ha 1500 ms (camera m=0.8, 1 seguida(s)): para, re e desvia"}`;
  no modo phone tambem `ESP32 assumiu: batida` (e devolve ao celular no fim, como no veto).
- O credito somado com as rodas presas (fantasma) sai da fila. DESVIO sem a ESPERA da guarda: PARADA -> **RE** de pelo
  menos `stall_rev_ms` (500; cega e curta, o carro acabou de vir de tras; o credito real pode dar mais, ate `max_rev_ms`)
  -> varredura -> curva de pelo menos `stall_turn_deg` x batidas seguidas (45, 90..., ate `escape_deg`), mesmo que a
  varredura ache um angulo menor (o sonar NAO viu o obstaculo). `why` = `batida: desvia -45`.
- `bat` volta a 0 depois de `stall_free_ms` (3000) andando COM movimento. Sem saida na varredura: fluxo normal do "sem saida".

### Credito de re e fuga (0.6.0)
A re e cega (o sensor olha so para a frente). O ESP32 guarda uma fila de ate 6 trechos retos `{psi, credito}`: em reta
(`|s| < hold_s_eps`, rodas iguais) soma `v*dt` do comando base; trecho novo quando o rumo muda > 10 graus. A re **consome**
o credito, do trecho mais novo para tras, e so conta os trechos com `|psi_agora - psi_trecho| <= cred_psi_tol` (sem yaw: a
fila zera a cada curva). Total limitado a `cred_max`.
- Desvio com angulo: re = `min(reverse_ms, credito)`; menos que `cred_min_ms` = gira direto, sem re.
- Sem saida (nenhum angulo >= `d_stop`): ate `avoid_retries` vezes, re pelo credito e nova olhada; depois **fuga**: gira
  `escape_deg` para fora do cone varrido, sempre para o **mesmo lado** (escolhido na 1a fuga pela maior soma de notas;
  empate = esquerda), sem re. Depois de `escape_max` fugas sem andar livre (>= 30% de `cred_max` num trecho), **PRESO**.
  As fugas zeram ao andar livre. `why`: `sem saida: fuga -90 (1/4)`.

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
