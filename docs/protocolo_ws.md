# Protocolo celular <-> ESP32 (firmware 0.2)

## Rede
- ESP32 em softAP: SSID `CARRINHO`, senha `carrinho142`, IP `192.168.4.1`.
- `http://192.168.4.1` -> pagina do visor (pensamento + motivacao). Funciona em qualquer navegador.
- `ws://192.168.4.1:81/` -> WebSocket. Uma mensagem JSON por frame, mesmo formato da serial.
- Android: ao conectar no CARRINHO, aceitar "rede sem internet / manter conectado".

## Papeis
| Quem | Conecta em | Faz |
|---|---|---|
| Visor (2o celular) | pagina web | escolhe motivacao, modo, PARAR TUDO; ve estado, radar e pensamento |
| Cerebro (S9, Python) | WebSocket | recebe goal/state/range; manda drive/aim (>= 10 Hz) e think |
| ESP32 | - | reflexo (parada por distancia, watchdog 500 ms, limite de re) e repasse |

## Mensagens que o ESP32 recebe
| JSON | Efeito |
|---|---|
| `{"t":"drive","v":0.3,"s":0}` | so no modo phone; vale por `wd_ms` |
| `{"t":"aim","deg":45}` | so no modo phone |
| `{"t":"mode","m":"auto"\|"phone"}` | troca o modo |
| `{"t":"arm","on":true\|false}` | arma (3 s) / desarma |
| `{"t":"cfg","v_cruise":0.35}` | muda parametros |
| `{"t":"goal","g":"explorar"\|"reto"\|"parar"}` | motivacao: repassa a todos; `parar` desarma; no modo auto qualquer outra arma |
| `{"t":"think","msg":"...","state":"LIVRE","conf":0.8}` | so repassa a todos (pensamento do cerebro) |
| `{"t":"ping"}` | responde `pong` |

## Mensagens que o ESP32 emite (serial + WebSocket)
`hello`, `state` (a cada 500 ms), `range` (cada leitura; no modo auto so com clientes WS),
`log`, `cfg_ok`, `pong`, e os `goal`/`think` repassados. Ao conectar, cada cliente recebe o `goal` atual.