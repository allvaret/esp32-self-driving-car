# Exemplos de log

Um arquivo de cada tipo, pequenos, para ver o formato (decodificação em [`../como_ler_o_log.md`](../como_ler_o_log.md)):

| Arquivo | O que é |
|---|---|
| `carrinho_2026-11-26T15-39-47-755Z.jsonl` | log do **visor** (só mensagens do ESP32; botão "Baixar log") |
| `corrida_2026-11-26_14h57m45_visao.jsonl` | log do **celular** desde a 0.7.0 (registros `k`/`w`), corrida com fonte "visão" |
| `corrida_2026-11-26_14h57m45_visao.mp4` | vídeo da mesma corrida (conversão do `.mjpg` por `tools.video_to_mp4`) |
| `explorar_visao_20261126_110817.jsonl` | formato do celular **antes** da 0.7.0 (`explorar_*`) |

Os logs brutos completos (e os `.mjpg`) não ficam no repositório: baixe-os do Release do GitHub e coloque em `docs/logs/` (pasta ignorada pelo git).
