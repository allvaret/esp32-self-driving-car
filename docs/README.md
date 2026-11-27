# Documentação

Índice dos documentos do projeto. O resumo e a visão geral estão no [README](../README.pt-BR.md) (em inglês: [README.md](../README.md)).
Política de idioma: documentos técnicos em português; `architecture.md` e `engineering-decisions.md` em inglês; READMEs nos dois idiomas.

| Documento | Para quê |
|---|---|
| [architecture.md](architecture.md) | arquitetura atual (en): camadas, firmware, visor |
| [engineering-decisions.md](engineering-decisions.md) | decisões de projeto, alternativas descartadas e evidências (en) |
| [protocolo_ws.md](protocolo_ws.md) | protocolo celular ↔ ESP32 (WebSocket/JSON), fiação dos motores, comandos |
| [parametros.md](parametros.md) | parâmetros do sistema: valor atual, status e como ajustar a outro hardware |
| [como_ler_o_log.md](como_ler_o_log.md) | formato dos logs e do vídeo da corrida, e como convertê-los |
| [android.md](android.md) | compilar e instalar o app Android (Chaquopy) |
| [desenvolvimento/fluxo_de_execucao.md](desenvolvimento/fluxo_de_execucao.md) | caminho de um quadro até o motor, quem decide em cada modo |
| [desenvolvimento/diario_de_campo.md](desenvolvimento/diario_de_campo.md) | testes de campo, achados e checklist da próxima sessão |
| [exemplos_de_log/](exemplos_de_log/) | um log de cada tipo, pequeno |
| [media/LISTA_DE_GRAVACAO.md](media/LISTA_DE_GRAVACAO.md) | tomadas de vídeo/foto planejadas para o README |

## Por onde começar a ler o código

Siga o caminho de um quadro, do celular ao motor:

1. `main.py`: `StreamRunner.process_rgba_frame` recebe o quadro da câmera e devolve a opinião da visão em JSON. Só visão, nenhuma decisão de manobra.
2. `vision/ground.py`: aprende o histograma do chão e gera a máscara e os 5 setores de espaço livre.
3. `vision/decision.py`: converte a confiança relativa `r` em LIVRE / INCERTO / OBSTÁCULO (histerese, tempos em segundos). `vision/motion.py` gera o índice de movimento da câmera.
4. `brain.py`: dono da conexão WebSocket (`tools/ws_link.py`); envia `vis`, `yaw` (`control/heading.py`) e, no modo visão, `drive`/`avoid`; grava o log (`tools/flight_log.py`, `tools/video_log.py`).
5. `car_firmware/car_firmware.ino`: a FSM (FRENTE → VARREDURA → DESVIO), parada de emergência, watchdog, rumo em reta; `web_ui.h` é a página do operador.
6. `sim/check_firmware.py` e `tests/`: como cada comportamento é verificado.

Fora do caminho ao vivo (legado, só replay e testes): `control/controller.py` (controlador v1 congelado), `control/mocks.py` e `replay/`.

Restrições que valem em todo o código: no celular roda Python 3.10 com OpenCV 4.5.1 headless (Chaquopy); no PC, Python 3.10/3.11 com OpenCV 4.8; a visão usa uma thread do OpenCV; todo tempo de decisão é em segundos, não em quadros. Valores ajustáveis ficam em `config.py`.
