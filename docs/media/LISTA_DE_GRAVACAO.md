# Lista de gravação

Tomadas que precisam ser feitas com outra câmera (celular de apoio) e que entram no README nos marcadores `<!-- MIDIA n -->`.
Regras gerais: sem pessoas, rostos, janelas com vista, documentos ou telas com dados pessoais no quadro; limpe o chão de objetos da casa fora da cena; luz acesa e estável.

| # | Tomada | Duração | Enquadramento | Onde entra |
|---|---|---|---|---|
| 1 | **Carro andando e desviando de obstáculo.** Modo *Explorar: visao*. Um obstáculo (caixa) a ~1,5 m do carro; o carro anda, para, varre com o servo, escolhe o lado e passa. | 15 a 20 s | Câmera na altura de ~30 cm do chão, 3/4 de frente/lado, horizontal, carro e obstáculo inteiros, sem cortar o servo. Sem zoom digital. | topo do README (MIDIA 1), vira GIF de até 2 MB |
| 2 | **Batida + ré + desvio.** Obstáculo baixo que o sonar não vê (ex.: uma folha rígida na altura do chão ao lado) ou a parede em ângulo. O carro empurra, o firmware detecta a batida, dá ré e desvia. | 10 a 15 s | Mesmo enquadramento da 1, mais aberto para ver o contato e o recuo. | seção Resultados (MIDIA 2) |
| 3 | **Visor no celular em tela cheia.** Página `http://192.168.4.1` com o carro em movimento: estado, radar, imagem da visão **expandida** (toque na imagem). | 15 s de gravação de tela | Gravação de tela nativa, retrato; ocultar notificações (modo avião com Wi-Fi ligado) para não aparecerem apps nem horário. | O que faz (MIDIA 3) |
| 4 | **Log baixado pelo /logs.** Página `http://<ip-do-celular>:8080/logs` com a lista de corridas e um download em andamento. | print (ou 8 s) | Captura de tela do navegador, retrato, sem outras abas à mostra. | Como replicar (MIDIA 4) |
| 5 | **Foto do hardware com a fiação atual.** Carro montado: ESP32, L298N, HC-SR04 no servo, divisor do ECHO, alimentação do servo, celular fixado. | 2 fotos | (a) de cima, tampa aberta, fios legíveis; (b) lateral, mostrando o sensor e o celular. Fundo liso, boa luz; sem etiquetas ou endereço. | Como replicar (MIDIA 5), ao lado da tabela de pinagem |

Depois de gravar, coloque os arquivos brutos em `docs/media/` (vídeos grandes não entram no git: converta para GIF de até 2 MB ou MP4 curto) e troque o marcador do README pela imagem.
