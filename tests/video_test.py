from vision.ground import *

cap = cv2.VideoCapture("../assets/simulation_Scar/Reframed_videos/12sec_reframed10.mp4")

contador = 0
histograma = None

def draw_free_space(frame, height_vector):
    debug_frame = frame.copy()
    h, w = debug_frame.shape[:2]

    num_points = len(height_vector)
    if num_points == 0:
        return debug_frame

    dx = w / max(num_points - 1, 1)
    points = []

    for i, val in enumerate(height_vector):
        x = int(i * dx)
        # Se val estiver entre 0.0 e 1.0, escala para os pixels do frame:
        pixel_height = val * h if val <= 1.0 else val

        y = int(h - pixel_height)
        y = max(0, min(y, h - 1))
        points.append((x, y))

    # Desenha as linhas
    for i in range(len(points) - 1):
        cv2.line(debug_frame, points[i], points[i + 1], (0, 255, 0), 2, cv2.LINE_AA)

    # Desenha os marcadores
    for pt in points:
        cv2.circle(debug_frame, pt, 4, (0, 0, 255), -1, cv2.LINE_AA)

    return debug_frame

while True:
    ret, frame = cap.read()
    if not ret:
        print("Fim do vídeo ou falha ao ler o arquivo.")
        break

    # Aprende o histograma do chão no primeiro frame
    if contador == 0:

        histograma = learn_floor_hist(frame)

    # 1. Gera a máscara do chão a partir da retroprojeção
    prob_teste = floor_mask(frame, histograma, 0.02)

    # 2. Gera o vetor de altura livre por coluna
    height_livre = free_space_vector(prob_teste)

    # 3. Gera a imagem de debug combinando o frame com o vetor
    debug_vis = draw_free_space(frame, height_livre)

    # Exibição das janelas
    cv2.imshow("Frame Original", frame)
    cv2.imshow("Mascara de Chao (Prob)", prob_teste)
    cv2.imshow("Debug - Espaco Livre", debug_vis)

    contador += 1

    # --- MODO SLIDESHOW INTERATIVO ---
    # waitKey(0) pausa a execução e espera qualquer tecla para avançar 1 frame.
    # Pressione 'q' para sair do loop.
    key = cv2.waitKey(0) & 0xFF
    if key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()