import cv2, numpy as np

def preprocess(frame_bgr, target_w = 320):
    h, w = frame_bgr.shape[:2]
    escala = target_w / w # Mantem proporcao
    h_out = max(1, round(h * escala))

    interp = cv2.INTER_AREA if escala < 1 else cv2.INTER_LINEAR
    resize = cv2.resize(frame_bgr, [target_w,h_out], interpolation=interp)
    blur = cv2.GaussianBlur(resize,(3,3),0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    return hsv


def get_roi(img, x0=0.15, x1=0.85, x2=0.65, x3=0.35, y0=0.95, y1=0.95, y2=0.75, y3=0.75):
    h, w = img.shape[:2]
    roi_poly = np.array([
        [int(x0 * w), int(y0 * h)],   # inferior esquerdo
        [int(x1 * w), int(y1 * h)],   # inferior direito
        [int(x2 * w), int(y2 * h)],   # superior direito
        [int(x3 * w), int(y3 * h)],   # superior esquerdo
    ], dtype=np.int32)
    return  roi_poly


def learn_floor_hist(frame_bgr):
    hsv = preprocess(frame_bgr)
    roi_poly = get_roi(hsv)
    mask = np.zeros(hsv.shape[:2], np.uint8) # Mascara apenas com zeros
    # Preenchemos a máscara com nosso ROI definido em forma de polígono (trapézio)
    cv2.fillPoly(mask, [roi_poly], 255)
    # Calcula o histograma do frame, so pixels em mask contam.
    # histSize eh o número de bins(agrupamento) em cada canal. Hue em 30 faixas e Saturation em 32.
    hist = cv2.calcHist([hsv], [0, 1], mask, [30, 32], [0, 180, 0, 256]) # Range min e max de cada canal
    cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX) # Normaliza o histograma entre 0 e 255, com base nas cores mais e menos dominantes
    return hist


def floor_mask(frame_bgr, hist, thr):
    hsv = preprocess(frame_bgr)
    prob_map = cv2.calcBackProject([hsv], [0, 1], hist, [0, 180, 0, 256], 1) # Mapa de prob. com base no histograma
    blur = cv2.medianBlur(prob_map, 3)
    mask = cv2.threshold(blur, thr, 255, cv2.THRESH_BINARY) # Binarização com base em THR (Guarda thr e img output)
    kernel = np.ones((3, 3), np.uint8)
    opening = cv2.morphologyEx(mask[1], cv2.MORPH_OPEN, kernel) # erosão seguida de uma dilatação.
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel) # dilatação seguida de uma erosão
    return closing


def free_space_vector(mask, strip_w=8, run=5, min_floor_frac = 0.5):
    # para cada faixa de strip_w colunas, suba de baixo p/ cima
    height, width = mask.shape
    width_bin = width // strip_w

    grid = (mask[:, :width_bin*strip_w] > 0).reshape(height, width_bin, strip_w).mean(axis=2) >= min_floor_frac
    count = np.zeros(width_bin, int)         # um contador por faixa
    free  = np.full(width_bin, height)            # padrão: livre até o topo
    done  = np.zeros(width_bin, bool)        # faixas já encerradas

    for y in range(height-1, -1, -1):     # de baixo para cima
        count = np.where(grid[y], 0, count + 1)   # zera onde é chão, soma onde não é
        hit = (count >= run) & ~done              # faixas que atingiram `run` agora
        free[hit] = height - y - run                 # sua conta | y, puxa do grid?
        done |= hit

    return free / height

