import cv2, numpy as np

def preprocess(frame_bgr):
    h, w = frame_bgr.shape[:2]
    TARGET_W = 320
    escala = TARGET_W / w # Mantem proporcao
    w_out = round(w * escala)
    h_out = round(h * escala)

    resize = cv2.resize(frame_bgr, [w_out,h_out], interpolation=cv2.INTER_AREA)
    blur = cv2.GaussianBlur(resize,(3,3),0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    return hsv


def learn_floor_hist(frame_bgr, roi_poly):
    hsv = preprocess(frame_bgr)
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


# def free_space_vector(mask, strip_w=8, run=5):
#     # para cada faixa de strip_w colunas, suba de baixo p/ cima
#     height, width = mask.shape
#     width_cut = width
#     # TODO 2: pare quando achar `run` pixels seguidos de "não chão"
#     # retorne a altura (em px) livre de cada faixa
#     return heights

