import cv2, numpy as np

def learn_floor_hist(frame_bgr, roi_poly):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8) # Mascara apenas com zeros
    # Preenchemos a mascara com nosso ROI definido em forma de poligono (trapezio)
    cv2.fillPoly(mask, [roi_poly], 255)

    # Calcula o histograma de uma lista de imagens, so pixels pintados em mask contam.
    # histSize eh o número de bins(agrupamento) em cada canal. Hue em 30 faixas e Saturation em 32.
    hist = cv2.calcHist([hsv], [0, 1], mask, [30, 32], [0, 180, 0, 256]) # Range min e max de cada canal
    cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX) # Normaliza o histograma entre 0 e 255, com base nas cores mais e menos dominantes
    return hist

def floor_mask(frame_bgr, hist, thr):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    prob = cv2.calcBackProject([hsv], [0, 1], hist, [0, 180, 0, 256], 1)
    # TODO 1: blur leve em prob, threshold, depois morfologia (open/close)
    return mask

def free_space_vector(mask, bin_w=8, run=5):
    # para cada faixa de bin_w colunas, suba de baixo p/ cima
    # TODO 2: pare quando achar `run` pixels seguidos de "não chão"
    # retorne a altura (em px) livre de cada faixa
    return heights