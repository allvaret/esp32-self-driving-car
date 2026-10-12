import cv2
import matplotlib.pyplot as plt
import numpy as np
from ground import *


imagem = cv2.imread('assets/simulation_Scar/Reframed_videos/screenshot_first_frame.png')
imagem = cv2.resize(imagem, (320, 480))

h, w = imagem.shape[:2]
roi_poly = np.array([
    [0.15*w, 0.95*h],   # inferior esquerdo
    [0.85*w, 0.95*h],   # inferior direito
    [0.65*w, 0.75*h],   # superior direito
    [0.35*w, 0.75*h],   # superior esquerdo
], dtype=np.int32)


histograma = learn_floor_hist(imagem, roi_poly)

def floor_mask(frame_bgr, hist, thr):
    hsv = preprocess(frame_bgr)
    prob_map = cv2.calcBackProject([hsv], [0, 1], hist, [0, 180, 0, 256], 1) # Mapa de prob. com base no histograma
    blur = cv2.medianBlur(prob_map, 3)
    mask = cv2.threshold(blur, thr, 255, cv2.THRESH_BINARY) # Binarização com base em THR (Guarda thr e img output)
    kernel = np.ones((3, 3), np.uint8)
    opening = cv2.morphologyEx(mask[1], cv2.MORPH_OPEN, kernel) # erosão seguida de uma dilatação.
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel) # dilatação seguida de uma erosão
    return closing


prob_teste = floor_mask(imagem,histograma, 9)
# plt.imshow(prob_teste)
# plt.show()

cv2.polylines(imagem, [roi_poly], True, 255, 2)
img_rgb = cv2.cvtColor(imagem, cv2.COLOR_BGR2RGB)

fig, axes = plt.subplots(1, 2, figsize=(10,5))
axes[0].imshow(img_rgb)
axes[0].set_title('Original image')

axes[1].imshow(prob_teste)
axes[1].set_title('Mascara do chao')
plt.show()

cv2.waitKey(0)
cv2.destroyAllWindows()