import cv2
import matplotlib.pyplot as plt
import numpy as np
from ground import *


imagem = cv2.imread('assets/simulation_Scar/Reframed_videos/screenshot_first_frame.png')
imagem = cv2.resize(imagem, (640, 480))

h, w = imagem.shape[:2]
roi_poly = np.array([
    [0.15*w, 0.95*h],   # inferior esquerdo
    [0.85*w, 0.95*h],   # inferior direito
    [0.65*w, 0.75*h],   # superior direito
    [0.35*w, 0.75*h],   # superior esquerdo
], dtype=np.int32)

cv2.imshow('imagem_Pura', imagem, )
# imagem = cv2.polylines(imagem, [roi_poly], True, 255, 2)
# cv2.imshow('imagem_Trapezio', imagem)


histograma = learn_floor_hist(imagem, roi_poly)
def floor_mask(frame_bgr, hist):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    prob = cv2.calcBackProject([hsv], [0, 1], hist, [0, 180, 0, 256], 1)
    return prob

prob_teste = floor_mask(imagem,histograma)


plt.imshow(prob_teste)
plt.show()

cv2.waitKey(0)
cv2.destroyAllWindows()