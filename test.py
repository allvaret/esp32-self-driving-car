import cv2
import matplotlib.pyplot as plt
import numpy as np
from ground import *


imagem = cv2.imread('assets/simulation_Scar/Reframed_videos/screenshot_first_frame.png')

h, w = imagem.shape[:2]
roi_poly = np.array([
    [int(0.15 * w), int(0.95 * h)],   # inferior esquerdo
    [int(0.85 * w), int(0.95 * h)],   # inferior direito
    [int(0.65 * w), int(0.75 * h)],   # superior direito
    [int(0.35 * w), int(0.75 * h)],   # superior esquerdo
], dtype=np.int32)



histograma = learn_floor_hist(imagem)
prob_teste = floor_mask(imagem,histograma, 0.02)



cu = free_space_vector(prob_teste)
print(cu)



plt.imshow(prob_teste)
plt.show()

#
#
img_rgb = cv2.polylines(imagem, [roi_poly], True, 255, 2)
img_rgb = cv2.cvtColor(imagem, cv2.COLOR_BGR2RGB)

fig, axes = plt.subplots(1, 2, figsize=(10,5))
axes[0].imshow(img_rgb)
axes[0].set_title('Original image')

axes[1].plot(cu, marker='o')
axes[1].set_title('Distribuição de 1 Dimensão')
plt.xlabel('Valores')
plt.ylabel('Altura')
plt.show()

cv2.waitKey(0)
cv2.destroyAllWindows()