import cv2

# 1. Carregar a imagem (OpenCV carrega em BGR)
img_bgr = cv2.imread('eu_e_meu_amor_popeys.jpg')
img_bgr = cv2.resize(img_bgr, (640, 480))

# 2. Converter para HSV (para limiarização, detecção de cor, etc.)
img_hsv = cv2.COLOR_BGR2HSV if hasattr(cv2, 'COLOR_BGR2HSV') else None
hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

# 3. Processamento no espaço HSV (Exemplo: separar os 3 canais)
h, s, v = cv2.split(hsv)

# PARA EXIBIR A IMAGEM HSV COMPLETA NA TELA:
# Você DEVE reconverter para BGR antes de passar para cv2.imshow
img_para_exibir = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
cv2.imshow('Imagem Original (Reconvertida de HSV)', img_para_exibir)

# PARA INSPECIONAR OS CANAIS INDIVIDUALMENTE:
# Exiba os canais isolados como mapas de cinza (grayscale)
cv2.imshow('Canal H (Matiz 0-179)', h)
cv2.imshow('Canal S (Saturação 0-255)', s)
cv2.imshow('Canal V (Brilho/Valor 0-255)', v)
print(h)
print()
print(s)
print()
print(v)

cv2.waitKey(0)
cv2.destroyAllWindows()