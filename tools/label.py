import cv2 as cv
import pandas as pd
from pathlib import Path

input_path = '/assets/simulation_Scar/Reframed_videos/24sec_reframed10.mp4'
path_obj = Path(input_path)
output_name = f"label_{path_obj.stem}.csv"


cap = cv.VideoCapture(input_path)
counter = 0
while cap.isOpened():
    ret, frame = cap.read()
    counter += 1

    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    colored = cv.cvtColor(frame, cv.COLOR_BGR2RGB)

    espera = cv.waitKey(2000)
    cv.imshow('frame', colored)
    if espera == ord('q'):
        break
    keys = [ord('a'), # Esquerda
             ord('d'), # Direita
             ord('s'), # Parar 
             ord('w')] # Frente
    
    if espera in keys:
        print(f'Frame {counter} - Key pressed: {espera}')
        pd.DataFrame({'frame': [counter], 'key': [espera]}).to_csv(f'assets\\simulation_Scar\\Reframed_videos\\{output_name}', mode='a', header=False, index=False)
        

cap.release()
cv.destroyAllWindows()