"""label.py - rotulagem manual: mostra o video frame a frame e grava a tecla pressionada.

Teclas: a = esquerda, d = direita, s = parar, w = frente, q = sair.
Grava (frame, tecla) em label_<video>.csv ao lado do video (modo append).

Uso:
    python -m tools.label assets/simulation_Scar/Reframed_videos/10sec_reframed10.mp4 [--delay 2000]
"""
import argparse
from pathlib import Path

import cv2 as cv
import pandas as pd

KEYS = {ord("a"): "esquerda", ord("d"): "direita", ord("s"): "parar", ord("w"): "frente"}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("video")
    ap.add_argument("--delay", type=int, default=2000, help="ms de espera por frame")
    a = ap.parse_args()

    src = Path(a.video)
    csv_path = src.with_name(f"label_{src.stem}.csv")
    cap = cv.VideoCapture(str(src))
    if not cap.isOpened():
        raise SystemExit(f"Nao consegui abrir {src}")

    counter = 0
    while cap.isOpened():
        ret, frame = cap.read()
        counter += 1
        if not ret:
            print("Fim do video.")
            break
        cv.imshow("frame", frame)
        key = cv.waitKey(a.delay)
        if key == ord("q"):
            break
        if key in KEYS:
            print(f"Frame {counter} - {KEYS[key]} ({key})")
            pd.DataFrame({"frame": [counter], "key": [key]}).to_csv(csv_path, mode="a", header=False, index=False)
    cap.release()
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()
