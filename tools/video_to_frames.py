"""tools/video_to_frames.py - extrai frames de um video para uma pasta de JPEGs.

Serve para (1) levar frames ao app Android quando o OpenCV de la nao le video, e
(2) simular a camera do carro (imagens soltas). Rode no PC, da raiz do repositorio:
    python -m tools.video_to_frames videoParede.mp4 android/app/src/main/assets/frames
    python -m tools.video_to_frames videoParede.mp4 out_frames --max-frames 120 --quality 90
Os arquivos saem como frame_0000.jpg, frame_0001.jpg, ... (ordem alfabetica = ordem do video).
"""
import argparse
import os

import cv2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("out_dir")
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--every", type=int, default=1, help="guarda 1 a cada N frames")
    ap.add_argument("--quality", type=int, default=90, help="qualidade JPEG 1-100")
    a = ap.parse_args()

    cap = cv2.VideoCapture(a.video)
    if not cap.isOpened():
        raise SystemExit(f"Nao abriu: {a.video}")
    os.makedirs(a.out_dir, exist_ok=True)
    fps = cap.get(cv2.CAP_PROP_FPS) or 0
    i = saved = 0
    while True:
        ok, frame = cap.read()
        if not ok or (a.max_frames and saved >= a.max_frames):
            break
        if i % a.every == 0:
            cv2.imwrite(os.path.join(a.out_dir, f"frame_{saved:04d}.jpg"), frame,
                        [cv2.IMWRITE_JPEG_QUALITY, a.quality])
            saved += 1
        i += 1
    cap.release()
    eff = fps / a.every if fps else 0
    print(f"{saved} frames em {a.out_dir} (fps efetivo ~{eff:g}; use --fps {eff:g} no modo pasta)")


if __name__ == "__main__":
    main()