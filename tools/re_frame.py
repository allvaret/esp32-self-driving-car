"""re_frame.py - reduz o FPS de um video descartando frames (ex.: 30 -> 10 fps).

Uso:
    python -m tools.re_frame entrada.mp4 --fps 10            # gera entrada_reframed10.mp4
    python -m tools.re_frame entrada.mp4 --fps 10 --out saida.mp4
"""
import argparse
from pathlib import Path

import cv2


def reframe(input_path, target_fps, output_path=None):
    cap = cv2.VideoCapture(str(input_path))
    if not cap.isOpened():
        raise SystemExit(f"Nao consegui abrir {input_path}")
    src = Path(input_path)
    if output_path is None:
        output_path = src.with_name(f"{src.stem}_reframed{int(target_fps)}{src.suffix}")

    source_fps = cap.get(cv2.CAP_PROP_FPS)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    skip_interval = max(1, round(source_fps / target_fps))

    out = cv2.VideoWriter(str(output_path), cv2.VideoWriter_fourcc(*"mp4v"), target_fps, (width, height))
    frame_idx = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if frame_idx % skip_interval == 0:
            out.write(frame)
        frame_idx += 1
    cap.release()
    out.release()
    return output_path


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("video")
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    print(f"Video salvo em: {reframe(a.video, a.fps, a.out)}")
