"""make_demo.py - gera um GIF do pipeline rodando sobre um video gravado (sensor MOCK, malha aberta).

Paineis: camera | chao segmentado (verde = chao, vermelho = nao-chao) e, embaixo, o estado da
visao (LIVRE/INCERTO/OBSTACULO), a confianca relativa r com os limiares, e o estado do controlador.

Uso (da raiz do repositorio):
    python -m tools.make_demo                                   # video do sofa -> docs/media/demo_sofa.gif
    python -m tools.make_demo <video.mp4> --out docs/media/x.gif --width 480 --fps 10
Requer Pillow (vem junto com matplotlib).
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

from config import Config
from main import Runner, iter_frames
from vision.ground import floor_mask

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_VIDEO = ROOT / "assets/simulation_Scar/Reframed_videos/10sec_reframed10.mp4"
DEFAULT_OUT = ROOT / "docs/media/demo_sofa.gif"

BG = (24, 24, 28)
FG = (235, 235, 235)
COLORS = {"LIVRE": (80, 190, 90), "INCERTO": (60, 180, 235), "OBSTACULO": (70, 70, 225),
          "WARMUP": (150, 150, 150), "-": (150, 150, 150)}      # BGR
CTRL_COLORS = {"CRUISE": (80, 190, 90), "SCAN": (60, 180, 235), "AVOID": (70, 70, 225), "WARMUP": (150, 150, 150)}


def _txt(img, s, org, scale=0.5, color=FG, thick=1):
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def _label(img, s):
    cv2.rectangle(img, (0, 0), (len(s) * 8 + 12, 20), (0, 0, 0), -1)
    _txt(img, s, (6, 14), 0.45)


def _r_bar(canvas, x, y, w, h, r, cfg):
    """Barra de r em [0, 1]; marcas: r_obst (0.2), r_exit (0.4, de cima) e r_free (0.5)."""
    cv2.rectangle(canvas, (x, y), (x + w, y + h), (60, 60, 66), -1)
    if r is not None:
        rr = float(np.clip(r, 0, 1))
        col = COLORS["LIVRE"] if rr >= cfg.r_free else COLORS["OBSTACULO"] if rr < cfg.r_obst else COLORS["INCERTO"]
        cv2.rectangle(canvas, (x, y), (x + int(w * rr), y + h), col, -1)
    for v, above in ((cfg.r_obst, False), (cfg.r_exit, True), (cfg.r_free, False)):
        xx = x + int(w * v)
        cv2.line(canvas, (xx, y - 3), (xx, y + h + 3), FG, 1)
        _txt(canvas, f"{v:g}", (xx - 8, y - 6 if above else y + h + 15), 0.35, (170, 170, 175))


def compose(frame, mask, info, cfg, width):
    h = int(width * frame.shape[0] / frame.shape[1])
    cam = cv2.resize(frame, (width, h))
    tint = cam.copy()
    m = cv2.resize(mask, (width, h), interpolation=cv2.INTER_NEAREST) > 0
    tint[m] = (0.45 * tint[m] + 0.55 * np.array((60, 200, 60))).astype(np.uint8)
    tint[~m] = (0.45 * tint[~m] + 0.55 * np.array((50, 50, 220))).astype(np.uint8)
    _label(cam, "camera")
    _label(tint, "chao segmentado")

    bar_h = 78
    canvas = np.full((h + bar_h, width * 2, 3), BG, np.uint8)
    canvas[:h, :width] = cam
    canvas[:h, width:] = tint

    vs = info["vision_state"]
    cs, sub = info["ctrl_state"], info["sub"]
    y = h + 26
    _txt(canvas, "visao", (12, y), 0.45, (170, 170, 175))
    cv2.rectangle(canvas, (62, y - 16), (62 + 118, y + 6), COLORS.get(vs, COLORS["-"]), -1)
    _txt(canvas, vs.replace("OBSTACULO", "OBSTACULO"), (68, y), 0.5, (20, 20, 20), 1)
    _txt(canvas, "controle", (210, y), 0.45, (170, 170, 175))
    cv2.rectangle(canvas, (278, y - 16), (278 + 150, y + 6), CTRL_COLORS.get(cs, (150, 150, 150)), -1)
    _txt(canvas, f"{cs} {sub}".strip(), (284, y), 0.5, (20, 20, 20), 1)
    _txt(canvas, f"t = {info['t']:4.1f} s", (width * 2 - 100, y), 0.5)
    rx, rw = 470, width * 2 - 470 - 40
    _txt(canvas, "r", (rx - 18, y), 0.45, (170, 170, 175))
    _r_bar(canvas, rx, y - 12, rw, 12, info.get("r"), cfg)
    return canvas


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("video", nargs="?", default=str(DEFAULT_VIDEO))
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    ap.add_argument("--width", type=int, default=320, help="largura de cada painel (px)")
    ap.add_argument("--fps", type=float, default=10.0, help="fps do GIF")
    ap.add_argument("--colors", type=int, default=64, help="cores da paleta do GIF")
    ap.add_argument("--no-edges", action="store_true")
    a = ap.parse_args()

    cfg = Config()
    runner = Runner(cfg, use_edges=not a.no_edges)
    frames = []
    for frame, t in iter_frames(a.video):
        info = runner.feed(frame, t)
        if not runner.ready:
            continue                                   # aquecimento: ainda sem mascara
        mask = floor_mask(frame, runner.hist, runner.thr)
        frames.append(Image.fromarray(cv2.cvtColor(compose(frame, mask, info, cfg.decision, a.width), cv2.COLOR_BGR2RGB)))
    if not frames:
        raise SystemExit("Nenhum frame processado.")

    # paleta global pequena (sem dither) -> GIF ~2 MB em vez de ~5 MB
    step = max(1, len(frames) // 12)
    sample = Image.new("RGB", (frames[0].width, frames[0].height * len(frames[::step])))
    for i, f in enumerate(frames[::step]):
        sample.paste(f, (0, i * f.height))
    pal = sample.quantize(colors=a.colors, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE)
    frames = [f.quantize(palette=pal, dither=Image.Dither.NONE) for f in frames]

    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(out, save_all=True, append_images=frames[1:], duration=int(1000 / a.fps), loop=0,
                   optimize=True, disposal=1)
    print(f"{len(frames)} frames -> {out} ({out.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
