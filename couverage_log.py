"""coverage_log.py - diagnostico ADITIVO: cobertura de chao por frame.

Nao altera ground.py nem o loop interativo. Roda o video inteiro sem janelas,
aprende o histograma no 1o frame (igual ao seu loop) e registra, para duas
faixas da parte inferior do frame, tres sinais:

  floor_bin_*  : fracao de pixels de chao na mascara binaria (floor_mask atual)
  prob_mean_*  : media do mapa de retroprojecao (0..1), ANTES do threshold
  unknown_*    : fracao de pixels com V escuro/estourado (H-S nao confiavel)

Faixas: 'inf' = ultimas linhas, largura toda; 'cen' = mesmas linhas, terco central.

Uso:
    python coverage_log.py                       # video padrao do corredor
    python coverage_log.py outro.mp4 --thr 0.02 --out cov_outro
    python coverage_log.py outro.mp4 --thr auto --out cov_outro_auto   # thr derivado da ROI
"""
import argparse
import csv

import cv2
import numpy as np

from ground import preprocess, learn_floor_hist, floor_mask, calibrate_thr

VIDEO_DEFAULT = "assets/simulation_Scar/Reframed_videos/12sec_reframed10.mp4"

STRIP_Y0 = 0.75            # faixa = ultimos 25% das linhas
CEN_X0, CEN_X1 = 1 / 3, 2 / 3
V_LOW, V_HIGH = 40, 235    # V fora disso => "desconhecido" (escuro / estourado)
# S baixo NAO entra como desconhecido: seu piso e cinza/claro, entao S baixo e normal nele.

FIELDS = ["frame_idx", "t",
          "floor_bin_inf", "floor_bin_cen",
          "prob_mean_inf", "prob_mean_cen",
          "unknown_inf", "unknown_cen"]


def floor_prob(frame_bgr, hist):
    """Mapa de retroprojecao suavizado (uint8 0..255), antes do threshold.
    Mesmas operacoes de floor_mask ate o medianBlur; devolve tambem o HSV."""
    hsv = preprocess(frame_bgr)
    prob = cv2.calcBackProject([hsv], [0, 1], hist, [0, 180, 0, 256], 1)
    return cv2.medianBlur(prob, 3), hsv


def strip_slices(h, w):
    y0 = int(STRIP_Y0 * h)
    inf = (slice(y0, h), slice(0, w))
    cen = (slice(y0, h), slice(int(CEN_X0 * w), int(CEN_X1 * w)))
    return {"inf": inf, "cen": cen}


def coverage_row(mask, prob, hsv, idx, t):
    unknown = (hsv[:, :, 2] < V_LOW) | (hsv[:, :, 2] > V_HIGH)
    row = {"frame_idx": idx, "t": round(t, 3)}
    for name, sl in strip_slices(*mask.shape).items():
        row[f"floor_bin_{name}"] = float((mask[sl] > 0).mean())
        row[f"prob_mean_{name}"] = float(prob[sl].mean() / 255.0)
        row[f"unknown_{name}"] = float(unknown[sl].mean())
    return row


WARMUP = 10   # frames 1..WARMUP usados para calibrar o thr (frame 0 so aprende o hist)


def log_floor_coverage(video_path, csv_path, thr=0.02, pct=5.0):
    """thr: float fixo, ou "auto" (percentil `pct` da ROI nos frames de aquecimento)."""
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise FileNotFoundError(f"Nao abriu: {video_path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0

    buf = []                                   # frame 0 + aquecimento
    while len(buf) < WARMUP + 1:
        ok, frame = cap.read()
        if not ok:
            break
        buf.append(frame)
    if not buf:
        raise RuntimeError("Video vazio.")

    hist = learn_floor_hist(buf[0])
    if thr == "auto":
        thr, p = calibrate_thr(buf[1:], hist, pct) if len(buf) > 1 else (0.02, 0.0)
        print(f"thr auto = {thr:.2f} (percentil {pct:g} da ROI = {p:.1f})")
    else:
        thr = float(thr)
        print(f"thr fixo = {thr}")

    def frames():
        yield from buf
        while True:
            ok, f = cap.read()
            if not ok:
                return
            yield f

    idx = 0
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        for frame in frames():
            mask = floor_mask(frame, hist, thr)
            prob, hsv = floor_prob(frame, hist)
            writer.writerow(coverage_row(mask, prob, hsv, idx, idx / fps))
            idx += 1
    cap.release()
    return idx


def plot_coverage(csv_path, png_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r["t"]) for r in rows])
    col = lambda k: np.array([float(r[k]) for r in rows])

    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    panels = [
        ("floor_bin", "Mascara binaria (frac. de chao)"),
        ("prob_mean", "Retroprojecao media (antes do threshold)"),
        ("unknown", "Desconhecido (V escuro/estourado)"),
    ]
    for ax, (key, title) in zip(axs, panels):
        ax.plot(t, col(f"{key}_inf"), label="faixa inferior (largura toda)")
        ax.plot(t, col(f"{key}_cen"), label="faixa inferior (terco central)")
        ax.set_ylim(0, 1.02)
        ax.set_title(title, fontsize=10)
        ax.grid(alpha=0.3)
    axs[0].legend(loc="lower left", fontsize=8)
    axs[-1].set_xlabel("tempo (s)")
    fig.tight_layout()
    fig.savefig(png_path, dpi=120)
    plt.close(fig)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("video", nargs="?", default=VIDEO_DEFAULT)
    ap.add_argument("--thr", default="auto", help='numero fixo ou "auto"')
    ap.add_argument("--pct", type=float, default=5.0, help="percentil da ROI p/ --thr auto")
    ap.add_argument("--out", default="coverage")
    a = ap.parse_args()

    thr = "auto" if a.thr == "auto" else float(a.thr)
    n = log_floor_coverage(a.video, f"{a.out}.csv", thr, a.pct)
    plot_coverage(f"{a.out}.csv", f"{a.out}.png")
    print(f"{n} frames -> {a.out}.csv, {a.out}.png")