"""flow_probe.py - passo 5: sonda de optical flow (Farneback) na regiao central.

Nao depende de ground.py nem do decision.py. Gera, por par de frames, features
para voce comparar "parede de frente" x "chao limpo" e escolher limiares:

  radial_mean : fluxo projetado radialmente a partir do centro da ROI (px/frame, @320 px).
                > 0 = aponta para FORA do centro (expansao)
  div_mean    : divergencia media du/dx + dv/dy na ROI
  mag_raw     : magnitude media do fluxo (com a translacao/tremor)
  mag_mean    : magnitude media apos subtrair o fluxo MEDIANO da ROI (tira o tremor)
  mag_top/mid/bot : magnitude (sem tremor) em 3 faixas horizontais da ROI
  bt_ratio    : (bot - top) / (bot + top). Hipotese: chao => alto (cai rumo ao
                horizonte); parede de frente => perto de 0 (quase uniforme)
  tex         : gradiente medio da ROI. Baixo = pouca textura = fluxo NAO confiavel

Uso:
    python flow_probe.py sofa.mp4 --out flow_sofa --mark 1.9 --mark 6.7
    python flow_probe.py sofa.mp4 --out flow_sofa --seg chao:0.2:1.5 --seg parede:2.3:4.0
--mark: linhas verticais no grafico (seus eventos anotados a olho).
--seg nome:t0:t1: imprime a media de cada feature no intervalo (para escolher limiar).
"""
import argparse
import csv
import warnings

import cv2
import numpy as np

TARGET_W = 320
ROI_X0, ROI_X1 = 0.30, 0.70     # regiao central (fracao da largura)
ROI_Y0, ROI_Y1 = 0.35, 0.95     # fracao da altura
TEX_MIN = 10.0    # gradiente Sobel minimo p/ o pixel contar (abaixo disso o fluxo e ruido/zero)
MIN_PIX = 50      # minimo de pixels texturizados; abaixo disso as estatisticas viram NaN

FEATS = ["radial_mean", "div_mean", "mag_raw", "mag_mean",
         "mag_top", "mag_mid", "mag_bot", "bt_ratio", "tex", "frac_tex"]


def to_gray(frame_bgr):
    h, w = frame_bgr.shape[:2]
    k = TARGET_W / w
    interp = cv2.INTER_AREA if k < 1 else cv2.INTER_LINEAR
    small = cv2.resize(frame_bgr, (TARGET_W, max(1, round(h * k))), interpolation=interp)
    return cv2.GaussianBlur(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY), (3, 3), 0)


def _mean(a, sel):
    """Media de `a` so onde `sel` (texturizado); NaN se houver poucos pixels."""
    return float(a[sel].mean()) if sel.sum() >= MIN_PIX else float("nan")


def flow_features(prev_g, curr_g):
    flow = cv2.calcOpticalFlowFarneback(prev_g, curr_g, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    flow = cv2.GaussianBlur(flow, (5, 5), 0)
    u, v = flow[..., 0], flow[..., 1]
    h, w = prev_g.shape
    y0, y1, x0, x1 = int(ROI_Y0 * h), int(ROI_Y1 * h), int(ROI_X0 * w), int(ROI_X1 * w)

    # mascara de textura: so pixels com gradiente suficiente tem fluxo confiavel
    gx = cv2.Sobel(curr_g, cv2.CV_32F, 1, 0)[y0:y1, x0:x1]
    gy = cv2.Sobel(curr_g, cv2.CV_32F, 0, 1)[y0:y1, x0:x1]
    gmag = np.hypot(gx, gy)
    sel = gmag > TEX_MIN

    ur, vr = u[y0:y1, x0:x1], v[y0:y1, x0:x1]
    mag_raw = _mean(np.hypot(ur, vr), sel)

    if sel.sum() >= MIN_PIX:                      # tremor = mediana SO dos texturizados
        ur0, vr0 = ur - np.median(ur[sel]), vr - np.median(vr[sel])
    else:
        ur0, vr0 = ur, vr
    mag = np.hypot(ur0, vr0)

    div = (np.gradient(u, axis=1) + np.gradient(v, axis=0))[y0:y1, x0:x1]

    ys, xs = np.mgrid[y0:y1, x0:x1].astype(np.float32)
    dx, dy = xs - (x0 + x1) / 2, ys - (y0 + y1) / 2
    r = np.maximum(np.hypot(dx, dy), 1.0)
    radial = (ur0 * dx + vr0 * dy) / r

    bands = np.array_split(np.arange(mag.shape[0]), 3)
    top, mid, bot = (_mean(mag[i[0]:i[-1] + 1], sel[i[0]:i[-1] + 1]) for i in bands)
    bt_ratio = (bot - top) / (bot + top + 1e-6)

    return {"radial_mean": _mean(radial, sel), "div_mean": _mean(div, sel),
            "mag_raw": mag_raw, "mag_mean": _mean(mag, sel),
            "mag_top": top, "mag_mid": mid, "mag_bot": bot, "bt_ratio": float(bt_ratio),
            "tex": float(gmag.mean()), "frac_tex": float(sel.mean())}


def run(video_path):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise FileNotFoundError(f"Nao abriu: {video_path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    rows, prev, idx = [], None, 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        g = to_gray(frame)
        if prev is not None:                       # o par (idx-1, idx) vira a linha idx
            row = {"frame_idx": idx, "t": round(idx / fps, 3)}
            row.update(flow_features(prev, g))
            rows.append(row)
        prev, idx = g, idx + 1
    cap.release()
    return rows


def plot(rows, png_path, marks):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = np.array([r["t"] for r in rows])
    col = lambda k: np.array([r[k] for r in rows])
    fig, axs = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    spec = [(("radial_mean", "div_mean"), "Expansao (radial_mean, div_mean)"),
            (("mag_raw", "mag_mean"), "Magnitude (bruta x sem tremor)"),
            (("mag_top", "mag_mid", "mag_bot", "bt_ratio"), "Perfil vertical da magnitude (+ bt_ratio)"),
            (("tex", "frac_tex"), "Textura da ROI: gradiente medio e fracao de pixels texturizados")]
    for ax, (keys, title) in zip(axs, spec):
        for k in keys:
            ax.plot(t, col(k), label=k)
        for m in marks:
            ax.axvline(m, color="gray", ls="--", lw=0.8)
        ax.set_title(title, fontsize=10)
        ax.grid(alpha=0.3)
        ax.legend(loc="upper left", fontsize=7, ncol=4)
    axs[-1].set_xlabel("tempo (s)")
    fig.tight_layout()
    fig.savefig(png_path, dpi=120)
    plt.close(fig)


def summarize(rows, segs):
    warnings.filterwarnings("ignore", message="Mean of empty slice")   # NaN = sem textura
    print(f"{'segmento':<12}{'n':>4}" + "".join(f"{k:>13}" for k in FEATS))
    for s in segs:
        name, t0, t1 = s.split(":")
        sel = [r for r in rows if float(t0) <= r["t"] <= float(t1)]
        if not sel:
            print(f"{name:<12}   0  (sem frames)")
            continue
        print(f"{name:<12}{len(sel):>4}" +
              "".join(f"{np.nanmean([r[k] for r in sel]):>13.3f}" for k in FEATS))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("--out", default="flow")
    ap.add_argument("--mark", type=float, action="append", default=[])
    ap.add_argument("--seg", action="append", default=[], help="nome:t0:t1")
    a = ap.parse_args()

    rows = run(a.video)
    with open(f"{a.out}.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["frame_idx", "t"] + FEATS)
        w.writeheader()
        w.writerows(rows)
    plot(rows, f"{a.out}.png", a.mark)
    print(f"{len(rows)} pares -> {a.out}.csv, {a.out}.png")
    if a.seg:
        summarize(rows, a.seg)