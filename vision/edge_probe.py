"""edge_probe.py - sonda de bordas: linha de contato chao-parede (rodape).

Independente de ground.py / decision.py. Para cada frame:
  1) Canny automatico (limiares pela mediana do cinza, robusto a exposicao);
  2) mantem so bordas quase HORIZONTAIS (|gy| > HORIZ_RATIO*|gx|) e LONGAS
     (abertura horizontal de MIN_LEN px) -> rodape sim, junta diagonal de ladrilho nao;
  3) em cada faixa de STRIP_W colunas (mesma grade do free_space_vector), sobe de
     baixo para cima e para na primeira linha com borda.

Altura = fracao da imagem a partir da base (mesma convencao do free_space_vector):
  1.0 = nenhuma borda ate o topo da varredura; menor = barreira mais PERTO.

Colunas do CSV:
  h_left / h_cen / h_right : mediana das alturas nos tercos esquerdo / central / direito
  cov     : fracao das faixas onde achou borda (alto = linha atravessando a imagem)
  dens    : fracao de pixels de borda horizontal na area varrida (alto = muita "sujeira":
            juntas de ladrilho, tapetes, bordas de moveis)
  str_cen : forca media do gradiente nas bordas achadas no centro (NaN se nao achou)

Uso:
    python edge_probe.py videoParede.mp4 --out edge_parede --mark 6.4 \
        --seg chao:0.0:2.8 --seg parede2:9.2:13.4 --snap 5.0 --snap 12.0
--mark: linha vertical no grafico. --seg nome:t0:t1: media das colunas no intervalo.
--canny LO,HI: limiares fixos do Canny (baixe para bordas de baixo contraste).
--snap T: salva <out>_snap_T.png (bordas em vermelho, altura por faixa em verde)
          do frame mais proximo de T s, para voce conferir contra o rodape real.
"""
import argparse
import csv
import warnings

import cv2
import numpy as np

TARGET_W = 320
STRIP_W = 8                      # igual ao free_space_vector
SCAN_TOP, SCAN_BOTTOM = 0.25, 0.97   # linhas varridas (fracao da altura)
CANNY_SIGMA = 0.33
HORIZ_RATIO = 4.0                # ~ +-14 graus da horizontal
MIN_LEN = 20                     # extensao horizontal minima (px @320)
MIN_FRAC = 0.5                   # fracao das colunas da faixa que precisa ter borda
CANNY_FIXED = None               # (lo, hi) fixos; None = automatico (--canny LO,HI)

FEATS = ["h_left", "h_cen", "h_right", "cov", "dens", "str_cen"]


def resize_w(frame_bgr):
    h, w = frame_bgr.shape[:2]
    k = TARGET_W / w
    interp = cv2.INTER_AREA if k < 1 else cv2.INTER_LINEAR
    return cv2.resize(frame_bgr, (TARGET_W, max(1, round(h * k))), interpolation=interp)


def auto_canny(gray, sigma=CANNY_SIGMA):
    if CANNY_FIXED is not None:
        return cv2.Canny(gray, CANNY_FIXED[0], CANNY_FIXED[1])
    m = float(np.median(gray))
    lo = int(max(0, (1 - sigma) * m))
    hi = int(min(255, (1 + sigma) * m))
    return cv2.Canny(gray, lo, max(hi, lo + 1))


def horiz_edges(gray):
    """Mascara bool de bordas quase horizontais e longas + magnitude do gradiente."""
    gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0)
    gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1)
    m = ((auto_canny(gray) > 0) & (np.abs(gy) > HORIZ_RATIO * np.abs(gx))).astype(np.uint8)
    m = cv2.dilate(m, np.ones((3, 3), np.uint8))          # tolera leve inclinacao
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((1, MIN_LEN), np.uint8))
    return m > 0, np.hypot(gx, gy)


def first_edge_heights(horiz):
    """Altura (0..1, da base) da 1a borda subindo em cada faixa; 1.0 se nao ha.
    Devolve tambem a linha (y) de cada faixa (-1 se nao ha)."""
    h, w = horiz.shape
    y_hi, y_lo = int(SCAN_TOP * h), int(SCAN_BOTTOM * h)
    nb = w // STRIP_W
    grid = horiz[:, :nb * STRIP_W].reshape(h, nb, STRIP_W).mean(axis=2) >= MIN_FRAC
    heights = np.ones(nb)
    rows = np.full(nb, -1)
    for b in range(nb):
        ys = np.nonzero(grid[y_hi:y_lo, b])[0]
        if ys.size:
            y = y_hi + int(ys.max())                      # a mais proxima da base
            rows[b] = y
            heights[b] = (h - y) / h
    return heights, rows


def edge_features(gray):
    horiz, gmag = horiz_edges(gray)
    heights, rows = first_edge_heights(horiz)
    nb = len(heights)
    left, cen, right = np.array_split(np.arange(nb), 3)
    h, _ = gray.shape
    y_hi, y_lo = int(SCAN_TOP * h), int(SCAN_BOTTOM * h)

    strengths = [float(gmag[rows[b], b * STRIP_W:(b + 1) * STRIP_W].mean())
                 for b in cen if rows[b] >= 0]
    feats = {"h_left": float(np.median(heights[left])),
             "h_cen": float(np.median(heights[cen])),
             "h_right": float(np.median(heights[right])),
             "cov": float((rows >= 0).mean()),
             "dens": float(horiz[y_hi:y_lo].mean()),
             "str_cen": float(np.mean(strengths)) if strengths else float("nan")}
    return feats, horiz, heights


def save_snap(small_bgr, horiz, heights, path):
    h, w = horiz.shape
    ov = small_bgr.copy()
    ov[horiz] = (0, 0, 255)
    pts = [(b * STRIP_W + STRIP_W // 2, int(round((1 - hh) * h))) for b, hh in enumerate(heights)]
    for p, q in zip(pts[:-1], pts[1:]):
        cv2.line(ov, p, q, (0, 255, 0), 1, cv2.LINE_AA)
    for p in pts:
        cv2.circle(ov, p, 2, (0, 255, 0), -1)
    cv2.imwrite(path, cv2.resize(ov, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))


def run(video_path, out, snaps):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise FileNotFoundError(f"Nao abriu: {video_path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    pending = sorted(snaps)
    rows, idx = [], 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        small = resize_w(frame)
        gray = cv2.GaussianBlur(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY), (3, 3), 0)
        feats, horiz, heights = edge_features(gray)
        t = idx / fps
        row = {"frame_idx": idx, "t": round(t, 3)}
        row.update(feats)
        rows.append(row)
        while pending and t >= pending[0]:
            T = pending.pop(0)
            save_snap(small, horiz, heights, f"{out}_snap_{T:g}.png")
            print(f"snap t={t:.2f}s -> {out}_snap_{T:g}.png")
        idx += 1
    cap.release()
    return rows


def plot(rows, png_path, marks):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = np.array([r["t"] for r in rows])
    col = lambda k: np.array([r[k] for r in rows])
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    spec = [(("h_left", "h_cen", "h_right"), "Altura da 1a borda horizontal (1.0 = nenhuma; menor = mais perto)"),
            (("cov", "dens"), "Cobertura (faixas com borda) e densidade de bordas horizontais"),
            (("str_cen",), "Forca do gradiente na borda central")]
    for ax, (keys, title) in zip(axs, spec):
        for k in keys:
            ax.plot(t, col(k), label=k)
        for m in marks:
            ax.axvline(m, color="gray", ls="--", lw=0.8)
        ax.set_title(title, fontsize=10)
        ax.grid(alpha=0.3)
        ax.legend(loc="upper left", fontsize=7, ncol=3)
    axs[0].set_ylim(0, 1.05)
    axs[-1].set_xlabel("tempo (s)")
    fig.tight_layout()
    fig.savefig(png_path, dpi=120)
    plt.close(fig)


def summarize(rows, segs):
    warnings.filterwarnings("ignore", message="Mean of empty slice")   # NaN = sem borda
    print(f"{'segmento':<12}{'n':>4}" + "".join(f"{k:>10}" for k in FEATS))
    for s in segs:
        name, t0, t1 = s.split(":")
        sel = [r for r in rows if float(t0) <= r["t"] <= float(t1)]
        if not sel:
            print(f"{name:<12}   0  (sem frames)")
            continue
        print(f"{name:<12}{len(sel):>4}" +
              "".join(f"{np.nanmean([r[k] for r in sel]):>10.3f}" for k in FEATS))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("--out", default="edge")
    ap.add_argument("--mark", type=float, action="append", default=[])
    ap.add_argument("--seg", action="append", default=[], help="nome:t0:t1")
    ap.add_argument("--snap", type=float, action="append", default=[], help="tempo (s) p/ salvar PNG")
    ap.add_argument("--canny", default=None, help="LO,HI fixos (ex.: 15,40); padrao = automatico")
    a = ap.parse_args()
    if a.canny:
        CANNY_FIXED = tuple(int(v) for v in a.canny.split(","))

    rows = run(a.video, a.out, a.snap)
    with open(f"{a.out}.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["frame_idx", "t"] + FEATS)
        w.writeheader()
        w.writerows(rows)
    plot(rows, f"{a.out}.png", a.mark)
    print(f"{len(rows)} frames -> {a.out}.csv, {a.out}.png")
    if a.seg:
        summarize(rows, a.seg)