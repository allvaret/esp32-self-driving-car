"""notebooks/replay_main.py - esqueleto do main em tempo real, alimentado por VIDEO + sensor MOCK.

Por frame:  frame -> ground (mascara/prob) -> decision.py (estado da visao)
                  -> edge_probe (borda proxima?)  -> controller (fusao + FSM)
                  -> comando (MockDriveLink)      -> linha de log

ATENCAO: o replay e de MALHA ABERTA. O video nao reage aos comandos (a re/curva nao
muda o que a camera ve) e o cenario do sensor e indexado pelo tempo. Serve para
validar a logica e o timing das decisoes, nao o comportamento em malha fechada.

Uso (da raiz do repositorio):
    python -m notebooks.replay_main videoParede.mp4 --scenario scenario_parede_exemplo.csv --out run_parede
    python -m notebooks.replay_main videoParede.mp4 --scenario sc.csv --canny 8,24 --fault none:16.2:18.8
    python -m notebooks.replay_main sofa.mp4 --no-edges --max-range 2.0 --noise 0.02 --dropout 0.05
--fault modo:t0:t1 (repetivel): modo none (sem eco, ex. sol) | far (devolve alcance maximo, ex. perna fina)
Gera <out>.csv, <out>.png e imprime as transicoes do controller.
"""
import argparse
import csv

import cv2
import numpy as np

from config import Config
from control.controller import Controller, Vision, edge_is_near
from control.hal import MockDriveLink, MockRangeHead, Scenario
from vision import edge_probe
from vision.coverage_log import WARMUP, coverage_row, floor_prob
from vision.decision import DecisionMaker
from vision.ground import floor_mask, learn_floor_hist, calibrate_thr

FIELDS = ["frame_idx", "t", "vision_state", "r", "edge_h_cen", "edge_cov", "edge_near",
          "ctrl_state", "sub", "angle", "sensor_m", "truth_center_m", "v", "steer", "why", "guard_level"]


def edge_gray(frame_bgr):
    small = edge_probe.resize_w(frame_bgr)
    return cv2.GaussianBlur(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY), (3, 3), 0)


def run(video, scenario, cfg, thr="auto", use_edges=True, mock_kw=None):
    cap = cv2.VideoCapture(video)
    if not cap.isOpened():
        raise FileNotFoundError(f"Nao abriu: {video}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0

    buf = []                                        # frame 0 + aquecimento (como no coverage_log)
    while len(buf) < WARMUP + 1:
        ok, f = cap.read()
        if not ok:
            break
        buf.append(f)
    if not buf:
        raise RuntimeError("Video vazio.")
    hist = learn_floor_hist(buf[0])
    if thr == "auto":
        thr, p = calibrate_thr(buf[1:], hist) if len(buf) > 1 else (0.02, 0.0)
        print(f"thr auto = {thr:.2f} (percentil 5 da ROI = {p:.1f})")
    else:
        thr = float(thr)

    dm, head, link = DecisionMaker(cfg.decision), MockRangeHead(scenario, **(mock_kw or {})), MockDriveLink()
    ctl = Controller(head, cfg.control)

    def frames():
        yield from buf
        while True:
            ok, f = cap.read()
            if not ok:
                return
            yield f

    rows = []
    for idx, frame in enumerate(frames()):
        t = idx / fps
        mask = floor_mask(frame, hist, thr)
        prob, hsv = floor_prob(frame, hist)
        cov = coverage_row(mask, prob, hsv, idx, t)
        vstate = dm.update(cov["prob_mean_cen"], cov["unknown_inf"], cov["unknown_cen"], t)

        e_h, e_cov, e_near = float("nan"), float("nan"), False
        if use_edges:
            feats, _, _ = edge_probe.edge_features(edge_gray(frame))
            e_h, e_cov = feats["h_cen"], feats["cov"]
            e_near = edge_is_near(e_h, e_cov, cfg.control)

        drive, info = ctl.update(t, Vision(vstate, e_near))
        link.send(t, drive)
        rows.append({"frame_idx": idx, "t": round(t, 3), "vision_state": vstate.value,
                     "r": dm.r, "edge_h_cen": e_h, "edge_cov": e_cov, "edge_near": e_near,
                     "truth_center_m": scenario.at(t, 0.0), **info})
    cap.release()
    return rows


def print_transitions(rows):
    prev = None
    print("transicoes do controller:")
    for r in rows:
        key = (r["ctrl_state"], r["sub"])
        if key != prev:
            sm = "-" if r["sensor_m"] is None else f"{r['sensor_m']:.2f}"
            print(f"  t={r['t']:5.1f}s  {r['ctrl_state']:<6}{r['sub']:<8} vis={r['vision_state']:<9} "
                  f"sensor={sm:>5}  {r['why']}")
            prev = key


def plot(rows, png_path, ccfg):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = np.array([r["t"] for r in rows])
    f = lambda k: np.array([np.nan if r[k] is None else float(r[k]) for r in rows])
    cmap_v = {"LIVRE": "#2ca02c", "INCERTO": "#ffbf00", "OBSTACULO": "#d62728"}
    cmap_c = {"CRUISE": "#2ca02c", "SCAN": "#ffbf00", "AVOID": "#d62728"}
    dt = np.median(np.diff(t)) if len(t) > 1 else 0.1

    fig, axs = plt.subplots(5, 1, figsize=(10, 11), sharex=True,
                            gridspec_kw={"height_ratios": [2, 2, 2, 0.6, 0.6]})
    axs[0].plot(t, f("r"), color="k", label="r (visao)")
    axs[0].plot(t, f("edge_h_cen"), color="tab:blue", label="edge h_cen")
    axs[0].set_title("Sinais de visao", fontsize=10); axs[0].legend(fontsize=7, loc="upper left")
    axs[1].plot(t, np.minimum(f("truth_center_m"), 4), color="gray", ls="--", label="verdade (cenario, centro)")
    axs[1].plot(t, np.minimum(f("sensor_m"), 4), color="tab:purple", label="leitura do sensor (centro)")
    axs[1].axhline(ccfg.d_stop_m, color="r", lw=0.8, ls=":"); axs[1].axhline(ccfg.d_clear_m, color="g", lw=0.8, ls=":")
    axs[1].set_title("Distancia (m)  [d_stop_m / d_clear_m pontilhadas]", fontsize=10)
    axs[1].legend(fontsize=7, loc="upper left")
    axs[2].plot(t, f("v"), label="v"); axs[2].plot(t, f("steer"), label="steer")
    axs[2].set_title("Comando", fontsize=10); axs[2].legend(fontsize=7, loc="upper left")
    for ax, key, cmap, title in ((axs[3], "vision_state", cmap_v, "Estado da visao"),
                                 (axs[4], "ctrl_state", cmap_c, "Estado do controller")):
        for r in rows:
            ax.axvspan(r["t"] - dt / 2, r["t"] + dt / 2, color=cmap[r[key]])
        ax.set_yticks([]); ax.set_title(title, fontsize=9)
    for ax in axs[:3]:
        ax.grid(alpha=0.3)
    axs[-1].set_xlabel("tempo (s)")
    fig.tight_layout(); fig.savefig(png_path, dpi=110); plt.close(fig)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("--scenario", default=None, help="CSV t,center_m,left_m,right_m (padrao: tudo livre)")
    ap.add_argument("--out", default="run")
    ap.add_argument("--config", default=None, help="config.json (padrao: valores DEMO)")
    ap.add_argument("--thr", default="auto")
    ap.add_argument("--no-edges", action="store_true")
    ap.add_argument("--canny", default=None, help="LO,HI fixos p/ as bordas (ex.: 8,24)")
    ap.add_argument("--max-range", type=float, default=2.0)
    ap.add_argument("--noise", type=float, default=0.01)
    ap.add_argument("--dropout", type=float, default=0.0)
    ap.add_argument("--latency", type=float, default=0.0)
    ap.add_argument("--fault", action="append", default=[], help="modo:t0:t1 (none|far)")
    a = ap.parse_args()

    if a.canny:
        edge_probe.CANNY_FIXED = tuple(int(v) for v in a.canny.split(","))
    print("canny:", f"fixo {edge_probe.CANNY_FIXED}" if edge_probe.CANNY_FIXED else "automatico")
    faults = []
    for s in a.fault:
        m, t0, t1 = s.split(":")
        faults.append((float(t0), float(t1), m))
    sc = Scenario.from_csv(a.scenario) if a.scenario else Scenario.constant()
    cfg = Config.load(a.config)
    rows = run(a.video, sc, cfg, a.thr, not a.no_edges,
               dict(max_range_m=a.max_range, noise_m=a.noise, dropout_p=a.dropout,
                    latency_s=a.latency, faults=faults))
    with open(f"{a.out}.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)
    plot(rows, f"{a.out}.png", cfg.control)
    print(f"{len(rows)} frames -> {a.out}.csv, {a.out}.png")
    print_transitions(rows)