"""tools/bench_s9.py - mede fps e latencia do pipeline completo (sem janelas, sem graficos).

O MESMO script roda no PC, no Termux e dentro do app Chaquopy (bench_text), para comparar
os numeros. Pipeline por frame: ground (mascara + retroprojecao + features) -> decision
-> bordas -> controller com sensor MOCK. A decodificacao do video e medida a parte.

Uso (da raiz do repositorio):
    python -m tools.bench_s9 videoParede.mp4
    python -m tools.bench_s9 videoParede.mp4 --max-frames 120 --no-edges --json bench_pc.json
    python -m tools.bench_s9 videoParede.mp4 --threads 1      # limita threads do OpenCV
No app Chaquopy:  tools.bench_s9.bench_text("/caminho/video.mp4")  -> str

Veredito: compara o p95 do tempo de processamento por frame com o orcamento
(padrao: 1000/fps do video; 100 ms para 10 fps). Se p95 > orcamento, a visao nao acompanha.
"""
import argparse
import json
import os
import platform
import sys
import time

import cv2
import numpy as np

from config import Config
from control.controller import Controller, Vision, edge_is_near
from hardware.hal import MockRangeHead, Scenario
from vision import edge_probe
from vision.coverage_log import WARMUP, coverage_row, floor_prob
from vision.decision import DecisionMaker
from vision.ground import floor_mask, learn_floor_hist
from vision.thr_auto import calibrate_thr

STAGES = ["decode", "ground", "decision", "edges", "controller"]


def _edge_gray(frame_bgr):
    small = edge_probe.resize_w(frame_bgr)
    return cv2.GaussianBlur(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY), (3, 3), 0)


def _pct(values, p):
    s = sorted(values)
    return s[min(len(s) - 1, int(round(p / 100 * (len(s) - 1))))] if s else float("nan")


def run_bench(video, max_frames=None, use_edges=True, cfg=None, threads=None, budget_ms=None):
    """Roda o pipeline e devolve um dicionario com plataforma, tempos por etapa e veredito."""
    if threads is not None:
        cv2.setNumThreads(int(threads))
    cfg = cfg or Config()
    cap = cv2.VideoCapture(video)
    if not cap.isOpened():
        raise FileNotFoundError(f"Nao abriu: {video}")
    fps_src = cap.get(cv2.CAP_PROP_FPS) or 30.0
    budget_ms = budget_ms or 1000.0 / fps_src

    t0 = time.perf_counter()                       # setup: aquecimento + histograma + thr auto
    buf = []
    while len(buf) < WARMUP + 1:
        ok, f = cap.read()
        if not ok:
            break
        buf.append(f)
    if not buf:
        raise RuntimeError("Video vazio.")
    hist = learn_floor_hist(buf[0])
    thr, _ = calibrate_thr(buf[1:], hist) if len(buf) > 1 else (0.02, 0.0)
    setup_ms = (time.perf_counter() - t0) * 1000

    dm = DecisionMaker(cfg.decision)
    ctl = Controller(MockRangeHead(Scenario.constant(), noise_m=0.0), cfg.control)
    tm = {k: [] for k in STAGES}
    pc = time.perf_counter

    def frames():
        for f in buf:
            yield f, None
        while True:
            a = pc()
            ok, f = cap.read()
            if not ok:
                return
            yield f, (pc() - a) * 1000

    n = 0
    for idx, (frame, dec_ms) in enumerate(frames()):
        if max_frames and idx >= max_frames:
            break
        t = idx / fps_src
        if dec_ms is not None:
            tm["decode"].append(dec_ms)
        a = pc()
        mask = floor_mask(frame, hist, thr)
        prob, hsv = floor_prob(frame, hist)
        cov = coverage_row(mask, prob, hsv, idx, t)
        b = pc()
        vstate = dm.update(cov["prob_mean_cen"], cov["unknown_inf"], cov["unknown_cen"], t)
        c = pc()
        near = False
        if use_edges:
            feats, _, _ = edge_probe.edge_features(_edge_gray(frame))
            near = edge_is_near(feats["h_cen"], feats["cov"], cfg.control)
        d = pc()
        ctl.update(t, Vision(vstate, near))
        e = pc()
        for k, v in (("ground", b - a), ("decision", c - b), ("edges", d - c), ("controller", e - d)):
            tm[k].append(v * 1000)
        n += 1

    cap.release()
    total = [sum(x) for x in zip(*(tm[k] for k in ("ground", "decision", "edges", "controller")))]
    stats = {k: {"mean": float(np.mean(v)), "p95": _pct(v, 95), "max": float(max(v))}
             for k, v in tm.items() if v}
    stats["total_compute"] = {"mean": float(np.mean(total)), "p95": _pct(total, 95), "max": float(max(total))}
    h, w = (buf[0].shape[:2])
    return {"python": sys.version.split()[0], "numpy": np.__version__, "cv2": cv2.__version__,
            "machine": platform.machine(), "cpus": os.cpu_count(), "cv2_threads": cv2.getNumThreads(),
            "video": os.path.basename(video), "resolution": f"{w}x{h}", "fps_source": round(fps_src, 2),
            "frames": n, "edges": use_edges, "setup_ms": setup_ms, "budget_ms": budget_ms,
            "stats": stats, "fps_compute": 1000.0 / stats["total_compute"]["mean"],
            "keeps_up": stats["total_compute"]["p95"] <= budget_ms}


def format_report(r):
    L = [f"python {r['python']} | numpy {r['numpy']} | cv2 {r['cv2']} | {r['machine']} | "
         f"{r['cpus']} CPUs | cv2 threads {r['cv2_threads']}",
         f"video {r['video']} {r['resolution']} @ {r['fps_source']} fps | {r['frames']} frames | "
         f"bordas {'ligadas' if r['edges'] else 'desligadas'} | setup {r['setup_ms']:.0f} ms",
         f"{'etapa':<16}{'media ms':>10}{'p95 ms':>10}{'max ms':>10}"]
    for k in STAGES + ["total_compute"]:
        if k in r["stats"]:
            s = r["stats"][k]
            L.append(f"{k:<16}{s['mean']:>10.1f}{s['p95']:>10.1f}{s['max']:>10.1f}")
    L.append(f"fps de computacao (1000/media): {r['fps_compute']:.1f}")
    L.append(f"orcamento por frame: {r['budget_ms']:.0f} ms (p95 do total) -> "
             + ("ACOMPANHA o video" if r["keeps_up"] else "NAO acompanha o video"))
    return "\n".join(L)


def bench_text(video_path, max_frames=150, use_edges=True):
    """Entrada para o app Chaquopy: devolve o relatorio como texto."""
    return format_report(run_bench(video_path, max_frames=max_frames, use_edges=use_edges))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--no-edges", action="store_true")
    ap.add_argument("--threads", type=int, default=None)
    ap.add_argument("--budget-ms", type=float, default=None)
    ap.add_argument("--config", default=None)
    ap.add_argument("--json", default=None, help="salva o resultado (para me enviar)")
    a = ap.parse_args()
    res = run_bench(a.video, a.max_frames, not a.no_edges, Config.load(a.config), a.threads, a.budget_ms)
    print(format_report(res))
    if a.json:
        json.dump(res, open(a.json, "w"), indent=2)