"""main.py - loop principal: frame -> visao -> controller -> comando  (raiz do repositorio).

O MESMO codigo roda no PC (pasta de frames ou video) e no celular (app Chaquopy).
Hoje o sensor e o enlace sao MOCKS; troque `head` e `link` quando o hardware existir.

  Runner.feed(frame_bgr, t) -> dict de telemetria (um por frame)
      fase WARMUP : junta as primeiras WARMUP+1 frames, aprende o histograma do chao e o thr
                    (o carro fica PARADO; premissa: so ha chao livre a frente nesse trecho)
      fase RUN    : ground -> decision -> bordas -> controller -> link.send(comando)

Uso no PC (da raiz do repositorio):
    python main.py frames_dir/ --fps 10
    python main.py video.mp4 --scenario scenario_parede_exemplo.csv --csv run.csv
No app:  main.run_text("/caminho/frames", use_edges=True)  ->  str
"""
import argparse
import csv
import glob
import os
import time

import cv2
import numpy as np

from config import Config
from control.controller import Controller, Vision, edge_is_near
from control.hal import Drive, MockDriveLink, MockRangeHead, Scenario
from vision import edge_probe
from vision.coverage_log import WARMUP, coverage_row, floor_prob
from vision.decision import DecisionMaker
from vision.ground import calibrate_thr, floor_mask, learn_floor_hist

STOP = Drive(0.0, 0.0)


def _edge_gray(frame_bgr):
    small = edge_probe.resize_w(frame_bgr)
    return cv2.GaussianBlur(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY), (3, 3), 0)


def frame_from_rgba(data, width, height, row_stride=None):
    """Bytes RGBA_8888 (ex.: ImageProxy do CameraX) -> frame BGR (height, width, 3).
    row_stride = bytes por linha no buffer (pode ser maior que width*4 por causa de padding)."""
    stride = int(row_stride or width * 4)
    arr = np.frombuffer(data, dtype=np.uint8)
    need = height * stride
    if arr.size < need:                                   # a ultima linha pode vir sem padding
        arr = np.concatenate([arr, np.zeros(need - arr.size, np.uint8)])
    rgba = arr[:need].reshape(height, stride)[:, :width * 4].reshape(height, width, 4)
    return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)


class Runner:
    """Estado completo de uma execucao. Um objeto por corrida; chame feed() a cada frame."""

    def __init__(self, cfg=None, head=None, link=None, use_edges=True, cv_threads=1):
        if cv_threads:
            cv2.setNumThreads(int(cv_threads))   # no S9 o numero de threads nao fez diferenca; 1 deixa CPU livre
        self.cfg = cfg or Config()
        self.head = head if head is not None else MockRangeHead(Scenario.constant(), noise_m=0.0)
        self.link = link if link is not None else MockDriveLink()
        self.use_edges = use_edges
        self.dm = DecisionMaker(self.cfg.decision)
        self.ctl = Controller(self.head, self.cfg.control)
        self.hist = self.thr = None
        self._buf, self._n, self._t0 = [], 0, None

    @property
    def ready(self):
        return self.hist is not None

    def _learn(self):
        self.hist = learn_floor_hist(self._buf[0])
        self.thr = calibrate_thr(self._buf[1:], self.hist)[0] if len(self._buf) > 1 else 0.02
        self._buf = []

    def feed(self, frame_bgr, t=None):
        """Processa UM frame. t = instante em segundos (None -> relogio real desde o 1o frame)."""
        t_in = time.perf_counter()
        if t is None:
            if self._t0 is None:
                self._t0 = time.monotonic()
            t = time.monotonic() - self._t0

        if not self.ready:
            self._buf.append(frame_bgr)
            if len(self._buf) >= WARMUP + 1:
                self._learn()
            self.link.send(t, STOP)
            info = {"phase": "WARMUP", "vision_state": "-", "r": None, "edge_h_cen": None,
                    "edge_cov": None, "edge_near": False, "ctrl_state": "WARMUP", "sub": "",
                    "angle": None, "sensor_m": None, "v": 0.0, "steer": 0.0,
                    "why": "aprendendo o chao", "guard_level": 0}
        else:
            mask = floor_mask(frame_bgr, self.hist, self.thr)
            prob, hsv = floor_prob(frame_bgr, self.hist)
            cov = coverage_row(mask, prob, hsv, self._n, t)
            vstate = self.dm.update(cov["prob_mean_cen"], cov["unknown_inf"], cov["unknown_cen"], t)
            e_h = e_cov = None
            near = False
            if self.use_edges:
                feats, _, _ = edge_probe.edge_features(_edge_gray(frame_bgr))
                e_h, e_cov = feats["h_cen"], feats["cov"]
                near = edge_is_near(e_h, e_cov, self.cfg.control)
            drive, info = self.ctl.update(t, Vision(vstate, near))
            self.link.send(t, drive)
            info.update({"phase": "RUN", "vision_state": vstate.value, "r": self.dm.r,
                         "edge_h_cen": e_h, "edge_cov": e_cov, "edge_near": near})
        info.update({"t": round(t, 3), "n": self._n,
                     "proc_ms": (time.perf_counter() - t_in) * 1000})
        self._n += 1
        return info


# ------------------------------------------------------------ fontes de frames ---
def iter_frames(path, fps=None):
    """Gera (frame_bgr, t). Pasta de imagens (jpg/png, ordem alfabetica) ou arquivo de video."""
    if os.path.isdir(path):
        files = sorted(f for f in glob.glob(os.path.join(path, "*"))
                       if f.lower().endswith((".jpg", ".jpeg", ".png")))
        if not files:
            raise FileNotFoundError(f"Pasta sem imagens: {path}")
        rate = float(fps or 10.0)
        for i, f in enumerate(files):
            img = cv2.imread(f)
            if img is not None:
                yield img, i / rate
        return
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise FileNotFoundError(f"Nao abriu o video: {path} (no Android use uma pasta de frames)")
    rate = float(fps or cap.get(cv2.CAP_PROP_FPS) or 30.0)
    i = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        yield frame, i / rate
        i += 1
    cap.release()


def run_source(path, cfg=None, scenario=None, use_edges=True, fps=None, max_frames=None, cv_threads=1):
    """O 'for com frames': devolve uma lista de dicts (telemetria por frame)."""
    head = MockRangeHead(scenario or Scenario.constant(), noise_m=0.0)
    runner = Runner(cfg, head=head, use_edges=use_edges, cv_threads=cv_threads)
    rows = []
    for k, (frame, t) in enumerate(iter_frames(path, fps)):
        if max_frames and k >= max_frames:
            break
        rows.append(runner.feed(frame, t))
    return rows


def summarize(rows):
    if not rows:
        return "nenhum frame processado"
    run = [r for r in rows if r["phase"] == "RUN"]
    ms = sorted(r["proc_ms"] for r in rows)
    p95 = ms[min(len(ms) - 1, int(round(0.95 * (len(ms) - 1))))]
    L = [f"{len(rows)} frames ({len(rows) - len(run)} de aquecimento) | por frame: "
         f"media {sum(ms) / len(ms):.1f} ms, p95 {p95:.1f} ms, max {ms[-1]:.1f} ms",
         "transicoes do controller:"]
    prev = None
    for r in rows:
        key = (r["ctrl_state"], r["sub"])
        if key != prev:
            sm = "-" if r["sensor_m"] is None else f"{r['sensor_m']:.2f}"
            L.append(f"  t={r['t']:6.2f}s {r['ctrl_state']:<7}{r['sub']:<8} vis={r['vision_state']:<9} "
                     f"sensor={sm:>5}  {r['why']}")
            prev = key
    counts = {}
    for r in rows:
        counts[r["ctrl_state"]] = counts.get(r["ctrl_state"], 0) + 1
    L.append("frames por estado: " + ", ".join(f"{k}={v}" for k, v in counts.items()))
    return "\n".join(L)


def run_text(path, use_edges=True, fps=None, max_frames=300, scenario_csv=None, config_json=None):
    """Entrada para o app Chaquopy: roda o loop sobre `path` e devolve o resumo como texto."""
    sc = Scenario.from_csv(scenario_csv) if scenario_csv else None
    rows = run_source(path, Config.load(config_json), sc, bool(use_edges), fps, int(max_frames))
    return summarize(rows)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("source", help="pasta de frames (jpg/png) ou arquivo de video")
    ap.add_argument("--fps", type=float, default=None)
    ap.add_argument("--scenario", default=None, help="CSV t,center_m,left_m,right_m do sensor MOCK")
    ap.add_argument("--config", default=None)
    ap.add_argument("--no-edges", action="store_true")
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--csv", default=None, help="salva a telemetria por frame")
    a = ap.parse_args()
    sc = Scenario.from_csv(a.scenario) if a.scenario else None
    rows = run_source(a.source, Config.load(a.config), sc, not a.no_edges, a.fps, a.max_frames)
    print(summarize(rows))
    if a.csv:
        with open(a.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)