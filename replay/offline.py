"""replay/offline.py - loop OFFLINE [LEGADO: fora do caminho ao vivo; usa o Controller v1 congelado] (pasta de frames ou video) com Controller + sensor MOCK.

Era o Runner do main.py. O caminho ao vivo (main.StreamRunner) NAO usa nada daqui: o ESP32 e o
unico executor de manobras (arquitetura "ESP32 executor"). Isto serve a replay, testes, tools/make_demo.py e ao app de
bancada (MainActivity -> android_entry.run_main), que por isso copia este arquivo no syncPython.

Uso:  python -m replay.offline <pasta_ou_video> [--scenario cenario.csv] [--csv saida.csv]
"""
import argparse
import csv
import glob
import os

import cv2

from config import Config
from control.controller import Controller, Vision
from control.hal import Drive
from control.mocks import MockDriveLink, MockRangeHead, Scenario
from main import VisionRunner
from vision.decision import State as VState

STOP = Drive(0.0, 0.0)


class Runner(VisionRunner):
    """VisionRunner + Controller v1 com sensor MOCK (referencia/replay). Um objeto por corrida."""

    def __init__(self, cfg=None, head=None, link=None, use_edges=True, cv_threads=1):
        super().__init__(cfg, use_edges=use_edges, cv_threads=cv_threads)
        self.head = head if head is not None else MockRangeHead(Scenario.constant(), noise_m=0.0)
        self.link = link if link is not None else MockDriveLink()
        self.ctl = Controller(self.head, self.cfg.control)

    def feed(self, frame_bgr, t=None):
        info = super().feed(frame_bgr, t)
        t = info["t"]
        if info["phase"] != "RUN":
            self.link.send(t, STOP)
            info.update({"ctrl_state": "WARMUP", "sub": "", "angle": None, "sensor_m": None,
                         "v": 0.0, "steer": 0.0, "why": "aprendendo o chao", "guard_level": 0})
        else:
            drive, cinfo = self.ctl.update(t, Vision(VState(info["vision_state"]), info["edge_near"]))
            self.link.send(t, drive)
            info.update(cinfo)
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