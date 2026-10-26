"""tests/test_main.py - o loop de frames (main.py): aquecimento parado, sensor decide, helper RGBA.
Rode:  python -m tests.test_main"""
import os
import tempfile

import cv2
import numpy as np

from control.hal import Scenario
from main import Runner, frame_from_rgba, run_source, summarize
from tests.test_pipeline_smoke import synth_frame
from vision.coverage_log import WARMUP


def _frames_dir(n=40):
    d = tempfile.mkdtemp()
    for i in range(n):
        cv2.imwrite(os.path.join(d, f"frame_{i:04d}.jpg"), synth_frame(i))
    return d


def test_warmup_is_stopped_then_runs():
    rows = run_source(_frames_dir(), use_edges=False)
    warm = [r for r in rows if r["phase"] == "WARMUP"]
    assert len(warm) == WARMUP + 1 and all(r["v"] == 0.0 for r in warm)
    assert rows[len(warm)]["phase"] == "RUN"
    print(f"ok  {len(warm)} frames de aquecimento (carro parado), depois fase RUN")


def test_sensor_scenario_drives_the_controller():
    sc = Scenario([0, 2.0, 2.1, 100], [2.0, 2.0, 0.3, 0.3], [2.0] * 4, [0.4] * 4)
    rows = run_source(_frames_dir(), scenario=sc, use_edges=False, fps=10)
    states = [r["ctrl_state"] for r in rows]
    assert "AVOID" in states and rows[0]["ctrl_state"] == "WARMUP"
    txt = summarize(rows); assert "AVOID" in txt
    print("ok  cenario com obstaculo a 2 s leva o controller a AVOID")


def test_frame_from_rgba_with_row_padding():
    h, w, pad = 6, 5, 8                                   # stride = w*4 + pad
    bgr = np.random.default_rng(0).integers(0, 255, (h, w, 3), dtype=np.uint8)
    rgba = np.dstack([bgr[:, :, ::-1], np.full((h, w), 255, np.uint8)])
    buf = np.zeros((h, w * 4 + pad), np.uint8); buf[:, :w * 4] = rgba.reshape(h, w * 4)
    raw = buf.tobytes()[:-pad]                            # ultima linha sem padding
    out = frame_from_rgba(raw, w, h, w * 4 + pad)
    assert out.shape == (h, w, 3) and np.array_equal(out, bgr)
    print("ok  frame_from_rgba com padding por linha")


def test_runner_accepts_live_clock():
    r = Runner(use_edges=False)
    info = r.feed(synth_frame(0))                         # t=None -> relogio real
    assert info["phase"] == "WARMUP" and info["t"] >= 0.0; print("ok  Runner.feed com relogio real")


if __name__ == "__main__":
    for n, f in list(globals().items()):
        if n.startswith("test_"):
            f()
    print("todos os testes passaram")