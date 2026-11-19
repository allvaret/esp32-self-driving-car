"""main.py - caminho AO VIVO da visao (Chaquopy). So visao: nenhuma decisao de manobra.

O Kotlin chama StreamRunner.process_rgba_frame() por frame e recebe JSON (string) com a OPINIAO
da visao (vision_state, r, setores...). Quem age e o ESP32; o brain.py so repassa a opiniao.
Este modulo NAO importa control.controller nem control.mocks (ver tests/test_live_path.py).
O loop offline com Controller/mocks esta em replay/offline.py.
"""
import csv
import json
import time

import cv2
import numpy as np

from config import Config
from vision import edge_probe
from vision.coverage_log import WARMUP, coverage_row, floor_prob
from vision.decision import DecisionMaker
from vision.ground import calibrate_thr, floor_mask, learn_floor_hist
import viz


def _edge_gray(frame_bgr):
    small = edge_probe.resize_w(frame_bgr)
    return cv2.GaussianBlur(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY), (3, 3), 0)


_ROT = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}


def _as_u8(data):
    """bytes/bytearray (PC) ou byte[] Java (Chaquopy) -> np.uint8 1D."""
    try:
        return np.frombuffer(data, dtype=np.uint8)          # sem copia se suportar buffer protocol
    except TypeError:
        return np.frombuffer(bytes(data), dtype=np.uint8)   # fallback: 1 copia


def frame_from_rgba(data, width, height, row_stride=None, rotation=0):
    """Bytes RGBA_8888 (ImageProxy do CameraX) -> frame BGR ja rotacionado para 'em pe'.
    row_stride: bytes por linha (pode ter padding). rotation: imageInfo.rotationDegrees."""
    stride = int(row_stride or width * 4)
    arr = _as_u8(data)
    need = height * stride
    if arr.size < need:                                   # ultima linha pode vir sem padding
        arr = np.concatenate([arr, np.zeros(need - arr.size, np.uint8)])
    rgba = arr[:need].reshape(height, stride)[:, :width * 4].reshape(height, width, 4)
    bgr = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
    rot = _ROT.get(int(rotation) % 360)
    return cv2.rotate(bgr, rot) if rot is not None else bgr


class VisionRunner:
    """Estado da visao de uma corrida: chao aprendido + DecisionMaker. feed() por frame."""

    def __init__(self, cfg=None, use_edges=True, cv_threads=1):
        if cv_threads:
            cv2.setNumThreads(int(cv_threads))   # no S9: 1 thread deixa CPU livre para os outros nucleos
        self.cfg = cfg or Config()
        self.use_edges = use_edges
        self.dm = DecisionMaker(self.cfg.decision)
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
            info = {"phase": "WARMUP", "ok": False, "vision_state": "-", "r": None,
                    "edge_h_cen": None, "edge_cov": None, "edge_near": False}
        else:
            mask = floor_mask(frame_bgr, self.hist, self.thr)
            viz.push(frame=frame_bgr, mask=mask)
            prob, hsv = floor_prob(frame_bgr, self.hist)
            cov = coverage_row(mask, prob, hsv, self._n, t)
            vstate = self.dm.update(cov["prob_mean_cen"], cov["unknown_inf"], cov["unknown_cen"], t)
            e_h = e_cov = None
            near = False
            if self.use_edges:
                feats, _, _ = edge_probe.edge_features(_edge_gray(frame_bgr))
                e_h, e_cov = feats["h_cen"], feats["cov"]
                near = edge_probe.edge_is_near(e_h, e_cov, self.cfg.control)
            info = {"phase": "RUN", "ok": True, "vision_state": vstate.value, "r": self.dm.r,
                    "edge_h_cen": e_h, "edge_cov": e_cov, "edge_near": near}
        info.update({"t": round(t, 3), "n": self._n,
                     "proc_ms": (time.perf_counter() - t_in) * 1000})
        self._n += 1
        return info


LOG_FIELDS = ["wall_s", "t", "n", "phase", "ok", "vision_state", "r", "edge_h_cen", "edge_cov",
              "edge_near", "why", "conv_ms", "proc_ms", "w", "h", "rot"]


def _jsonable(o):
    if isinstance(o, np.generic):
        return o.item()
    if hasattr(o, "value"):          # Enum
        return o.value
    return str(o)


class StreamRunner:
    """Camera ao vivo no Android (Chaquopy). Um objeto por sessao; o Kotlin chama
    process_rgba_frame() por frame e recebe JSON (string)."""

    def __init__(self, use_edges=True, log_path=None, settle_s=1.0, config_json=None):
        self.use_edges = bool(use_edges)
        self.cfg = Config.load(config_json)
        self.settle_s = float(settle_s)
        self._log_f = self._log_w = None
        self._rows = 0
        if log_path:
            self._log_f = open(str(log_path), "w", newline="")
            self._log_w = csv.DictWriter(self._log_f, LOG_FIELDS, extrasaction="ignore", restval="")
            self._log_w.writeheader()
        self.recalibrate()

    def recalibrate(self):
        """Zera chao aprendido + controller. Aponte para chao LIVRE antes de chamar."""
        self.runner = VisionRunner(self.cfg, use_edges=self.use_edges, cv_threads=1)
        self._t_start = time.monotonic()
        return "ok"

    def process_rgba_frame(self, data, width, height, row_stride=0, rotation=0):
        try:
            t0 = time.perf_counter()
            frame = frame_from_rgba(data, width, height, row_stride, rotation)
            conv_ms = (time.perf_counter() - t0) * 1000
            if time.monotonic() - self._t_start < self.settle_s:   # auto-exposicao estabilizando
                info = {"phase": "SETTLE", "ok": False, "vision_state": "-", "r": None,
                        "why": "estabilizando exposicao", "proc_ms": 0.0}
            else:
                info = self.runner.feed(frame)
            info.update({"conv_ms": conv_ms, "w": frame.shape[1], "h": frame.shape[0],
                         "rot": int(rotation), "wall_s": round(time.monotonic() - self._t_start, 3)})
            self._log(info)
            return json.dumps(info, default=_jsonable)
        except Exception as e:
            import traceback
            return json.dumps({"phase": "ERROR", "ok": False, "vision_state": "-", "why": f"{type(e).__name__}: {e}",
                               "trace": traceback.format_exc()[-1200:]})

    def _log(self, info):
        if self._log_w is None:
            return
        self._log_w.writerow(info)
        self._rows += 1
        if self._rows % 30 == 0:
            self._log_f.flush()

    def close(self):
        if self._log_f:
            self._log_f.close()
            self._log_f = self._log_w = None
        return "closed"
