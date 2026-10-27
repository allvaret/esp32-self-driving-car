import json
import cv2
import numpy as np
from main import StreamRunner, frame_from_rgba
from vision.coverage_log import WARMUP


def _rgba_bytes(bgr, pad=16):
    h, w = bgr.shape[:2]
    stride = w * 4 + pad
    buf = np.zeros((h, stride), np.uint8)
    buf[:, :w * 4] = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGBA).reshape(h, -1)
    return buf.tobytes()[:-pad], stride          # ultima linha sem padding, como no Android


def test_stride_and_rotation():
    bgr = np.random.randint(0, 255, (36, 64, 3), np.uint8)
    data, stride = _rgba_bytes(bgr)
    assert np.array_equal(frame_from_rgba(data, 64, 36, stride, 0), bgr)
    out = frame_from_rgba(data, 64, 36, stride, 90)
    assert np.array_equal(out, cv2.rotate(bgr, cv2.ROTATE_90_CLOCKWISE))


def test_stream_reaches_run():
    rng = np.random.default_rng(0)
    floor = np.clip(np.full((360, 640, 3), (170, 180, 190), np.int16)
                    + rng.integers(-8, 8, (360, 640, 3)), 0, 255).astype(np.uint8)
    data, stride = _rgba_bytes(floor)
    sr = StreamRunner(settle_s=0.0)
    phases = [json.loads(sr.process_rgba_frame(data, 640, 360, stride, 0))["phase"]
              for _ in range(WARMUP + 4)]
    assert "ERROR" not in phases, phases
    assert phases[-1] == "RUN"

