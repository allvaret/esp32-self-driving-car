"""tests/test_pipeline_smoke.py - exercita a visao inteira em frames sinteticos (sem video).

Existe para pegar incompatibilidades de API do OpenCV. Rode em DOIS ambientes:
  - o do dia a dia (ex.: OpenCV 4.8 / numpy 1.26.4), e
  - um espelho do celular: Python 3.9 + numpy==1.26.2 + opencv-python-headless==4.5.1.48
    (o OpenCV 4.5.1 e mais estrito: nao aceita LISTA onde espera tupla, ex. cv2.resize(img, [w, h])).
Da raiz do repositorio:   python -m tests.test_pipeline_smoke
"""
import numpy as np

from config import Config
from vision import edge_probe
from vision.coverage_log import coverage_row, floor_prob
from vision.decision import DecisionMaker
from vision.ground import calibrate_thr, floor_mask, free_space_vector, learn_floor_hist


def synth_frame(shift=0, size=(480, 848)):
    h, w = size
    rng = np.random.default_rng(shift)
    f = np.full((h, w, 3), (125, 125, 130), np.uint8)
    f[int(h * 0.55):] = (150, 155, 160)                       # chao
    f[int(h * 0.55) - 10:int(h * 0.55) + 6] = (60, 60, 65)    # rodape
    return np.clip(f.astype(int) + rng.integers(-3, 4, f.shape), 0, 255).astype(np.uint8)


def test_pipeline_on_synthetic_frames():
    frames = [synth_frame(i) for i in range(12)]
    hist = learn_floor_hist(frames[0])
    thr, _ = calibrate_thr(frames[1:], hist)
    dm = DecisionMaker(Config().decision)
    for i, fr in enumerate(frames):
        t = i * 0.1
        mask = floor_mask(fr, hist, thr)
        prob, hsv = floor_prob(fr, hist)
        assert mask.ndim == 2 and mask.shape == prob.shape == hsv.shape[:2]
        row = coverage_row(mask, prob, hsv, i, t)
        vec = free_space_vector(mask)
        assert len(vec) > 0 and 0.0 <= float(np.min(vec)) and float(np.max(vec)) <= 1.0
        dm.update(row["prob_mean_cen"], row["unknown_inf"], row["unknown_cen"], t)
        small = edge_probe.resize_w(fr)
        import cv2
        gray = cv2.GaussianBlur(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY), (3, 3), 0)
        feats, horiz, heights = edge_probe.edge_features(gray)
        assert 0.0 <= feats["h_cen"] <= 1.0 and len(heights) > 0
    print("ok  pipeline sintetico (ground, thr auto, coverage, decision, bordas)")


if __name__ == "__main__":
    test_pipeline_on_synthetic_frames()
    print("todos os testes passaram")