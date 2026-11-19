"""tests/test_sectors.py - free_space_vector -> 5 setores (-30..+30) e orientacao esquerda/direita."""
import json

import cv2
import numpy as np

from main import StreamRunner, frame_from_rgba
from tests.test_stream import _rgba_bytes
from vision.coverage_log import WARMUP
from vision.ground import SECTOR_ANGLES, free_sectors, free_space_vector


def test_free_sectors_reduction():
    vec = np.ones(40)
    vec[0:8] = 0.0                       # setor -30 todo bloqueado
    vec[16:24] = [1, 1, 1, 1, 1, 0, 0, 0]  # setor 0: 3 de 8 faixas bloqueadas
    out = free_sectors(vec)
    assert len(out) == 5 == len(SECTOR_ANGLES)
    assert out[0] == 0 and out[1] == 100 and out[3] == 100 and out[4] == 100
    assert out[2] == 0, out              # percentil 20 de 8 faixas: 3 zeros -> 0 (pessimista)
    vec[16:24] = [1, 1, 1, 1, 1, 1, 1, 0]
    assert 0 < free_sectors(vec)[2] <= 100
    print("ok  free_sectors: 5 setores, percentil 20, 0..100")


def test_free_space_vector_has_40_strips_at_320px():
    mask = np.full((180, 320), 255, np.uint8)
    assert len(free_space_vector(mask)) == 40
    print("ok  320 px / 8 = 40 faixas")


def _scene(block_left, w=640, h=360, seed=0):
    rng = np.random.default_rng(seed)
    img = np.clip(np.full((h, w, 3), (170, 180, 190), np.int16) + rng.integers(-8, 8, (h, w, 3)), 0, 255).astype(np.uint8)
    if block_left:                       # obstaculo vermelho no terco ESQUERDO, de baixo ate o meio
        img[int(h * 0.35):, : w // 3] = (30, 30, 200)
    return img


def _sensor_bytes(upright_bgr):
    """Simula o sensor do S9: a imagem 'em pe' sai do sensor girada 90 graus anti-horario;
    o app informa rotationDegrees=90 (girar horario para ficar em pe)."""
    sensor = cv2.rotate(upright_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return _rgba_bytes(sensor)


def test_left_of_image_is_left_sector_after_rotation():
    sr = StreamRunner(settle_s=0.0)
    for i in range(WARMUP + 2):
        data, stride = _sensor_bytes(_scene(False, seed=i))
        sr.process_rgba_frame(data, 360, 640, stride, 90)   # sensor: largura 360 x altura 640
    last = None
    for i in range(3):
        data, stride = _sensor_bytes(_scene(True, seed=50 + i))
        last = json.loads(sr.process_rgba_frame(data, 360, 640, stride, 90))
    assert last["phase"] == "RUN" and last["ok"] is True, last
    s = last["sectors"]
    assert len(s) == 5
    assert s[0] < 40 and s[1] < 40 and s[4] > 80 and s[3] > 80, s
    print(f"ok  obstaculo a esquerda -> setores {s} (esq baixo, dir alto), rotation=90 tratada")


def test_info_flags_and_fields():
    sr = StreamRunner(settle_s=100.0)
    data, stride = _rgba_bytes(_scene(False)[:36 * 2, :64 * 2][:36, :64])
    info = json.loads(sr.process_rgba_frame(data, 64, 36, stride, 0))
    assert info["phase"] == "SETTLE" and info["ok"] is False and info["sectors"] is None
    sr2 = StreamRunner(settle_s=0.0)
    info = json.loads(sr2.process_rgba_frame(data, 64, 36, stride, 0))
    assert info["phase"] == "WARMUP" and info["ok"] is False
    for k in ("vision_state", "r", "sectors", "edge_h_cen", "edge_cov", "edge_near", "ok"):
        assert k in info, k
    print("ok  info ao vivo: ok=False em SETTLE/WARMUP, campos vision_state/r/sectors/edge_*")
