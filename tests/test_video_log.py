"""tests/test_video_log.py - video da corrida (tools/video_log.py), indice de movimento da camera (vision/motion.py),
"m" no vis do brain, rotas /logs, /log e /play do viz e o conversor tools/video_to_mp4.py."""
import json
import os
import socket
import sys
import tempfile
import threading
import time
import types
import urllib.error
import urllib.request

import cv2
import numpy as np

try:
    import websocket  # noqa: F401
except ImportError:
    sys.modules["websocket"] = types.ModuleType("websocket")

import brain
import viz
from config import Config, RecordConfig
from tests.test_brain import _info, _setup
from tools import flight_log, video_log
from tools.flight_log import FlightLog
from tools.video_log import VideoLog, iter_jpegs, read_index, rotate
from tools.video_to_mp4 import convert, schedule
from vision.motion import MotionMeter


class Clock:
    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t


def _frame(k, w=160, h=90):
    """Quadro com textura que anda com k (para o movimento e para distinguir os quadros)."""
    x = np.arange(w)[None, :] + 7 * k
    img = (((x // 8) % 2) * 200 + 20).astype(np.uint8).repeat(h, 0)
    return cv2.merge([img, img, img])


def _wait(cond, timeout=3.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.01)
    return False


def test_records_at_rate_closes_on_stop_and_is_readable():
    d, clk = tempfile.mkdtemp(), Clock()
    v = VideoLog(RecordConfig(fps=10.0), clock=clk)
    assert v.push(_frame(0)) is None                          # sem gravacao: nada
    assert v.start(d, "corrida_x_visao")
    got = []
    for k in range(30):                                       # 30 quadros a 30 fps = 1 s -> ~10 no video
        i = v.push(_frame(k))
        if i is not None:
            got.append(i)
            _wait(lambda: v._rec["q"].empty())
        clk.t += 1 / 30
    info = v.stop()
    assert got == list(range(len(got))) and 9 <= len(got) <= 11, got
    assert info["written"] == len(got) and not v.recording
    idx = read_index(os.path.join(d, "corrida_x_visao_indice.csv"))
    jp = list(iter_jpegs(os.path.join(d, "corrida_x_visao.mjpg"), idx))
    assert [r[0] for r in idx] == got and len(jp) == len(got)
    ts = [r[2] for r in idx]
    assert all(b - a >= 0.05 - 1e-6 for a, b in zip(ts, ts[1:])) and abs((ts[-1] - ts[0]) / (len(ts) - 1) - 0.1) < 0.01, ts
    img = cv2.imdecode(np.frombuffer(jp[0], np.uint8), cv2.IMREAD_COLOR)
    assert img.shape == (90, 160, 3)                           # resolucao original do quadro
    print(f"ok  video: {len(got)} quadros em 1 s a 30 fps (fps=10), indice casa com o .mjpg, fecha no stop")


def test_full_queue_drops_and_never_blocks():
    d, clk = tempfile.mkdtemp(), Clock()
    v = VideoLog(RecordConfig(fps=1000.0, queue_max=2), clock=clk)
    v.start(d, "r")
    gate = threading.Event()
    # trava o gravador (JPEG demorando): a fila enche e push() tem que descartar sem bloquear
    real_imencode = cv2.imencode
    video_log.cv2.imencode = lambda *a, **k: (gate.wait(5), real_imencode(*a, **k))[1]
    try:
        t0 = time.perf_counter()
        res = []
        for k in range(20):
            res.append(v.push(_frame(k)))
            clk.t += 0.01
        dt = time.perf_counter() - t0
    finally:
        gate.set()
        video_log.cv2.imencode = real_imencode
    v.stop()
    assert dt < 0.5, dt
    assert v.dropped >= 15 and sum(r is not None for r in res) <= 4, (v.dropped, res)
    print(f"ok  video: fila cheia descarta ({v.dropped} de 20) e push() nao bloqueia ({dt * 1000:.1f} ms para 20)")


def test_truncated_file_stays_readable():
    d, clk = tempfile.mkdtemp(), Clock()
    v = VideoLog(RecordConfig(fps=100.0), clock=clk)
    v.start(d, "c")
    for k in range(5):
        v.push(_frame(k))
        _wait(lambda: v._rec["q"].empty())
        clk.t += 0.02
    v.stop()
    p = os.path.join(d, "c.mjpg")
    data = open(p, "rb").read()
    open(p, "wb").write(data[:-300])                            # app caiu no meio do ultimo quadro
    with open(os.path.join(d, "c_indice.csv"), "a") as f:
        f.write("5,123,0.1")                                    # linha cortada no indice
    idx = read_index(os.path.join(d, "c_indice.csv"))
    assert len(idx) == 5
    assert len(list(iter_jpegs(p, idx))) == 4 and len(list(iter_jpegs(p))) == 4
    out = convert(p, os.path.join(d, "c.avi"), fps=10.0, avi=True)
    cap = cv2.VideoCapture(out)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    assert n >= 1, n
    print(f"ok  video cortado (crash): 4 de 5 quadros legiveis com e sem indice; converte ({n} quadros no AVI)")


def test_size_and_duration_limits():
    d, clk = tempfile.mkdtemp(), Clock()
    v = VideoLog(RecordConfig(fps=100.0, max_mb=0.004), clock=clk)    # ~4 KB: cabem poucos quadros
    v.start(d, "s")
    for k in range(20):
        v.push(_frame(k))
        _wait(lambda: v._rec is None or v._rec["q"].empty())
        clk.t += 0.02
    info = v.stop()
    size = os.path.getsize(os.path.join(d, "s.mjpg"))
    assert info["full"] and info["why"] == "max_mb" and size <= 4000 and info["written"] < 20, (info, size)
    assert "cheio" in open(os.path.join(d, "s_indice.csv")).read()
    v = VideoLog(RecordConfig(fps=10.0, max_s=1.0), clock=clk)
    v.start(d, "t")
    n = 0
    for _ in range(40):
        n += v.push(_frame(0)) is not None
        clk.t += 0.1
    info = v.stop()
    assert info["why"] == "max_s" and n <= 11, (info, n)
    print(f"ok  video: teto de tamanho ({size} B, {info['written']} quadros) e de duracao (max_s=1 -> {n} quadros)")


def test_rotation_keeps_newest():
    d = tempfile.mkdtemp()
    for i in range(5):
        for suf in (".mjpg", "_indice.csv"):
            p = os.path.join(d, f"run{i}{suf}")
            open(p, "wb").write(b"x" * 1000)
            os.utime(p, (1000 + i, 1000 + i))
    open(os.path.join(d, "run0.jsonl"), "w").write("{}\n")
    removed = rotate(d, 3, 1e9)
    assert removed == ["run0.mjpg", "run1.mjpg"] and not os.path.exists(os.path.join(d, "run0_indice.csv"))
    assert os.path.exists(os.path.join(d, "run0.jsonl"))       # o JSONL nao e apagado pela rotacao do video
    assert rotate(d, 10, 4500) == ["run2.mjpg"]                # teto de bytes (2 KB por corrida)
    v = VideoLog(RecordConfig(keep_n=2), clock=Clock())
    v.start(d, "novo")
    v.stop()
    assert sorted(f for f in os.listdir(d) if f.endswith(".mjpg")) == ["novo.mjpg", "run4.mjpg"], os.listdir(d)
    print("ok  video: rotacao apaga os mais velhos (com o indice), mantem o JSONL e abre espaco para o novo")


def test_flight_log_opens_video_with_same_name_and_vf():
    d = tempfile.mkdtemp()
    v = VideoLog(RecordConfig(fps=1000.0))
    lg = FlightLog(d, video=v).start()
    lg.recv({"t": "goal", "g": "explorar", "src": "visao"})
    assert _wait(lambda: v.recording)
    i = v.push(_frame(1))
    lg.frame({"state": "LIVRE", "ok": True, "sectors": [1] * 5, "raw": {"vf": i, "mov": 7.5, "phase": "RUN"}},
             "explorar", "visao", 0.35, 0.0)
    lg.recv({"t": "goal", "g": "parar"})
    assert _wait(lambda: not v.recording and v.current is None)
    lg.close()
    js = [f for f in os.listdir(d) if f.endswith(".jsonl")][0]
    stem = js[:-6]
    assert os.path.exists(os.path.join(d, stem + ".mjpg")) and os.path.exists(os.path.join(d, stem + "_indice.csv"))
    recs = [json.loads(l) for l in open(os.path.join(d, js), encoding="utf-8")]
    f = [r for r in recs if r["k"] == "f"][0]
    assert f["vf"] == 0 and f["mov"] == 7.5
    assert any(r["k"] == "info" and r.get("video", {}).get("written") == 1 for r in recs)
    runs = flight_log.list_runs(d)
    assert runs[0][0] == stem and [k for _, _, k in runs[0][1]] == ["log", "video", "indice"]
    print(f"ok  video abre/fecha com o JSONL ({stem}.mjpg), registro f com vf/mov, /logs agrupa por corrida")


def test_safe_path_for_downloads():
    d = tempfile.mkdtemp()
    for n in ("a.jsonl", "a.mjpg", "a_indice.csv", "x.txt"):
        open(os.path.join(d, n), "w").write("1")
    ex = flight_log.DOWNLOAD_EXTS
    assert all(flight_log.safe_path(d, n, ex) for n in ("a.jsonl", "a.mjpg", "a_indice.csv"))
    for bad in ("x.txt", "../a.mjpg", "..\\a.mjpg", "/etc/a.mjpg", ".a.mjpg", "sub/a.mjpg", "", "nao.mjpg"):
        assert flight_log.safe_path(d, bad, ex) is None, bad
    assert flight_log.safe_path(d, "a.mjpg") is None            # padrao continua so .jsonl
    print("ok  download: so log/video/indice da pasta, sem barras/'..'/ocultos")


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def test_viz_logs_page_download_and_play_do_not_block_stream():
    d, clk = tempfile.mkdtemp(), Clock()
    v = VideoLog(RecordConfig(fps=100.0), clock=clk)
    v.start(d, "corrida_1_visao")
    for k in range(3):
        v.push(_frame(k))
        _wait(lambda: v._rec["q"].empty())
        clk.t += 0.05
    v.stop()
    open(os.path.join(d, "corrida_1_visao.jsonl"), "w").write("{}\n")
    port = _free_port()
    viz.set_log_dir(d)
    viz.start(port)
    try:
        base = f"http://127.0.0.1:{port}"
        page = urllib.request.urlopen(base + "/logs", timeout=3).read().decode()
        assert "corrida_1_visao" in page and "/log/corrida_1_visao.mjpg" in page and "/play/corrida_1_visao.mjpg" in page
        body = urllib.request.urlopen(base + "/log/corrida_1_visao.mjpg", timeout=3).read()
        assert body == open(os.path.join(d, "corrida_1_visao.mjpg"), "rb").read()
        r = urllib.request.urlopen(base + "/play/corrida_1_visao.mjpg", timeout=3)
        assert r.headers["Content-Type"].startswith("multipart/x-mixed-replace")
        assert r.read().count(b"Content-Type: image/jpeg") == 3
        for bad in ("/log/..%2Fx.mjpg", "/play/x.jsonl", "/log/x.txt"):
            try:
                urllib.request.urlopen(base + bad, timeout=3)
                assert False, bad
            except urllib.error.HTTPError as e:
                assert e.code == 404
        # download lento aberto + stream: o stream continua recebendo quadros
        slow = socket.create_connection(("127.0.0.1", port))
        slow.sendall(b"GET /log/corrida_1_visao.mjpg HTTP/1.0\r\n\r\n")
        st = urllib.request.urlopen(base + "/stream", timeout=3)
        viz._last_push = 0
        viz.push(frame=_frame(0))
        assert b"--frame" in st.read(64)
        slow.close()
    finally:
        viz.stop()
    print("ok  viz: /logs lista video e indice ao lado do JSONL, /log baixa, /play toca, stream nao trava")


def test_motion_meter_moving_vs_still_in_seconds():
    for fps in (5, 10, 20):
        m = MotionMeter(dt_s=0.3)
        still = [m.update(_frame(0), k / fps) for k in range(int(2 * fps))]
        m.reset()
        moving = [m.update(_frame(k * 10 / fps), k / fps) for k in range(int(2 * fps))]
        assert still[0] is None and all(x is not None and x < 0.5 for x in still[fps:])
        assert all(x is not None and x > 10 for x in moving[fps:]), (fps, moving)
    # brilho geral mudando (auto-exposicao) nao conta como movimento
    m = MotionMeter(dt_s=0.3)
    vals = [m.update(np.clip(_frame(0).astype(int) + 3 * k, 0, 255).astype(np.uint8), k * 0.1) for k in range(10)]
    assert all(x < 0.5 for x in vals[4:]), vals
    print("ok  movimento da camera: parado < 0,5, andando > 10 a 5/10/20 fps; exposicao mudando nao conta")


def test_brain_sends_motion_in_vis_even_without_opinion():
    lk, clk = _setup("explorar", "sensor", armed=True)
    raw = json.loads(_info(ok=False))
    raw["mov"] = 2.345
    brain.step(json.dumps(raw))
    vis = lk.of("vis")[-1]
    assert vis["ok"] == 0 and vis["m"] == 2.3, vis
    clk.adv(0.2)
    brain.step(_info())                                        # sem "mov": sem "m"
    assert "m" not in lk.of("vis")[-1]
    assert len(json.dumps(vis, separators=(",", ":"))) <= brain.MAX_MSG
    print("ok  brain: vis leva o movimento da camera (m) mesmo com ok=0 (calibrando)")


def test_stream_runner_reports_mov_and_vf():
    import main
    d, clk = tempfile.mkdtemp(), Clock()
    old = video_log.REC
    video_log.REC = VideoLog(RecordConfig(fps=1000.0))
    try:
        sr = main.StreamRunner(use_edges=False, settle_s=0.0)
        video_log.REC.configure(RecordConfig(fps=1000.0))     # o StreamRunner aplica o config do app (10 fps)
        video_log.REC.start(d, "sr")
        h, w = 90, 160
        outs = []
        for k in range(6):
            rgba = cv2.cvtColor(_frame(k), cv2.COLOR_BGR2RGBA)
            outs.append(json.loads(sr.process_rgba_frame(rgba.tobytes(), w, h, w * 4, 0)))
            time.sleep(0.07)
        video_log.REC.stop()
    finally:
        video_log.REC = old
    assert [o["vf"] for o in outs] == list(range(6)), [o.get("vf") for o in outs]
    assert outs[0]["mov"] is None and outs[-1]["mov"] is not None and outs[-1]["mov"] > 1
    assert video_log.push(_frame(0)) is None                   # gravador do app parado: nada
    print("ok  main: cada quadro traz vf (indice no video) e mov (movimento da camera)")


def test_converter_constant_rate_schedule():
    assert schedule([0.0, 0.1, 0.2, 0.5], 10) == [0, 1, 2, 2, 2, 3]   # celular pulou 0,3 s: repete o quadro
    assert schedule([0.0, 0.05, 0.1], 10) == [0, 2]                  # sobrou quadro: descarta o do meio
    print("ok  conversor: taxa constante pelos tempos do indice (repete quando o celular pulou)")


def test_converter_finds_video_from_jsonl_stem_and_folder():
    from tools import video_to_mp4 as vm
    d = tempfile.mkdtemp()
    stem = os.path.join(d, "corrida_2026-11-26_13h01m18_sensor")
    open(stem + ".mjpg", "wb").close()
    open(stem + ".jsonl", "w").close()
    for arg in (stem, stem + ".jsonl", stem + ".mjpg", stem + "_indice.csv"):
        assert vm.video_path(arg) == stem + ".mjpg", arg
    assert vm.pending(d) == [stem + ".mjpg"]
    open(stem + ".mp4", "wb").close()
    assert vm.pending(d) == []
    try:
        vm.video_path(os.path.join(d, "corrida_sem_video.jsonl"))
        assert False, "devia falhar"
    except SystemExit as e:
        assert "nao achei" in str(e)
