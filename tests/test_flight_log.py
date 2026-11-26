"""tests/test_flight_log.py - log local JSONL (um arquivo por goal) e as rotas /logs e /log/<nome> do viz."""
import json
import os
import socket
import sys
import tempfile
import time
import types
import urllib.error
import urllib.request

try:
    import websocket  # noqa: F401
except ImportError:
    sys.modules["websocket"] = types.ModuleType("websocket")

import brain
import viz
from tests.test_brain import Clock, FakeLink, _info
from tools.flight_log import FlightLog, list_logs, safe_path


def _lines(path):
    return [json.loads(l) for l in open(path, encoding="utf-8") if l.strip()]


def test_one_file_per_goal_and_everything_is_logged():
    d = tempfile.mkdtemp()
    lg = FlightLog(d).start()
    lg.recv({"t": "state", "armed": False})                      # sem goal aberto: nao grava
    lg.recv({"t": "goal", "g": "explorar", "src": "visao"})
    lg.sent({"t": "vis", "ok": 1, "s": [1, 2, 3, 4, 5]})
    lg.sent({"t": "drive", "v": 0.35, "s": 0.0})
    lg.sent({"t": "yaw", "psi": 1.0, "r": 0.5})
    lg.sent({"t": "avoid", "deg": -15})
    lg.frame({"state": "LIVRE", "conf": 0.9, "ok": True, "sectors": [1, 2, 3, 4, 5],
              "raw": {"t": 1.2, "n": 7, "r": 0.9, "edge_h_cen": 0.4, "edge_cov": 0.1, "edge_near": False, "phase": "RUN", "proc_ms": 30.0}},
             "explorar", "visao", 0.35, 0.01)
    for m in ({"t": "state", "armed": True}, {"t": "range", "deg": 0.0, "cm": 120.0}, {"t": "scan", "k": "decide", "ang": -15},
              {"t": "log", "msg": "ARMADO"}):
        lg.recv(m)
    lg.recv({"t": "goal", "g": "parar", "src": "sensor"})        # fecha o 1o arquivo
    lg.recv({"t": "state", "armed": False})                      # sem arquivo: nao grava
    time.sleep(0.4)
    lg.recv({"t": "goal", "g": "explorar", "src": "sensor"})     # 2o arquivo (outro goal/src)
    lg.recv({"t": "state", "armed": True})
    lg.close()
    files = sorted((n for n, _ in list_logs(d)), key=lambda n: n.endswith("_visao.jsonl"))   # [sensor, visao]
    assert len(files) == 2 and all(f.startswith("corrida_") for f in files), files
    assert files[0].endswith("_sensor.jsonl") and files[1].endswith("_visao.jsonl"), files
    a = _lines(os.path.join(d, files[1]))
    kinds = [(r["k"], (r.get("m") or {}).get("t")) for r in a]
    for want in (("tx", "vis"), ("tx", "drive"), ("tx", "yaw"), ("tx", "avoid"), ("f", None), ("rx", "state"), ("rx", "range"),
                 ("rx", "scan"), ("rx", "log"), ("rx", "goal")):
        assert want in kinds, (want, kinds)
    assert kinds[-1] == ("rx", "goal") and a[-1]["m"]["g"] == "parar"
    f = [r for r in a if r["k"] == "f"][0]
    assert f["vs"] == "LIVRE" and f["sec"] == [1, 2, 3, 4, 5] and f["e"] == {"h": 0.4, "c": 0.1, "n": 0} and f["v"] == 0.35 and "w" in f
    assert len(_lines(os.path.join(d, files[0]))) == 2           # goal explorar + state; o resto depois de parar nao entra
    assert lg.dropped == 0
    print(f"ok  log local: 1 arquivo por goal ({files[0][:22]}.. / {files[1][:20]}..), frame/tx/rx gravados, nada fora do goal")


def test_brain_writes_frames_and_tx_with_a_thread():
    d = tempfile.mkdtemp()
    from tests.test_brain import _setup
    lk, clk = _setup("explorar", "visao", armed=True)
    brain._logger = FlightLog(d).start()
    brain._on_msg({"t": "goal", "g": "explorar", "src": "visao"})
    t0 = time.monotonic()
    for _ in range(50):
        brain.step(_info("LIVRE"))
        clk.adv(0.05)
    assert time.monotonic() - t0 < 1.0                          # so enfileira: nao bloqueia o laco
    brain._on_msg({"t": "state", "armed": True, "st": "FRENTE"})
    brain._logger.close()
    (name, _), = list_logs(d)
    recs = _lines(os.path.join(d, name))
    ks = {}
    for r in recs:
        ks[r["k"]] = ks.get(r["k"], 0) + 1
    assert ks.get("f") == 50 and ks.get("tx", 0) > 50 and ks.get("rx") == 2, ks
    assert {(r["m"] or {}).get("t") for r in recs if r["k"] == "tx"} >= {"vis", "drive", "mode", "think"}
    brain._logger = None
    print(f"ok  brain grava {ks} no JSONL sem bloquear o laco da camera")


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def test_viz_routes_list_and_download_logs():
    d = tempfile.mkdtemp()
    body = b'{"k":"rx","w":1,"m":{"t":"state"}}\n' * 3
    open(os.path.join(d, "explorar_visao_20260101_000000.jsonl"), "wb").write(body)
    open(os.path.join(d, "segredo.txt"), "w").write("nao")
    port = _free_port()
    viz.set_log_dir(d)
    viz.start(port)
    try:
        base = f"http://127.0.0.1:{port}"
        html_ = urllib.request.urlopen(base + "/logs", timeout=5).read().decode()
        assert "explorar_visao_20260101_000000.jsonl" in html_ and "segredo" not in html_
        r = urllib.request.urlopen(base + "/log/explorar_visao_20260101_000000.jsonl", timeout=5)
        assert r.read() == body and "attachment" in r.headers["Content-Disposition"]
        for bad in ("/log/segredo.txt", "/log/..%2Fconfig.py", "/log/%2e%2e%2f%2e%2e%2fconfig.py", "/log/nao_existe.jsonl", "/log/"):
            try:
                urllib.request.urlopen(base + bad, timeout=5)
                raise AssertionError("deveria dar 404: " + bad)
            except urllib.error.HTTPError as e:
                assert e.code == 404, (bad, e.code)
    finally:
        viz.stop()
        viz.set_log_dir(None)
    assert safe_path(d, "../x.jsonl") is None and safe_path(d, ".oculto.jsonl") is None
    print("ok  viz: /logs lista, /log/<nome> baixa; so .jsonl da pasta (traversal -> 404)")


def test_run_name_is_readable_and_sorts_by_time():
    from tools.flight_log import run_name
    t = time.mktime((2026, 11, 26, 13, 1, 18, 0, 0, -1))
    assert run_name("explorar", "sensor", t) == "corrida_2026-11-26_13h01m18_sensor"
    assert run_name("explorar", None, t) == "corrida_2026-11-26_13h01m18_sensor"
    assert run_name("seguir", "visao", t) == "corrida_2026-11-26_13h01m18_visao_seguir"
    assert run_name("explorar", "visao", t + 3600) > run_name("explorar", "sensor", t)
