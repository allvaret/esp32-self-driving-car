"""tests/test_live_path.py - o caminho AO VIVO nao importa controller nem mocks (nem indiretamente),
e o conjunto copiado pelo syncPython do Gradle basta para o app (ao vivo + bancada).
Rode:  python -m tests.run_all"""
import os
import re
import shutil
import subprocess
import sys
import tempfile
import textwrap

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FORBIDDEN = ("control.controller", "control.mocks", "replay", "replay.offline")

_STUB = ("import sys, types\n"
         "try:\n    import websocket\nexcept ImportError:\n    sys.modules['websocket'] = types.ModuleType('websocket')\n")


def _py(code, cwd):
    pre = "import sys; sys.path.insert(0, %r)\n" % cwd     # -I ignora cwd/PYTHONPATH: poe a pasta na mao
    r = subprocess.run([sys.executable, "-I", "-c", pre + _STUB + textwrap.dedent(code)], cwd=cwd,
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    return r.stdout


def test_live_modules_do_not_import_controller_or_mocks():
    out = _py("""
        import sys, main, brain, viz
        import tools.ws_link
        bad = [m for m in %r if m in sys.modules]
        print("BAD", bad)
        assert not bad, bad
    """ % (FORBIDDEN,), ROOT)
    assert "BAD []" in out
    print("ok  main/brain/viz/ws_link nao importam controller, mocks nem replay")


def _gradle_includes():
    s = open(os.path.join(ROOT, "android", "app", "build.gradle.kts"), encoding="utf-8").read()
    block = s[s.index("val syncPython"):s.index("into(layout")]
    return re.findall(r'"([^"]+\.py|[^"]+/\*\.py)"', block)


def _sync_copy(dst):
    import glob
    for pat in _gradle_includes():
        for f in glob.glob(os.path.join(ROOT, pat)):
            rel = os.path.relpath(f, ROOT)
            os.makedirs(os.path.dirname(os.path.join(dst, rel)) or dst, exist_ok=True)
            shutil.copy(f, os.path.join(dst, rel))


def test_sync_set_runs_live_and_bench():
    d = tempfile.mkdtemp()
    _sync_copy(d)
    assert not os.path.exists(os.path.join(d, "replay", "replay_main.py"))
    # ao vivo + bancada, so com o que o syncPython copia
    out = _py("""
        import sys, tempfile, os
        import cv2, numpy as np
        import main, brain, viz
        from tools import android_entry
        d = tempfile.mkdtemp()
        rng = np.random.default_rng(0)
        for i in range(16):
            img = np.clip(np.full((90, 160, 3), (170, 180, 190), np.int16) + rng.integers(-6, 6, (90, 160, 3)), 0, 255).astype(np.uint8)
            cv2.imwrite(os.path.join(d, "f%03d.jpg" % i), img)
        txt = android_entry.run_main(d, True, 16)
        assert "frames" in txt, txt
        assert "ok" in android_entry.bench(d, 1, True, 16).lower() or "etapa" in android_entry.bench(d, 1, True, 16)
        print("BANCADA_OK")
    """, d)
    assert "BANCADA_OK" in out
    print("ok  syncPython copia o necessario: app de bancada (run_main/bench) funciona")
