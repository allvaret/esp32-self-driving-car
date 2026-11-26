"""tests/test_firmware_sim.py - compila o simulador host (g++) e roda sim/check_firmware.py.
Sem g++ no PC o teste e pulado (imprime SKIP). Rode:  python -m tests.run_all"""
import os
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIM = os.path.join(ROOT, "sim")


def test_firmware_scenarios_in_simulator():
    gxx = shutil.which("g++")
    if not gxx:
        print("SKIP  firmware no simulador: g++ nao encontrado")
        return
    exe = "sim3.exe" if os.name == "nt" else "sim3"
    r = subprocess.run([gxx, "-std=c++17", "-O1", "-static", "-DESP_ARDUINO_VERSION_MAJOR=3", "-I.", "-o", exe, "sim.cpp"],
                       cwd=SIM, capture_output=True, text=True)
    assert r.returncode == 0, r.stderr[-2000:]
    r = subprocess.run([sys.executable, "check_firmware.py"], cwd=SIM, capture_output=True, text=True)
    fails = [l for l in r.stdout.splitlines() if l.startswith("FALHA")]
    assert r.returncode == 0 and not fails, "\n".join(fails) + r.stderr[-1500:]
    print(f"ok  firmware no simulador: {sum(l.startswith('ok') for l in r.stdout.splitlines())} verificacoes")
