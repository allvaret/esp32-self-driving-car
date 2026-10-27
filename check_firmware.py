"""check_firmware.py - cenarios de verificacao do car_firmware.ino no simulador (sem hardware).

Compile antes (ver esp32/README_FIRMWARE.md):
    cd esp32/sim && g++ -std=c++17 -DESP_ARDUINO_VERSION_MAJOR=3 -I. -o sim3 sim.cpp
Rode:  python check_firmware.py        (sai com codigo 1 se algo falhar)
"""
import json, os, subprocess, sys, tempfile
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..")))   # raiz do repositorio (control.hal)
def run(inputs, args=(), t_end=20000, exe=os.path.join(HERE, "sim3")):
    with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as f:
        for ms, s in sorted(inputs): f.write(f"{ms}\t{s}\n")
    out = subprocess.run([exe, "--t-end", str(t_end), "--input", f.name, *map(str, args)], capture_output=True, text=True).stdout.splitlines()
    tr, o = [], []
    for l in out:
        p = l.split()
        if l.startswith("TR "):
            d = dict(kv.split("=") for kv in p[2:]); d["t"] = int(p[1]); tr.append(d)
        elif l.startswith("OUT "): o.append((int(p[1]), l.split(" ", 2)[2]))
    return tr, o
def drives(t0, t1, v, s=0.0, step=100):
    return [(t, json.dumps({"t": "drive", "v": v, "s": s}, separators=(",", ":"))) for t in range(t0, t1, step)]
def at(tr, t): return min(tr, key=lambda r: abs(r["t"] - t))
ok = lambda c, m: print(("ok   " if c else "FALHA"), m) or c
base = [(100, '{"t":"mode","m":"phone"}'), (200, "go")]       # arma aos 3.2 s
fails = 0

# 1) aim + range, drive, watchdog
inp = base + [(3500, '{"t":"aim","deg":-45}'), (4000, '{"t":"aim","deg":0}')] + drives(4500, 6500, 0.4)
tr, o = run(inp, t_end=9000)
rng = [json.loads(m) for t, m in o if m.startswith('{"t":"range"')]
m45 = [r for r in rng if r["deg"] == -45.0]
fails += not ok(len(m45) > 0 and m45[0]["cm"] is None, f"range com deg=-45.0 e cm=null (sem obstaculo): {len(m45)} msgs")
fails += not ok(abs(float(at(tr, 6000)["curL"]) - 0.4) < 0.02, f"drive v=0.4 -> motores em {at(tr,6000)['curL']}")
fails += not ok(float(at(tr, 7500)["curL"]) == 0.0, f"watchdog: sem drive por 500 ms -> motores {at(tr,7500)['curL']} aos 7.5 s")
fails += not ok(not any(r["deg"] == -45.0 and r["cm"] is not None for r in rng), "nenhum range de -45 sem eco foi mal rotulado")

# 2) reflexo: obstaculo a 15 cm (< hard_stop 20) com drive continuo; e a 30 cm (> 20) nao freia
tr, _ = run(base + drives(4000, 12000, 0.4), ["--obst", 9000, 15], t_end=12000)
fails += not ok(float(at(tr, 8800)["curL"]) > 0.3 and float(at(tr, 9400)["curL"]) == 0.0, f"reflexo a 15 cm: antes {at(tr,8800)['curL']} depois {at(tr,9400)['curL']}")
tr, _ = run(base + drives(4000, 12000, 0.4), ["--obst", 9000, 30], t_end=12000)
fails += not ok(float(at(tr, 9400)["curL"]) > 0.3, f"a 30 cm (> hard_stop) o ESP32 NAO freia (decisao e do celular): {at(tr,9400)['curL']}")

# 3) servo fora do centro: sem leitura central fresca nao avanca
inp = base + drives(4000, 12000, 0.4) + [(9000, '{"t":"aim","deg":-45}')]
tr, _ = run(inp, t_end=12000)
fails += not ok(float(at(tr, 8800)["curL"]) > 0.3 and float(at(tr, 10000)["curL"]) == 0.0, f"servo a -45 por > fresh_ms: motores {at(tr,8800)['curL']} -> {at(tr,10000)['curL']}")

# 4) limite de re continua (max_rev_ms=1500)
tr, _ = run(base + drives(4000, 9000, -0.3), t_end=10000)
rev = [r for r in tr if float(r["curL"]) < -0.05]
dur = (rev[-1]["t"] - rev[0]["t"]) if rev else 0
fails += not ok(0 < dur <= 1700 and float(at(tr, 8000)["curL"]) == 0.0, f"re limitada: {dur} ms de re continua (max 1500), parada aos 8 s")

# 5) nao arma sozinho / desarma
tr, _ = run([(100, '{"t":"mode","m":"phone"}')] + drives(1000, 8000, 0.4), t_end=9000)
fails += not ok(all(float(r["curL"]) == 0.0 for r in tr), "desarmado: ignora drive (motores sempre 0)")
tr, _ = run(base + drives(4000, 9000, 0.4) + [(6000, "stop")], t_end=9000)
fails += not ok(float(at(tr, 5900)["curL"]) > 0.3 and float(at(tr, 6500)["curL"]) == 0.0 and all(float(r["curL"]) == 0.0 for r in tr if r["t"] > 6500), "comando `stop` desarma e para")
# botao BOOT arma/desarma
tr, _ = run([(100, '{"t":"mode","m":"phone"}'), (300, "@btn_down"), (500, "@btn_up")] + drives(4000, 6000, 0.4) + [(7000, "@btn_down"), (7200, "@btn_up")], t_end=9000)
fails += not ok(at(tr, 5000)["armed"] == "1" and at(tr, 8000)["armed"] == "0", "botao BOOT: arma (3 s) e desarma")

# 6) cfg e comandos de texto
tr, o = run(base + [(3300, '{"t":"cfg","trim_left":0.8,"wd_ms":1000}')] + drives(4000, 7000, 0.5), t_end=8000)
fails += not ok(any('"cfg_ok"' in m for t, m in o) and abs(float(at(tr, 6500)["curL"]) - 0.4) < 0.02 and abs(float(at(tr, 6500)["curR"]) - 0.5) < 0.02,
                f"cfg trim_left=0.8: L={at(tr,6500)['curL']} R={at(tr,6500)['curR']} (esperado 0.40 / 0.50)")
tr, o = run([(100, "set trim_right 0.9"), (200, "get"), (300, "help"), (400, "set nao_existe 1")], t_end=1000)
txt = " | ".join(m for t, m in o)
fails += not ok("trim_right = 0.900" in txt and "parametro desconhecido: nao_existe" in txt and "d_stop_cm = 50.000" in txt, "comandos de texto: set/get/help")

# 7) lixo na serial nao derruba
junk = [(1000, "{garbage"), (1100, "\x01\x02\x03"), (1200, "{" + "x" * 400), (1300, '{"t":"drive","v":abc}'), (1400, '{"t":"nada"}'), (1500, "")]
tr, o = run(base + junk + drives(4000, 6000, 0.4), t_end=7000)
fails += not ok(abs(float(at(tr, 5500)["curL"]) - 0.4) < 0.02, "lixo/linha longa/JSON invalido ignorados; drive seguinte funciona")

# 8) falha 'far' (perna fina) nao e vista; 'none' = sem eco = livre
tr, _ = run(base + drives(4000, 12000, 0.4), ["--obst", 9000, 15, "--far", 8000, 12000], t_end=12000)
fails += not ok(float(at(tr, 10000)["curL"]) > 0.3, "[limitacao conhecida] objeto invisivel ao HC-SR04 nao aciona o reflexo")

# ---------------- modo AUTO e formatos Python <-> firmware ----------------
from control.hal import Drive, LinkDrive, LinkRangeHead, MemoryLink
tr, o = run([(500, "go")], ["--obst", 8000, 30, "--left", 200, "--right", 40], t_end=30000)
fails += not ok(abs(float(at(tr, 6000)["curL"]) - 0.5) < 0.02 and abs(float(at(tr, 6000)["curR"]) - 0.5) < 0.02, f"auto: CRUISE a v=0.5 -> L={at(tr,6000)['curL']} R={at(tr,6000)['curR']}")
fails += not ok(any(int(r["steerPWM"]) < 4000 for r in tr if r["sub"] == "6"), "auto: curva para a ESQUERDA quando a esquerda e mais livre")
tr, _ = run([(500, "go")], ["--obst", 8000, 30, "--left", 40, "--right", 200], t_end=12000)
fails += not ok(any(int(r["steerPWM"]) > 5800 for r in tr if r["sub"] == "6"), "auto: curva para a DIREITA quando a direita e mais livre")
tr, _ = run([(500, "go")], [], t_end=15000)
fails += not ok(all(r["cst"] == "0" for r in tr) and abs(float(at(tr, 14000)["curL"]) - 0.5) < 0.02, "auto: sem obstaculo fica em CRUISE")
tr, _ = run([(500, "go")], ["--obst", 8000, 30, "--clear", 9000], t_end=20000)
fails += not ok(at(tr, 19000)["cst"] in ("0", "1"), "auto: obstaculo removido -> volta a andar")
tr, _ = run([(500, "go")], ["--obst", 8000, 70], t_end=12000)
fails += not ok(at(tr, 10000)["cst"] == "1" and abs(float(at(tr, 10000)["curL"]) - 0.2) < 0.02, f"auto: 70 cm -> SCAN a v=0.2 ({at(tr,10000)['curL']})")
tr, _ = run([(500, "go")], ["--obst", 8000, 30, "--left", 30, "--right", 30, "--clear", 25000], t_end=40000)
fails += not ok(any(r["sub"] == "7" for r in tr) and at(tr, 39000)["cst"] in ("0", "1"), "auto: sem saida -> HOLD -> libera quando o centro libera")
tr, o = run([(500, "go")], ["--obst", 8000, 30, "--left", 200, "--right", 40], t_end=45000)
link = MemoryLink(); head = LinkRangeHead(link, settle_s=0.15)
head.aim(-45.0, 3.5); aim_line = json.dumps(link.sent[-1], separators=(",", ":"))
aim0 = json.dumps({"t": "aim", "deg": 0.0}, separators=(",", ":"))
LinkDrive(link).send(5.0, Drive(0.4, -1)); drive_line = json.dumps(link.sent[-1], separators=(",", ":"))
tr, o = run([(100, '{"t":"mode","m":"phone"}'), (200, "go"), (3500, aim_line), (4000, aim0)] + [(t, drive_line) for t in range(4500, 6000, 100)],
            ["--right", 123.4, "--left", 77.7], t_end=7000)
rng = [json.loads(m) for t, m in o if m.startswith('{"t":"range"')]
left = [r for r in rng if r["deg"] == -45.0]
fails += not ok(left and abs(left[-1]["cm"] - 77.7) < 1.0, "firmware aceita o aim gerado pelo hal.py e responde range deg=-45.0")
fails += not ok(abs(float(at(tr, 5800)["curL"]) - 0.4) < 0.02 and int(at(tr, 5800)["steerPWM"]) == 3833, "firmware aceita o drive gerado pelo hal.py (v=0.4, s=-1)")
h2 = LinkRangeHead(l2 := MemoryLink(), settle_s=0.15); h2.aim(-45.0, 0.0); l2.push(left[-1]); r = h2.read(0.3)
fails += not ok(r.settled and abs(r.dist_m - 0.777) < 0.01, f"LinkRangeHead le a mensagem do firmware: {r.dist_m} m")
l2.push({"t": "range", "deg": -45.0, "cm": None}); fails += not ok(h2.read(0.4).dist_m is None, "cm=null -> dist_m=None (sem eco)")

print("\nFALHAS:", fails); sys.exit(1 if fails else 0)