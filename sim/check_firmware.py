"""check_firmware.py - cenarios de verificacao do car_firmware.ino no simulador (sem hardware).

Compile antes:
    cd sim && g++ -std=c++17 -DESP_ARDUINO_VERSION_MAJOR=3 -I. -o sim3 sim.cpp
Rode:  python check_firmware.py        (sai com codigo 1 se algo falhar)

Mundo do simulador (sim.cpp): sensor por angulo (-30,-15,0,+15,+30 graus), obstaculo temporario no centro
(--obst t cm [--clear t]), janelas sem eco (--none t0 t1), yaw simulado das rodas (--yaw --world-dps N).
Relogio virtual: `go` aos 200 ms arma aos 4200 ms (ARM_DELAY_MS = 4000).
"""
import json, math, os, re, subprocess, sys, tempfile
HERE = os.path.dirname(os.path.abspath(__file__))
def _repo_root(start):                         # sobe ate achar a pasta com control/hal.py
    d = start
    for _ in range(6):
        if os.path.exists(os.path.join(d, "control", "hal.py")):
            return d
        d = os.path.dirname(d)
    raise SystemExit("Nao achei a raiz do repositorio (pasta com control/hal.py) acima de " + start)
sys.path.insert(0, _repo_root(HERE))

EXE = os.path.join(HERE, "sim3.exe" if os.path.exists(os.path.join(HERE, "sim3.exe")) else "sim3")
LAST = {"ev": [], "ws": []}


def run(inputs, args=(), t_end=20000):
    """Roda o simulador. Devolve (TR, OUT); eventos de estado em LAST['ev'], linhas WSC em LAST['ws']."""
    with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as f:
        for ms, s in sorted(inputs, key=lambda x: x[0]): f.write(f"{ms}\t{s}\n")
    out = subprocess.run([EXE, "--t-end", str(t_end), "--input", f.name, *map(str, args)], capture_output=True, text=True).stdout.splitlines()
    os.unlink(f.name)
    tr, o, ev, ws = [], [], [], []
    for l in out:
        p = l.split()
        if l.startswith("TR "):
            d = dict(kv.split("=", 1) for kv in p[2:]); d["t"] = int(p[1]); tr.append(d)
        elif l.startswith("OUT "): o.append((int(p[1]), l.split(" ", 2)[2]))
        elif l.startswith("EV "):
            head, _, why = l.partition(" why=")
            d = dict(kv.split("=", 1) for kv in head.split()[2:]); d["t"] = int(head.split()[1]); d["why"] = why; ev.append(d)
        elif l.startswith("WSC "):
            _, ms, num, txt = l.split(" ", 3); ws.append((int(ms), int(num), txt))
    LAST["ev"], LAST["ws"] = ev, ws
    return tr, o


def drives(t0, t1, v, s=0.0, step=100):
    return [(t, json.dumps({"t": "drive", "v": v, "s": s}, separators=(",", ":"))) for t in range(t0, t1, step)]
def vis(t0, t1, s, ok=1, step=100):
    return [(t, json.dumps({"t": "vis", "ok": ok, "s": s}, separators=(",", ":"))) for t in range(t0, t1, step)]
def at(tr, t): return min(tr, key=lambda r: abs(r["t"] - t))
def decides(o): return [json.loads(m) for t, m in o if m.startswith('{"t":"scan"') and '"k":"decide"' in m]
def evs(subn=None, why=None):
    return [e for e in LAST["ev"] if (subn is None or e["subn"] == subn) and (why is None or why in e["why"])]
ok = lambda c, m: print(("ok   " if c else "FALHA"), m) or c
base = [(100, '{"t":"mode","m":"phone"}'), (200, "go")]       # arma aos 4.2 s
AUTO = [(500, "go")]                                           # auto: arma aos 4.5 s
fails = 0

# ===================== modo phone: reflexos e protocolo =====================
# 1) aim + range, drive, watchdog (fallback_auto=1: sem celular o ESP32 segue so com o sensor)
inp = base + [(3500, '{"t":"aim","deg":-45}'), (4000, '{"t":"aim","deg":0}')] + drives(5000, 7000, 0.4)
tr, o = run(inp, t_end=10000)
rng = [json.loads(m) for t, m in o if m.startswith('{"t":"range"')]
m45 = [r for r in rng if r["deg"] == -45.0]
fails += not ok(len(m45) > 0 and m45[0]["cm"] is None, f"range com deg=-45.0 e cm=null (sem obstaculo): {len(m45)} msgs")
fails += not ok(abs(float(at(tr, 6500)["curL"]) - 0.4) < 0.02, f"drive v=0.4 -> motores em {at(tr,6500)['curL']}")
fails += not ok(at(tr, 8500)["delegated"] == "1" and abs(float(at(tr, 8500)["curL"]) - 0.35) < 0.02,
                f"watchdog: sem drive por 500 ms -> ESP32 assume e anda so com o sensor (v_cruise): {at(tr,8500)['curL']}")
tr, o = run(inp, ["--set", "fallback_auto", 0], t_end=10000)
fails += not ok(float(at(tr, 8500)["curL"]) == 0.0, "watchdog com fallback_auto=0: para")

# 2) reflexo: obstaculo a 15 cm (< hard_stop 20) com drive continuo
tr, _ = run(base + drives(5000, 13000, 0.4), ["--obst", 9000, 15], t_end=13000)
fails += not ok(float(at(tr, 8800)["curL"]) > 0.3 and abs(float(at(tr, 9800)["curL"])) < 0.35 and at(tr, 9800)["delegated"] == "1",
                f"reflexo/veto a 15 cm: antes {at(tr,8800)['curL']} depois {at(tr,9800)['curL']} (ESP32 assumiu)")

# 3) servo fora do centro: sem leitura central fresca nao avanca
inp = base + drives(5000, 13000, 0.4) + [(9000, '{"t":"aim","deg":-45}')]
tr, _ = run(inp, t_end=13000)
fails += not ok(float(at(tr, 8800)["curL"]) > 0.3 and float(at(tr, 10500)["curL"]) == 0.0, f"servo a -45 por > fresh_ms: motores {at(tr,8800)['curL']} -> {at(tr,10500)['curL']}")

# 4) limite de re continua (max_rev_ms=1500)
tr, _ = run(base + drives(5000, 10000, -0.3), t_end=11000)
rev = [r for r in tr if float(r["curL"]) < -0.05]
dur = (rev[-1]["t"] - rev[0]["t"]) if rev else 0
fails += not ok(0 < dur <= 1700 and float(at(tr, 9500)["curL"]) == 0.0, f"re limitada: {dur} ms de re continua (max 1500), parada aos 9.5 s")

# 5) nao arma sozinho / desarma / BOOT / delay de 4 s
tr, _ = run([(100, '{"t":"mode","m":"phone"}')] + drives(1000, 8000, 0.4), t_end=9000)
fails += not ok(all(float(r["curL"]) == 0.0 for r in tr), "desarmado: ignora drive (motores sempre 0)")
tr, _ = run(base + drives(5000, 9000, 0.4) + [(7000, "stop")], t_end=10000)
fails += not ok(float(at(tr, 6900)["curL"]) > 0.3 and float(at(tr, 7500)["curL"]) == 0.0 and all(float(r["curL"]) == 0.0 for r in tr if r["t"] > 7500), "comando `stop` desarma e para")
tr, _ = run([(100, '{"t":"mode","m":"phone"}'), (300, "@btn_down"), (500, "@btn_up")] + drives(5000, 7000, 0.4) + [(8000, "@btn_down"), (8200, "@btn_up")], t_end=10000)
fails += not ok(at(tr, 4000)["armed"] == "0" and at(tr, 5000)["armed"] == "1" and at(tr, 9000)["armed"] == "0", "botao BOOT: arma so depois de 4 s e desarma")

# 6) cfg e comandos de texto
tr, o = run(base + [(4300, '{"t":"cfg","trim_left":0.8,"wd_ms":1000}')] + drives(5000, 8000, 0.5), t_end=9000)
fails += not ok(any('"cfg_ok"' in m for t, m in o) and abs(float(at(tr, 7000)["curL"]) - 0.4) < 0.02 and abs(float(at(tr, 7000)["curR"]) - 0.5) < 0.02,
                f"cfg trim_left=0.8: L={at(tr,7000)['curL']} R={at(tr,7000)['curR']} (esperado 0.40 / 0.50)")
tr, o = run([(100, "set trim_right 0.9"), (200, "get"), (300, "help"), (400, "set nao_existe 1")], t_end=1000)
txt = " | ".join(m for t, m in o)
fails += not ok("trim_right = 0.900" in txt and "parametro desconhecido: nao_existe" in txt and "d_stop_cm = 30.000" in txt
                and "turn_dps = 90.000" in txt and "vis_free_min = 50.000" in txt and "scan_deg = 30.000" in txt and "reverse_ms = 500.000" in txt,
                "comandos de texto: set/get/help (parametros novos presentes)")

# 6b) fiacao fisica (sim.cpp: ENA/IN1/IN2 = roda ESQUERDA, ENB/IN3/IN4 = roda DIREITA; o yaw sai dos PINOS)
tr, o = run(base + drives(5000, 7000, 0.3, 1.0), t_end=7000)
r = at(tr, 6500)
fails += not ok(float(r["uL"]) > float(r["uR"]) and float(r["rate"]) > 0 and int(r["enaPWM"]) > int(r["enbPWM"]),
                f"(fiacao) s=+1 (direita): roda ESQUERDA (ENA) mais rapida e o carro gira para a DIREITA "
                f"(uE={r['uL']} uD={r['uR']} giro={r['rate']} ENB={r['enbPWM']} ENA={r['enaPWM']})")
tr, o = run(base + drives(5000, 7000, 0.3, 0.0), t_end=7000)
r = at(tr, 6500)
fails += not ok(float(r["uL"]) > 0.25 and float(r["uR"]) > 0.25 and abs(float(r["rate"])) < 0.5
                and r["in1"] == "0" and r["in2"] == "1" and r["in3"] == "0" and r["in4"] == "1",
                f"(fiacao) v=0.3 s=0: as duas rodas PARA FRENTE (IN1/IN3=LOW, IN2/IN4=HIGH, sentido de giro inalterado), sem giro")
tr, o = run(base + [(4300, '{"t":"cfg","trim_left":0.5}')] + drives(5000, 7000, 0.4), t_end=7000)
r = at(tr, 6500)
fails += not ok(float(r["uL"]) < float(r["uR"]) and abs(float(r["uL"]) - 0.2) < 0.03,
                f"(fiacao) trim_left atua na roda ESQUERDA fisica (ENA): uE={r['uL']} uD={r['uR']}")

# 7) lixo na serial nao derruba
junk = [(1000, "{garbage"), (1100, "\x01\x02\x03"), (1200, "{" + "x" * 400), (1300, '{"t":"drive","v":abc}'), (1400, '{"t":"nada"}'), (1500, ""),
        (1600, '{"t":"vis","ok":1,"s":[1,2]}'), (1700, '{"t":"yaw","psi":"x"}')]
tr, o = run(base + junk + drives(5000, 7000, 0.4), t_end=8000)
fails += not ok(abs(float(at(tr, 6500)["curL"]) - 0.4) < 0.02, "lixo/linha longa/JSON invalido/vis incompleta ignorados; drive seguinte funciona")

# 8) falha 'far' (perna fina) nao e vista; sem eco sem historico = livre
tr, _ = run(base + drives(5000, 13000, 0.4), ["--obst", 9000, 15, "--far", 8000, 13000], t_end=13000)
fails += not ok(float(at(tr, 10500)["curL"]) > 0.3, "[limitacao conhecida] objeto invisivel ao HC-SR04 sem historico nao aciona o reflexo")

# ===================== modo AUTO =====================
tr, o = run(AUTO, ["--obst", 8000, 25, "--left", 200, "--right", 40], t_end=30000)
fails += not ok(abs(float(at(tr, 6500)["curL"]) - 0.35) < 0.02 and abs(float(at(tr, 6500)["curR"]) - 0.35) < 0.02, f"auto: FRENTE a v_cruise=0.35 -> L={at(tr,6500)['curL']} R={at(tr,6500)['curR']}")
d0 = decides(o)[0]
fails += not ok(d0["ang"] < 0 and d0["d"] == [-30, -15, 15, 30], f"auto: escolhe o MELHOR ANGULO a esquerda quando a esquerda e mais livre: {d0['ang']} notas {d0['sc']}")
fails += not ok(any(r["subn"] == "CURVA" and float(r["curL"]) < float(r["curR"]) for r in tr), "auto: gira para a ESQUERDA (roda esquerda mais lenta)")
tr, o = run(AUTO, ["--obst", 8000, 25, "--left", 40, "--right", 200], t_end=14000)
fails += not ok(decides(o)[0]["ang"] > 0 and any(r["subn"] == "CURVA" and float(r["curL"]) > float(r["curR"]) for r in tr), "auto: gira para a DIREITA quando a direita e mais livre")
tr, _ = run(AUTO, [], t_end=15000)
fails += not ok(all(r["cst"] == "0" for r in tr) and abs(float(at(tr, 14000)["curL"]) - 0.35) < 0.02, "auto: sem obstaculo fica em FRENTE")
tr, _ = run(AUTO, ["--obst", 8000, 25, "--clear", 9000], t_end=22000)
fails += not ok(at(tr, 21000)["cst"] in ("0", "1") and float(at(tr, 21000)["curL"]) > 0.1, "auto: obstaculo removido -> volta a andar")
tr, _ = run(AUTO, ["--obst", 8000, 50], t_end=12000)
fails += not ok(at(tr, 10000)["cst"] == "1" and abs(float(at(tr, 10000)["curL"]) - 0.15) < 0.02, f"auto: 50 cm -> VARREDURA a v_slow=0.15 ({at(tr,10000)['curL']})")
tr, _ = run(AUTO, ["--obst", 8000, 25, "--left", 25, "--right", 25, "--clear", 30000], t_end=45000)
fails += not ok(any(r["subn"] == "PRESO" for r in tr) and at(tr, 44000)["cst"] in ("0", "1"), "auto: sem saida -> PRESO -> libera quando o centro libera")

# ===== (a) parede em diagonal: valido 40 cm -> sem eco => NAO pode ir a v_cruise =====
tr, _ = run(AUTO, ["--obst", 6000, 40, "--none", 8000, 14000], t_end=12000)
sus = evs(why="sem eco suspeito")
fails += not ok(sus and 8000 <= sus[0]["t"] <= 8600, f"(a) valido 40 cm -> sem eco: classificado SUSPEITO e para ({[e['t'] for e in sus][:1]} ms)")
bad = [r for r in tr if 8150 <= r["t"] <= 9500 and r["cst"] == "0" and float(r["curL"]) > 0.3]
fails += not ok(not bad, "(a) nunca vai a v_cruise na janela do suspeito (suspect_ms = 1500)")
fails += not ok(not any(r["subn"] == "RE" for r in tr if 8000 <= r["t"] <= 11000), "(a) suspeito varre SEM re inicial")
tr, _ = run(AUTO, ["--none", 8000, 14000], t_end=12000)
fails += not ok(all(r["cst"] == "0" for r in tr) and abs(float(at(tr, 11000)["curL"]) - 0.35) < 0.02, "(a) controle: sem eco SEM historico recente = livre, anda normalmente")

# ===== (b) obstaculo parado a 25 cm: depois de ESPERA, faz manobra, nao loop =====
tr, o = run(AUTO, ["--obst", 6000, 25], t_end=45000)
seq = [e["subn"] for e in LAST["ev"] if e["subn"] != "-"]
n_wait, n_para = seq.count("ESPERA"), seq.count("PARADA")
after = [seq[i + 1] for i, s_ in enumerate(seq[:-1]) if s_ == "ESPERA"]
fails += not ok(n_wait >= 1 and all(a == "PARADA" for a in after), f"(b) apos ESPERA vem manobra completa (PARADA): {after}")
fails += not ok(n_wait == 1 and n_para >= 4, f"(b) nao ha loop de ESPERA: {n_wait} ESPERA e {n_para} manobras em 45 s")
tr, _ = run(AUTO, ["--obst", 6000, 25, "--clear", 7000], t_end=45000)
ra = [(r["avoidN"], r["guardLevel"]) for r in tr if r["t"] in (6900,)]
fails += not ok(int(at(tr, 6900)["avoidN"]) >= 1 and at(tr, 44000)["avoidN"] == "0" and at(tr, 44000)["guardLevel"] == "0",
                f"(b) avoidLog/guardLevel decaem: {at(tr,6900)['avoidN']}/{at(tr,6900)['guardLevel']} -> {at(tr,44000)['avoidN']}/{at(tr,44000)['guardLevel']}")

# ===== (c) lado sem eco + visao 'livre' vence lado com eco em 35 cm (e o contrario) =====
side = ["--obst", 6000, 25, "--ang", -15, 300, "--ang", -30, 300, "--ang", 15, 35, "--ang", 30, 35]
tr, o = run(base + drives(5000, 16000, 0.35) + vis(4500, 16000, [100, 100, 100, 100, 100]), side, t_end=16000)
d = decides(o)[0]
fails += not ok(d["ang"] < 0 and d["sc"][0] == 200 and d["sc"][2] == 35, f"(c) visao livre: lado SEM eco (nota {d['sc'][0]}) vence o lado com eco em 35 cm -> {d['ang']} graus")
tr, o = run(base + drives(5000, 16000, 0.35) + vis(4500, 16000, [0, 0, 100, 100, 100]), side, t_end=16000)
d = decides(o)[0]
fails += not ok(d["ang"] > 0 and d["sc"][0] == 15, f"(c) visao diz bloqueado a esquerda (nota {d['sc'][0]}): vence o eco de 35 cm da direita -> {d['ang']} graus")
tr, o = run(base + drives(5000, 16000, 0.35), side, t_end=16000)
d = decides(o)[0]
fails += not ok(d["ang"] < 0 and d["sc"][0] == 45, f"(c) sem visao: lado sem eco vale none_score_cm (45) > 35 -> {d['ang']} graus")

# ===== (d) vis velha e ignorada =====
allnone = ["--obst", 6000, 25, "--ang", -15, 300, "--ang", -30, 300, "--ang", 15, 300, "--ang", 30, 300]
tr, o = run(base + drives(5000, 16000, 0.35) + vis(5000, 5100, [0, 0, 0, 0, 0]), allnone, t_end=16000)
fails += not ok(decides(o)[0]["sc"] == [45, 45, 45, 45], f"(d) vis de 1 s atras (> vis_fresh_ms) e ignorada: notas {decides(o)[0]['sc']}")
tr, o = run(base + drives(5000, 16000, 0.35) + vis(4500, 16000, [0, 0, 0, 0, 0]), allnone, t_end=16000)
fails += not ok(decides(o)[0]["sc"] == [15, 15, 15, 15], f"(d) controle: vis fresca 'bloqueado' vale d_stop/2: notas {decides(o)[0]['sc']}")
tr, o = run(base + drives(5000, 16000, 0.35) + vis(4500, 16000, [100] * 5, ok=0), allnone, t_end=16000)
fails += not ok(decides(o)[0]["sc"] == [45, 45, 45, 45], "(d) vis com ok:0 = sem opiniao")
tr, o = run(AUTO + vis(4500, 16000, [0, 0, 0, 0, 0]), allnone, t_end=16000)
fails += not ok(decides(o)[0]["sc"] == [45, 45, 45, 45], "(d) em modo auto (fonte sensor) a visao e ignorada")

# ===== (e) curva ate o angulo com yaw; sem yaw usa turn_dps =====
def curva(args):
    tr, o = run(AUTO, ["--obst", 6000, 25, "--left", 300, "--right", 40, *args], t_end=9500)
    c = evs("CURVA")[0]; nxt = [e for e in LAST["ev"] if e["t"] > c["t"] and e["subn"] != "CURVA"][0]
    return nxt["t"] - c["t"], float(nxt["psi"]) - float(c["psi"])
dur0, dpsi0 = curva(["--yaw", "--world-dps", 60, "--set", "yaw_lead_ms", 0])
fails += not ok(14 <= abs(dpsi0) <= 19 and dur0 >= 220, f"(e) com yaw (lead 0): gira ate o alvo de 15 graus: {dpsi0:+.1f} graus em {dur0} ms (carro real: 60 graus/s)")
dur, dpsi = curva(["--yaw", "--world-dps", 60])
fails += not ok(8 <= abs(dpsi) <= 16, f"(e) com yaw e yaw_lead_ms=100: antecipa o fim em giro*lead (sem inercia no simulador): {dpsi:+.1f} graus")
dur2, dpsi2 = curva(["--world-dps", 60])
fails += not ok(abs(dur2 - 167) <= 45 and abs(dpsi2) < 12, f"(e) sem yaw: tempo = alvo/turn_dps = 167 ms: {dur2} ms ({dpsi2:+.1f} graus)")
dur3, dpsi3 = curva(["--yaw", "--world-dps", 0])
tout = 1.5 * 15 / 90 * 1000 + 300 + 25 / (90 * 0.35) * 1000       # + tempo da desaceleracao perto do alvo
fails += not ok(abs(dur3 - tout) <= 60, f"(e) com yaw mas o carro nao gira: timeout da curva ({dur3} ms, esperado {tout:.0f})")
dur4, dpsi4 = curva(["--yaw", "--world-dps", 90, "--set", "turn_dps", 30])
fails += not ok(10 <= abs(dpsi4) <= 20 and dur4 < 1000, f"(e) com yaw, turn_dps errado nao importa: {dpsi4:+.1f} graus em {dur4} ms")

# ===== (f) reset completo =====
def goal_line(g, src=None):
    o = {"t": "goal", "g": g}
    if src: o["src"] = src
    return json.dumps(o, separators=(",", ":"))
for name, off in (("goal parar", (8000, goal_line("parar"))), ("arm false", (8000, '{"t":"arm","on":false}')), ("BOOT", (8000, "@btn_down"))):
    tr, o = run([(300, goal_line("explorar", "visao")), off, (8200, "@btn_up")] + drives(4500, 12000, 0.3) + vis(4500, 7000, [100] * 5),
                ["--obst", 7000, 25], t_end=12000)
    a, b = at(tr, 7600), at(tr, 9000)
    fails += not ok(a["armed"] == "1" and a["mode"] == "1" and a["rtcArmed"] == "1" and a["rtcMode"] == "1" and a["rtcGoal"] == "explorar" and a["netSrc"] == "visao",
                    f"(f) {name}: antes do reset: armado, modo phone, RTC {a['rtcArmed']}/{a['rtcMode']}/{a['rtcGoal']}")
    fails += not ok(b["armed"] == "0" and b["mode"] == "0" and b["delegated"] == "0" and b["cst"] == "0" and b["subn"] == "-" and b["curL"] == "0.000"
                    and b["rtcArmed"] == "0" and b["rtcMode"] == "0" and b["rtcGoal"] == "-" and b["netGoal"] == "parar" and b["avoidN"] == "0",
                    f"(f) {name}: depois: desarmado, modo padrao, FSM limpa, rtcArmed/rtcMode/rtcGoal limpos")
tr, o = run([(300, goal_line("explorar", "sensor"))], t_end=6000)
fails += not ok(at(tr, 5000)["armed"] == "1" and at(tr, 5000)["mode"] == "0" and at(tr, 3000)["armed"] == "0", "(f) goal explorar/sensor arma sozinho (4 s) em modo auto")
tr, o = run([(300, goal_line("explorar", "visao"))], t_end=6000)
fails += not ok(at(tr, 5000)["armed"] == "1" and at(tr, 5000)["mode"] == "1", "(f) goal explorar/visao arma sozinho em modo phone")
tr, o = run([(300, goal_line("explorar", "visao")), (500, '{"t":"mode","m":"phone"}'), (1000, '{"t":"mode","m":"phone"}')], t_end=6000)
fails += not ok(at(tr, 5000)["armed"] == "1" and at(tr, 5000)["mode"] == "1", "(f) mode repetido e idempotente (nao reinicia)")

# ===== (g) modo phone veta a 30 cm pela mediana =====
tr, _ = run(base + drives(5000, 13000, 0.35), ["--obst", 9000, 25], t_end=13000)
fails += not ok(at(tr, 8800)["delegated"] == "0" and float(at(tr, 8800)["curL"]) > 0.3 and at(tr, 9600)["delegated"] == "1",
                "(g) phone: obstaculo a 25 cm (< d_stop 30) -> o ESP32 assume o desvio sozinho (veto)")
tr, _ = run(base + drives(5000, 13000, 0.35), ["--obst", 9000, 35], t_end=13000)
fails += not ok(all(r["delegated"] == "0" for r in tr if r["t"] > 5200) and float(at(tr, 11000)["curL"]) > 0.3, "(g) controle: a 35 cm (> d_stop) o celular segue no comando")
tr, _ = run(base + [(t, m) for t, m in drives(5000, 13000, 0.0)], ["--obst", 9000, 25], t_end=13000)
fails += not ok(all(r["delegated"] == "0" for r in tr if r["t"] > 5200), "(g) phone parado (v=0) nao dispara veto")

# ===== (h) avoid com angulo sugerido pelo celular =====
hint = ["--ang", 30, 300, "--ang", 15, 300, "--ang", -15, 300, "--ang", -30, 300]
tr, o = run(base + drives(5000, 12000, 0.35) + vis(4500, 12000, [100] * 5) + [(6000, '{"t":"avoid","deg":30}')], hint, t_end=12000)
d = decides(o)[0]
fails += not ok(d["ang"] == 30 and d["sc"] == [-1, -1, -1, 200], f"(h) sugestao +30 com visao livre: aceita de cara (sem varrer o resto): {d['sc']}")
tr, o = run(base + drives(5000, 12000, 0.35) + vis(4500, 12000, [100] * 5) + [(6000, '{"t":"avoid","deg":30}')],
            ["--ang", 30, 20, "--ang", 15, 300, "--ang", -15, 300, "--ang", -30, 300], t_end=12000)
d = decides(o)[0]
fails += not ok(d["ang"] != 30 and d["sc"][3] == 20 and all(x > 0 for x in d["sc"]), f"(h) sensor veta a sugestao (eco 20 cm < d_stop): varre tudo e escolhe outro: {d['ang']} {d['sc']}")
aims = [json.loads(m)["deg"] for t, m in o if m.startswith('{"t":"scan"') and '"k":"aim"' in m]
fails += not ok(aims[0] == 30.0, f"(h) a varredura comeca pelo angulo sugerido: {aims[:3]}")

# ===== (i) sem eco com visao: suspeito/livre =====
tr, _ = run(base + drives(5000, 13000, 0.35) + vis(4500, 13000, [100, 100, 0, 100, 100]), ["--obst", 6000, 50, "--none", 8000, 13000], t_end=13000)
fails += not ok(evs(why="veto: sem eco suspeito") and 8000 <= evs(why="veto: sem eco suspeito")[0]["t"] <= 8800, "(i) phone: sem eco + visao fresca com setor 0 bloqueado -> SUSPEITO (veto)")
tr, _ = run(base + drives(5000, 13000, 0.35) + vis(4500, 13000, [100] * 5), ["--obst", 6000, 50, "--none", 8000, 13000], t_end=13000)
fails += not ok(not evs(why="veto") and float(at(tr, 9500)["curL"]) > 0.3, "(i) phone: sem eco + visao fresca livre -> LIVRE (nao veta, mesmo logo apos leitura valida)")
tr, _ = run(base + drives(5000, 13000, 0.35), ["--obst", 6000, 50, "--none", 8000, 13000], t_end=13000)
fails += not ok(evs(why="veto: sem eco suspeito"), "(i) phone: sem visao, sem eco logo apos leitura valida -> SUSPEITO (veto pelo historico)")

# ===== (j) diario do servo reenviado a TODO cliente novo =====
tr, o = run(AUTO + [(16000, "@ws_connect"), (16100, "@ws_connect")], ["--obst", 6000, 25], t_end=17000)
by = {}
for t, n, txt in LAST["ws"]:
    by.setdefault(n, []).append(txt)
fails += not ok(len(by) == 2 and all(any('"k":"decide"' in x and '"replay":1' in x for x in v) for v in by.values()),
                f"(j) os dois clientes novos recebem o diario do servo (replay): {[len(v) for v in by.values()]} mensagens")
fails += not ok(all(json.loads(x) for v in by.values() for x in v), "(j) o diario reenviado e JSON valido")

# ===== (k) toda linha emitida e JSON valido e cabe no buffer =====
tr, o = run(AUTO, ["--obst", 6000, 25, "--left", 300, "--right", 40, "--yaw"], t_end=20000)
badj = []
for t, m in o:
    if m.startswith("{"):
        try: json.loads(m)
        except ValueError: badj.append(m)
fails += not ok(not badj and max(len(m) for t, m in o) < 384, f"(k) {len(o)} linhas emitidas, todas JSON valido (maior: {max(len(m) for t, m in o)} bytes)")

# ===== (l) rumo em reta no ESP32: rodas desiguais (direita com 65% de eficiencia e zona morta maior) =====
WHEELS = ["--yaw", "--world-dps", 200, "--yaw-lag", 60, "--tau", 50, "--wheel-left", 1.0, 100, "--wheel-right", 0.65, 105]
NOHOLD = ["--set", "hold_kp", 0, "--set", "hold_ki", 0, "--set", "trim_rate", 0]
seg_inp, segs, t = list(base), [], 5000
for _ in range(6):                                             # 6 trechos: 2 s reto (s = 0), 1 s parado
    seg_inp += drives(t, t + 2000, 0.35) + drives(t + 2000, t + 3000, 0.0); segs.append(t); t += 3000
def seg_stats(tr, s0):
    win = [float(r["rate"]) for r in tr if s0 + 1000 <= r["t"] < s0 + 2000]
    return sum(win) / len(win), float(at(tr, s0 + 1990)["trim"]), float(at(tr, s0 + 1500)["hc"])
tr, o = run(seg_inp, WHEELS + NOHOLD, t_end=t)
m0 = seg_stats(tr, segs[0])[0]
fails += not ok(m0 > 100, f"(l) controle: sem correcao, com L = R o carro gira {m0:+.0f} graus/s (como no log de 20/11)")
tr, o = run(seg_inp, WHEELS, t_end=t)
st = [seg_stats(tr, s0) for s0 in segs]
fails += not ok(all(abs(m) < 10 for m, _, _ in st[2:]), f"(l) com correcao: do 3o trecho em diante, giro medio de 1 a 2 s < 10 graus/s: {[round(m, 1) for m, _, _ in st]}")
fails += not ok(abs(st[0][0]) < 60 and abs(st[1][0]) < 25,
                f"(l) carro POUCO sensivel e trim 0 (ganhos baixos para nao oscilar no carro real): 135 -> {st[0][0]:+.1f} (1o trecho) -> {st[1][0]:+.1f} (2o)")
trims = [tm for _, tm, _ in st]
fails += not ok(trims[0] < -0.4 and abs(trims[-1] - trims[-2]) < 0.03 and abs(trims[-2] - trims[-3]) < 0.06 and abs(st[-1][2] - trims[-1]) < 0.15,
                f"(l) o trim aprendido persiste entre trechos e converge: {[round(x, 3) for x in trims]} (correcao final {st[-1][2]:+.2f})")
r = at(tr, segs[-1] + 1500)
fails += not ok(float(r["curR"]) > 1.0 and abs(float(r["curL"]) - 0.35) < 0.01,
                f"(l) sobe a roda LENTA (direita: {r['curR']}) e mantem a rapida em v_cruise (esquerda: {r['curL']})")
fails += not ok(r["hold"] == "2", f"(l) state.hold = 2 (malha fechada com yaw fresco): {r['hold']}")
# NVS: grava ao desarmar; carrega no boot; sem yaw aplica so o trim (malha aberta)
tr, o = run(seg_inp + [(t + 100, "stop")], WHEELS, t_end=t + 300)
nvs = [l for _, l in o if "trim do rumo salvo" in l]
fails += not ok(nvs and abs(float(nvs[-1].split(": ")[1].rstrip('"}')) - trims[-1]) < 0.05, f"(l) trim salvo na NVS ao desarmar: {nvs[-1:] }")
OPEN = ["--world-dps", 200, "--wheel-left", 1.0, 100, "--wheel-right", 0.65, 105]
tr, o = run(base + drives(5000, 8000, 0.35), OPEN + ["--nvs", "trim", -1.21], t_end=8000)
w = [float(r["rate"]) for r in tr if 6000 <= r["t"] < 8000]
fails += not ok(abs(sum(w) / len(w)) < 15 and at(tr, 7000)["hold"] == "1" and abs(float(at(tr, 7000)["trim"]) + 1.21) < 0.01,
                f"(l) boot com trim -1,21 na NVS e SEM yaw: so o trim (hold=1), giro medio {sum(w)/len(w):+.1f} graus/s")
tr, o = run(base + drives(5000, 8000, 0.35), OPEN, t_end=8000)
w = [float(r["rate"]) for r in tr if 6000 <= r["t"] < 8000]
fails += not ok(sum(w) / len(w) > 100, f"(l) controle: sem trim e sem yaw o carro gira ({sum(w)/len(w):+.0f} graus/s)")
tr, o = run(AUTO, WHEELS, t_end=8000)
w = [float(r["rate"]) for r in tr if 6500 <= r["t"] < 8000]
fails += not ok(at(tr, 7000)["cst"] == "0" and abs(sum(w) / len(w)) < 30,
                f"(l) modo auto (fonte sensor): FRENTE tambem corrige (trim 0: {sum(w)/len(w):+.1f} graus/s; sem correcao 135)")
tr, o = run(base + drives(5000, 8000, 0.3, 0.5), WHEELS, t_end=8000)
fails += not ok(at(tr, 7000)["hold"] == "0" and float(at(tr, 7000)["hc"]) == 0, "(l) com s fora de +-hold_s_eps (curva pedida) nao corrige")
tr, o = run([(100, "trim 0.4"), (200, "trim")], [], t_end=500)
fails += not ok(any("trim = 0.400" in m for _, m in o), "(l) comando de texto `trim <valor>`")

# ===== (m) diagnostico pelo visor =====
def diags(o): return [json.loads(m) for t, m in o if m.startswith('{"t":"diag"')]
def dline(k): return json.dumps({"t": "diag", "teste": k}, separators=(",", ":"))
tr, o = run([(1000, dline("dir"))], ["--yaw"], t_end=2000)
fails += not ok(diags(o) and "erro" in diags(o)[0] and all(float(r["curL"]) == 0 for r in tr), f"(m) desarmado: recusa o teste ({diags(o)[:1]})")
ARMD = [(300, dline("armar"))]                                  # arma aos 4,3 s e fica parado (modo auto, caminho livre)
tr, o = run(ARMD, ["--yaw"], t_end=7000)
sts = [json.loads(m) for t, m in o if m.startswith('{"t":"state"')]
fails += not ok(at(tr, 4400)["armed"] == "1" and all(float(r["curL"]) == 0 for r in tr) and sts[-1]["st"] == "DIAG" and sts[-1]["sub"] == "parado",
                f"(m) 'Armar parado': arma e NAO anda (o modo auto andaria): st={sts[-1]['st']}/{sts[-1]['sub']}")
tr, o = run(ARMD + [(5000, dline("dir")), (7000, dline("esq"))], ["--yaw"], t_end=9000)
dd = diags(o)
d1, d2 = (dd + [{}, {}])[:2]
fails += not ok(d1.get("teste") == "dir" and d1["psi"] > 30 and d1["rate_med"] > 30 and d1["L"] > 0.5 and d1["R"] < -0.5 and 950 <= d1["ms"] <= 1100,
                f"(m) Girar a direita 1 s (s=+1, v=0): psi {d1.get('psi')} graus, giro mediano {d1.get('rate_med')}, L {d1.get('L')} R {d1.get('R')}")
fails += not ok(d2.get("teste") == "esq" and d2["psi"] < -30 and d2["L"] < -0.5 and d2["R"] > 0.5, f"(m) Girar a esquerda: psi {d2.get('psi')}")
fails += not ok(all(abs(float(r["curL"])) < 0.02 for r in tr if 6100 <= r["t"] <= 6900 or r["t"] >= 8200), "(m) depois de cada teste o carro fica parado")
UNEQ = WHEELS + ["--nvs", "trim", -1.22]
tr, o = run(ARMD + [(5000, dline("reto")), (8000, dline("reto_corr"))], UNEQ, t_end=11000)
dd = {d["teste"]: d for d in diags(o)}
a, b = dd.get("reto", {}), dd.get("reto_corr", {})
fails += not ok(a.get("rate_med", 0) > 100 and abs(a["L"] - a["R"]) < 0.01 and b.get("rate_med") is not None and abs(b["rate_med"]) < 10,
                f"(m) Reto 2 s SEM correcao (nem trim): giro {a.get('rate_med')} graus/s, L {a.get('L')} R {a.get('R')}; "
                f"COM correcao: {b.get('rate_med')} graus/s (L {b.get('L')} R {b.get('R')})")
tr, o = run(ARMD + [(5000, dline("dir"))], [], t_end=7000)
fails += not ok(diags(o) and diags(o)[0]["psi"] is None and diags(o)[0]["rate_med"] is None, "(m) sem yaw do celular: psi/rate_med = null")
tr, o = run(ARMD + [(5000, dline("reto"))], ["--yaw", "--obst", 5500, 15], t_end=8000)
fails += not ok(diags(o) and "abortado" in diags(o)[0] and all(float(r["curL"]) < 0.02 for r in tr if r["t"] > 6200),
                f"(m) hard stop vale no diagnostico (obstaculo a 15 cm): {diags(o)[:1]}")
tr, o = run(ARMD + [(5000, dline("dir")), (7000, goal_line("explorar", "sensor"))], ["--yaw"], t_end=9000)
fails += not ok(abs(float(at(tr, 8500)["curL"]) - 0.35) < 0.05, f"(m) goal explorar depois do diagnostico: volta a andar ({at(tr, 8500)['curL']})")
tr, o = run([(300, goal_line("explorar", "sensor")), (6000, dline("dir"))], ["--yaw"], t_end=9000)
fails += not ok(float(at(tr, 5500)["curL"]) > 0.3 and diags(o) and diags(o)[0]["psi"] > 30 and abs(float(at(tr, 8000)["curL"])) < 0.02,
                "(m) teste pedido explorando: interrompe a FSM, gira e fica parado")
fails += not ok(all(json.loads(m) for t, m in o if m.startswith("{")), "(m) linhas emitidas sao JSON valido")
# teste POR RODA: duty bruto em degraus DESCENDO (220..45, 600 ms, o 1o com 1200 ms), uma roda so; roda direita com
# zona morta maior (como no carro de 21/11)
WTEST = ["--yaw", "--world-dps", 40, "--wheel-left", 1.0, 120, "--wheel-right", 1.0, 141]
WDUTY = [220, 195, 170, 145, 120, 95, 70, 45]
tr, o = run(ARMD + [(5000, dline("roda_esq")), (11000, dline("roda_dir"))], WTEST, t_end=17500)
dd = {d["teste"]: d for d in diags(o)}
e, d_ = dd.get("roda_esq", {}), dd.get("roda_dir", {})
fails += not ok(e.get("duty") == WDUTY and 5350 <= e.get("ms", 0) <= 5500 and len(e.get("dr", [])) == 8,
                f"(m) Roda esq.: 8 degraus de duty 220..45 DESCENDO em ~5,4 s (dr por degrau): {e.get('duty')} ({e.get('ms')} ms)")
rs = [r for r in e.get("rate", []) if r is not None]
fails += not ok(len(rs) == 8 and all(r >= -0.5 for r in rs) and rs[0] > 100 and e.get("minimo") == 145,
                f"(m) Roda esq.: carro gira para a DIREITA (sobre a roda parada), caindo com o duty; minimo {e.get('minimo')} (esperado 145): {e.get('rate')}")
rs = [r for r in d_.get("rate", []) if r is not None]
fails += not ok(len(rs) == 8 and all(r <= 0.5 for r in rs) and rs[0] < -100 and d_.get("minimo") == 170,
                f"(m) Roda dir.: gira para a ESQUERDA; zona morta maior => minimo {d_.get('minimo')} (esperado 170): {d_.get('rate')}")
w_e = [r for r in tr if 5400 <= r["t"] <= 10200]
w_d = [r for r in tr if 11400 <= r["t"] <= 16200]
fails += not ok(all(int(r["enbPWM"]) == 0 and int(r["enaPWM"]) >= 45 and r["in1"] == "0" and r["in2"] == "1" for r in w_e)
                and all(int(r["enaPWM"]) == 0 and int(r["enbPWM"]) >= 45 and r["in3"] == "0" and r["in4"] == "1" for r in w_d),
                "(m) Roda esq. aciona SO o ENA (para frente), Roda dir. SO o ENB; o sentido de cada roda e o de sempre")
fails += not ok(all(abs(float(r["curL"])) < 0.02 and int(r["enaPWM"]) == 0 and int(r["enbPWM"]) == 0 for r in tr if 10600 <= r["t"] <= 10900 or r["t"] >= 16700),
                "(m) depois do teste por roda o carro fica parado (duty 0)")
tr, o = run(ARMD + [(5000, '{"t":"diag","teste":"roda_dir","d0":130,"d1":160,"dd":10,"step_ms":200}')], WTEST, t_end=7000)
d_ = (diags(o) or [{}])[0]
fails += not ok(d_.get("duty") == [160, 150, 140, 130] and 950 <= d_.get("ms", 0) <= 1100, f"(m) degraus configuraveis (d0/d1/dd/step_ms): {d_.get('duty')} {d_.get('ms')} ms")
tr, o = run(ARMD + [(5000, dline("roda_esq"))], WTEST + ["--obst", 6000, 15], t_end=8000)
d_ = (diags(o) or [{}])[0]
fails += not ok("abortado" in d_ and all(int(r["enaPWM"]) == 0 for r in tr if r["t"] > 6300), f"(m) hard stop vale no teste por roda: {d_.get('abortado')}")

# faixa de duty POR RODA (pwm_min_l/r, pwm_max_l/r): mesma conta do visor (web_ui.h: settled/pwmIgnored/fitW/suggest)
def settled(r, d): return d is None or abs(d) <= max(20, 0.15 * abs(r))
def pwm_ignored(m):
    v = sorted((d, abs(r)) for d, r in zip(m["duty"], m["rate"]) if r is not None)
    if len(v) < 4: return False
    mx = max(r for _, r in v)
    return mx >= 60 and v[0][1] >= 0.7 * mx and v[-1][0] - v[0][0] >= 100
def _ls(q):
    n = len(q); sx = sum(x for x, _ in q); sy = sum(y for _, y in q)
    sxx = sum(x * x for x, _ in q); sxy = sum(x * y for x, y in q)
    k = (n * sxy - sx * sy) / (n * sxx - sx * sx)
    return k, (sx - sy / k) / n
def fit_w(m):
    if pwm_ignored(m): return None
    drs = m.get("dr") or [None] * len(m["duty"])
    p = sorted((d, abs(r)) for d, r, dr_ in zip(m["duty"], m["rate"], drs) if r is not None and abs(r) >= 15 and settled(r, dr_))
    while len(p) > 3:                                    # teto: degraus do alto em que o giro quase nao sobe mais
        k, _ = _ls(p[:-1]); (xa, ya), (xb, yb) = p[-2], p[-1]
        if (yb - ya) / (xb - xa) < 0.5 * k: p.pop()
        else: break
    if len(p) < 2: return None
    k, d0 = _ls(p)
    if not k > 0: return None
    return {"k": k, "d0": d0, "min": m["minimo"] if m.get("minimo") is not None else p[0][0], "vmax": p[-1][1]}
def suggest(a, b, wm=5, wspan=25):
    L, R = fit_w(a), fit_w(b)
    if not L or not R: return None
    ref = L if L["d0"] >= R["d0"] else R
    vlo = max([ref["k"] * wm] + [w["k"] * (w["min"] - w["d0"]) for w in (L, R)])
    vhi = vlo + ref["k"] * wspan
    rnd = lambda x: max(0, min(255, int(math.floor(x + 0.5))))   # Math.round do JS
    return {"pwm_min_l": rnd(L["d0"] + vlo / L["k"]), "pwm_max_l": rnd(L["d0"] + vhi / L["k"]),
            "pwm_min_r": rnd(R["d0"] + vlo / R["k"]), "pwm_max_r": rnd(R["d0"] + vhi / R["k"])}
# log de 21/11 (0.5.1, degraus SUBINDO de 300 ms): a esquerda trava em ~395 graus/s de 170 em diante. Com esses pontos
# a reta antiga dava 2,8 graus/s por ponto (igual a direita) e a faixa 64..89 / 106..131 que nao endireitou o carro.
F21E = {"duty": list(range(110, 231, 15)), "rate": [59.3, 142.9, 227.7, 352.3, 396.9, 391.5, 395.2, 399.5, 393.3], "minimo": None}
F21D = {"duty": list(range(110, 231, 15)), "rate": [-29.2, -56.2, -100.2, -147.7, -195.5, -244.5, -282.6, -309.1, -348], "minimo": None}
fe, fd = fit_w(F21E), fit_w(F21D)
fails += not ok(fe and fe["k"] > 5 and 95 < fe["d0"] < 110 and fe["vmax"] < 360 and fd and 2.3 < fd["k"] < 3.2,
                f"(p) log 21/11: o teto da esquerda (~395 de 170 a 230) sai da reta: esquerda {fe and round(fe['k'], 1)} graus/s por ponto "
                f"(parte ~{fe and round(fe['d0'])}), direita {fd and round(fd['k'], 1)} (antes: 2,8 nas duas)")
tr, o = run(ARMD + [(5000, dline("roda_esq"))], WTEST + ["--nopwm-left", "--tau", 150], t_end=11000)
e = (diags(o) or [{}])[0]
fails += not ok(e.get("rate") and pwm_ignored(e) and fit_w(e) is None and not pwm_ignored(F21E),
                f"(p) roda que ignora o PWM (gira igual de 220 a 45) e detectada e fica sem faixa: {e.get('rate')}")
tr, o = run(ARMD + [(5000, dline("roda_esq"))], WTEST + ["--tau", 150], t_end=11000)
e = (diags(o) or [{}])[0]
fails += not ok(e.get("rate") and all(settled(r, d) for r, d in zip(e["rate"], e["dr"]) if r is not None) and not pwm_ignored(e),
                f"(p) com inercia (tau 150 ms) os degraus de 600 ms descendo assentam: rate {e.get('rate')} dr {e.get('dr')}")
UNEQ2 = ["--yaw", "--world-dps", 40, "--wheel-left", 1.0, 120, "--wheel-right", 0.7, 141]
tr, o = run(ARMD + [(5000, dline("roda_esq")), (11000, dline("roda_dir"))], UNEQ2, t_end=17500)
dd = {d["teste"]: d for d in diags(o)}
sug = suggest(dd["roda_esq"], dd["roda_dir"])
fails += not ok(sug["pwm_min_r"] > sug["pwm_min_l"] and (sug["pwm_max_r"] - sug["pwm_min_r"]) > (sug["pwm_max_l"] - sug["pwm_min_l"]),
                f"(p) teste por roda -> faixa sugerida: a direita (parte mais tarde e rende 70%) ganha duty maior: {sug}")
cfg = json.dumps(dict(t="cfg", **sug), separators=(",", ":"))
NOCORR = ["--set", "hold_kp", 0, "--set", "hold_ki", 0, "--set", "trim_rate", 0]
tr, o = run(base + drives(5000, 8000, 0.35), UNEQ2 + NOCORR, t_end=8000)
w0 = [float(r["rate"]) for r in tr if 6000 <= r["t"] < 8000]
tr, o = run(base + [(4500, cfg)] + drives(5000, 8000, 0.35), UNEQ2 + NOCORR, t_end=8000)
w1 = [float(r["rate"]) for r in tr if 6000 <= r["t"] < 8000]
m0, m1 = sum(w0) / len(w0), sum(w1) / len(w1)
fails += not ok(abs(m0) > 20 and abs(m1) < 5 and len(cfg) <= 160,
                f"(p) reta SEM correcao: {m0:+.1f} graus/s com a faixa unica -> {m1:+.1f} graus/s com a faixa por roda (cfg {len(cfg)} bytes)")
nv = [l for _, l in o if l.startswith('{"t":"log"') and "rodas (NVS)" in l]
fails += not ok(nv, f"(p) cfg com a faixa por roda grava na NVS: {nv[:1]}")
tr, o = run(base + drives(5000, 8000, 0.35), UNEQ2 + NOCORR + ["--nvs", "pmin_l", sug["pwm_min_l"], "--nvs", "pmax_l", sug["pwm_max_l"],
            "--nvs", "pmin_r", sug["pwm_min_r"], "--nvs", "pmax_r", sug["pwm_max_r"]], t_end=8000)
w2 = [float(r["rate"]) for r in tr if 6000 <= r["t"] < 8000]
fails += not ok(abs(sum(w2) / len(w2)) < 5, f"(p) boot le a faixa por roda da NVS: {sum(w2)/len(w2):+.1f} graus/s sem correcao")
tr, o = run(base + drives(5000, 7000, 0.35), [], t_end=7000)
r = at(tr, 6500)
fails += not ok(int(r["enaPWM"]) == int(r["enbPWM"]) == 137, f"(p) sem faixa por roda (-1) vale pwm_min/pwm_max: duty {r['enaPWM']}/{r['enbPWM']} (130 + 0,35*20)")
tr, o = run(ARMD + [(5000, '{"t":"trim","v":0.7}'), (5200, dline("zerar_trim"))], [], t_end=6000)
fails += not ok(any('"zerar_trim"' in m for _, m in o) and float(at(tr, 5800)["trim"]) == 0, "(m) Zerar trim")

# ===== (n) sensor refuta a visao; sem ciclo avoid -> ESPERA -> devolve -> avoid =====
# "brain" simulado na fonte visao com OBSTACULO: drive v = 0 e avoid a cada 2 s; visao com o centro bloqueado
def brain_obst(t0, t1, v=0.0):
    return (drives(t0, t1, v) + vis(t0, t1, [100, 100, 0, 100, 100])
            + [(t, '{"t":"avoid","deg":15}') for t in range(t0 + 500, t1, 2000)])
tr, o = run(base + brain_obst(5000, 45000, 0.15), ["--ang", 0, 86], t_end=45000)
refs = [m for _, m in o if "avoid recusado" in m]
fails += not ok(not evs("PARADA") and not evs("ESPERA") and all(r["delegated"] == "0" for r in tr if r["t"] > 5000) and refs,
                f"(n) visao OBSTACULO + 86 cm VALIDOS: nenhum avoid aceito, nenhuma manobra; log: {refs[:1]}")
fails += not ok(float(at(tr, 30000)["curL"]) > 0.1, f"(n) refutada: segue andando devagar (v_slow do celular): {at(tr, 30000)['curL']}")
tr, o = run(base + brain_obst(5000, 45000), ["--ang", 0, 45], t_end=45000)
seq = [e["subn"] for e in LAST["ev"] if e["subn"] != "-"]
after = [seq[i + 1] for i, s_ in enumerate(seq[:-1]) if s_ == "ESPERA"]
fails += not ok(seq.count("ESPERA") <= 1 and all(a == "PARADA" for a in after) and seq.count("PARADA") >= 6,
                f"(n) zona intermediaria (45 cm) + visao OBSTACULO persistente: {seq.count('ESPERA')} ESPERA e "
                f"{seq.count('PARADA')} manobras em 40 s; depois da ESPERA: {after}")
tr, o = run(base + brain_obst(5000, 20000), ["--ang", 0, 86, "--obst", 9000, 50], t_end=20000)
fails += not ok(evs("PARADA") and min(e["t"] for e in evs("PARADA")) >= 9000,
                "(n) a distancia cai para a zona intermediaria (50 cm): o avoid volta a valer (fluxo normal)")

# ===== (o) curva ate o angulo sem passar do alvo (giro rapido, inercia e yaw atrasado) =====
def turns(o): return [json.loads(m) for t, m in o if m.startswith('{"t":"turn"')]
FAST = ["--yaw", "--world-dps", 150, "--tau", 80, "--yaw-lag", 60]
TGT = {-30: ["--left", 40, "--ang", -30, 300, "--right", 40], 30: ["--right", 40, "--ang", 30, 300, "--left", 40],
       -15: ["--left", 300, "--right", 40]}
res, old = {}, {}
for tg, w in TGT.items():
    tr, o = run(AUTO, ["--obst", 6000, 25, *w, *FAST], t_end=11000)
    res[tg] = (turns(o) or [{}])[0]
    tr, o = run(AUTO, ["--obst", 6000, 25, *w, *FAST, "--set", "turn_k_min", 1], t_end=11000)
    old[tg] = (turns(o) or [{}])[0]
fails += not ok(all(res[tg].get("alvo") == tg and res[tg].get("erro") is not None and abs(res[tg]["erro"]) < 5 for tg in TGT),
                "(o) curva com erro final < 5 graus (giro 150 graus/s, inercia 80 ms, yaw 60 ms atrasado): "
                + ", ".join(f"alvo {tg}: girou {res[tg].get('girou')} erro {res[tg].get('erro')}" for tg in TGT))
fails += not ok(any(abs(old[tg].get("erro") or 0) > 5 for tg in TGT),
                "(o) controle: sem desacelerar (turn_k_min = 1) passa do alvo: "
                + ", ".join(f"{tg}: erro {old[tg].get('erro')}" for tg in TGT))
fails += not ok(all(set(res[tg]) >= {"t", "alvo", "girou", "erro", "ms", "yaw"} and res[tg]["yaw"] == 1 for tg in TGT), "(o) {\"t\":\"turn\"} com alvo, girou, erro, ms, yaw")
tr, o = run(AUTO, ["--obst", 6000, 25, *TGT[-30], "--world-dps", 150], t_end=11000)
tt = turns(o)
fails += not ok(tt and tt[0]["girou"] is None and tt[0]["yaw"] == 0, f"(o) sem yaw: curva pelo tempo, girou = null: {tt[:1]}")

# ===== (q) carro do diagnostico de 21/11: zona morta esq 128 / dir 140, reto a duty 137 = +175 graus/s, dir = ~310 =====
CAR21 = ["--yaw", "--world-dps", 467, "--yaw-lag", 60, "--tau", 50, "--wheel-left", 1.0, 128, "--wheel-right", 1.0, 140]
tr, o = run(ARMD + [(5000, dline("reto")), (8000, dline("dir"))], CAR21, t_end=11000)
dd = {d["teste"]: d for d in diags(o)}
fails += not ok(150 < dd["reto"]["rate_med"] < 200 and 270 < dd["dir"]["rate_med"] < 350,
                f"(q) modelo reproduz o log de 21/11 (com a fiacao certa): reto {dd['reto']['rate_med']} graus/s, dir {dd['dir']['rate_med']}")
def reto_corr_series(extra, n=4, pre=()):
    inp, t = ARMD + list(pre), 5000
    for _ in range(n):
        inp.append((t, dline("reto_corr"))); t += 3000
    tr, o = run(inp, CAR21 + extra, t_end=t)
    res = []
    for i in range(n):
        w = [float(r["rate"]) for r in tr if 5000 + 3000 * i + 1000 <= r["t"] < 5000 + 3000 * i + 2000]
        m = sum(w) / len(w)
        res.append((round(m, 1), round((sum((x - m) ** 2 for x in w) / len(w)) ** .5, 1)))
    return res
old = reto_corr_series(["--set", "hold_kp", 0.01, "--set", "hold_ki", 0.05])
fails += not ok(max(sd for _, sd in old) > 100, f"(q) controle: com os ganhos antigos (0,01/0,05) a correcao OSCILA: (media, desvio) {old}")
new = reto_corr_series([])
fails += not ok(all(sd < 15 for _, sd in new) and abs(new[0][0]) < 15 and all(abs(m) < 5 for m, _ in new[2:]),
                f"(q) ganhos novos (0,0015/0,015, PWM 10 bits): estavel, 1o reto ja < 15 graus/s e < 5 a partir do 3o: {new}")
tr, o = run(ARMD + [(5000, dline("roda_esq")), (11000, dline("roda_dir"))], CAR21, t_end=17500)
dd = {d["teste"]: d for d in diags(o)}
sug = suggest(dd["roda_esq"], dd["roda_dir"])
cfg = json.dumps(dict(t="cfg", **sug), separators=(",", ":"))
raw = reto_corr_series(["--set", "hold_kp", 0, "--set", "hold_ki", 0, "--set", "trim_rate", 0], n=1, pre=[(4500, cfg)])
both = reto_corr_series([], n=2, pre=[(4500, cfg)])
fails += not ok(abs(raw[0][0]) < 10 and all(abs(m) < 5 and sd < 10 for m, sd in both),
                f"(q) com a faixa por roda sugerida {sug}: reto SEM correcao {raw[0][0]:+.1f} graus/s; com correcao {both}")

# ===== Python <-> firmware =====
from control.hal import Drive, LinkDrive, LinkRangeHead, MemoryLink
link = MemoryLink(); head = LinkRangeHead(link, settle_s=0.15)
head.aim(-45.0, 3.5); aim_line = json.dumps(link.sent[-1], separators=(",", ":"))
aim0 = json.dumps({"t": "aim", "deg": 0.0}, separators=(",", ":"))
LinkDrive(link).send(5.0, Drive(0.4, -1)); drive_line = json.dumps(link.sent[-1], separators=(",", ":"))
tr, o = run([(100, '{"t":"mode","m":"phone"}'), (200, "go"), (3500, aim_line), (4000, aim0)] + [(t, drive_line) for t in range(5000, 6500, 100)],
            ["--right", 123.4, "--left", 77.7], t_end=8000)
rng = [json.loads(m) for t, m in o if m.startswith('{"t":"range"')]
left = [r for r in rng if r["deg"] == -45.0]
fails += not ok(left and abs(left[-1]["cm"] - 77.7) < 1.0, "firmware aceita o aim gerado pelo hal.py e responde range deg=-45.0")
fails += not ok(float(at(tr, 6450)["curL"]) < 0.1 < float(at(tr, 6450)["curR"]), "firmware aceita o drive gerado pelo hal.py (v=0.4, s=-1: roda esquerda mais lenta = curva para a esquerda)")
h2 = LinkRangeHead(l2 := MemoryLink(), settle_s=0.15); h2.aim(-45.0, 0.0); l2.push(left[-1]); r = h2.read(0.3)
fails += not ok(r.settled and abs(r.dist_m - 0.777) < 0.01, f"LinkRangeHead le a mensagem do firmware: {r.dist_m} m")
l2.push({"t": "range", "deg": -45.0, "cm": None}); fails += not ok(h2.read(0.4).dist_m is None, "cm=null -> dist_m=None (sem eco)")

print("\nTODOS OS CENARIOS PASSARAM" if not fails else f"\n{fails} FALHA(S)")
sys.exit(1 if fails else 0)
