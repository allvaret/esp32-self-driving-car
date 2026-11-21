"""check_firmware.py - cenarios de verificacao do car_firmware.ino no simulador (sem hardware).

Compile antes:
    cd sim && g++ -std=c++17 -DESP_ARDUINO_VERSION_MAJOR=3 -I. -o sim3 sim.cpp
Rode:  python check_firmware.py        (sai com codigo 1 se algo falhar)

Mundo do simulador (sim.cpp): sensor por angulo (-30,-15,0,+15,+30 graus), obstaculo temporario no centro
(--obst t cm [--clear t]), janelas sem eco (--none t0 t1), yaw simulado das rodas (--yaw --world-dps N).
Relogio virtual: `go` aos 200 ms arma aos 4200 ms (ARM_DELAY_MS = 4000).
"""
import json, os, re, subprocess, sys, tempfile
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

# 6b) fiacao fisica (sim.cpp: ENA/IN1/IN2 = roda DIREITA, ENB/IN3/IN4 = roda ESQUERDA; o yaw sai dos PINOS)
tr, o = run(base + drives(5000, 7000, 0.3, 1.0), t_end=7000)
r = at(tr, 6500)
fails += not ok(float(r["uL"]) > float(r["uR"]) and float(r["rate"]) > 0 and int(r["enbPWM"]) > int(r["enaPWM"]),
                f"(fiacao) s=+1 (direita): roda ESQUERDA (ENB) mais rapida e o carro gira para a DIREITA "
                f"(uE={r['uL']} uD={r['uR']} giro={r['rate']} ENB={r['enbPWM']} ENA={r['enaPWM']})")
tr, o = run(base + drives(5000, 7000, 0.3, 0.0), t_end=7000)
r = at(tr, 6500)
fails += not ok(float(r["uL"]) > 0.25 and float(r["uR"]) > 0.25 and abs(float(r["rate"])) < 0.5
                and r["in1"] == "0" and r["in2"] == "1" and r["in3"] == "0" and r["in4"] == "1",
                f"(fiacao) v=0.3 s=0: as duas rodas PARA FRENTE (IN1/IN3=LOW, IN2/IN4=HIGH, sentido de giro inalterado), sem giro")
tr, o = run(base + [(4300, '{"t":"cfg","trim_left":0.5}')] + drives(5000, 7000, 0.4), t_end=7000)
r = at(tr, 6500)
fails += not ok(float(r["uL"]) < float(r["uR"]) and abs(float(r["uL"]) - 0.2) < 0.03,
                f"(fiacao) trim_left atua na roda ESQUERDA fisica (ENB): uE={r['uL']} uD={r['uR']}")

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
fails += not ok(500 <= dur3 <= 700, f"(e) com yaw mas o carro nao gira: timeout da curva ({dur3} ms)")
dur4, dpsi4 = curva(["--yaw", "--world-dps", 90, "--set", "turn_dps", 30])
fails += not ok(abs(dpsi4) <= 20 and dur4 < 400, f"(e) com yaw, turn_dps errado nao importa: {dpsi4:+.1f} graus")

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
fails += not ok(all(abs(m) < 10 for m, _, _ in st), f"(l) com correcao: giro medio de 1 a 2 s de cada trecho < 10 graus/s: {[round(m, 1) for m, _, _ in st]}")
first = [float(r["rate"]) for r in tr if segs[0] + 500 <= r["t"] < segs[0] + 1000]
fails += not ok(abs(sum(first) / len(first)) < 25 and abs(st[0][0]) < 10,
                f"(l) 1o trecho (trim 0): em 1 s o giro medio ja cai abaixo de 10 graus/s ({st[0][0]:+.1f}; 0,5-1 s: {sum(first)/len(first):+.1f})")
trims = [tm for _, tm, _ in st]
fails += not ok(trims[0] < -0.5 and abs(trims[-1] - trims[-2]) < 0.03 and abs(trims[-2] - trims[-3]) < 0.06 and abs(st[-1][2] - trims[-1]) < 0.15,
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
fails += not ok(at(tr, 7000)["cst"] == "0" and abs(sum(w) / len(w)) < 10, f"(l) modo auto (fonte sensor): FRENTE tambem corrige ({sum(w)/len(w):+.1f} graus/s)")
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
