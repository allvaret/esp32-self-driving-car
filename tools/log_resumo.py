"""log_resumo.py - resume um log de voo do S9 (corrida_*.jsonl; antes da 0.7.0 explorar_*.jsonl).

Uso:  python tools/log_resumo.py docs/logs/corrida_2026-11-26_13h01m18_sensor.jsonl [--giro]

Mostra, em segundos desde o inicio do arquivo:
  - mudancas de estado da VISAO (vis ...)
  - mudancas de estado do ESP32 (ESP ...), logs do ESP32 e decisoes de varredura (DECIDE)
  - pedidos de desvio enviados pelo celular (TX avoid)
Com --giro: para cada heartbeat do ESP com motores ligados, a taxa de giro mediana
nos 500 ms seguintes (deveria ser ~0 quando L == R).
"""
import json
import sys


def load(path):
    out = []
    for line in open(path, encoding="utf-8"):
        line = line.strip()
        if line:
            try:
                out.append(json.loads(line))
            except ValueError:
                pass
    return out


def resumo(L):
    t0 = L[0]["w"]
    sec = lambda w: (w - t0) / 1000
    out = []
    frames = [x for x in L if x["k"] == "f"]
    run = [x for x in frames if x.get("ph") == "RUN"]
    if run:
        ms = sorted(x["ms"] for x in run)
        print(f"frames RUN: {len(run)} | processamento mediano {ms[len(ms)//2]:.1f} ms, "
              f"p95 {ms[int(.95*len(ms))]:.1f} ms")
    prev = None
    for x in frames:
        k = (x.get("ph"), x.get("vs"))
        if k != prev:
            r = None if x.get("r") is None else round(x["r"], 2)
            out.append((sec(x["w"]), f"vis {x.get('ph')}/{x.get('vs')} r={r} sec={x.get('sec')}"))
            prev = k
    prev = None
    for x in L:
        if x["k"] == "rx":
            m = x["m"]
            t = m.get("t")
            if t == "state":
                k = (m["armed"], m["st"], m["sub"], m.get("esp"), m.get("cls"))
                if k != prev:
                    out.append((sec(x["w"]), f"ESP armed={m['armed']} {m['st']}/{m['sub']} esp={m.get('esp')} "
                                f"cls={m.get('cls')} c={m['c']} ang={m.get('ang')} L={m['L']} R={m['R']} "
                                f"why={m['why']}"))
                    prev = k
            elif t == "log":
                out.append((sec(x["w"]), f"ESPlog {m['msg']}"))
            elif t == "scan" and m.get("k") == "decide":
                out.append((sec(x["w"]), f"DECIDE ang={m.get('ang')} sem_saida={m.get('x')} "
                            f"angulos={m.get('d')} notas={m.get('sc')}"))
        elif x["k"] == "tx" and x["m"].get("t") == "avoid":
            out.append((sec(x["w"]), f"TX avoid {x['m']}"))
    for t, s in sorted(out, key=lambda p: p[0]):
        print(f"{t:7.2f}  {s}")


def giro(L):
    t0 = L[0]["w"]
    yaws = [(x["w"], x["m"]["r"], x["m"]["psi"]) for x in L if x["k"] == "tx" and x["m"].get("t") == "yaw"]
    print("\n  t      estado            L      R    giro(°/s)  psi")
    for x in L:
        if x["k"] != "rx" or x["m"].get("t") != "state":
            continue
        m, w = x["m"], x["w"]
        if not m["armed"] or (abs(m["L"]) < 0.02 and abs(m["R"]) < 0.02):
            continue
        seg = [(r, p) for (ww, r, p) in yaws if w <= ww < w + 500]
        if not seg:
            continue
        rates = sorted(r for r, _ in seg)
        print(f"{(w-t0)/1000:6.2f}  {m['st'] + '/' + m['sub']:<16} {m['L']:5.2f}  {m['R']:5.2f}  "
              f"{rates[len(rates)//2]:8.1f}   {seg[0][1]:.0f}->{seg[-1][1]:.0f}")


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    for p in args:
        print("=====", p)
        L = load(p)
        resumo(L)
        if "--giro" in sys.argv:
            giro(L)