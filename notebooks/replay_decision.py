"""replay_decision.py - reproduz decision.py sobre um coverage_corredor_auto.csv (sem video).

Uso:
    python -m notebooks.replay_decision coverage_sofa.csv
    python -m notebooks.replay_decision assets/simulation_Scar/Reframed_videos/docs/logs/flow_parede.csv --out dec_parede
Gera <out>.csv (estado por frame), <out>.png e imprime as transicoes.
"""
import argparse
import csv

import numpy as np

from vision import decision as D
from vision.decision import DecisionMaker, State

COLORS = {State.LIVRE: "#2ca02c", State.INCERTO: "#ffbf00", State.OBSTACULO: "#d62728"}


def replay(csv_path):
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    dm = DecisionMaker()
    out = []
    for row in rows:
        st = dm.update(float(row["prob_mean_cen"]),
                       float(row["unknown_inf"]), float(row["unknown_cen"]))
        out.append({"frame_idx": int(row["frame_idx"]), "t": float(row["t"]),
                    "r": dm.r, "state": st.value, "warmup": dm.warmup,
                    "unknown_inf": float(row["unknown_inf"]),
                    "unknown_cen": float(row["unknown_cen"])})
    return out, dm.ref


def print_transitions(out, ref):
    print(f"ref (mediana prob_mean_cen, {D.REF_FRAMES} 1os frames) = {ref:.3f}")
    prev = None
    for o in out:
        if o["state"] != prev:
            r = "  -  " if o["r"] is None else f"{o['r']:.2f}"
            print(f"  t={o['t']:5.1f}s  frame {o['frame_idx']:3d}  -> {o['state']:<9}  r={r}")
            prev = o["state"]
    n = {s.value: sum(o["state"] == s.value for o in out) for s in State}
    print("  frames por estado:", n)


def plot(out, png_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = np.array([o["t"] for o in out])
    r = np.array([np.nan if o["r"] is None else o["r"] for o in out])
    fig, (a1, a2, a3) = plt.subplots(3, 1, figsize=(10, 7), sharex=True,
                                     gridspec_kw={"height_ratios": [3, 2, 1]})
    a1.plot(t, r, color="k", label="r = prob_mean_cen / ref")
    for v, lab in ((D.R_FREE, "R_FREE"), (D.R_EXIT, "R_EXIT"), (D.R_OBST, "R_OBST")):
        a1.axhline(v, ls="--", lw=0.8, color="gray")
        a1.text(t[-1], v, f" {lab}", va="center", fontsize=8)
    a1.set_ylim(0, max(1.6, np.nanmax(r) * 1.05))
    a1.set_title("Confianca relativa do centro", fontsize=10)
    a1.grid(alpha=0.3)
    a1.legend(loc="upper left", fontsize=8)

    a2.plot(t, [o["unknown_inf"] for o in out], label="unknown_inf")
    a2.plot(t, [o["unknown_cen"] for o in out], label="unknown_cen")
    a2.axhline(D.U_UNC, ls="--", lw=0.8, color="gray")
    a2.set_ylim(0, 1.02)
    a2.set_title("Desconhecido", fontsize=10)
    a2.grid(alpha=0.3)
    a2.legend(loc="upper left", fontsize=8)

    dt = np.median(np.diff(t)) if len(t) > 1 else 0.1
    for o in out:
        a3.axvspan(o["t"] - dt / 2, o["t"] + dt / 2, color=COLORS[State(o["state"])])
    a3.set_yticks([])
    a3.set_title("Estado (verde=LIVRE, amarelo=INCERTO, vermelho=OBSTACULO)", fontsize=10)
    a3.set_xlabel("tempo (s)")
    fig.tight_layout()
    fig.savefig(png_path, dpi=120)
    plt.close(fig)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    base = a.out or a.csv.rsplit(".", 1)[0].replace("coverage", "decision")
    out, ref = replay(a.csv)
    with open(f"{base}.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(out[0].keys()))
        w.writeheader()
        w.writerows(out)
    plot(out, f"{base}.png")
    print_transitions(out, ref)
    print(f"-> {base}.csv, {base}.png")