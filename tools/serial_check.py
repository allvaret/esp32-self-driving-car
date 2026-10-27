"""tools/serial_check.py - confere, do PC, o protocolo serial com o ESP32 (e registra leituras do HC-SR04).

Usa o MESMO hal.py do celular (SerialLink + LinkRangeHead), entao tambem valida esse codigo
no hardware real. Nao liga motores, a menos que voce passe --drive-test.

    python -m tools.serial_check COM5                         # varre -45/0/+45/0 e mostra as leituras
    python -m tools.serial_check COM5 --log bancada.csv       # grava t,deg,cm (bancada do HC-SR04)
    python -m tools.serial_check COM5 --hold 0 --secs 30      # fica no centro 30 s (objeto a distancia conhecida)
    python -m tools.serial_check COM5 --drive-test --v 0.3    # CUIDADO: arma e anda 1 s (carro no chao, livre)
Porta: Windows 'COM5'; Linux '/dev/ttyUSB0'. Feche o Monitor Serial do Arduino antes (a porta e exclusiva).
"""
import argparse
import csv
import statistics
import time

from control.hal import Drive, LinkDrive, LinkRangeHead


def collect(head, link, angle, secs, log=None, clock=time.monotonic, sleep=time.sleep):
    """Aponta o servo para `angle` e le por `secs` s. Devolve (leituras_cm, n_sem_eco)."""
    t0 = clock()
    head.aim(float(angle), t0)
    cms, none = [], 0
    last = None
    while clock() - t0 < secs:
        r = head.read(clock())
        if r.settled and r.t != last:
            last = r.t
            if r.dist_m is None:
                none += 1
            else:
                cms.append(r.dist_m * 100.0)
            if log:
                log.writerow([f"{r.t:.3f}", angle, "" if r.dist_m is None else f"{r.dist_m * 100.0:.1f}"])
        sleep(0.02)
    return cms, none


def run_check(link, angles=(-45, 0, 45, 0), secs=1.5, log=None, drive_test=False, v=0.3,
              clock=time.monotonic, sleep=time.sleep):
    head = LinkRangeHead(link, settle_s=0.15)
    link.send({"t": "mode", "m": "phone"})
    link.send({"t": "ping"})
    t0 = clock()
    got_pong = False
    while clock() - t0 < 2.0 and not got_pong:
        got_pong = any(m.get("t") == "pong" for m in link.poll())
        sleep(0.05)
    print("ping/pong:", "ok" if got_pong else "SEM RESPOSTA (porta errada? firmware gravado? Monitor Serial aberto?)")
    summary = {}
    for a in angles:
        cms, none = collect(head, link, a, secs, log, clock, sleep)
        n = len(cms) + none
        med = statistics.median(cms) if cms else None
        summary.setdefault(a, []).append((n, none, med))
        print(f"  angulo {a:+4d} graus: {n:3d} leituras, sem eco {none:3d}, mediana "
              f"{'-' if med is None else f'{med:6.1f} cm'}")
    if drive_test:
        drv = LinkDrive(link)
        print("drive-test: armando (3 s)... afaste-se.")
        link.send({"t": "arm", "on": True})
        t1 = clock()
        while clock() - t1 < 3.5:
            head.aim(0.0, clock()); head.read(clock()); sleep(0.05)
        t1 = clock()
        while clock() - t1 < 1.0:
            drv.send(clock(), Drive(v, 0.0)); head.read(clock()); sleep(0.1)
        drv.send(clock(), Drive(0.0, 0.0))
        link.send({"t": "arm", "on": False})
        print("drive-test: fim (desarmado).")
    return got_pong, summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--hold", type=float, default=None, help="so um angulo (graus), por --secs")
    ap.add_argument("--secs", type=float, default=1.5)
    ap.add_argument("--log", default=None)
    ap.add_argument("--drive-test", action="store_true")
    ap.add_argument("--v", type=float, default=0.3)
    a = ap.parse_args()
    from control.hal import SerialLink
    link = SerialLink(a.port, a.baud)
    time.sleep(2.0)                                  # o ESP32 reinicia ao abrir a porta
    f = open(a.log, "w", newline="") if a.log else None
    log = csv.writer(f) if f else None
    if log:
        log.writerow(["t", "deg", "cm"])
    angles = (a.hold,) if a.hold is not None else (-45, 0, 45, 0)
    try:
        run_check(link, angles, a.secs, log, a.drive_test, a.v)
    finally:
        link.send({"t": "drive", "v": 0.0, "s": 0.0})
        if f:
            f.close()


if __name__ == "__main__":
    main()