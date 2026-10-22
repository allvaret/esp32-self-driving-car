"""test_controller.py - testes sinteticos da fusao + FSM (sem video, sem control).
Rode:  python test_controller.py"""
import numpy as np

from vision.decision import State as V
from control.hal import (FAR_M, LinkDrive, LinkRangeHead, MemoryLink, MockRangeHead, Scenario)
from control.controller import CState, Controller, Vision

DT = 0.1


def run(scn, vis_fn, T=10.0, **mock):
    head = MockRangeHead(scn, noise_m=0.0, **mock)
    ctl, log = Controller(head), []
    for i in range(int(T / DT)):
        t = i * DT
        drive, info = ctl.update(t, vis_fn(t))
        log.append((t, info))
    return log


def states(log):
    out, prev = [], None
    for t, i in log:
        key = (i["ctrl_state"], i["sub"])
        if key != prev:
            out.append((round(t, 1), *key)); prev = key
    return out


def has(log, cstate, sub=None):
    return any(i["ctrl_state"] == cstate and (sub is None or i["sub"] == sub) for _, i in log)


def scn_obstacle(t_close=3.0, c_after=FAR_M, left=2.0, right=0.3):
    # centro livre ate t_close, depois obstaculo a 0.3 m; lados: left/right
    return Scenario([0, t_close, t_close + 0.1, 100], [3, 3, 0.3, 0.3], [left] * 4, [right] * 4)


def test_cruise_all_clear():
    log = run(Scenario.constant(), lambda t: Vision(V.LIVRE), T=5)
    assert all(i["ctrl_state"] == "CRUISE" for _, i in log); print("ok  cruise limpo")


def test_sensor_stops_even_if_vision_says_livre():     # parede lisa: cor cega, sensor salva
    log = run(scn_obstacle(), lambda t: Vision(V.LIVRE), T=6)
    t_av = next(t for t, i in log if i["ctrl_state"] == "AVOID")
    assert 3.0 <= t_av <= 3.3, t_av
    assert has(log, "AVOID", "STOP"); print(f"ok  sensor para com visao LIVRE (t={t_av:.1f}s)")


def test_picks_freer_side_left():
    log = run(scn_obstacle(left=2.0, right=0.3), lambda t: Vision(V.LIVRE), T=9)
    steer = [i["steer"] for _, i in log if i["sub"] == "TURN" and i["v"] > 0]   # v>0: ignora o tick de transicao
    assert steer and all(s == -1.0 for s in steer), steer
    assert has(log, "AVOID", "REVERSE") and log[-1][1]["ctrl_state"] in ("SCAN", "AVOID")
    print("ok  escolhe lado esquerdo (mais livre)  ", states(log)[:7])


def test_picks_freer_side_right():
    log = run(scn_obstacle(left=0.3, right=2.0), lambda t: Vision(V.LIVRE), T=9)
    steer = [i["steer"] for _, i in log if i["sub"] == "TURN" and i["v"] > 0]
    assert steer and all(s == +1.0 for s in steer); print("ok  escolhe lado direito")


def test_vision_obstacle_refuted_by_sensor():          # falso obstaculo (sol/reflexo)
    log = run(Scenario.constant(center=1.5), lambda t: Vision(V.OBSTACULO), T=4)
    assert not has(log, "AVOID") and has(log, "SCAN")
    assert all(i["v"] == 0.2 for _, i in log if i["ctrl_state"] == "SCAN"); print("ok  OBSTACULO refutado -> segue devagar")


def test_vision_obstacle_sensor_no_echo():              # sem eco nao refuta
    log = run(Scenario.constant(), lambda t: Vision(V.OBSTACULO), T=2, max_range_m=2.0)
    assert has(log, "AVOID"); print("ok  OBSTACULO sem eco -> AVOID")


def test_incerto_goes_scan_then_back_to_cruise():
    vis = lambda t: Vision(V.INCERTO if 1.0 <= t < 2.0 else V.LIVRE)
    log = run(Scenario.constant(), vis, T=4)
    seq = [s for s in states(log)]
    assert any(s[1] == "SCAN" for s in seq) and log[-1][1]["ctrl_state"] == "CRUISE"
    print("ok  INCERTO -> SCAN -> CRUISE      ", seq)


def test_edge_near_escalates_to_scan_only():
    vis = lambda t: Vision(V.LIVRE, edge_near=(1.0 <= t < 2.0))
    log = run(Scenario.constant(), vis, T=4)
    assert has(log, "SCAN") and not has(log, "AVOID"); print("ok  borda proxima escala a SCAN (nao a AVOID)")


def test_thin_leg_fault_is_a_known_miss():            # limitacao documentada
    scn = Scenario.constant(center=0.3)
    log = run(scn, lambda t: Vision(V.LIVRE), T=3, faults=[(0, 3, "far")])
    assert not has(log, "AVOID"); print("ok  [limitacao] sensor 'far' + visao LIVRE nao freia (perna fina)")


def test_boxed_in_goes_hold_and_releases():
    scn = Scenario([0, 3, 3.1, 14, 14.1, 100], [3, 3, .3, .3, 3, 3], [.3] * 6, [.3] * 6)
    log = run(scn, lambda t: Vision(V.LIVRE), T=20)
    assert has(log, "AVOID", "HOLD"); assert log[-1][1]["ctrl_state"] in ("SCAN", "CRUISE")
    print("ok  sem saida -> HOLD -> libera  ", states(log)[:12])


def test_link_head_roundtrip():
    link = MemoryLink(); head = LinkRangeHead(link, settle_s=0.1)
    head.aim(0, 0.0)
    assert link.sent[-1] == {"t": "aim", "deg": 0.0}
    assert not head.read(0.05).settled
    link.push({"t": "range", "deg": 0.0, "cm": 123.4})
    r = head.read(0.2); assert r.settled and abs(r.dist_m - 1.234) < 1e-9
    link.push({"t": "range", "deg": 0.0, "cm": None})
    assert head.read(0.3).dist_m is None
    LinkDrive(link).send(0.3, __import__("control.hal", fromlist=["Drive"]).Drive(0.4, -1))
    assert link.sent[-1] == {"t": "drive", "v": 0.4, "s": -1}; print("ok  LinkRangeHead/LinkDrive (MemoryLink)")


def test_serial_loopback():
    try:
        import serial  # noqa
    except ImportError:
        print("--  pyserial ausente: teste de loopback pulado"); return
    from control.hal import SerialLink
    s = SerialLink("loop://")
    s.send({"t": "range", "deg": 0, "cm": 50.0}); s.ser.write(b"lixo{\n")
    got = s.poll()
    assert got == [{"t": "range", "deg": 0, "cm": 50.0}], got; print("ok  SerialLink loop:// (ruido ignorado)")


if __name__ == "__main__":
    for n, f in list(globals().items()):
        if n.startswith("test_"):
            f()
    print("todos os testes passaram")