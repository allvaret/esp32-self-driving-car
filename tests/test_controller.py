"""tests/test_controller.py - testes sinteticos de fusao + FSM + config + enlaces.
Rode da raiz do repositorio:   python -m tests.test_controller"""
import json
import os
import tempfile

from config import Config, ControlConfig, derive_maneuver_times
from control.controller import CState, Controller, Vision
from control.hal import (FAR_M, Drive, LinkDrive, LinkRangeHead, MemoryLink,
                          MockRangeHead, Scenario)
from vision.decision import State as V


def run(scn, vis_fn, T=10.0, dt=0.1, cfg=None, **mock):
    head = MockRangeHead(scn, noise_m=0.0, **mock)
    ctl, log = Controller(head, cfg), []
    for i in range(int(round(T / dt))):
        t = i * dt
        _, info = ctl.update(t, vis_fn(t))
        log.append((t, info))
    return log


def states(log):
    out, prev = [], None
    for t, i in log:
        key = (i["ctrl_state"], i["sub"])
        if key != prev:
            out.append((round(t, 2), *key)); prev = key
    return out


def has(log, cstate, sub=None):
    return any(i["ctrl_state"] == cstate and (sub is None or i["sub"] == sub) for _, i in log)


def scn_obstacle(t_close=3.0, left=2.0, right=0.3):
    return Scenario([0, t_close, t_close + 0.1, 100], [3, 3, 0.3, 0.3], [left] * 4, [right] * 4)


# ------------------------------------------------------------- fusao / FSM ---
def test_cruise_all_clear():
    log = run(Scenario.constant(), lambda t: Vision(V.LIVRE), T=5)
    assert all(i["ctrl_state"] == "CRUISE" for _, i in log); print("ok  cruise limpo")


def test_sensor_stops_even_if_vision_says_livre():
    log = run(scn_obstacle(), lambda t: Vision(V.LIVRE), T=6)
    t_av = next(t for t, i in log if i["ctrl_state"] == "AVOID")
    assert 3.0 <= t_av <= 3.3, t_av; print(f"ok  sensor para com visao LIVRE (t={t_av:.1f}s)")


def test_picks_freer_side():
    for left, right, want in ((2.0, 0.3, -1.0), (0.3, 2.0, +1.0)):
        log = run(scn_obstacle(left=left, right=right), lambda t: Vision(V.LIVRE), T=9)
        steer = [i["steer"] for _, i in log if i["sub"] == "TURN" and i["v"] > 0]
        assert steer and all(s == want for s in steer), steer
    print("ok  escolhe o lado mais livre (esq e dir)")


def test_vision_obstacle_refuted_by_sensor():
    log = run(Scenario.constant(center=1.5), lambda t: Vision(V.OBSTACULO), T=4)
    assert not has(log, "AVOID") and has(log, "SCAN"); print("ok  OBSTACULO refutado -> segue devagar")


def test_vision_obstacle_sensor_no_echo():
    log = run(Scenario.constant(), lambda t: Vision(V.OBSTACULO), T=2, max_range_m=2.0)
    assert has(log, "AVOID"); print("ok  OBSTACULO sem eco -> AVOID")


def test_edge_near_escalates_to_scan_only():
    log = run(Scenario.constant(), lambda t: Vision(V.LIVRE, edge_near=(1.0 <= t < 2.0)), T=4)
    assert has(log, "SCAN") and not has(log, "AVOID"); print("ok  borda proxima escala a SCAN (nao a AVOID)")


def test_thin_leg_fault_is_a_known_miss():
    log = run(Scenario.constant(center=0.3), lambda t: Vision(V.LIVRE), T=3, faults=[(0, 3, "far")])
    assert not has(log, "AVOID"); print("ok  [limitacao] sensor 'far' + visao LIVRE nao freia (perna fina)")


def test_boxed_in_goes_hold_and_releases():
    scn = Scenario([0, 3, 3.1, 14, 14.1, 100], [3, 3, .3, .3, 3, 3], [.3] * 6, [.3] * 6)
    log = run(scn, lambda t: Vision(V.LIVRE), T=20)
    assert has(log, "AVOID", "HOLD") and log[-1][1]["ctrl_state"] in ("SCAN", "CRUISE")
    print("ok  sem saida -> HOLD -> libera")


# ------------------------------------------------- segundos, nao ticks (fps) ---
def test_time_based_hysteresis_is_fps_independent():
    vis = lambda t: Vision(V.INCERTO if 1.0 <= t < 2.0 else V.LIVRE)
    back = []
    for dt in (0.05, 0.1, 0.2):
        log = run(Scenario.constant(), vis, T=4, dt=dt)
        back.append(next(t for t, i in log if t > 1.5 and i["ctrl_state"] == "CRUISE"))
    assert max(back) - min(back) < 1e-6, back
    print("ok  volta a CRUISE no mesmo instante com dt = 0.05/0.1/0.2 :", back)


# ------------------------------------------------ guarda de reentrada/backoff ---
def test_reentry_guard_backoff_sequence():
    scn = Scenario.constant(center=0.3, left=2.0, right=0.3)     # obstaculo que nunca sai
    log = run(scn, lambda t: Vision(V.LIVRE), T=60)
    seq = states(log)
    waits = [(t, next((t2 for t2, *_ in seq if t2 > t), log[-1][0] + 0.1)) for t, cs, sub in seq if sub == "WAIT"]
    durs = [round(b - a, 2) for a, b in waits]
    assert len(durs) >= 3 and abs(durs[0] - 1.0) < 0.11 and abs(durs[1] - 2.0) < 0.11 \
           and abs(durs[2] - 4.0) < 0.11, durs
    n_before = sum(1 for t, cs, sub in seq if sub == "STOP" and t < waits[0][0])
    assert n_before == 2, n_before                               # 2 manobras normais antes do 1o WAIT
    print("ok  guarda: 2 manobras, depois esperas de", durs[:4], "s")


def test_reentry_guard_resets_after_quiet_window():
    scn = Scenario([0, 4, 4.1, 39.9, 40, 100], [.3, .3, 3, 3, .3, .3], [2.0] * 6, [.3] * 6)
    log = run(scn, lambda t: Vision(V.LIVRE), T=44)
    late = [(t, i) for t, i in log if t >= 40 and i["ctrl_state"] == "AVOID"]
    assert late and late[0][1]["sub"] == "STOP" and late[0][1]["guard_level"] == 0
    print("ok  guarda zera depois de uma janela sem AVOID")


# --------------------------------------------------------------- config ---
def test_config_roundtrip_and_errors():
    cfg = Config(); cfg.control.guard_backoff_s = (0.5, 1.5)
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "c.json"); cfg.save(p)
        back = Config.load(p)
        assert back.control.guard_backoff_s == (0.5, 1.5) and back == cfg
        bad = json.load(open(p)); bad["control"]["d_stopp_m"] = 1
        json.dump(bad, open(p, "w"))
        try:
            Config.load(p); assert False
        except ValueError as e:
            assert "d_stopp_m" in str(e)
    try:
        Config.from_dict({"control": {"d_stop_m": 2.0, "d_clear_m": 1.0}}); assert False
    except ValueError:
        pass
    print("ok  config: JSON ida e volta, typo e valores incoerentes rejeitados")


def test_derive_maneuver_times():
    cfg = Config()
    cfg.calib.speed_at_v_rev_mps, cfg.calib.yaw_rate_dps = 0.25, 45.0
    rev, turn = derive_maneuver_times(cfg, reverse_m=0.20, turn_deg=45)
    assert abs(rev - 0.8) < 1e-9 and abs(turn - 1.0) < 1e-9
    try:
        derive_maneuver_times(Config(), 0.2, 45); assert False
    except ValueError:
        pass
    print("ok  derive_maneuver_times (0.20 m a 0.25 m/s -> 0.8 s; 45 graus a 45 graus/s -> 1.0 s)")


# ---------------------------------------------------------------- enlaces ---
def test_link_head_roundtrip():
    link = MemoryLink(); head = LinkRangeHead(link, settle_s=0.1)
    head.aim(0, 0.0)
    assert link.sent[-1] == {"t": "aim", "deg": 0.0} and not head.read(0.05).settled
    link.push({"t": "range", "deg": 0.0, "cm": 123.4})
    r = head.read(0.2); assert r.settled and abs(r.dist_m - 1.234) < 1e-9
    link.push({"t": "range", "deg": 0.0, "cm": None}); assert head.read(0.3).dist_m is None
    LinkDrive(link).send(0.3, Drive(0.4, -1))
    assert link.sent[-1] == {"t": "drive", "v": 0.4, "s": -1}; print("ok  LinkRangeHead/LinkDrive (MemoryLink)")


def test_serial_loopback():
    try:
        import serial  # noqa
    except ImportError:
        print("--  pyserial ausente: teste de loopback pulado"); return
    from control.hal import SerialLink
    s = SerialLink("loop://")
    s.send({"t": "range", "deg": 0, "cm": 50.0}); s.ser.write(b"lixo{\n")
    assert s.poll() == [{"t": "range", "deg": 0, "cm": 50.0}]; print("ok  SerialLink loop:// (ruido ignorado)")


if __name__ == "__main__":
    for n, f in list(globals().items()):
        if n.startswith("test_"):
            f()
    print("todos os testes passaram")