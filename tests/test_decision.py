"""tests/test_decision.py - decision.py em segundos: independente do fps.
Rode:  python -m tests.test_decision"""
from config import DecisionConfig
from vision.decision import DecisionMaker, State


def synth(dt, T=8.0):
    """prob_mean_cen: 0.4 (chao) ate 3 s, 0.02 (obstaculo) de 3 a 5 s, volta a 0.4."""
    out, n = [], int(round(T / dt))
    for i in range(n):
        t = i * dt
        out.append((t, 0.02 if 3.0 <= t < 5.0 else 0.4))
    return out


def times(dt):
    dm, t_obst, t_back = DecisionMaker(DecisionConfig()), None, None
    for t, p in synth(dt):
        s = dm.update(p, 0.0, 0.0, t)
        if s == State.OBSTACULO and t_obst is None:
            t_obst = t
        if t_obst is not None and t > 5.0 and s == State.LIVRE and t_back is None:
            t_back = t
    return t_obst, t_back


def test_fps_independent():
    res = {dt: times(dt) for dt in (0.05, 0.1, 0.2)}
    for k in (0, 1):
        vals = [v[k] for v in res.values()]
        assert None not in vals and max(vals) - min(vals) <= 0.2 + 1e-9, res
    print("ok  decision: entrada/saida de OBSTACULO coerentes com dt = 0.05/0.1/0.2 :", res)


def test_warmup_is_time_based():
    for dt in (0.05, 0.2):
        dm, t_ready = DecisionMaker(DecisionConfig()), None
        for t, p in synth(dt):
            dm.update(p, 0.0, 0.0, t)
            if dm.ref is not None and t_ready is None:
                t_ready = t
        assert 0.9 <= t_ready < 0.9 + dt + 1e-9, (dt, t_ready)
    print("ok  referencia pronta apos ~0.9 s, qualquer fps")


if __name__ == "__main__":
    test_fps_independent(); test_warmup_is_time_based(); print("todos os testes passaram")