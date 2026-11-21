"""tests/test_brain.py - brain.py (opiniao da visao + rumo -> ESP32) com um enlace falso."""
import json
import sys
import types

try:
    import websocket  # noqa: F401
except ImportError:                                   # o PC dos testes pode nao ter websocket-client
    sys.modules["websocket"] = types.ModuleType("websocket")

import brain
from config import HeadingConfig
from control.heading import YawIntegrator


class FakeLink:
    def __init__(self):
        self.sent, self.connected = [], True
        self.goal, self.src, self.state = "nenhuma", None, {"armed": False}
        self.calib_seq, self.conn_seq = 0, 1
        self.center = None                         # (cm|None, t) como o WsLink

    def send(self, obj):
        self.sent.append(obj)
        return True

    def drive(self, v, s=0.0):
        return self.send({"t": "drive", "v": round(v, 3), "s": round(s, 3)})

    def arm(self, on=True):
        return self.send({"t": "arm", "on": bool(on)})

    def think(self, msg, **extra):
        return self.send({"t": "think", "msg": msg, **extra})

    def stop(self):
        pass

    def of(self, t):
        return [m for m in self.sent if m.get("t") == t]


class Clock:
    t = 100.0

    def __call__(self):
        return self.t

    def adv(self, dt):
        self.t += dt


def _setup(goal="nenhuma", src=None, armed=False):
    brain._link = lk = FakeLink()
    lk.goal, lk.src, lk.state = goal, src, {"armed": armed, "st": "FRENTE", "L": 0.0, "R": 0.0}
    brain._clock = clk = Clock()
    brain._last_key, brain._last_conn_seq, brain._armed_prev, brain._moving_prev = None, 0, False, False
    brain._last_vis_t = brain._last_think_t = brain._last_viz_t = brain._last_avoid_t = brain._last_yaw_t = -99.0
    brain._last_state, brain._oversize, brain._logger = None, 0, None
    brain._reset_motion()
    return lk, clk


def _info(vs="LIVRE", r=0.9, why="", sectors=(100, 100, 100, 100, 100), ok=True):
    return json.dumps({"phase": "RUN" if ok else "WARMUP", "ok": ok, "vision_state": vs, "r": r,
                       "sectors": list(sectors) if ok else None, "why": why})


def test_vision_state_is_explicit_not_substring():
    assert brain._norm_state(_info("LIVRE", why="visao OBSTACULO sem refutacao")) == "LIVRE"
    assert brain._norm_state(_info("INCERTO", why="OBSTACULO LIVRE")) == "INCERTO"
    assert brain._norm_state(json.dumps({"vision_state": "-", "why": "OBSTACULO"})) is None
    assert brain._norm_state("LIVRE") == "LIVRE" and brain._norm_state("OBSTÁCULO") == "OBSTACULO"
    assert brain._norm_state("tem a palavra OBSTACULO no meio") is None
    p = brain._parse(_info("LIVRE", r=7.3))
    assert p["conf"] == 1.0 and brain._parse(_info("LIVRE", r=-1))["conf"] == 0.0
    print("ok  brain: vision_state lido explicitamente (why com OBSTACULO + LIVRE -> LIVRE); conf = r em [0,1]")


def test_src_sets_mode_and_never_arms():
    lk, clk = _setup("explorar", "sensor", armed=False)
    brain.step(_info())
    assert lk.of("mode") == [{"t": "mode", "m": "auto"}]
    assert not lk.of("drive") and not lk.of("arm") and not lk.of("avoid")
    lk.src = "visao"
    clk.adv(0.2)
    brain.step(_info())
    assert lk.of("mode")[-1] == {"t": "mode", "m": "phone"} and len(lk.of("mode")) == 2
    lk.src = None                                  # src ausente -> sensor
    clk.adv(0.2)
    brain.step(_info())
    assert lk.of("mode")[-1] == {"t": "mode", "m": "auto"}
    assert not lk.of("arm")
    print("ok  src sensor->auto, visao->phone, ausente->auto; reage a mudanca de src; nunca arma")


def test_reconnect_resends_mode_without_arming():
    lk, clk = _setup("explorar", "visao", armed=True)
    brain.step(_info())
    n = len(lk.of("mode"))
    clk.adv(0.2)
    brain.step(_info())
    assert len(lk.of("mode")) == n                 # sem mudanca: nada reenviado
    lk.conn_seq += 1                               # WS caiu e voltou
    clk.adv(0.2)
    brain.step(_info())
    assert len(lk.of("mode")) == n + 1 and lk.of("mode")[-1]["m"] == "phone" and not lk.of("arm")
    print("ok  reconexao reenvia o mode do src atual e nao rearma")


def test_parar_resets_goal_heading_and_integrator():
    lk, clk = _setup("explorar", "visao", armed=True)
    brain.step(_info())
    brain.on_gyro(30.0)
    clk.adv(0.1)
    brain.on_gyro(30.0)
    assert brain._last_key == ("explorar", "visao") and abs(brain._yaw.psi) > 0
    lk.goal = "parar"
    lk.state["armed"] = False
    clk.adv(0.1)
    brain.step(_info())
    assert brain._last_key is None and brain._yaw.psi == 0.0
    lk.goal, lk.src = "explorar", "visao"          # nova motivacao: manda mode de novo, nao arma
    clk.adv(0.1)
    n = len(lk.of("mode"))
    brain.step(_info())
    assert len(lk.of("mode")) == n + 1 and not lk.of("arm")
    print("ok  goal parar zera goal/src lembrados, rumo e integrador; nova motivacao so manda mode")


def test_disarm_resets_key():
    lk, clk = _setup("explorar", "sensor", armed=True)
    brain.step(_info())
    n = len(lk.of("mode"))
    lk.state["armed"] = False                      # BOOT desarmou
    clk.adv(0.1)
    brain.step(_info())
    assert len(lk.of("mode")) == n + 1             # goal/src lembrado foi zerado: reafirma o mode (so isso)
    clk.adv(0.1)
    brain.step(_info())
    assert len(lk.of("mode")) == n + 1 and not lk.of("arm")     # e nao repete a cada frame, nem rearma
    print("ok  desarme (arm false) zera o goal/src lembrado; nunca rearma")


def test_src_visao_drives_and_asks_avoid_with_freest_sector():
    lk, clk = _setup("explorar", "visao", armed=True)
    brain.step(_info("LIVRE"))
    assert lk.of("drive")[-1]["v"] == brain.V_FREE
    clk.adv(0.1)
    brain.step(_info("INCERTO"))
    assert lk.of("drive")[-1]["v"] == brain.V_SLOW
    clk.adv(0.1)
    brain.step(_info("OBSTACULO", sectors=(10, 80, 0, 60, 40)))
    assert lk.of("drive")[-1]["v"] == 0.0
    assert lk.of("avoid") == [{"t": "avoid", "deg": -15}]
    clk.adv(0.1)
    brain.step(_info("OBSTACULO", sectors=(10, 80, 0, 60, 40)))
    assert len(lk.of("avoid")) == 1                # limitado a 1 a cada AVOID_EVERY_S
    assert brain._free_sector_deg((50, 50, 0, 50, 50)) == -15      # empate: menor |ang|, depois esquerda
    assert brain._free_sector_deg((70, 60, 0, 60, 70)) == -30
    print("ok  src=visao: v pela visao, avoid com o setor mais livre, limitado a cada 2 s")


def test_drive_is_straight_no_heading_correction_in_brain():
    lk, clk = _setup("explorar", "visao", armed=True)
    lk.state.update(L=0.35, R=0.35)
    for _ in range(40):                            # o carro girando 120 graus/s para a direita por 0,8 s
        brain.on_gyro(120.0)
        brain.step(_info("LIVRE"))
        clk.adv(0.02)
    ds = lk.of("drive")
    assert ds and all(d["s"] == 0.0 for d in ds), ds[-3:]
    assert lk.of("yaw")[-1]["r"] > 50              # o rumo vai ao ESP32, que e quem corrige
    assert not hasattr(brain, "_heading")
    lk.state.update(hold=2, hc=-1.25, trim=-1.2)
    assert "corrigindo" in brain.status() and "trim=-1.20" in brain.status()
    print("ok  brain manda s = 0 em reta (o ESP32 corrige o rumo); status mostra hold/corr/trim do ESP32")


def test_sensor_refutes_vision_obstacle():
    obst = _info("OBSTACULO", sectors=(100, 100, 0, 100, 100))
    lk, clk = _setup("explorar", "visao", armed=True)
    lk.center = (86.0, clk.t)                      # leitura central VALIDA, fresca, >= d_clear (60)
    for _ in range(30):                            # 3 s de OBSTACULO
        brain.step(obst)
        clk.adv(0.1)
        lk.center = (86.0, clk.t)
    assert not lk.of("avoid"), lk.of("avoid")
    assert all(d["v"] == brain.V_SLOW for d in lk.of("drive"))
    th = [m["msg"] for m in lk.of("think") if "refutada" in m.get("msg", "")]
    assert th and th[0] == "visão refutada pelo sensor (86 cm)", th[:1]
    for cm, age, why in ((45.0, 0.0, "zona intermediaria"), (25.0, 0.0, "perto"), (86.0, 1.0, "leitura velha"), (None, 0.0, "sem eco")):
        lk, clk = _setup("explorar", "visao", armed=True)
        lk.center = (cm, clk.t - age)
        brain.step(obst)
        assert lk.of("avoid") and lk.of("drive")[-1]["v"] == 0.0, why
    lk, clk = _setup("explorar", "visao", armed=True)   # LIVRE com sensor longe: nada muda
    lk.center = (86.0, clk.t)
    brain.step(_info("LIVRE"))
    assert lk.of("drive")[-1]["v"] == brain.V_FREE and not any("refutada" in m.get("msg", "") for m in lk.of("think"))
    print("ok  OBSTACULO + 86 cm validos e frescos: visao refutada (sem avoid, v_slow, think); 45/25 cm, velha, sem eco: fluxo normal")


def test_ws_link_tracks_center_reading():
    from tools.ws_link import WsLink
    lk = WsLink()
    lk._on_message(None, '{"t":"range","deg":-30.0,"cm":20.0}')
    assert lk.center is None
    lk._on_message(None, '{"t":"range","deg":0.0,"cm":86.4}')
    assert lk.center[0] == 86.4
    lk._on_message(None, '{"t":"range","deg":5.0,"cm":null}')
    assert lk.center[0] is None
    print("ok  WsLink guarda a leitura central (|deg| <= 10) com relogio monotonico")


def test_src_sensor_sends_vis_but_no_drive():
    lk, clk = _setup("explorar", "sensor", armed=True)
    brain.step(_info("OBSTACULO", sectors=(0, 0, 0, 0, 0)))
    assert lk.of("vis") and not lk.of("drive") and not lk.of("avoid")
    print("ok  src=sensor: so opiniao (vis); sem drive nem avoid do celular")


def test_vis_rate_and_ok_flag():
    lk, clk = _setup("explorar", "visao", armed=True)
    for _ in range(10):
        brain.step(_info())
        clk.adv(0.03)
    n = len(lk.of("vis"))
    assert 2 <= n <= 4, n                          # ~10 Hz em 0,3 s
    assert lk.of("vis")[0] == {"t": "vis", "ok": 1, "s": [100] * 5}
    clk.adv(0.2)
    brain.step(_info(ok=False, vs="-"))
    assert lk.of("vis")[-1] == {"t": "vis", "ok": 0, "s": [0] * 5}
    print("ok  vis ~10 Hz; ok=0 em calibracao/erro")


def test_yaw_rate_and_message():
    lk, clk = _setup("explorar", "sensor", armed=True)
    for _ in range(20):                            # 50 Hz por 0,4 s
        brain.on_gyro(10.0)
        clk.adv(0.02)
    ys = lk.of("yaw")
    assert 6 <= len(ys) <= 10, len(ys)             # ~20 Hz
    assert ys[-1]["psi"] > 0 and set(ys[-1]) == {"t", "psi", "r"}
    print(f"ok  yaw ~20 Hz ({len(ys)} msgs em 0,4 s), + = direita")


def _spin(clk, rate, secs, fps=20):
    """Giroscopio a 50 Hz intercalado com quadros da visao (brain.step) a `fps`."""
    n, every = int(secs / 0.02), max(1, int(round(50 / fps)))
    for i in range(n):
        brain.on_gyro(rate)
        if i % every == 0:
            brain.step(_info())
        clk.adv(0.02)


def test_diag_armed_without_goal_integrates_psi():
    # log de 21/11: armado pelo diagnostico (goal "nenhuma") o brain zerava o psi A CADA QUADRO -> psi ~0 a 300 graus/s
    lk, clk = _setup("nenhuma", None, armed=True)
    lk.state.update(L=0.6, R=-0.6)                 # rodas girando (sem estimar vies)
    _spin(clk, 300.0, 1.0)
    ys = lk.of("yaw")
    psi = ys[-1]["psi"] - ys[0]["psi"]
    assert 250 < psi < 320, psi
    print(f"ok  diagnostico (armado, goal nenhuma): psi integra entre quadros ({psi:.0f} graus em 1 s a 300 graus/s)")


def test_psi_resets_once_when_goal_ends():
    lk, clk = _setup("explorar", "sensor", armed=True)
    lk.state.update(L=0.35, R=0.35)
    _spin(clk, 50.0, 0.5)
    assert brain._yaw.psi > 15
    lk.goal, lk.src = "nenhuma", None              # motivacao acabou, ainda armado
    _spin(clk, 0.0, 0.05)
    assert abs(brain._yaw.psi) < 1.0, brain._yaw.psi
    _spin(clk, 100.0, 0.5)
    assert brain._yaw.psi > 30, brain._yaw.psi     # depois da transicao volta a integrar
    lk.state["armed"] = False
    _spin(clk, 100.0, 0.2)
    assert brain._yaw.psi == 0.0
    print("ok  psi zera uma vez quando a motivacao acaba e sempre que desarmado")


def test_every_message_fits_160_bytes():
    lk, clk = _setup("explorar", "visao", armed=True)
    brain.step(_info("OBSTACULO", r=0.123456, why="x" * 500), msg="y" * 400)
    clk.adv(0.5)
    brain.step(_info("LIVRE"), msg="á" * 400)
    brain.on_gyro(-123.456)
    clk.adv(0.1)
    brain.on_gyro(-123.456)
    brain._send({"t": "think", "viz": "http://192.168.43.123:8080/stream"})
    assert lk.sent
    worst = max(len(json.dumps(m, separators=(",", ":"), ensure_ascii=False).encode("utf-8")) for m in lk.sent)
    assert worst <= 160 and brain._oversize == 0, (worst, brain._oversize)
    big = {"t": "vis", "ok": 1, "s": [100] * 5, "pad": "z" * 300}
    assert brain._send(big) is False and brain._oversize == 1      # fora do limite e descartada
    print(f"ok  toda mensagem enviada cabe em 160 bytes (maior: {worst})")


def test_think_only_narrates_vision():
    lk, clk = _setup("explorar", "sensor", armed=True)
    brain.step(_info("LIVRE", why="visao OBSTACULO sem refutacao"))
    th = [m for m in lk.of("think") if m.get("state")]
    assert th and th[-1]["state"] == "LIVRE" and "setores" in th[-1]["msg"]
    assert "OBSTACULO" not in th[-1]["msg"]
    print("ok  think narra so a visao (estado, conf, setores)")


def test_yaw_integrator_unbounded_and_bias():
    yi = YawIntegrator(HeadingConfig())
    t = 0.0
    for _ in range(100):                           # parado 2 s com vies +0,8 dps
        yi.feed_rate(0.8, t, still=True)
        t += 0.02
    assert abs(yi.bias - 0.8) < 0.3, yi.bias
    yi.reset()
    for _ in range(100):                           # girando 90 dps por 2 s = 180 graus (> psi_max 20 do HeadingHold)
        yi.feed_rate(90.0 + yi.bias, t, still=False)
        t += 0.02
    psi = yi.psi
    assert 160 < psi < 185, psi
    assert yi.rate > 80
    yi.reset(full=True)
    assert yi.psi == 0.0 and yi.bias == 0.0
    print(f"ok  YawIntegrator: psi sem limite ({psi:.0f} graus), vies estimado parado")
