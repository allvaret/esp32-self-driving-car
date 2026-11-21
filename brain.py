"""brain.py - cola entre a visao (StreamRunner) e o ESP32 via WebSocket. UNICO dono da conexao.

Caminho 1: o ESP32 e o UNICO executor de manobras. O celular NAO tem maquina de estados: so
  - narra a OPINIAO da visao ao ESP32: {"t":"vis","ok":0|1,"s":[5 ints]}  (~10 Hz; s = setores livres
    0..100 em -30,-15,0,+15,+30 graus; ok=0 = sem opiniao: calibrando/erro)
  - manda o rumo do giroscopio:          {"t":"yaw","psi":graus,"r":graus/s}  (~20 Hz; + = direita;
    psi = integrador SEM limite, so o delta importa; r = giro filtrado sem vies)
  - em src=visao: {"t":"drive","v","s"} a cada frame (v pela visao, s = 0: reta) e, com
    OBSTACULO, {"t":"avoid","deg":<setor mais livre>}; o ESP32 faz a manobra.
  - o RUMO em reta e corrigido NO ESP32 (hold_*/trim aprendido, nos dois modos) com o yaw que o
    celular manda; o brain nao corrige mais o s (evita correcao dupla).
  - think: so narra a VISAO (estado, conf, setores). As decisoes aparecem no "state" do ESP32.

Uso (Kotlin): start() no onResume; step(json) por frame; on_gyro(rate) do giroscopio; stop() no onPause;
pop_calibrate() por frame; status() para a faixa do app.

Motivacao: o visor manda {"t":"goal","g":"explorar","src":"sensor"|"visao"} (o ESP32 repassa).
  src=sensor -> mode auto  (o ESP32 anda so com o sensor; sem drive do celular)
  src=visao  -> mode phone (visao + rumo; reflexos e manobras no ESP32)
  src ausente -> sensor (o mais seguro). O brain reage a mudanca de goal OU de src.
  O brain NUNCA arma o carro (quem arma e o ESP32 ao receber o goal, ou o botao BOOT);
  na reconexao do WS ele so reenvia o mode do src atual.
  goal "parar"/"nenhuma"/None, ou ESP32 desarmado: zera goal/src lembrados e o integrador de yaw.
Parametros abaixo [DEMO]; filtro/vies do giroscopio em config.HeadingConfig; correcao de rumo em
config.ControlConfig.hold_* (espelho dos Params do firmware).
"""
import json
import threading
import time

import viz
from config import Config
from control.heading import YawIntegrator
from tools.flight_log import FlightLog
from tools.ws_link import WsLink

# ----------------------------- parametros [DEMO] -----------------------------
URL = "ws://192.168.4.1:81/"
V_FREE = 0.35        # velocidade em LIVRE (normalizada 0..1, o ESP32 aplica pwm_min/pwm_max)
V_SLOW = 0.15        # velocidade em INCERTO
THINK_MIN_S = 0.5    # no maximo 2 pensamentos/s se o estado nao mudar
GOALS_MOVE = ("explorar",)
SRC_TO_MODE = {"sensor": "auto", "visao": "phone"}
VIS_EVERY_S = 0.1    # opiniao da visao ao ESP32 (~10 Hz)
YAW_EVERY_S = 0.05   # rumo ao ESP32 (~20 Hz)
AVOID_EVERY_S = 2.0  # OBSTACULO em src=visao: pede desvio no maximo a cada 2 s
VIZ_ANNOUNCE_S = 3.0  # reanuncia o endereco do stream (visor que conecta depois tambem recebe)
MAX_MSG = 160        # lineBuf do firmware: nenhuma mensagem pode passar disso (bytes)
SECTOR_DEGS = (-30, -15, 0, 15, 30)
AVOID_DEGS = (-30, -15, 15, 30)   # o ESP32 varre estas posicoes (centro e o obstaculo)

_STATES = ("LIVRE", "INCERTO", "OBSTACULO")

# ----------------------------------- estado ----------------------------------
_clock = time.monotonic          # trocado nos testes
_link = None
_lock = threading.Lock()
_last_key = None                 # (goal, src) ja tratado
_last_conn_seq = 0
_last_state = None
_armed_prev = False
_last_think_t = _last_viz_t = _last_vis_t = _last_yaw_t = _last_avoid_t = 0.0
_calib_seen = 0
_last_vis = {"ok": False, "s": None, "state": None, "conf": None}
_oversize = 0                    # mensagens descartadas por passarem de MAX_MSG (deve ficar em 0)
_yaw = YawIntegrator(Config().heading)        # psi sem limite para o ESP32 (sempre): curvas e rumo em reta
_logger = None                                # log local JSONL (tools/flight_log.py)
_log_dir = None


def _reset_motion():
    _yaw.reset()


def set_log_dir(path):
    """Kotlin, antes de start(): pasta do log local (filesDir/logs). viz.py serve os arquivos em /logs."""
    global _log_dir
    _log_dir = str(path) if path else None
    viz.set_log_dir(_log_dir)


def start(url=URL):
    """Abre a conexao (thread propria com reconexao). Pode chamar mais de uma vez."""
    global _link, _calib_seen, _last_key, _last_conn_seq, _logger
    with _lock:
        if _logger is None and _log_dir:
            _logger = FlightLog(_log_dir).start()
        if _link is None:
            _link = WsLink(url, on_msg=_on_msg).start()
            _calib_seen = 0
            _last_key = None
            _last_conn_seq = 0
            _reset_motion()
        if viz is not None:
            viz.start()
    return True


def stop():
    """Para o carro e fecha a conexao (e o log)."""
    global _link, _last_key, _logger
    with _lock:
        if _link is not None:
            _link.drive(0, 0)
            _link.arm(False)
            time.sleep(0.1)
            _link.stop()
            _link = None
        if viz is not None:
            viz.stop()
        _last_key = None
        _reset_motion()
        lg, _logger = _logger, None
    if lg is not None:
        lg.close()


def _on_msg(m):
    """Toda mensagem do ESP32 (thread do WsLink). Gancho do log local."""
    lg = _logger
    if lg is not None:
        lg.recv(m)


def _send(obj):
    """Envia compacto; garante <= MAX_MSG bytes (corta o texto de think; o resto e descartado)."""
    global _oversize
    link = _link
    if link is None:
        return False
    s = json.dumps(obj, separators=(",", ":"), ensure_ascii=False)
    if len(s.encode("utf-8")) > MAX_MSG:
        if obj.get("t") == "think" and isinstance(obj.get("msg"), str):
            o = dict(obj)
            while len(json.dumps(o, separators=(",", ":"), ensure_ascii=False).encode("utf-8")) > MAX_MSG and o["msg"]:
                o["msg"] = o["msg"][:-4] if len(o["msg"]) > 4 else ""
            obj = o
        else:
            _oversize += 1
            return False
    ok = link.send(obj)
    lg = _logger
    if lg is not None:
        lg.sent(obj)
    return ok


def on_gyro(rate_dps):
    """Kotlin, a cada amostra do giroscopio (~50 Hz): giro em graus/s, > 0 = para a DIREITA
    (ja projetado no eixo vertical). Alimenta o integrador; manda {"t":"yaw"} a ~20 Hz."""
    global _last_yaw_t
    now = _clock()
    r = float(rate_dps)
    link = _link
    esp = (link.state if link is not None else None) or {}
    armed = bool(esp.get("armed"))
    still = (not armed) or (abs(float(esp.get("L", 1.0))) < 0.02 and abs(float(esp.get("R", 1.0))) < 0.02)
    if not armed:
        _yaw.reset()                       # sem rumo de referencia enquanto desarmado (mantem o vies)
    _yaw.feed_rate(r, now, still)
    if link is not None and link.connected and now - _last_yaw_t >= YAW_EVERY_S:
        _last_yaw_t = now
        _send({"t": "yaw", "psi": round(_yaw.psi, 1), "r": round(_yaw.rate, 1)})


def pop_calibrate():
    """Kotlin, a cada frame: True UMA vez por comando "calibrar" vindo do visor.
    O app entao chama StreamRunner.recalibrate(); enquanto calibra, info.ok = False (o ESP32 ignora)."""
    global _calib_seen
    with _lock:
        link = _link
        if link is None or link.calib_seq == _calib_seen:
            return False
        _calib_seen = link.calib_seq
        _send({"t": "think", "msg": "calibrar: aprendendo a cor do chao"})
        return True


# --------------------------- leitura EXPLICITA da visao ---------------------------
def _fold(txt):
    return (str(txt).strip().upper().replace("Á", "A").replace("Ã", "A").replace("É", "E")
            .replace("Ó", "O").replace("Ê", "E"))


def _parse(estado):
    """JSON do process_rgba_frame (str/dict) ou estado puro ("LIVRE") -> dict normalizado:
    {"state": LIVRE|INCERTO|OBSTACULO|None, "conf": 0..1|None, "ok": bool, "sectors": [5 ints]|None}.
    NUNCA procura palavras dentro de textos livres (why etc.): so o campo vision_state (igualdade exata)."""
    d = estado
    if isinstance(d, str) and d.lstrip().startswith("{"):
        try:
            d = json.loads(d)
        except ValueError:
            d = None
    st = conf = sectors = None
    ok = False
    if isinstance(d, dict):
        raw = d.get("vision_state")
        st = _fold(raw) if isinstance(raw, str) else None
        r = d.get("r")
        if isinstance(r, (int, float)) and not isinstance(r, bool):
            conf = max(0.0, min(1.0, float(r)))
        sec = d.get("sectors")
        if isinstance(sec, (list, tuple)) and len(sec) == 5 and all(isinstance(x, (int, float)) for x in sec):
            sectors = [int(max(0, min(100, round(x)))) for x in sec]
        ok = bool(d.get("ok")) if "ok" in d else d.get("phase") == "RUN"
    elif isinstance(d, str):
        st, ok = _fold(d), True
    if st not in _STATES:
        st, ok = None, False
    return {"state": st, "conf": conf, "ok": ok, "sectors": sectors, "raw": d if isinstance(d, dict) else None}


def _norm_state(estado):
    """Compat: so o estado (LIVRE/INCERTO/OBSTACULO ou None)."""
    return _parse(estado)["state"]


def _free_sector_deg(sectors):
    """Angulo (entre os que o ESP32 varre) do setor mais livre; empate -> menor |angulo|, depois esquerda."""
    best = None
    for deg in AVOID_DEGS:
        v = sectors[SECTOR_DEGS.index(deg)]
        key = (-v, abs(deg), deg)
        if best is None or key < best[0]:
            best = (key, deg)
    return best[1]


def _resolve(link):
    """(goal, src) efetivos: so 'explorar' tem src; ausente -> 'sensor'."""
    goal = link.goal
    if goal in GOALS_MOVE:
        src = link.src if link.src in SRC_TO_MODE else "sensor"
        return goal, src
    return goal, None


def step(estado, conf=None, msg=None):
    """Chamar a cada frame (depois da visao). Retorna (v, s) enviado em src=visao, senao (0, 0)."""
    global _armed_prev, _last_key, _last_conn_seq, _last_state, _last_think_t, _last_viz_t, _last_vis_t, _last_avoid_t
    with _lock:
        link = _link
        if link is None or not link.connected:
            return (0.0, 0.0)
        now = _clock()
        p = _parse(estado)
        st, sectors = p["state"], p["sectors"]
        if conf is None:
            conf = p["conf"]
        esp = link.state or {}
        goal, src = _resolve(link)
        armed = bool(esp.get("armed"))
        _last_vis.update(ok=p["ok"], s=sectors, state=st, conf=conf)

        # parar / sem motivacao / desarmado: esquece goal/src e zera rumo (nao rearma nada)
        if goal not in GOALS_MOVE or not armed:
            if goal not in GOALS_MOVE or _armed_prev:
                _last_key = None
            _reset_motion()
        _armed_prev = armed

        # motivacao ou fonte nova -> so manda o MODE (quem arma e o ESP32)
        key = (goal, src)
        if goal in GOALS_MOVE and (key != _last_key or link.conn_seq != _last_conn_seq):
            _last_key, _last_conn_seq = key, link.conn_seq
            _send({"t": "mode", "m": SRC_TO_MODE[src]})
            _send({"t": "think", "msg": f"motivacao '{goal}' fonte '{src}'"})
        _last_conn_seq = link.conn_seq

        # opiniao da visao ao ESP32 (~10 Hz); ok=0 = sem opiniao
        if now - _last_vis_t >= VIS_EVERY_S:
            _last_vis_t = now
            use = p["ok"] and sectors is not None
            _send({"t": "vis", "ok": 1 if use else 0, "s": sectors if use else [0, 0, 0, 0, 0]})

        v = s = 0.0
        if goal in GOALS_MOVE and src == "visao":
            v = V_FREE if st == "LIVRE" else (V_SLOW if st == "INCERTO" else 0.0)
            s = 0.0                                # reta: o ESP32 corrige o rumo (hold_*), nao o celular
            link.drive(v, s)
            lg = _logger
            if lg is not None:
                lg.sent({"t": "drive", "v": round(v, 3), "s": round(s, 3)})
            if (st == "OBSTACULO" and armed and esp.get("st") != "DESVIO"
                    and now - _last_avoid_t >= AVOID_EVERY_S):
                _last_avoid_t = now
                o = {"t": "avoid"}
                if sectors is not None:
                    o["deg"] = _free_sector_deg(sectors)
                _send(o)

        if now - _last_viz_t >= VIZ_ANNOUNCE_S:
            _last_viz_t = now
            u = viz.url()
            if u:
                _send({"t": "think", "viz": u})

        # pensamento: SO a visao (estado, conf, setores)
        if st != _last_state or now - _last_think_t >= THINK_MIN_S:
            _last_state = st
            _last_think_t = now
            o = {"t": "think", "msg": msg or _vision_text(st, sectors, p["ok"]), "state": st or "-"}
            if conf is not None:
                try:
                    o["conf"] = round(float(conf), 2)
                except (TypeError, ValueError):
                    pass
            _send(o)
        lg = _logger
        if lg is not None:
            lg.frame(p, goal, src, v, s)
        return (v, s)


def _vision_text(st, sectors, ok):
    if not ok:
        return "visao sem opiniao (calibrando)"
    sec = " ".join(str(x) for x in sectors) if sectors else "-"
    return f"visao {st}  setores(-30..+30) {sec}"


def status():
    """Texto curto para a tela do app."""
    link = _link
    if link is None:
        return "brain: parado"
    if not link.connected:
        return "brain: sem conexao com o carro (rede CARRINHO? dados moveis desligados?)"
    stt = link.state or {}
    goal, src = _resolve(link)
    hold = {0: "fora de reta", 1: "so trim", 2: "corrigindo"}.get(stt.get("hold"), "-")
    rumo = (f" | rumo(ESP): {hold} corr={float(stt.get('hc') or 0):+.2f} trim={float(stt.get('trim') or 0):+.2f}"
            f" giro={_yaw.rate:+.0f}°/s")
    vs = _last_vis
    return (f"conectado | goal={goal} fonte={src} | armado={stt.get('armed')} | "
            f"frente={stt.get('c')} cm | visao={vs['state']} ok={vs['ok']} | "
            f"psi={_yaw.psi:+.0f}°{rumo}")
