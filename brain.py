"""brain.py - cola entre a visao (StreamRunner) e o ESP32 via WebSocket.

Uso (Kotlin, LiveActivity):
    onCreate  : py.getModule("brain").callAttr("start")
    por frame : py.getModule("brain").callAttr("step", estado, conf, msg)   # depois da visao
    onPause   : py.getModule("brain").callAttr("stop")
    (opcional): py.getModule("brain").callAttr("status")  -> texto para mostrar na tela

Ou direto do Python (StreamRunner / replay_main):
    import brain; brain.start(); ...; brain.step("LIVRE", 0.8, "chao livre"); ...; brain.stop()

Logica:
    - motivacao (goal) vem do visor: "explorar"/"reto" -> anda; "parar"/"nenhuma"/None -> parado
    - ao chegar motivacao nova: mode phone + arm (UMA vez; arm repetido reinicia os 3 s)
    - LIVRE -> V_FREE, INCERTO -> V_SLOW, OBSTACULO/desconhecido -> 0
    - drive a cada frame (o ESP32 para sozinho se nao chegar nada em 500 ms)
    - think so quando o estado muda ou a cada THINK_MIN_S
v1: "reto" e "explorar" se comportam igual (andar reto ate obstaculo). Parametros abaixo.
"""
import json
import threading
import time
import viz
from tools.ws_link import WsLink

# ----------------------------- parametros [DEMO] -----------------------------
URL = "ws://192.168.4.1:81/"
V_FREE = 0.5       # velocidade em LIVRE (normalizada 0..1, o ESP32 aplica pwm_min/pwm_max)
V_SLOW = 0.3        # velocidade em INCERTO
THINK_MIN_S = 0.5    # no maximo 2 pensamentos/s se o estado nao mudar
GOALS_MOVE = ("explorar", "reto")
VIZ_ANNOUNCE_S = 3.0  # reanuncia o endereco do stream (visor que conecta depois tambem recebe)

# ----------------------------------- estado ----------------------------------
_link = None
_lock = threading.Lock()
_last_goal = None
_last_state = None
_last_think_t = 0.0
_last_viz_t = 0.0


def start(url=URL):
    """Abre a conexao (thread propria com reconexao). Pode chamar mais de uma vez."""
    global _link
    with _lock:
        if _link is None:
            _link = WsLink(url).start()
        if viz is not None:
            viz.start()
    return True


def stop():
    """Para o carro e fecha a conexao."""
    global _link, _last_goal
    with _lock:
        if _link is not None:
            _link.drive(0, 0)
            _link.arm(False)
            time.sleep(0.1)
            _link.stop()
            _link = None
        if viz is not None:
            viz.stop()
        _last_goal = None


_STATES = ("OBSTACULO", "INCERTO", "LIVRE")   # ordem: o mais conservador ganha
_KEYS_STATE = ("state", "estado", "decision", "decisao")
_KEYS_CONF = ("conf", "confidence", "prob_mean")


def _maybe_json(x):
    """String JSON (retorno do process_rgba_frame) -> dict; o resto passa igual."""
    if isinstance(x, str) and x.lstrip().startswith("{"):
        try:
            d = json.loads(x)
        except ValueError:
            return x
        if isinstance(d, dict):
            for k in ("decision", "decisao", "result"):     # aceita um nivel aninhado
                if isinstance(d.get(k), dict):
                    merged = dict(d)
                    merged.update(d[k])
                    return merged
            return d
    return x


def _get(obj, keys):
    for k in keys:
        if isinstance(obj, dict) and obj.get(k) is not None:
            return obj[k]
        v = getattr(obj, k, None)
        if v is not None and not callable(v):
            return v
    return None


def _norm_state(estado):
    """Aceita 'LIVRE', dict/objeto com state/estado/decision, ou qualquer coisa cujo texto
    contenha LIVRE/INCERTO/OBSTACULO (ex.: o retorno do StreamRunner)."""
    if estado is None:
        return None
    estado = _maybe_json(estado)
    if not isinstance(estado, str):
        inner = _get(estado, _KEYS_STATE)
        estado = inner if inner is not None else estado
    txt = str(estado).upper()
    for s in _STATES:
        if s in txt:
            return s
    return None


def step(estado, conf=None, msg=None):
    """Chamar a cada frame. Retorna (v, s) enviado, para debug."""
    global _last_goal, _last_state, _last_think_t, _last_viz_t
    with _lock:
        link = _link
        if link is None or not link.connected:
            return (0.0, 0.0)

        st = _norm_state(estado)
        src = _maybe_json(estado)
        if conf is None and src is not None and not isinstance(src, str):
            conf = _get(src, _KEYS_CONF)
        if msg is None and isinstance(src, dict):
            msg = src.get("why") or src.get("reason") or src.get("motivo")
        goal = link.goal

        # motivacao nova -> prepara o ESP32 (uma vez)
        if goal != _last_goal:
            _last_goal = goal
            if goal in GOALS_MOVE:
                link.mode("phone")
                link.arm(True)
                link.think(f"motivacao '{goal}': armando")
            else:
                link.think(f"motivacao '{goal}': parado")

        # decisao -> movimento
        if goal not in GOALS_MOVE:
            v = 0.0
        elif st == "LIVRE":
            v = V_FREE
        elif st == "INCERTO":
            v = V_SLOW
        else:                      # OBSTACULO ou desconhecido: conservador
            v = 0.0
        s = 0.0
        link.drive(v, s)

        now = time.time()
        if viz is not None and now - _last_viz_t >= VIZ_ANNOUNCE_S:
            _last_viz_t = now
            u = viz.url()
            if u:
                link.send({"t": "think", "viz": u})

        # pensamento (limitado)
        if st != _last_state or now - _last_think_t >= THINK_MIN_S:
            _last_state = st
            _last_think_t = now
            extra = {"state": st or "-"}
            if conf is not None:
                try:
                    extra["conf"] = round(float(conf), 2)
                except (TypeError, ValueError):
                    pass
            texto = msg or f"{st or '?'} -> v={v:.2f}"
            link.think(str(texto)[:120], **extra)
        return (v, s)


def status():
    """Texto curto para a tela do app."""
    link = _link
    if link is None:
        return "brain: parado"
    if not link.connected:
        return "brain: sem conexao com o carro (rede CARRINHO? dados moveis desligados?)"
    stt = link.state or {}
    return (f"conectado | goal={link.goal} | armado={stt.get('armed')} | "
            f"frente={stt.get('c')} cm")