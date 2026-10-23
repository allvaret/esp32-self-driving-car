"""control/controller.py - fusao visao + sensor e maquina de estados do V1 (3 estados).

  CRUISE : anda reto em v_cruise; sensor fica apontado ao centro (sentinela).
           (parado ate a 1a leitura assentada do sensor)
  SCAN   : algo estranho (visao INCERTO/OBSTACULO, borda proxima, sensor em zona
           intermediaria) -> anda devagar e deixa o sensor decidir.
  AVOID  : STOP -> olha esquerda/direita -> REVERSE curta -> TURN para o lado mais livre -> SCAN.
           sem saida: re maior e olha de novo; esgotadas as tentativas, HOLD ate liberar.
           WAIT: guarda de reentrada (ver abaixo).

Fusao (o sensor e a autoridade; a visao dispara e pode ESCALAR, nunca liberar sozinha):
  sensor perto (< d_stop_m)                                  -> AVOID
  visao OBSTACULO e sensor NAO refuta (sem eco / < d_clear_m) -> AVOID
  visao OBSTACULO mas sensor valido >= d_clear_m (refuta)     -> segue devagar (SCAN)
  visao INCERTO, borda proxima ou sensor em zona intermediaria -> SCAN
  caso contrario                                              -> CRUISE

Guarda de reentrada: se ja houve guard_max_episodes AVOIDs nos ultimos guard_window_s,
o proximo AVOID vira WAIT (parado) por guard_backoff_s[nivel]; o nivel sobe a cada
reentrada e zera depois de uma janela sem AVOID. Evita o loop re/curva/re/curva com
uma pessoa (ou obstaculo) que volta para a frente do carro.

Todos os tempos em SEGUNDOS (independem do fps). Parametros: config.py (todos [DEMO]).
V1.5 (fora do escopo): curva ANTES do obstaculo; deteccao de obstaculo movel pela
taxa de variacao da leitura; veto da visao para desnivel/escada.
"""
from dataclasses import dataclass
from enum import Enum

from config import ControlConfig
from control.hal import Drive
from vision.decision import State as VState

EPS = 1e-6
STOP = Drive(0.0, 0.0)


class CState(Enum):
    CRUISE = "CRUISE"
    SCAN = "SCAN"
    AVOID = "AVOID"


@dataclass(frozen=True)
class Vision:
    state: VState = VState.LIVRE
    edge_near: bool = False


def edge_is_near(h_cen, cov, cfg=None):
    cfg = cfg or ControlConfig()
    return cov >= cfg.edge_cov_min and h_cen <= cfg.edge_near_h


class Controller:
    def __init__(self, head, cfg=None):
        self.head = head
        self.cfg = cfg or ControlConfig()
        self.state, self.sub, self.why = CState.CRUISE, "", ""
        self._angle = None
        self._c = None                 # ultima distancia central assentada (None = sem eco/sem info)
        self._have_c = False           # ja houve alguma leitura assentada?
        self._clear_since = None       # desde quando nao ha gatilho (SCAN) / condicao ok (HOLD)
        self._t_sub = 0.0
        self._scores = {}
        self._side = 0
        self._tries = 0
        self._rev_s = self.cfg.reverse_s
        self._avoid_log = []           # instantes de entrada em AVOID (guarda de reentrada)
        self._guard_level = 0
        self._wait_until = 0.0

    # ---- utilidades --------------------------------------------------------
    def _aim(self, angle, t):
        if self._angle != angle:
            self.head.aim(angle, t)
            self._angle = angle

    def _read_center(self, t):
        self._aim(0, t)
        r = self.head.read(t)
        if r.settled:
            self._c, self._have_c = r.dist_m, True
        return self._c

    def _go(self, sub, t):
        self.sub, self._t_sub = sub, t

    def _enter_avoid(self, t, why):
        c = self.cfg
        self._avoid_log = [x for x in self._avoid_log if t - x <= c.guard_window_s]
        if not self._avoid_log:
            self._guard_level = 0                       # janela tranquila: zera o backoff
        self.state, self._tries = CState.AVOID, 0
        if len(self._avoid_log) >= c.guard_max_episodes:
            wait = c.guard_backoff_s[min(self._guard_level, len(c.guard_backoff_s) - 1)]
            self._guard_level += 1
            self._wait_until = t + wait
            self.why = (f"reentrada ({len(self._avoid_log) + 1}o AVOID em "
                        f"{c.guard_window_s:g}s): espera {wait:g}s")
            self._go("WAIT", t)
        else:
            self.why = why
            self._go("STOP", t)
        self._avoid_log.append(t)
        return STOP

    # ---- passo principal ---------------------------------------------------
    def update(self, t, vision):
        drive = (self._avoid(t, vision) if self.state == CState.AVOID
                 else self._cruise_or_scan(t, vision))
        info = {"ctrl_state": self.state.value, "sub": self.sub, "angle": self._angle,
                "sensor_m": self._c, "v": drive.v, "steer": drive.steer, "why": self.why,
                "guard_level": self._guard_level}
        return drive, info

    def _cruise_or_scan(self, t, vision):
        c = self.cfg
        d = self._read_center(t)
        if not self._have_c:                       # nao anda antes da 1a leitura do sensor
            self.why = "aguardando sensor"
            return STOP
        if d is not None and d < c.d_stop_m:
            return self._enter_avoid(t, "sensor perto")
        refuted = d is not None and d >= c.d_clear_m
        vis_block = vision.state == VState.OBSTACULO
        if vis_block and not refuted:
            return self._enter_avoid(t, "visao OBSTACULO sem refutacao")

        mid_zone = d is not None and d < c.d_clear_m
        trigger = (vision.state == VState.INCERTO or vision.edge_near
                   or vis_block or mid_zone)
        if trigger:
            self.state, self._clear_since = CState.SCAN, None
            self.why = ("visao OBSTACULO refutada" if vis_block else
                        "borda proxima" if vision.edge_near else
                        "zona intermediaria" if mid_zone else "visao INCERTO")
            return Drive(c.v_slow, 0.0)
        if self.state == CState.SCAN:
            if self._clear_since is None:
                self._clear_since = t
            if t - self._clear_since >= c.clear_s - EPS:
                self.state, self.why, self._clear_since = CState.CRUISE, "", None
                return Drive(c.v_cruise, 0.0)
            return Drive(c.v_slow, 0.0)
        return Drive(c.v_cruise, 0.0)

    # ---- AVOID: parar -> olhar L/R -> re -> curva -> SCAN -------------------
    def _avoid(self, t, vision):
        c = self.cfg
        el, s = t - self._t_sub, self.sub          # el = tempo decorrido no subestado atual
        if s == "WAIT":
            self._read_center(t)                   # mantem a telemetria viva
            if t >= self._wait_until - EPS:
                self.state, self.sub, self.why, self._clear_since = CState.SCAN, "", "fim da espera", None
                return Drive(c.v_slow, 0.0)
            return STOP
        if s == "STOP":
            if el >= c.stop_s - EPS:
                self._scores = {}
                self._aim(-c.scan_deg, t)
                self._go("LOOK_L", t)
            return STOP
        if s in ("LOOK_L", "LOOK_R"):
            r = self.head.read(t)
            if r.settled or el >= c.look_timeout_s - EPS:
                d = r.dist_m if r.settled else None
                self._scores[s[-1]] = d if d is not None else c.none_score_m
                if s == "LOOK_L":
                    self._aim(+c.scan_deg, t)
                    self._go("LOOK_R", t)
                else:
                    self._decide(t)
            return STOP
        if s == "REVERSE":
            if el >= self._rev_s - EPS:
                if self._side == 0:                       # sem saida: olha de novo
                    self._scores = {}
                    self._aim(-c.scan_deg, t)
                    self._go("LOOK_L", t)
                else:
                    self._go("TURN", t)
            return Drive(-c.v_rev, 0.0)
        if s == "TURN":
            if el >= c.turn_s - EPS:
                self.state, self.sub, self.why, self._clear_since = CState.SCAN, "", "pos-desvio", None
                return Drive(c.v_slow, 0.0)
            return Drive(c.v_turn, float(self._side))
        # HOLD: preso; so sai quando a visao nao bloqueia e o sensor nao contradiz
        cd = self._read_center(t)
        ok = vision.state != VState.OBSTACULO and (cd is None or cd >= c.d_clear_m)
        if not ok:
            self._clear_since = None
        else:
            if self._clear_since is None:
                self._clear_since = t
            if t - self._clear_since >= c.clear_s - EPS:
                self.state, self.sub, self.why, self._clear_since = CState.SCAN, "", "liberado", None
        return STOP

    def _decide(self, t):
        """Escolhe o lado. score = distancia lida (ou none_score_m se sem eco);
        -1 = esquerda, +1 = direita; empate -> esquerda."""
        c = self.cfg
        self._aim(0, t)
        sL, sR = self._scores["L"], self._scores["R"]
        if max(sL, sR) < c.d_stop_m:                      # nenhum lado livre
            self._tries += 1
            if self._tries > c.avoid_retries:
                self.why, self._clear_since = "sem saida", None
                self._go("HOLD", t)
                return
            self._side, self._rev_s = 0, c.reverse_s * 1.5
        else:
            self._side, self._rev_s = (-1 if sL >= sR else +1), c.reverse_s
        self._go("REVERSE", t)