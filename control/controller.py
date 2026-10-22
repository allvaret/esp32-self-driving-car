"""controller.py - fusao visao + sensor e maquina de estados do V1 (3 estados).

  CRUISE : anda reto em V_CRUISE; sensor fica apontado ao centro (sentinela).
           (parado ate a 1a leitura assentada do sensor)
  SCAN   : algo estranho (visao INCERTO/OBSTACULO, borda proxima, sensor em zona
           intermediaria) -> anda devagar e deixa o sensor decidir.
  AVOID  : para -> olha esquerda/direita -> re curta -> curva para o lado mais livre -> SCAN.
           (sem saida: re maior e olha de novo; esgotadas as tentativas, HOLD ate liberar)

Fusao (o sensor e a autoridade; a visao dispara e pode ESCALAR, nunca liberar sozinha):
  sensor perto (< D_STOP)                                -> AVOID
  visao OBSTACULO e sensor NAO refuta (sem eco / < D_CLEAR) -> AVOID
  visao OBSTACULO mas sensor valido >= D_CLEAR (refuta)   -> segue devagar (SCAN)
  visao INCERTO, borda proxima ou sensor em zona intermediaria -> SCAN
  caso contrario                                          -> CRUISE

V1.5 (fora do escopo): curva ANTES do obstaculo (desvio antecipado, sem re).
Valores abaixo sao de DEMONSTRACAO: derive-os da velocidade/latencia reais do carro.
"""
from dataclasses import dataclass
from enum import Enum

from vision.decision import State as VState
from hardware.hal import Drive

# ------------------------------------------------------------------ parametros ---
D_STOP = 0.50           # m: sensor central mais perto que isso -> AVOID
D_CLEAR = 1.00          # m: leitura valida mais longe que isso "refuta" a visao
V_CRUISE, V_SLOW = 0.5, 0.2
V_REV, V_TURN = 0.3, 0.3
SCAN_DEG = 45           # angulo de olhada lateral
STOP_S = 0.3            # tempo parado antes de olhar
LOOK_TIMEOUT_S = 1.0    # se o servo nao assentar, segue com 'sem informacao'
REVERSE_S = 0.8
TURN_S = 1.0
AVOID_RETRIES = 2       # re maior + nova olhada antes de desistir (HOLD)
N_CLEAR = 3             # ticks seguidos "limpos" p/ voltar a CRUISE / sair de HOLD
NONE_SCORE_M = 0.5 * D_CLEAR   # lado sem eco vale mais que lado perto, menos que lado livre
EDGE_NEAR = 0.20        # h_cen <= isso (borda perto) ...
EDGE_COV_MIN = 0.50     # ... com cobertura >= isso -> edge_near (hipotese, nao validada)
# --------------------------------------------------------------------------------

STOP = Drive(0.0, 0.0)


class CState(Enum):
    CRUISE = "CRUISE"
    SCAN = "SCAN"
    AVOID = "AVOID"


@dataclass(frozen=True)
class Vision:
    state: VState = VState.LIVRE
    edge_near: bool = False


def edge_is_near(h_cen, cov):
    return cov >= EDGE_COV_MIN and h_cen <= EDGE_NEAR


class Controller:
    def __init__(self, head):
        self.head = head
        self.state, self.sub, self.why = CState.CRUISE, "", ""
        self._angle = None
        self._c = None                 # ultima distancia central assentada (None = sem eco/sem info)
        self._have_c = False           # ja houve alguma leitura assentada?
        self._clear_n = 0
        self._t_sub = 0.0
        self._scores = {}
        self._side = 0
        self._tries = 0
        self._rev_s = REVERSE_S

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
        self.state, self.why, self._tries = CState.AVOID, why, 0
        self._go("STOP", t)
        return STOP

    # ---- passo principal ---------------------------------------------------
    def update(self, t, vision):
        drive = (self._avoid(t, vision) if self.state == CState.AVOID
                 else self._cruise_or_scan(t, vision))
        info = {"ctrl_state": self.state.value, "sub": self.sub, "angle": self._angle,
                "sensor_m": self._c, "v": drive.v, "steer": drive.steer, "why": self.why}
        return drive, info

    def _cruise_or_scan(self, t, vision):
        d = self._read_center(t)
        if not self._have_c:                       # nao anda antes da 1a leitura do sensor
            self.why = "aguardando sensor"
            return STOP
        if d is not None and d < D_STOP:
            return self._enter_avoid(t, "sensor perto")
        refuted = d is not None and d >= D_CLEAR
        vis_block = vision.state == VState.OBSTACULO
        if vis_block and not refuted:
            return self._enter_avoid(t, "visao OBSTACULO sem refutacao")

        mid_zone = d is not None and d < D_CLEAR
        trigger = (vision.state == VState.INCERTO or vision.edge_near
                   or vis_block or mid_zone)
        if trigger:
            self.state, self._clear_n = CState.SCAN, 0
            self.why = ("visao OBSTACULO refutada" if vis_block else
                        "borda proxima" if vision.edge_near else
                        "zona intermediaria" if mid_zone else "visao INCERTO")
            return Drive(V_SLOW, 0.0)
        if self.state == CState.SCAN:
            self._clear_n += 1
            if self._clear_n >= N_CLEAR:
                self.state, self.why = CState.CRUISE, ""
                return Drive(V_CRUISE, 0.0)
            return Drive(V_SLOW, 0.0)
        return Drive(V_CRUISE, 0.0)

    # ---- AVOID: parar -> olhar L/R -> re -> curva -> SCAN -------------------
    def _avoid(self, t, vision):
        el, s = t - self._t_sub, self.sub
        if s == "STOP":
            if el >= STOP_S:
                self._scores = {}
                self._aim(-SCAN_DEG, t)
                self._go("LOOK_L", t)
            return STOP
        if s in ("LOOK_L", "LOOK_R"):
            r = self.head.read(t)
            if r.settled or el >= LOOK_TIMEOUT_S:
                d = r.dist_m if r.settled else None
                self._scores[s[-1]] = d if d is not None else NONE_SCORE_M
                if s == "LOOK_L":
                    self._aim(+SCAN_DEG, t)
                    self._go("LOOK_R", t)
                else:
                    self._decide(t)
            return STOP
        if s == "REVERSE":
            if el >= self._rev_s:
                if self._side == 0:                       # sem saida: olha de novo
                    self._scores = {}
                    self._aim(-SCAN_DEG, t)
                    self._go("LOOK_L", t)
                else:
                    self._go("TURN", t)
            return Drive(-V_REV, 0.0)
        if s == "TURN":
            if el >= TURN_S:
                self.state, self.sub, self.why, self._clear_n = CState.SCAN, "", "pos-desvio", 0
                return Drive(V_SLOW, 0.0)
            return Drive(V_TURN, float(self._side))
        # HOLD: preso; so sai quando a visao nao bloqueia e o sensor nao contradiz
        c = self._read_center(t)
        ok = vision.state != VState.OBSTACULO and (c is None or c >= D_CLEAR)
        self._clear_n = self._clear_n + 1 if ok else 0
        if self._clear_n >= N_CLEAR:
            self.state, self.sub, self.why, self._clear_n = CState.SCAN, "", "liberado", 0
        return STOP

    def _decide(self, t):
        self._aim(0, t)
        sL, sR = self._scores["L"], self._scores["R"]
        if max(sL, sR) < D_STOP:
            self._tries += 1
            if self._tries > AVOID_RETRIES:
                self.why = "sem saida"
                self._clear_n = 0
                self._go("HOLD", t)
                return
            self._side, self._rev_s = 0, REVERSE_S * 1.5
        else:
            self._side, self._rev_s = (-1 if sL >= sR else +1), REVERSE_S
        self._go("REVERSE", t)