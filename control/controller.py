"""control/controller.py - CONGELADO: referencia v1, NAO usado ao vivo.

Arquitetura "ESP32 executor": o ESP32 (car_firmware.ino) e o UNICO executor de manobras; o S9 manda so a
opiniao da visao e o rumo do giroscopio (brain.py). Este arquivo fica como referencia da logica
e e usado apenas por replay/, tools/bench_s9.py e testes. Nao importe no caminho ao vivo.

fusao visao + sensor e maquina de estados do V1 (3 estados).

  FRENTE    : anda reto em v_cruise; sensor fica apontado ao centro (sentinela).
              (parado ate a 1a leitura assentada do sensor)
  VARREDURA : algo estranho (visao INCERTO/OBSTACULO, borda proxima, sensor em zona
              intermediaria) -> anda devagar e deixa o sensor decidir.
  DESVIO    : PARADA -> olha esquerda/direita -> RE curta -> CURVA para o lado mais livre -> VARREDURA.
              sem saida: re maior e olha de novo; esgotadas as tentativas, PRESO ate liberar.
              ESPERA: guarda de reentrada (ver abaixo).

Fusao (o sensor e a autoridade; a visao dispara e pode ESCALAR, nunca liberar sozinha):
  sensor perto (< d_stop_m)                                  -> DESVIO
  visao OBSTACULO e sensor NAO refuta (sem eco / < d_clear_m) -> DESVIO
  visao OBSTACULO mas sensor valido >= d_clear_m (refuta)     -> segue devagar (VARREDURA)
  visao INCERTO, borda proxima ou sensor em zona intermediaria -> VARREDURA
  caso contrario                                              -> FRENTE

Guarda de reentrada: se ja houve guard_max_episodes AVOIDs nos ultimos guard_window_s,
o proximo DESVIO vira ESPERA (parado) por guard_backoff_s[nivel]; o nivel sobe a cada
reentrada e zera depois de uma janela sem DESVIO. Evita o loop re/curva/re/curva com
uma pessoa (ou obstaculo) que volta para a frente do carro.

Todos os tempos em SEGUNDOS (independem do fps). Parametros: config.py (valores iniciais).
V1.5 (fora do escopo): curva ANTES do obstaculo; deteccao de obstaculo movel pela
taxa de variacao da leitura; veto da visao para desnivel/escada.
"""
from dataclasses import dataclass
from enum import Enum

from config import ControlConfig
from control.hal import Drive
from vision.decision import State as VState
from vision.edge_probe import edge_is_near  # noqa: F401  (reexportado para replay/bench/testes)

EPS = 1e-6
STOP = Drive(0.0, 0.0)


class CState(Enum):
    FRENTE = "FRENTE"
    VARREDURA = "VARREDURA"
    DESVIO = "DESVIO"


@dataclass(frozen=True)
class Vision:
    state: VState = VState.LIVRE
    edge_near: bool = False


class Controller:
    def __init__(self, head, cfg=None):
        self.head = head
        self.cfg = cfg or ControlConfig()
        self.state, self.sub, self.why = CState.FRENTE, "", ""
        self._angle = None
        self._c = None                 # ultima distancia central assentada (None = sem eco/sem info)
        self._have_c = False           # ja houve alguma leitura assentada?
        self._clear_since = None       # desde quando nao ha gatilho (VARREDURA) / condicao ok (PRESO)
        self._t_sub = 0.0
        self._scores = {}
        self._side = 0
        self._tries = 0
        self._rev_s = self.cfg.reverse_s
        self._avoid_log = []           # instantes de entrada em DESVIO (guarda de reentrada)
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
        self.state, self._tries = CState.DESVIO, 0
        if len(self._avoid_log) >= c.guard_max_episodes:
            wait = c.guard_backoff_s[min(self._guard_level, len(c.guard_backoff_s) - 1)]
            self._guard_level += 1
            self._wait_until = t + wait
            self.why = (f"reentrada ({len(self._avoid_log) + 1}o DESVIO em "
                        f"{c.guard_window_s:g}s): espera {wait:g}s")
            self._go("ESPERA", t)
        else:
            self.why = why
            self._go("PARADA", t)
        self._avoid_log.append(t)
        return STOP

    # ---- passo principal ---------------------------------------------------
    def update(self, t, vision):
        drive = (self._avoid(t, vision) if self.state == CState.DESVIO
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
            self.state, self._clear_since = CState.VARREDURA, None
            self.why = ("visao OBSTACULO refutada" if vis_block else
                        "borda proxima" if vision.edge_near else
                        "zona intermediaria" if mid_zone else "visao INCERTO")
            return Drive(c.v_slow, 0.0)
        if self.state == CState.VARREDURA:
            if self._clear_since is None:
                self._clear_since = t
            if t - self._clear_since >= c.clear_s - EPS:
                self.state, self.why, self._clear_since = CState.FRENTE, "", None
                return Drive(c.v_cruise, 0.0)
            return Drive(c.v_slow, 0.0)
        return Drive(c.v_cruise, 0.0)

    # ---- DESVIO: parar -> olhar L/R -> re -> curva -> VARREDURA -------------------
    def _avoid(self, t, vision):
        c = self.cfg
        el, s = t - self._t_sub, self.sub          # el = tempo decorrido no subestado atual
        if s == "ESPERA":
            self._read_center(t)                   # mantem a telemetria viva
            if t >= self._wait_until - EPS:
                self.state, self.sub, self.why, self._clear_since = CState.VARREDURA, "", "fim da espera", None
                return Drive(c.v_slow, 0.0)
            return STOP
        if s == "PARADA":
            if el >= c.stop_s - EPS:
                self._scores = {}
                self._aim(-c.scan_deg, t)
                self._go("OLHA_E", t)
            return STOP
        if s in ("OLHA_E", "OLHA_D"):
            r = self.head.read(t)
            if r.settled or el >= c.look_timeout_s - EPS:
                d = r.dist_m if r.settled else None
                self._scores[s[-1]] = d if d is not None else c.none_score_m
                if s == "OLHA_E":
                    self._aim(+c.scan_deg, t)
                    self._go("OLHA_D", t)
                else:
                    self._decide(t)
            return STOP
        if s == "RE":
            if el >= self._rev_s - EPS:
                if self._side == 0:                       # sem saida: olha de novo
                    self._scores = {}
                    self._aim(-c.scan_deg, t)
                    self._go("OLHA_E", t)
                else:
                    self._go("CURVA", t)
            return Drive(-c.v_rev, 0.0)
        if s == "CURVA":
            if el >= c.turn_s - EPS:
                self.state, self.sub, self.why, self._clear_since = CState.VARREDURA, "", "pos-desvio", None
                return Drive(c.v_slow, 0.0)
            return Drive(c.v_turn, float(self._side))
        # PRESO: preso; so sai quando a visao nao bloqueia e o sensor nao contradiz
        cd = self._read_center(t)
        ok = vision.state != VState.OBSTACULO and (cd is None or cd >= c.d_clear_m)
        if not ok:
            self._clear_since = None
        else:
            if self._clear_since is None:
                self._clear_since = t
            if t - self._clear_since >= c.clear_s - EPS:
                self.state, self.sub, self.why, self._clear_since = CState.VARREDURA, "", "liberado", None
        return STOP

    def _decide(self, t):
        """Escolhe o lado. score = distancia lida (ou none_score_m se sem eco);
        -1 = esquerda, +1 = direita; empate -> esquerda."""
        c = self.cfg
        self._aim(0, t)
        sL, sR = self._scores["E"], self._scores["D"]
        if max(sL, sR) < c.d_stop_m:                      # nenhum lado livre
            self._tries += 1
            if self._tries > c.avoid_retries:
                self.why, self._clear_since = "sem saida", None
                self._go("PRESO", t)
                return
            self._side, self._rev_s = 0, c.reverse_s * 1.5
        else:
            self._side, self._rev_s = (-1 if sL >= sR else +1), c.reverse_s
        self._go("RE", t)