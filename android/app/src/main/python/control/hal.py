"""hal.py - camada de abstracao de control: sensor de distancia + servo, enlace com o ESP32.

A logica (controller.py) so conhece estas interfaces. Hoje usamos MOCKS; quando o
sensor/firmware existirem, troca-se a implementacao sem tocar no controller.

Convencao de angulo: graus, 0 = centro, negativo = esquerda, positivo = direita.

PROTOCOLO celular <-> ESP32 (PROPOSTA - ainda nao existe firmware; JSON por linha, '\\n'):
  celular -> ESP32:  {"t":"aim","deg":-45}              aponta o servo
                     {"t":"drive","v":0.4,"s":0.0}      v em [-1,1] (re < 0); s em [-1,1] (- esq, + dir)
                     {"t":"cfg","hard_stop_cm":20,"wd_ms":500}
  ESP32 -> celular:  {"t":"range","deg":0,"cm":123.4}   cm = null se nao houve eco
REFLEXO no ESP32 (independente do celular/Wi-Fi): parar se range < hard_stop_cm;
parar se nenhum 'drive' chegar em wd_ms (watchdog).
"""
import csv
import json
import random
from dataclasses import dataclass
from typing import Optional

import numpy as np

FAR_M = 99.0            # "sem obstaculo" nos cenarios
ANGLE_BIN = 15.0        # |angulo| <= isso -> coluna 'center' do cenario


@dataclass(frozen=True)
class Reading:
    t: float
    angle: float
    dist_m: Optional[float]     # None = sem eco / fora de alcance
    settled: bool = True        # False = servo ainda se movendo (leitura nao vale)


@dataclass(frozen=True)
class Drive:
    v: float = 0.0              # [-1, 1], re < 0
    steer: float = 0.0          # [-1, 1], negativo = esquerda


# ------------------------------------------------------------------ cenario ---
class Scenario:
    """Distancia 'verdadeira' (m) por direcao ao longo do tempo, interpolada linearmente.
    CSV: t,center_m,left_m,right_m   (celula vazia = sem obstaculo)."""

    def __init__(self, t, center, left, right):
        self.t = np.asarray(t, float)
        self.cols = {"c": np.asarray(center, float), "l": np.asarray(left, float),
                     "r": np.asarray(right, float)}

    @classmethod
    def constant(cls, center=FAR_M, left=FAR_M, right=FAR_M):
        return cls([0.0, 1e6], [center] * 2, [left] * 2, [right] * 2)

    @classmethod
    def from_csv(cls, path):
        rows = list(csv.DictReader(open(path)))
        val = lambda r, k: float(r[k]) if r[k].strip() else FAR_M
        return cls([float(r["t"]) for r in rows], [val(r, "center_m") for r in rows],
                   [val(r, "left_m") for r in rows], [val(r, "right_m") for r in rows])

    def at(self, t, angle_deg):
        k = "l" if angle_deg < -ANGLE_BIN else "r" if angle_deg > ANGLE_BIN else "c"
        return float(np.interp(t, self.t, self.cols[k]))


# --------------------------------------------------------------------- mocks ---
class MockRangeHead:
    """Servo + sensor simulados. Falhas injetaveis: faults = [(t0, t1, modo)]
       modo 'none' -> sem eco (ex.: sol forte, superficie macia)
       modo 'far'  -> devolve alcance maximo mesmo com objeto perto (ex.: perna fina de cadeira)"""

    def __init__(self, scenario, max_range_m=2.0, min_range_m=0.02, noise_m=0.01,
                 dropout_p=0.0, latency_s=0.0, settle_s=0.15, faults=(), seed=0):
        self.sc, self.max_r, self.min_r = scenario, max_range_m, min_range_m
        self.noise, self.dropout, self.latency, self.settle = noise_m, dropout_p, latency_s, settle_s
        self.faults, self.rng = list(faults), random.Random(seed)
        self.angle, self._t_aim = 0.0, -1e9

    def aim(self, angle_deg, t):
        self.angle, self._t_aim = float(angle_deg), t

    def read(self, t):
        if t - self._t_aim < self.settle:
            return Reading(t, self.angle, None, settled=False)
        d = self.sc.at(t - self.latency, self.angle)
        for t0, t1, mode in self.faults:
            if t0 <= t <= t1:
                d = None if mode == "none" else self.max_r
        if d is not None:
            d += self.rng.gauss(0.0, self.noise)
            if d > self.max_r or self.rng.random() < self.dropout:
                d = None
            else:
                d = max(d, self.min_r)
        return Reading(t, self.angle, d, settled=True)


class MockDriveLink:
    def __init__(self):
        self.log = []

    def send(self, t, drive):
        self.log.append((t, drive))


# --------------------------------------------------------- enlaces (reais) ---
class MemoryLink:
    """Enlace em memoria, para testes: 'push' simula mensagens vindas do ESP32."""

    def __init__(self):
        self.sent, self._inbox = [], []

    def send(self, msg):
        self.sent.append(msg)

    def poll(self):
        out, self._inbox = self._inbox, []
        return out

    def push(self, msg):
        self._inbox.append(msg)


class SerialLink:
    """USB serial (JSON por linha). port: 'COM3', '/dev/ttyUSB0' ou 'loop://' (teste).
    NAO testado em control real; testado apenas em loopback."""

    def __init__(self, port, baud=115200):
        import serial                                   # pip install pyserial
        self.ser = serial.serial_for_url(port, baudrate=baud, timeout=0)
        self._buf = b""

    def send(self, msg):
        self.ser.write((json.dumps(msg, separators=(",", ":")) + "\n").encode())

    def poll(self):
        self._buf += self.ser.read(self.ser.in_waiting or 0)
        msgs = []
        while b"\n" in self._buf:
            line, self._buf = self._buf.split(b"\n", 1)
            try:
                msgs.append(json.loads(line))
            except ValueError:
                pass                                    # linha truncada/ruido: ignora
        return msgs


class LinkRangeHead:
    """Servo + sensor que vivem no ESP32, acessados por um enlace (Serial/WebSocket/Memory)."""

    def __init__(self, link, settle_s=0.15):
        self.link, self.settle = link, settle_s
        self.angle, self._t_aim, self._last, self._got = 0.0, -1e9, None, False

    def aim(self, angle_deg, t):
        self.angle, self._t_aim, self._last, self._got = float(angle_deg), t, None, False
        self.link.send({"t": "aim", "deg": self.angle})

    def read(self, t):
        for m in self.link.poll():
            if m.get("t") == "range" and float(m.get("deg", 1e9)) == self.angle:
                cm = m.get("cm")
                self._last = None if cm is None else cm / 100.0
                self._got = True
        settled = (t - self._t_aim >= self.settle) and self._got
        return Reading(t, self.angle, self._last if settled else None, settled)


class LinkDrive:
    def __init__(self, link):
        self.link = link

    def send(self, t, drive):
        self.link.send({"t": "drive", "v": round(drive.v, 3), "s": round(drive.steer, 3)})