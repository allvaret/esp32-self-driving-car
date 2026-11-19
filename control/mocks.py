"""control/mocks.py - sensor/servo/enlace SIMULADOS e Scenario (so testes, replay e bancada).

NAO e importado pelo caminho ao vivo (main.StreamRunner, brain). Ver tests/test_live_path.py.
Tipos compartilhados (Reading, Drive) e convencao de angulo: control/hal.py.
"""
import csv
import random

import numpy as np

from control.hal import Reading

FAR_M = 99.0            # "sem obstaculo" nos cenarios
ANGLE_BIN = 15.0        # |angulo| <= isso -> coluna 'center' do cenario

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
