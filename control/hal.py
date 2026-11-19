"""hal.py - camada de abstracao de control: sensor de distancia + servo, enlace com o ESP32.

Aqui ficam Drive/Reading, as interfaces e os adaptadores reais (Serial/Memory/LinkRangeHead).
Os MOCKS (sensor/servo/enlace simulados e Scenario) estao em control/mocks.py (so teste/replay).

Convencao de angulo: graus, 0 = centro, negativo = esquerda, positivo = direita.

PROTOCOLO celular <-> ESP32 (resumo; completo em docs/protocolo_ws.md; JSON por linha, '\n'):
  celular -> ESP32:  {"t":"aim","deg":-45}              aponta o servo
                     {"t":"drive","v":0.4,"s":0.0}      v em [-1,1] (re < 0); s em [-1,1] (- esq, + dir)
                     {"t":"cfg","hard_stop_cm":20,"wd_ms":500}
  ESP32 -> celular:  {"t":"range","deg":0,"cm":123.4}   cm = null se nao houve eco
REFLEXO no ESP32 (independente do celular/Wi-Fi): parar se range < hard_stop_cm;
parar se nenhum 'drive' chegar em wd_ms (watchdog).
"""
import json
from dataclasses import dataclass
from typing import Optional


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