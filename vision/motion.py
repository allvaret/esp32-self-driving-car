"""vision/motion.py - "a camera esta se mexendo?" (indice de movimento para o ESP32 detectar BATIDA, firmware 0.7.0).

m = media de |cinza(t) - cinza(t - dt_s)| numa miniatura (64 px de largura), com a media de cada quadro tirada antes
(a auto-exposicao mudando o brilho geral nao conta como movimento). Escala 0..255 (niveis de cinza).
Medido nos videos de assets/ (10 fps): andando ~5..40, parado ~1..3. O ESP32 compara com stall_cam_max (3,0): rodas
empurrando + m baixo por stall_ms = batida. Limiares a ajustar com os logs de campo (o celular grava "mov" em cada quadro).

Quem chama: main.StreamRunner (campo "mov"); o brain repassa em vis.m ao firmware.
Custo: um resize INTER_AREA + uma subtracao em 64x36 (~0,2 ms no S9). Tempo em segundos, nao em quadros: compara
com o quadro mais novo que tenha pelo menos dt_s de idade (vale a 5, 10 ou 20 fps).
"""
import cv2
import numpy as np


class MotionMeter:
    def __init__(self, dt_s=0.3, width=64):
        self.dt_s = float(dt_s)
        self.width = int(width)
        self._hist = []                      # [(t, miniatura float32 sem a media)], do mais velho ao mais novo
        self.m = None                        # ultimo indice (None ate haver dt_s de historico)

    def reset(self):
        self._hist = []
        self.m = None

    def _small(self, frame_bgr):
        g = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY) if frame_bgr.ndim == 3 else frame_bgr
        h, w = g.shape[:2]
        sw = max(8, min(self.width, w))
        sh = max(4, int(round(h * sw / float(w))))
        s = cv2.resize(g, (sw, sh), interpolation=cv2.INTER_AREA).astype(np.float32)
        return s - float(s.mean())

    def update(self, frame_bgr, t):
        """Um quadro no instante t (s). Devolve o indice m (float) ou None enquanto nao ha dt_s de historico."""
        s = self._small(frame_bgr)
        if self._hist and (t < self._hist[-1][0] or self._hist[-1][1].shape != s.shape):
            self._hist = []                  # relogio voltou ou resolucao mudou: recomeca
        self._hist.append((t, s))
        # descarta os velhos, mantendo o mais novo que ainda tem >= dt_s de idade
        while len(self._hist) > 2 and t - self._hist[1][0] >= self.dt_s:
            self._hist.pop(0)
        t0, s0 = self._hist[0]
        if t - t0 < self.dt_s:
            self.m = None                    # ainda sem dt_s de historico
            return None
        self.m = float(np.mean(np.abs(s - s0)))
        return self.m
