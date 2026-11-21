"""control/heading.py - segura o rumo em linha reta com o giroscopio (malha fechada).

Por que existe: os dois lados do carro nao rendem igual (caixa/embreagem com defeito) e a
diferenca muda com carga, bateria e velocidade; um trim fixo nao segurou. Aqui o carro MEDE
o quanto esta girando e corrige o steer em tempo real.

Convencoes (iguais as do resto do projeto):
  steer s > 0  = direita (firmware: roda ESQUERDA mais rapida).
  rate  r > 0  = girando para a DIREITA, em graus/s (o Kotlin ja entrega assim).
  psi   (graus) = rumo acumulado desde que o trecho reto comecou; > 0 = desviou p/ direita.

Lei de controle (PI no giro; psi e a integral de r):
  s_out = s_cmd - (kp_rate * r + kh * psi)        limitado a +-s_max
  Em regime, r -> 0 e psi fica num valor pequeno que cancela o desvio fixo dos motores.

So atua quando e um trecho reto de verdade: v > v_min e |s_cmd| < s_eps. Fora disso
(parado, curva, re) repassa o s_cmd e zera o psi, entao o rumo de referencia e sempre o
do inicio de cada trecho reto. Sem amostra recente do giroscopio (stale_s) tambem repassa.

Uso (brain.py):
    hh = HeadingHold(cfg.heading)
    hh.feed_rate(rate_dps, t)                 # a cada amostra do giroscopio (~50 Hz)
    s, info = hh.update(t, v, s_cmd=0.0)      # a cada frame, antes do link.drive(v, s)
`t` = segundos de um relogio monotonico (o mesmo nas duas chamadas).
"""
from config import HeadingConfig


class HeadingHold:
    def __init__(self, cfg=None):
        self.cfg = cfg or HeadingConfig()
        self.reset()

    def reset(self):
        """Zera referencia e filtros (chamar ao recalibrar / trocar de motivacao)."""
        self._t_rate = None          # instante da ultima amostra
        self._r_raw = 0.0            # ultima amostra (sem vies)
        self._r = 0.0                # amostra filtrada
        self._bias = 0.0             # vies do giroscopio (estimado com o carro parado)
        self._psi = 0.0
        self._t_still = None         # desde quando o carro esta parado
        self._t_engage = None        # desde quando o trecho reto atual comecou
        self._prev_engaged = False
        self.last = {"on": False, "why": "sem amostra"}

    # ---- giroscopio ---------------------------------------------------------
    def feed_rate(self, rate_dps, t):
        """Uma amostra: rate_dps > 0 = girando para a direita."""
        c = self.cfg
        if self._t_rate is not None:
            dt = t - self._t_rate
            if dt <= 0.0:
                return
            dt = min(dt, c.max_dt_s)                 # amostra atrasada nao vira salto de psi
            a = dt / (c.rate_tau_s + dt)             # filtro passa-baixa de 1a ordem
            if (self._t_still is not None and t - self._t_still >= c.bias_still_s
                    and abs(rate_dps - self._bias) <= c.bias_gate_dps):   # mao mexendo no carro != vies
                self._bias += (rate_dps - self._bias) * min(1.0, dt / c.bias_tau_s)
            r = rate_dps - self._bias
            self._r += a * (r - self._r)
            if self._prev_engaged and self._t_engage is not None and t - self._t_engage >= c.engage_delay_s:
                self._psi += self._r * dt
                lim = c.psi_max_deg
                self._psi = max(-lim, min(lim, self._psi))      # anti-windup
        else:
            self._r = rate_dps - self._bias
        self._r_raw, self._t_rate = rate_dps, t

    # ---- controle -----------------------------------------------------------
    def update(self, t, v, s_cmd=0.0):
        """Retorna (s_out, info). Chamar a cada frame, depois de decidir v e s_cmd."""
        c = self.cfg
        engaged = c.enabled and v > c.v_min and abs(s_cmd) < c.s_eps

        # "parado" para estimar o vies: v ~ 0 e sem comando de giro
        if v < 0.02 and abs(s_cmd) < c.s_eps:
            if self._t_still is None:
                self._t_still = t
        else:
            self._t_still = None

        if not engaged:
            self._prev_engaged, self._t_engage, self._psi = False, None, 0.0
            self.last = {"on": False, "why": "desligado" if not c.enabled else "fora de trecho reto",
                         "rate": self._r, "psi": 0.0, "bias": self._bias, "s_corr": 0.0}
            return s_cmd, self.last
        if not self._prev_engaged:                   # comecou um trecho reto
            self._prev_engaged, self._t_engage, self._psi = True, t, 0.0

        stale = self._t_rate is None or t - self._t_rate > c.stale_s
        if stale:
            self.last = {"on": False, "why": "giroscopio sem amostra", "rate": self._r,
                         "psi": self._psi, "bias": self._bias, "s_corr": 0.0}
            return s_cmd, self.last
        if t - self._t_engage < c.engage_delay_s:    # arrancada: kick dos motores nao conta
            self.last = {"on": False, "why": "arrancando", "rate": self._r,
                         "psi": 0.0, "bias": self._bias, "s_corr": 0.0}
            return s_cmd, self.last

        corr = -(c.kp_rate * self._r + c.kh * self._psi)
        corr = max(-c.s_max, min(c.s_max, corr))
        s_out = max(-1.0, min(1.0, s_cmd + corr))
        self.last = {"on": True, "why": "", "rate": self._r, "psi": self._psi,
                     "bias": self._bias, "s_corr": corr}
        return s_out, self.last


class YawIntegrator:
    """Rumo acumulado SEM limite (graus, + = direita) e giro filtrado, para o ESP32 medir curvas.

    Mesmo tratamento de vies do HeadingHold (filtro de 1a ordem; vies estimado com o carro PARADO,
    ignorando amostras fora de +-bias_gate_dps), mas sem anti-windup: o psi precisa valer 90 graus
    numa curva de 90 graus. So o DELTA de psi importa para o firmware; reset() zera o psi e
    mantem o vies ja estimado (reset(full=True) zera tudo).

    Uso (brain.on_gyro):   yi.feed_rate(rate_dps, t, still)   # still = rodas paradas
    """

    def __init__(self, cfg=None):
        self.cfg = cfg or HeadingConfig()
        self.reset(full=True)

    def reset(self, full=False):
        self._t_rate = None
        self._r = 0.0
        self._psi = 0.0
        self._t_still = None
        if full:
            self._bias = 0.0

    @property
    def psi(self):
        return self._psi

    @property
    def rate(self):
        return self._r

    @property
    def bias(self):
        return self._bias

    def feed_rate(self, rate_dps, t, still=False):
        c = self.cfg
        if still:
            if self._t_still is None:
                self._t_still = t
        else:
            self._t_still = None
        if self._t_rate is None:
            self._r = rate_dps - self._bias
            self._t_rate = t
            return
        dt = t - self._t_rate
        if dt <= 0.0:
            return
        dt = min(dt, c.max_dt_s)
        a = dt / (c.rate_tau_s + dt)
        if (self._t_still is not None and t - self._t_still >= c.bias_still_s
                and abs(rate_dps - self._bias) <= c.bias_gate_dps):
            self._bias += (rate_dps - self._bias) * min(1.0, dt / c.bias_tau_s)
        self._r += a * ((rate_dps - self._bias) - self._r)
        self._psi += self._r * dt
        self._t_rate = t
