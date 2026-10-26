"""vision/decision.py v0.1 - maquina de 3 estados com histerese, agora em SEGUNDOS.

Entradas por frame (mesmas colunas do coverage.csv) + o instante t (s):
    prob_mean_cen : retroprojecao media (0..1) na faixa inferior, terco central
    unknown_inf   : fracao de pixels com V escuro/estourado, faixa inferior toda
    unknown_cen   : idem, terco central
    t             : timestamp do frame, em segundos

Confianca RELATIVA  r = prob_mean_cen / ref, ref = mediana de prob_mean_cen na janela
inicial de cfg.ref_s segundos (o frame 0 sozinho nao serve: o histograma "acerta a si
mesmo" nele). Premissa: nessa janela so ha chao a frente.

Estados:
    LIVRE      r alto e pouca area "desconhecida" nas laterais
    INCERTO    luz ruim ou r intermediario -> nao declara obstaculo
    OBSTACULO  r muito baixo COM o centro bem exposto

Persistencia: "a condicao se mantem por >= X s". Para frames a 10 fps isto reproduz
exatamente a versao anterior (N frames seguidos = (N-1)*0.1 s), mas nao muda de
significado se o fps mudar (ex.: no S9).

Uso no loop de video:
    dm = DecisionMaker(cfg.decision)
    estado = dm.update(row["prob_mean_cen"], row["unknown_inf"], row["unknown_cen"], t)
"""
from enum import Enum

from config import DecisionConfig

EPS = 1e-6


class State(Enum):
    LIVRE = "LIVRE"
    INCERTO = "INCERTO"
    OBSTACULO = "OBSTACULO"


class DecisionMaker:
    def __init__(self, cfg=None):
        self.cfg = cfg or DecisionConfig()
        self.reset()

    def reset(self):
        self.state = State.LIVRE
        self.ref = None
        self.r = None
        self._ref_buf, self._t_ref0 = [], None
        self._obst_since = None     # desde quando raw == OBSTACULO (ininterrupto)
        self._exit_since = None     # desde quando r >= r_exit dentro de OBSTACULO
        self._free_since = None     # desde quando raw == LIVRE
        self._dark_since = None     # desde quando INCERTO com r < r_obst

    @property
    def warmup(self):
        return self.ref is None

    @staticmethod
    def _held(since, t, dur):
        return since is not None and t - since >= dur - EPS

    def _raw(self, r, u_inf, u_cen):
        """Classificacao instantanea, sem memoria."""
        c = self.cfg
        if r < c.r_obst:
            return State.OBSTACULO if u_cen <= c.u_obst else State.INCERTO
        if r >= c.r_free and u_inf <= c.u_unc:
            return State.LIVRE
        return State.INCERTO

    def update(self, prob_mean_cen, unknown_inf, unknown_cen, t):
        c = self.cfg
        # --- aquecimento: junta a referencia; assume chao (LIVRE) ---
        if self.ref is None:
            if self._t_ref0 is None:
                self._t_ref0 = t
            self._ref_buf.append(prob_mean_cen)
            if t - self._t_ref0 >= c.ref_s - EPS:
                buf = sorted(self._ref_buf)
                self.ref = max(buf[len(buf) // 2], c.ref_min)
            return self.state

        r = prob_mean_cen / self.ref
        self.r = r
        raw = self._raw(r, unknown_inf, unknown_cen)

        # timeout: centro escuro persistente com r baixo vira OBSTACULO
        if raw == State.INCERTO and r < c.r_obst:
            if self._dark_since is None:
                self._dark_since = t
        else:
            self._dark_since = None
        if self._held(self._dark_since, t, c.timeout_s):
            raw = State.OBSTACULO

        if self.state == State.OBSTACULO:
            # saida lenta: r >= r_exit mantido por exit_obst_s
            if r >= c.r_exit:
                if self._exit_since is None:
                    self._exit_since = t
            else:
                self._exit_since = None
            if self._held(self._exit_since, t, c.exit_obst_s):
                self.state = State.LIVRE if raw == State.LIVRE else State.INCERTO
                self._exit_since = self._obst_since = self._free_since = self._dark_since = None
            return self.state

        # entrada rapida em OBSTACULO, cautelosa enquanto confirma
        if raw == State.OBSTACULO:
            if self._obst_since is None:
                self._obst_since = t
        else:
            self._obst_since = None
        if self._held(self._obst_since, t, c.enter_obst_s):
            self.state = State.OBSTACULO
            self._exit_since = self._obst_since = self._free_since = None
        elif raw in (State.INCERTO, State.OBSTACULO):
            self.state = State.INCERTO      # duvida -> cautela imediata
            self._free_since = None
        else:  # raw == LIVRE
            if self._free_since is None:
                self._free_since = t
            if self.state == State.INCERTO and self._held(self._free_since, t, c.enter_livre_s):
                self.state = State.LIVRE
        return self.state