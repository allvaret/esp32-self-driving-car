"""decision.py v0 - maquina de 3 estados com histerese.

Entradas por frame (as mesmas colunas do coverage_corredor_auto.csv):
    prob_mean_cen : retroprojecao media (0..1) na faixa inferior, terco central
    unknown_inf   : fracao de pixels com V escuro/estourado, faixa inferior toda
    unknown_cen   : idem, terco central

Ideia central: confianca RELATIVA  r = prob_mean_cen / ref,
onde ref = mediana de prob_mean_cen nos primeiros REF_FRAMES frames.
(O frame 0 sozinho nao serve: o histograma "acerta a si mesmo" nele.)
Premissa: os primeiros REF_FRAMES frames mostram so chao a frente.

Estados:
    LIVRE      r alto e pouca area "desconhecida" nas laterais
    INCERTO    luz ruim ou r intermediario -> nao declara obstaculo
    OBSTACULO  r muito baixo COM o centro bem exposto (assinatura do sofa)

Uso no seu loop de video (depois de ter hist e thr):
    from coverage_log import floor_prob, coverage_row
    from decision import DecisionMaker
    dm = DecisionMaker()
    ...
    mask = floor_mask(frame, hist, thr)
    prob, hsv = floor_prob(frame, hist)
    row = coverage_row(mask, prob, hsv, contador, 0.0)
    estado = dm.update(row["prob_mean_cen"], row["unknown_inf"], row["unknown_cen"])
"""
from enum import Enum


class State(Enum):
    LIVRE = "LIVRE"
    INCERTO = "INCERTO"
    OBSTACULO = "OBSTACULO"


# ---------------- parametros (hipoteses a partir de 2 videos) ----------------
REF_FRAMES = 10      # frames usados para a referencia (1 s a 10 fps)
REF_MIN = 0.05       # piso da referencia (evita dividir por ~0)

R_FREE = 0.5         # r >= R_FREE  -> candidato a LIVRE
R_OBST = 0.2         # r <  R_OBST  -> candidato a OBSTACULO
R_EXIT = 0.4         # para SAIR de OBSTACULO r precisa voltar a >= R_EXIT (histerese)

U_UNC = 0.3          # unknown_inf acima disso -> INCERTO (mesmo com r alto)
U_OBST = 0.15        # OBSTACULO so se unknown_cen <= isso (centro escuro != obstaculo)

N_ENTER_OBST = 2     # frames consecutivos para ENTRAR em OBSTACULO (rapido)
N_EXIT_OBST = 5      # frames consecutivos (r >= R_EXIT) para SAIR de OBSTACULO (lento)
N_ENTER_LIVRE = 3    # frames consecutivos LIVRE para voltar de INCERTO a LIVRE
N_TIMEOUT = 10       # INCERTO com r < R_OBST por N frames -> escala p/ OBSTACULO
# (cobre obstaculo escuro: centro com V baixo nunca passa U_OBST)
# -----------------------------------------------------------------------------


class DecisionMaker:
    def __init__(self):
        self.reset()

    def reset(self):
        self.state = State.LIVRE
        self.ref = None
        self.r = None
        self._ref_buf = []
        self._n_obst = 0    # candidatos consecutivos a OBSTACULO
        self._n_exit = 0    # frames consecutivos com r >= R_EXIT dentro de OBSTACULO
        self._n_free = 0    # frames consecutivos LIVRE
        self._n_dark = 0    # frames consecutivos INCERTO com r < R_OBST

    @property
    def warmup(self):
        return self.ref is None

    def _raw(self, r, u_inf, u_cen):
        """Classificacao instantanea, sem memoria."""
        if r < R_OBST:
            return State.OBSTACULO if u_cen <= U_OBST else State.INCERTO
        if r >= R_FREE and u_inf <= U_UNC:
            return State.LIVRE
        return State.INCERTO

    def update(self, prob_mean_cen, unknown_inf, unknown_cen):
        # --- aquecimento: junta a referencia; assume chao (LIVRE) ---
        if self.ref is None:
            self._ref_buf.append(prob_mean_cen)
            if len(self._ref_buf) >= REF_FRAMES:
                buf = sorted(self._ref_buf)
                self.ref = max(buf[len(buf) // 2], REF_MIN)
            return self.state

        r = prob_mean_cen / self.ref
        self.r = r
        raw = self._raw(r, unknown_inf, unknown_cen)

        # timeout: centro escuro persistente com r baixo vira OBSTACULO
        if raw == State.INCERTO and r < R_OBST:
            self._n_dark += 1
        else:
            self._n_dark = 0
        if self._n_dark >= N_TIMEOUT:
            raw = State.OBSTACULO

        if self.state == State.OBSTACULO:
            # saida lenta: precisa de N_EXIT_OBST frames com r >= R_EXIT
            self._n_exit = self._n_exit + 1 if r >= R_EXIT else 0
            if self._n_exit >= N_EXIT_OBST:
                self.state = State.LIVRE if raw == State.LIVRE else State.INCERTO
                self._n_exit = self._n_obst = self._n_free = self._n_dark = 0
            return self.state

        # entrada rapida em OBSTACULO (2 frames), cautelosa enquanto confirma
        self._n_obst = self._n_obst + 1 if raw == State.OBSTACULO else 0
        if self._n_obst >= N_ENTER_OBST:
            self.state = State.OBSTACULO
            self._n_exit = self._n_obst = self._n_free = 0
        elif raw in (State.INCERTO, State.OBSTACULO):
            self.state = State.INCERTO      # duvida -> cautela imediata
            self._n_free = 0
        else:  # raw == LIVRE
            self._n_free += 1
            if self.state == State.INCERTO and self._n_free >= N_ENTER_LIVRE:
                self.state = State.LIVRE
        return self.state