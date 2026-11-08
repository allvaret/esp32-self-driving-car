"""config.py - parametros do projeto num lugar so (raiz do repositorio).

Todo valor aqui e [DEMO]: foi escolhido por falta de informacao real (carro, sensor,
S9). A lista completa de o-que-medir-e-como esta em docs/parametros_a_calibrar.md.

Use:
    cfg = Config.load("config.json")       # ou Config() para os padroes
    Config().save("config.json")           # gera um JSON editavel
    ctl = Controller(head, cfg.control);  dm = DecisionMaker(cfg.decision)

Tempos sao SEMPRE em segundos (nunca em ticks/frames), para nao dependerem do fps.
Velocidades 'v' sao normalizadas em [-1, 1] (nao sao m/s).
"""
import json
from dataclasses import asdict, dataclass, field, fields
from typing import Optional, Tuple


@dataclass
class ControlConfig:
    # --- distancias do sensor (m) ---
    d_stop_m: float = 0.50        # sensor central mais perto -> AVOID. Derivar: v*(latencia)+frenagem+margem
    d_clear_m: float = 1.00       # leitura valida mais longe "refuta" a visao
    none_score_m: float = 0.50    # valor de um lado SEM eco ao escolher a curva (> perto, < livre)
    # --- velocidades normalizadas [0, 1] ---
    v_cruise: float = 0.5
    v_slow: float = 0.2
    v_rev: float = 0.3
    v_turn: float = 0.3
    # --- servo / varredura ---
    scan_deg: float = 45.0        # angulo da olhada lateral (e alvo aproximado da curva)
    look_timeout_s: float = 1.0   # se o servo nao assentar, segue como 'sem informacao'
    # --- manobra de desvio (s) ---
    stop_s: float = 0.3
    reverse_s: float = 0.8        # idealmente derivado de reverse_m (ver derive_maneuver_times)
    turn_s: float = 1.0           # idealmente derivado de turn_deg ~ scan_deg
    avoid_retries: int = 2        # re maior + nova olhada antes de desistir (HOLD)
    # --- histerese (s) ---
    clear_s: float = 0.2          # tempo "limpo" p/ voltar a CRUISE / sair de HOLD (antes: 3 ticks @10 fps)
    # --- guarda de reentrada (pessoa/obstaculo que volta) ---
    guard_window_s: float = 20.0
    guard_max_episodes: int = 2   # a partir do (N+1)-esimo AVOID na janela, espera em vez de manobrar
    guard_backoff_s: Tuple[float, ...] = (1.0, 2.0, 4.0)   # sobe a cada reentrada; reseta apos a janela
    # --- bordas (hipoteses, nao validadas) ---
    edge_near_h: float = 0.20     # h_cen <= isso (borda perto) ...
    edge_cov_min: float = 0.50    # ... com cobertura >= isso -> edge_near


@dataclass
class DecisionConfig:
    # --- referencia de confianca ---
    ref_s: float = 0.9            # janela p/ a mediana de prob_mean_cen (antes: 10 frames @10 fps)
    ref_min: float = 0.05         # piso da referencia
    # --- limiares de r = prob_mean_cen / ref (hipoteses a partir de 2 videos) ---
    r_free: float = 0.5
    r_obst: float = 0.2
    r_exit: float = 0.4           # histerese: sair de OBSTACULO exige r >= r_exit
    u_unc: float = 0.3            # unknown_inf acima disso -> INCERTO
    u_obst: float = 0.15          # OBSTACULO so se unknown_cen <= isso
    # --- persistencia (s) - 'condicao mantida por pelo menos X' ---
    enter_obst_s: float = 0.1     # antes: 2 frames @10 fps
    exit_obst_s: float = 0.4      # antes: 5 frames
    enter_livre_s: float = 0.2    # antes: 3 frames
    timeout_s: float = 0.9        # INCERTO com r < r_obst por X s -> OBSTACULO (antes: 10 frames)


@dataclass
class CarCalib:
    """Medidas do carro real. None = ainda nao medido."""
    speed_at_v_rev_mps: Optional[float] = None    # m/s medido com v = control.v_rev, em linha reta
    yaw_rate_dps: Optional[float] = None          # graus/s medidos com steer=1, v = control.v_turn


@dataclass
class Config:
    control: ControlConfig = field(default_factory=ControlConfig)
    decision: DecisionConfig = field(default_factory=DecisionConfig)
    calib: CarCalib = field(default_factory=CarCalib)

    def validate(self):
        c, d = self.control, self.decision
        if not 0 < c.d_stop_m < c.d_clear_m:
            raise ValueError("control: precisa 0 < d_stop_m < d_clear_m")
        for n in ("v_cruise", "v_slow", "v_rev", "v_turn"):
            if not 0 <= getattr(c, n) <= 1:
                raise ValueError(f"control.{n} deve estar em [0, 1]")
        for n in ("stop_s", "reverse_s", "turn_s", "clear_s", "look_timeout_s", "guard_window_s"):
            if getattr(c, n) < 0:
                raise ValueError(f"control.{n} nao pode ser negativo")
        if not c.guard_backoff_s or min(c.guard_backoff_s) <= 0:
            raise ValueError("control.guard_backoff_s precisa de valores > 0")
        if not 0 < d.r_obst < d.r_exit <= d.r_free:
            raise ValueError("decision: precisa 0 < r_obst < r_exit <= r_free")
        for n in ("ref_s", "enter_obst_s", "exit_obst_s", "enter_livre_s", "timeout_s"):
            if getattr(d, n) < 0:
                raise ValueError(f"decision.{n} nao pode ser negativo")
        return self

    # ---- JSON ----
    @classmethod
    def from_dict(cls, data):
        sections = {"control": ControlConfig, "decision": DecisionConfig, "calib": CarCalib}
        extra = set(data) - set(sections)
        if extra:
            raise ValueError(f"secoes desconhecidas: {sorted(extra)}")
        built = {}
        for name, klass in sections.items():
            vals = dict(data.get(name, {}))
            bad = set(vals) - {f.name for f in fields(klass)}
            if bad:
                raise ValueError(f"[{name}] parametros desconhecidos (typo?): {sorted(bad)}")
            if "guard_backoff_s" in vals:
                vals["guard_backoff_s"] = tuple(vals["guard_backoff_s"])
            built[name] = klass(**vals)
        return cls(**built).validate()

    @classmethod
    def load(cls, path=None):
        if path is None:
            return cls().validate()
        with open(path) as f:
            return cls.from_dict(json.load(f))

    def to_dict(self):
        return asdict(self)

    def save(self, path):
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)


def derive_maneuver_times(cfg, reverse_m, turn_deg):
    """(reverse_s, turn_s) a partir de medidas do carro e do que se quer da manobra.
    reverse_s = reverse_m / velocidade medida;  turn_s = turn_deg / taxa de giro medida."""
    cal = cfg.calib
    if not cal.speed_at_v_rev_mps or not cal.yaw_rate_dps:
        raise ValueError("preencha calib.speed_at_v_rev_mps e calib.yaw_rate_dps antes")
    return reverse_m / cal.speed_at_v_rev_mps, turn_deg / cal.yaw_rate_dps