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
    d_stop_m: float = 0.30        # sensor central mais perto -> DESVIO. Derivar: v*(latencia)+frenagem+margem
    d_clear_m: float = 0.60       # leitura valida mais longe "refuta" a visao
    none_score_m: float = 0.45    # valor de um lado SEM eco ao escolher a curva (> perto, < livre)
    # --- velocidades normalizadas [0, 1] (iguais as do firmware: Params) ---
    v_cruise: float = 0.35
    v_slow: float = 0.15
    v_rev: float = 0.3
    v_turn: float = 0.3
    # --- servo / varredura: o ESP32 varre -scan_deg..+scan_deg em passos de scan_step_deg (sem o 0) ---
    scan_deg: float = 30.0
    scan_step_deg: float = 15.0
    look_timeout_s: float = 1.0   # se o servo nao assentar, segue como 'sem informacao'
    # --- manobra de desvio (s) ---
    stop_s: float = 0.3
    reverse_s: float = 0.5        # idealmente derivado de reverse_m (ver derive_maneuver_times); x1,5 se nao ha saida
    turn_s: float = 1.0           # SO o controller v1 congelado (tempo fixo). O firmware gira ate o angulo: yaw ou turn_dps
    turn_dps: float = 90.0        # graus/s com v_turn e steer=1 (CALIBRAR; = calib.yaw_rate_dps). So vale sem yaw do celular
    avoid_retries: int = 2        # re maior + nova olhada antes de desistir (PRESO)
    # --- histerese (s) ---
    clear_s: float = 0.2          # tempo "limpo" p/ voltar a FRENTE / sair de PRESO (antes: 3 ticks @10 fps)
    # --- guarda de reentrada (pessoa/obstaculo que volta) ---
    guard_window_s: float = 20.0
    guard_max_episodes: int = 2   # a partir do (N+1)-esimo DESVIO na janela, espera em vez de manobrar
    guard_backoff_s: Tuple[float, ...] = (1.0, 2.0, 4.0)   # sobe a cada reentrada; reseta apos a janela
    # --- "sem eco" em 3 estados e opiniao do celular (firmware; ver docs/parametros_a_calibrar.md) ---
    sensor_fresh_s: float = 0.4   # leitura central mais velha que isso nao vale (firmware fresh_ms; brain: refutar a visao)
    vis_fresh_s: float = 0.3      # vis mais velha que isso = sem opiniao
    yaw_fresh_s: float = 0.2      # yaw mais velho que isso = curva pelo tempo (turn_dps)
    vis_free_min: float = 50.0    # setor 0 da visao (0..100) a partir do qual "livre"; abaixo = bloqueado/suspeito
    suspect_s: float = 1.5        # sem eco ate X s depois de leitura valida < d_clear = suspeito
    yaw_lead_s: float = 0.1       # antecipa o fim da curva em giro*lead (compensa a inercia)
    # --- rumo em reta NO ESP32 (firmware: hold_*, trim_*). Unidades de RODA: 0,1 = +0,1 de comando na roda lenta ---
    # s_corr = -(hold_kp*giro + hold_ki*psi_trecho) + trim aprendido, limitado a +-hold_s_max; > 0 = mais forca na esquerda
    hold_kp: float = 0.01         # por grau/s de giro
    hold_ki: float = 0.05         # por grau de rumo desde o inicio do trecho reto
    hold_s_max: float = 2.0       # teto da correcao (compensa ~35% de diferenca entre as rodas)
    hold_s_eps: float = 0.05      # |s| abaixo disso = trecho reto
    hold_delay_s: float = 0.1     # o PI entra no maximo 0,1 s depois da arrancada (o trim vale desde o inicio)
    hold_psi_max_deg: float = 30.0    # anti-windup do psi do trecho
    hold_pwm_max: float = 200.0   # teto de duty (0..255) da roda lenta corrigida (pode passar do pwm_max)
    trim_rate: float = 0.5        # por s: velocidade com que o trim absorve o termo hold_ki*psi
    trim_max: float = 1.5         # limite do trim aprendido (salvo em NVS ao desarmar)
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
class HeadingConfig:
    """Rumo em linha reta com o giroscopio (control/heading.py). Todos [DEMO]: ver docs."""
    enabled: bool = True
    kp_rate: float = 0.005        # steer por (grau/s) de giro. Alto demais -> o carro balanca
    kh: float = 0.02              # steer por grau de rumo acumulado (e quem cancela o desvio fixo)
    s_max: float = 0.4            # limite da correcao (nao toma o controle do desvio)
    psi_max_deg: float = 20.0     # limite do rumo acumulado (anti-windup)
    v_min: float = 0.05           # so corrige andando para a frente
    s_eps: float = 0.05           # so corrige quando o comando e "reto"
    engage_delay_s: float = 0.4   # ignora a arrancada (kick/escalonamento dos motores)
    stale_s: float = 0.3          # sem amostra do giroscopio por X s -> nao corrige
    rate_tau_s: float = 0.05      # filtro do giro
    max_dt_s: float = 0.1         # amostra atrasada nao integra mais que isso
    bias_still_s: float = 0.5     # parado ha X s -> estima o vies do giroscopio
    bias_tau_s: float = 1.0
    bias_gate_dps: float = 5.0    # so aceita amostra parada dentro de +-X graus/s do vies atual (ignora manuseio)


@dataclass
class CarCalib:
    """Medidas do carro real. None = ainda nao medido."""
    speed_at_v_rev_mps: Optional[float] = None    # m/s medido com v = control.v_rev, em linha reta
    yaw_rate_dps: Optional[float] = None          # graus/s medidos com steer=1, v = control.v_turn


@dataclass
class Config:
    control: ControlConfig = field(default_factory=ControlConfig)
    decision: DecisionConfig = field(default_factory=DecisionConfig)
    heading: HeadingConfig = field(default_factory=HeadingConfig)
    calib: CarCalib = field(default_factory=CarCalib)

    def validate(self):
        c, d = self.control, self.decision
        if not 0 < c.d_stop_m < c.d_clear_m:
            raise ValueError("control: precisa 0 < d_stop_m < d_clear_m")
        for n in ("v_cruise", "v_slow", "v_rev", "v_turn"):
            if not 0 <= getattr(c, n) <= 1:
                raise ValueError(f"control.{n} deve estar em [0, 1]")
        if c.scan_step_deg <= 0 or c.scan_deg < c.scan_step_deg or c.turn_dps <= 0:
            raise ValueError("control: scan_step_deg > 0, scan_deg >= scan_step_deg, turn_dps > 0")
        for n in ("stop_s", "reverse_s", "turn_s", "clear_s", "look_timeout_s", "guard_window_s",
                  "vis_fresh_s", "yaw_fresh_s", "suspect_s", "yaw_lead_s", "sensor_fresh_s"):
            if getattr(c, n) < 0:
                raise ValueError(f"control.{n} nao pode ser negativo")
        for n in ("hold_kp", "hold_ki", "hold_s_max", "hold_s_eps", "hold_delay_s", "hold_psi_max_deg",
                  "trim_rate", "trim_max"):
            if getattr(c, n) < 0:
                raise ValueError(f"control.{n} nao pode ser negativo")
        if not 0 < c.hold_pwm_max <= 255:
            raise ValueError("control.hold_pwm_max deve estar em (0, 255]")
        if not c.guard_backoff_s or min(c.guard_backoff_s) <= 0:
            raise ValueError("control.guard_backoff_s precisa de valores > 0")
        if not 0 < d.r_obst < d.r_exit <= d.r_free:
            raise ValueError("decision: precisa 0 < r_obst < r_exit <= r_free")
        for n in ("ref_s", "enter_obst_s", "exit_obst_s", "enter_livre_s", "timeout_s"):
            if getattr(d, n) < 0:
                raise ValueError(f"decision.{n} nao pode ser negativo")
        h = self.heading
        if not 0 <= h.s_max <= 1 or min(h.kp_rate, h.kh) < 0:
            raise ValueError("heading: s_max em [0, 1] e ganhos >= 0")
        for n in ("engage_delay_s", "stale_s", "rate_tau_s", "max_dt_s", "bias_still_s", "bias_tau_s"):
            if getattr(h, n) <= 0:
                raise ValueError(f"heading.{n} deve ser > 0")
        return self

    # ---- JSON ----
    @classmethod
    def from_dict(cls, data):
        sections = {"control": ControlConfig, "decision": DecisionConfig,
                    "heading": HeadingConfig, "calib": CarCalib}
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