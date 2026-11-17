"""tests/test_heading.py - controle de rumo com o giroscopio, contra uma planta simulada.
Rode da raiz do repositorio:   python -m tests.test_heading"""
from config import Config, HeadingConfig
from control.heading import HeadingHold


def simulate(cfg, bias_s=0.15, gain_dps=150.0, v=0.35, T=15.0, hold=True,
             gyro_hz=50, frame_hz=10, lag_s=0.1, gyro_bias_dps=0.0):
    """Planta: giro = gain_dps * (s_aplicado + bias_s) com lag de 1a ordem.
    bias_s > 0 = o carro puxa para a direita com steer 0 (caso real do carrinho)."""
    hh = HeadingHold(cfg)
    dt = 1.0 / gyro_hz
    every = gyro_hz // frame_hz
    rate = psi_true = 0.0
    s_applied, hist = 0.0, []
    for i in range(int(T * gyro_hz)):
        t = i * dt
        if i % every == 0:                                    # frame: decide s
            s_applied, _ = hh.update(t, v, 0.0) if hold else (0.0, None)
        target = gain_dps * (s_applied + bias_s)
        rate += (target - rate) * dt / lag_s
        psi_true += rate * dt
        hh.feed_rate(rate + gyro_bias_dps, t)
        hist.append((t, rate, psi_true, s_applied))
    return hist


def test_gating():
    hh = HeadingHold()
    hh.feed_rate(30.0, 0.0)
    assert hh.update(0.0, 0.0, 0.0)[0] == 0.0                  # parado: nao mexe
    assert hh.update(0.1, 0.3, 0.5)[0] == 0.5                  # curva comandada: nao mexe
    assert hh.update(0.2, -0.3, 0.0)[0] == 0.0                 # re: nao mexe
    print("ok  so atua em trecho reto (parado/curva/re passam direto)")


def test_arrancada_e_giroscopio_velho():
    cfg = HeadingConfig()
    hh = HeadingHold(cfg)
    t = 0.0
    while t < 1.0:
        hh.feed_rate(40.0, t)
        s, info = hh.update(t, 0.3, 0.0)
        if t < cfg.engage_delay_s:
            assert s == 0.0 and info["why"] == "arrancando", (t, info)
        t += 0.02
    assert s < 0.0, s                                          # girando p/ direita -> corrige p/ esquerda
    s2, info = hh.update(t + 1.0, 0.3, 0.0)                    # 1 s sem amostra
    assert s2 == 0.0 and info["why"] == "giroscopio sem amostra"
    print("ok  ignora a arrancada; sem giroscopio recente nao corrige; sinal certo (direita -> s<0)")


def test_malha_fechada():
    cfg = HeadingConfig()
    sem = simulate(cfg, hold=False)
    com = simulate(cfg, hold=True)
    psi_sem = abs(sem[-1][2])
    psi_com_max = max(abs(h[2]) for h in com)
    rate_final = abs(com[-1][1])
    assert psi_sem > 200, psi_sem                              # sem correcao o carro roda
    assert psi_com_max < 25.0 and rate_final < 2.0, (psi_com_max, rate_final)
    print(f"ok  desvio fixo de 0.15: sem controle {psi_sem:.0f} graus em 15 s; "
          f"com controle max {psi_com_max:.1f} graus, giro final {rate_final:.2f} graus/s")


def test_robusto_a_planta_errada():
    """Ganho da planta 3x maior/menor e desvio dos dois sinais: nao pode divergir."""
    cfg = HeadingConfig()
    for gain in (50.0, 150.0, 450.0):
        for b in (-0.2, 0.1, 0.25):
            h = simulate(cfg, bias_s=b, gain_dps=gain, T=20.0)
            tail = [abs(x[1]) for x in h[-100:]]
            assert max(tail) < 3.0, (gain, b, max(tail))
    print("ok  estavel com ganho da planta 50..450 graus/s e desvio -0.2..+0.25")


def test_vies_do_giroscopio():
    cfg = HeadingConfig()
    hh = HeadingHold(cfg)
    t = 0.0
    while t < 6.0:                                             # parado: giroscopio marca 0.5 graus/s
        hh.feed_rate(0.5, t)
        hh.update(t, 0.0, 0.0)
        t += 0.02
    assert abs(hh._bias - 0.5) < 0.05, hh._bias
    print(f"ok  vies do giroscopio estimado parado: {hh._bias:.2f} graus/s")


def test_config():
    d = Config().to_dict()
    assert "heading" in d and Config.from_dict(d).heading == HeadingConfig()
    print("ok  HeadingConfig entra no config.json (ida e volta)")


if __name__ == "__main__":
    test_gating()
    test_arrancada_e_giroscopio_velho()
    test_malha_fechada()
    test_robusto_a_planta_errada()
    test_vies_do_giroscopio()
    test_config()
    print("\nTODOS OS TESTES DE RUMO PASSARAM")
