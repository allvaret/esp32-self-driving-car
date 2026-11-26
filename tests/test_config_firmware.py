"""tests/test_config_firmware.py - config.py (ControlConfig) e Params do firmware dizem a mesma coisa."""
import os
import re

from config import Config

INO = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "car_firmware", "car_firmware.ino")


def _params():
    src = open(INO, encoding="utf-8").read()
    body = src[src.index("struct Params {"):src.index("static Params P;")]
    out = {}
    for name, val in re.findall(r"\b(\w+)\s*=\s*(-?[\d.]+)f?\b", body):
        out[name] = float(val)
    return out


def test_control_config_matches_firmware_params():
    p, c = _params(), Config().control
    pairs = {  # (campo do config, valor esperado calculado do firmware)
        "d_stop_m": p["d_stop_cm"] / 100, "d_clear_m": p["d_clear_cm"] / 100, "none_score_m": p["none_score_cm"] / 100,
        "v_cruise": p["v_cruise"], "v_slow": p["v_slow"], "v_rev": p["v_rev"], "v_turn": p["v_turn"],
        "scan_deg": p["scan_deg"], "scan_step_deg": p["scan_step"], "turn_dps": p["turn_dps"],
        "stop_s": p["stop_ms"] / 1000, "reverse_s": p["reverse_ms"] / 1000, "look_timeout_s": p["look_timeout_ms"] / 1000,
        "clear_s": p["clear_ms"] / 1000, "avoid_retries": p["avoid_retries"], "guard_window_s": p["guard_window_ms"] / 1000,
        "guard_max_episodes": p["guard_max"], "vis_fresh_s": p["vis_fresh_ms"] / 1000, "yaw_fresh_s": p["yaw_fresh_ms"] / 1000,
        "vis_free_min": p["vis_free_min"], "suspect_s": p["suspect_ms"] / 1000, "yaw_lead_s": p["yaw_lead_ms"] / 1000,
        "hold_kp": p["hold_kp"], "hold_ki": p["hold_ki"], "hold_s_max": p["hold_s_max"], "hold_s_eps": p["hold_s_eps"],
        "hold_delay_s": p["hold_delay_ms"] / 1000, "hold_psi_max_deg": p["hold_psi_max"], "hold_pwm_max": p["hold_pwm_max"],
        "trim_rate": p["trim_rate"], "trim_max": p["trim_max"], "sensor_fresh_s": p["fresh_ms"] / 1000,
        "turn_slow_deg": p["turn_slow_deg"], "turn_k_min": p["turn_k_min"],
        "cred_max": p["cred_max"], "cred_psi_tol_deg": p["cred_psi_tol"], "cred_min_s": p["cred_min_ms"] / 1000,
        "escape_deg": p["escape_deg"], "escape_max": p["escape_max"],
        "stall_s": p["stall_ms"] / 1000, "stall_cam_max": p["stall_cam_max"], "stall_yaw_dps": p["stall_yaw_dps"],
        "stall_sonar_cm": p["stall_sonar_cm"], "stall_rev_s": p["stall_rev_ms"] / 1000, "stall_turn_deg": p["stall_turn_deg"],
        "stall_free_s": p["stall_free_ms"] / 1000,
        "pwm_min": p["pwm_min"], "pwm_max": p["pwm_max"], "pwm_min_l": p["pwm_min_l"], "pwm_max_l": p["pwm_max_l"],
        "pwm_min_r": p["pwm_min_r"], "pwm_max_r": p["pwm_max_r"],
    }
    bad = {k: (getattr(c, k), v) for k, v in pairs.items() if abs(getattr(c, k) - v) > 1e-9}
    assert not bad, f"config.py x Params divergem (config, firmware): {bad}"
    assert tuple(c.guard_backoff_s) == (p["backoff1_ms"] / 1000, p["backoff2_ms"] / 1000, p["backoff3_ms"] / 1000)
    print(f"ok  config.py == Params do firmware em {len(pairs) + 1} parametros")
