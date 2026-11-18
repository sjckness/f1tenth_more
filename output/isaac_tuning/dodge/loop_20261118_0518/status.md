# Dodge loop loop_20261118_0518

- updated 2026-11-18 05:53:40
- iteration 5 / 40, phase done
- best: d_fail 0, params {"avoidance_margin": 0.12, "class_margin_unknown_m": 0.0, "corr_half_width_start_m": 0.64, "corridor_lateral_reanchor": false, "mission_speed_mps": 0.5, "mpc_obs_vel_lead_s": 0.0, "mpc_obs_vel_trust": 1.0, "mpc_obs_wake_horizon_s": 1.0, "mpc_obs_wake_weight": 0.0, "mpc_obstacle_side_mode": "fixed", "mpc_prediction_dt_s": 0.1, "mpc_return_gain": 1.0, "mpc_return_lookahead_ratio": 2.5, "mpc_return_max_heading_deg": 35.0, "mpc_return_overshoot_max_m": 0.15, "mpc_return_psi_scale": 0.5, "mpc_w_corr": 3.5714, "mpc_w_du_delta": 8.33, "mpc_w_obs": 200.0, "mpc_w_psi": 4.5, "mpc_w_term": 9.0, "mpc_w_v": 4.2857}
- stop reason: 5 iterations without improvement

| iter | change | why | d_fail before | d_fail after | confirm | regressions | accepted | proposer | wall s |
|---|---|---|---|---|---|---|---|---|---|
| 1 | `{"class_margin_unknown_m": 0.08, "mpc_w_obs": 400.0}` | mpc_w_obs 200 → 400** (per-stage effective 70 → 140): in most failures the car never hit its steering limits (steer at limit 0.00 in D00_s1, D01_s2, M1, M3 and two of the three S4 runs; rate at limit mostly 0.07–0.43). So it swerved too little or too late because the obstacle cost lost to the other  | 0 | 0 |  |  | False | claude | 153.6 |
| 2 | `{"mpc_obs_vel_lead_s": 0.2, "mpc_prediction_dt_s": 0.15}` | `mpc_prediction_dt_s` 0.1 → 0.15.** With N = 20 the horizon grows from 2.0 s to 3.0 s, so the car sees 1.5 m ahead at 0.5 m/s instead of 1.0 m. The first level sits 1.5 m ahead, so today it starts beyond the horizon. Nearly every failure shows little or no swerve before contact while steering never  | 0 | 0 |  |  | False | claude | 157.5 |
| 3 | `{"mpc_obs_wake_horizon_s": 0.9953, "mpc_prediction_dt_s": 0.1025}` | random perturbation of the best set | 0 | 0 |  |  | False | random | 767.7 |
| 4 | `{"mpc_obs_wake_weight": 150.0, "mpc_w_term": 6.0}` | `mpc_obs_wake_weight` 0 → 150** (effective 52.5, below w_obs's per-stage 70). It is the one moving-obstacle term never switched on. Several collisions show the car barely leaving the line: M2 at 0.06 m lateral, D01_s2 at 0.39 m, with steering far from its limit. That pattern fits the car trying to p | 0 | 0 |  |  | False | claude | 298.7 |
| 5 | `{"class_margin_unknown_m": 0.0506}` | random perturbation of the best set | 0 | 0 |  |  | False | random | 706.6 |
