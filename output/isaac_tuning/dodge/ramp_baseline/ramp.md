# Dodge ramp baseline

- d_fail: **0**, highest level passed: None
- params: `{"avoidance_margin": 0.12, "corr_half_width_start_m": 0.64, "mpc_w_corr": 3.5714, "mpc_w_du_delta": 8.33, "mpc_w_obs": 200.0, "mpc_w_term": 9.0}`

| d | speed | levels | diag | run | outcome | margin m | body m | fail level | fail margin m | delta sat | rate sat |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 0.2 | 2 | 0.0 | D00_s0 | collision | -0.12 | 0.00 | 1 | -0.12 | 0.07 | 0.12 |
| 0 | 0.2 | 2 | 0.0 | D00_s1 | collision | -0.08 | -0.04 | 0 | -0.08 | 0.07 | 0.16 |
| 0 | 0.2 | 2 | 0.0 | D00_s2 | success | 0.50 | 0.60 |  |  | 0.00 | 0.00 |
