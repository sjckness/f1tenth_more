S2 (D1.0, r = 0.5) collided in 1 of 2 runs at -0.086 m. Clearing it needs about 0.82 m of lateral offset, but the corridor is only 0.64 m wide at the car end.
S1 is good 3/3 but with only 0.023 m of margin, so avoidance starts late even on the small cylinder.
Widen the corridor start, 0.64 → 0.85, so a 0.82 m offset is reachable before the obstacle.
Raise w_obs, 200 → 300 (effective 70 → 105), so the obstacle cost wins over w_term (9) sooner.
Leave w_term, w_corr, w_du_delta and the margin at their defaults to keep the change small.
Note: 8 of 13 runs were invalid, and S3, S4 and S5 have n = 0. Check the harness before reading too much into this score.

```yaml
mpc_w_obs: 300.0
mpc_w_term: 9.0
mpc_w_corr: 3.5714
mpc_w_du_delta: 8.33
avoidance_margin: 0.12
corr_half_width_start_m: 0.85
```
