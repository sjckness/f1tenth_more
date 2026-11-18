S4 is the only failing scenario: 3 of 3 runs e-stopped. Lateral offset never went above 0.07 m, so the car barely steered before it got too close.
At 1.0 m the obstacle cost has to win sooner and the steering has to move faster.
- mpc_w_obs 200 → 450: a stronger push from inside the disc against w_term, whose target point is behind the obstacle.
- mpc_w_du_delta 8.33 → 4.0: lets the steering rate react within the ~0.5 m the car has to turn in S4.
S1, S2 and S3 each changed steering sign at most once, so there's room for less damping. These changes should also help S3's -0.072 m margin and S2's -0.000 m.
Width, margin, w_corr and w_term stay the same so the effect of these two changes can be measured.

```yaml
mpc_w_obs: 450.0
mpc_w_term: 9.0
mpc_w_corr: 3.5714
mpc_w_du_delta: 4.0
avoidance_margin: 0.12
corr_half_width_start_m: 0.64
```
