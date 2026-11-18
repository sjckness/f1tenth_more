**`mpc_obs_wake_weight` 0 → 150** (effective 52.5, below w_obs's per-stage 70). It is the one moving-obstacle term never switched on. Several collisions show the car barely leaving the line: M2 at 0.06 m lateral, D01_s2 at 0.39 m, with steering far from its limit. That pattern fits the car trying to pass in front of a sliding cylinder that then moves into it. The predicted disc alone does not cover the swept band ahead of the motion; the wake penalises that band, so the car commits early to passing behind. Static runs (S1–S5) have no approaching velocity, so they are unaffected.

**`mpc_w_term` 9 → 6.** The terminal pull goes to a centreline point that obstacles never move. Most failures show under-committed swerves (steer at limit 0.00 in D00_s1, D01_s2, M1, M3 and two of the three S4 runs). Raising w_obs was already tried together with a larger margin. This tests the other side of the ratio: the obstacle cost gains relative weight inside the disc without inflating the cost field near static walls or cylinders.

```yaml
avoidance_margin: 0.12
class_margin_unknown_m: 0.0
corr_half_width_start_m: 0.64
corridor_lateral_reanchor: false
mission_speed_mps: 0.5
mpc_obs_vel_lead_s: 0.0
mpc_obs_vel_trust: 1.0
mpc_obs_wake_horizon_s: 1.0
mpc_obs_wake_weight: 150.0
mpc_obstacle_side_mode: fixed
mpc_prediction_dt_s: 0.1
mpc_return_gain: 1.0
mpc_return_lookahead_ratio: 2.5
mpc_return_max_heading_deg: 35.0
mpc_return_overshoot_max_m: 0.15
mpc_return_psi_scale: 0.5
mpc_w_corr: 3.5714
mpc_w_du_delta: 8.33
mpc_w_obs: 200.0
mpc_w_psi: 4.5
mpc_w_term: 6.0
mpc_w_v: 4.2857
```
