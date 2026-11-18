- **mpc_w_obs 200 → 400** (per-stage effective 70 → 140): in most failures the car never hit its steering limits (steer at limit 0.00 in D00_s1, D01_s2, M1, M3 and two of the three S4 runs; rate at limit mostly 0.07–0.43). So it swerved too little or too late because the obstacle cost lost to the other costs, not because the actuators ran out. The best set makes the pull to the line stronger (`corridor_lateral_reanchor` false and `mpc_return_gain` 1.0), so the obstacle weight has to go up to match. The S4 estops with about 0 lateral offset and D01_s2 with 0.39 m point to the same cause.
- **class_margin_unknown_m 0.0 → 0.08**: the obstacle cost is soft, so the car always cuts into the disc a little. With no class margin, the MPC disc is the same as the scoring R_safe, which leaves only the 0.03 m tolerance. That fits the near-misses at margins of −0.01 to −0.10 (M3 and S4 estops, S3 and D00_s0 successes). A 0.08 m buffer on LiDAR-only discs (the Isaac cylinders) should move them back above R_safe − 0.03.

```yaml
avoidance_margin: 0.12
class_margin_unknown_m: 0.08
corr_half_width_start_m: 0.64
corridor_lateral_reanchor: false
mission_speed_mps: 0.5
mpc_obs_vel_lead_s: 0.0
mpc_obs_vel_trust: 1.0
mpc_obs_wake_horizon_s: 1.0
mpc_obs_wake_weight: 0.0
mpc_obstacle_side_mode: fixed
mpc_prediction_dt_s: 0.1
mpc_return_gain: 1.0
mpc_return_lookahead_ratio: 2.5
mpc_return_max_heading_deg: 35.0
mpc_return_overshoot_max_m: 0.15
mpc_return_psi_scale: 0.5
mpc_w_corr: 3.5714
mpc_w_du_delta: 8.33
mpc_w_obs: 400.0
mpc_w_psi: 4.5
mpc_w_term: 9.0
mpc_w_v: 4.2857
```
