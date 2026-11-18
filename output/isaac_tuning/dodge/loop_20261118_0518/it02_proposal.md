- **`mpc_prediction_dt_s` 0.1 → 0.15.** With N = 20 the horizon grows from 2.0 s to 3.0 s, so the car sees 1.5 m ahead at 0.5 m/s instead of 1.0 m. The first level sits 1.5 m ahead, so today it starts beyond the horizon. Nearly every failure shows little or no swerve before contact while steering never reached full lock (steer at limit 0.00 in D00_s1, D01_s2, M1, M3 and most S4 runs). S4 e-stopped with 0.00 to 0.07 m lateral, and M2 collided with only 0.06 m. That pattern points to a late start, not a lack of authority. Iteration 1 raised w_obs with no effect, which supports this. The far lookahead floor also moves out, so the terminal pull fights a swerve less. CPU stays the same because N is unchanged.
- **`mpc_obs_vel_lead_s` 0.0 → 0.2.** The disc velocity and position lag about 0.2 s (EMA at 10 Hz). At 0.2 to 0.3 m/s that is 4 to 6 cm of position error. The moving-obstacle misses run 0.06 to 0.24 m short of the margin (D01_s2, M1, M2), and the predicted discs trail the real cylinders by exactly this lag. 0.2 s is the documented lag value and only half the allowed maximum, which limits overshoot when a cylinder bounces off a wall.

```yaml
avoidance_margin: 0.12
class_margin_unknown_m: 0.0
corr_half_width_start_m: 0.64
corridor_lateral_reanchor: false
mission_speed_mps: 0.5
mpc_obs_vel_lead_s: 0.2
mpc_obs_vel_trust: 1.0
mpc_obs_wake_horizon_s: 1.0
mpc_obs_wake_weight: 0.0
mpc_obstacle_side_mode: fixed
mpc_prediction_dt_s: 0.15
mpc_return_gain: 1.0
mpc_return_lookahead_ratio: 2.5
mpc_return_max_heading_deg: 35.0
mpc_return_overshoot_max_m: 0.15
mpc_return_psi_scale: 0.5
mpc_w_corr: 3.5714
mpc_w_du_delta: 8.33
mpc_w_obs: 200.0
mpc_w_psi: 4.5
mpc_w_term: 9.0
mpc_w_v: 4.2857
```
