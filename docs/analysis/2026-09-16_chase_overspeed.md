# Reverse and overspeed in mpc_corr's /drive (2026-09-16)

Why `max_forward_speed_mps` / `max_reverse_speed_mps` exist, and what chose
their defaults. Rig numbers are reproduced by
`docs/analysis/chase_overspeed_ablation.py` (nominal model, no hardware).

## Bounds as resolved

| Quantity | Value | Where |
|---|---|---|
| Solver speed box on every predicted state | vMin −1.0, vMax 3.0 m/s | `MPC_corr.py` `self.limits` (literals), applied at `mpc_solver.py` `row_v` |
| Solver acceleration box | a_min −2.0, a_max 3.0 m/s² | same dict; `mpc_solver.py` `row_a` |
| Solver acceleration rate | dAMin/dAMax ±2.0 m/s³ × ts | same |
| Speed reference | soft cost `w_v·(v − vdes)²`, effective w_v 1.5 | `mpc_solver.py` velocity-tracking block |
| Published speed | `v + a·ts`, **no clamp** before this change | `MPC_corr.py` control loop |
| `v_min`/`v_max` in stack_params.yaml (−0.5/+0.5) | belong to `f1tenth_control/mpc.launch.py`, a different node | — |
| ackermann_mux | priority arbitration, no speed limit | `f1tenth_bringup/config/mux.yaml` |
| ackermann_to_vesc | velocity correction passes through above its top knot | `vesc_ackermann/src/ackermann_to_vesc.cpp` |
| vesc_driver | ERPM clip ±23250 = **±4.23 m/s** at gain 5499.27, reverse included | `vesc_driver.cpp` `setSpeed(speed_limit_.clip(...))`, `config/vesc.yaml` |
| Safety nodes | IsProximityTooClose trips on 0.15 m contact range; nothing limits speed | `f1tenth_behavior` |

## Requested speeds (nothing requests reverse or more than 0.5 m/s)

- Missions (HEAD): drive 0.3–0.5, turn 0.3–0.5, go_to_object 0.4.
  `test_03_bounce_walls.json`'s `goal_distance: -5.0` does not reverse: the
  goal_distance check `traveled >= goal_distance` is true on the first tick.
- LLM translator: `mission_translator_speed_straight` 0.4, `_turn` 0.5.
- mpc_corr's own `vdes` 0.5.
- Not through mpc_corr, so not clamped: joystick teleop (`joy_teleop.yaml`
  scale 5.0, mux teleop lane), `calibration_drive`, `twist_to_ackermann_node`,
  hand scripts publishing `/drive`.

## What the car was actually commanded (archive, read-only)

101 runs in `~/f1tenth_archive/complete/*/*.extract.parquet` carry `/drive`
samples. Overall min −0.231, max 1.206 m/s.

- Reverse (< −0.01 m/s) in 13 runs, deepest −0.231 in
  `2026-09-14T13-32-08_mission-wall_turn`; also 2026-09-07T12-39-23 and
  12-48-40 bottle_then_person, 09-09T14-49-58 llm_plan, 09-10T15-35-24
  straight_then_wall_turn_3m, 09-10T15-56-51 wall_turn, 09-11T14-03-25
  drive_stop_2m_from_wall, 09-11T14-14-24 llm_plan, 09-14T13-54-42 /
  14-05-47 / 14-07-58 wall_turn, 09-15T09-20-04 / 09-42-05 llm.
- Above 1.0 m/s in 3 runs: `2026-09-08T11-41-39_mission-bottle_then_person`
  (1.183), `2026-09-09T14-41-09_mission-llm_plan_1788964869` (1.206),
  `2026-09-09T14-49-58_mission-llm_plan_1788965397` (1.194).

Every one of those ran against references ≤ 0.5 m/s: the solver chose them.

## Rig: what drives it (unbounded lateral chase, footprint person, standoff 1.0)

| run | peak forward m/s | peak reverse m/s | ticks > 1.0 | ticks < −0.01 |
|---|---|---|---|---|
| baseline (footprint, standoff 1.0) | +2.074 (tick 167) | −1.043 (tick 196) | 21 | 90 |
| w_obs = 0 | +1.616 (tick 239) | −1.016 (tick 214) | 33 | 65 |
| no obstacle in the list | +1.616 (tick 239) | −1.016 (tick 214) | 33 | 65 |
| w_term = 0 | +1.319 (tick 293) | −0.920 (tick 204) | 17 | 23 |
| w_corr = 0 | +2.160 (tick 161) | −1.066 (tick 312) | 63 | 130 |
| **w_psi = 0 and w_psi_stage = 0** | **+0.756 (tick 12)** | **none** | 0 | 0 |
| w_v × 10 | +0.948 (tick 138) | none | 0 | 0 |
| legacy radius | +0.756 (tick 12) | none | 0 | 0 |
| baseline + /drive clamp (+1.0 / −0.0) | +1.000 (tick 289) | none | 0 | 0 |

At the peak (tick 167) the speed reference was 0.00 (inside the standoff),
the car's yaw was +87° and the bearing to the person −151°.

**Inference.** The heading costs drive it. With the steering at its bound,
yaw rate is v·tan(δ)/L, so speed is the only remaining way to rotate toward
the held heading, and reverse rotates the other way. Against an effective
w_v of 1.5, the heading terms (w_psi 4.5 terminal, w_psi_stage 1.67 per
stage) win. Removing the obstacle cost lowers the peak but does not remove
it; removing the heading terms does.

## Change

One clamp at `MPC_corr._publish_drive` (`drive_limits.clamp_drive_speed`),
defaults +1.0 / 0.0 m/s: twice the highest requested speed, and no reverse,
which no path requests. Clamp events go out on `/mpc/drive_clamp`
(`f1tenth_messages/DriveClamp`). The solver's own bounds are unchanged, so on
a clamped tick the plan assumes a speed the car will not fly; the next tick
starts from measured odometry. That mismatch is not addressed here.
