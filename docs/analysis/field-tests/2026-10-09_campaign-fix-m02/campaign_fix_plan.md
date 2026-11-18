# Campaign fix plan — M02 R009–R011 (Step 0, from the code only)

Branch `test-campaign` @ `1d00853` (pulled 2026-10-09). Worked in a separate
worktree on linus (`~/dev_ws/f1tenth_more_tc`), so the `jazzy` checkout's
uncommitted work is untouched. No run data here; every number below is from
the code or from `stack_params.yaml` **in this repo** — the Orin may differ
(see the param tables; diff them against the Orin).

Paths: `MPC` = `src/f1tenth_control/mpc_controller/mpc_controller/MPC_corr.py`,
`SOL` = `.../mpc_solver.py`, `YAML` = `src/f1tenth_params/config/stack_params.yaml`,
`FCN` = `src/f1tenth_perception/f1tenth_perception/front_clearance_node.py`,
`CSC` = `src/f1tenth_behavior/f1tenth_behavior/behaviours/check_stop_condition.py`.
"(inferred)" marks reasoning that is not a direct read of the code.

---

## 1. Code-level answers

### 1.1 MPC horizon, dt, control period, warm start

| Item | Value | Where |
|---|---|---|
| Solver | **OSQP RTI** (no acados; nothing code-generated). One linearisation + one QP per tick, dense → sparse, `osqp.OSQP().setup(...)` every tick. SLSQP kept as rollback (`use_rti_solver`, YAML true) | SOL:732, SOL:1118-1154 |
| `N` | 20, bare literal | MPC:728 |
| prediction dt | `self.ts` = 0.1, bare literal | MPC:727 |
| control period | **the same** `self.ts` (timer `create_timer(self.ts, ...)`) | MPC:1859 |
| model | forward Euler `x + ts·f(x,u)`, state = rear axle `[x, y, yaw, v]`, `L` = `mpc_wheelbase_m` 0.305, `lr` 0.17 | `vehicle_model.py`, MPC:837-840 |
| horizon | 20 × 0.1 = **2.0 s → 0.8 m at 0.4 m/s** (1.0 m at the 0.5 default `vdes`) | — |
| rate rows | `dDelta·ts`, `dA·ts`, k = 0 relative to `last_u` | SOL:908-928 |
| speed command | `v_cmd = v_measured + a_0 · ts` | MPC:4053 |
| warm start | `shift_warm_start`: drop stage 0, duplicate last. Assumes one tick == one stage. Kept on success (incl. `solved_inaccurate`/`max_iter`), dropped on failure / new move | SOL:700-729, MPC:3934-3937 |
| x0 | latest odom, no delay compensation | MPC:3770 |
| solve time | published (`/mpc/solver_status.solve_dt_sec`, `/mpc/status` `solve_time_ms`), no benchmark tool exists | MPC:3886-3983 |

Other users of `ts` that mean "physical horizon reach" (`N·ts·vdes`):
lookahead floor `lookahead_clamp_length` (MPC:51-67, 5347-5440), the wall_turn
horizon heading cap (MPC:4336 → `wall_turn.horizon_heading_reach`), the object
heading cap (MPC:4491) and the object arc switch (MPC:4531).

### 1.2 Corridor

| Param | Repo value | Where | Note |
|---|---|---|---|
| `corr_wmin` | 0.4333 (HALF-width at the car) | MPC:874, bare literal | not a ROS param |
| `corr_wmax` | 0.7667 (HALF-width at the far end, rate-limited by `length_ref`) | MPC:875, bare literal | not a ROS param |
| `corr_L_base` | 3.0 | MPC:872, bare literal | you report 1.5 live on the Orin (uncommitted there) |
| `corr_N` | 120 | MPC:873 | |
| `corr_lookahead_frac` | 0.5 | MPC:889 | |
| `corr_lookahead_reach_margin` | 1.25 | MPC:896 | |

**The bounds constrain the rear-axle point (the MPC state), not the footprint.**
Hard two-sided linear row `n·pc − half_w ≤ n·p_{k+1} ≤ n·pc + half_w` at every
stage, no slack (SOL:977-984; SLSQP SOL:1402-1406). `car_radius` is **not**
subtracted anywhere for the corridor. (The costmap wall rows do subtract
`car_radius + avoidance_margin`, soft, SOL:217-225.) So with the shipped
0.4333 half-width the axle can be ±0.43 m off the centreline, the body ~0.2 m more.

### 1.3 Obstacles in the MPC

- Source: `/perception/obstacles_2d` (`Obstacle2DArray` {x, y, r}, base_link) from
  `obstacle_projector_node`, converted to odom on arrival, overwritten per message,
  no tracking (MPC:1654, 3094-3107).
- **Size:** `r = bbox_width/2` (`obstacle_radius_source: footprint`), reject
  outside [0.03, 1.5], + per-class margin (`obstacle_class_margin_m`, `{}`),
  merge in-frame within 0.2 m keeping the max r. No lidar, no class prior.
- **Position:** `detection_3d_node` uses the **median mask depth = the visible
  front surface**, and the box-centre pixel for the bearing (detection_3d_node.py:488-513,
  588-603). The disk of radius r is centred on the front surface, so it bulges r
  *towards* the car and under-covers the back (inferred consequence).
- **Margins** (`car_radius` 0.20, `avoidance_margin` = `obstacle_safety_margin_m` 0.12):

| Mechanism | Formula | Reference point | Double count? |
|---|---|---|---|
| `compute_local_target` (MPC:5521) | `R_safe = r + car_radius + margin` | lookahead target, shift capped 0.30 m | no |
| solver `w_obs` soft cost (SOL:528-601) | `softplus(car_radius + margin − d_front)²`, `d_front = min‖pF − o‖ − r` | front points 0.25/0.40/0.55 m ahead of the rear axle (bumper at 0.443) | **yes**: front points already reach past the bumper, then `car_radius` is added again |
| predicted-clearance warning (MPC:4167-4185, 4020) | `‖p − o‖ − r − car_radius < car_radius + margin` | rear axle | **yes** (warning only) |
| front-surface centring | +r towards the car | — | **effectively yes** (inferred) |

Obstacles are a **soft** cost only (`mpc_w_obs` 8.0, effective 2.8). Part A does
not change the obstacle model (that is B2); A3 adds a collision check on the plan.

### 1.4 Speed floor

- `min_moving_speed_mps` 0.4 (YAML:1105) is enforced **only for go_to_object**:
  `object_speed_decision` + `floor_moving_speed` (MPC:3602-3621, 4054-4058) and the
  mission loader. Not for drive/straight, wall_turn, goal_distance, goal_pose, turn.
- Published speed = `v_measured + a_0·0.1` (MPC:4053), then only
  `clamp_drive_speed` to [−0.0, +0.5] (drive_limits.py).
- **Why 0.09 m/s on a straight move:** there is no floor; with the car stopped
  (odom reads 0 below the VESC 500-ERPM deadband) the command is just `0.1·a_0`,
  and `a_0` is rate-limited to ±0.2 m/s² per tick. When the obstacle cost and the
  hard ±0.43 m corridor rows make slowing the cheapest option (R011: plant
  centred, full left lock already), the QP settles on a small positive `a`
  → 0.09 m/s, below the ~0.12–0.14 m/s needed to move. Nothing detects that the
  car is not moving; only the per-move `timeout_sec` (80 s from the translator,
  plan_translate.py:572-575) ends it (inferred mechanism, consistent with the R011 description).
- Reverse: QP allows `vMin` −1.0 (MPC:1320); only the publish clamp (`max_reverse_speed_mps` 0.0) stops it.

### 1.5 front_distance pipeline (FCN)

- Publisher `front_clearance_node`, `std_msgs/Float32`, one per depth frame (~17 Hz);
  inputs: depth `/zed2/zed_node/depth/depth_registered`, `/camera/detections`,
  `/camera/detection_masks`. Latest-message caching, no message_filters.
- **ROI**: centred 180×70 px (`front_clearance_roi_half_width_px` 90, `_half_height_px` 35).
- **Masking**: all classes, label image `> 0` (box fallback when mask/detection stamps differ),
  no dilation, no class filter, no min area. Per-pixel release EMA
  (`mask_time_constant` 0.35 s → ~0.24 s hold), `max(raw, ema)`.
- **Stamps**: mask vs detections exact match only. **The mask stamp is never compared
  with the depth frame stamp**, and has no age limit.
- **Estimator**: mean of valid depths between the 5th and 50th percentile
  (`background_percentile_low/high` 5/50), ≥ 60 valid px else hold.
- **EMA** α 0.25. **−1.0** = never seeded, or ≥ 2 % of the ROI reads −Inf (too close).
- Consumers: mission `front_clearance` guard (`0 < d < distance`, 3-tick debounce,
  CSC + condition_eval.py:326-374); MPC wall_turn (age-gated).

**Leak paths, ranked (from the code):**
1. Depth bleeding past an undilated mask edge (stereo smear, PERFORMANCE depth mode);
   those pixels are counted as background **and land in the near half the estimator averages**.
2. Mask geometry stale vs depth (~278 ms detection lag documented in detection.launch.py:65,
   no stamp check). Approaching → silhouette grows → new pixels unmasked.
3. Wall depth holes shrink the background count, so a fixed leak becomes a larger fraction.
4. Incomplete segmentation (limbs, gaps); no box ∪ mask fallback.
5. Detection drop-outs > 0.24 s or score < 0.3.
6. Unclassified objects (never masked; by design).
The estimator amplifies every leak: with a 4 m wall, 30 % of valid ROI pixels on a
person at 1.5 m reads ≈ 2.6 m, ≥ 41 % reads < 2.0 m (inferred arithmetic). R009's
stop at ~1.5 m is consistent with 1+2.

### 1.6 Object merging (duplicates)

`f1tenth_costmap/semantic_layer.py` `update_tracks_batch`: gate 0.5 m + 2σ against the
predicted position, same class only, Hungarian, EMA α ≤ 0.3, confirm 3 hits, delete
after 5 missed frames, IDs never reused, **no track–track merge**. Duplicates come from:
no track merge + one-to-one matching lets two tracks share one object (ping-pong, neither
reaches 5 misses); two detections of one object in a frame; class flicker; map-pose jumps
(nearest-in-time EKF pose, not interpolated) beyond the gate; velocity extrapolation of
static objects; front-surface position shifting with the viewing angle. Only go_to_object
consumes these tracks; MPC obstacles and front_distance masking do not (they are per-frame).
Replacement is B3.

### 1.7 Mission end / BLOCKED plumbing

No stuck detector exists. No generic MPC → executor failure channel (only
`target_behind_terminal` for go_to_object). BT-internal aborts publish an **empty reason**
on `/test/mission_event` (loader.py:712-745), which the campaign logger turns into
"mission aborted".

---

## 2. What Part A changes (one commit each)

### A1 — prediction dt 0.2 s, control period 0.1 s
Files: MPC (`__init__`, `control_loop`, the four reach call sites), SOL
(`shift_warm_start`, `_solve_rti` k = 0 rate rows), YAML, mpc_corr.launch.py.
- New params `mpc_control_period_sec` 0.1, `mpc_prediction_dt_sec` 0.2,
  `mpc_horizon_steps` 20 (were the literals 0.1/0.1/20).
- Timer, `v_cmd = v + a·T_ctrl`, `t_mpc`, the k = 0 rate rows and the "solver slower than
  the period" check use the control period; the dynamics, stage rate rows and the physical
  reach (`N·dt·v`) use dt.
- `shift_warm_start(z, N, shift=T_ctrl/dt)`: linear interpolation of the control sequence
  at `t + T_ctrl` (half a stage at 0.1/0.2); `shift=1` reproduces the old behaviour exactly.
- Horizon: 20 × 0.2 = 4.0 s → **1.6 m at 0.4 m/s** (2.0 m at 0.5).
- Consequence: the lookahead floor becomes 1.25·4.0·0.5 = 2.5 m > 0.5·L = 1.5 m, so
  `corr_lookahead_frac` 0.5 → 0.85 (param) to keep the target outside the reach without a
  WARN every 5 s. The wall_turn/object heading caps double with the reach (physically
  correct, but changes wall_turn trigger/cap — flagged for Gazebo/car).
- New `tools/mpc_solve_benchmark.py`: solve time mean/p95/max, before vs after.

### A2 — wide fixed corridor
Bounds are on the rear axle ≈ the car "centre" case → half-width ≥ 0.25 + 0.20 + 0.12
= 0.57 → **0.60 (width 1.2 m)**, no funnel. Make the literals ROS params read from YAML:
`corr_wmin` 0.4333 → 0.60, `corr_wmax` 0.7667 → 0.60, `corr_L_base` 3.0 → 3.0 (Orin live
1.5 → 3.0), `corr_lookahead_frac` (A1).

### A3 — speed 0 or ≥ 0.4, stuck → BLOCKED
- MPC: on every moving move the published speed is `max(v_cmd, 0.4)`; a move speed in
  (0, 0.4) is raised to 0.4 (logged); zero only from the stop paths. The QP gets a hard
  `v_{k} ≥ 0.4` row from the first stage reachable under the accel/jerk limits
  (`v_floor`), so the plan it returns is one driven at ≥ 0.4.
- New `mpc_controller/blocked.py` (pure): BLOCKED when, while a move is active,
  (a) commanded ≥ 0.4 but odom path < `mpc_blocked_min_progress_m` 0.15 m in
  `mpc_blocked_progress_window_sec` 2.5 s; (b) non-turn move at ≥ 95 % steering lock
  with < 0.15 m progress along the corridor heading in 2.0 s; (c) solve failed or the
  ≥ 0.4 plan's front points enter an obstacle disk for `mpc_blocked_infeasible_sec` 1.0 s.
  BLOCKED latches: speed 0, steering held, `/mpc/blocked` (String JSON, reason + details)
  every tick until the next move. Reverse untouched (clamp 0.0).
- Behaviour: CSC subscribes `/mpc/blocked`, aborts the mission (hold, move outcome
  `mpc_blocked:<reason>`, summary `final_state: BLOCKED`), and the abort reason now reaches
  `/test/mission_event` (fixes the empty reason for every BT-internal abort). Campaign logger
  records it as outcome `aborted`, reason `BLOCKED: …` (no new mission state, so no logger
  outcome schema change).

### A4 — wall stop
- `wall_stop_default_distance_m` 0.5 (YAML) replaces `WALL_THRESH_DEFAULT_M` 2.0;
  `wall_stop_default_distance_before_turn_m` 3.0 (unchanged value, now a param).
  `stop_condition.distance_source` = `llm` / `default` / `default_before_turn` in the
  mission JSON; logged at move start. Prompt: stop telling the model "default 2.0 m"; tell it
  to leave `thresh` out when the user gives no distance (so the default is applied and
  recorded as such).
- Lidar confirmation: new pure `f1tenth_perception/wall_confirm.py` + hook in FCN:
  RANSAC line on the forward `/scan` sector, accepted only if the inlier segment is
  ≥ `wall_confirm_min_extent_m` 0.8 m long (a person/plant is not), crosses the forward
  bearing, and its distance agrees with the camera distance within
  `wall_confirm_tolerance_m` 0.30 m. FCN publishes `/perception/front_wall_confirmed`
  (Float32, −1.0 when not confirmed) + a JSON debug topic. CSC's wall guard reads it when
  `wall_stop_require_lidar` is true (default true).

### A5 — logging + replay
- `front_clearance_publish_debug_raw` false → true; FCN debug JSON per frame: ROI mask
  coverage, masked detection indices/classes/scores, mask-vs-depth stamp delta, estimator
  outputs, lidar confirmation.
- Campaign logger: new `front.csv` (front_distance, raw, bg count, coverage, stamp delta,
  confirmed), `blocked` event, extra solver columns (success, status_message); corridor
  bounds are already in `corridors.jsonl` (`defn.w0/w1`, now 0.6). Mission bag topics +
  the front-distance/debug/blocked topics + an ROI depth crop (`/perception/debug/front_roi`,
  ~50 KB/frame) so a bag can be replayed.
- `tools/replay_front_distance.py` (recompute from a bag with the ROI crop) and
  `tools/m02_orin_report.py` (run on the Orin over R009–R011).
