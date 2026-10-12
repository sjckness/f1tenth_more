# Campaign fix — Part A review (final state, 2026-10-12)

Branch `test-campaign`, as pushed. Part A is commits `16b7b95..75ccbe6` (plan, A1, A2, A3, A4 ×2, A5). The review items and decisions that followed are `0b9e8ee..babc885`; see `output/campaign_review_items.md`. Part B, with B5–B8 added from the Orin's R011 analysis, is in `output/campaign_partB_backlog.md`.

Everything was verified on linus only: unit tests, a nominal-model rig, and the scenario harness driving the real `mpc_corr` node. Nothing was run on hardware or in Gazebo.

## Final effective values (compare with the Orin after it pulls)

| Value | Final | Source |
|---|---|---|
| prediction dt | **0.2 s** | `mpc_prediction_dt_sec` |
| control period | **0.1 s** | `mpc_control_period_sec` |
| N | **20** (horizon 4.0 s, 1.6 m at 0.4 m/s) | `mpc_horizon_steps` |
| corridor half-widths | **0.60 / 0.60 m** (width 1.2 m, no funnel) | `corr_wmin` / `corr_wmax` |
| `corr_L_base` | **3.0 m** | `corr_L_base` |
| `corr_lookahead_frac` | **0.85** (lookahead 2.55 m on L 3.0) | `corr_lookahead_frac` |
| `w_du_delta` raw | **8.5714** | `mpc_w_du_delta` |
| `w_du_delta` effective | **3.0** per stage at the 0.1 s step (×7/20); in the solver ×(0.1/step)²: **0.75** stage to stage (0.2 s), **3.0** on the first step (0.1 s control period) | `mpc_solver.rate_weight_factor` |
| `corridor_heading_return` | **false** | `corridor_heading_return` |
| steering bounds | **−0.309 / +0.319 rad**, rate ±0.5 rad/s (hard rows: ×0.2 s stage to stage, ×0.1 s for u₀) | `mpc_steering_angle_min/max_rad`, `MPC_corr.limits` |
| speed | 0 or ≥ 0.4 m/s, max 0.5, reverse 0 | `min_moving_speed_mps`, `max_forward/reverse_speed_mps` |
| keep-outs | 3 footprint circles (centres +0.006 / +0.181 / +0.356 m, r 0.162), committed pass side | `mpc_keepout_footprint_circles`, `pass_side.py` |
| BLOCKED | progress < 0.15 m in 2.5 s; steering lock with < 0.15 m in 2.0 s; infeasible for 1.0 s (plan checked over 2.0 s) | `mpc_blocked_*` |

## Parameters changed (repo value → new)

| Param | File | Old | New |
|---|---|---|---|
| `mpc_control_period_sec` | stack_params | literal `ts` 0.1 | 0.1 |
| `mpc_prediction_dt_sec` | stack_params | literal `ts` 0.1 | **0.2** |
| `mpc_horizon_steps` | stack_params | literal 20 | 20 |
| `corr_lookahead_frac` | stack_params | literal 0.5 | **0.85** (see the lookahead note below) |
| `mpc_w_du_delta` | stack_params | 8.33 | **8.5714** (+ dt scaling in the solver) |
| `corridor_heading_return` | stack_params | true | **false** |
| `mpc_keepout_footprint_circles` | stack_params | — | 3 |
| `mpc_pass_side_match_radius_m` / `_forget_sec` | stack_params | — | 0.5 / 2.0 |
| `mpc_blocked_plan_check_sec` | stack_params | — | 2.0 |
| `corr_wmin` | stack_params | literal 0.4333 | **0.60** |
| `corr_wmax` | stack_params | literal 0.7667 | **0.60** |
| `corr_L_base` | stack_params | literal 3.0 (Orin: 1.5) | 3.0 |
| `mpc_min_speed_all_moves` | stack_params | — | true |
| `mpc_obstacle_keepout_enabled` | stack_params | — | true |
| `mpc_blocked_enabled` | stack_params | — | true |
| `mpc_blocked_progress_window_sec` / `_min_progress_m` | stack_params | — | 2.5 / 0.15 |
| `mpc_blocked_lock_window_sec` / `_lock_fraction` | stack_params | — | 2.0 / 0.95 |
| `mpc_blocked_infeasible_sec` | stack_params | — | 1.0 |
| `mission_translator_wall_default_distance` | stack_params | const 2.0 | **0.5** |
| `mission_translator_wall_default_distance_before_turn` | stack_params | const 3.0 | 3.0 |
| `front_clearance_wall_confirm_*` | stack_params | — | half-angle 30°, window 0.6 m, min inliers 10, min extent 0.8 m, tol 0.30 m + 10 % |
| `wall_stop_require_lidar` | stack_params | — | true |
| `front_clearance_publish_debug_raw` | stack_params | false | **true** |

Read, unchanged: `min_moving_speed_mps` 0.4, `max_forward_speed_mps` 0.5,
`max_reverse_speed_mps` 0.0, `car_radius` 0.20, `obstacle_safety_margin_m` 0.12,
`mpc_steering_angle_min/max_rad` −0.309/0.319, `mpc_wheelbase_m` 0.305,
`swept_clearance_body_*` (front 0.443, rear −0.082, half-width 0.136).

The Orin's parameter snapshots from the 2026-10-08 runs (archived on linus) match the
`1d00853` file for the 12:xx runs. The 11:xx runs used steering limits 0.278/−0.283.
`corr_*` were literals, so the snapshots cannot show the Orin's uncommitted
`corr_L_base` 1.5.

## Numbers

- **Horizon:** 20 × 0.2 s = 4.0 s, which is 1.6 m at 0.4 m/s (was 0.8 m).
- **Solve time on linus (i7-9700K, mean/p95/max):** before 5.98/6.59/12.32 ms with no obstacle and 8.47/8.99/9.51 ms with one obstacle. After: 6.03/6.52/6.97 and 8.49/9.07/9.63 ms. The Orin will be slower. Run `python3 tools/mpc_solve_benchmark.py` there.
- **Lookahead 0.5 vs 0.85.** A1 raised `corr_lookahead_frac` because at 0.5 the lookahead (1.5 m) falls inside the doubled horizon reach, so the reach floor (1.25 × N × dt × vdes) takes over. That floor is 2.0 m during a 0.4 m/s drive move, and the controller logs a geometry WARN every 5 s (34 per harness run). In the final-state harness (both reporting modes), 0.85 gave the same outcomes as 0.5 except:
  - with true-centre reporting, the 0.5 m object appearing late at 1.4 m: BLOCKED at 4.7 s with gap +0.59 m (0.85) vs at 8.0 s with gap +0.17 m (0.5);
  - with front-surface reporting, the 1.0 m object off-centre seen early: BLOCKED (0.85) vs passed (0.5). That case is Part B.

  Clearances on the early-seen passes were comparable: true-centre +0.11 to +0.15 m with 0.85 vs +0.10 to +0.15 m with 0.5 (3 higher, 2 lower, 1 equal). Passes finished about 0.5 s sooner with 0.85, and the warning is gone. **Kept 0.85.**
- **Final scenario harness** (real `mpc_corr`, `tools/run_scenarios.sh`, 16 scenarios × 2 reporting modes): every 0.1 m, 0.3 m and 0.5 m obstacle seen early passes, with min outline gap ≥ +0.11 m. Of the late (1.4 m) cases:
  - the centred 0.3 m object ends BLOCKED with front-surface reporting and passes with true-centre reporting;
  - the centred 0.5 m object ends BLOCKED in both modes;
  - the off-centre 0.5 m object ends BLOCKED with front-surface reporting and passes with true-centre reporting.

  1.0 m objects pass only off-centre with true-centre reporting (Part B). No command in (0, 0.4) m/s and no pass-side flip in any run.
- **Nominal-model closed loop, as at A3 time** (`test_blocked_closed_loop.py`, 1.2 m corridor; superseded by the harness above):

  | Scenario | Result |
  |---|---|
  | Plant (r 0.15) centred, seen ≥ 2.0 m | passes, body gap +0.22 m |
  | Person (r 0.25) centred, seen ≥ 2.0 m | passes, body gap ≥ +0.21 m |
  | Offset obstacles | pass |
  | R011 (plant centred, seen at 1.4 m) | BLOCKED, `no_feasible_plan`, about 1 s after it appears |
  | 1.0 m object centred | BLOCKED |
  | Any scenario | never a command in (0, 0.4) m/s |
  | Pre-campaign stack on R011 | drives into the plant (body gap −0.10 m) |

  The real `MPCController.control_loop` behaved the same in a scratch run with a simulated clock.

## Inferred from the archived 2026-10-08 bags on linus

The run `2026-10-08T12-29-44_mission-llm_2b356aac445f` matches R011. This match is inferred, not confirmed:

- It asked for a median 0.087 m/s (max 0.099) at 0.318 rad of steering for 32.4 s.
- The solver reported "solved" 324/324 times.
- A person appears at t+31.5 s.

From `tools/m02_orin_report.py` on that bag:

- **Obstacle:** r 0.26 m (box width / 2), at (0.91, −0.24) m in base_link.
- **Corridor at the obstacle:** half-width 0.51 m on the left, 0.45 m on the right.
- **Footprint:** d = 0.575 m.
- **Passing left:** needs the axle at +0.39 m, which fits the corridor geometrically. Passing right needs −0.76 m, which does not fit.
- **From where the car stopped,** passing needed 1.13 m of travel at R_min 0.92 m, and 0.09 m was available.
- **Detection lag:** `/camera/detections` and `/camera/detection_masks` arrive 0.44 s (p50) after their stamp, against the 0.28 s the launch file documents. That is the mask-vs-depth staleness B1 has to handle.
- **The bags start after the plant was already in view** (logger late start), so "why it first appeared at 1.4 m" cannot be answered from these bags. The report says so.

## Wall_turn with stale or missing front distance

This was a crash and is fixed in `babc885`: `float(None)` in the corridor record and the log. It is now NaN / "unknown", with a test.

## Not verified

- Gazebo. The test-campaign sim is the old Fortress one, and the camera → YOLO pipeline runs on the Thor.
- Behaviour on the car, real detection noise and latency, every new threshold (all untuned), and the 3B LLM's response to the prompt change.

## Exact commands on the Orin

```bash
cd ~/dev_ws/f1tenth_more && git fetch && git checkout test-campaign && git pull
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mpc_controller f1tenth_behavior \
  f1tenth_perception f1tenth_logger llm f1tenth_params
source install/setup.bash
python3 tools/mpc_solve_benchmark.py --repeat 5
python3 tools/m02_orin_report.py \
  --run R009=~/f1tenth_archive/complete/<R009 run>,<campaign>/M02_*/P###-R009-* \
  --run R010=~/f1tenth_archive/complete/<R010 run>,<campaign>/M02_*/P###-R010-* \
  --run R011=~/f1tenth_archive/complete/<R011 run>,<campaign>/M02_*/P###-R011-* \
  --out ~/m02_report.md
# any new run, after a test:
python3 tools/replay_front_distance.py ~/f1tenth_archive/complete/<run> --csv fd.csv
python3 tools/replay_front_distance.py ~/f1tenth_archive/complete/<run> --estimator median
```

## Procedure (a): parked car, person at 1.0 / 1.2 / 1.5 / 2.0 m, wall about 4 m behind them

1. Park the car square to a wall about 5 m ahead. Start the stack, so `front_clearance_publish_debug_raw` is on, and keep `/scan` running.
2. `ros2 bag record -o parked_person /perception/front_distance /perception/front_wall_confirmed /perception/debug/front_clearance /perception/debug/front_roi_depth /perception/debug/front_roi_class /perception/debug/front_distance_raw /camera/detections /camera/detection_masks /perception/obstacles_2d /scan /tf_static /odometry/filtered`
3. Record 10 s with no person (baseline). Then have the person stand still, centred, for 10 s at each distance: 2.0, 1.5, 1.2 and 1.0 m (measure with tape from the front bumper). Repeat once with the person 0.3 m off-centre.
4. Run `tools/replay_front_distance.py parked_person --csv pp.csv`.
5. Expected:
   - `front_wall_confirmed` stays at the wall distance or −1, never the person's distance.
   - `masked_objects` lists the person.
   - `coverage_excluded` grows as the person comes closer.
   - The gap between `front_distance` and the wall distance is the leak, per distance.

## Procedure (b): mocap precision, objects at 1 / 2 / 3 m

1. Put mocap markers on the car (base_link) and on each object (centre, plus the two lateral edges).
2. Use objects: person, 0.3 m plant, chair. Place each one centred at 1, 2 and 3 m, then 0.5 m left and right. Hold each for 10 s with the car parked.
3. Record `/camera/detections_3d /perception/obstacles_2d /scan /tf_static /odometry/filtered` and the mocap topics.
4. Compare for each placement:
   - The obstacle centre from `/perception/obstacles_2d` (base_link) against mocap. Target ≤ 5 cm.
   - The width (2r) against the mocap edge-to-edge distance. Target ≤ 3 cm.
5. Today's pipeline will miss both targets: the position is the front surface, and the width is the box width. The test is the baseline for B2.

**Stopping here for your review before Part B.**
