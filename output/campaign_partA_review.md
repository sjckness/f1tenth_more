# Campaign fix — Part A review (2026-10-09)

Branch `test-campaign`, commits `16b7b95..75ccbe6` (plan, A1, A2, A3, A4 ×2, A5).
Everything below was verified on linus only. Nothing was run on hardware or in Gazebo.

## Parameters changed (repo value → new)

| Param | File | Old | New |
|---|---|---|---|
| `mpc_control_period_sec` | stack_params | literal `ts` 0.1 | 0.1 |
| `mpc_prediction_dt_sec` | stack_params | literal `ts` 0.1 | **0.2** |
| `mpc_horizon_steps` | stack_params | literal 20 | 20 |
| `corr_lookahead_frac` | stack_params | literal 0.5 | **0.85** |
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
- **Nominal-model closed loop** (`test_blocked_closed_loop.py`, 1.2 m corridor):

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
