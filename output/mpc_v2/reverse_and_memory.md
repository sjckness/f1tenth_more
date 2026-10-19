# Reverse manoeuvre, object memory, BT integration, LLM start — report

Branch `mpc_corr_v2_reverse`. The base is `868ec8a` (origin `campaign-final` on
2026-10-19). The Orin's newer `campaign-final` / `campaign-final-doors` are not in this
base, and this branch is to be rebased onto `campaign-final-doors`. Each part is one
self-contained commit, in order: MPC v2 (4 commits, `output/reverse/report.md`), **A**,
**B**, **D**, **C**.

Everything below was built and tested on **linus**: Ubuntu 24.04, ROS 2 Jazzy, Python 3.12,
i7-9700K. That means colcon builds, unit tests and offline benchmarks only. **Nothing ran
on the car, on the Orin or with the stack launched.**

---

## Part A — Reverse MPC with automatic arbitration

### In one paragraph

The forward MPC drives, exactly as before. When it is **stuck** continuously for 0.5 s, a
separate **rear solver** takes over if two more things hold:

- the strip behind the rear bumper is **free and known** in the costmap, with no object in
  it, and the rear solver is feasible;
- the behaviour tree **allows it** on `/bt/allow_reverse`.

The car then stops, reverses 0.5 m along a frozen rear corridor at a fixed −0.4 m/s, stops
again, and the forward MPC resumes. Raw costs are never compared: the decision rests on
"stuck", "ready" and "allowed", and nothing else.

After 3 reverses that do not get the car past the point where it was stuck, it gives up: it
holds zero speed, reports `stuck`, and does not reverse again until a new move.

**On the car, nothing happens until two things are set:**

- `/bt/allow_reverse` is published. That is Part D's job; this part only consumes it.
- `max_reverse_speed_mps` is raised to ≥ 0.4. It is 0.0 today, and the rear side then
  reports `reverse_speed_limit`.

### A0 — Audit

**The forward MPC after v2.**

- **Model and linearization.** A kinematic bicycle with slip angle
  (`vehicle_model.py:9-44`). RTI linearizes it once per tick around the warm start rolled
  out from the current state (`mpc_solver._solve_rti`). Model and Jacobian are
  sign-agnostic, so nothing in them assumes v > 0 (`output/reverse/recon.md` §2).
- **Sign of v.** The forward QP's speed box is still `vMin −1.0 / vMax 3.0`
  (`MPC_corr.py:1380`). The `/drive` clamp at `max_reverse_speed_mps` 0.0 is what keeps the
  car from reversing.
- **v2.** v2 added a per-stage sign schedule behind `allow_reverse` (default off). With it
  off, the solve is bit-identical to `868ec8a` (`test_reverse_flag_off_identity.py`).
- **Corridor builder.** `build_straight_corridor` runs forward from the car's pose, rebuilt
  every `corridor_update_period` (1 s).
- **Warm start.** The previous solution shifted by one control period (`shift_warm_start`,
  `MPC_corr.py:4372`). It is dropped on a failed solve.
- **Rate.** 10 Hz control, 0.1 s prediction step, N = 20, so a 2.0 s horizon.

**Rear bumper offset.**

- There is no `base_rear` frame in the URDF. `stack_params.yaml` says so ("The description
  has no measured rear bumper").
- The only number was `swept_clearance_body_rear_x_m: -0.082`, from the URDF chassis-mesh
  bounds, not a tape measurement.
- **Added `car_rear_offset_m: 0.082` with a TODO to tape-measure it.** It is now the single
  source for the rear corridor, the rear pre-check, the rear stop margin and v2's rear
  probes. The swept-clearance key stays for its own consumer.
- **⚠ Flag: unmeasured.**

**How the MPC knows the mission and move status.**

- **`/mpc/hold`** (Bool) is the BT's stop. AdvanceMove publishes it when a move is terminal,
  on abort and on timeout; `hold_callback` freezes the control loop (`MPC_corr.py:2078`).
  Every mission stop condition (CheckStopCondition, for example `front_clearance` for
  "approach the wall") reaches the MPC this way, or as the next move's goal.
- **The goal topics** (`/mpc/goal_distance`, `/goal_pose`, `/goal_turn`, `/goal_drive`,
  `/goal_object`): each new goal calls `_invalidate_move_state`.
- **The MPC's own arrival flags** (`goal_reached`, `pose_goal_reached`). They publish
  `/mpc/goal_reached` and return early (`:4064-4156`).
- **`/mission/status`** (MissionStatus: IDLE / LOADED / RUNNING / HOLDING / COMPLETE /
  ABORTED, transient-local) was **not** read by MPC_corr before this commit. Now it is: only
  RUNNING counts as an active mission, every other state is an intentional stop, and
  `emergency_stop_active` aborts a reverse (`_mission_status_cb`, `:3606`).
- **The BT tree** is root `Selector[emergency, handle_obstacle, navigation]`
  (`behavior_executor_node.py:499-574`). **There is no stuck branch.** Nothing publishes
  `/bt/allow_reverse` yet; that is Part D.

### What was built

| Piece | Where | What |
|---|---|---|
| Supervisor | `mpc_controller/reverse_manoeuvre.py` `ReverseSupervisor` (`:394`) | The state machine: `forward` → `stopping` → `reverse` → `stopping` → `forward`, plus `stuck`. MPC_corr calls `before_forward` (`:466`) before its forward solve and `after_forward` (`:498`) after it (`MPC_corr.py:4315`, `:4538`). |
| Stuck detector | `StuckDetector` (`:328`) | Any of three conditions on **every** tick for `stuck_time_s`: (a) forward solve failed or fell back; (b) mission RUNNING, \|v\| < 0.05, something within `stuck_front_dist_m` of the front bumper; (c) the forward plan's predicted displacement < `stuck_progress_m`. One tick with none, or a tick the detector does not see (the node's early returns: hold, goal reached), resets it. An intentional stop holds it at 0. |
| Rear corridor | `rear_corridor` (`:139`) | Starts at the **rear bumper** and runs `rear_corridor_length_m` straight back. Half-width grows from the forward start half-width to ×2.3/1.3 at the far end (the forward rule, not a rectangle). Frozen at reverse start and **anchored in map**, re-expressed each tick through the current map→odom. Arrays are flipped so the tangents point the way the car faces: heading costs hold the car's heading, and psiRef = heading. |
| Rear solver | `RearSolver` (`:254`) | The forward QP at a **fixed** −0.4 m/s: acceleration pinned at 0, every stage's speed box [−0.4, −0.4], so steering is the only decision. Same model, steering limits and steering rate. Own horizon N = ceil(0.5 / (0.4 · 0.2)) = **7** with its own 0.2 s step; own weights `mpc_rear.*` (copied); own warm start. Rear-bumper `w_obs` probes on every stage, front ones kept. Same boundary rows as forward. |
| Pre-checks | `rear_strip_check` (`:201`), every tick | The strip the body sweeps behind the bumper: `rear_corridor_length_m` long, body half-width + 0.10 m to each side. It must be **FREE and KNOWN** in the costmap (`rear_costmap_topic`, default `/slam/map`; −1 is not free, which gives `rear_unknown`), with no object disk reaching into it (`rear_object`; the MPC's obstacle list, plus semantic memory in Part B). No map→odom edge to place the car on the map counts as `rear_unknown`. |
| Lazy solver | `after_forward` | The rear solver runs only while the stuck timer runs; it is reset (warm start dropped) when the timer clears. `rear_ready` = pre-checks OK **and** a feasible rear solve within budget on the latest tick. `rear_solver_mode: always` runs it every tick (timing only). |
| Arbitration | `after_forward` | Switch only if stuck **and** rear_ready **and** `/bt/allow_reverse` true and < 0.5 s old **and** ≥ `forward_min_time_s` of forward since the last reverse **and** fewer than 3 attempts. Both gear changes command 0 and wait for \|v\| < 0.05 for 0.3 s. The reverse is committed. |
| Terminations | `_reverse_tick` (`:628`), `before_forward` | First wins, then zero speed and back to forward: `distance_done` (cap = corridor length); `rear_clearance` (free distance behind < `rear_stop_margin_m` + braking distance); `rear_infeasible` / `rear_over_budget` twice in a row; `timeout` (2 × expected); `abort` (hold, `emergency_stop_active`); `bt_withdrawn` (permission false or stale mid-reverse). |
| Give-up | `after_forward` | 3 consecutive reverses without 0.3 m of progress **past the point where it was stuck** give mode `stuck`: zero speed, `last_termination_reason: max_attempts`, no more reverses until a new move (`new_move`, from `_invalidate_move_state`). |
| One solver drives | node hooks | On a manoeuvre tick the forward solver is **not run** (`before_forward` returns a command and the loop returns). Its warm start stays as it was and is re-rolled from the current state when forward resumes. `last_u` is set to the manoeuvre's steering, so the forward rate row starts from what was actually commanded. |
| Manual override | `/mpc/request_reverse` (`_request_reverse_cb`, `:3655`) | `RequestReverse.srv`: `distance_m` (≤ 0 means the corridor length; capped at it) → `accepted`, `reason`. Same pre-checks, BT permission and speed-limit check; no stuck condition. |
| Diagnostics | `/mpc/reverse_status` (`ReverseStatus.msg`), every tick | mode, reverse_active, stuck + reason + timer, intentional_stop, rear_ready + reason, rear_solver_running, rear solve ms, rear clearance, BT permission + freshness, attempts, last switch time, last termination reason, requested and travelled distance. |

**Interpretation choices** (each is a parameter or a one-line change if you meant otherwise):

- **"Rear corridor area + half-width + margin"** is read as the strip the body sweeps (body
  half-width + `rear_check_margin_m`) along the rear corridor's centre line. The whole
  corridor polygon (0.64 → 1.13 m half-width) would refuse every reverse in a 1.3 m hall.
- **"Forward progress ≥ 0.3 m"** is measured past the pose where the car was stuck. Driving
  back up to the same obstacle after a reverse is not progress. Measured from the reverse's
  end instead, the attempts never accumulated: the harness reversed forever.
- **The stop margin includes the braking distance** (`rear_brake_delay_s` 0.144,
  `rear_brake_decel_mps2` 1.5: the fit to the archived forward stops). Without it the car
  ended 2 cm from an obstacle that appeared 0.30 m behind, against a 0.15 m margin.
  Reverse braking is unmeasured.
- **`/bt/allow_reverse` going false or stale during a reverse ends it**, the same as a hold.
- **`max_reverse_speed_mps` is not raised by this commit.** It is the one authority on
  reverse speed, and raising it is an operator decision. Below `v_rev_mps` the rear side
  reports `reverse_speed_limit`.
- **One reverse mechanism at a time.** With `allow_reverse` (v2) on, the manoeuvre is
  disabled (logged).

### New parameters (`stack_params.yaml`)

| Key | Default | Meaning |
|---|---|---|
| `car_rear_offset_m` | 0.082 | rear bumper behind base_link — **unmeasured, TODO tape** |
| `reverse_auto_enabled` | true | start reverses automatically (still gated) |
| `rear_solver_mode` | `lazy` | `lazy` / `always` |
| `rear_corridor_length_m` | 0.5 | rear corridor length, distance cap |
| `rear_corridor_half_width_m` | −1.0 (unset) | ≤ 0 means `corr_half_width_start_m`; far end × 2.3/1.3 |
| `v_rev_mps` | 0.4 | fixed reverse speed (the operating floor) |
| `mpc_rear.prediction_dt_s` | 0.2 | rear step; gives N_rear = 7 |
| `mpc_rear.w_*` (11 keys) | copies of `mpc_w_*` | rear weights |
| `stuck_time_s` | 0.5 | continuous stuck time |
| `stuck_speed_mps` | 0.05 | "stopped" |
| `stuck_front_dist_m` | 0.6 | condition (b); just above where w_obs parks the nose (0.47 m from the bumper) |
| `stuck_progress_m` | 0.05 | condition (c) |
| `gear_change_dwell_s` | 0.3 | standstill dwell at both gear changes |
| `forward_min_time_s` | 2.0 | forward time between reverses |
| `reverse_max_attempts` | 3 | |
| `reverse_progress_reset_m` | 0.3 | |
| `rear_stop_margin_m` | 0.15 | |
| `rear_brake_delay_s` / `rear_brake_decel_mps2` | 0.144 / 1.5 | braking distance added to the margin |
| `rear_check_margin_m` | 0.10 | strip margin beyond the body |
| `rear_costmap_topic` | `/slam/map` | OccupancyGrid for the pre-check |
| `rear_costmap_free_max` | 50 | free cell threshold |
| `rear_solver_budget_s` | 0.05 | over this a rear solve counts as a failure |
| `reverse_timeout_factor` | 2.0 | |
| `bt_allow_reverse_max_age_s` | 0.5 | |

### Message and service changes (`f1tenth_messages`)

- `msg/ReverseStatus.msg` (new), on `/mpc/reverse_status`.
- `srv/RequestReverse.srv` (new), on `/mpc/request_reverse`.
- Built with `colcon build --symlink-install --packages-select f1tenth_messages` before
  `mpc_controller`.

### Tests (A8), `test/test_reverse_manoeuvre.py` (35 tests, all pass)

**Closed loop** (`test/reverse_manoeuvre_harness.py`): the real forward MPC on a
goal_distance move, the real rear solver, and a first-order speed plant (τ 0.25 s) so gear
changes take real time.

| A8 case | Test | Result |
|---|---|---|
| stuck against a wall → reverse 0.5 m → forward resumes | `test_stuck_against_a_wall_reverses_half_a_metre_then_forward_resumes` | stuck at x 1.65, reverse 0.50 m, `distance_done`, forward solver drives again |
| 3 failed attempts → stuck, no 4th | `test_three_reverses_without_progress_then_stuck_and_no_fourth` | 3 reverses, then mode `stuck`, `max_attempts`, zero speed |
| `forward_min_time_s` respected | `test_forward_min_time_is_respected` | ≥ 2.0 s between resuming forward and the next switch |
| only one solver drives | `test_only_one_solver_drives_each_tick` | the forward solver never runs on a reverse tick |
| gear change waits for \|v\| < 0.05 + dwell | `test_gear_change_waits_for_standstill_and_the_dwell` | 0.3 s of zero commands at standstill before the first reverse command |
| normal driving → rear solver never called | `test_normal_driving_never_runs_the_rear_solver` | 0 rear solves |
| (no BT permission → no change) | `test_without_bt_permission_the_supervisor_changes_no_command` | command sequence identical to supervisor off |
| wall behind → stop margin | `test_a_wall_appearing_behind_ends_the_reverse_at_the_stop_margin` | `rear_clearance`, the car stays > 5 cm from it |
| steering sign in reverse | `test_steering_brings_a_lateral_offset_back_while_reversing` (+ v2's `TestReverseKinematics`) | right steer for a left offset; heading and offset move the correcting way |

**State machine**, scripted inputs with a stub rear solver:

- stuck 0.5 s → rear warmed and switch on time (tick 5);
- stuck clears at 0.3 s → rear solver stopped, warm start dropped, no reverse;
- infeasible on alternating ticks for 2 s → no reverse;
- "approach the wall" done at 0.3 m (intentional) → no reverse;
- mission paused → no reverse;
- unknown cell / object behind → `rear_unknown` / `rear_object`, no reverse;
- high forward cost but not stuck → no reverse (the supervisor has no cost input at all);
- stale / false BT permission → no reverse, and the manual request is refused the same way;
- drive clamp below `v_rev_mps` → `reverse_speed_limit`;
- manual request caps the distance and uses the same checks;
- two infeasible / over-budget rear solves → end;
- timeout;
- hold aborts;
- BT withdrawal aborts;
- rear corridor stays put in map through a map→odom jump;
- a new move lifts `stuck`.

**Plus** the geometry and costmap tests: rear horizon (7, clamps), the corridor starts at
the bumper and widens like the forward ones, map↔odom round trip, strip checks for free /
unknown / occupied / no-grid / object, and the strip checked in the map frame.

### A7 — Timing

**linus**, `test/bench_reverse_manoeuvre.py --n 1000`; raw output in
`output/mpc_v2/bench_reverse_manoeuvre_linus.json`.

| Configuration (1000 synthetic ticks each) | p50 | p95 | max | rear solves |
|---|---|---|---|---|
| forward solve alone | 5.64 ms | 6.15 ms | 6.95 ms | — |
| rear solve alone | 2.52 ms | 2.75 ms | 3.30 ms | — |
| **tick, normal driving (lazy, forward only)** | 5.71 ms | 6.44 ms | 7.82 ms | 0 |
| **tick, stuck window (forward + rear)** | 8.45 ms | 9.23 ms | 11.83 ms | 1000 |
| **tick, `always` mode** | 8.47 ms | 9.20 ms | 10.26 ms | 1000 |
| forward corridor build | 0.18 ms | 0.23 ms | 0.35 ms | — |
| rear corridor build | 0.15 ms | 0.20 ms | 0.40 ms | — |

A "tick" here is corridor + target + forward solve + supervisor (pre-check and rear solve).
ROS callbacks, logging and publishing are not in it.

- **Budget check:** p95 and max < 100 ms on every tick configuration — **PASS** on linus.
  The lazy mode costs normal driving 0.07 ms (the pre-check).
- **To rerun on the Orin:**

  ```
  source /opt/ros/humble/setup.bash && source ~/dev_ws/f1tenth_more/install/setup.bash
  cd ~/dev_ws/f1tenth_more/src/f1tenth_control/mpc_controller
  python3 test/bench_reverse_manoeuvre.py --n 1000 --json /tmp/bench_orin.json
  ```

  It prints `BUDGET … PASS/FAIL` and exits non-zero on FAIL.
- Even at 5× linus, the stuck-window tick would be about 46 ms p95. That is an estimate;
  the Orin was not measured.

### Open TODOs and risks (Part A)

- **Rear bumper offset is unmeasured** (`car_rear_offset_m` 0.082, mesh bound). Tape it.
- **Nothing publishes `/bt/allow_reverse`** until Part D. Until then no automatic reverse
  happens. The node logs and reports `bt_allow_stale` on `/mpc/reverse_status`.
- **`max_reverse_speed_mps` must be ≥ 0.4** for any reverse. That is a deliberate, separate
  operator step. The tests that pin 0.0 change with it (`output/reverse/recon.md` §4).
- **The rear is blind.** The pre-check reads the SLAM map behind the car, which only holds
  what was scanned on the way in; the LiDAR's rear 90° is never scanned live. Objects
  behind come only from what perception reported (camera, plus semantic memory with Part
  B). A person who walked in behind the car after it passed is not in either.
- **`mpc_rear.*` weights are the forward copies.** They hold the corridor heading hard: a
  10 cm lateral offset is corrected by about 5 mm over 0.5 m. The sign is right; the amount
  is tuning.
- **Reverse braking is unmeasured.** The stop margin uses the forward fit.
- **Nominal model, first-order speed plant.** The VESC's behaviour at a direction change
  is not modelled.

---

## Part B — Semantic layer: object memory with visibility-aware decay

### B0 — Audit (line numbers at `868ec8a`)

- **Detection source.** `/camera/detections_3d` (vision_msgs/Detection3DArray, frame
  `zed2_left_camera_frame`) at `semantic_layer_node.py:163`. These are YOLO classes only:
  no LiDAR object source feeds the layer. `/perception/obstacles_2d` is not used because it
  carries no class label (node docstring :12-32).
- **Pose.** `/ekf_global/odometry/filtered` (map frame), `semantic_layer_node.py:184`.
- **Frame of stored objects.** MAP (`SemanticObject.x_map/y_map`, `semantic_layer.py:145-146`).
  - camera → base_link is a TF lookup (node :465-467);
  - base_link → map uses the pose nearest the batch stamp (node :474,
    `compose_base_link_to_map` `semantic_layer.py:103-118`);
  - markers and tracks are published in `map` (node :623, :703).
- **Removal and timeouts.** There was **no wall-clock expiry** (`semantic_layer.py:126-128`).
  - A track confirms after `confirm_hit_count = 3` consecutive hits, with
    `confirm_grace_misses = 2` (node :217, :233).
  - Any track, confirmed or not, was dropped at `lost_miss_count = 5` consecutive missed
    detection batches (node :218; pruning `semantic_layer.py:378, 441`). That is about 0.4 s
    at the ~12.5 Hz detection rate.
  - Empty batches count as misses (node :448-463). With the camera silent, nothing counted
    misses and tracks never expired.
- **Association.** Gate `merge_distance_m 0.5` + `gate_sigma_scale 2.0` × sigma (node :203,
  :244), against the predicted position (prediction capped at 2 s, `semantic_layer.py:86`).
- **Other trackers.**
  - The wall layer rebuilds from confirmed walls on every message (node :372-397). The wall
    tracker deletes a wall after `lidar_front_wall_wall_tracker_delete_after_sec` 2.0 s.
  - The MPC's obstacles were `/perception/obstacles_2d` only: live, no persistence
    (`MPC_corr.py:674-677, 3171-3185`).
  - The BT's go_to_object reads `/costmap/semantic_tracks`, which is stale after
    `object_tracks_max_gap_sec`.
- **Pre-existing, not fixed.** The node docstring :37-39 says "no map->odom edge exists".
  `ekf_global.yaml:102` publishes it, and MPC_corr already looks it up.

### What was built

- **Visibility.** Ten times a second, `f1tenth_costmap/object_memory.py` (pure Python) works
  out from the car's current map pose whether each sensor could see each confirmed object.
  - **LiDAR** (`lidar_visibility`): the object must be inside the scan's **own**
    `angle_min..angle_max`, trimmed to the beams actually present (±135° on the UST-10LX), so
    the 90° rear blind cone is out of view by construction. Its near surface must be within
    `range_min..range_max`, and it must not be occluded: ≥ 50 % of the beams spanning it
    return closer than its near surface − `occlusion_margin_m`.
  - **Stereo** (`stereo_frustum`, `columns_occluded`, `object_visibility`): the camera is live
    (a batch processed within `sensor_stale_s`); the whole footprint is inside the 101°
    frustum; the depth is within [0.8, 5.0] m; and it is not occluded, either by StereoColumns
    wall depth or by the LiDAR verdict on the same object.
  - **in_fov** uses the sensor able to detect the class: stereo for every YOLO class.
    `LIDAR_ONLY_CLASSES` exists and is empty.
- **Lifecycle** (`step_memory`, `memory_cycle`):

  | Situation | Result |
  |---|---|
  | detected | TRACKED, confidence 1 |
  | in view, undetected for > `fov_miss_remove_s` (1.5 s, cumulative since last seen) | **removed** |
  | out of view or occluded | **MEMORY**: confidence 1 → 0 over `memory_decay_s` (15 s, the node enforces 10-20), then removed |
  | MEMORY, back in view, undetected | decay continues and the 1.5 s rule applies |
  | MEMORY, detected | TRACKED with the **same id**: the tracker keeps confirmed tracks (`prune_confirmed=False`), memory zeroes the stale velocity so the detection is matched at the stored position, and the gate is widened by the person growth |

  A **person in MEMORY** grows by `person_memory_growth_mps` 0.2 m/s, with the growth capped
  at `person_memory_growth_max_m` 0.6 m. Static classes don't grow. Everything stays in the
  map frame: a map→odom jump changes what the car can see, never where a remembered object
  is.
- **Exposed** on `/costmap/semantic_objects` (new `SemanticObjectArray` / `SemanticObject`,
  map frame, 10 Hz). Per object: id, class, x/y, base and effective radius, state
  (TRACKED / MEMORY), confidence, time in memory, time left, time since seen, in-view time
  unseen, and `in_fov` (overall, LiDAR, stereo).
- **Foxglove:** `/viz/semantic_objects`. Remembered objects are drawn semi-transparent,
  fading with confidence and labelled with the time left (e.g.
  `person #4 MEM 7.3s r+0.40 [S-]`). It is built only while something subscribes, at 5 Hz
  at most.
- `/costmap/semantic_markers` and `/costmap/semantic_tracks` are unchanged, with one
  exception: a remembered object that is detected again reappears at once under its old id.
  `semantic_layer_memory_enabled: false` restores the old pruning exactly.
- **Helper for odom consumers:**
  `f1tenth_params.semantic_objects.objects_to_odom_circles(objects, map_to_odom,
  include_memory, min_confidence, class_margins)`. It re-expresses through the current
  map→odom every call, using the effective (grown) radius.

### B4 — Consumers (mpc_corr)

- **The obstacle list.** MPC_corr subscribes to `/costmap/semantic_objects`. Every tick it
  adds the **MEMORY** objects to the obstacle list the solver (`w_obs`) and
  `compute_local_target` use, as odom circles through the current map→odom with the
  inflated radius and `obstacle_class_margin_m`. The TRACKED ones are already on
  `/perception/obstacles_2d`; adding them twice would double the target deflection.
- **Part A's rear pre-check and rear clearance** get **every** semantic object, TRACKED and
  MEMORY.
- **When nothing is added:** the message is older than `mpc_semantic_memory_max_age_s`
  (0.5 s), there is no map→odom edge, or `mpc_semantic_memory_enabled: false`.
- front_clearance, the e-stop and the BT are **not** touched.

### Parameters

| Key | Default | |
|---|---|---|
| `semantic_layer_memory_enabled` | true | false = the old lifecycle |
| `semantic_layer_memory_rate_hz` | 10 | memory cycle and publish |
| `semantic_layer_fov_miss_remove_s` | 1.5 | in view and undetected → removed |
| `semantic_layer_memory_decay_s` | 15 (10–20 enforced) | MEMORY lifetime |
| `semantic_layer_person_memory_growth_mps` / `_max_m` | 0.2 / 0.6 | person growth and its cap |
| `semantic_layer_stereo_hfov_deg` | 101 | |
| `semantic_layer_stereo_depth_min_m` / `_max_m` | 0.8 / 5.0 | **untuned**, class-blind |
| `semantic_layer_occlusion_margin_m` | 0.15 | |
| `semantic_layer_stereo_occlusion_use_lidar` | true | |
| `semantic_layer_sensor_stale_s` | 0.5 | |
| `semantic_layer_stereo_columns_max_age_s` | 1.0 | |
| `mpc_semantic_memory_enabled` | true | B4 consumer on |
| `mpc_semantic_memory_max_age_s` | 0.5 | |
| `mpc_semantic_memory_min_confidence` | 0.0 | |

### Messages

`msg/SemanticObject.msg` and `msg/SemanticObjectArray.msg` (new). Build `f1tenth_messages`
before `f1tenth_costmap` and `mpc_controller`.

### Tests

- **f1tenth_costmap:** 232 → **305 passed**.
  - `test_object_memory.py` (56): every B5 case.
    - leaves the view → about 15 s, then gone;
    - in view and undetected → present at 1.4 s, removed by 1.6 s;
    - occluded → MEMORY;
    - person growth 0.2 m/s, capped at +0.6;
    - reappears → same id;
    - map→odom jump → unmoved in map;
    - plus the visibility geometry.
  - `test_object_memory_node.py` (17): node wiring, parameter range and the state-constant
    match.
- **f1tenth_params:** 54 → **69 passed** (`test_semantic_objects.py`, 15: the helper and the
  frames).
- **mpc_controller:** `test_semantic_memory_consumer.py` (7).
  - MEMORY objects join the obstacle list with the grown radius, TRACKED ones do not;
  - the rear check gets every object;
  - stale, unplaceable or disabled gives nothing;
  - the confidence filter;
  - **a remembered person behind the car (out of view) refuses the reverse** (closed loop,
    `rear_object`).
- **Lint, flake8 / pep257, unchanged:** costmap 357 / 775, params 100 / 129, messages 0 / 0,
  mpc_controller 1156 / 1197.

### Open TODOs and risks (Part B)

- **Not run live; CPU unmeasured on the Orin.** The semantic layer now also subscribes to
  `/scan` (40 Hz, best-effort, depth 1) and `/perception/stereo_columns` (5 Hz), and runs a
  10 Hz timer.
- **Not implemented:**
  - stereo occlusion by **another detected object** (StereoColumns masks object pixels, so
    only the LiDAR catches it);
  - a vertical field-of-view check;
  - per-class depth windows;
  - a LiDAR-only object class.
- **The detector is about 0.4 s behind the pose**, so a fast turn can spend part of the
  1.5 s "looked and it isn't there" budget before the detector has seen the frame.
- **Re-association in a crowd:** a different person inside the gate (0.5 m + growth) can
  inherit a remembered person's id.
- `/costmap/semantic_tracks` consumers (FOLLOW, go_to_object) now see a remembered object
  reappear immediately under its old id. The BT code is untouched; this is probably better.
- **In sim**, scan and camera frame names may differ. A failed TF lookup counts as "not
  looking", so objects go to MEMORY rather than being removed.
