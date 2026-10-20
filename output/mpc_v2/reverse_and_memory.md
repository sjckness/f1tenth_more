# Reverse manoeuvre, object memory, BT integration, LLM start — report

Branch `mpc_corr_v2_reverse`. The base is `868ec8a` (origin `campaign-final` on
2026-10-19). The Orin's newer `campaign-final` / `campaign-final-doors` are not in this
base, and this branch is to be rebased onto `campaign-final-doors`. Each part is one
self-contained commit, in order: MPC v2 (4 commits, `output/reverse/report.md`), **A**,
**B**, **C**, the review and the Part B fixes, then **D**. The branch was already pushed,
so D came after C rather than before it.

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
  - "No edge" includes `use_map_frame_goal_anchor: false`. `_lookup_map_odom()` returns
    None then, and the flag defaults to true only for `localization_source: ekf`.
  - With `raw_odom`, B4 is therefore off. Part A's rear grid is not placed either, so the
    rear side stays `rear_unknown` and no reverse ever starts. Since the review fixes this
    is one WARN at startup (`SEMANTIC/memory | inactive: use_map_frame_goal_anchor is
    false ...`), not silent.
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
- **mpc_controller:** `test_semantic_memory_consumer.py` (9).
  - MEMORY objects join the obstacle list with the grown radius, TRACKED ones do not;
  - the rear check gets every object;
  - stale, unplaceable or disabled gives nothing;
  - the confidence filter;
  - MPC_corr's own `_reverse_env` appends every semantic object to the live ones, and
    nothing from a stale message (added in review);
  - **a remembered person behind the car (out of view) refuses the reverse** (closed loop,
    `rear_object`). This one runs the supervisor on the harness, not MPC_corr's wiring:
    the two `_reverse_env` tests above cover that.
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
- **B4 changes the forward MPC by default** (`mpc_semantic_memory_enabled: true`). Part A's
  "the forward MPC is unchanged" stops holding whenever a MEMORY object exists.
  `test_reverse_flag_off_identity` runs without semantic messages, so it does not cover
  this.
- ~~B4 bypasses `obstacle_projector_node`'s plausibility filters~~ — **fixed**, see "Part B
  fixes" below. Still open: `obstacle_class_margin_m` (and now the gate's four keys) are
  read from `stack_params.yaml`. A launch override reaches the projector's ROS parameter,
  but not mpc_corr.
- ~~The go_to target can become an obstacle at the end of its own approach~~ — **fixed**,
  see below. The original wording overstated the remembered half: with the shipped gap
  the camera never gets inside the 0.8 m stereo depth floor of the target (camera to a
  person's centre ≥ 1.03 m at a 0.5 m gap).
- ~~MEMORY objects reach `RearEnv.objects` twice~~ — **fixed**: `_reverse_env` now
  builds its own list.

### Part B fixes after review

Three fixes, in their own commit on top of the review commit. B4 stays on by default.

**a) The current move's target is never an obstacle** (`mpc_controller/move_target.py`).
- **Which object is the target.** During an object move (`object_psi_c` set by
  `goal_object_callback`):
  - a semantic object is the target when its id equals `ObjectGoal.track_id`;
  - with no id, it is the target when its class equals `target_class` and its centre lies
    within `mpc_target_exclusion_gate_m` (0.5 m) of the goal point;
  - a live `/perception/obstacles_2d` circle has no id or class, so it is the target when
    its centre lies within that gate of the goal point.
- **Left out of:**
  - the obstacle list (`_planning_obstacles`: the solver's `w_obs`, `compute_local_target`,
    the predicted clearance);
  - the rear check (`_reverse_env`, TRACKED and MEMORY);
  - the stuck detector's front distance (`_reverse_front_distance`), so a car stopped at
    its standoff is never "stuck" on its target.
- **Growth.** The person growth has no path to the target: it is in none of those lists.
- **Not changed:** `/mpc/min_obstacle_distance` and `_forward` still use the raw live list.
  They are reported distances that mission stop conditions read, not avoidance.
- **This changes the live approach too.** The target was a live obstacle by design: the
  standoff is sized so the goal lies outside its R_safe (`min_standoff_clear_of`). Now
  nothing deflects the car around its own target. What stops it there is the stop latch,
  plus `front_clearance` and the proximity e-stop, which read the sensors directly.
  `gap_min` stays the safety minimum where the gap is set.
- **Risk:** a different object within 0.5 m of the target's centre is left out of the
  live list with it (live circles carry no id). By id, an impostor on the goal point
  stays an obstacle.
- **Tests:**
  - `test_go_to_target_not_an_obstacle.py` (10), on the go_to end-to-end rig (real
    GoToObject, CheckStopCondition, goal callback, corridor and solver), with MPC_corr's
    own `_planning_obstacles` and `_reverse_env` every DRIVE tick.
  - **Lost at 0.8 m** (front bumper to near edge, the go_to gap measure): the move ends
    `reached` in the stop window. The target is in no planning or rear list on any tick,
    before the loss (live, TRACKED) or after (MEMORY, grown).
  - **A different person in MEMORY** 0.7 m beside the path: an obstacle every tick with
    its grown radius 0.55. The car passes it ≥ 0.02 m farther than without it, and ≥ 0.75
    m (grown radius + car radius) from its centre. The approach completes.
  - **Lost earlier, at 1.2 m:** DRIVE ticks run on the remembered, grown target (the BT's
    tracks hold the person 0.4 s, as semantic_layer_node does). None carries it.
  - **Mutation check:** with `_move_target` returning None, 9 tests fail.
  - `test_semantic_memory_consumer.py`: by id, by class + position, the impostor, the live
    gate, the stuck distance.
- **The geometry, and a limit this does not fix.**
  - With the shipped 0.6 m gap the stop latch trips at about 0.84 m
    (`object_reach_tol_m` 0.10 + `object_stop_distance_m` 0.14 past the gap). A loss at
    0.8 m therefore falls in the stop-and-wait, where no solve runs.
  - A loss **before** the latch ends `target_lost`. The go_to handler waits in GRACE at
    speed 0 once `/costmap/semantic_tracks` stops carrying the track, and semantic_tracks
    never carries MEMORY objects.
  - Finishing such an approach on the remembered position would need the handler to read
    `/costmap/semantic_objects`. **Not done; your call.** The 1.2 m test pins today's
    `target_lost`.

**b) One gate for live and remembered objects** (`f1tenth_params.object_geometry`).
- **The function.** `passes_obstacle_filter(z_m, radius_m, ObstacleFilter)`:
  - `obstacle_z_min` ≤ z ≤ `obstacle_z_max`;
  - `min_obstacle_radius` ≤ r ≤ `max_obstacle_radius`;
  - NaN fails.
- **The projector** calls it in place of its two inline checks.
- **mpc_corr** calls it on every semantic object, on `z_m` and the **base** radius, before
  the class margin.
- **The radius bounds** moved from the projector's literals into `stack_params.yaml`, with
  unchanged values (0.03 / 1.5). The projector's defaults now come from there.
- **The height.** The semantic layer now keeps the rotated z it used to drop:
  - per detection, `z_base` = the centre's base_link height;
  - fused per track like the width;
  - published as the new `SemanticObject.z_m` (NaN when no detection carried one).
- **Tests:**
  - params: the gate's bounds and NaN, the keys it reads, the old literals;
  - perception: the projector's gate *is* `obstacle_filter()`, and its height band drops
    what the function drops;
  - costmap: height fusion (7), the published `z_m` with a pitched camera;
  - mpc: overhead, ground, NaN, too wide and noise-sized remembered objects are dropped
    from both lists. The growth does not rescue a noise-sized one, and the bounds
    themselves pass.

**c) The startup WARN.** With `mpc_semantic_memory_enabled` and `use_map_frame_goal_anchor:
false`, `_init_semantic_memory` logs once:

> `SEMANTIC/memory | inactive: use_map_frame_goal_anchor is false
> (localization_source='raw_odom') ...`

The message also says the reverse manoeuvre cannot place its rear costmap. Tested: one
WARN in that case, none when B4 runs or is switched off.

---

## Part D — BT integration

### In one paragraph

`behavior_executor_node` now gives mpc_corr the permission Part A waits for. A post-tick
handler publishes `/bt/allow_reverse` every tick: true only while the mission is RUNNING,
no battery / operator e-stop / overheat condition is tripped and no proximity reading
blocks the rear. The front stops are direction-aware. While `/mpc/reverse_status` says the
MPC is stopping for a reverse or reversing, a front trip at or beyond `front_hard_floor_m`
(0.20 m) no longer puts zero on the safety lane. Side and rear trips, and anything under the
floor, still stop the car in every direction. When the MPC gives up (`stuck`, three
reverses without progress), the move FAILS with `stuck_after_reverse` and the mission is
aborted, the same path as go_to_object's failures. The MPC's mode is mirrored onto the
blackboard and into the tree view, and each change is logged once. Everything sits behind
`bt_reverse_integration` (default true). **Nothing reverses on the car yet:
`max_reverse_speed_mps` is still 0.0**, so mpc_corr reports `reverse_speed_limit` (Part A).

### D0 — Audit

- **Tree** (`behavior_executor_node.create_root`): root `Selector(memory=False)` over
  `emergency`, `handle_obstacle` (only with `enable_camera_obstacle_stop`, default
  **false**), `mission`, `navigation`. The task's list leaves out `mission`.
- **handle_obstacle**: `IsObstacleDetected` on `/camera/detections_3d`. It trips on a
  detection centre within 1.0 m of the front bumper, inside a ±0.32 m corridor. Then
  `Stop(safety_stop, base_link/obstacle)`.
- **is_proximity_too_close**: in the emergency lane. Front cone ±45° measured from the
  bumper, 0.15 m. Side/rear window on raw ranges, 0.15 m. Its `/scan` callback also
  publishes the emergency stop directly (`enable_proximity_event_stop`, stack default
  true). Both paths decide through one `should_stop()`.
- **"front_clearance stop"**: there is no front_clearance stop on the safety lane.
  - The mission's `front_clearance` stop condition (`CheckStopCondition`) ends a move
    through `/mpc/hold`, never through the safety lane.
  - `/perception/front_clearance` and `/front_blocked` have no subscribers, by design.
  - The hold condition is left alone: while held, mpc_corr is never stuck, and a hold
    during a reverse ends it (`abort`, Part A).
  - So the two front stops gated here are IsProximityTooClose's front cone and
    handle_obstacle's camera corridor.
- **Mux lanes** (`f1tenth_bringup/config/mux.yaml`): safety_stop 200 > joystick 100 >
  calibration 50 > navigation `/drive` 10, each with a 0.2 s timeout.
- **Mission state** reaches the tree as `MissionRuntimeState` on the blackboard
  (`MISSION_KEY`), behind `MissionActive` (RUNNING or HOLDING).

### What was built

| Piece | Where | What |
|---|---|---|
| D1 permission | `reverse_integration.py` `ReverseIntegration.decide` (`:158`), `post_tick` (`:170`) | `Bool` on `/bt/allow_reverse` once per tick, after the tree. False unless the mission is **RUNNING**; false if IsBatteryLow, IsEmergencyStopTriggered or IsSystemOverheated is SUCCESS this tick; false if a front stop's `blocks_reverse()`. Each change of the value is logged once, with its reason. |
| Status order | `create_root` (`behavior_executor_node.py:532`) | IsProximityTooClose is now the **last** emergency condition. A memory=False Selector leaves the children after its first SUCCESS unticked, and the front trip that got the car stuck must not hide the battery/e-stop/overheat statuses D1 reads. |
| D2 mode | `ReverseModeMonitor` (`:63`) | Holds `/mpc/reverse_status`. `reverse_active()` is true only on a fresh status (younger than `bt_allow_reverse_max_age_s`, 0.5 s, reused) that says stopping-toward-reverse or reverse. Stale or absent reads as forward. |
| D2 front cone | `is_proximity_too_close.py` `_front_full_stop` (`:428`), `_tripped` (`:442`), `blocks_reverse` (`:447`), `_event_stop_held` (`:464`) | A front trip counts unless the monitor says reverse_active and the front return is ≥ `front_hard_floor_m`. Side/rear trips always count. The gate is inside `should_stop()`, so the scan callback and the tick still decide the same way. A front-only callback stop is not re-held for its 0.2 s once a reverse releases it. |
| D2 camera corridor | `is_obstacle_detected.py` `blocks_reverse` (`:99`), `should_stop` (`:108`) | Same rule on the nearest blocker's distance ahead of the bumper (the callback now keeps the nearest one, not the first one). |
| D3 give-up | `_stuck_this_move` (`:194`), `_fail_move_stuck` (`:205`) | Fires when mpc_corr's mode is `stuck` and that `stuck` arrived after the current move started. It does the same steps as GoToObject's failure outcomes, then the mission is ABORTED (details below). |
| D4 visibility | `MpcReverseMode` (`:103`), blackboard `mpc_reverse_mode` | A pass-through node between MissionActive and mission_selector, renamed `mpc_mode: <mode>` every tick it runs, so the tree image shows `mpc_mode: reverse`. The blackboard key is written every tick by the post-tick handler. One INFO line per mode change (`[reverse] MPC mode forward -> stopping (attempts 0, ...)`), including the change to `unknown` when the status goes stale. |

**Why D3 is not a node in the mission subtree.** While the front stop trips, the emergency
lane wins every tick and the mission subtree is never ticked. A stuck-check node there
would never see the give-up. This is the starvation shape `create_root`'s docstring already
describes. The post-tick handler runs on every tick.

**What follows a FAILED move** (existing logic, unchanged). The failure takes the path
every failed move takes today, GoToObject's `target_lost`/`target_unreachable`:

- `/mpc/hold` true;
- `last_stop_reason = failed:stuck_after_reverse`, recorded in the move outcome and so on
  `/mission/move_outcome`;
- for a go_to_object move, the record's outcome `stuck_after_reverse` and the end on
  `/mpc/goal_object_end`;
- the summary written as ABORTED;
- `state.abort()`, so `/mission/status` reads ABORTED and the campaign logger gets
  `mission_finished`.

There is no per-move failure policy to consult: `on_timeout` (abort / skip / stop) governs
timeouts only. mpc_corr itself stays in `stuck` (zero speed) until it gets the next move's
goal. A `stuck` left over from an earlier move started before the new move did and does not
fail it.

### Tree before / after

Before (`6f48d19`):

```
root  Selector
├── emergency  Sequence
│   ├── emergency_condition  Selector
│   │   ├── IsBatteryLow
│   │   ├── IsEmergencyStopTriggered
│   │   ├── IsProximityTooClose          (enable_lidar_safety_stop)
│   │   └── IsSystemOverheated           (enable_sys_obs)
│   └── Stop  safety_stop, base_link/emergency
├── handle_obstacle  Sequence            (enable_camera_obstacle_stop, default off)
│   ├── IsObstacleDetected
│   └── Stop  safety_stop, base_link/obstacle
├── mission  Sequence
│   ├── MissionActive
│   └── mission_selector  Selector
│       ├── mission_object_response  [ObjectSeen, HandleObjectAction]
│       └── mission_progress  [PublishMoveGoal, GoToObject, CheckStopCondition, AdvanceMove]
└── navigation  [HasMpcGoal]  or  [HasGoalPose, NavigateThroughPosesClient]
```

After, with `bt_reverse_integration` true (`*` = direction-aware with
`direction_aware_front_stop`):

```
root  Selector
├── emergency  Sequence
│   ├── emergency_condition  Selector
│   │   ├── IsBatteryLow
│   │   ├── IsEmergencyStopTriggered
│   │   ├── IsSystemOverheated           (enable_sys_obs)
│   │   └── IsProximityTooClose *        (moved last)
│   └── Stop  safety_stop, base_link/emergency
├── handle_obstacle  Sequence            (default off)
│   ├── IsObstacleDetected *
│   └── Stop  safety_stop, base_link/obstacle
├── mission  Sequence
│   ├── MissionActive
│   ├── mpc_mode: <forward|stopping|reverse|stuck|unknown>     (new, always SUCCESS)
│   └── mission_selector  (unchanged)
└── navigation  (unchanged)
post-tick: ReverseIntegration -> /bt/allow_reverse, stuck -> FAILED, blackboard mpc_reverse_mode
```

With `bt_reverse_integration` false, the tree is the old one, apart from the emergency
order. Nothing publishes `/bt/allow_reverse`, and mpc_corr never starts a reverse.

### Safety-lane (`/safety_stop`) publishers, and when they are suppressed now

| Publisher | Frame | Stops when | Suppressed now |
|---|---|---|---|
| Emergency lane `Stop` (tree) | `base_link/emergency` | any emergency condition SUCCESS | **Only for a front-cone trip at or above the floor** while the MPC is stopping-for-reverse or reversing. Battery, operator e-stop, overheat, side/rear and under-floor front: never. |
| IsProximityTooClose scan-callback stop | `base_link/emergency` | a scan trips `should_stop()` | The same rule (one decision). Its 0.2 s re-hold of a front-only stop also ends once a reverse releases it. |
| handle_obstacle `Stop` (off by default) | `base_link/obstacle` | camera blocker ≤ 1.0 m ahead of the bumper | While the MPC is stopping-for-reverse or reversing, unless the nearest blocker is under the floor. |
| `reset_manager_node` (f1tenth_bringup) | — | during `/reset_all` | Never; not touched. |

**With the shipped thresholds the front-cone gate changes nothing.**
`proximity_front_threshold_m` is 0.15 m, under the 0.20 m floor, so every front trip is
already a full stop. The floor applies to front trips only: a reading between the
threshold and the floor is not a trip. So the floor does not raise the forward threshold,
and no threshold was changed. The gate starts to act on IsProximityTooClose only if that
threshold is set above 0.20 m (D5 tests it at 0.40 m). It does act on handle_obstacle
(1.0 m) whenever that lane is on.

### Parameters (`stack_params.yaml`, forwarded by `behavior_bringup.launch.py`)

| Name | Default | Meaning |
|---|---|---|
| `bt_reverse_integration` | true | D1, D3, D4, and D2's mode monitor. |
| `direction_aware_front_stop` | true | D2. False: the front stops stop in every direction, as before. |
| `front_hard_floor_m` | 0.20 | Absolute front floor for D2, measured from the front bumper. |

`/mpc/reverse_status`'s staleness limit reuses mpc_corr's `bt_allow_reverse_max_age_s`
(0.5 s) instead of a fourth parameter.

### Tests (D5), `test/test_bt_reverse_integration.py`, 8 tests, mocked

| Work-order case | Test |
|---|---|
| Stuck at 0.35 m from a wall with the front stop active: reverse allowed, no zero on the safety lane, the front stop resumes for forward | `test_stuck_at_the_front_stop_reverses_without_a_safety_stop_then_it_resumes` (front threshold test-configured at 0.40 m; callback and tick both checked) |
| Front clearance 0.15 m: full stop, no reverse | `test_front_under_the_floor_stops_in_every_direction_and_forbids_reversing` |
| Emergency active: allow false | `test_an_emergency_condition_withdraws_the_permission` |
| Side obstacle during reverse: stop | `test_a_side_obstacle_during_the_reverse_stops_and_withdraws` |
| 3 failed attempts: move FAILED `stuck_after_reverse` | `test_three_failed_attempts_fail_the_move_stuck_after_reverse`, plus `test_a_stuck_left_over_from_before_the_move_does_not_fail_it` |
| `/bt/allow_reverse` stale: no reverse | mpc_corr's side, already covered by `test_reverse_manoeuvre.py` (`bt_allow_stale` in the permission table, `bt_withdrawn: bt_allow_stale` mid-reverse). The BT's own staleness rule is `test_a_stale_status_reads_as_forward`. |

Also `test_no_permission_without_a_running_mission`. The tree wiring (`create_root`) was
checked with a throw-away script, not a committed test. It covers the emergency order, the
`mpc_mode` node, the monitor handed to both front stops, `direction_aware_front_stop:=false`
leaving them ungated, and `bt_reverse_integration` off leaving the mission subtree as before.

**`py_trees_ros` is still not installed on linus**, so `colcon test` still cannot collect
f1tenth_behavior. The new test file does not import it and passes on its own. With a
scratch import stub for `py_trees_ros` (`trees.BehaviourTree` only, kept out of the repo),
the whole suite passes: 440 tests, including the files that build the tree. It needs a
real run once `ros-jazzy-py-trees-ros` is in.

### Risks of the direction-aware change

- **The front is released on mpc_corr's word.** A stale or missing status reads as
  forward, so that case fails safe. A fresh but wrong `reverse_active` does not: an MPC bug
  that drove forward while reporting a reverse would lose the front stop down to the floor.
- **During a reverse, only the floor guards the front.** Something that moves into the
  front cone between the floor and the threshold is not a stop while the car backs away.
  That is the intent, but it relies on the car actually moving backwards: a direction
  change the VESC does not honour is not modelled (Part A risk).
- **The rear is still blind.** The side/rear proximity window is what keeps blocking in
  every direction, but the LiDAR never scans the rear 90° live (Part A). During a reverse,
  an obstacle straight behind is caught only by mpc_corr's rear pre-check and
  rear-clearance termination, which read the map and the perception objects.
- **Hand-back latency.** The tree learns the mode from a 10 Hz status and ticks at 10 Hz,
  so the front stop returns up to about 0.2 s after a reverse ends. mpc_corr's
  stopping-toward-forward phase commands zero for at least `gear_change_dwell_s` (0.3 s),
  so no forward command goes out before the front stop is back.
- **Allow stays true while the front stop trips.** This is deliberate: that trip is what
  makes the car stuck. Withdrawing it there would deadlock the case the reverse exists for.
- **D3 aborts while the e-stop holds the car.** A stuck give-up against a persistent front
  trip ends the mission: the operator restarts it, the car does not wait.

---

## Two reverse mechanisms: at most one at startup

**Before:** with both on, mpc_corr only WARNed and silently dropped Part A. **Now** it
refuses to start. The two mechanisms are `allow_reverse` (the multi-candidate MPC v2,
`reverse_mpc.py`, default false) and `reverse_auto_enabled` (Part A's manoeuvre, default
true). `validate_reverse_mechanisms` (`reverse_manoeuvre.py`) runs in
`MPC_corr._init_reverse_manoeuvre`:

- **Both true:** one FATAL line `REVERSE | STARTUP REFUSED: ...` naming both parameters
  and the defaults, then a `ValueError` that ends the node.
- **What it reads:** the parameters as set. `allow_reverse` is read before the speed-limit
  check that can switch it off.
- **Kept as before:** `allow_reverse` on with `reverse_auto_enabled` off still builds no
  Part A supervisor (now an INFO line).
- **Deleted:** neither mechanism.
- **Tests:** 4 cases in `test_reverse_manoeuvre.py`.

**Why the multi-candidate version fails door scenarios b and c**
(`output/reverse/report.md`, "Why the door does not work"):

- **The cause.** The selector scores each direction's QP plan with the lattice cost-to-go,
  and the two disagree at a jamb. The cost-to-go checks the whole footprint. The QP sees
  only its nose-line probes and the rear-axle point, so it cannot see a body corner meeting
  the jamb.
- **Scenario c (door seen early).** The forward plan's end flips between "collision" and
  "free" every few centimetres of creep, its score between about 7 and about 150. The
  selector dithers: 13 direction changes, no entry.
- **Scenario b (door seen late).** The car brakes and reverses 1.1 m correctly. From there,
  no forward plan the QP can produce ends free.
- **The lattice is also optimistic about steering.** It changes steering instantly; the QP
  is held to 0.5 rad/s, which costs about 0.3 m of path per full change, more than a 1.3 m
  hall with a 0.9 m door leaves.

## Pre-existing test failures (not fixed here)

All 19 mpc_controller and 12 llm failures also fail on a `git archive 868ec8a` export
(this branch's base). The causes are read from the failure messages, not root-caused.

| Test | Count | Cause, one line |
|---|---|---|
| mpc_controller `test_corridor_direction_recovery` | 1 | Frozen-heading geometry assertion (`True is not false`). |
| mpc_controller `test_corridor_sizing` | 1 | Passing-person half-width 0.8645 m < the asserted 0.870 m. |
| mpc_controller `test_d_wall_correction` | 1 | `drive_stop_2m_from_wall.json` is a `straight` move; the test expects `wall_turn`. |
| mpc_controller `test_drive_limits` | 1 | go_to approach speed: the stack has 0.5 m/s, the test expects 0.4. |
| mpc_controller `test_model_log` | 1 | The model-log CSV header differs from the analysis script's header. |
| mpc_controller `test_prediction_step` | 2 | dt 0.1 vs the expected 0.2 s; lookahead floor 1.25 vs 2.5 m. |
| mpc_controller `test_steering_limits` | 8 | Package `f1tenth_hardware` is not built/found in this workspace. |
| mpc_controller `test_wall_turn_increment` | 3 | Trigger 2.30 vs 3.71 m (work-order table), an asymmetry assertion, a psi increment of 0. |
| mpc_controller `test_weight_set` | 1 | `w_du_delta` 2.9155 vs the expected 3.0. |
| llm (`f1tenth_intelligence`) | 12 | The test stubs lack `_note_llm_sent` (missing since `dc158d2`). |

---

## Part C — LLM: start as soon as loaded

**Where the ~15 s came from.** Not from the LLM planner.

- `llm_planner_node` already called `/mission/start_mission` right after
  `/mission/load_mission` answered, with no sleep or timer in between
  (`llm_planner_node.py:1410-1421` at `868ec8a`).
- The wait is the behaviour node's start countdown:
  - `f1tenth_behavior/mission/loader.py:239` sets `mission_countdown_sec = 15.0`;
  - `start_mission` does not start the mission: it arms a 15 s timer (`loader.py:432`) and
    answers "starting in 15.0 s" (`loader.py:440`);
  - the mission stays in LOADED with the hold on until the timer fires (`loader.py:444`).
- So "loaded" and "started" are 15 s apart in the status topic and in the test-campaign
  events. The countdown exists so every test begins with a measured standstill and the bag
  recorder has time to start (`f1tenth_logger/TEST_CAMPAIGN.md:172-185`).

**What confirms the load.** The reply of `/mission/load_mission` itself. The loader only
answers after the mission file is parsed and installed (`loader.py:694-702`). Before this
change the planner waited at most 2 s to find the service plus 5 s for the reply, using two
hard-coded constants it also uses for abort and start.

**What changed.**

- `llm_planner_node` calls `start_mission` the instant the load is confirmed (`_deliver_mission`).
  Nothing waits a fixed time.
- New parameter **`llm_load_confirm_timeout_s`** (default **15 s**, `stack_params.yaml`). It
  is only the **longest** the planner waits for the confirmation: one deadline covering
  finding the service and getting the reply (`wait_for_load_confirmation`). A confirmation
  after 0.3 s starts at 0.3 s.
- If the timeout runs out, the loader is unreachable, or the loader refuses the file, the
  planner does **not** call `start_mission`. It logs why (`load_mission NON CONFERMATO …` /
  `load_mission FALLITO …`, ending in `start_mission NON chiamato.`) and reports the
  failure to the test-campaign logger.
- A timeout ≤ 0 is rejected at node start.

**Tests.** `src/f1tenth_intelligence/llm/test/test_load_then_start.py`, 9 tests with mocked
services:

- start follows a successful load within 0.1 s;
- a refused load never starts;
- no confirmation within the timeout never starts;
- a late or unreachable loader never starts;
- the timeout is a single deadline;
- the shipped default is 15 s.

The llm package suite is 14 failed / 242 passed / 25 skipped.

- 12 of the failures were there before this change: stubs missing `_note_llm_sent` since
  `dc158d2`, untouched.
- The other 2 are `test_flake8` (165 findings) and `test_pep257`, both also failing at
  `868ec8a`. flake8 is unchanged.

**⚠ Decision for you — the 15 s countdown is still there.** "No fixed delay anywhere" is
read here as the planner's own waiting. The countdown is a deliberate test-campaign
feature, not a wait for the load, and **the car still waits 15 s after an LLM command**.
To drop it:

1. **Per session:** `ros2 param set /behavior_executor_node mission_countdown_sec 0`.
2. **Or for the LLM path only:** add a `/mission/start_mission_now` (Trigger) to the loader
   and have the planner call it. LLM-driven test runs would then lose the measured
   standstill and the bag lead-in. This is a small change in `f1tenth_behavior`, and
   `test_mission_countdown.py` would need extending.

---

## Branch and verification summary

- **Commits** on `mpc_corr_v2_reverse`, oldest first: recon → v2 candidates → v2 cost-to-go
  → v2 tests and report → **A** → **B** → **C**. The ids are in `git log`.
- **Tests on linus**, re-run after the review commit (`colcon test`, DDS isolated):
  - mpc_controller: 869 tests, 21 failed / 845 passed / 1 skipped / 2 strict xfails.
    - 19 failures are functional tests. All 19 also fail at `868ec8a`, checked by running
      the same files on a `git archive` of it.
    - The other 2 are `test_flake8` and `test_pep257`, also failing at baseline.
  - f1tenth_costmap: 305 passed.
  - f1tenth_params: 69 passed.
  - llm: 14 failed / 242 passed / 25 skipped. The failures are 12 pre-existing
    `_note_llm_sent` stubs plus flake8 and pep257.
  - f1tenth_behavior (not touched on this branch): `colcon test` **cannot collect it on
    linus**, because `py_trees_ros` is not installed.
    - File by file, 365 pass.
    - The 2 files that import `behavior_executor_node` cannot be collected, and the 6
      `test_safety_stop_config` tests that import it fail on the same `ModuleNotFoundError`.
    - Part D's BT tests will hit this.
  - f1tenth_logger: the suite **cannot be collected on linus** (`pyarrow` is not installed;
    pre-existing, unrelated). The topic-list tests that need no `pyarrow`
    (`test_viz_topics_not_recorded`, plus the topic tests of `test_mission_logger_node`)
    pass with the three new topics.
- **Lint (flake8 / pep257):** no package is above its `868ec8a` baseline.
- **Builds:** `colcon build --symlink-install --packages-up-to mpc_controller`, then
  `f1tenth_messages`, `f1tenth_params` and `f1tenth_costmap` again after the message
  changes. All in the worktree.
- **Not done on purpose:** no stack launch, no car or Orin run, `max_reverse_speed_mps`
  unchanged (0.0).
