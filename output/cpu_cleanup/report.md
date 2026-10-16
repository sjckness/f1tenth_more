# CPU cleanup — branch `cpu-cleanup` (from `9c46add` = orin/campaign-final)

Done on linus without building the branch, without the stack and without the car.
Static analysis, then the changed Python nodes run under Jazzy rclpy (source on
`PYTHONPATH`) against synthetic inputs at the Orin's real topic rates, to measure CPU
before and after. Every linus number is **% of one i7-9700K core**. The Orin's A78AE
runs this Python roughly 3–4x slower, so all Orin figures below are **estimates** until
`cpu_test.sh` confirms them. Nothing was pushed. Commits are listed at the end.
Worktree: `~/dev_ws/f1tenth_more_cpu_cleanup` (decisions.md D0 explains why).

## SUMMARY

1. **The BT's 86% was not the tick and was not a busy-wait.** The tick is an rclpy timer at 10 Hz (`behavior_executor_node.py:718-769`); p50 0.38 ms, p95 0.96 ms. The cost was per received message:
   - 76 default QoS event-handler waitables, re-checked on every executor wake-up;
   - a per-beam Python loop over 1081 beams at 40 Hz, plus an rclpy throttled logger call costing 0.24 ms on every scan (`is_proximity_too_close.py:277-319`);
   - a graphviz render that blocked the executor ~80 ms every 0.4 s (`behavior_executor_node.py:263-280`).
2. **New tick:** `bt_tick_hz = 10`. Mission `debounce_ticks` count ticks, so 20 Hz would halve them. The lidar e-stop is now **event-driven**:
   - worst case ~130 ms → ~28 ms (~210 ms before if a render was in the way);
   - 5.1 → 1.1 cm at 0.4 m/s, 12.8 → 2.8 cm at 1.0 m/s.
3. **wall_distance_node: not deleted, off by default** (`enable_wall_distance: false`).
   - MPC_corr's psi_correction depends on it and has no equivalent.
   - But that value falls back to 0.0, and with scene labels on (the default) it is 0.0 anyway.
4. **ekf_cost_observer: off by default.** Diagnostics only, read offline by `ekf_cost_report.py`.
5. **Rates:**
   - costmap_boundary: 20 → **10 Hz**.
   - semantic_layer wall layer: per message (~10 Hz) → **5 Hz** latest-only; detections stay per batch.
   - reset_manager: 20 Hz timer + 4 subscriptions → **event-driven**. Its 36% came from always-on subscriptions (~150 Odometry/s) and a no-op 20 Hz timer, all read only during a reset.
6. **Estimated Orin gain, % of one core:**

   | node | before | after (est.) | basis |
   |---|---|---|---|
   | BT | 86 | ~47 | linus 23 → 12.6 |
   | wall_distance | 42 | 0 | off by default |
   | ekf_cost_observer | 32 | 0 | off by default |
   | costmap_boundary | 48 | ~24 | linus 23.3 → 11.5 |
   | semantic_layer | 41 | ~20 | microbench + listener, not measured whole |
   | reset_manager | 36 | ~0 | linus 10.9 → 0.0 |

   About **1.9 of 12 cores** freed in total.

---

## 1. Behaviour tree

### How it ticks (9c46add)
- `main()` calls `tree.tick_tock(period_ms=bt_loop_duration_ms)` with `bt_loop_duration_ms` defaulting to 100 (`behavior_executor_node.py:718-719, 769`).
  - In py_trees_ros, `tick_tock` is one `node.create_timer(period, ...)` (py_trees_ros `trees.py:532`).
  - Then `rclpy.spin(node)` runs a SingleThreadedExecutor (`:772`).
  - Configured rate 10 Hz; measured 10.00 Hz.
- Every busy-wait source on the brief's list was checked; none is there:
  - no while loop, `spin_once`, `spin_until_future_complete`, sleep or synchronous call in any `update()` (now enforced by an AST test);
  - no MultiThreadedExecutor;
  - no per-tick tree rebuild, deepcopy, or subscription/client creation;
  - no 0-period timer.
  - The only loop with `spin_once` is `wait_for_trigger_service_node.py:98-99` (timeout 0.2 s). It is a separate process that exits once Nav2 is up.

### Where the CPU went
Measured on linus with synthetic inputs at the Orin's rates:
- `/scan`: 1081 beams at 40 Hz
- both EKF odometries: 50 Hz each
- `/perception/virtual_wall`: 40 Hz
- MPC distances: 20 Hz
- `/tf_static`

| scenario | CPU |
|---|---|
| no inputs at all (tick + timers + viz) | 5.7% |
| `/scan` only | +8.7% |
| the two odometries only | +10% |
| everything, 9c46add | 20.1–25.9% (mean ~23%) |
| everything, cpu-cleanup | 11.9–13.9% (mean ~12.6%) |

What each finding was, and what changed:

- **Per-message executor overhead (most of it).** rclpy re-adds and re-checks every Waitable on every wake-up. Each publisher and subscription carries a default incompatible-QoS event handler: 76 of them in this node.
  - Removing only those took the node from 21.7% to 16.3%.
  - Fix: `qos_events.drop_qos_event_handlers`, called after setup.
  - Each entity's own `event_handlers` list is emptied too, because `destroy_publisher` would otherwise raise at shutdown. A real-rclpy test covers this.
  - `bt_keep_qos_event_handlers:=true` keeps them.
  - Mismatches are still reported, because the peer node keeps its own handler.
- **The `/scan` loop.**
  - Was: per-beam Python with cos/sin/atan2/hypot (0.54 ms/scan), plus `get_logger().info(..., throttle_duration_sec=2.0)` on every scan. rclpy builds the f-string and walks the stack before it applies the throttle: 0.24 ms per call, measured.
  - Now: numpy with cos/sin tables cached per (layout, laser yaw), 0.13 ms/scan, and the DIAG line throttled before the call.
  - Matches the old loop to 1e-12 on random scans: nan/inf/below-range values, 4 laser poses, unequal windows.
- **The PNG visualizer** (`enable_bt_visualization: true`). It ran `pydot.create_png()` in a post-tick handler, i.e. a `dot` subprocess the tick waited for: ~80 ms per render on linus, every 0.4 s.
  - That blocked the executor, so the e-stop's `/scan` callback waited too.
  - Now the tick freezes the statuses and a worker thread renders and publishes. The image is identical (a test compares the dot source).

### What the BT must react to, and latency

| input | path | rate needed |
|---|---|---|
| proximity e-stop (IsProximityTooClose) | `/scan`, 40 Hz | as fast as possible: the trip distance is 0.15 m |
| battery / overheat | 1 Hz `/diagnostics/*` | tick is plenty |
| mission start / abort / e-stop service | service → latched blackboard | tick is plenty |
| obstacle lane | off by default (`enable_camera_obstacle_stop`) | — |
| navigation / mission progress | counted in **ticks** (`ConditionDebouncer`, `debounce_ticks: 3` in `wall_turn_then_straight.json` and `drive_straight_to_wall_3m.json`) | changing the rate changes these semantics |

Worst-case e-stop latency (scan period + tick period + processing). The scan period is 25 ms. Processing on the Orin is ~3 ms before and ~0.5 ms after (linus 0.8 → 0.13 ms, ×3–4).

| | latency | 0.4 m/s | 1.0 m/s |
|---|---|---|---|
| 9c46add, tick-driven | 25 + 100 + ~3 ≈ **128 ms** | **5.1 cm** | **12.8 cm** |
| 9c46add, a render in the way | +~80 ms ≈ 210 ms | 8.4 cm | 21 cm |
| tick-driven at 20 Hz (the brief's default) | ≈ 78 ms | 3.1 cm | 7.8 cm |
| **cpu-cleanup, event-driven** | 25 + ~0.5 + ≤1 queued tick (~1–2 ms) ≈ **28 ms** | **1.1 cm** | **2.8 cm** |

Even at 20 Hz, tick-driven, the car covers half of the 15 cm trip distance at 1 m/s before anything is sent. So the e-stop was made event-driven:
- `enable_proximity_event_stop: true`.
- The `/scan` callback publishes the emergency Stop's exact message (`safety_stop`, frame_id `base_link/emergency`, zeros) when a scan trips.
- The tree is untouched: the emergency lane still trips and publishes on its tick, and `/behavior/tree_status` and `/safety/event` are unchanged.
- The tick stays at **10 Hz**, the old rate, because the tick is 0.4% of a core and the `debounce_ticks` semantics stay intact.
- Mux, VESC and braking distance come on top of every row and are not changed.

### Instrumentation
- `tick_timing.BtTicker` drives the tree from one timer at `bt_tick_hz` and times every tick end to end, handlers included.
- Once a second it publishes `DiagnosticStatus 'behavior_executor_node: BT tick timing'` on `/diagnostics` with: `tick_ms_p50/p95/max`, `ticks`, `rate_hz_measured`, `period_ms`, `budget_ms`, `overruns`. Level WARN if any tick exceeded 50% of the period.
- Each overrun also logs a throttled WARN.
- Live check on linus: p50 0.382 ms, p95 0.955 ms, 10.00 Hz, 0 overruns.

### Tests
`test_bt_tick_and_event_estop.py`, 50 tests. They cover:
- timer period = 1/`bt_tick_hz`, and a measured rate under a mock clock;
- the over-budget WARN and its report;
- percentiles;
- an AST check that no behaviour `update`/`initialise`/`terminate` and no post-tick handler makes a blocking or entity-creating call;
- that the visualizer returns while a render is still blocked;
- the stop is published inside the scan callback;
- callback < 5 ms on 1081 beams, so latency < 30 ms (< 1.2 cm and < 3 cm);
- `create_root` wiring;
- vectorised = loop on random scans;
- the QoS handler drop under real rclpy.

## 2. wall_distance_node — kept, off by default

Consumers of `/perception/d_wall`, `/segment`, `/psi_correction`, `/gate_margin`, `/swept_arc`, the WallDistance and WallSegment msgs, and the node name:

| consumer | what it uses | control path? | equivalent from the multi-wall tracker? |
|---|---|---|---|
| `MPC_corr.py:1750-1755, 3301-3333, 4538-4544` | `/perception/d_wall/psi_correction` added to psiEnd on a straight move after a wall_turn | **yes** | **no**: MultiWallTrack has no signed left-positive base_link offset, no phase, no correction |
| `viz_relays.yaml:72-75` | relays `/perception/d_wall` | viz | — |
| `reset_manager.yaml:78`, `reset_manager_node.py` | `hooks.restart_components: [wall_distance]` | reset | — |
| `component_supervisor_node.py:325` | `_ALWAYS_AUTO_START` | launch | — |
| `tools/cpu_test_probe.py:41`, `tools/wall_distance_replay.py` | monitoring / offline | no | — |
| BT (wall stop, wall_turn, preflight, `check_stop_condition`) | **none**: they read `/perception/virtual_wall` (lidar_front_wall_node) | — | — |
| mission logger | records no `d_wall*` | — | — |
| `/segment`, `/gate_margin`, `/swept_arc` | no subscriber anywhere | — | `/segment` ≈ `MultiWallTrack.p0_bl/p1_bl` |

Reading the brief's rule:
- There is a consumer that depends on the node's semantics, so it cannot be deleted.
- That consumer has a fallback: `_fresh_d_wall_correction` returns 0.0, silently, when nothing has ever arrived.
- With `enable_scene_labels: true` (the default) the value is 0.0 anyway. The phase machine only leaves UNKNOWN on `/mpc/wall_track`, and only MPC_corr's private tracker publishes that. Scene labels switch that tracker off (`MPC_corr.py:1085-1089, 1909-1911`).

So the node is gated, and no command changes:
- stack_params `enable_wall_distance: false`, with the full reason in a comment;
- the supervisor auto-starts the component only when that key is true (`_CONDITIONAL_AUTO_START`);
- `wall_distance.launch.py` has an `enable_wall_distance` arg and gates the node on `use_lidar AND enable_wall_distance`;
- `reset_manager.launch.py` passes `hooks.skip_components: [wall_distance]` while it is disabled, so `/reset_all` does not **start** it through `/restart_component`;
- docs updated: `docs/bringup_checklist.md`, `src/f1tenth_perception/README.md`.

Turn it back on with `enable_wall_distance: true`. That only makes sense with `enable_scene_labels: false`.

## 3. ekf_cost_observer — off by default

It publishes `ekf_cost_observer: …` statuses on the shared `/diagnostics`. The only reader is `scripts/ekf_cost_report.py`, offline, from a bag. There is no control-path consumer.

Change: `enable_ekf_cost_observer` default `true` → `false` (`localization.launch.py`). The comment there says how to turn it on under the supervisor: `args: {enable_ekf_cost_observer: 'true'}` on the `localization` entry in `components.yaml`.

`/diagnostics` stays in the mission logger's topics, because battery and calibration statuses share it.

Open item: `docs/history/LOG.md:59-62` still waits on a 50 Hz cost baseline taken with this instrument.

Likely cause of its 32%, from the code: 6–7 RELIABLE depth-500 Python subscriptions to the 50 Hz EKF topics, with `/odometry/filtered` subscribed twice.

## 4. costmap_boundary, semantic_layer, reset_manager

### The MPC's own rates
- Control loop: `self.ts = 0.1`, **hardcoded**, 10 Hz (`MPC_corr.py:740, 1918`). There is no param.
- Corridor rebuild: not a timer. A gate inside the control loop, `corridor_update_period = 1.0 s` (`MPC_corr.py:1179-1182, 3866-3885`), only reached with an active goal.

### The three nodes

| node | ran as | idle work | change | old → new |
|---|---|---|---|---|
| costmap_boundary | timer `extraction_rate_hz` 20, SingleThreaded | 4x redundant crop/rotate of ~41k cells per tick; a TransformListener that deserialises every `/tf` (2x50 Hz) for a buffer only the polytope path (off) reads | default 10; TF listener only with `use_convex_polytope` | **20 → 10 Hz**; linus 23.3% → 11.5% (no `/tf`: 19.2 → 11.4) |
| semantic_layer | no timer, all per message; TF listener | `/perception/walls` (~10 Hz): 240x240 raster + two 57,600-element `.tolist()` per message, even with no walls; `/tf` listener for a static camera chain | walls kept latest-only (depth 1), rasterised by a timer at `wall_layer_rate_hz` (launch arg `semantic_layer_rate_hz`); `array.array` instead of lists; TF from `/tf_static` only (`tf_static_only`, default true) | **walls ~10 → 5 Hz**; per rebuild 4.25 → 0.38 ms (no walls), 6.2 → 2.1 ms (3 walls); detections unchanged |
| reset_manager | MultiThreadedExecutor(4), reentrant; 4 always-on subscriptions + a 20 Hz no-op timer | ~150 Odometry/s handed to worker threads, 20 empty timer calls/s | subscriptions and hold timer created inside `/reset_all`, destroyed in its `finally` | **20 Hz timer + subs → event-driven**; linus 10.9% → 0.0% |

Why reset_manager showed 36% and 9 threads:
- There was no busy-wait or polling.
- The cost was always-on subscriptions to two 50 Hz EKF outputs, plus `/odom` and `/slam/pose`. Every message was handed from the executor's wait thread to one of 4 workers, and a permanent 20 Hz timer did nothing.
- The 9 threads are 1 wait thread + 4 workers + the DDS threads, all of them waking for that traffic.
- All of that data is read only during a reset. The hold timer stays at 20 Hz while a reset is holding the car: it is a stop path, so it was not lowered.

### Consumers and their staleness gates (each must be ≥ 2x the new period)

| output | consumer | gate | new period | ok |
|---|---|---|---|---|
| `/costmap/boundaries` | MPC_corr hard constraints (`MPC_corr.py:1713-1717, 3224-3248`) | `odom_stale_timeout_sec` 0.5 s; never-received → `[]` | 0.1 s | 0.5 ≥ 0.2 ✔ |
| `/costmap/front_clearance` | lidar_front_wall_node seed (`lidar_front_wall.py:312-323`) | `lidar_front_wall_seed_max_age_sec` 0.5 s | 0.1 s | ✔ (stack_params text updated) |
| `/costmap/front_clearance` | steering_offset_calibration | `clearance_stale_sec` 1.0 s | 0.1 s | ✔ |
| `/costmap/front_clearance` | BT | no longer read (it reads `/perception/virtual_wall`) | — | — |
| `/costmap/wall_cells` | mission logger only | — | 0.2 s | ✔ |
| `/costmap/wall_cell_ids` | none | — | 0.2 s | ✔ |
| `/costmap/semantic_tracks` | BT GoToObject, `object_tracks_max_gap_sec` 0.5 s | **unchanged**, still per detection batch | — | ✔ |
| `/costmap/semantic_markers` | costmap_renderer (2 Hz, no gate) | unchanged | — | ✔ |

Why the detection path still does work per message: the track lifecycle counts **batches** (`confirm_hit_count`, `lost_miss_count`, `confirm_grace_misses`). Rate-limiting it would change object confirmation and loss for go_to_object. With the ZED off (the measured case) it gets no messages at all.

## 5. Pinning — proposal only, not applied

Current pins come from the launch-file `taskset` defaults. With wall_distance off, **core 5** holds:
- lidar_front_wall_node (the tracker, ~35%);
- swept_clearance_node and obstacle_clearance_node (both light, but their CPU was not in the given numbers).

That frees about 42 points, which were wall_distance's.

| node | now | proposed | why |
|---|---|---|---|
| ekf_filter_node, ekf_global | 0,1 | 0,1 | unchanged |
| slam_toolbox | 2 | 2 | unchanged |
| foxglove_bridge, viz_relays | 3 | 3 | unchanged |
| behavior_executor_node | 4 | **4** | e-stop core, now ~47% est. |
| lidar_front_wall_node (tracker) | 5 | **5** | as asked |
| swept_clearance, obstacle_clearance | 5 | 5 | `/scan` consumers kept off the perception and e-stop cores; room freed by wall_distance |
| wall_distance_node | 5 | (off) 5 if enabled | — |
| detection_3d, obstacle_projector | 6,7 | **6,7** | as asked |
| yolo_detector | 8,9 | **8,9** | as asked |
| mpc_corr | 10,11 | **10,11** | as asked |
| semantic_layer_node | floating | **6,7** | same pipeline as detection_3d (consumes `/camera/detections_3d`); idle with the ZED off. Check 6,7 headroom with the ZED on |
| costmap_boundary_node | floating | **2,3** | aux cores; feeds MPC at 10 Hz, ~24% est. |
| costmap_renderer_node | floating | **2,3** | 2 Hz viz |
| reset_manager | floating | **2,3** | ~0 idle |
| component_supervisor, diagnostics_server, system_observer, battery_voltage_check | floating | **2,3** | low, housekeeping |
| mission_logger (when on), ekf_cost_observer (when on) | floating / 2-11 | **2,3** | recording / debugging; measure phase b first |
| urg_node | floating | **4** (optional) | puts the e-stop's sensor driver on the e-stop core, safe from a saturated 5 or 2,3; verify 4's headroom first |
| ZED wrapper, VESC driver, ackermann_mux | floating | leave | not measured here |

`scripts/check_cpu_pinning.py`'s `NODE_CORE_MAP` covers neither lidar_front_wall_node nor any of the floating nodes. Add them when the pins are applied.

## 6. Final checks (linus)

### Compile and lint
- `py_compile`: all 23 touched `.py` files OK.
- flake8, E/W/F codes (max-line 99): **no new errors** on any touched file versus `9c46add`.
- ament `test_flake8`/`test_pep257` (docstring and import-order rules): no new findings in f1tenth_bringup, costmap, localization, behavior or perception. The one changed line is `test_wall_distance_component.py`'s existing I100, whose import list grew.
- Those ament lint tests already fail at 9c46add: 267 findings in bringup, plus `test_copyright`.

### pytest, cpu-cleanup against 9c46add

**Passing, with the same failures before and after:**
- f1tenth_behavior: 418 passed.
- f1tenth_bringup: 42 passed, 3 ament lint tests failing.
- f1tenth_localization: 15 passed.
- f1tenth_params: 54 passed.
- f1tenth_diagnostics: 13 failed / 139 passed.
- mpc_controller: 53 failed / 157 errors / 511 passed.
- perception and costmap, run per file: everything that imports passes, including `test_wall_distance_launch` (19).

**Failing already at 9c46add (pre-existing):**
- `f1tenth_behavior test_go_to_object_behaviour.py::TestFailureOutcomes::test_grace_then_target_lost`
- `mpc_controller test_d_wall_correction.py::TestTheMissionThatExercisesIt::test_every_other_wall_turn_mission_is_still_terminal_on_the_turn` (`drive_stop_2m_from_wall.json`)
- the diagnostics and mpc_controller sets above. Most look like Jazzy/Humble or environment differences; not investigated.

**Skipped on linus, with the reason:**
- The Jazzy `f1tenth_messages` install lacks `MultiWallTrack`, `MultiWallTrackArray` and `StereoColumns`. Affected:
  - perception: `test_front_clearance_node`, `test_front_depth_monitor_node`, `test_lidar_front_wall_node`, `test_yolo_detector_node`;
  - costmap: `test_semantic_layer_node`, `test_semantic_tracks_stamp`, 7 tests in `test_wall_raster`.
  - **The costmap ones were also run with a local stand-in module for the missing messages: all pass**, including the 3 new semantic-layer tests. The stand-in is in the session scratchpad and is not committed.
- `f1tenth_logger` `test_extract_render_split` and `test_object_run_summary`: no `pyarrow`.
- `test_zed_extrinsic`: 2 failures, pre-existing.

### grep for remaining references
The node was gated rather than deleted, so its code, tests and docs remain. What remains:
- **wall_distance_node:** its own code, launch file and tests; stack_params comments; MPC_corr's warning text, which only fires if messages arrived and then stopped; `swept_clearance.launch.py` comments; docs; the replay tool; msg comments.
- **ekf_cost_observer:** its own code and tests; `localization.launch.py`, where the arg now defaults to false; `ekf_cost_report.py`; `battery_voltage_check` and `package.xml` comments; `docs/history/LOG.md`.
- Nothing launches either node by default any more.

### Launch description diff (textual, 9c46add → cpu-cleanup)
```
behavior_bringup.launch.py
  + ARG  bt_tick_hz = 10.0
  + ARG  enable_proximity_event_stop = True
  + behavior_executor_node param bt_tick_hz = $(bt_tick_hz):float
  + behavior_executor_node param enable_proximity_event_stop = $(enable_proximity_event_stop):bool
  (node-local, not in launch: bt_keep_qos_event_handlers=false; bt_loop_duration_ms removed)
wall_distance.launch.py
  + ARG  enable_wall_distance = False
  ~ wall_distance_node condition: $(use_lidar)  ->  '$(use_lidar)'=='true' and '$(enable_wall_distance)'=='true'
reset_manager.launch.py
  + reset_manager param hooks.skip_components = ['wall_distance']   (while enable_wall_distance is false)
localization.launch.py
  ~ ARG  enable_ekf_cost_observer = true -> false
costmap.launch.py
  ~ ARG  costmap_boundary_extraction_rate_hz = 20.0 -> 10.0
  + ARG  semantic_layer_rate_hz = 5.0
  + semantic_layer_node param wall_layer_rate_hz = $(semantic_layer_rate_hz):float
  (node-local defaults: semantic_layer tf_static_only=true)
component_supervisor (not a launch file): auto-start set loses 'wall_distance'
  (now _CONDITIONAL_AUTO_START on enable_wall_distance)
```
Nodes added: none. Nodes removed from the default bringup: wall_distance_node, ekf_cost_observer_node (both by condition).

## To verify on the Orin

1. **The CPU numbers.** Run `cpu_test.sh` phase a before and after, using the commands below.
2. **Event e-stop.** With the car on blocks and the stack up, put a hand within 15 cm of the front bumper:
   - `ros2 topic echo /safety_stop` should show `frame_id: base_link/emergency` at ~40 Hz;
   - `/behavior/tree_status` should show the emergency lane on the next tick.
3. **Tick timing.** `ros2 topic echo /diagnostics | grep -A20 'BT tick timing'` should give p95 well under 50 ms, `rate_hz_measured` ≈ 10, and 0 overruns with the visualizer on.
4. **QoS handlers.** The BT log should show `Dropped N default QoS event handlers`, with no errors at shutdown (Ctrl-C).
5. **semantic_layer `tf_static_only`.** With the ZED on, `/costmap/semantic_tracks` should still fill, with no `tf2 lookup … failed` WARN.
6. **costmap_boundary at 10 Hz.** `ros2 topic hz /costmap/boundaries /costmap/front_clearance` should read ≈ 10, and mpc_corr should log no stale-boundary messages during a corridor run.
7. **`/reset_all` end to end.**
   - It should still succeed.
   - The step 1 standstill wait should now include DDS discovery of the fresh subscriptions, well under the 5 s timeout.
   - `top -H -p $(pgrep -f reset_manager)` should show ~0% while idle.
8. **wall_distance is off.**
   - `ros2 node list | grep wall_distance` should be empty.
   - The supervisor log should not list it under "Auto-starting".
   - MPC_corr should not WARN "Is wall_distance_node running?".
9. **lidar_front_wall** on core 5 should no longer saturate: `top -H`, per-core view.

### cpu_test.sh, phase a — before and after
The stack must be up first; phase a measures it with the car stationary and the mission logger off. Use the same ZED state for both runs.
```bash
# ---- BEFORE (campaign-final, as built now) ----
# terminal 1 (Orin)
cd ~/dev_ws/f1tenth_more && git switch campaign-final && source install/setup.bash
ros2 launch f1tenth_bringup supervisor_bringup.launch.py
# terminal 2, after ~60 s, car stationary, ZED off as in the original measurement
cd ~/dev_ws/f1tenth_more && source install/setup.bash
tools/cpu_test.sh --phase a --out output/cpu_test/cpu_cleanup_before
# Ctrl-C terminal 1

# ---- AFTER (cpu-cleanup) ----
cd ~/dev_ws/f1tenth_more
git fetch andreas@linus:dev_ws/f1tenth_more cpu-cleanup:cpu-cleanup && git switch cpu-cleanup
colcon build --symlink-install --packages-select f1tenth_params f1tenth_behavior \
    f1tenth_bringup f1tenth_costmap f1tenth_localization f1tenth_perception
source install/setup.bash
ros2 launch f1tenth_bringup supervisor_bringup.launch.py        # terminal 1
tools/cpu_test.sh --phase a --out output/cpu_test/cpu_cleanup_after   # terminal 2
# compare output/cpu_test/cpu_cleanup_{before,after}/summary.md
```

## Commits (`git log 9c46add..cpu-cleanup`)
| sha | point |
|---|---|
| `69d2e1c` | 1 — BT: vectorised e-stop scan, QoS handler drop, render off-thread, `bt_tick_hz`, tick timing, event-driven e-stop |
| `686d911` | 2 — wall_distance_node off by default (`enable_wall_distance`) |
| `f2b4393` | 3 — ekf_cost_observer off by default |
| `752f160` | 4 — costmap_boundary 10 Hz, semantic wall layer 5 Hz, reset_manager event-driven |
| `9dde6ee` | 1 (follow-up) — tick timer handed to the tree so `tree.shutdown()` stops it |
| (this report) | output/cpu_cleanup/{report,decisions,log}.md |

## Bring the branch to the Orin
linus over Tailscale: user `andreas`, repo at `/home/andreas/dev_ws/f1tenth_more`, MagicDNS `linus` (100.89.193.76). sshd is active on linus. The branch lives in that repository; it was made in a worktree, which shares the branches.
```bash
# on the Orin
cd ~/dev_ws/f1tenth_more
git fetch andreas@linus:dev_ws/f1tenth_more cpu-cleanup:cpu-cleanup && git switch cpu-cleanup
# or with a named remote:
git remote add linus andreas@linus:dev_ws/f1tenth_more   # once
git fetch linus cpu-cleanup && git switch -c cpu-cleanup linus/cpu-cleanup
# without MagicDNS: andreas@100.89.193.76:dev_ws/f1tenth_more
```
