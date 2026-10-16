# cpu-cleanup decisions

## D0 — worktree instead of `git switch` in the main checkout
- Decision: `git worktree add -b cpu-cleanup ~/dev_ws/f1tenth_more_cpu_cleanup 9c46add`.
- Alternatives: `git switch -c cpu-cleanup 9c46add` in ~/dev_ws/f1tenth_more (as written in the brief).
- Why: the main checkout is on `jazzy-campaign` with uncommitted work (dirty submodules
  zed_ros2_wrapper / zed-ros2-interfaces, untracked output/sim_next, src/husarion_gz_worlds,
  docs/branches.md). Switching it to a Humble commit would move submodule gitlinks under
  that dirty state and swap the Jazzy sources under any later linus build. The worktree
  shares the same object store and branch namespace, so the branch is still `cpu-cleanup`
  in the repo at andreas@linus:dev_ws/f1tenth_more and the Orin fetch command is unchanged.

## D1.1 — the BT's 86% is NOT the tick; measure it on linus instead of guessing
- Decision: reproduce the parked-car load on linus: the campaign-final BT source run under
  Jazzy rclpy (PYTHONPATH, no colcon build of the Humble branch) against synthetic inputs at
  the Orin's real rates (/scan 1081 beams @ 40 Hz, /odometry/filtered and
  /ekf_global/odometry/filtered @ 50 Hz, /perception/virtual_wall @ 40 Hz,
  /mpc/min_obstacle_distance(+_forward) @ 20 Hz, /tf_static). Harness in the session
  scratchpad (bench/run_bench.py, fake_inputs.py). CPU = utime+stime of the process from /proc.
- Alternatives: static reading only (the brief's default).
- Why: static reading found no busy-wait (tick is py_trees_ros tick_tock = an rclpy timer at
  bt_loop_duration_ms=100, behavior_executor_node.py:718-769; no spin_once/sleep/sync calls
  in any update()). Without a measurement the fix would have been aimed at the wrong thing.
  Findings (x86 i7-9700K; the Orin's A78AE runs this Python ~3-4x slower, which is how
  ~22% here becomes ~86% there):
  * no inputs at all: 5.7% -> the tick + timers + viz are small; tick p50 0.38 ms.
  * /scan alone adds ~8.7%; both odometries alone add ~10%: the cost is per received
    MESSAGE, not per tick.
  * per message: rclpy executor wake-up re-adding/re-checking 76 default QoS event
    handler waitables (removing only them: 21.7% -> 16.3%), deserialisation, then the
    callback; IsProximityTooClose._scan_callback was a pure-Python per-beam loop
    (0.54 ms/scan) plus an rclpy throttled logger call that costs 0.24 ms EVERY call
    even when throttled (is_proximity_too_close.py:277-319 at 9c46add).
  * the BT PNG visualizer (enable_bt_visualization: true) ran graphviz `dot` as a
    subprocess inside a post-tick handler every 0.4 s and WAITED for it: ~80 ms of
    executor blocking per render (behavior_executor_node.py:263-280 at 9c46add). Not much
    CPU in this process (dot is a child), but it is a blocking call inside the tick that
    delays the /scan callback, i.e. the e-stop.

## D1.2 — bt_tick_hz default 10 (not 20) + event-driven lidar e-stop
- Decision: bt_tick_hz (stack_params, default 10.0 = the old bt_loop_duration_ms=100),
  and enable_proximity_event_stop (default true): IsProximityTooClose publishes the
  emergency Stop's exact message (safety_stop, frame_id base_link/emergency) from its
  /scan callback when a scan trips. The tree is unchanged and still trips on its tick.
- Alternatives: 20 Hz tick (brief's default); tick-only e-stop.
- Why: the brief says 20 Hz unless the e-stop needs more, in which case event-driven +
  tick <= 20. Tick-only worst case at 1.0 m/s is 13 cm (21 cm with a render in the way)
  against a 15 cm trip distance, so the e-stop does need more -> event-driven, ~30 ms,
  3 cm at 1.0 m/s. With the e-stop off the tick, 10 Hz is kept because mission
  stop_conditions count TICKS (condition_eval.ConditionDebouncer; debounce_ticks: 3 in
  wall_turn_then_straight.json and drive_straight_to_wall_3m.json = 300 ms at 10 Hz,
  would silently become 150 ms at 20 Hz), and the tick is 0.4% of a core anyway.

## D1.3 — drop rclpy's default QoS event handlers in behavior_executor_node
- Decision: after every publisher/subscription exists, remove and destroy the default
  incompatible-QoS event handlers (qos_events.py); param bt_keep_qos_event_handlers
  (default false) keeps them.
- Alternatives: keep them (no CPU gain); pass event_callbacks=...(use_default_callbacks=False)
  at each of ~40 create_* call sites incl. py_trees_ros internals (not reachable).
- Why: ~25% of the node's CPU for a log line; incompatible QoS fires on BOTH ends and every
  peer keeps its handler, so a mismatch is still reported. Each entity's event_handlers list
  is emptied too, because rclpy's destroy_publisher/subscription removes them from the node
  again (would raise ValueError at shutdown otherwise) -- covered by a real-rclpy test.

## D1.4 — PNG render moved to a worker thread; scan loop vectorised; DIAG log pre-throttled
- Render: the tick freezes the statuses (_BehaviourSnapshot) and a daemon thread builds the
  pydot graph, runs dot and publishes; newest snapshot wins. Same image (test compares the
  dot source of snapshot vs live tree).
- Scan: numpy over the same per-sample arithmetic with cos/sin tables cached per
  (layout, laser yaw); 0.54 -> 0.13 ms/scan on linus. Equal to the old loop within 1e-12
  on random scans with nan/inf/below-range_min, four laser poses and unequal windows. Not
  claimed bit-identical: numpy's cos/sin may differ from libm by 1 ulp, which can only move
  a sample lying exactly on a window edge.
- DIAG line: same 2 s cadence, throttled before the f-string and the rclpy logger call.

## D2 — wall_distance_node: NOT deleted, gated behind enable_wall_distance (default false)
- Decision: stack_params `enable_wall_distance: false`; supervisor moves `wall_distance`
  from _ALWAYS_AUTO_START to _CONDITIONAL_AUTO_START (still registered, restartable by
  name); wall_distance.launch.py declares `enable_wall_distance` (default from
  stack_params) and gates the node on use_lidar AND it; reset_manager.launch.py passes
  `hooks.skip_components: [wall_distance]` while disabled so /reset_all does not START it
  through /restart_component. Code, tests, params, msgs, replay tool kept.
- Alternatives: (a) delete node + wall_distance.py + glass_detect + params + msgs + MPC
  psi_correction path; (b) leave it running.
- Why: the brief deletes only if no consumer outside the package, or all have an
  equivalent. There is one: MPC_corr.py:1750-1755 subscribes
  /perception/d_wall/psi_correction and adds it to psiEnd on a straight move after a
  wall_turn (MPC_corr.py:4538-4544). The multi-wall tracker (lidar_front_wall_node ->
  /perception/walls, MultiWallTrack) has no signed left-positive base_link offset, no
  phase and no psi correction, so it is not equivalent. That consumer HAS a fallback:
  _fresh_d_wall_correction (MPC_corr.py:3305-3333) returns 0.0 silently when nothing ever
  arrived. And in the default config it is 0.0 anyway: the phase machine only leaves
  UNKNOWN on /mpc/wall_track, published only by MPC_corr's private wall tracker
  (MPC_corr.py:1909-1911), which enable_scene_labels (default true) switches off
  (MPC_corr.py:1085-1089). -> the brief's "behind a launch arg, default false, only if the
  consumer has a fallback" case.
- Other consumers checked: BT wall stop/wall_turn/preflight read /perception/virtual_wall
  (lidar_front_wall_node), never /perception/d_wall*; no subscriber for /segment,
  /gate_margin, /swept_arc; viz relay /perception/d_wall -> /viz/... (viz only; relays an
  absent topic harmlessly); mission logger records none of d_wall*; cpu_test_probe pattern
  (monitoring only).

## D3 — ekf_cost_observer: launch arg default false; code, tests, report script kept
- Decision: localization.launch.py `enable_ekf_cost_observer` default 'true' -> 'false'
  (the arg and the IfCondition already existed). Nothing removed from the mission
  logger's default topics.
- Alternatives: delete the node; drop /diagnostics from the logger's _DEFAULT_TOPICS.
- Why: its only output is DiagnosticStatus 'ekf_cost_observer: ...' on the SHARED
  /diagnostics; the only reader is scripts/ekf_cost_report.py, offline, from a bag; no
  control-path subscriber (BT reads /diagnostics/battery_status and /system_status only;
  MPC/costmap/navigation read no /diagnostics). The brief says keep it behind a launch arg
  default false. The logger records /diagnostics, not an observer-specific topic, and
  /diagnostics also carries battery_voltage_check and steering calibration statuses, so it
  stays recorded; with the observer off it just has no ekf_cost_observer entries.
- Open planning dependency (docs/history/LOG.md:59-62): the 50 Hz -> 20 Hz EKF change was
  "gated on capturing the 50Hz cost baseline first" with this instrument. Turning it on for
  that run: components.yaml localization entry `args: {enable_ekf_cost_observer: 'true'}`.
- Likely cause of its 32% (from code, not measured): 6-7 RELIABLE depth-500 Python
  subscriptions to 50 Hz EKF topics, /odometry/filtered subscribed twice
  (ekf_cost_observer_node.py:196-236).

## D4.1 — costmap_boundary: extraction_rate_hz 20 -> 10; TF listener only with the polytope
- Decision: keep the existing param/launch-arg names (extraction_rate_hz /
  costmap_boundary_extraction_rate_hz), default 10.0; create the TransformListener only
  when use_convex_polytope is true.
- Alternatives: rename to costmap_boundary_rate_hz (brief's naming); 5 Hz.
- Why: renaming breaks tools/front_phantom_replay.py, which reads the launch defaults by
  name (costmap_boundary_ + extraction_rate_hz). 10 Hz = mpc_corr's control rate
  (MPC_corr.py:740 ts = 0.1, hardcoded, the only MPC timer, :1918); every consumer gate
  is >= 2x the new 0.1 s period: MPC odom_stale_timeout_sec 0.5 (MPC_corr.py:3224-3248),
  lidar_front_wall_seed_max_age_sec 0.5 (lidar_front_wall.py:312-323), steering
  calibration clearance_stale_sec 1.0. Going to 5 Hz would leave MPC re-using a boundary
  for 2 control ticks for no measured need. The listener's buffer is read only by
  _map_from_odom, reached only on the polytope path (off by default, read once at start).
- linus bench (synthetic /slam/map, global odom 50 Hz, /tf 100 Hz): 23.3% -> 11.5%; without
  /tf 19.2% -> 11.4% (so ~4 points was the listener).

## D4.2 — semantic_layer: wall layer latest-only at wall_layer_rate_hz=5; detections per batch
- Decision: /perception/walls callback only stores the message (depth 1); a 5 Hz timer
  rasterises the newest; message arrays filled with array.array instead of .tolist();
  TF buffer fed from /tf_static only (param tf_static_only, default true); launch arg
  semantic_layer_rate_hz -> node param wall_layer_rate_hz.
- Alternatives: rate-limit the detection path too (brief: "no work per received message").
- Why detections stay per message: the track lifecycle counts BATCHES (confirm_hit_count,
  lost_miss_count, confirm_grace_misses -- costmap.launch.py says so in their
  descriptions); dropping batches would change confirmation latency and miss counting,
  i.e. object behaviour for go_to_object. /costmap/semantic_tracks is gated at 0.5 s by
  the BT (object_tracks_max_gap_sec), which a 5 Hz throttle would only just meet. And the
  input is 0 Hz with the ZED off -- the measured 41% had no detections at all.
- Why the walls can drop: the layer is rebuilt from scratch per message (never
  accumulated), /costmap/wall_cells is only recorded by the mission logger and
  /costmap/wall_cell_ids has no consumer. 5 Hz <= mpc 10 Hz.
- Why /tf_static only: the one lookup is camera -> base_link; every joint on that chain is
  fixed (sensors.xacro:110-128), same reasoning yolo_detector_node/front_clearance_node
  already use. If a moving joint ever enters it, the lookup fails loudly (throttled WARN,
  batch skipped) and tf_static_only:=false restores the listener.
- linus microbench, one wall rebuild: 4.25 -> 0.38 ms with no walls, 6.2 -> 2.1 ms with 3
  walls; x 10 -> 5 Hz. TF listener: ~4 points of a linus core at 100 /tf msgs/s (measured
  on costmap_boundary, same listener).
- Tests touching MultiWallTrack cannot import on linus (the Jazzy f1tenth_messages install
  lacks MultiWallTrack/MultiWallTrackArray); they were run with a LOCAL-ONLY stand-in
  module (session scratchpad msgshim/sitecustomize.py, not committed): all pass.

## D4.3 — reset_manager: event-driven, nothing subscribed or timed between resets
- Decision: the 4 subscriptions and the stop.rate_hz hold timer are created at the start
  of /reset_all and destroyed in its finally. stop.rate_hz stays 20 Hz while holding (the
  mux's safety_stop lane times out at 0.2 s; this is a stop path, not lowered).
  MultiThreadedExecutor(4) kept: /reset_all blocks in its callback while service
  responses and topic samples arrive on the other threads.
- Alternatives: 1-2 Hz timer + keep subscriptions (still ~150 deserialisations/s);
  fewer executor threads (idle threads cost nothing; fewer risks a deadlock).
- Why: cause of the 36% / 9 threads was always-on subscriptions to two 50 Hz EKF outputs
  plus /odom (+ /slam/pose) read only during a reset, each message handed from the wait
  thread to a worker, plus a permanent 20 Hz no-op timer. No busy-wait. Every wait in the
  steps already accepts only samples newer than the step start (or the last 0.5 s for the
  standstill), so subscriptions created at reset start see what permanent ones would have.
  New risk: DDS discovery of the fresh readers (~10-100 ms locally) eats into stop.timeout_s
  (5 s) -- negligible, but a "to verify on the Orin" item.
- linus bench (odom, global odom, /odom at 50 Hz): 10.9% -> 0.0%.
