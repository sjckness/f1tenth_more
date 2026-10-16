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

# Round 2

## R2-D1a — Humble compatibility: no required change to commits 1-4; drop hardened anyway
- Read against the Humble sources copied from the Orin (humble_src/: rclpy 3.3.21, py_trees_ros
  2.4.0 pkg / version.py 2.2.2): Node.publishers/subscriptions/waitables (rclpy/node.py:231/236/261),
  remove_waitable (:1213), entity.event_handlers list (publisher.py:57, subscription.py:70),
  destroy_publisher/subscription iterate it (node.py:1562/1580), QoSEventHandler.destroy
  (qos_event.py:125); py_trees_ros BehaviourTree sets self.timer=None in __init__ (trees.py:270)
  and shutdown() cancels/destroys it (:549-551). All 22 changed .py files parse with
  ast feature_version=(3,10). Humble makes ONE default handler per entity (qos_event.py:160-214),
  Jazzy about two, so the BT saving on the Orin is smaller than the linus figure -> to measure.
- Decision: still harden: drop_qos_event_handlers probes every internal first and is a no-op
  with exactly one WARN if any is missing; per-handler node+entity consistency so a failure
  part-way never leaves a handler destroy_* would remove twice; never raises.
  Alternatives: leave as is (works on Humble today). Why: the brief asks for it, and an rclpy
  upgrade must not be able to take the e-stop node down at startup.
- Found by the review and fixed here (not Humble-specific):
  * reset_manager (commit 4) destroys watch subs/timer from a worker thread at the end of
    /reset_all; a sub already marked ready raises InvalidHandle in the worker's take
    (Humble executors.py:385-400, :477-478, re-raised :857-860) and would end the node.
    main() now spins with spin_once in a loop that drops InvalidHandle. Alternative:
    keep subscriptions permanent (back to the idle cost).
  * _PngRenderWorker.stop() now joins (1 s) and the thread never publishes after stop:
    a render in flight could publish on a destroyed node at shutdown.
  * bt_keep_qos_event_handlers was node-local, unreachable from launch: now stack_params +
    typed launch arg.
  * wall_distance.launch.py condition: PythonExpression(... == 'true') rejected `1`; now
    AndSubstitution (Humble launch/substitutions/boolean_substitution.py:68), same truthiness
    as IfCondition.
  * costmap package.xml: exec_depend tf2_msgs (semantic_layer imports TFMessage directly).
  * stale tick_tock/bt_loop_duration_ms mentions in docs and two docstrings.

## R2-D1b — event e-stop: one predicate, tree holds as long as the mux
- Scope (understand/estop_scope.md): one construction site (create_root, gated only by
  enable_lidar_safety_stop at construction, so both paths exist or neither); update() reads no
  blackboard key and no runtime param; mission phases, countdown, preflight, /mpc/hold,
  go_to_object/docking (gap_min >= 0.42 m > 0.15 m), operator e-stop latch never gate it; no
  reverse move type exists (drive speed>=0, turn speed>0, MPC max_reverse_speed_mps=0.0); the
  emergency lane is root child 0 of memory=False Selectors, re-ticked every tick.
- Decision: should_stop() = _active() and _tripped() decides for BOTH the callback and update();
  _active() is the single place for any future gate (today True). update() also reports
  SUCCESS for event_stop_hold_s (0.2 s = mux safety_stop timeout) after the last callback stop.
  Structural test: emergency = root.children[0], memory=False, no decorator, no blackboard.
- Alternatives: leave update() = _tripped() only. Why not: a trip shorter than a tick was
  actuated by the callback (mux holds ~200 ms) but invisible in /behavior/tree_status and
  /safety/event -- exactly the "callback stops while the tree has resumed" state the brief
  forbids. Cost: after such a trip the tree's own Stop runs on the next tick, so the hold can
  end up to one tick later than the callback's alone.
- Remaining "tree stops, callback does not" states, all fail-safe: another emergency cause
  (battery/operator/overheat); <=1 tick status lag after a clear scan; /scan stopping after a
  tripping scan (tree stops until a scan arrives). Pre-existing gaps reported, not changed:
  no /scan freshness guard (a stalled /scan after a clear scan disarms both paths), preflight
  does not require /scan, rear side window measured from the laser.

## R2-D1c — semantic_layer_tf_static_only in stack_params with the camera-pan warning
- The node param default now comes from stack_params (single source), with the comment "must be
  false if the camera pan joint is ever enabled" next to it. Alternative: comment only in the
  node. Why: the brief asks for it in stack_params, and a key there is where an operator looks.

## R2-D2a — rate audit
- Full table with every subscriber and gate: output/cpu_cleanup/rate_audit.md (static, cited;
  a skeptic pass appended corrections at its end). Summary in report.md, Round 2.

## R2-D2b — EKF 50 -> 20 Hz via stack_params, offset 0.08 with it, tracker gates raised
- Decision: stack_params ekf_frequency_hz 20.0 and ekf_transform_time_offset_s 0.08, applied by
  ekf.launch.py and ekf_global.launch.py as parameter overrides on top of the yaml (the yaml keeps
  its documented 50 Hz values, marked OVERRIDDEN). Both filters read the same two keys.
  lidar_front_wall_pose_max_extrapolation_sec 0.15 (was a hardcoded 0.1 in PoseHistory.at calls)
  and lidar_front_wall_odom_max_age_sec 0.2 -> 0.3. check_ekf_update_rate.py reads the target
  from stack_params (+ --target-hz).
- Alternatives: edit the yaml frequency directly (not "a param in stack_params", and the two files
  could drift); keep the offset at 0.05 (the 2026-09-08 decision says 0.08 lands with 20 Hz).
- Consumer verdicts (rate_audit.md 2.2): MPC (odom_stale_timeout 0.5 s, turn unwrap per control
  tick), ekf_global (sensor_timeout 0.1 = 2 input periods, predict-only on a late message, by
  design), reset_manager (time-based standstill), steering calibration (/slam/pose only), BT
  (per-message turn unwrap, step 0.085 rad << pi), semantic_layer (200-sample history = 10 s),
  costmap_boundary, slam_toolbox (transform_timeout 0.2), python tf2 users (Time() latest): OK.
  The one gate at exactly 2x the 20 Hz period was PoseHistory.at(0.1) in lidar_front_wall_node:
  a None there makes the tracker step NO_DATA -> /perception/virtual_wall -1 -> the BT holds.
  Raised to 0.15. No debounce anywhere counts odometry samples.
- The 50 Hz cost baseline was not taken: docs/history/LOG.md entry "50 Hz EKF cost baseline:
  PENDING-DECISION", left for the owner.

## R2-D2c — /perception/virtual_wall only on a new estimate, <= ~10 Hz
- Decision: lidar_front_wall.virtual_wall_due(): with the tracker on publish only on scans the
  tracker processed (the only ones that change _front_d_ray / _tracker_has_data /
  _front_coast_expired), without it every processed scan; capped by
  lidar_front_wall_virtual_wall_rate_hz (10) with a 75 % tolerance so an early tracker step is
  not dropped. The value and WallEstimate state are still computed every scan; WallLineFit /
  WallEstimate stay per scan (their documented contract; viz/debug only).
- Alternatives: heartbeat at <= 20 Hz on skipped scans (audit's suggestion: keeps today's
  receipt gaps). Why not: the brief asks "only when there is a new estimate"; every receipt gate
  is 0.5 s (BT wall_distance_max_age_s, MPC corr_wall_turn_front_distance_max_age_sec), >= 2x the
  0.1 s period and > the ~0.17 s of a CPU-bound 6 Hz tracker. debounce_ticks counts BT ticks
  (10 Hz), now ~1 new estimate per tick instead of ~4 duplicates.
- Side effect: while the laser transform is missing (startup) nothing is published (before:
  the last tracker verdict at scan rate); consumers then treat it as absent (BT holds, MPC drops
  the wall_turn cap) -- the same outcome as NO_DATA.

## R2-D2d — no other change needed for the BT; other >20 Hz streams listed, not changed
- After 2b the BT's only >20 Hz input is /scan (safety, 40 Hz, kept). Its odometries are 20 Hz,
  virtual_wall <= 10 Hz. So no raw=True subscription (it exists in Humble rclpy node.py:1332, but
  only saves Python message construction, not the executor wake-up).
- Not consumed by 10 Hz control/BT, so outside 2d's scope and left alone, listed as proposals:
  MPC_corr / detection_3d / obstacle_projector full /tf listeners (static-only would do for the
  last two), MPC_corr per-message throttled logger calls, swept_clearance (no consumer) and
  obstacle_clearance (campaign logger only) at 40 Hz, /perception/lidar_front_wall(_virtual)
  per scan (debug/viz). 2e: /scan, the event e-stop and the mux safety_stop lane unchanged.

## R2-D3 — wall tracker: phantom FRONT from collinear pieces across an opening
Root cause (static reading, bag not on linus): the tracker's extent was an ever-growing union
(MWT _update/_merge took min/max of endpoints), association needed only 0.2 m overlap and
_merge had only the chi2 line test, so the left and right faces of the corridor mouth merged
into one 13.67 m line spanning the opening; is_front never looked at the beams in the strip,
so a line the LiDAR saw straight through stayed FRONT and was dead-reckoned to 1.87 m.
- 3a Measured extent: each track keeps its associated observations' odom endpoints with stamps;
  t0/t1 are rebuilt every step from those of the last lidar_front_wall_wall_extent_decay_sec
  (1.0 s), keeping the newest and only intervals linked to it by a gap <= the collinear gap.
  FRONT distance = where the travel strip (|y| <= body 0.136 + lidar_front_wall_front_strip_margin_m
  0.15 = 0.286, from the bumper) meets the measured SEGMENT; no intersection -> no FRONT
  (ray_margin slack removed). Alternative: keep the union and only add the beam veto -- rejected,
  the union is what built the bridge.
- 3b Collinear gap 0.5 m (lidar_front_wall_wall_assoc_max_gap_m) in association AND merge,
  replacing the 0.2 m overlap test; counters gap_rejected / merge_gap_refused + reason codes.
  Only the part of the gap the beams saw OPEN counts: a raw 0.5 m rule split a wall behind a
  person (cut beams, 1.52 m "gap") into two tracks and gave CLEAR with a wall ahead (existing
  M02 test). Alternative: raw geometric gap -- rejected for that reason.
- 3c Strip negative evidence: nearest valid in-strip return (cut beams excluded, no return =
  None, not inf); 2 consecutive cycles with it > predicted + 0.5 m (front_contradict_margin_m,
  front_contradict_cycles) mark the track contradicted -> role other, not an occluder, kept
  alive so the wall behind becomes visible. Applied to every confirmed track meeting the strip
  (LiDAR-fitted or not, glass not exempt): goes beyond the brief, because a single-scan bridge
  from the extraction's 1.5 m merge is fitted every cycle. Cost: a pane the LiDAR sees through
  in the strip reads CLEAR (front_clearance / IsProximityTooClose remain the guard).
  Coasting only while the strip's nearest return is not beyond the wall, capped at
  wall_tracker_front_max_coast_sec 0.5 (was 0.3) or ..._front_max_coast_m 0.3, first wins.
  Also fixed: negative_evidence counted cut (object-sector) beams as PASS.
- 3d FRONT gate front_reason(): confirmed, not contradicted, role != other, meets the strip,
  coast caps, confidence >= front_min_confidence 0.5, and (LiDAR fit in the last
  front_lidar_recent_cycles 3 OR rect-VERIFIED). Confidence = mean of the last 5 matched
  observation confidences, raised to the rectangle support when VERIFIED (stereo-only is capped
  at 0.35 by wall_fusion, so only verified stereo passes).
- 3e Contract: the NO_DATA coast latch is gone; no FRONT -> CLEAR (50 m); NO_DATA only with no
  pose / no laser transform. Returns closer than front_clear_range_m (2.0) with no FRONT stay
  CLEAR with front_reason 'unexplained_returns' (debug only): inventing a WALL from raw returns
  is the phantom path again, NO_DATA would hold the car on data that is not missing, and the
  raw safety net is IsProximityTooClose.
- 3f Stereo observations with |rho_bl| < wall_obs_stereo_min_rho_m 0.4 dropped and counted.
  Self polygons: two added (lower-right corner x 0.80-1.0 / y 0.72-1.0, thin strut x
  0.735-0.775 / y 0.58-1.0), traced from frames of this run's camera.mp4 on linus (debug JPEG,
  ~1 % accuracy), UNTUNED, re-trace on raw /camera/image_raw. The rho filter alone removes the
  rho ~0.20 m tracks.
- Also: geometric_role got 0.186 in its front_x slot (fixed to the bumper x); _merge keeps the
  max of miss counts and the full history window.
- Test changes encoding old behaviour: an unverified glancing stereo-only wall is no longer
  FRONT (renamed test); node coast test now expects CLEAR; TestTrackerDecimation updated for
  block 2c (virtual wall on tracker steps only).

## R2-D4a — recording profile 'campaign' (mission_logger_profile)
- Decision: stack_params mission_logger_profile ('default' | 'campaign'); the node adds the
  profile's topics (only adds, never removes; unknown profile -> WARN + default); the launch
  file starts a topic_tools throttle /camera/image_raw -> /camera/image_raw_5hz and widens the
  depth throttle to record_depth OR campaign. Recorded: /perception/scene_labels/classes (the
  scene-label image, at its native 5 Hz mask tick), /camera/image_raw_5hz, depth_5hz +
  camera_infos, /perception/wall_observations (already in the default list).
- Alternatives: /camera/detection_masks (YOLO instance mask, not the class labels the replay
  draws); /camera/image_raw/viz (already 5 Hz, but only exists with foxglove up); also record
  /perception/scene_labels/depth_mm (+138 MB/min; the only depth that shares a stamp with the
  classes, needed to feed the recorded pair to the rectangle check) -- not in the owner's list,
  left out, noted as an option. Name: 'campaign' as asked, although a separate
  test_campaign_logger exists.
- Size (from measured message sizes in archived bags + arithmetic): classes 230 kB x 5 Hz = 69
  MB/min; image 922 kB x 5 Hz = 276; depth_5hz 922 kB x 5 Hz = 277 (already with
  record_depth); wall_observations 1.2 kB x 10 Hz = 0.7. Measured default+depth baseline ~308-330
  MB/min (mostly empty masks) -> campaign ~650 MB/min, 800+ with objects in view.

## R2-D4b — corridor_wall_replay.py v2 (sub-agent, reviewed here)
- Mask drawn from the recorded /perception/scene_labels/classes (--mask auto|recorded|replay);
  "MASK NOT RECORDED" / "DEPTH NOT RECORDED" instead of tinting the debug JPEG; the recomputed
  mask only on request, labelled. floor_inferred drawn yellow (tool only; scene_mask colours
  untouched) + per-class % strip and a class strip in the chart. Raw frame preferred when
  within 0.1 s; msg.step respected; MASK STALE back on canvas. Exactly one mask+depth source
  feeds the rectangle check (no double feed).
- Tracker run from the bag start on the car's own steps: old bags (debug every scan) ->
  tracker_skipped == 0; new bags (debug only on steps, block 2c) -> every payload; gate forced
  open and only step scans fed; virtual_wall_rate_hz cap disabled in replay; --t0/--t1 crop
  only summary/video. Alternative: feed every scan with a per-scan gate (estimator fields would
  match better, but the old installed node would publish payloads for skipped scans).
- Metrics for recorded and replay passes: phantom FRONT episodes (median value - strip <
  -0.5 m, an empty strip counts as -inf, so empty-strip WALLs are phantoms), end-wall error
  (moving cycles with true strip < 3 m; non-WALL there reported as misses), NO_DATA % (time-
  weighted, holds capped at 0.5 s, all and moving), confirmed tracks with rho_bl < 0.5 m;
  --compare-summary prints old vs new. Split bags read in numeric order.
- 4c: the bag is not on linus -> Orin commands in report.md.

# Round 3

## R3-D5 — EKF decided at 20 Hz; ekf_cost_observer deleted
- Owner's decision recorded in docs/history/LOG.md (the PENDING entry replaced): EKF stays
  at 20 Hz, the 50 Hz cost baseline is deliberately skipped, ekf_cost_observer removed.
- Deleted: f1tenth_diagnostics/ekf_cost_observer_node.py, its test, scripts/ekf_cost_report.py,
  the setup.py entry point, the localization.launch.py args (enable_ekf_cost_observer,
  ekf_cost_observer_cpu_affinity) and Node, and rosidl_runtime_py from the diagnostics
  package.xml (only that node used it; diagnostic_msgs and python3-psutil stay: battery check,
  steering calibration, system_observer). Not referenced in components.yaml, the supervisor,
  stack_params, the CPU-pinning checker or the cpu_test probe (grep).
- /diagnostics stays in the mission logger topics.
- Remaining mentions outside docs/history: output/cpu_cleanup/* (this work's own records) and
  one regression test, test_ekf_global_config.py::test_the_ekf_cost_observer_is_gone, which
  asserts the arg and node are absent. Alternative: no test -- rejected, the launch file is
  where it would come back.
