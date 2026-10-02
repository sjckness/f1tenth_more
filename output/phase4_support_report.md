# Phase 4: behaviour tree, logger, diagnostics, LLM (offline, Thor)

Previous phase: `output/phase3_control_report.md`, verdict **GO-WITH-NOTES**
(Orin comparison pending, declared non-blocking). Gate read before starting.

## Verdict: **GO-WITH-NOTES — Orin BT comparison pending; live-LLM check awaits your build decision**

- **Two real Jazzy breaks found and fixed, both in `f1tenth_logger`.**
  - On Jazzy, `mission_logger_node` recorded **nothing**: `rosbag2_py.RecordOptions.all`
    no longer exists.
  - With that fixed, every MCAP run (the logger's Jazzy default) failed its
    extract step, because `mission_extract` hard-coded `sqlite3`.
  - Both were found by running the node end to end on Thor. Neither was caught
    by the unit tests (292 pass with or without the bugs).
- **Behaviour tree:**
  - py_trees 2.5.0 / py_trees_ros 2.5.0. The node uses only API that is
    present and unchanged.
  - 343/343 tests pass, 20 runs out of 20 (10 idle, 10 under 2× CPU
    oversubscription).
  - The replay loads and starts the real mission through the real services.
    It stops 0.21 s after the stop signal crosses 2.0 m, as `debounce_ticks: 3`
    dictates. The stop reason, the post-stop tree-status sequence and the
    published `/mpc/goal_drive` match the recorded live Humble run exactly.
  - Three Jazzy runs agree to within 2 ms.
- **Diagnostics:** every node starts and either runs or exits with a clear
  message when there's no hardware, except `system_observer_node`. That one
  crashes on **Thor's** sysfs, a platform condition reproduced identically
  under Python 3.10. `ekf_cost_observer` matches the recorded Orin output
  wherever the inputs exist.
- **LLM translation is identical across distros:** all 10 golden missions
  reproduce exactly on Jazzy/Python 3.12 and in a Humble/Python 3.10
  container. The 12 known stub failures are unchanged.

**Pending, both non-blocking for Phase 5:**
- Humble BT replays and a Humble extract on the Orin (`output/phase4/ORIN_BT_INSTRUCTIONS.md`).
- A live llama-server check on Thor. llama.cpp and the model aren't on Thor;
  building them needs your go-ahead (Decision 1).

---

## Code version for this phase's parts

`bc3b049 → e47e646` and `e47e646 → HEAD` for `f1tenth_behavior`,
`f1tenth_logger`, `f1tenth_diagnostics`, `f1tenth_intelligence`,
`f1tenth_messages` and `f1tenth_params`:
- 2 comment-only lines in `f1tenth_behavior`, plus 2 docstring lines in
  `f1tenth_logger/test_campaign/corridor_plot.py`;
- `stack_params.yaml`'s 4 comment-only lines (parsed values identical; Phase 3);
- Phase 0's install-hint string and test fixes in `f1tenth_logger`.

So the bag's recorded BT output is a valid Humble reference for this code.

---

## Step 1: behaviour tree (`f1tenth_behavior`, py_trees_ros from apt)

### Inventory (from code)

`behavior_executor_node`, launched by `behavior_bringup.launch.py`
(`taskset -c 4`, with `twist_to_ackermann_node`; `enable_nav2: false`):
- builds `py_trees_ros.trees.BehaviourTree(root)`;
- `tree.setup(timeout=60)`;
- `tree.tick_tock(period_ms=bt_loop_duration_ms=100)`, i.e. **10 Hz**, on an rclpy timer.

Root Selector, `memory=False`:

1. **`emergency`:** `IsBatteryLow` (`/diagnostics/battery_status`), `IsEmergencyStopTriggered`
   (`/mission/status`), `IsProximityTooClose` (`/scan`, 0.15 m / 0.15 m),
   `IsSystemOverheated` (`/diagnostics/system_status`, since `enable_sys_obs: true`;
   load trip off) → `Stop` on `/safety_stop` (`frame_id base_link/emergency`).
2. **`handle_obstacle`:** off (`enable_camera_obstacle_stop: false`).
3. **`mission`:** `MissionActive` → Selector[`ObjectSeen`+`HandleObjectAction`,
   `PublishMoveGoal` → `GoToObject` → `CheckStopCondition` → `AdvanceMove`].
4. **`navigation`:** `HasMpcGoal` (`/mpc/goal_distance`).

| Inputs | Outputs |
|---|---|
| `/odometry/filtered` (`CheckStopCondition`, via `get_odom_topic()`), `/ekf_global/odometry/filtered`, **`/perception/front_distance`** (the front_clearance stop), `/mpc/min_obstacle_distance[_forward]`, `/mpc/goal_reached`, `/mpc/object_status`, `/scan`, `/diagnostics/system_status`, `/diagnostics/battery_status`, `/camera/detections` (`DetectedClassesBridge`), `/costmap/semantic_tracks` (`GoToObject`), `/goal_pose`, `/mpc/goal_distance` | `/mpc/goal_drive` and `/mpc/goal_distance` / `_pose` / `_turn` / `_object` / `_object_end`, `/mpc/hold`, `/safety_stop`, `/mission/status` (RELIABLE + TRANSIENT_LOCAL, published on change by a 0.1 s watch timer), `/mission/move_outcome`, `/behavior/tree_status` (every tick), `/safety/event`, `/test/mission_event`, `/bt/tree_visualization` |
| **Services served:** `/mission/load_mission`, `/mission/start_mission` (preflight, then a 3.0 s countdown), `/mission/abort_mission`, `/mission/emergency_stop`. **Topic:** `/mission/load_path` | mission report JSON in `src/f1tenth_behavior/mission_reports/` (gitignored) |

**Live start path, from the code:** `llm_planner_node` translates the intent,
writes `llm_generated/<id>.json`, then calls `/mission/load_mission` and
`/mission/start_mission`.

**Two facts that shaped the comparison:**

- **The BT's stop input was never recorded.** `CheckStopCondition` reads
  `/perception/front_distance` (ZED wall distance from `front_clearance_node`).
  Its docstring explicitly says this is *not* `/costmap/front_clearance`,
  which is the only clearance topic in the bag. The recorded
  `/costmap/front_clearance` is ≤ 2.0 m from t=5.2 s on, yet the live BT kept
  driving, which confirms it wasn't the input. The stop decision therefore
  can't be compared with recorded output directly.
- **Pre-existing finding (not changed):** `mission/preflight.py` requires a
  node **named `costmap_boundary_node`** for any front_clearance stop
  condition (from `f960cf2`, 2026-08-21). Since `63a6080` (2026-09-14) the
  condition reads `/perception/front_distance`, so this check verifies the
  wrong producer. It's pure Python, the same on both distros.

### py_trees / py_trees_ros versions and API

- **Thor/Jazzy:** `ros-jazzy-py-trees` 2.5.0, `ros-jazzy-py-trees-ros` 2.5.0,
  `py-trees-ros-interfaces` 2.1.2.
- **Orin/Humble:** not recorded anywhere on Thor. The Orin instructions record it.
- **API used** (grep of the package): `behaviour.Behaviour`, `composites.Sequence`
  and `Selector` (always with explicit `memory=`), `common.Status` / `Access`,
  `blackboard.Client.register_key`, `display.ascii_tree`, `isinstance` checks
  against `composites.Parallel` / `decorators.Decorator`, and
  `py_trees_ros.trees.BehaviourTree` (`__init__(root)`, `setup(timeout=)`,
  `tick_tock(period_ms=)`, `add_post_tick_handler`, `.node`, `shutdown()`).
  All are present in 2.5.0, with the signatures the code uses.
- **Changelogs** (upstream `CHANGELOG.rst` of both repos; the debs ship only
  `changelog.Debian`):
  - py_trees 2.3.0 #444 "Improve timing of tick_tock()". **Does not apply:**
    the node uses `py_trees_ros`'s own `tick_tock`, a re-implementation on an
    rclpy timer (checked in the installed source).
  - py_trees 2.4.0 (instance checks, `ForEach`, `CompareBlackboardVariables`)
    and 2.5.0 (ports/XML parser) are additive.
  - py_trees_ros 2.5.0: #250 "always publish snapshot on setup()", #249
    "setup() callable multiple times", #253 shutdown without destroying the
    node. The node calls `setup()` once and doesn't use snapshot streams.
  - The 2.2.x memory-semantics change predates 2.3. The code passes `memory=`
    explicitly everywhere.

### Replay (same inputs on both stacks; Humble side PENDING)

The harness is `replay_localization.sh` layer **`bt`**:
- The **production** `behavior_bringup.launch.py`, via `bt_replay.launch.py`
  (`use_sim_time` only), on isolated domain 78.
- Input bag: `/odometry/filtered`, `/ekf_global/odometry/filtered`, `/scan`,
  `/diagnostics/system_status`, `/camera/detections`, `/costmap/semantic_tracks`,
  and **`/costmap/front_clearance` renamed to `/perception/front_distance`**.
  That's a documented stand-in for the unrecorded input, fed byte-identically
  to both stacks (content digest `89146b2d…`).
- The mission is built by `make_mpc_goal.build_mission()` (translate, asserted
  equal to the golden fixture and the bag's id). It is written as the planner
  writes it, then loaded and started through `/mission/load_mission` and
  `/mission/start_mission` at **bag t = 36.5 s**. The 3 s countdown then makes
  it RUNNING at about 39.5 s, so the run ends on the same physical wall
  approach as the live run.
- Three name-only stub nodes (`mpc_corr`, `ackermann_to_vesc_node`,
  `costmap_boundary_node`) satisfy the preflight's node-existence checks.
  They publish nothing; no driver runs.

**Acceptance rule for Jazzy vs Humble, fixed now, before the data exists.**
Each Humble run must have:
1. the same published `/mpc/goal_drive` fields;
2. the same `/mission/status` sequence;
3. the same `/mission/move_outcome` fields (`mission_id`, `move_id`,
   `move_type`, `stop_reason`, `outcome`);
4. the same post-stop `/behavior/tree_status` transition sequence;
5. a stop delay after the stand-in's 2.0 m crossing within **one BT tick
   (0.1 s)** of the Jazzy runs.

On (5): the Jazzy floor is about 2 ms. One tick is the justified tolerance,
because the debounce counts 100 ms ticks, and a different machine could
legitimately land the decisive input sample on the neighbouring tick. Any
larger difference gets root-caused.

**Jazzy noise floor (3 runs):**

| | run 1 | run 2 | run 3 |
|---|---|---|---|
| mission RUNNING [bag s] | 39.508 | 39.506 | 39.506 |
| stand-in crosses 2.0 m | 41.948 | 41.948 | 41.948 |
| `/mpc/hold` true | 42.160 | 42.162 | 42.161 |
| stop delay | 0.2125 s | 0.2145 s | 0.2135 s |
| `/mission/status` | IDLE, LOADED, LOADED, RUNNING, COMPLETE | same | same |
| outcome | `stop_condition:front_clearance`, drive, `move_0_straight` | same | same |
| post-stop tree sequence | mission [F, S, I] → [F, F, F] | same | same |

- Pairwise: Δ hold ≤ 2 ms, and every equality check passes.
- The 0.21 s delay is `debounce_ticks: 3` at 10 Hz: three consecutive
  satisfied ticks after the crossing.
- **The BT's own `/mpc/goal_drive` equals Phase 3's injected message field
  for field.** That closes the loop on Phase 3's goal reconstruction.

**Sanity check against the recorded live Humble run (not a parity verdict):**
- Same `stop_reason` and move fields.
- Same post-stop tree sequence: mission [FAILURE, SUCCESS, INVALID] →
  [FAILURE, FAILURE, FAILURE].
- Same RUNNING → COMPLETE.
- Live hold at 42.420 s versus 42.160 s in the replay. Different stop input
  (the real `/perception/front_distance` versus the stand-in), so the times
  aren't comparable.
- Live duration 55.98 s; the replay is 2.6 s, by design.

Metrics: `output/phase4/bt_metrics.json`.

### Tests, 10× (the Phase 0 flaky test)

- **Idle:** 10 of 10 runs, **343 passed**, 3.62–3.73 s each.
- **Under `stress-ng --cpu 28`** (2× oversubscribed, unpacked in the
  scratchpad as in Phase 3): 10 of 10 runs, **343 passed**, 5.44–6.23 s.
- 20 runs, zero failures (`output/phase4/tests/`). Phase 0's single failure
  happened during a whole-workspace parallel `colcon test` and its report
  doesn't name the test. No timing sensitivity reproduces here, even under
  heavy oversubscription.
- The 3 warnings are `test_mission_report_atomic`'s `fork()` notices, as in
  Phase 0.

---

## Step 2: logger (`f1tenth_logger`)

### Two Jazzy breaks, fixed (evidence first)

`scripts/jazzy_parity/logger_check.sh` drives `mission_logger_node` through a
synthetic mission on isolated domain 79: RUNNING, 12 s of the original bag,
then COMPLETE.

1. **`RecordOptions.all` was removed in Jazzy's rosbag2_py 0.26.11**; it is
   now `all_topics` / `all_services`. Live log:
   `FAILED to start recording ...: 'rosbag2_py._transport.RecordOptions' object has no attribute 'all' -- the mission runs normally, but this run will not be captured.`
   **Every mission on Jazzy would have gone unrecorded, while running
   normally.** Fix (`de1f8d8`): set `all_topics` when present, `all`
   otherwise.
2. **`mission_extract.read_bag` hard-coded `storage_id='sqlite3'`.** With the
   recorder fixed, every MCAP run failed its finalize step
   (`Could not open '.../bag_0.mcap' with 'sqlite3' ... file is not a database`),
   so no `extract.parquet` was written. Jazzy registers `mcap` and the logger
   defaults to it. `read_bag` is the single bag-reading path for extract,
   render and replay video. Fix (`91d361e`): `mcap` when the bag holds a
   `.mcap` file, else `sqlite3`.

The unit tests stub the recorder, so neither break was visible to them. The
`f1tenth_logger` suite is identical before and after the fixes: 292 passed,
plus 3 pre-existing lint failures (`copyright`, `flake8`, `pep257`, with
3,517 flake8 findings either way). My edits leave the file-level flake8
count unchanged.

### End-to-end result after the fixes

| Check | mcap (default) | sqlite3 |
|---|---|---|
| single-instance lock: second logger refused (exit 1, names the lock and PID) | PASS | PASS |
| RUNNING → `active/<run_id>/` with start manifest and params snapshot | PASS | PASS |
| COMPLETE → whole run renamed to `complete/<run_id>/`, `active/` empty | PASS | PASS |
| manifest `outcome=COMPLETE`, `end_time` set | PASS | PASS |
| `bag_sha256` recomputed per file and equal; `bag_bytes` equal | PASS (`bag_0.mcap`, `metadata.yaml`) | PASS |
| params snapshot byte-identical to the `stack_params.yaml` in effect | PASS | PASS |
| storage as requested | PASS | PASS |
| `extract.parquet` written | PASS (226 kB) | PASS |
| snapshots | 10 from `/camera/image_annotated` | — |
| SIGINT shutdown releases the lock | PASS | PASS |

Output: `output/phase4/logger_check_{mcap,sqlite3}.txt`. The post-run
`systemctl --user start f1tenth-archive.service` sync is a no-op here: the
unit doesn't exist on Thor, and the check aborts if it ever does.

### rosbag2_py in production code (every call site)

- **`mission_logger_node.py`:** `get_registered_writers`, `StorageOptions(uri, storage_id, max_cache_size)`,
  `RecordOptions` (`all` → fixed; `topics`, `is_discovery_disabled`,
  `rmw_serialization_format`, `topic_polling_interval`,
  `topic_qos_profile_overrides` all exist), `Recorder().record(storage, record)`,
  `Recorder.cancel()`, and the snapshot `SequentialReader` + `StorageFilter`.
- **`mission_extract.py`:** `SequentialReader` (storage → fixed) + `StorageFilter`.
- Nothing else in `src/` outside tests uses rosbag2_py.
- **Note:** `Recorder()` and the two-argument `record(...)` are *deprecated*
  in Jazzy (a `DeprecationWarning`) but work. Not changed: no style fixes.

### Extract and render on MCAP and on all four archived `.db3` bags

There is **no HTML report generator in this repo.** The pipeline is
`bag → mission_extract (.extract.parquet) → mission_render (.mp4)`. All five
bags went through it on Jazzy; `output/phase4/reports/index.html` is a
harness page collecting the results, not a stack feature.

| Bag | Storage | Extract | MP4 | PNG frames (hash list) |
|---|---|---|---|---|
| Jazzy logger run `2026-10-02T13-23-13_mission-llm_2b356aac445f` | mcap | 0.23 MB | ok | 120 |
| `humble_obstacle_run` | sqlite3 | ok (7,667 rows) | ok | 426 |
| `2026-09-02T09-46-43_mission-bottle_then_person` | sqlite3 | 0.08 MB | ok | 98 |
| `2026-09-02T15-21-57_mission-bottle_then_person` | sqlite3 | 0.92 MB | ok | 131 |
| `2026-09-02T15-27-43_mission-bottle_then_person` | sqlite3 | 0.63 MB | ok | 93 |

- Same machine, re-rendered: all 98 frame hashes identical. Re-extracted:
  identical content and identical bytes.
- **Cross-distro (PENDING):** the Orin extracts `humble_obstacle_run` with
  Humble's code. `compare_extracts.py` compares table content, because
  parquet bytes embed writer metadata. Frame-hash lists are committed for
  any cross-machine render check (never MP4 checksums).
- The MP4s (14 MB) are not committed; they regenerate from the committed
  extracts.

---

## Step 3: diagnostics (`f1tenth_diagnostics`)

### Every node started, no hardware (`diag_start_check.sh`, isolated domain 80)

Plain `ros2 run` with the node defaults, deliberately **not** the calibration
launch files, which can bring up the VESC driver group.

| Node | Result |
|---|---|
| `battery_voltage_check_node` | exits 0: `no telemetry received on "/sensors/core" within 10.0s`, battery NOT measured (designed path) |
| `diagnostics_server_node` | runs; clean SIGINT stop (rc 0) |
| `ekf_cost_observer_node` | runs; clean SIGINT stop (rc 0) |
| `gyro_bias_calibration_node`, `sensor_covariance_calibration_node`, `slam_pose_covariance_calibration_node` | exit 2: `Stationary check failed ... "/sensors/core" never published. Aborting without sampling` (designed path; light-motion mode needs an interactive confirmation and is off by default) |
| `steering_offset_calibration_node` | runs, waits; on SIGINT `publishing zero velocity`, exits 10 (`EXIT_PREFLIGHT_REFUSED`) |
| **`system_observer_node`** | **crashes on the first tick (rc 1, traceback). Platform, see below** |

`vesc_msgs` imports fine everywhere.

**A harness artefact, investigated rather than reported as a Jazzy bug.** The
first run showed `ExternalShutdownException` tracebacks on SIGINT
(`diagnostics_server_node`, `ekf_cost_observer_node`).
- **Cause:** a background job of a non-interactive shell inherits SIGINT as
  *ignored*, so Python never installs `KeyboardInterrupt`.
- **The same probe on Humble's rclpy 3.3.21** (`f1tenth_more:jetson`
  container) and on Jazzy gives identical results: `KeyboardInterrupt` with
  default handling, and `ExternalShutdownException` on both when SIGINT is
  inherited as ignored (`output/phase4/probes/sigint_probe_results.txt`).
- The check now restores `SIG_DFL` before `exec`, and both nodes stop cleanly.
  No stack change.

**`system_observer_node` on Thor: a platform condition, not a distro issue.**
- `jtop` isn't installed in Thor's Jazzy environment, so the node uses its
  sysfs fallback.
- On Thor, reading `/sys/.../thermal_zone1` (`gpu-thermal`) returns EAGAIN
  whenever the GPU is idle. Python's buffered `read()` then returns `None`,
  `.strip()`/decode raises `TypeError`, and the handler only catches
  `(OSError, ValueError)`.
- Reproduced with the identical code under **Python 3.10 (Humble container)**:
  it reads fine right after a container start wakes the GPU, and fails after
  10 s idle (`output/phase4/probes/thermal_probe_results.txt`).
- Pure Python, pre-existing, platform-dependent: out of scope and not
  changed. Its relevance to the target is Decision 3.

### Replay vs recorded output: `ekf_cost_observer_node` (`ekf_cost_observer` layer, 3 runs)

Inputs `/odom`, `/odometry/filtered`, `/ekf_global/odometry/filtered`,
`/slam/pose_calibrated` and `/diagnostics` (robot_localization's own
FrequencyStatus), with the observer's own recorded statuses removed. Production
defaults, wall clock, as live. Per-window fields are compared as distributions,
since 1 s wall windows don't align.

| Field (median over windows) | Jazzy ×3 | floor | recorded live (Orin) |
|---|---|---|---|
| local/global `ticks_inproc` | 504 / 503 | 0 / 0 | 504 / 503 |
| local/global `ticks_selfcount` | 47–48 / 47 | 1 / 0 | 48 / 47 |
| local/global `tick_count_agreement` | 10.52–10.70 / 10.69–10.72 | 0.18 / 0.03 | 10.52 / 10.70 |
| local/global `period_ms_p50` | 20.01 / 20.02 | <0.01 | 20.01 / 20.01 |
| local/global `period_ms_max` | 38.46 / 39.98–39.99 | 0 / 0.01 | 37.76 / 39.96 |
| global `period_ms_p90` | 34.45–34.68 | 0.22 | 33.46 |
| global `meas_delivered` / `meas_per_tick` | 49 / 0.098 | 0 | 49 / 0.097 |
| local `period_ms_p90` | 32.7–34.4 | 1.7 | 27.4 (see below) |
| local `meas_delivered` | 50 | 0 | 100 (see below) |

- **Local `meas_delivered` 50 vs 100:** the local EKF's IMU input
  `/sensors/imu/raw` (about 50 msg/s) was never recorded. A structural gap
  in the bag.
- **Local `period_ms_p90`:** recomputed from the *same* recorded header stamps
  at 20 window offsets, the median p90 ranges from 26.5 to 34.6 ms. Both the
  live 27.4 and the replays' 32.7–34.4 lie inside that range. The periods are
  mostly exactly 20 or 40 ms, so a per-window p90 is window-alignment
  sensitive. Not a behaviour difference.
- **Process-derived fields** (`pid`, `cpu_ms_per_*`, `cpu_percent_of_core`)
  come from `/proc` of live EKF processes. None run in an isolated replay
  (status "ekf_filter_node not found in /proc", WARN). They are
  platform-dependent by nature; not compared.

### Platform-dependent fields (document, don't compare)

- **`/diagnostics/system_status`** (`system_observer_node`): `cpu_percent`,
  `cpu_per_core` (12 entries on the Orin, 14 on Thor), `ram_used_mb` /
  `ram_total_mb` (Orin 62,841 MB), `cpu_temp_c` (Orin 54.7–55.3 °C), and the
  jtop-only `gpu_percent`, `gpu_temp_c`, `emc_percent` (0.0 without jtop;
  Orin recorded gpu 5.6–99.2 %, gpu_temp 48.7–49.3 °C, emc 0).
- **`ekf_cost_observer`:** `pid`, `cpu_ms_per_tick`, `cpu_ms_per_meas`, `cpu_percent_of_core`.
- **`battery_status` / calibrations:** VESC hardware only.
- The BT's `IsSystemOverheated` consumes `system_status`. Its thresholds
  (85 °C, load trip off) act on platform values by design.

Metrics: `output/phase4/ekf_cost_observer_metrics.json`,
`output/phase4/diag_start_check.txt`.

---

## Step 4: LLM (`f1tenth_intelligence/llm`)

### Inventory

- **`llm_planner_node`** reaches llama-server through its raw **`/completion`**
  endpoint, `llm_url` default `http://127.0.0.1:8083/completion`, using plain
  `requests`.
  - Sampling: `temperature 0.0`, `n_predict 512` (warm-up `n_predict 8`), no
    seed parameter, no grammar/json_schema.
  - Output goes through `plan_translate.translate()` (jsonschema +
    `mission_config` validation), then `/mission/load_mission` and
    `/mission/start_mission`.
  - `go_to_enabled` comes from `stack_params`.
- **llama-server:** started by `llm.launch.py` as an `ExecuteProcess` (the
  `intelligence` component of `component_supervisor_node`).
  - Binary `llama_server_path = /scratch/fabiocar/llama.cpp/build/bin/llama-server`,
    cwd `/scratch/fabiocar/llama.cpp` (`stack_params`).
  - Model from `config/models.yaml`: `qwen25_3b_instruct` =
    `~/projects/llm/models/qwen2.5-3b-instruct-q5_k_m.gguf`, `--port 8083`,
    `-ngl 999` (all layers on the GPU).
- **What the Docker target needs if llama-server stays on the Orin host:**
  - point `llm_url` at the host (with `--network host`, 127.0.0.1 keeps
    working);
  - don't let the in-container `intelligence` component try to start the
    binary, since `llama_server_path` is a host path;
  - keep port 8083 free.

### Tests and golden translation

- **llm suite on Jazzy: 231 passed, 25 skipped, 14 failed.**
  - The 14: the **12 known** `_note_llm_sent` stub failures (7× `_Stub`,
    5× `_StubNode`, pre-existing since `dc158d2`, Phase 0 A4) plus
    `test_flake8` and `test_pep257` (style debt, not touched).
  - Skips: mainly `test_intent_evals_live` (needs llama-server on :8083).
  - Everything else passes, including all 97 tests in `test_intent_translate.py`.
- **Golden fixtures, cross-distro** (`scripts/jazzy_parity/llm_golden_check.py`):
  all **10/10** golden missions reproduce exactly on Jazzy/Python 3.12 *and*
  in the Humble container under Python 3.10 (same jsonschema 4.10.3). The
  per-fixture canonical-JSON sha256s are identical
  (`output/phase4/llm_golden_*.txt`).

### Live LLM on Thor: not run, needs your go-ahead

Thor has **no llama.cpp, no model file and no `/scratch`**. It does have what
a CUDA build needs:
- CUDA 13.0 from JetPack 7.1 (`/usr/local/cuda-13.0`, `nvcc` present but not
  on PATH);
- the GPU "NVIDIA Thor", compute capability 11.0, driver 580.00;
- cmake 3.28, 118 GB of free RAM, 795 GB of free disk.

Build steps, per llama.cpp's `docs/build.md` (CUDA section:
`cmake -B build -DGGML_CUDA=ON` then `cmake --build build --config Release`;
"By default llama.cpp will be built for the hardware that is connected to the
system", and `-DCMAKE_CUDA_ARCHITECTURES` overrides it):

```bash
git clone https://github.com/ggml-org/llama.cpp ~/llama.cpp && cd ~/llama.cpp
export PATH=/usr/local/cuda-13.0/bin:$PATH
cmake -B build -DGGML_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=110
cmake --build build --config Release -j 12
# model: the same file the Orin uses -- copy it from the Orin
#   (~/projects/llm/models/qwen2.5-3b-instruct-q5_k_m.gguf, ~2.3 GB) rather than
#   re-downloading, so the comparison uses byte-identical weights
build/bin/llama-server -m <gguf> --port 8083 -ngl 999
```

The steps use no pip CUDA or torch packages, only the apt JetPack toolkit
that's already installed. Planned check, once built: run the planner against
it with the golden prompts and judge on **mission equality after
translation**, not raw text. The same llama.cpp commit should be used on the
Orin for a like-for-like comparison.

---

## Separate, non-migration proposal: the mission logger's default topic list (describe only, NOT applied)

**Change** `f1tenth_logger/mission_logger_node.py` `_DEFAULT_TOPICS` by
appending:
`/mpc/goal_drive`, `/imu`, `/joint_states`, `/perception/front_distance`,
`/perception/d_wall/psi_correction`, `/test_campaign/logger_status`,
`/mpc/status`.

**Why:** Phase 3 had to reconstruct the unrecorded drive goal. Phase 4 found
the BT's actual stop input (`/perception/front_distance`) unrecorded. Both
the straight-drive heading correction (`d_wall/psi_correction`) and the
OSQP iteration counts (`/mpc/status`) are invisible in every bag today.

**Before applying, on the car:** check each publisher's QoS
(`ros2 topic info -v <topic>`). `/imu` and `/joint_states` come from drivers.
Any BEST_EFFORT publisher must also go into `_DEFAULT_BEST_EFFORT_TOPICS`,
or the recorder silently captures nothing for it (the trap the node's own
docstring describes).

Two further notes:
- `/sensors/imu/raw` (the local EKF's IMU input) is also unrecorded; it would
  have made `ekf_cost_observer`'s local measurement counts comparable.
- Starting the recording before the move starts (this bag began 13.5 s in)
  would have made the goal heading recoverable.

---

## What isn't covered, and where it goes

- **BT Jazzy vs Humble, Humble py_trees versions, cross-distro extract
  content:** pending, Orin (`output/phase4/ORIN_BT_INSTRUCTIONS.md`).
  Comparison commands pre-written; acceptance rule above.
- **The BT with its real stop input:** needs a bag that records
  `/perception/front_distance` (the proposal above).
- **Live llama-server planning on Thor:** pending your build decision; on the
  Orin in the container later.
- **`system_observer_node` and `IsSystemOverheated` on real Orin values:**
  in the Orin container (phases 6–8).
- **Branches not exercised:** emergency-lane trips, the `on_object` /
  `go_to_object` missions, abort and estop services; the calibration nodes'
  measurement paths (hardware).

---

## Commits

```
de1f8d8 jazzy/p4: f1tenth_logger: RecordOptions.all -> all_topics — Jazzy rosbag2_py removed 'all', so no mission was ever recorded
91d361e jazzy/p4: f1tenth_logger: read_bag picks the storage plugin from the bag — mcap runs failed extraction with a hardcoded sqlite3
3968af0 jazzy/p4: bag_digest hashes message content — raw CDR bytes include uninitialised padding
c437903 jazzy/p4: BT and ekf_cost_observer replay layers — Phase 4 needs both replayed identically on Jazzy and Humble
8cba2b3 jazzy/p4: logger, diagnostics and LLM check scripts — end-to-end evidence for Phase 4 Steps 2-4
(next)  jazzy/p4: Phase 4 results, Orin BT instructions and report — GO-WITH-NOTES, Orin BT comparison pending
```

All local and unpushed.
- `de1f8d8` and `91d361e` are the only production-code changes (the 2 logger fixes).
- `3968af0` also corrects the Phase 3 Orin instructions and report. The old
  raw-byte digest would have made the Orin's input-bag check fail spuriously.

---

## Decisions for Andreas

1. **Build llama.cpp (CUDA, sm_110) on Thor and copy the Orin's .gguf, for
   the live-LLM check?** Steps above. Nothing is built or downloaded until
   you say so.
2. **Run `output/phase4/ORIN_BT_INSTRUCTIONS.md` on the Orin** (3 BT replays,
   one extract, about 5 min, plus a 250 MB copy). It can be combined with the
   pending Phase 2 and Phase 3 Orin runs in one session.
3. **`system_observer_node` crashes when the GPU thermal sensor returns
   EAGAIN.** On Thor that happens whenever the GPU idles. On the Orin
   natively, jtop is used instead, so the fallback never runs. **In the
   Jazzy container on the Orin**, jtop is only available if the container
   gets jtop and the `/run/jtop.sock` socket; otherwise this fallback runs.
   Whether the Orin's GPU sensor also returns EAGAIN when idle is unknown.
   A crash there would also starve the BT's `IsSystemOverheated` of data. A
   two-line fix (treat a `None` read like `OSError`) is available; it's
   pre-existing and platform-dependent, so I haven't applied it. Your call:
   fix now, or verify on the Orin container first.
4. **The preflight's stale `costmap_boundary_node` requirement** for
   front_clearance missions. This is pre-existing; the condition reads
   `/perception/front_distance` from `front_clearance_node`. A mission could
   start with `costmap_boundary_node` up and `front_clearance_node` down,
   although the blackboard-liveness half of the same check would catch
   missing data. Recommend aligning it; not changed here.
5. **Approve or reject the logger topic-list proposal above** (separate from
   the migration).
6. **Style-test debt is visible on Jazzy.** `test_flake8`, `test_pep257` and
   `test_copyright` fail in `f1tenth_logger` and `llm`; flake8 alone reports
   3,517 findings in `f1tenth_logger`. This is pre-existing. Untouched under
   the no-style-fixes rule.
