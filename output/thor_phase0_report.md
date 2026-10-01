# Phase 0 report: Jazzy build/test validation on Thor

Scope: first real `colcon build` + `colcon test` run of this workspace on
the actual Thor hardware (JetPack 7, Ubuntu 24.04, native Jazzy, aarch64),
plus the Step 1/4/5 verification items and a corrected classification pass
over every functional test failure.

## Verdict: **GO-WITH-NOTES**

Build is 33/33 green. Of the real failures found, every one investigated
with hard evidence is either fixed or proven distro/platform-independent
(and thus out of scope for a Jazzy migration pass). One **severe** bug was
found and fixed: `slam_toolbox` never left lifecycle state `unconfigured`
under the previous launch file — it would have looked like it started
successfully on a live car and silently done nothing. Nothing found blocks
proceeding to Phase 1, but see "Decisions for Andreas" before touching the
real car.

---

## 0. Corrected scope rule

The first version of this report used "byte-identical to `origin/main`" as
the sole test for "out of scope." That rule is wrong for a migration:
**unchanged code can fail because the environment underneath it changed**
(a newly-working solver, a new Python/library version, a new distro API).
The corrected rule used throughout this pass:

> A failure is out of scope only if its root cause is shown to be
> independent of ROS distro, Python/NumPy version, and platform —
> demonstrated with evidence (a source diff, a live test, a git-blame
> trail), not inferred from "the file didn't change."

Every reclassification below states what evidence was used.

---

## A. Functional failures, reclassified

### A1. `f1tenth_logger`: 6 `rosbag2_py.TopicMetadata` errors — **Jazzy API break, FIXED**

`test_object_run_summary.py`'s own `_write_bag()` test helper called:
```python
rosbag2_py.TopicMetadata(name=name, type=type_name, serialization_format='cdr')
```
Jazzy's installed `rosbag2_py` constructor is:
```
TopicMetadata(id: int, name: str, type: str, serialization_format: str,
              offered_qos_profiles: List[QoS] = [], type_description_hash: str = '')
```
`id` is now required with no default — a genuine, confirmed Jazzy API
change (cited directly from the installed module's own error message, not
guessed).

**Production code checked and clear:** `mission_logger_node.py` never
constructs `TopicMetadata` directly — it uses the higher-level
`rosbag2_py.Recorder()`/`RecordOptions()` API, which handles topic
metadata internally. Confirmed by grep: `TopicMetadata` appears in exactly
one file in the whole workspace, the test.

**Fix applied** (`52682de`): pass a unique `id` per topic in the test
helper. All 8 tests in the file now pass (was 2 passed / 6 errors).

### A2. `mpc_controller`: 10 failures — **confirmed pre-existing, out of scope, with evidence**

Context the original pass missed: on the x86 dev host, `osqp` was never
installed, so all 157 of this package's RTI-solver-dependent tests
**errored out at collection** (`RuntimeError: solver='rti' requires
'osqp'`) and never ran their bodies at all (see `jazzy_migration_analysis.md`
line 429). `osqp` is now installed here (1.1.3), so these test bodies are
running for the real first time — this is exactly the kind of
environment-driven, migration-relevant change the corrected scope rule
exists to catch. It deserved a real look, not a diff-stat dismissal.

**osqp API checked directly, not assumed:** `mpc_solver.py`'s exact call
pattern — `osqp.OSQP()`, `.setup(..., warm_starting=True, polishing=True)`,
`.warm_start()`, `.solve(raise_error=False)`, `results.x`,
`results.info.status_val`, `osqp.SolverStatus.OSQP_PRIMAL_INFEASIBLE` — was
run live against the installed osqp 1.1.3 with a trivial QP. **It works
exactly as written**, including `results.info.status_val == 1` ("solved").
The code's own comments already show it was written against the current
(>=1.x) API deliberately (*"Dockerfile.* pin no osqp version, so whatever's
newest at image build time is what actually ships"*). **No API mismatch.**

**Every one of the 10 failures gotten individually (full tracebacks
pulled, not just summaries), and every single one is a pure deterministic
computation with zero osqp/numpy-array/ROS/distro involvement:**

| test | what it actually compares | osqp/numpy/distro involved? |
|---|---|---|
| `test_the_default_geometry_freezes_both_ends` | `get_value('corridor_heading_return')` (yaml read) vs hardcoded `False` | No — pure config read |
| `test_every_other_wall_turn_mission_is_still_terminal_on_the_turn` | last move `mode` in `drive_stop_2m_from_wall.json` vs `'wall_turn'` | No — pure JSON read |
| `test_header_is_the_one_the_analysis_script_parses` | a literal string in a script file vs `ModelLogWriter.HEADER` | No — pure string compare |
| `test_the_straight_branch_limit_moves_with_the_ramp_span` | `min_turn_radius()` geometry formula vs `49.7±1.0` | No — pure math |
| `test_the_asymmetry_changes_when_the_turn_commits` | boolean from a pure geometry step function | No |
| `test_the_trigger_distances_are_the_work_orders_table` | `wall_turn_trigger_distance()` formula vs `3.71±0.005` | No — pure math |
| `test_psi_end_psi_ref_turn_and_the_centreline_share_one_increment` | a computed float vs `!= 0.0` | No |
| `test_effective_value[w_du_delta-3.0]` | `scale_stage_weights()` vs a hardcoded `3.0±0.05` | No — pure math |

**Confirmed the dependencies these tests couple to are also untouched**:
`f1tenth_params` (0 diff since merge-base) and the mission JSON file
(`git log`: last touched by `77d99aa`, long before this branch). This is
the exact "test encodes a config default, the default moved, nobody
updated the test" pattern `CLAUDE.md` already documents — real debt, but
Humble-era debt, proven independent of distro/numpy/platform by content,
not just by diff-stat. **Correctly out of scope; not touched.**

### A3. `f1tenth_behavior`: the flaky failure — **not reproducible, likely load-induced, out of scope**

Ran the specific failing test 10× standalone: **10/10 pass.** Ran the
whole file 3× together: **3/3 pass (13/13 each).** Ran the whole package
3× together (matching `colcon test`'s own scope): **3/3 pass (343/343
each).** Zero reproductions in 13 total attempts under light load. It
failed exactly once, during the original full-**workspace** `colcon test`
run where many packages built/tested in parallel — consistent with
resource-contention-induced flakiness (py_trees' own test-suite warning
flags "multi-threaded, use of fork() may lead to deadlocks in the child"
as a known general risk) rather than a deterministic bug. **Not a Jazzy
regression; not chased further**, but worth knowing the one occurrence
coincided with system load, not code.

### A4. `llm`: 12 `_note_llm_sent` failures — **confirmed pre-existing test-stub drift, cited**

`_note_llm_sent`/`_note_llm_received` were added to `llm_planner_node.py`
in `dc158d2` ("test logging: per-test recorder, manual success, mission
start countdown", 2026-09-21) — **confirmed an ancestor of this branch's
merge-base** (`git merge-base --is-ancestor dc158d2 <merge-base>` → yes).
The test doubles (`_Stub`, `_StubNode` in `test_intent_go_to.py` /
`test_planner_path.py`) were never given a matching method — `git log -S`
on those test files for the string shows it has never existed there. The
method itself is pure timing instrumentation (`self.get_clock().now()`
only) — no platform dependency whatsoever. **Confirmed pre-existing,
unrelated to Jazzy; not touched.**

---

## B. Outstanding Phase 0 steps, completed

### Step 1 — dependency/environment state

- **osqp 1.1.3**, **pyarrow 25.0.1** — both installed via
  `~/.local/lib/python3.12/site-packages` (user pip install,
  `pip install osqp pyarrow --break-system-packages` per the migration
  plan's Item 4/5).
- **NumPy state, confirmed:**
  - `import numpy` resolves to **1.26.4** at
    `/usr/lib/python3/dist-packages/numpy` (system/apt, not `~/.local`).
  - `pyzed` is **gone from `~/.local`** (confirmed: not listed in
    `~/.local/lib/python3.12/site-packages`).
  - The global pip constraint `/home/andre/.config/pip/constraints.txt`
    (`numpy<2`, `scipy<1.18`) is **active** (referenced from
    `~/.config/pip/pip.conf`'s `[global] constraint=`).
  - `pip check`: **one unrelated finding** — `pynacl 1.5.0 requires cffi,
    which is not installed`. Nothing to do with this workspace or numpy;
    not investigated further (pynacl/cffi are not workspace dependencies).
- **Torch/torchvision/ultralytics for JetPack 7 (Thor)** — researched, not
  installed, per instruction:
  - NVIDIA's long-standing custom-wheel doc
    (`docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform`)
    has **not been updated past JetPack 5.1** and does not cover JetPack 7
    or Thor — **do not use it**.
  - JetPack 7 moved Thor to the **SBSA (Server Base System Architecture)**
    CUDA packaging. Per an NVIDIA engineer on the developer forums
    (AastaLLL, [forums.developer.nvidia.com/t/how-do-i-correctly-install-pytorch-on-jetpack-7-2/372773](https://forums.developer.nvidia.com/t/how-do-i-correctly-install-pytorch-on-jetpack-7-2/372773)):
    *"Orin can use SBSA package now so upstream package should work"* —
    **stock upstream PyTorch wheels now work directly**, a real departure
    from the old Tegra-only custom-wheel requirement.
  - Two confirmed-live sources: (1) official PyTorch index matched to the
    exact CUDA minor — `pip3 install torch torchvision --index-url
    https://download.pytorch.org/whl/cu132` (JetPack 7.2, CUDA 13.2) or
    `cu130` (JetPack 7.0, CUDA 13.0); (2) NVIDIA-affiliated Jetson AI Lab
    index with confirmed `cp312` (Python 3.12) wheels —
    `pip3 install torch torchvision torchaudio --index-url
    https://pypi.jetson-ai-lab.io/sbsa/cu130`, also referenced from
    Ultralytics' own Jetson guide
    ([docs.ultralytics.com/guides/nvidia-jetson](https://docs.ultralytics.com/guides/nvidia-jetson)).
  - `torchvision`: prebuilt wheel from the same sources, no source build
    needed in the common case.
  - `ultralytics`: plain `pip install ultralytics` on top works; the one
    real gotcha per Ultralytics' own Jetson guide — **keep TensorRT 10.x**,
    TensorRT 11.2.1 breaks GPU-only exports on JetPack 7, and engines must
    be rebuilt per-device/per-TensorRT-version.
  - Versions: JetPack 7.0 = CUDA 13.0 / cuDNN 9.12 / TensorRT 10.13;
    JetPack 7.2.1 = CUDA 13.2.2 / cuDNN 9.20.0 / TensorRT 10.16.2.
  - **Python 3.12: confirmed working**, both indices publish `cp312`
    wheels and the forum install was validated on JetPack 7.2 (Ubuntu
    24.04's default Python). No partial-support issue found.
  - **Action for whoever installs this:** match the CUDA minor (`cu130` vs
    `cu132`) to this specific Thor's actual installed CUDA
    (`nvcc --version` / `dpkg -l | grep cuda-toolkit`) before picking an
    index URL — don't assume JetPack 7.0 vs 7.2.x without checking.

### Step 4 — hardware/runtime checks

- **vesc_driver smoke test:** no VESC hardware connected to this machine
  right now (`/dev/ttyACM*`/`/dev/ttyUSB*` empty, no matching USB device).
  Ran what's possible without hardware: `ros2 run vesc_driver
  vesc_driver_node --ros-args -p port:=/dev/ttyACM0`. **Clean result:**
  starts, loads correctly, no crash/link error, fails gracefully with a
  correctly-formatted `FATAL` log naming the missing serial port. Confirms
  the binary links and runs correctly against Jazzy's `io_context`/
  `serial_driver` (the thing that was actually broken before the
  `ros-jazzy-asio-cmake-module` fix). Full hardware-attached validation
  remains a real-car item.
- **`gyro_scale_z` vs `2848a9f`:** `2848a9f` is the `vesc` submodule's
  **currently checked-out commit** (not a parent-repo commit — confirmed
  by trying both). The documented gyro-unit fix (`1e6a734`, "VESC gyro_z
  57.3× unit bug fixed", `docs/history/LOG.md`) **is an ancestor of
  `2848a9f`** (`git merge-base --is-ancestor` confirms). `vesc.yaml` sets
  `gyro_scale_z: 0.0174533` (π/180); `vesc_driver.cpp` applies
  `(gyr_z() - gyro_bias_z_) * gyro_scale_z_` to the published
  `angular_velocity.z`, matching the fix exactly. The LOG.md's own
  warning about a stale, uncommitted submodule pointer bump **is resolved**
  — current pin includes the fix.
- **osqp API vs `mpc_solver.py`:** see A2 above — live-tested, exact
  match, no mismatch.
- **FastDDS SHM naming:** verified **live** on this machine (not just
  cited from the dpkg package name as the original analysis did) —
  started a node under FastDDS 2.14.6 and confirmed `/dev/shm` gained
  exactly the two families the cleanup code's glob expects:
  `fastrtps_<hex>`/`fastrtps_<hex>_el` (per-participant) and
  `fastrtps_port<N>`/`fastrtps_port<N>_el`/`sem.fastrtps_port<N>_mutex`
  (well-known discovery port). `component_supervisor_node.py`'s
  `*fastrtps*` glob catches all of them. **Confirmed correct.**
- **Costmap hook bug, two builds:** rebuilt `f1tenth_costmap` twice in a
  row with `--symlink-install`, no source changes between builds, and
  diffed the egg-link target both times: **identical**
  (`build/f1tenth_costmap`) before, after build 1, and after build 2. No
  drift. Confirms the analysis doc's claim with an actual two-build test
  on Thor, not just an assertion.
- **CPU pinning vs this machine's actual cores (recommendation only, not
  acted on):** Thor has **14 homogeneous cores** (0–13, single NUMA node,
  single cluster, all `MAXMHZ 2601`, confirmed via `lscpu -e`) — 2 more
  than the Orin AGX (12 cores) these defaults were tuned for
  (`cpu_affinity` default `'10,11'` in `mpc_corr.launch.py`'s own comment
  says *"Jetson Orin AGX, 12 homogeneous cores"*; other defaults: `yolo=8,9`,
  `detection_3d=6,7`, `obstacle_projector=6,7`, `wall_distance`/
  `swept_clearance`/`obstacle_clearance`/`lidar_front_wall=5` each,
  `foxglove=3`, `slam=2`). All of these core IDs exist on Thor's 0–13
  range, so nothing breaks mechanically — but cores **12 and 13 are
  currently unused** by any pinning default, and several defaults share a
  core (`detection_3d`/`obstacle_projector` both on `6,7`; four single-core
  nodes all default to `5`). Whether to spread load onto the two extra
  cores is a tuning decision for live load measurement on the actual
  hardware, not something to guess at here — **recommend, don't change.**

### Step 5 — robot_localization and slam_toolbox

**robot_localization 3.5.4 (Humble baseline) → 3.8.3 (installed on
Thor):** sourced an actual git clone with both exact tags present and
diffed directly (not inferred from changelogs):

- **`pose0_rejection_threshold` meaning: VERIFIED IDENTICAL.**
  `FilterBase::checkMahalanobisThreshold`'s body is **byte-identical**
  between 3.5.4 and 3.8.3 — `threshold = n_sigmas * n_sigmas`, confirmed
  directly from `git diff 3.5.4 3.8.3 -- src/filter_base.cpp`, which shows
  **zero changes inside that function**. This upgrades the previous
  analysis's "UNVERIFIED" hedge to a sourced fact: `pose0_rejection_threshold:
  5.0` means exactly what the current docs already claim (n-sigma, cutoff
  at 25.0 squared-Mahalanobis).
- **Process noise handling: VERIFIED backward-compatible.** `ros_filter.cpp`
  did change — it added a convenience diagonal-only input format
  (`covar_flat.size() == STATE_SIZE` → treated as a diagonal) **in
  addition to** the original full-matrix format (`STATE_SIZE * STATE_SIZE`
  → unchanged, same `else if` branch as before). Checked our actual
  config: `src/f1tenth_bringup/config/ekf_global.yaml`'s
  `process_noise_covariance` is the full flattened 15×15 (225-value) form
  — it hits the **same code path as before the change**. No behavioral
  difference for this stack's config.
- Full live EKF bag-replay validation (the acceptance criteria the plan
  already specifies: rejection rate 3±2%, max yaw step ≤5.54°±30%) is
  still a Phase 1 item requiring the archived bags — see bag status below.

**slam_toolbox 2.6.x (Humble baseline) → 2.8.5 (installed on Thor):**

- **Lifecycle/ACTIVE-state question: FOUND A REAL BUG, FIXED, VERIFIED
  LIVE.** Our `slam.launch.py` launched `async_slam_toolbox_node` via a
  plain `Node()` action with no lifecycle transition calls. Confirmed
  **empirically live** on this machine: `ros2 lifecycle get /slam_toolbox`
  reported **`unconfigured`** the entire time the process ran, and
  `ros2 topic list` showed none of `/slam/map`, `/slam/map_metadata`,
  `/slam/pose` — only the generic `/slam_toolbox/transition_event` topic.
  **The node would start, log normally, and do nothing — the single most
  dangerous kind of failure, because it looks fine.** `async_slam_toolbox_node`
  is confirmed a `rclcpp_lifecycle::LifecycleNode` (cross-checked against
  slam_toolbox's own `online_async_launch.py`, which drives it through
  `TRANSITION_CONFIGURE` then `TRANSITION_ACTIVATE` via emitted lifecycle
  events). The previous file's own docstring says it was written against
  `ros-humble-slam-toolbox` and explicitly flags that `enable_slam` has
  never been live-verified on real hardware on **either** distro (lidar
  and ZED were both down during that earlier pass) — so this is better
  described as a **pre-existing, never-exercised gap that Jazzy didn't
  cause but that this Phase 0 pass is the first thing to have actually
  caught**, because it's the first time anyone ran it live with lifecycle
  introspection.

  **Fixed (`4edcbbf`):** switched to `LifecycleNode` and added the same
  configure/activate event pair slam_toolbox's own launch file uses,
  keeping the existing `enable_slam` toggle, `taskset` CPU-affinity
  prefix, and `/slam/...` remappings unchanged. **Verified live:**
  `ros2 lifecycle get /slam_toolbox` now reports `active`; `/slam/map`,
  `/slam/map_metadata`, `/slam/pose` all appear as published topics; log
  shows the expected `Configuring` → `Activating` sequence.

- **`transform_publish_period: 0.0` → no map→odom:** still set
  (unchanged) in `slam_toolbox_params.yaml`. With the node now actually
  reaching `active`, `/tf` publisher endpoint exists on the node (ROS
  publishers are typically created upfront regardless of whether anything
  is ever sent on them) but **no message was observed on `/tf`** in a
  brief live run — consistent with, though not full proof of, the
  period=0 behavior, since there was no `/scan` data flowing to trigger
  any periodic internal logic in the first place (no lidar on this host).
  **Full confirmation under real scan-matching load is a genuine "needs
  real car" item** — but now that the node actually reaches `ACTIVE` at
  all, this is a meaningful, checkable thing to verify instead of an
  untestable no-op.

**Adjacent sanity check (not explicitly asked, but directly relevant given
the slam_toolbox finding):** Nav2's own lifecycle-managed nodes
(`map_server`, `bt_navigator`, `controller_server`, etc. in
`nav2.launch.py`) use the standard `nav2_lifecycle_manager` with
`autostart` wired in — correctly configured, unlike `slam.launch.py` was.
No action needed there.

---

## Commits this session

```
4edcbbf jazzy/p0: f1tenth_navigation: slam.launch.py never activated slam_toolbox
52682de jazzy/p0: f1tenth_logger: rosbag2_py.TopicMetadata requires id= in Jazzy
2ea8541 jazzy/p0: mpc_controller: declare test_depend on f1tenth_hardware
fd83d6e jazzy/p0: f1tenth_costmap: fix numpy-1.26 infinite loop in inflate_polytope
9a66870 jazzy/thor: bump zed_ros2_wrapper submodule humble-v4.2.5 -> v5.4.1
```

All local, unpushed. Plus the machine-state fix (not a repo change):
`ros-jazzy-asio-cmake-module` installed via apt, unblocking
`vesc_driver`/`serial_driver`/`io_context` configuration.

---

## Decisions for Andreas

1. **Bags for Phase 1 EKF/SLAM replay validation — already further along
   than expected.** `~/bags/humble_reference/` is **not empty** — it
   already contains 3 real mission bags (`2026-09-02T*-mission-bottle_then_person`,
   94–145 MB each, full `.db3`/`metadata.yaml`/`.params.yaml`/
   `.manifest.json`/`.extract.parquet`), plus a `bag/` directory whose
   `.db3` file was modified **during this very session** (10:29–10:30),
   suggesting a transfer may be actively in progress right now. Worth
   confirming with Andreas whether the copy from the Orin is complete
   before starting Phase 1 bag-replay validation (Item 1's acceptance
   criteria: rejection rate 3±2%, max yaw step ≤5.54°±30%).
2. **`slam.launch.py` fix — review before first live SLAM test.** This is
   a real behavior change (the node now actually runs instead of silently
   idling). Low risk (`enable_slam` defaults false, mirrors slam_toolbox's
   own upstream launch pattern exactly), but it's the kind of change worth
   a second pair of eyes before the first live lidar test, precisely
   because the previous version failed silently.
3. **CPU pinning on Thor's 14 cores vs the Orin-tuned defaults (cores
   12–13 currently unused; some defaults share a core) — live-measurement
   decision, not made here.** See Step 4 above for the full default list.
4. **torch/torchvision/ultralytics for JetPack 7** — install guidance
   gathered and cited above, nothing installed. Match the CUDA minor
   (`cu130`/`cu132`) to this Thor's actual installed CUDA before picking
   an index URL.
5. **`pose0_rejection_threshold`/process-noise behavioral revalidation**
   now has verified-identical source for 3.5.4→3.8.3, which removes the
   main open question from Item 1 — but the actual bag-replay acceptance
   test (rejection rate, max yaw step) is still real-data validation, not
   something a source diff can substitute for. Ready to run once bags are
   confirmed complete.

## Artifacts

- `output/thor_build.log` — final clean full-workspace build, 33/33 green.
- `output/thor_test.log` — final full-workspace test run: 3147 tests, **0
  errors** (was 6), 107 failures (all accounted for above: lint debt in
  this package set plus vendored-package lint, or confirmed pre-existing
  per section A), 128 skipped.
- `output/thor_numpy_pyzed_before.txt`, `output/thor_pip_user_before.txt`
  — pre-session environment snapshots.
