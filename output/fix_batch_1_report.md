# Fix batch 1: non-migration fixes (Thor, branch `jazzy`)

These three fixes are approved but sit outside the Humble→Jazzy parity
work. Each has its own `fix/<area>:` commit. Nothing has been pushed.

## Verdict: **GO-WITH-NOTES**

- **1. `system_observer_node`:** fixed and proven. Over a 330 s soak with
  the GPU idle, every one of 329 GPU-zone probes hit EAGAIN, and the node
  stayed up the whole time and published 330 messages.
- **2. Preflight:** fixed. A `front_clearance` condition now requires
  `front_clearance_node`. The BT replay matches Phase 4.
- **3. Logger topics:** 7 of the 8 topics were added. All 7 are proven to
  land in the recording (mcap and sqlite3). **`/test_campaign/logger_status`
  was held back:** adding it breaks a deliberate isolation invariant
  (Decision A).
- Two findings for Phase 6:
  - **`/imu` has no publisher anywhere in this workspace** (Decision B).
  - `/joint_states` and `/sensors/imu/raw` are marked "verify on the car in
    Phase 6".

---

## 1. `system_observer_node`: a `None` sysfs read no longer kills the node

**Cause** (Phase 4, Decision 3):
- On Thor, `thermal_zone1/temp` (`gpu-thermal`) returns EAGAIN while the
  GPU is idle.
- The raw read returns `None`. Checked: `open(...,'rb',buffering=0).read()`
  gives `None`.
- The text-mode `f.read()` then raises `TypeError: can't concat NoneType to bytes`.
- `_read_thermal_zone_cpu_temp()` caught only `(OSError, ValueError)`.

**Fix** (`931a0eb`): `TypeError` was added to that `except`, with a one-line
comment. The zone is skipped for that tick, just like an `OSError`, and the
loop moves on to the next zone. On Thor that is `cpu-thermal` (zone 2).
An empty read was already handled: `int('')` raises `ValueError`. The
jtop path is unchanged.

**How the GPU temperature is represented:** without jtop, the fallback
never reads the GPU temperature, and never did. `gpu_temp_c`,
`gpu_percent` and `emc_percent` stay at the message default **0.0**, as
the node's docstring says. `SystemStatus` has no separate "unavailable"
flag, and I didn't add one (that would be a message change).
- The GPU zone's EAGAIN affected only the CPU-temperature search, which
  walks the zones in order.
- If the CPU zone itself became unreadable, `cpu_temp_c` falls back to the
  first readable zone (`tj-thermal`), and to 0.0 if no zone is readable.
  Unit tests cover both.

**`IsSystemOverheated` with that representation:** 0.0 never exceeds the
positive threshold, so it fails safe. Its docstring already states this
for the jtop-less case. The soak confirmed it with production values from
`stack_params.yaml`: `sys_obs_max_temp_c` **100.0**,
`enable_sys_obs_load_trip` false. (Correction to the Phase 4 report: it
quoted 85 °C, which is the class default, not the value in effect.)

**Soak** (`scripts/fix_batch_1/system_observer_soak.sh`):
- **Setup:** the real node via `ros2 run` on isolated domain 81, with no
  jtop. A monitor runs a real `IsSystemOverheated` at 10 Hz and probes
  the GPU zone once a second.
- **Results:**

| | |
|---|---|
| duration | 330 s |
| `/diagnostics/system_status` messages | 330 (max gap 1.004 s) |
| GPU-zone probes that returned EAGAIN | **329 / 329** |
| `cpu_temp_c` | 33.4–37.1 °C (from `cpu-thermal`) |
| `gpu_temp_c` / `gpu_percent` / `emc_percent` | 0.0 throughout |
| `IsSystemOverheated` ticks | 3,300, all FAILURE (no trip), 0 exceptions |
| node at end | alive; SIGINT → rc 0; 0 tracebacks |

Data: `output/fix_batch_1/system_observer_soak/`.

**Tests:** `test_system_observer_thermal_read.py` (6 tests).
- They feed the node a real `TextIOWrapper` over a raw stream that
  returns `None`, so they exercise the same CPython path as the real
  failure, not a mocked exception.
- Covered: the GPU zone is skipped and the CPU zone is used; the CPU zone
  falls back to the first zone; all zones unreadable gives 0.0; an empty
  read is skipped; `_publish_tick()` publishes with `gpu_temp_c == 0.0`.
- **6 pass with the fix, 4 fail without it.**
- `f1tenth_diagnostics`: 156 passed, plus the 3 pre-existing lint
  failures (copyright, flake8, pep257).
- The new test file is clean under flake8 and pep257. The node file's
  flake8 count is unchanged (5 before, 5 after).

## 2. Preflight: `front_clearance` requires `front_clearance_node`

**Cause:**
- `f960cf2` (2026-08-21) added the preflight. For a `front_clearance`
  stop or resume condition it required a node named
  `costmap_boundary_node`, because the condition then read
  `/costmap/front_clearance`.
- `63a6080` (2026-09-14) switched the condition to
  `/perception/front_distance`, published by
  `f1tenth_perception/front_clearance_node.py:508` (node name
  `front_clearance_node`, `:385`). The preflight wasn't updated.

**Fix** (`0397d6b`):
- `Requirement(name/node_name='front_clearance_node')`.
- `blackboard_key=FRONT_CLEARANCE_KEY` is unchanged: that is the liveness
  half.
- The docstring list in `loader.py` now names the right node.

**Tests** (`test_preflight.py`):
- Existing assertions were updated to the new node name.
- Two new cases:
  - only `costmap_boundary_node` up: **refused**, with "front_clearance_node
    … not found in the ROS graph";
  - `front_clearance_node` up and publishing: passes.
- **13 pass with the fix, 5 fail without it.** `f1tenth_behavior`: **345
  passed** (343 + 2).

**Harness:**
- The BT layer of `replay_localization.sh` now also starts a name-only
  `front_clearance_node` stub. The `costmap_boundary_node` stub stays, so
  the same harness still serves the Orin's `e47e646` reference run
  (`ORIN_BT_INSTRUCTIONS.md` doesn't change).
- A Jazzy BT replay with this commit, compared with Phase 4's `bt_jazzy_1`
  (`output/fix_batch_1/bt_metrics_fix2.json`):
  - load and start succeed;
  - the same outcome, `/mission/status` sequence, post-stop tree sequence
    and `/mpc/goal_drive`;
  - hold Δ = 1.0 ms, inside Phase 4's 2 ms floor.

## 3. Logger default topics

**Added** (`7155f79`): the topics below. The publisher and QoS for each come
from the code. Every one is depth 10 with the default RELIABLE + VOLATILE
QoS, so **none** belongs in `_DEFAULT_BEST_EFFORT_TOPICS`.

| Topic | Publisher (source) | QoS | Note |
|---|---|---|---|
| `/mpc/goal_drive` | `PublishMoveGoal`, `publish_move_goal.py:108` | RELIABLE/VOLATILE, 10 | |
| `/imu` | **none in the workspace**; only `MPC_corr.py:1718` subscribes (depth 10) | n/a | **verify on the car in Phase 6** (Decision B) |
| `/joint_states` | `joint_state_publisher` (apt), `joint_state_publisher.py:425`; launched by `description.launch.py`, not in sim | RELIABLE/VOLATILE, 10 | **verify on the car in Phase 6** |
| `/perception/front_distance` | `front_clearance_node.py:508` | RELIABLE/VOLATILE, 10 | |
| `/perception/d_wall/psi_correction` | `wall_distance_node.py:268` (`output_topic` default `/perception/d_wall`) | RELIABLE/VOLATILE, 10 | |
| `/mpc/status` | `MPC_corr.py:1805` | RELIABLE/VOLATILE, 10 | |
| `/sensors/imu/raw` | `vesc_driver.cpp:123`, `rclcpp::QoS{10}`; in sim, `ros_gz_bridge` (no QoS override) | RELIABLE/VOLATILE, 10 | **verify on the car in Phase 6** |

**Not added:** `/test_campaign/logger_status`. `8e0d48c` pins, in
`test_test_campaign_isolation.py`, that "nothing the campaign side
publishes is read or recorded by the mission logger". With the topic in
the list, that test fails. It's a deliberate design invariant, so I left
it intact (Decision A).

**End to end** (`scripts/jazzy_parity/logger_check.sh`, check 7):
- While the bag plays, `publish_logger_topics.py` publishes on every new
  topic with its production QoS. Each topic must then appear in the
  recording's `metadata.yaml` with a non-zero count.
- None of these topics is in the source bag, so what was recorded came
  from the test publisher.

| | mcap | sqlite3 | mcap, **without** the change |
|---|---|---|---|
| 7 new topics recorded | PASS, 119 msgs each | PASS, 117–118 each | **FAIL, 0 each** |
| offered QoS in the bag | reliable/volatile | reliable/volatile | n/a |
| all other logger checks | PASS | PASS | PASS |

About 120 messages were published per topic. The 1–3 missing at the start
are pre-discovery messages on a VOLATILE topic.

Output: `output/fix_batch_1/logger_check_{mcap,sqlite3,mcap_without_fix}.txt`.

`f1tenth_logger`: 292 passed, plus the 3 pre-existing lint failures.

---

## Commits

```
931a0eb fix/diagnostics: system_observer skips a thermal zone whose sysfs read returns None — Thor's idle GPU zone crashed the node on its first tick
0397d6b fix/behavior: preflight requires front_clearance_node for a front_clearance condition — it checked costmap_boundary_node, the producer before 63a6080
7155f79 fix/logger: record the drive goal, the front_clearance stop input, IMU, joint and MPC status topics — Phases 3-4 found each of them missing from every bag
e4d6110 fix/batch1: report and evidence for the three non-migration fixes — soak, BT replay and logger-check data behind each commit
```

All local and unpushed.

## Decisions for Andreas

A. **`/test_campaign/logger_status` in the mission logger?** Adding it
   contradicts `8e0d48c`'s isolation invariant. If you still want it
   recorded, the invariant test has to be relaxed in the same commit
   ("not read" stays, "not recorded" goes). Otherwise, leave it out as now.

B. **`/imu` has no publisher.**
   - `MPC_corr.py:1718` subscribes to `/imu` and uses
     `linear_acceleration.x` (`imu_callback`, with a debug line "if this
     line never appears, /imu isn't publishing").
   - The ZED's IMU publish is disabled, and `zed2_perception.yaml` claims
     `/imu` "is that same VESC topic". But the VESC publishes
     `sensors/imu/raw`, and nothing remaps it.
   - `ax_imu` only feeds the `a_imu` column of MPC_corr's control-log CSV
     (`MPC_corr.py:4072`), not the controller. So the effect is diagnostic
     only: that column would be NaN.
   - It's recorded now, so Phase 6 will show whether anything publishes it
     on the car. That's pre-existing and not changed here.
