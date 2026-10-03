# Fix batch 3: graph reads under plain Discovery Server clients (Thor, branch `jazzy`)

Decision H6: plain clients stay the Jazzy default. This batch audits every
call that reads the ROS graph, tests the affected ones live in the full
stack, and fixes only the broken ones.

- One `fix/<area>:` commit per node or tool. Nothing pushed.
- Data in `output/fix_batch_3/`. Live-check script:
  `scripts/jazzy_parity/graph_calls_live.py`.

## Verdict: **GO-WITH-NOTES**

- **Fixed:** `ekf_cost_observer_node` and `stackctl.py`.
- **Partial:** the steering calibration's e-stop preflight now *can* see
  `/safety_stop`, but its own preflight timing means it still refuses in
  3 of 4 runs. Code change needed; Decision 1.
- **Side finding:** the silent discovery isolation (backlog H1) happened
  again, to 2 nodes in one run.

---

## 1. Audit (from the code)

**Scope:** `src/` excluding tests and vendored packages, plus `scripts/`.

**C++:** no graph read in the C++ we own (`vesc`, `ackermann_mux`,
`teleop_tools` grepped).

**Subprocesses:** the supervisor's `ros2 launch`, the logger's `systemctl`
and `git`, `robot_logger`'s `git`, `kill_ros2.py`'s `ros2 daemon stop`, and
`ensure_discovery_server.py`'s `tail` are all non-graph. No `ros2` CLI
graph query runs from stack code.

**What a plain client gets right (Phase 5 Step 3, re-measured here with
`graph_calls_live.py`, plain client, full stack):**

| Graph read | Plain client |
|---|---|
| `get_node_names()` | **right**: 31 of 31 |
| `get_topic_names_and_types()` | **wrong**: 5 topics, `/odometry/filtered` absent |
| `count_publishers(topic)`, no own endpoint on the topic | **wrong**: 0 for `/odometry/filtered` and `/safety_stop` |
| `count_subscribers(topic)` with an own publisher on it | **right**: 1 on `/calibration_drive` |
| `publisher.get_subscription_count()` | **right**: 1 |
| `get_service_names_and_types()` | **wrong**: 8 services, `/restart_component` absent |
| `client.wait_for_service()` | **right**: true |

**The audit:**

| Call | file:line | What it decides | Plain client? | Class |
|---|---|---|---|---|
| `get_node_names()` | `f1tenth_behavior/mission/loader.py:394` (→ `preflight.py:232`) | Mission start refuses if required nodes are absent | right (Phase 5 missions started with plain clients) | OK |
| `get_node_names()` | `f1tenth_diagnostics/diagnostics_server_node.py:158` | `/calibration/in_progress` | right | OK |
| `get_topic_names_and_types()` | `f1tenth_diagnostics/ekf_cost_observer_node.py:221` | Whether to subscribe to the EKF input/output topics at all | **wrong**: never subscribes | **BROKEN → fixed** |
| `get_node_names()` | `steering_offset_calibration_node.py:602` | Preflight: other participants visible | right (32 visible) | OK |
| `count_subscribers(/calibration_drive)` | `steering_offset_calibration_node.py:668` | Preflight: mux lane live | right (own publisher) | OK as plain; see 2b |
| `count_publishers(/safety_stop)` | `steering_offset_calibration_node.py:678` | Preflight: BT safety lane live, refuse otherwise | **wrong**: 0 | **BROKEN → partially fixed** |
| `pub.get_subscription_count()` | `mpc_controller/MPC_corr.py:3432` | Log line only ("PUB /drive … subs=1") | right | OK (cosmetic anyway) |
| `pub.get_subscription_count()` | `f1tenth_logger/test_campaign/trigger.py:59-60` | Wait for the campaign logger before publishing | right (own publishers) | OK (classified by the API probe; hand tool, not run) |
| `wait_for_service()` | `llm/llm_planner_node.py:1020, 1044` | `/mission/*` services up | right | OK |
| rosbag2 `Recorder` | `f1tenth_logger/mission_logger_node.py` | Which topics to record | was **wrong** | fixed in fix batch 2 (`84a12f1`) |
| foxglove_bridge | `foxglove_bridge.launch.py` | Channel list | already a super client | OK |
| `get_node_names_and_namespaces()` | `scripts/stackctl.py:71` | `status`: preflight nodes present | right | OK |
| `get_service_names_and_types()` | `scripts/stackctl.py:72` | `status`: supervisor services present | **wrong**: "MISSING" | **BROKEN → fixed** |
| `wait_for_service()` | `scripts/stackctl.py:54`, `scripts/check_floor_values.py:68` | Service calls | right | OK |

`battery_voltage_check_node` deliberately does not read the graph (its
docstring).

No call was "affected but only cosmetic": the one cosmetic call
(`MPC_corr`'s log line) gets the right answer anyway.

---

## 2. Live tests (Phase 5 harness, full stack, plain clients)

- **Setup:** `phase5_bringup.sh hold`, started from a fresh shell with the
  fix batch 2 env script (no `ROS_SUPER_CLIENT`), `ROS_DOMAIN_ID=85`.
- **Before:** `output/fix_batch_3/hold_before/`, `live_before.json`.
- **After the fixes:** `hold_after/`, `live_after_plain_shell.json`.

### 2a. `ekf_cost_observer_node`: does it measure anything?

Values from its own `/diagnostics` status, local / global EKF:

| | Before (plain client) | After (`a936c75`) | Phase 4, recorded live on the Orin |
|---|---|---|---|
| status | OK (!) | OK | – |
| `ticks_selfcount` | **0 / 0** | 49 / 49 | 48 / 47 |
| `meas_delivered` | **0 / 0** | 49 / 51 | 100* / 49 |
| `period_ms_p50` | **0 / 0** | 20.2 / 20.2 ms | 20.01 / 20.01 ms |
| `ticks_inproc` (from `/diagnostics`, its own topic) | 754 / 504 | non-zero | 504 / 503 |

- **Broken, silently:** as a plain client it never subscribed to the EKF
  topics, so half its metrics were 0 while the status said OK.
- **After:** it measures again, at Phase 4's values. \*Local
  `meas_delivered` on the Orin counts the IMU, which the bag lacks
  (Phase 4).
- **Environment check (`hold_after/environ_settled.txt`):**
  `ekf_cost_observer_node` has `ROS_SUPER_CLIENT=TRUE`. Both `ekf_node`
  processes from the same launch file do not.
- Super clients in the stack are now 5 of 45 processes: foxglove and its
  2 throttles, the logger, and ekf_cost_observer.

### 2b. Steering calibration: does it refuse?

Tested without commanding anything:
- **Plain client, before:** the real node class, constructed only (no
  preflight run), with its checks called directly; and three times via
  `ros2 run`.
- **Via the launch file, after:** run with `amplitude_rad:=-0.1`. Every
  stage-1 check is evaluated and logged, and the invalid amplitude
  guarantees a refusal before any nudge. Each run ended with "preflight
  refused -- nothing was commanded."

| Run | `/calibration_drive` subscribers | `/safety_stop` publishers | `_check_estop_path()` |
|---|---|---|---|
| plain, class call (10 s settle) | 1 | **0** (the BT is running) | **refuse** |
| plain, `ros2 run` ×3 | 1, 1, 1 | **0, 0, 0** | **refuse ×3** |
| launch file (super client) ×4 | 0, 0, 0, 1 | –, –, –, **1** | refuse, refuse, refuse, **pass** |

- **As a plain client it always refused:** "NO publisher on
  /safety_stop", with the BT running.
- **With the launch-scoped super client** it found both the mux lane and
  the `/safety_stop` publisher once. The other three times it refused
  earlier, on "nothing is subscribed to /calibration_drive".
- **Cause:** the node's preflight window ends as soon as map, mission
  status and clearance have arrived (2.15 s after start in the logged
  run). A new super client needs several seconds to be told the whole
  graph: with `ros2cli`, 1–2 s returned the full graph only some of the
  time, 3–5 s always (fix batch 2).
- The env setting is necessary but not sufficient (Decision 1). Every
  failure is a refusal before anything is commanded, never a false pass.

### 2c. `stackctl.py status`

| | Services seen | Supervisor services |
|---|---|---|
| plain, before | 7 | MISSING ×3 |
| super client (`ROS_SUPER_CLIENT=TRUE` by hand) | 288 | OK ×3 |
| after `d29b3e0`, plain shell | 288 | OK ×3 |

- The variable is set inside the script before `rclpy.init()`, so the shell
  and any stack started from it stay plain.
- `stackctl.py` is a hand tool, not a launched node, so the
  launch-scoped pattern does not apply to it.

### Side finding: discovery isolation again (backlog H1)

In the *before* run, two nodes stayed alive for the whole 3.5 minutes but
received nothing:
- `mpc_corr`: 2,189 "ODOM non disponibile" lines; a normal run logs about
  200, only until the bag starts.
- `costmap_boundary_node`: map and pose "no message ever received".

Both were in the graph at startup (the startup probe saw them, last at
22.8 s). Neither was in it when a fresh participant joined later. No error
was logged and the supervisor saw nothing.

This is not caused by plain vs super clients: the stack's environment was
identical in Phase 5, where it happened once in about 320 node starts, and
here 2 of 32 nodes in one run. The *after* run was normal. Backlog H1 is
updated with this evidence.

---

## Commits

```
a936c75 fix/diagnostics: ekf_cost_observer_node runs as a Discovery Server super client — as a plain client it subscribed to nothing and half its metrics read 0
5b320a5 fix/diagnostics: steering_offset_calibration.launch.py sets ROS_SUPER_CLIENT — as a plain client its e-stop preflight never sees the /safety_stop publisher and always refuses
d29b3e0 fix/tools: stackctl.py runs as a super client — as a plain Discovery Server client `status` reported every supervisor service MISSING on a running stack
ee9b0a0 fix/batch3: report, live-check script and data, backlog update — plain-client graph audit
```

**How each fix is scoped:**
- **`ekf_cost_observer` uses `additional_env`** on its `Node`, not a
  launch-wide `SetEnvironmentVariable`. `localization.launch.py` also
  starts both EKFs, which must stay plain clients. Same effect, scoped to
  the one process.
- **The steering calibration launch file** starts only that node, so it
  uses `SetEnvironmentVariable`, like foxglove and the logger.
- **flake8 counts** of both launch files are unchanged.

## Decisions for Andreas

1. **Steering calibration preflight (backlog H6, still open).**
   - With the super client, the e-stop check has to wait for the graph to
     converge.
   - **Proposal:** in `_check_estop_path()`, poll `count_subscribers(drive)`
     and `count_publishers(/safety_stop)` while spinning, for up to about
     5 s, before refusing.
   - It only delays a refusal; it never passes anything that would fail
     today once the graph is complete.
   - It is a code change in a safety preflight, so it awaits your OK.
     Until then the tool usually refuses on Jazzy, and in every case
     before commanding anything.
2. **Discovery isolation (backlog H1)** now has three affected nodes in two
   of the roughly 20 full-stack runs so far. It should move ahead of the first Jazzy test drive.
