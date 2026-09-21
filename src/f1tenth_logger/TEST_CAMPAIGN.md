# Test campaign logging for LLM-driven runs

One folder per test, one row per test, one campaign per folder. The car drives,
this records; afterwards **you** decide what counted as a success.

That last part is the rule everything else is built around: `finish()` records
only an automatic *hint* (`auto_outcome` / `auto_success`). The verdict that
counts is the `success` column you fill in by hand in `campaign_results.csv`.
The export never overwrites it, and the analysis counts nothing else.

The code lives in `f1tenth_logger/test_campaign/`, a subpackage of this
package. It sits next to the mission logger and shares nothing with it (see
[Next to the mission logger](#next-to-the-mission-logger)). Nothing starts it
automatically.

---

## The pieces

| executable / module | what it is |
|---|---|
| `test_campaign_logger` (`logger_node.py`) | **The recorder.** Node `test_campaign_logger`, started by hand with `test_campaign_logger.launch.py` and left running all session. It opens and closes tests by itself. |
| `test_campaign_trigger` (`trigger.py`) | Manual trigger: fakes a plan result and drives the mission lifecycle by hand. Use it for calibration runs; no LLM is involved. |
| `test_campaign_export` (`export_campaign_csv.py`) | Turns the raw logs into `campaign_results.csv`, one reviewable row per test. The manual columns are left alone. |
| `test_campaign_analyze` (`analyze_tests.py`) | Report and plots from the manual verdicts: k/N, Wilson CI, maps, dashboard. |
| `robot_logger.py` | `TestLogger`: one test folder. Standard library + PyYAML, thread-safe, no ROS. |
| `demo_simulated.py` | Fake car, fake LLM, fake MPC. Proves the whole chain without hardware: `python3 -m f1tenth_logger.test_campaign.demo_simulated` |
| `config/test_campaign_logger.yaml` | The recorder's parameters. |
| `launch/test_campaign_logger.launch.py` | The only way the recorder is meant to start. |

### Where the data comes from

| topic | type | published by |
|---|---|---|
| `/test/plan_result` | String JSON | `llm_planner_node`, after every LLM call (or `test_campaign_trigger`) |
| `/test/mission_event` | String JSON | `behavior_executor_node` (mission/loader.py) on every lifecycle edge, or `test_campaign_trigger` |
| `/odometry/filtered` | nav_msgs/Odometry | the local EKF, ~50 Hz. It is the recorder's default `odom_topic`, because mpc_corr builds its corridors from this estimate. Raw `/odom` (vesc_to_odom) also works, but its yaw drifts. |
| `/sensors/imu/raw` | sensor_msgs/Imu | vesc_driver, 50 Hz |
| `/drive` | AckermannDriveStamped | mpc_corr, 10 Hz: what the MPC asked for |
| `/mpc/status` | String JSON | mpc_corr, after **every** solve: `{status, solve_time_ms, cost, iterations, ...}` |
| `/corridor` | String JSON | mpc_corr, on every corridor rebuild: `{id, polygon, frame_id: odom, odom_topic, ...}` |
| `/safety/event` | String JSON | `behavior_executor_node`: `{event: estop, cause}` once per emergency stop. `obstacle_clearance_node`: `{event: contact, cause}` |
| `/obstacle_clearance` | Float32 | `obstacle_clearance_node` (f1tenth_perception), once per `/scan`. **Optional**; start it for a campaign session (step 1). |

Two files outside this package are part of the chain:
`src/f1tenth_intelligence/llm/llm/llm_planner_node.py` publishes the plan
result, and `src/f1tenth_behavior/f1tenth_behavior/mission/loader.py` publishes
the mission events and runs the start countdown.

---

## A session, command by command

Four terminals. Everything below assumes `source install/setup.bash` first.

### 1. The car stack

```bash
ros2 launch f1tenth_bringup supervisor_bringup.launch.py
./scripts/stackctl.py status                          # what is actually up (the ros2 CLI lies under the discovery server)
./scripts/stackctl.py start obstacle_clearance        # the obstacle_clearance column and contact events
```

`obstacle_clearance` is not auto-started. Without it a test has no
`min_clear_m` and no contact events, and the recorder says so: an ERROR line
and a `no_obstacle_clearance` event when nothing arrives within
`clearance_grace_s` (2 s) of `mission_started`.

### 2. The recorder: start it once and leave it running

```bash
ros2 launch f1tenth_logger test_campaign_logger.launch.py
```

Parameters come from `config/test_campaign_logger.yaml`: campaign
`first_test_campaing`, robot radius, pre/post roll, timeout, robot and model
names, and the topics. Override any of them with `config:=<your.yaml>`.

A bare `ros2 run f1tenth_logger test_campaign_logger` loads the same yaml by
itself when no `--params-file` is given (`-p` still overrides it), and says
so at startup. Before that, `ros2 run` silently ran on the code defaults --
poses from raw `/odom`, robot radius 0, no pre-roll -- which is how the whole
2026-09-21 session was recorded.

At startup it prints:
- the absolute campaign folder, which is always
  `<f1tenth_more>/first_test_campaing`;
- every subscription with its QoS.

While a test is open, it logs one status line every 10 s: test id, seconds
recorded, rows per stream. It publishes the same line as JSON on
`/test_campaign/logger_status`, plus a final one when the test closes. It
records test after test without restarting. **Ctrl+C closes any open test
cleanly** as `aborted / "logger node shut down"`, so nothing is lost.

The root is resolved by the launch file:
1. `root:=<path>` if given;
2. else `$F1TENTH_MORE_ROOT`;
3. else the workspace that holds `src/f1tenth_logger`.

The same answer comes out whether the package was built with `colcon build`,
with `--symlink-install`, or is run from source. With no root, for example
from an install outside the workspace, the node refuses to start and says so.

The prompt table is `<f1tenth_more>/first_test_campaing/prompts.yaml` and is
committed. Each entry looks like this:

```yaml
# expected_sequence: [straight distance 3.0 m, stop]
- prompt_num: 1
  mission: M01_forward_3m
  text: "go ahead 3 meters"
  success_criterion: "stops at 3.0 m +- 0.15 m from start, lateral error < 0.2 m, no contact"
```

The mission folder name comes from this table and nowhere else. A prompt that
is not in it is refused rather than guessed. `text` must be exactly what the
planner receives; see [Gotchas](#gotchas).

### 3. Calibration runs (no LLM)

```bash
ros2 run f1tenth_logger test_campaign_trigger 0 --countdown 5
```

The trigger publishes the plan result and `mission_loaded`, waits out the
countdown, publishes `mission_started`, and then waits. Do the manoeuvre, then
press **Enter** to finish or **Ctrl+C** to abort. See the
[checklist](#calibration-checklist).

### 4. Real missions

```bash
ros2 run llm llm_planner_node "go ahead 3 meters" --prompt-num 1
```

`--prompt-num` ties the run to a row in `prompts.yaml`. Without it the logger
falls back to matching the exact command text. That works, but it gives no
warning if the text drifts. Add `--kind replan` for a second call against a
test that is already open.

The planner publishes `/test/plan_result`, the loader publishes
`/test/mission_event`, and the recorder writes the folder. The planner sends
`/test/plan_result` **before** it aborts the previous mission and loads the
new one, so the test is already open when `mission_loaded` arrives. If load
or start then fails, a follow-up `/test/plan_result` with `kind: delivery`
records `delivery_failed` in the test and closes it. The abort's own
`mission_aborted` (the previous mission's end) lands in the new test before
its `mission_loaded`; it is recorded as `stale_mission_event` and ignored. The mission waits
`mission_countdown_sec` (default 3 s) at a standstill before it drives. That
standstill is the noise floor every jerk number is measured against.

The countdown is a parameter of the behaviour node. It is read fresh at every
start, so you can retune it between tests without restarting anything:

```bash
ros2 param get /behavior_executor_node mission_countdown_sec     # 3.0 by default
ros2 param set /behavior_executor_node mission_countdown_sec 5.0
```

It is not in `stack_params.yaml`, so a restart goes back to 3.0. Set it once
per session, or add it there if you want it permanent.

### 5. Export, judge, analyse

The metrics exist only in `campaign_results.csv`, and only after the export
has run. `results.csv`, which the recorder writes, is a per-test summary, not
the table you judge.

```bash
ros2 run f1tenth_logger test_campaign_export                   # Excel-EU: ';' and decimal comma
ros2 run f1tenth_logger test_campaign_export --plain           # ',' and decimal point
```

Open `first_test_campaing/campaign_results.csv` in Excel. Fill in **`success`**
(1/0), and optionally `transl_ok` and `notes`. Save and close.

You can re-run the export whenever you like:
- it keeps every hand-entered value;
- it adds rows for new tests;
- it never deletes a row, not even for a test folder you deleted.

If Excel still has the file open, it says so and writes
`campaign_results_NEW.csv` instead of losing the run.

**Backfill.** `python3 tools/test_campaign_backfill.py` writes
`backfill.json` for tests whose inputs never reached their folder (the
2026-09-21 session: see the tool's docstring). The export fills a column from
it only where the test's own files leave it empty, and lists each such column
in `backfilled`, with a tag like `partial 39%`. `--no-backfill` ignores it.
The test folders are never modified.

```bash
ros2 run f1tenth_logger test_campaign_analyze --footprints 2.0
```

This prints the report and writes `first_test_campaing/analysis/`:
- `summary.csv`
- `report.txt`
- one `overview_map_<MISSION>.png` per mission
- a combined `overview_map.png`
- `dashboard.png`

Tests with an empty `success` are excluded from k/N and listed as *not yet
evaluated*.

---

## Next to the mission logger

`mission_logger_node`, in this same package, starts with the stack (the
`diagnostics` component). It records one rosbag per mission, triggered by
`/mission/status`, under `mission_logger_runs_dir` (`~/f1tenth_archive`).
Running both loggers at once is the normal case, and they cannot collide:

| | mission logger | test-campaign logger |
|---|---|---|
| node | `mission_logger_node` | `test_campaign_logger` |
| started by | the supervisor, with the stack | you, by hand, only |
| reads | `/mission/status`, and records 37 topics into the bag | the topics in the table above |
| publishes | nothing | `/test_campaign/logger_status` only |
| writes | `~/f1tenth_archive/{active,complete,incomplete}/`, `runs.db`, lock `/tmp/mission_logger.lock` | `<f1tenth_more>/first_test_campaing/` only |
| code | `f1tenth_logger/*.py` | `f1tenth_logger/test_campaign/*.py` |

`test/test_campaign/test_test_campaign_isolation.py` checks each row.

---

## Folder structure

```
<f1tenth_more>/first_test_campaing/
├── campaign.json              created, robot, llm model, git commit
├── prompts.yaml               the prompt table -- the only source of mission names
├── results.csv                one row per finished test, written by the logger
├── campaign_results.csv       one row per test, written by the export, judged by you
├── export_settings.json       the filter settings the last export used
├── backfill.json              values NOT recorded live (tools/test_campaign_backfill.py)
├── analysis/                  report.txt, summary.csv, the plots
└── M01_red_box/
    ├── mission.json           mission name, prompt text, success criterion
    └── P001-R003-20260921T143512/
        ├── kinematics.csv     t, x, y, yaw, yaw_rate, vx, vy, speed, ax, ay, acc,
        │                      corridor_clearance, obstacle_clearance
        ├── imu.csv            t, sensor_stamp, ax, ay, az, gx, gy, gz, imu_yaw
        ├── commands.csv       t, cmd_speed, cmd_steer, cmd_yaw_rate, cmd_throttle,
        │                      cmd_brake, source
        ├── mpc.csv            t, status, solve_time_ms, cost, iterations
        ├── llm_calls.csv      call_idx, tag, t_sent, t_received, latency_ms, ttft_ms,
        │                      prompt_chars, response_chars, ok, error
        ├── llm_calls.jsonl    the full prompt of each call, llm_raw (what the model
        │                      wrote), rejections, translated_plan (the translator's
        │                      output), plan_file, plan_hash; `response` is a
        │                      deprecated copy of translated_plan
        ├── plan.json          the initial plan as the planner delivered it
        ├── plan_replan_N.json the N-th replan
        ├── corridors.jsonl    every corridor: t, id, source, polygon, meta
        ├── events.jsonl       mission lifecycle, contact, estop, replan, ...
        └── meta.json          everything above, summarised
```

`P001-R003-20260921T143512` = prompt 1, repetition 3, started 2026-09-21
14:35:12. Repetitions count up on their own and a folder is never reused or
deleted, including for failed runs.

**Conventions.** Body frame x forward, y left, z up. IMU acceleration in m/s²
**including gravity**. Gyro in rad/s. Yaw in rad, world frame,
counter-clockwise positive. `cmd_steer` in rad. Every `t` is seconds since the
test folder was created — so the prompt that started the test has a *negative*
`t_sent`, and pre-roll samples have negative `t` too. Clearances are measured
from the robot's **edge**: negative means the footprint is over the line.

---

## The metric window

Recording starts at `mission_loaded` (earlier with pre-roll) and ends
`post_roll_s` after the mission does. But the car is only driving between
`mission_started` and the end, so that is the only part that gets scored.

```
 pre-roll │ mission_loaded ── countdown ── mission_started ─── driving ─── mission_finished │ post-roll
          │◄──── standstill_jerk_rms ────►│◄───── every driving metric ─────►│
```

A test with no `mission_started` — an LLM failure, a cancelled start, a
timeout — is still exported, with the driving columns empty. That is not a
gap in the data; it is the finding.

## Every column in `campaign_results.csv`

| column | window | meaning |
|---|---|---|
| `mission` | — | mission folder name, from `prompts.yaml` |
| `test_id` | — | `P<prompt>-R<repetition>-<timestamp>` |
| `prompt_num`, `repetition` | — | decoded from `test_id` |
| `date`, `time` | — | when the test started |
| `plan_id` | — | the planner's mission_id, echoed by every mission event |
| `plan_hash` | — | sha256 of `plan.json` as canonical JSON. Equal hashes, same plan. A copy that differs from the loader's `llm_generated/<plan_id>.json` is a `plan_file_mismatch` event |
| `llm_latency_ms` | the initial call | what the operator waited for the first plan |
| `n_replans` | whole run | LLM calls tagged `replan` |
| `countdown_s` | — | standstill the mission node held before driving |
| `drive_duration_s` | driving | `mission_started` → end |
| `standstill_jerk_rms` | **countdown** | this test's own noise floor, m/s³ |
| **`success`** | — | **MANUAL. 1/0. The only verdict that counts.** |
| `auto_outcome` | — | `completed`/`aborted` from the logger. A hint, nothing more. |
| `estop` | whole run | 1 if an `estop` event was recorded |
| `contact` | whole run | 1 if a `contact` event was recorded |
| `viol_rate_pct` | driving | % of **time** with `corridor_clearance < 0` |
| `min_clear_m` | driving | smallest obstacle clearance, forced to 0 if estop or contact |
| `min_clear_raw_m` | driving | the same minimum, never clamped, can be negative |
| `feas_pct` | driving | MPC solves with an ok status / all solves × 100 |
| `max_infeas_streak_s` | driving | longest unbroken run of not-ok solves |
| `mpc_solve_time_p95_ms` | driving | p95 solve time |
| `jerk_rms` | driving | RMS horizontal jerk, m/s³, low-passed at `--cutoff-hz` |
| `steer_rev_per_m` | driving | filtered steering reversals clearing `--deadband-rad`, per metre |
| `backfilled` | — | columns filled from `backfill.json`, not recorded live, `\|`-separated |
| **`transl_ok`** | — | **MANUAL.** Did the translator produce what the prompt meant? |
| **`notes`** | — | **MANUAL.** Free text. Semicolons and accents are safe. |

`jerk_rms` and `steer_rev_per_m` depend on the filter: `--cutoff-hz` (default
5) and `--deadband-rad` (default 0.02). Whatever you used is recorded in
`export_settings.json`. Change it and the numbers change — re-export
everything, or the campaign is not comparable with itself.

---

## Calibration checklist

Run these first, once per session, before any mission. They cost two minutes
and they are what makes every later number trustworthy. Prompt `0`,
mission `M00_calibration`, via `test_campaign_trigger 0`.

| # | manoeuvre | expected in the log | what a failure means |
|---|---|---|---|
| 1 | **Standstill, motor on.** Do nothing for ~5 s. | `imu.csv` `ax`,`ay` ≈ 0, `az` ≈ +9.81; `standstill_jerk_rms` well under the driving value | a floor as loud as the driving numbers: the IMU or the mount is the problem, not the controller |
| 2 | **Push forward by hand**, slowly, in a straight line. | `ax` **positive** while accelerating; `x` in `kinematics.csv` increases | `ax` negative → the IMU x axis is mounted backwards; fix the sign before trusting any jerk figure |
| 3 | **Turn left by hand**, on the spot. | `gz` **positive**; `yaw` increases | `gz` negative → the gyro z axis is inverted, and every `max_abs_yaw_rate` has the wrong sign |
| 4 | **Straight line, constant speed**, a few metres. | speed flat in the dashboard, `jerk_rms` small, `steer_rev_per_m` near 0 | a high `steer_rev_per_m` on a straight line means the controller is hunting, not the corridor |

Runs 1 and 4 together are the noise floor: the jerk and steering numbers of a
mission only mean something relative to them. Judge them in
`campaign_results.csv` like any other test.

---

## Tests

```bash
colcon test --packages-select f1tenth_logger f1tenth_behavior    # the suite runs under colcon
python3 -m pytest src/f1tenth_logger/test/test_campaign -q       # the same, directly (sourced)
python3 -m f1tenth_logger.test_campaign.demo_simulated           # the whole chain, simulated, no ROS
```

| file | covers |
|---|---|
| `test/test_campaign/test_robot_logger.py` | ids, geometry, thread safety, the summary, every abort path, root detection from source, from an install and from a symlink install |
| `test/test_campaign/test_export_campaign.py` | each metric against a closed-form answer, the window, the manual columns |
| `test/test_campaign/test_logger_node_roundtrip.py` | the node against real DDS traffic: all seven lifecycle branches, the startup log and the status topic |
| `test/test_campaign/test_test_campaign_isolation.py` | never in a bringup; disjoint names, folders and topics from the mission logger |
| `f1tenth_behavior/test/test_mission_countdown.py` | the loader's countdown and **every way of stopping it**. It sits with the loader it tests, and its contract test imports this package's event names. |

The data sources have their own tests in their packages:
- `mpc_controller/test/test_campaign_status.py`
- `f1tenth_behavior/test/test_safety_event_published.py`
- `f1tenth_perception/test/test_obstacle_clearance*.py`

The ROS tests move the whole pytest process onto its own `ROS_DOMAIN_ID`
(`test/test_campaign/conftest.py`), with localhost-only discovery and no
discovery server. They cannot reach the car, and the car cannot reach them.
They skip themselves if `rclpy` is missing.

**Not covered here:** a full planner → loader → logger chain in one process.
The loader alone never reaches `mission_finished`, because that transition
comes from the behaviour tree completing the mission. An end-to-end test
without the tree would have to fake the very thing it claims to prove. The
chain is verified in two halves instead:
- the loader's services and timer, directly;
- the node against published traffic;
- plus a contract test that both sides agree on the four event names.

It is verified end to end on the car during the pilot.

---

## Gotchas

- **`ros2 node list` is blind here.** Under the discovery server it under- and
  over-reports, so `test_campaign_logger` can be missing from it while it
  records. Trust `./scripts/stackctl.py status`, the recorder's own startup
  lines and `/test_campaign/logger_status`.
- **The prompt text must match byte for byte.** The planner publishes the
  command exactly as it received it. On the command line that is the
  arguments joined by single spaces; in interactive mode it is the typed line
  with leading and trailing whitespace stripped. Case is never changed and no
  template is added. With `--prompt-num`, a mismatch is recorded as
  `prompt_text_mismatch` in `events.jsonl`. Without it, the test is refused.
  `"Go ahead 3 meters"` is not prompt 1.
- **The countdown is a safety window.** `/mission/abort_mission` and
  `/mission/emergency_stop` both cancel a pending start, and the timer
  re-checks before it starts anything. The car never drives off after a stop.
  Such a run is recorded as `aborted / "cancelled before start"`.
- **`mission_countdown_sec: 0`** restores the old behaviour — start_mission
  starts the mission immediately. `standstill_jerk_rms` is then empty.
- **A failed LLM call is still a test.** It gets a folder, the call is recorded
  with `ok=0`, and the test closes as aborted. Do not treat a missing folder as
  "the LLM failed"; a missing folder means the recorder was not running.
- **Nothing is ever deleted.** Not failed runs, not rows in
  `campaign_results.csv`. If a folder is gone its row stays, so a deletion
  cannot quietly improve the statistics.
