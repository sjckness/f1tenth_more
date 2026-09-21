# Test logging for LLM-driven runs

One folder per test, one row per test, one campaign per folder. The car drives,
this records; afterwards **you** decide what counted as a success.

That last part is the rule everything else is built around: `finish()` records
only an automatic *hint* (`auto_outcome` / `auto_success`). The verdict that
counts is the `success` column you fill in by hand in `campaign_results.csv`.
The export never overwrites it, and the analysis counts nothing else.

---

## The files

| file | what it is |
|---|---|
| `robot_logger.py` | The logger. One `TestLogger` = one test folder. Standard library + PyYAML, thread-safe, no ROS. |
| `test_logger_node.py` | **The recorder.** An rclpy node you leave running all session; it opens and closes tests by itself. |
| `test_trigger.py` | Manual trigger: fakes a plan result and drives the mission lifecycle by hand. For calibration runs, no LLM involved. |
| `export_campaign_csv.py` | Turns the raw logs into `campaign_results.csv`, one reviewable row per test, with the manual columns left alone. |
| `analyze_tests.py` | Report + plots from the manual verdicts: k/N, Wilson CI, maps, dashboard. |
| `demo_simulated.py` | Fake car, fake LLM, fake MPC. Proves the whole chain without hardware. |
| `conftest.py`, `test_*.py` | The pytest suite (see [Tests](#tests)). `test_logger_node.py` and `test_trigger.py` are executables, not test modules. |

Two files outside this folder are part of the chain:

- `src/f1tenth_intelligence/llm/llm/llm_planner_node.py` publishes `/test/plan_result` after every LLM call.
- `src/f1tenth_behavior/f1tenth_behavior/mission/loader.py` publishes `/test/mission_event` on every mission lifecycle edge, and runs the start countdown.

---

## A session, command by command

Four terminals. Everything below assumes `source install/setup.bash` first.

### 1. The car stack

```bash
ros2 launch f1tenth_bringup supervisor_bringup.launch.py
./scripts/stackctl.py status          # what is actually up (the ros2 CLI lies under the discovery server)
```

### 2. The recorder — start it once, leave it

```bash
python3 tools/test_logging/test_logger_node.py --ros-args \
    -p campaign:=first_test_campaing \
    -p robot_radius:=0.3 \
    -p post_roll_s:=2.0 \
    -p pre_roll_s:=2.0 \
    -p max_test_duration_s:=300.0 \
    -p robot_name:=f1tenth-01 \
    -p llm_model:=qwen2.5-3b-instruct
```

It prints one line per test opened and closed. It records test after test
without restarting. **Ctrl+C closes any open test cleanly** as
`aborted / "logger node shut down"` — nothing is lost.

Before the first run, the campaign needs its prompt table:
`<repo>/first_test_campaing/prompts.yaml`

```yaml
- prompt_num: 0
  mission: M00_calibration
  text: Stand still with the motor on, then push the car forward by hand.
  success_criterion: ax positive while pushed, gz positive turning left, flat while still
- prompt_num: 1
  mission: M01_red_box
  text: Drive down the corridor and stop at the red box.
  success_criterion: stops within 0.3 m of the box, footprint inside the corridor throughout
```

The mission folder name comes from this table and nowhere else. A prompt that
is not in it is refused rather than guessed.

### 3. Calibration runs (no LLM)

```bash
python3 tools/test_logging/test_trigger.py 0 --countdown 5
```

It publishes the plan result, `mission_loaded`, waits out the countdown,
publishes `mission_started`, and then waits. Do the manoeuvre, press **Enter**
to finish, or **Ctrl+C** to abort. See the [checklist](#calibration-checklist).

### 4. Real missions

```bash
ros2 run llm llm_planner_node "Drive down the corridor and stop at the red box." --prompt-num 1
```

`--prompt-num` is what ties the run to a row in `prompts.yaml`. Leave it out
and the logger falls back to matching the exact command text, which works but
gives you no warning if the text drifts. Add `--kind replan` for a second call
against an already-open test.

The planner publishes `/test/plan_result`; the loader publishes
`/test/mission_event`; the recorder writes the folder. The mission waits
`mission_countdown_sec` (default 3 s) at a standstill before it drives — that
standstill is the noise floor every jerk number is measured against.

The countdown is a parameter of the behaviour node, read fresh at every
start, so it can be retuned between tests without restarting anything:

```bash
ros2 param get /behavior_executor_node mission_countdown_sec     # 3.0 by default
ros2 param set /behavior_executor_node mission_countdown_sec 5.0
```

It is not in `stack_params.yaml`, so a restart goes back to 3.0. Set it once
per session, or add it there if you want it permanent.

### 5. Export, judge, analyse

```bash
python3 tools/test_logging/export_campaign_csv.py                 # Excel-EU: ';' and decimal comma
python3 tools/test_logging/export_campaign_csv.py --plain         # ',' and decimal point
```

Open `first_test_campaing/campaign_results.csv` in Excel, fill in **`success`**
(1/0), optionally `transl_ok` and `notes`, save, close.

Re-run the export whenever you like: it keeps every hand-entered value, adds
rows for new tests, and never deletes a row — not even for a test folder you
deleted. If Excel still has the file open it says so and writes
`campaign_results_NEW.csv` instead of losing the run.

```bash
python3 tools/test_logging/analyze_tests.py --footprints 2.0
```

Prints the report and writes `first_test_campaing/analysis/`: `summary.csv`,
`report.txt`, one `overview_map_<MISSION>.png` per mission, a combined
`overview_map.png`, and `dashboard.png`. Tests with an empty `success` are
excluded from k/N and listed as *not yet evaluated*.

---

## Folder structure

```
<repo>/first_test_campaing/
├── campaign.json              created, robot, llm model, git commit
├── prompts.yaml               the prompt table -- the only source of mission names
├── results.csv                one row per finished test, written by the logger
├── campaign_results.csv       one row per test, written by the export, judged by you
├── export_settings.json       the filter settings the last export used
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
        ├── llm_calls.jsonl    the full prompt and response text of each call
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
mission `M00_calibration`, via `test_trigger.py 0`.

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
python3 -m pytest tools/test_logging -q          # 108 tests, ~35 s
python3 tools/test_logging/demo_simulated.py     # the whole chain, simulated, no ROS
```

| file | covers |
|---|---|
| `test_robot_logger.py` | ids, geometry, thread safety, the summary, every abort path |
| `test_export_campaign.py` | each metric against a closed-form answer, the window, the manual columns |
| `test_mission_countdown.py` | the loader's countdown and **every way of stopping it** |
| `test_logger_node_roundtrip.py` | the node against real DDS traffic, all seven lifecycle branches |

The ROS tests move the whole pytest process onto its own `ROS_DOMAIN_ID` with
localhost-only discovery and no discovery server, so they cannot reach the car
and the car cannot reach them. They skip themselves if `rclpy` is missing.

**Not covered here:** a full planner → loader → logger chain in one process.
The loader alone never reaches `mission_finished` — that transition comes from
the behaviour tree completing the mission — so an end-to-end test without the
tree would have to fake the very thing it claims to prove. The chain is
verified in two halves instead (the loader's services and timer directly, the
node against published traffic, plus a contract test that both sides agree on
the four event names), and end to end on the car during the pilot.

---

## Gotchas

- **`ros2 node list` is blind here.** Under the discovery server it under- and
  over-reports. Use `./scripts/stackctl.py status`.
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
