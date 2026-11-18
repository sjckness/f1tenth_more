# Hard-stop parity: Python vs Rust (main-jazzy-rust R4)

The last-resort LiDAR stop (`f1tenth_control/hard_stop.py` + `hard_stop_node.py`) ported to Rust:
`src/rust/f1tenth_hard_stop_rs` (rclrs 0.7.0, like the guard), selected by
`hard_stop_impl:=py|rust` (default py) in `ackermann_mux.launch.py`. linus (i7-9700K, x86_64,
Ubuntu 24.04, Python 3.12.3, glibc libm, rustc 1.99.1), 2026-10-23, branch `rust-hardstop` from
b6909dc1. Binaries under test: the installed `install/f1tenth_hard_stop_rs/lib/f1tenth_hard_stop_rs/`
`hard_stop_node` and `hard_stop_replay` (cargo dev profile, opt-level 3). Every ROS process on an
isolated localhost domain, no discovery server.
Numbers: `parity_hard_stop.json`, `live_hard_stop.json`, `timeout_parity_hard_stop.json` (this
directory); logs `output/rust_main/r4_{parity,live,timeout}.log`.

**Result: 0 differences.** 1,130,585 replay ops, every output compared exactly (bits for floats,
bytes for the diagnostic JSON and the log lines); the live nodes engage and release on the same
scan with byte-identical diagnostics; every timeout case agrees.

| Part | Ops | Scans | Stops | Engage events | Differences |
|---|---:|---:|---:|---:|---:|
| The 42 tests of `test_hard_stop.py` as JSON cases (+ committed file current) | 279 | 55 | 19 | | **0** (+0 vs the committed expectations) |
| CPython float forms (`round`, `repr`, `json.dumps`, `format`, `math.degrees`) on 30,187 values | 392,431 | | | | **0** |
| Pure functions, random inputs (6,000 rounds: hits, mask, polygon, lanes, rule, latch) | 76,200 | | | | **0** |
| Random node sessions (581: parameters, layouts, objects, lanes, ticks, odd clocks) | 169,403 | 100,869 | 32,740 | 2,348 | **0** |
| Car bags, default parameters (89 bags, every recorded /scan and /drive, 20 Hz tick) | 121,093 | 69,964 | 5,335 | 15 | **0** |
| Car bags, `hard_stop_allow_reverse` false | 121,093 | 69,964 | 5,335 | 15 | **0** |
| Car bags, no self-hit mask (every self-return becomes a trigger) | 121,093 | 69,964 | 34,438 | 10 | **0** |
| Car bags, injected obstacles + synthetic reverse commands | 125,793 | 69,964 | 21,293 | 214 | **0** |
| The two bags where the car's own hard stop fired, on the car's timer phase | 2,921 | 1,667 | 1,601 | | **0** (and 1,601 vs 1,600 recorded) |
| **Total** | **1,130,585** | | | | **0** |

Live, isolated domain, both real nodes side by side on the same input: same engage scan, same
release scan, 110/110 diagnostics byte-identical, same QoS, same 15 parameters, 20 Hz republish
through a dead LiDAR, reverse gate and lane timeout within 2 ms of each other; 23/23 timeout
cases agree (car clock and sim clock, running and frozen). CPU at 40 Hz: **4.82 % py, 0.99 %
rust**; per-scan callback compute **239-290 us py, 3.3-6.1 us rust** (x48-81).

## 1. What is compared, and how

`f1tenth_hard_stop_rs::replay` defines JSON-line ops; the same ops go to both sides and every
output is compared:

- **Rust**: `hard_stop_replay` runs `decision.rs` (hard_stop.py, statement for statement) and
  `core.rs` (HardStopNode's `_scan_cb`, `_republish`, `_lane_cb`, `_claim` without ROS). This is
  the code `hard_stop_node` runs; main.rs only adds subscriptions, publishers, the timer and the
  logger.
- **Python** (`tools/rust_parity/hard_stop/py_reference.py`): the real `hard_stop.py` functions and
  a real `HardStopNode` (rclpy, never spun) whose edges only are replaced: `_now` returns the op's
  clock values and counts the reads, `get_logger` captures the lines, the two publishers' `publish`
  is captured, the module's `trigger_range_hits` is wrapped to report the hits it returned.
- **Per callback**: stop yes/no; the latch state after it (`engaged`, `consecutive_hits`,
  `last_hit_time`, `trigger_count`, `ahead_only`, the lane table); the triggering returns
  (range, bearing, x, y by bit pattern); the `/safety/hard_stop` JSON byte for byte; every log line
  (level, text, throttle) byte for byte; the number of clock reads. A node start: refused or not,
  subscriptions, the republish period in ns, the startup line.
- Floats travel as shortest round-trip JSON numbers (serde_json `float_roundtrip`), non-finite ones
  as "nan"/"inf"/"-inf"; scan ranges are cast to f32 on both sides, as in a LaserScan.

What makes the byte-identical JSON possible (`pyfmt.rs`): `math.degrees` is `x * (180/pi)`;
`round(x, n)` is the correctly rounded n-digit decimal read back (Rust `{:.n}` rounds ties to even
like dtoa mode 3); `repr` is the shortest round-trip form with CPython's notation switch
(exponent < -4 or >= 16) and exponent format (`1e-05`, `1e+16`); `json.dumps` writes NaN/Infinity;
key order is `sort_keys`. One trap, found by the format sweep and fixed: on an exact tie between two
shortest candidates (0.65464019775390625 lies exactly between ...062 and ...063) Rust's `{:e}` takes
the upper one and CPython the even one; `pyfmt::shortest` now breaks the tie to even (unit-tested).
It cannot occur in the diagnostic's 4- and 2-decimal values, but it can in a `clear_s` parameter.

## 2. The 42 tests (`test_hard_stop.py` -> `tests/data/test_cases.jsonl`)

`make_test_cases.py` turns each test function into one case: the inputs the test builds (same
scans, times, lanes, parameters) as replay ops, and the Python's outputs for them. The test's own
assertions are re-checked on those outputs at generation, so a case cannot drift from its test.
Parametrised tests are one case each (55 pytest items -> 42 cases, 280 ops). The file lives in the
crate, so `cargo test` (`tests/test_cases.rs`) checks Rust against it without Python; the harness
regenerates it from Python and requires the committed file to be current.

| Test class | Tests | How the case is built |
|---|---:|---|
| TestTriggerAndDebounce | 12 | `trigger_range_hits` on the test's scans (incl. the 136-bearing blind band, 9 bearings all round, radius edges, NaN/inf, under range_min, nearest first); the latch; a node session for the zero-speed stop and its frame_id |
| TestSelfHitMask | 6 | polygon, `is_self_hit`, the rear deck and chassis bearings; node sessions: 20 scans of nothing but self-hits never engage, a real obstacle through them still does |
| TestLatchAndRelease | 8 | the latch op sequences; node sessions for the republish while engaged and after release (the test writes `last_hit_time`: a `set_state` op) |
| TestIndependentOfTheBT | 5 | subscriptions with and without the reverse gate, a stop with no BT input; two are not decisions: the import graph becomes the crate's dependency list (Cargo.toml, package.xml: nothing from the behaviour stack), the stack_params 0.40 m arithmetic is re-checked at generation and the case compares the startup line |
| TestDiagnostic | 3 | node sessions: the JSON every scan, range/bearing, thresholds |
| TestDisabled | 1 | 10 scans and a tick with `hard_stop_enabled` false: no stop, no diagnostic |
| TestReverseGate | 7 | `stop_now`, `winning_speed`, `all_ahead`; node sessions: reverse passes, the first non-negative command re-claims at once, an obstacle beside the nose and the gate off block reverse |

The Rust unit tests (57) port the same assertions directly (decision.rs, core.rs, config.rs).
Mutation check: a swapped JSON key order, a skipped clock read, an f32 bearing and an exclusive
radius each fail `tests/test_cases.rs` with the exact op and field.

## 3. Recorded car scans

All bags under `~/f1tenth_campaigns/first_test_campaign/raw` and `~/bags` with a /scan (89 bags,
69,964 scans, 15,408 /drive messages; no /teleop or /calibration_drive was recorded, no negative
/drive speed), read only. One bag (`~/bags/corridor_obstacles_20261019T135307`) has a metadata.yaml
that rosbag2's yaml-cpp rejects; its .db3 is read with sqlite3 in read-only mode instead. Each bag
is replayed as the node sees it: every /scan and lane message at its receive time, plus a republish
tick every 50 ms.

- **Self-hits are real and frequent**: 19,328 scans in 7 bags carry returns inside the footprint
  mask (510,362 returns within 0.2 m of the laser, down to 0.021 m); in the 2026-10-08 runs almost
  every scan (2,189/2,189 and 14,481/14,495). With the mask (default) none of them triggers; the
  "no mask" variant turns them into 34,438 stops. Both implementations agree on every one.
- **Real triggers**: 2,270 scans have returns outside the mask within 0.2 m; with the default
  parameters they give 15 engage events and 5,335 stops, identical on both sides.
- The pre-2026-10-22 bags record the laser at base_link x 0.12 on /tf_static (later ones 0.305);
  the node takes `laser_x_m` (0.305) as a parameter and never reads TF, so the replay does the same.
  The self-hit counts above use 0.305.

**The car's own hard stops.** In two runs of 2026-10-22 the car's (Python) hard_stop_node stopped
the car (1,600 `/safety_stop` messages with frame_id `base_link/hard_stop`). The replay reproduces
them (informative, not gated: the run's parameters are taken to be the launch defaults):

| Bag | Recorded stops | Replay (tick phase = first message + 50 ms) | Replay on the car's tick phase | Episodes recorded / replay |
|---|---:|---:|---:|---|
| 2026-10-22T12-07-54 | 424 | 423 | **424** (phase 24.66 ms, from 297 timer stops) | 1 / 1, ends within 3 ms |
| 2026-10-22T12-11-04 | 1,176 | 1,258 | **1,177** (phase 24.45 ms, from 761 timer stops) | 6 / 6 on the car's phase, every boundary within 6 ms |

With an arbitrary tick phase the second run merges two episodes: the last hit before a gap was at
19.002 s and the next at 19.506 s, 0.504 s later, so the 0.5 s clear hold releases only if a 20 Hz
tick lands in the 4 ms window before the next hit. The car's timer did; the phase is estimated from
the recorded stops that came from the timer (no scan within 3 ms before) and the replay on it
reproduces all six episodes. Python and Rust agree on both phases (gated, 0 differences).

## 4. Live, isolated domain (`live_hard_stop_test.py`)

The two real nodes as processes (car clock), one driver publishing the same `/live/scan` (the car's
1081-beam layout at 38 Hz) and `/drive` to both. All 26 checks pass (`live_hard_stop.json`).

| Phase | Python | Rust |
|---|---|---|
| A: clear 0.6 s, object at 0.18 m ahead 1 s, clear 1.2 s | engage on scan 25 (the 2nd object scan), release on scan 82, 86 stops, first stop 1.01 ms after the engaging scan | engage on scan 25, release on scan 82, 87 stops, first stop 0.54 ms after it |
| A: diagnostics | 110 | 110, **byte-identical to the Python's, scan by scan** |
| A: stops | speed 0, steering 0, frame_id `base_link/hard_stop` | the same |
| B: LiDAR stops for 1.5 s while engaged | 9 republished stops, the last 451 ms after the last scan, none after | 10, the last 488 ms after it, none after |
| C: /drive -0.4 while engaged ahead, then /drive 0.0 | 0 stops while reversing, 38 diagnostics `passing_reverse`, re-claim 0.73 ms after /drive 0.0 | 0, 38, re-claim 0.50 ms after it |
| D: /drive -0.4 then silence | claims again 208.9 ms after the last command | 207.3 ms |
| E: 20 s at 40 Hz, an object every 3 s | CPU 4.82 % | CPU **0.99 %** |
| QoS from the graph | /scan best effort volatile; lanes, stop, diagnostic reliable volatile | the same (history depth not reported by the graph) |
| Parameter services (list + get) | 15 hard-stop parameters (+ use_sim_time, start_type_description_service) | the same 15 (+ use_sim_time), every value equal |
| SIGINT | KeyboardInterrupt path, exit 1 (rclpy.shutdown raises after the context shut down: pre-existing) | default signal action, exit -2 (as the guard and costmap nodes) |

## 5. Timeout parity (`timeout_parity.py`)

Four nodes at once, the same inputs: py_car / rs_car (system clock) and py_sim / rs_sim
(use_sim_time, /clock from the driver at 200 Hz, frozen on demand). **23/23 cases agree**
(counts within 2, times within 60 ms; lane within 30 ms). Selected rows (ms after the last scan):

| Case | py_car | rs_car | py_sim | rs_sim |
|---|---|---|---|---|
| clear, every gap 150-1000 ms, clock running or frozen (10 cases) | no stop | no stop | no stop | no stop |
| engaged, 500 ms gap, clock running: gap stops / after resume / last stop | 10 / 0 / 486 | 10 / 0 / 494 | 10 / 0 / 496 | 10 / 0 / 496 |
| engaged, 1000 ms gap, clock running | 10 / 0 / 491 | 10 / 0 / 500 | 9 / 0 / 454 | 9 / 0 / 454 |
| engaged, 260 ms gap (the Isaac pause), clock running | 5 / 14 / 497 | 6 / 13 / 495 | 6 / 13 / 495 | 6 / 13 / 495 |
| engaged, 500 ms gap, clock FROZEN | 9 / 0 / 453 | 10 / 0 / 461 | 1 / 28 / 995 | 1 / 28 / 995 |
| engaged, 1000 ms gap, clock FROZEN | 10 / 0 / 458 | 10 / 0 / 466 | 1 / 28 / 1495 | 1 / 28 / 1494 |
| lane silence, no freeze: claims again after | 208.8 | 204.3 | 208.9 | 208.5 |
| lane silence, clock frozen for its first 150 ms | 208.9 | 208.6 | 365.2 | 364.5 |
| lane silence, clock frozen for its first 400 ms | 209.0 | 208.5 | 625.0 | 624.6 |

Under use_sim_time both republish timers run on ROS time (rclpy timers do; the Rust node asks
rclrs for `node_time()`, whose default would be the steady clock), so during a frozen /clock
neither republishes and the clear hold and the lane timeout pause with simulated time: the sim pair
agrees within 0.5 ms. On the car the system clock governs both, exactly as before.

## 6. Timing (offline, the node's per-scan work without ROS)

`_scan_cb` on a real HardStopNode (mocked publishers) vs `core::on_scan` (`bench_scan` op), median
of 5 batches:

| Scan | Python | Rust | Ratio |
|---|---:|---:|---:|
| blank, 1081 beams | 267 us | 3.3 us | x81 |
| object ahead (3 beams: hits, log text, JSON) | 290 us | 6.1 us | x48 |
| a car scan (2026-09-22T11-12-46) | 239 us | 3.5 us | x67 |

## 7. Deliberate differences and findings

- **Refused at startup (Rust only)**: `footprint_front_x_m` / `footprint_half_width_m` missing (the
  launch file passes the Python node's own defaults, `_default_front_x()` / `_default_half_width()`;
  the Rust node has no URDF or stack_params access and will not guess), and a
  `hard_stop_republish_hz` that gives no positive ns period. The Python dies at 0, negative and NaN
  too; at inf it would run a 0 ns timer, the Rust refuses. Everything else is accepted as the Python
  accepts it.
- **Read-only parameters**: `ros2 param set` is refused by the Rust node; the Python accepts it and
  ignores it (it reads every parameter once). The R1 corridor_impl pattern.
- **rclrs parameter handles** (found here, fixed in this crate): rclrs 0.7 undeclares a parameter
  when its `ReadOnlyParameter` handle is dropped, so a node that reads a value and drops the handle
  shows no parameters to `ros2 param` (only use_sim_time). The hard-stop node keeps its 15 handles
  for its lifetime (live check above). **The guard (`f1tenth_proximity_guard`) and the costmap node
  (`f1tenth_costmap_boundary_rs`) have the same pattern** (not changed here; recorded in
  decisions.md). Behaviour is unaffected, only introspection.
- **Doc mismatch on main-jazzy, not changed**: ackermann_mux.launch.py's `hard_stop_enabled`
  description says a disabled node "keeps publishing its diagnostic"; `hard_stop_node.py` returns
  before publishing anything when disabled. The Rust ports the code (no diagnostic when disabled;
  TestDisabled case).
- **Shutdown**: the Rust node ends on SIGINT by the signal's default action (exit -2, launch logs
  "process has died"), like the guard and costmap nodes; rclrs 0.7 installs no SIGINT handler.
- `ros2 param list --no-daemon /hard_stop_node` once printed only `use_sim_time` twice while the
  node's list_parameters service, called directly, returns all 16 names (live check); a CLI or
  discovery artefact of one-shot CLI participants, not a node fault.

## 8. How to run it

```bash
source ~/dev_ws/f1tenth_more_rust/output/rust_main/env_r4.sh; cd $WS   # workspace built, sourced
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=95 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 tools/rust_parity/hard_stop/make_test_cases.py --check      # the committed cases are current
python3 tools/rust_parity/hard_stop/parity_hard_stop.py             # everything offline, ~9 min
python3 tools/rust_parity/hard_stop/parity_hard_stop.py --quick     # small fuzz, 3 bags, ~20 s
python3 tools/rust_parity/hard_stop/live_hard_stop_test.py          # live, its own random domain, ~40 s
python3 tools/rust_parity/hard_stop/timeout_parity.py               # live, four nodes, ~75 s
(cd src/rust && cargo test -p f1tenth_hard_stop_rs)                 # incl. the 42 cases, no Python
```

Not done here: the Isaac B2 scenario py vs rust (linus has 19 GB free on /, `tools/e2e_test.sh` needs
25 GB; pending for R7.1 like R0/R3), aarch64 runs (Thor/Orin, R7.2/R7.3: bit parity is expected
there for the decision, which uses only +, *, cos, sin and comparisons, but not promised).
