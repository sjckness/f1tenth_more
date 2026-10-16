# Session H1–H5 — branch `campaign-final`, Orin, OFFLINE

Battery disconnected, ZED and Hokuyo absent. No sensor driver and no command
path was launched at any point; the only ROS processes started were a bag play
and the perception node it fed, in an isolated discovery domain, and they were
torn down by the script's own trap. `fdd65e0` → HEAD, one local commit per
point, nothing pushed. Files staged by name.

## MORNING SUMMARY

| point | state | the one line |
|---|---|---|
| **H1** | green | `extract_lines` in C: **24.618 → 1.093 ms p50, 22.5x**, and over all 2292 scans the two backends agree on every segment count, every gate count and to **7.1e-15 m** |
| **H2** | green | tracker on 1 scan in 4, counter-driven, depth-1 subscription; every other `/scan` consumer untouched; counts kept because at a decimated 10 Hz they are FASTER than the 4.2 Hz the node actually managed |
| **H3** | green | cap by role: **36 hidden front cycles → 0**, contract WALL **22.51 → 37.70 s** against the recorded 35.91 |
| **H4** | green | window `a92e133`→`d355bbe`, 17 commits, 11 hours; of 248 archive runs **exactly one** is affected, 42/71 frames, +0.2294 m per contaminated frame |
| **H5** | amber | the script is written and a dry run exercised it end to end — after it found four bugs in itself, all of which would have bitten on the car. The dry-run NUMBERS are bag replay and are not representative |

**The headline: the tracker cycle is 44.20 → 21.58 ms p50, 17 ms UNDER the
pre-E baseline of 38.59**, where last night it was 5.6 ms over. At the
decimated 10 Hz that is 21.6 % of one core, against 177 % for last night's
cycle at the full scan rate.

**Three things for you.** (1) H5's dry run cannot stand in for the car: run it
there, the commands are at the end. (2) H3 means an `'other'` wall is no longer
published at all, so last night's `ROLE_VALUE['other']` cell is unreachable
through the tracker path — if the costmap should carry those surfaces, that is
its own decision. (3) The decimation's one real cost is stated as a test: at
15.5 m a single 10° yaw step costs 0.271 m of `sigma_rho` against a 0.3 m
delete threshold, so a far wall survives one missed cycle at the steering limit
and not two.

**Decisions taken alone:** 9, in `output/cpu/decisions.md`. The two worth
reading are H1's angle-grid guard (the C's API can only express a uniform
bearing grid, so a caller that synthesises bearings gets the Python — it
changed one existing test's answer) and H5's three script bugs.

---
## H1 — extract_lines in C

New ament_cmake package `corridor_perception_c`: C11, no external dependency,
no ROS at build or run time, one shared library `libextract_lines.so` installed
under `lib/`. `corridor_perception.extraction_c` loads it through ctypes with
the path from `ament_index`, and `wall_extract_backend` (default `"c"`) chooses;
an absent library is ONE warning at startup and the Python.

### Equivalence — all 2292 scans of `corridor10m_20261015T171753`

`tools/extract_backends_equiv.py`, output in `output/cpu/h1_equivalence.json`.

| | |
|---|---|
| scans compared | **2292** |
| segments compared | **20 527** |
| segment-count mismatches | **0** |
| per-gate drop-count mismatches (seed/rms/fill/support) | **0** |
| max \|python − c\| on rho | **7.105e-15 m** |
| max \|python − c\| on alpha | **8.882e-16 rad** |
| max \|python − c\| on either endpoint | **7.105e-15 m** |
| max \|python − c\| on length | **3.775e-15 m** |
| max \|python − c\| on the covariance | **1.665e-16** |
| tolerance asked for | 1e-4 |

Eleven orders of magnitude inside it. What is left is float64 summation order:
numpy's pairwise sums against the C's sequential ones. Both backends are fed
the same float32 values (the C's API takes float32) so the comparison measures
the algorithm and not the input's rounding.

### C against Python

| | p50 | p95 | max |
|---|---|---|---|
| `extract_lines` python | 24.618 ms | 26.865 ms | 29.910 ms |
| `extract_lines` **c** | **1.093 ms** | **1.241 ms** | **1.460 ms** |
| speedup | **22.5x** | | |

Target was 2 ms.

### The whole tracker cycle, before and after

Measured the same way as last night's: the replay, payload off, which is the
car's configuration.

| stage | before H1 | after H1 |
|---|---|---|
| `extract_lines` alone | 24.62 ms | **1.09 ms** |
| whole tracker cycle (p50) | 44.20 ms | **21.58 ms** |
| whole tracker cycle (p95) | 53.63 ms | **30.30 ms** |
| scan callback (p50) | 51.13 ms | **28.67 ms** |
| `verify_rectangles` (unchanged) | 2.75 ms | 2.80 ms |
| pre-E baseline, same bag | 38.59 ms | — |

**The cycle is now 17 ms under the pre-E baseline**, where last night it was
5.6 ms over. And H2 bounds how often it is paid: 21.58 ms at 10 Hz is **21.6 %
of one core**, where 21.58 ms at the full 40 Hz would be 86 % and the 44.20 ms
of last night would have been 177 % — which is what "CPU-bound" meant.

## H2 — the tracker on one scan in four

`wall_tracker_scan_decimation` (default 4), a counter rather than a clock so a
replay decimates identically. The scan subscription is now depth 1
(`LATEST_FRAME_QOS`), still BEST_EFFORT. On a skipped scan the tracker's last
verdict stands, and `tracker_skipped` on the debug payload says how stale it is.

### The cycle-count thresholds: counts KEPT, time stated

The counts are what the car does today and what the tests pin, and the
arithmetic favours keeping them: the node was never running at 40 Hz — the
nightly audit measured **4.2 observations a second** because it was CPU-bound —
so every count-based threshold becomes FASTER in wall-clock at a decimated
10 Hz than it is now.

| param | value | before: 40 Hz nominal | before: 4.2 Hz measured | after: 10 Hz |
|---|---|---|---|---|
| `confirm_hits` / `confirm_window` | 3 of 5 cycles, unchanged | 0.125 s of 0.125 s | 1.19 s | **0.50 s** |
| `stereo_confirm_hits` / `_window` | 5 of 7 cycles, unchanged | 0.175 s | 1.67 s | **0.70 s** |
| `wall_negative_miss_to_kill` | 5 cycles, unchanged | 0.125 s | 1.19 s | **0.50 s** |
| `wall_rect_reject_cycles` | 3 verifiable cycles, unchanged | 0.075 s | 0.71 s | **0.30 s** |
| `wall_tracker_delete_after_sec` | 2.0 s, unchanged | 80 cycles | 8.4 cycles | **20 cycles** |
| `wall_tracker_front_max_coast_sec` | 0.3 s, unchanged | 12 cycles | 1.3 cycles | **3 cycles** |
| `wall_coast_max_odom_m` | 1.0 m, unchanged | 2.50 s at 0.4 m/s, rate-independent | | |

The two time-based ones get MORE forgiving in cycle terms, which is the right
direction: at the measured 4.2 Hz a front track that missed a single cycle was
already past `front_max_coast_sec`.

### Every other `/scan` consumer, untouched

Each is a separate process with its own subscription, so a counter inside
`lidar_front_wall_node` cannot reach them. From a grep of the whole workspace
for `LaserScan` subscriptions:

| consumer | its QoS | still at 40 Hz |
|---|---|---|
| `is_proximity_too_close.py` — the BT proximity e-stop | depth 10, reliable | yes |
| `mpc_controller/MPC_corr.py` | `qos_profile_sensor_data` | yes |
| `swept_clearance_node` | `qos_profile_sensor_data` | yes |
| `obstacle_clearance_node` | `qos_profile_sensor_data` | yes |
| `wall_distance_node` | `qos_profile_sensor_data` | yes |
| `front_clearance_node` | `LATEST_FRAME_QOS` (already depth 1) | yes |
| `slam_toolbox` | `slam_toolbox_params.yaml: scan_topic: /scan` | yes |
| nav2 obstacle layer | `nav2_params.yaml: topic: /scan`, twice | yes |
| `lidar_front_wall_node` **estimator half** | — | **yes**: both its topics still publish on every scan, and a test holds it |
| `lidar_front_wall_node` **tracker half** | — | **no: 1 in 4** |

### Association at a 10 Hz cycle, verified

Three tests. 20 cycles of **4 cm of travel** (0.4 m/s at 10 Hz) keep one track
with one id and every cycle associated, and the wall is still at 3.00 ± 0.02 m
after 80 cm. 8 cycles of **10° of yaw** at the steering limit likewise. The
real limit is stated rather than left to be found: `k_yaw` adds
`(k_yaw·|dyaw|·rho)²` per predict, so at 15.5 m a single 10° step costs
**0.271 m** of sigma_rho against the 0.3 m delete threshold — it survives one
missed cycle at that yaw rate and not two, where an undecimated tracker pays
exactly half for the same turn because the model adds squares per step.

## H3 — the publish cap by role

FRONT first (nearest along the heading), then the nearest LEFT, then the
nearest RIGHT, then BACK, then any remaining side walls. `'other'` never takes
a slot.

| | before | after | recorded run |
|---|---|---|---|
| cycles with a confirmed front wall hidden by the cap | **36** of 206 | **0** | — |
| contract WALL time | 22.51 s | **37.70 s** | 35.91 s |
| contract CLEAR time | 51.91 s | 36.72 s | 38.88 s |
| end-wall error vs the true strip, median | 0.010 m (n=54) | **0.010 m (n=90)** | 0.011 m (n=105) |

The cap was ranking by `length * hits / distance`, which put a short far wall
dead ahead below the long near side walls unless its hit count was large — and
hits grow with node uptime, not with evidence. The recorded run only ever
reported that wall because its track was 435 s old.

One consequence, in `decisions.md`: an `'other'` track is not published at all,
so it never reaches `semantic_layer_node` and last night's
`ROLE_VALUE['other'] = 50` is unreachable through the tracker path.

## H4 — runs affected by the front_distance bias

Window: **`a92e133`** (2026-10-15 15:17 — the first scene-mask tick, whose cloud
publish aliased the depth message's header) to **`d355bbe`** (2026-10-16 02:31 —
both publishes given their own header). 17 commits, about 11 hours. `6a9b843`
added the StereoColumns publish 16 minutes in and did the same thing, which is
why F2 found two aliasing sites.

Of the **248** archive runs, **exactly one** falls inside, and the bag says so
rather than the commit date alone:

| run | mission | commit | frames contaminated | median front_distance |
|---|---|---|---|---|
| `2026-10-15T15-18-27_mission-llm_86cff518e42d` | llm_86cff518e42d | e3793e4 | **42 of 71 (59.2 %)** | 2.6158 m contaminated vs 2.8698 m clean |

`output/cpu/front_distance_bias_runs.csv`. The raw clip
`corridor10m_20261015T171753` is affected too (163/241, 67.6 %) but is not an
archive run and has no manifest, so it is in the note, not the CSV.

**The correction: +0.2294 m, per CONTAMINATED FRAME.** Never a constant over a
run — the same run holds both kinds, and `frame_id == "base_link"` (equivalently
`front_offset_m == 0.405`) in `/perception/debug/front_wall` is the per-frame
flag. The five lines for the MATLAB side are in
`output/cpu/front_distance_bias_correction.md`.
## H5 — the CPU test

Three files, no sudo anywhere, everything under `output/cpu_test/<timestamp>/`:

| file | what it is |
|---|---|
| `tools/cpu_test.sh` | the orchestrator: phases, `tegrastats`, the probes, the static snapshot, a `trap` that kills only what it started |
| `tools/cpu_test_probe.py` | `procstat` (per-thread CPU and per-process I/O) and `rates` (the publish rate of each topic, outside any bag) |
| `tools/cpu_test_summary.py` | `summary.md`: a table per phase and the two diffs, a→b and c→d |

### What measures what, and the two substitutions I had to make

| asked for | what runs | why |
|---|---|---|
| `pidstat -u -t -d 1` | `cpu_test_probe.py procstat`, which reads `/proc/<pid>/task/<tid>/stat` and `/proc/<pid>/io` once a second | **`pidstat` is not installed** on this box (no `sysstat`) and installing it needs sudo. `cpu_test.sh` uses `pidstat` when it IS present and says which ran in `summary.md`. The probe reports per-thread CPU %, the CPU each thread last ran on, its affinity mask, and per-process read/write KB/s — the same columns |
| `tegrastats` 1 s | `tegrastats --interval 1000` | works unprivileged here. Parsed for per-core load, GPU %, tj, RAM, the CPU and 5 V rails, and any line mentioning throttling |
| `nvpmodel -q` | as asked | works unprivileged (`MODE_50W` on this box) |
| `jetson_clocks --show` | its refusal is recorded verbatim, and `/sys/devices/system/cpu/*/cpufreq` (cur/min/max/governor) plus every thermal zone is read instead | **it requires root** and the point forbids sudo |
| rates outside the bag | `cpu_test_probe.py rates` | one subscription per topic, from a static topic→type table rather than from graph introspection — see below |
| internal timings | the nodes' own JSON: `tracker_ms`, `rect_ms` and `tracker_skipped` on `/perception/virtual_wall/debug`, `ms` on `/perception/scene_labels/stats`, `solve_time_ms` on `/mpc/status` | they are already published every cycle; nothing new was added to a node for this |

### The dry run found four real bugs in the script

Which is what a dry run is for. All four would have bitten on the car:

1. **`ros2 topic pub -1` hangs** when no subscriber has matched yet, and phase b
   uses it to start the recorder. The first dry run stopped dead there. Both
   publishes are now `timeout N ros2 topic pub -r 2 ...`, which also survives
   the 5–14 s the recorder needs to discover.
2. **The rate counter measured 0 Hz on everything**, because it built its
   subscriptions from `get_topic_names_and_types()` at construction — empty for
   the first seconds of a node's life, and unreliable under the Discovery
   Server, where this box's `ros2 topic list` is routinely blank while topics
   flow. It now subscribes from a static type table and retries every 2 s for
   anything still unmatched.
3. **Nothing discovered anything** in a dry run: `ROS_DISCOVERY_SERVER` is set
   in the environment and the server is started by `supervisor_bringup`, so with
   the stack down every participant pointed at a server that was not there.
   `--dry-run` now unsets it for its own children; the car run does not touch it.

4. **Two `depth_throttle_5hz` nodes outlived the cleanup.** `ros2 launch`
   starts them and killing the launch pid does not always give it time to
   forward the signal. Each launch child now runs under `setsid` so it leads
   its own process group, and the trap signals the GROUP. A `pkill` by node
   name was rejected outright: on the car that would reach the live stack's own
   nodes, and this script must only ever be able to signal what it started.

The cleanup trap was exercised three times — twice by the hang, once
deliberately — and after the fix it leaves nothing behind.

### Dry-run results — **BAG REPLAY, NOT REPRESENTATIVE OF THE CAR**

Phases a and b, 30 s each, no sensors: `lidar_front_wall.launch.py` with the
recording's `/scan`, `/odometry/filtered`, `/tf`, `/tf_static`,
`/camera/detections_3d`, `/perception/stereo_columns`,
`/perception/wall_points`, `/costmap/front_clearance` and the depth
`camera_info` played into it, and in phase b `mission_logger.launch.py` with
`enable_mission_logger:=true` plus a published `/mission/status` RUNNING.

Read these as "the script works and the decimation is visible", nothing more.
What is missing against the car: the ZED and its wrapper, `front_clearance_node`
and its mask tick (the bag's only depth is the 5 Hz throttled copy on a
different topic name, so the node would idle), the rectangle verification
(`/perception/scene_labels/classes` did not exist when the bag was recorded),
the MPC, the behaviour tree, SLAM, nav2, the costmap and the VESC. The box was
otherwise idle, where the car's audit measured 95–97 % on all 12 cores.

What the run DOES show, measured outside the bag by the counter:

| topic | phase a | what it means |
|---|---|---|
| `/scan` | 35.81 Hz | what the bag play delivered |
| `/odometry/filtered` | 32.61 Hz | " |
| `/perception/virtual_wall/debug` | **16.34 Hz** | the contract, published on EVERY scan the node received |
| `/perception/walls` | **3.69 Hz** | the tracker, **one scan in four** — the decimation, end to end on real data |
| `/perception/stereo_columns` | 2.60 Hz | replayed |
| `/perception/scene_labels/stats` | 2.87 Hz | replayed |
| `/mpc/diagnostics` | absent | it does not exist in this stack; `/mpc/status` and `/diagnostics` are the real ones and are in the counter's list |

CPU, same caveat — an otherwise idle box running one node and a bag:

| | phase a (logger off) | phase b (logger on, recording) | delta |
|---|---|---|---|
| board CPU total, median | 186.0 % | 181.0 % | −5.0 (noise) |
| `lidar_front_wall_node` | 24.9 % (p95 32.3), **affinity 5** | 28.4 % (p95 41.5) | +3.5 |
| `mission_logger_node` | — | 1.0 % | — |
| `depth_throttle_5hz` | 1.0 % | 1.0 % | — |
| RAM max | 7169 MB | 7327 MB | +158 MB |
| tj median | 47.6 °C | 47.7 °C | +0.1 |
| GPU | 0 % | 0 % | — |
| throttling lines | 0 | 0 | — |

The a→b CPU delta is noise: recording nine small topics on an idle box costs
nothing measurable, and the useful number — what the full topic list costs on a
loaded box — is exactly what the car run is for. The affinity column confirms
the `taskset -c 5` prefix holds across all of the node's threads, and the
per-node table shows 5 threads in phase a against 9 in phase b.

The `/perception/walls` against `/perception/virtual_wall/debug` ratio in phase
a — **3.69 Hz against 16.34 Hz, 4.4 : 1** — is the decimation measured end to
end on recorded data, which is the one number from this dry run worth keeping.

### The exact commands for the car

Two stack sessions, because the logger's presence is a launch argument and
`enable_mission_logger` now defaults **false** (F6, last night).

```bash
# ---- phases a and c: the logger is NOT running ---------------------------
# terminal 1
ros2 launch f1tenth_bringup supervisor_bringup.launch.py
# terminal 2
cd ~/dev_ws/f1tenth_more && source install/setup.bash
tools/cpu_test.sh --phase a                 # 60 s, stationary, nothing to do
tools/cpu_test.sh --phase c                 # it PROMPTS: start the mission, then Enter

# ---- phases b and d: the logger IS running -------------------------------
# terminal 1 (restart the stack)
ros2 launch f1tenth_bringup supervisor_bringup.launch.py enable_mission_logger:=true
# terminal 2
tools/cpu_test.sh --phase b                 # needs a mission RUNNING for the recorder to record
tools/cpu_test.sh --phase d                 # it PROMPTS: start the mission, then Enter
```

All four in one go, in the session that has the logger on (a and c then measure
"the logger exists but is not recording", which is a different question from
"the logger is not there" — run them in the first session if that distinction
matters):

```bash
tools/cpu_test.sh                            # a b c d, 60 s each
tools/cpu_test.sh --no-motion                # c and d on a stand, no prompts
tools/cpu_test.sh --duration 90 --phase "c d"
```

Each run writes `output/cpu_test/<timestamp>/` with `run.txt`, a
`phase_<letter>/` per phase (`tegrastats.log`, `procstat.json` or
`pidstat.txt`, `rates.json`, `static.txt` with nvpmodel, the clocks, the
thermal zones and every stack process's affinity) and `summary.md` at the top.
`--out DIR` puts it elsewhere; `--bag` points the dry run at another recording.

## Commits

| commit | point |
|---|---|
| `c421578` | H1 — extract_lines in C, 24.6 ms a scan down to 1.1 |
| `e41f606` | H2 — the wall tracker on one scan in four |
| `eaffd16` | H3 — the publish cap is a budget spent on roles, not on a score |
| `6cd1609` | H4 — one archive run carries the front_distance bias |
| (below) | H5 — the CPU test, and the three bugs its dry run found |

## Housekeeping

- No ROS process left running. The only ones started were the dry run's bag
  play, `lidar_front_wall_node` and `mission_logger_node`, in a discovery
  domain isolated from any stack, all killed by the script's trap.
- `colcon build` of the eight touched packages: clean, including the new
  `corridor_perception_c`.
- Suites: `corridor_perception` + `f1tenth_perception` + `f1tenth_costmap`
  **1214 passed, 1 skipped, 1 xfailed**, of which **33 are new** (21 backend
  equivalence, 8 decimation, 4 cap-by-role). The three pre-existing
  `f1tenth_bringup` lint failures are untouched and still pre-existing.
