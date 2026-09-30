# Mission Analysis — 2026-09-01, runs 14:32–15:11 UTC

Source: `~/.ros/mission_bags/` (9 runs, ~1.78 GB), read offline via `sqlite3` +
`rclpy.serialization`. Supplemented by `~/.ros/log/python3_*.log` (MPC node logs)
where the needed signal is logged but not published.

**Headline:** the car does not stop because the MPC fails. Across all runs the MPC
solved **1285/1285 QPs successfully, with zero infeasible solves and zero missed
control deadlines**. Every stop is another node preempting the MPC on the
ackermann_mux `safety_stop` lane (priority 200). Three distinct trip causes were
identified, one of which — **transient GPU-utilisation spikes** — has nothing to do
with obstacles at all and was previously invisible.

---

## Method note — what "stopped" actually means here

The command chain is:

```
MPC_corr  --/drive-->  ackermann_mux  --/ackermann_drive-->  vesc (ackermann_to_vesc)
                            ^
        BT Stop() --/safety_stop--/  (priority 200 vs navigation's 10)
```

`/drive` is only the MPC's *request*. `/ackermann_drive` is what the VESC consumes,
so that is the ground truth for "stopped". The two disagree a lot:

| run | `/drive` non-zero | `/ackermann_drive` non-zero |
|---|---|---|
| 14-35-50 | 0 % (326 msgs) | 0 % (277) |
| 15-00-06 | 100 % (170) | **48 %** (149) |
| 15-03-07 | 100 % (74) | 100 % (71) |
| 15-04-45 | 100 % (220) | **88 %** (204) |
| 15-06-45 | 98 % (107) | 91 % (101) |
| 15-08-12 | 100 % (214) | **73 %** (198) |
| 15-09-43 | 99 % (286) | **73 %** (263) |
| 15-11-29 | 100 % (189) | **85 %** (175) |

`/safety_stop` is **not recorded** (see Gaps), but the mux republishes the winning
message verbatim and the two sources are distinguishable by `header.frame_id`:
`Stop` (behaviours/stop.py) stamps `base_link`; `MPC_corr._publish_drive()` leaves it
empty. That gives exact per-sample attribution:

- **1438** actuated samples total across the 8 non-empty runs.
- **508 (35 %)** carry `frame_id=base_link` → BT safety-stop lane.
- **31 (2 %)** are MPC-produced zeros (27 of them in 14-35-50 alone).
- Outside run 14-35-50, only **4** zero-speed actuated samples are MPC-produced;
  every other zero across all runs is `base_link`.

---

## Runs Analyzed

All 9 runs: branch `scene-graph`, commit `580a2af`, **`dirty: true`** (working-tree
changes in every run — none is reproducible evidence on its own). All share
`camera_source=zed`, `confidence_threshold=0.3`, `enable_slam=true`,
`localization_source=ekf`, `enable_nav2=false`, `enable_intelligence=true`,
`use_behavior_tree=true`, `enable_sys_obs=true`, `yolo_model=yolo26s-seg.pt`.

| # | run_id | mission | outcome | wall | recorded | disc. lat | stopped | size |
|---|---|---|---|---|---|---|---|---|
| 0 | `14-32-09_mission-test_01` | test_01 | COMPLETE | 5.3 s | **0 msgs** | – | – | 24 KB |
| 1 | `14-35-50_mission-test_01` | test_01 | COMPLETE | 35.6 s | 31.8 s | 3.50 s | **90 %** | 361 MB |
| 2 | `15-00-06_mission-llm_plan_1788274806` | llm_plan | **null** | – | 16.5 s | 5.43 s | 52 % | 189 MB |
| 3 | `15-03-07_mission-bottle_then_person` | bottle_then_person | ABORTED | 11.8 s | 7.0 s | 4.64 s | 0 % | 82 MB |
| 4 | `15-04-45_mission-bottle_then_person` | bottle_then_person | **null** | – | 21.0 s | 4.90 s | 12 % | 240 MB |
| 5 | `15-06-45_mission-llm_plan_1788275205` | llm_plan | COMPLETE | 14.7 s | 10.3 s | 4.17 s | 7 % | 114 MB |
| 6 | `15-08-12_mission-llm_plan_1788275291` | llm_plan | **null** | – | 20.9 s | 3.85 s | 27 % | 245 MB |
| 7 | `15-09-43_mission-bottle_then_person` | bottle_then_person | COMPLETE | 32.4 s | 28.1 s | 4.02 s | 26 % | 334 MB |
| 8 | `15-11-29_mission-bottle_then_person` | bottle_then_person | ABORTED | 22.4 s | 17.7 s | 4.57 s | 15 % | 211 MB |

`disc. lat` = gap between manifest `start_time` and the first recorded actuation
message. All `t+` offsets below are relative to **first recorded message**, not
mission start; add the latency column to compare against manifest times.

### Like-for-like warning

Two config groups; **do not compare stop rates across the boundary**:

- **Group A** (`yolo_model_task=detect`, `use_mask_depth=false`): runs 0, 1, 2
- **Group B** (`yolo_model_task=segment`, `use_mask_depth=true`): runs 3–8

Note both groups load the same `yolo26s-seg.pt` weights; Group A runs seg weights
under the `detect` task. Group A also lacks lidar entirely (run 1 recorded 0 `/scan`,
0 `/odom`), so its `map->odom` is identity and its proximity lane could never trip.
Run 0 is an **empty but valid bag** (5.3 s run vs ~4 s recorder discovery latency) —
excluded from all statistics, not evidence of a recording fault.

---

## Category 1 — Unexpected Stops

53 contiguous `base_link` fragments merge (gaps < 0.35 s = mux lane flicker, see
below) into **9 coherent stop episodes**, totalling **64.5 s of 153.3 s recorded
driving time (42 %)**.

Every episode was reproduced offline by replaying the BT's own predicates against
recorded data — `IsProximityTooClose` (front cone ±45° < 0.40 m; side/rear < 0.20 m,
from `/scan`), `IsObstacleDetected` (bbox centre in `0 ≤ x ≤ 1.0`, `|y| ≤ 0.32`,
`|z| ≤ 0.32` in `zed2_left_camera_frame`, latched until the next message), and
`IsSystemOverheated` (`cpu_percent`/`gpu_percent` > 95, temps > 100 °C).

### Cause buckets

| bucket | episodes | stopped time | share |
|---|---|---|---|
| Camera corridor blocker (`handle_obstacle`) | 3 | 42.5 s | 66 % |
| Lidar proximity (`IsProximityTooClose`) | 4 | 21.1 s | 33 % |
| **GPU load spike (`IsSystemOverheated`)** | 3 | 4.0 s | 6 % |
| EKF correction spike | 0 | – | – |
| Boundary/`front_clearance` tightening | 0 | – | – |
| Detection dropout (stale latched blocker) | 0 | – | – |
| Unexplained | **0** | – | – |

(Percentages exceed 100 % because episode `15-04-45 t+8.57` satisfies both the
proximity and corridor predicates simultaneously.)

### Per-episode evidence

Duty cycle = fraction of the episode during which each predicate held, sampled at
20 Hz.

| run | t+ | dur | PROX | OBST | SYS | front_clearance | verdict |
|---|---|---|---|---|---|---|---|
| 14-35-50 | 0.06 | **31.73 s** | 0 % | **100 %** | 0 % | 0.48 (constant) | chair |
| 15-00-06 | 6.41 | 10.10 s | **100 %** | 0 % | 0 % | 0.35–0.66 | lidar front |
| 15-04-45 | 8.57 | 3.12 s | 68 % | **95 %** | 0 % | 0.35–1.98 | person, both |
| 15-06-45 | 0.88 | 0.91 s | 0 % | 0 % | **100 %** | 2.90–3.12 | **GPU 99.0 %** |
| 15-08-12 | 1.61 | 2.11 s | 0 % | 0 % | **90 %** | 4.23–4.69 | **GPU 99.1/95.1 %** |
| 15-08-12 | 16.30 | 4.60 s | **100 %** | 0 % | 0 % | 0.28–0.82 | lidar front |
| 15-09-43 | 0.00 | 7.60 s | 0 % | **100 %** | 0 % | 0.78–6.00 | bottle |
| 15-09-43 | 16.10 | 1.00 s | 0 % | 0 % | **95 %** | 2.65–2.80 | **GPU 96.8 %** |
| 15-11-29 | 14.35 | 3.33 s | **100 %** | 0 % | 0 % | 0.25–1.05 | lidar front |

### 1a. GPU load spikes — a genuine, previously invisible stop cause

`IsSystemOverheated` trips on `gpu_percent > 95`. `/diagnostics/system_status` is
published at **1 Hz**, and the BT latches the last sample, so one spiking sample
stops the car for a full second. Onset correlation is essentially exact:

| run | sample | GPU % | stop onset | lag | duration |
|---|---|---|---|---|---|
| 15-06-45 | `1788275210.96` | 99.0 | `t+5.05` (abs `1788275211.02`) | **0.05 s** | 0.91 s ≈ 1 sample |
| 15-08-12 | `1788275297.57` | 99.1 | `t+5.46` (abs `1788275297.62`) | **0.05 s** | 2.11 s ≈ 2 samples (99.1 then 95.1) |
| 15-09-43 | `1788275404.09` | 96.8 | `t+20.12` (abs `1788275404.10`) | **0.01 s** | 1.00 s ≈ 1 sample |

Corroboration that these are not obstacle stops: `front_clearance` was 2.65–4.69 m
throughout (vs 0.25–0.82 m during genuine proximity stops), and neither perception
predicate held for a single sample. GPU utilisation is otherwise wildly variable
(3.0 %–99.1 % between consecutive 1 Hz samples in run 15-08-12) — this is YOLO
inference burst load, not thermal distress. CPU never exceeded 87.4 % and temps sat
at 55 °C CPU / 49 °C GPU against a 100 °C limit, so **load, not heat, is the trip**.

### 1b. Run 14-35-50 — the car never moved, and the mission reported COMPLETE

The most serious single finding. For the entire 31.8 s recording:

- `/drive` speed was **identically 0.000** (min = max = 0.000, 326 msgs).
- **100 % of the 378 `detections_3d` frames** contained a corridor blocker — a
  `chair` at `x ≈ 0.66 m, y ≈ -0.01, z ≈ -0.18`, score 0.84–0.86, essentially
  motionless. Also present: `potted plant` (374), `tv` (312), `dining table` (83).
- The MPC log (`~/.ros/log/python3_129004_1788273105459.log`) contains **zero
  `SOLVE/out` lines** — the solver was never invoked — and one warning:
  `Nessun comando su /mpc/goal_distance ancora ricevuto: robot fermo in attesa.`
- `/mpc/goal_distance`, `/mpc/goal_pose` and `/mpc/goal_turn` carried **0 messages**.

Mechanism: `create_root()` builds a `Selector(memory=False)` ordered
`[emergency, handle_obstacle, mission, navigation]`. A `Selector` stops at its first
succeeding child, so while `handle_obstacle` succeeds the **`mission` subtree is
never ticked at all** — `PublishMoveGoal` never runs, no goal is ever published, and
the MPC idles in its "no goal" branch. `test_01.json` needs `/mpc/goal_pose` (two
`goal_pose` moves) and `/mpc/goal_turn` (one turn); none was ever sent.

The mission nevertheless published `state=COMPLETE` at `t+32.58` with
`emergency_stop_active=false`, and the manifest recorded `outcome: COMPLETE`. **A
mission that never moved and never issued a goal was recorded as a success.** The
behaviour node's own log for this window has since been overwritten (see Gaps), so
the path by which it reached COMPLETE cannot be recovered from this data — this is
the single highest-value open question below.

### 1c. Mux lane flicker — a marginal timing budget

The BT ticks at `bt_loop_duration_ms = 100`, and the `safety_stop` mux lane expires
after `timeout: 0.2` (mux.yaml). That is only a **2× margin**: one late tick releases
the lane and the MPC's command is actuated for a frame. This is what fragments 9 real
episodes into 53 recorded fragments — e.g. run 14-35-50's single 31.7 s block of
sustained blockage appears as 27 fragments separated by 0.14–0.30 s gaps, during
which the 27 MPC-frame zeros were actuated. It is cosmetic here (MPC was commanding
0.0 anyway) but on a run where the MPC is commanding motion it produces brief
unintended actuation pulses inside what should be a solid stop.

### 1d. Buckets that the data ruled out

- **EKF/`map->odom` correction spikes: not a stop cause.** Corrections are real and
  occasionally large (below), but not one of the 9 episodes has a correction spike in
  its onset window, and MPC solve behaviour is uncorrelated with them (Category 2).

| run | steps | median | p95 | max | max yaw | steps > 5 cm |
|---|---|---|---|---|---|---|
| 14-35-50 | 449 | 0.0000 | 0.0000 | 0.0000 m | 0.00° | 0 |
| 15-00-06 | 822 | 0.0003 | 0.0111 | 0.3534 m | 6.45° | 14 |
| 15-03-07 | 297 | 0.0002 | 0.0020 | 0.0839 m | 1.51° | 10 |
| 15-04-45 | 985 | 0.0001 | 0.0024 | 0.0972 m | 4.55° | 14 |
| 15-06-45 | 513 | 0.0018 | 0.0144 | 0.2850 m | 5.36° | 15 |
| 15-08-12 | 947 | 0.0006 | 0.0202 | 0.5299 m | 6.37° | 24 |
| 15-09-43 | 1393 | 0.0006 | 0.0099 | 0.3589 m | 7.49° | 35 |
| 15-11-29 | 703 | 0.0022 | 0.0499 | **0.8877 m** | **9.86°** | 36 |

- **Boundary tightening: not a stop cause.** `/costmap/boundaries` `n_constraints` is
  effectively static (run 15-09-43: 3 for 560 msgs, 2 for 9; run 15-08-12: 3 for 410,
  2 for 12). The feasible region never collapsed.
- **Detection dropout → stale latched stop: did not occur in these runs.** The
  latched-blocker replay found no episode where the trip depended on a stale
  detection (all `OBST` onsets had blocker age ≤ 0.05 s). The mechanism is real and
  the dropouts exist (Category 3, gaps up to 6.16 s) — it just did not fire here.

---

## Category 2 — MPC Convergence

**There is no MPC convergence problem in this data.** Solver diagnostics *are*
computed (`MPC_corr.py:1317-1340`) but only written to the ROS logger, never
published — so they are absent from every bag and had to be recovered from
`~/.ros/log/python3_*.log`.

**1285 `SOLVE/out` records** recovered across the session window:

- `success=True`: **1285 / 1285 (100 %)**
- `status=1`, `status_message=solved`: **1285 / 1285**. (OSQP's "solved" code is 1,
  not 0 — a naive `status != 0` check reads as 100 % failure and is wrong.)
- **0 infeasible, 0 failed, 0 fallback-to-stop from the solver.**
- `cost`: min 0.443, median 3.220, max 41.827 — no divergence.

### Timing against the control period (`ts = 0.1 s`)

| metric | value |
|---|---|
| median `solve_dt` | **12.4 ms** |
| p95 | 20.4 ms |
| p99 | 27.2 ms |
| max | **48.4 ms** |
| solves > 100 ms (deadline miss) | **0 (0.00 %)** |
| solves > 50 ms (half the budget) | **0 (0.00 %)** |

The RTI/OSQP path (`use_rti_solver=true`) is comfortably inside budget — the worst
observed solve used 48 % of one period. The 93 ms SLSQP figure quoted in
`MPC_corr.py:263` is the *old* solver and no longer applies.

### Correlation against pose corrections and constraint churn

| run | n | Pearson r (`solve_dt` vs `map->odom` step) | median `solve_dt` when correction > 2 cm | vs quiet |
|---|---|---|---|---|
| 15-00-06 | 154 | +0.056 | 13.2 ms (n=2) | 10.6 ms |
| 15-03-07 | 72 | −0.108 | 13.3 ms (n=3) | 13.2 ms |
| 15-04-45 | 99 | −0.067 | 13.8 ms (n=4) | 13.4 ms |
| 15-06-45 | 103 | −0.095 | 10.6 ms (n=4) | 10.8 ms |
| 15-08-12 | 169 | −0.028 | 10.9 ms (n=12) | 11.9 ms |
| 15-09-43 | 281 | +0.173 | 22.0 ms (n=4) | 13.0 ms |
| 15-11-29 | 133 | +0.015 | 11.8 ms (n=21) | 12.1 ms |

No meaningful correlation (|r| ≤ 0.173, and the largest r rests on n=4). Warm-start
degradation from EKF corrections is **not** observable here. Constraint-set churn is
near-zero, so the optimisation landscape is stable between calls.

**Conclusion:** what reads as "MPC fails to converge" is the MPC solving correctly
at 12 ms/tick while its output is discarded at the mux. Any future investigation
should start at `/ackermann_drive` and `frame_id`, not at the solver.

---

## Category 3 — Obstacle Marker Jitter

**Precondition confirmed first:** markers *are* tracked. `semantic_layer_node`
runs a real batch-association tracker with confirm/lost lifecycle, and
`marker.id` is the track's own caller-assigned, never-reused `track_id` (not the
old `hash(class_id)+index` scheme). So frame-to-frame deltas per `(ns, id)` are
genuine same-object motion, and this is a noise problem, not a missing-tracking
problem.

### Jitter magnitude (map frame, per track, consecutive updates ≤ 0.5 s apart)

| run | frames | tracks | steps | median | p95 | max | > 10 cm | > 30 cm |
|---|---|---|---|---|---|---|---|---|
| 15-04-45 | 179 | 18 | 398 | 0.0227 | 0.1093 | 0.2428 m | 28 (7 %) | 0 |
| 15-06-45 | 24 | 4 | 8 | 0.0000 | 0.0000 | 0.0000 m | 0 | 0 |
| 15-08-12 | 163 | 18 | 276 | 0.0102 | 0.1205 | 0.2871 m | 26 (9 %) | 0 |
| 15-09-43 | 218 | 14 | 650 | 0.0096 | 0.0822 | 0.1239 m | 6 (1 %) | 0 |
| 15-11-29 | 60 | 4 | 44 | 0.0199 | 0.1287 | 0.1308 m | 4 (9 %) | 0 |

(Track counts include `<class>_label` companion markers; halve for object tracks.)

Restricting to steps where the robot was **near-stationary** (`|v| < 0.05 m/s`,
isolating obstacle noise from ego-motion) changes essentially nothing — run 15-04-45
median 0.0220 vs 0.0227 overall; run 15-09-43 0.0033 vs 0.0096. **Ego-motion is not
the driver.**

### Attribution

**Ruled out — pose inherited from `map->odom` corrections.** For every step with
jitter > 10 cm, the total `map->odom` motion in the same interval was:

| run | jitter > 10 cm steps | of which `map->odom` moved > 2 cm | median `map->odom` motion | max |
|---|---|---|---|---|
| 15-04-45 | 14 | **0 (0 %)** | 0.0001 m | 0.0004 m |
| 15-08-12 | 13 | 2 (15 %) | 0.0061 m | 0.3497 m |
| 15-09-43 | 3 | **0 (0 %)** | 0.0031 m | 0.0168 m |

And the converse holds: the *largest* `map->odom` motions (0.1064 m, 0.4925 m,
0.3250 m in runs 15-04-45 / 15-08-12 / 15-09-43) all coincided with **small** jitter
steps (≤ 2 cm). Localisation corrections and marker jitter are decoupled.

**Primary cause — detection/depth noise.** Measuring raw detection displacement in
`zed2_left_camera_frame` (nearest-neighbour match per class between consecutive
frames, ≤ 0.6 m gate) removes ego-motion and localisation entirely, and the noise
there is **larger than the published marker jitter**:

| run | confidence | n | median | p95 | max |
|---|---|---|---|---|---|
| 15-04-45 | ≥ 0.75 | 69 | 0.0711 | 0.1804 | 0.4995 m |
| | 0.50–0.75 | 104 | 0.0803 | 0.4166 | 0.5915 m |
| | < 0.50 | 70 | **0.1426** | 0.4525 | 0.5901 m |
| 15-08-12 | ≥ 0.75 | 25 | 0.1209 | 0.3511 | 0.5647 m |
| | 0.50–0.75 | 68 | 0.2213 | 0.5681 | 0.5995 m |
| | < 0.50 | 110 | 0.1448 | 0.4436 | 0.5601 m |
| 15-09-43 | ≥ 0.75 | 205 | 0.0329 | 0.1109 | 0.2673 m |
| | 0.50–0.75 | 107 | 0.0012 | 0.1065 | 0.4340 m |
| | < 0.50 | 10 | **0.2493** | 0.3145 | 0.4745 m |

Low-confidence detections jitter **2–7×** more than high-confidence ones in runs
15-04-45 and 15-09-43 — the signature of the YOLO/depth estimation stage, not
localisation. The tracker is already absorbing most of it (map-frame median 1–2 cm
vs camera-frame median 3–22 cm); the residual is what leaks through.

**Secondary cause — detection flicker and 3D yield loss.** Not position noise but
appear/disappear churn:

| run | `/camera/detections` | `/camera/detections_3d` | 3D yield | max 3D gap | 2D on/off toggles per class |
|---|---|---|---|---|---|
| 15-04-45 | 275 @ 12.6 Hz | 178 @ 8.2 Hz | 65 % | 1.57 s | person 24, bottle 20, tv 31 |
| 15-08-12 | 285 @ 13.6 Hz | 161 @ 7.7 Hz | 56 % | **6.16 s** | person 13, chair 3, bottle 20, tv 6 |
| 15-09-43 | 392 @ 13.8 Hz | 217 @ 8.1 Hz | 55 % | 3.79 s | person 7, bottle 7, tv 8 |

**35–45 % of 2D detection frames produce no 3D detection at all.** Track churn
follows: run 15-04-45 spawned 5 `person` + 4 `tv` tracks in 21 s for what is almost
certainly one person and one TV; 4 of 9 object tracks lived < 1 s. Run 15-08-12:
4 `person` + 5 `chair` tracks, 4 short-lived. Markers therefore appear and vanish
rather than moving smoothly — visually the dominant "jitter" even though the
position noise is only ~2 cm.

> **CORRECTION (2026-09-02).** The 35–45 % figure above is real but was
> misattributed here. Measuring it properly (see *Yield investigation* below)
> shows the missing frames are overwhelmingly **empty** ones, so almost no
> obstacle data was ever lost. The churn is driven by per-class flicker
> *within* non-empty frames, not by wholesale frame loss.


---

## Open Questions / Instrumentation Gaps

Ranked by how much they blocked this analysis.

1. **`/safety_stop` is not recorded.** `_DEFAULT_TOPICS` (mission_logger_node.py:174)
   omits the one topic that directly explains 35 % of all actuated samples. Recovered
   only via the `frame_id` side-channel, which distinguishes MPC from BT but **cannot
   distinguish the `emergency` Stop from the `handle_obstacle` Stop** — both publish
   `base_link` on the same topic. Bucketing above rests on offline predicate replay,
   not on a direct signal. `/mpc/goal_turn` is also missing from the list.
2. **No MPC solver diagnostics on any topic.** `solve_dt`, `success`, `status`,
   `status_message`, `cost` are all computed at `MPC_corr.py:1317-1340` and thrown at
   the logger. Category 2 was answerable only because `~/.ros/log` happened to still
   hold the files; log rotation would have made it unanswerable. Also: the log line
   is `INFO` and unthrottled at 10 Hz.
3. **Why did run 14-35-50 report COMPLETE?** The behaviour node's per-component log
   (`~/.ros/log/component_supervisor/f1tenth_behavior_behavior_bringup.launch.py.log`)
   is overwritten per supervisor restart and now covers only `1788275882–1788276022`
   — *after* the last run. BT snapshots for every mission window are gone. Without
   them the COMPLETE-without-moving path cannot be traced.
4. **No BT lane/condition telemetry on a topic at all.** Which lane won each tick,
   and which `emergency_condition` child tripped, exists only as text in that
   overwritten log. This is the single highest-value addition: it would turn every
   bucket above from "replayed inference" into "recorded fact".
5. **Three runs have `outcome: null`** (15-00-06, 15-04-45, 15-08-12) — start
   manifest written, stop manifest never was, despite each having a valid
   `metadata.yaml`. The recorder closed cleanly but the terminal transition never
   arrived, so those runs' end states are unknown.
6. **mcap unavailable** — `rosbag2_storage_mcap` is not installed, so all 9 runs fell
   back to sqlite3 exactly as the node's docstring warns.
7. **Recorder discovery latency measured at 3.85–5.43 s**, above the 2.4–3.4 s in the
   node's docstring. Run 0 (5.3 s) recorded nothing at all. Sub-6 s missions are not
   reliably captured.
8. **`/mpc/goal_pose` carried 0 messages in all 9 runs**, though `test_01.json`
   consists of two `goal_pose` moves and one turn. Consistent with the starved-mission
   mechanism in 1b, but worth confirming the pose lane works at all.

---

## Recommended Next Fixes

Ranked by attributable stopped time and by how much each unblocks future analysis.

1. **Debounce `IsSystemOverheated` against load spikes.** 3 of 9 episodes (4.0 s)
   were caused by a 1 Hz `gpu_percent` sample crossing 95 % while the car was in
   clear space (2.65–4.69 m clearance). A momentary YOLO inference burst is not a
   thermal emergency. Options, cheapest first: require N consecutive samples over
   threshold; separate the load limit from the temperature limit (temps never
   exceeded 55 °C against a 100 °C limit — the *thermal* guard never fired and is
   fine); or drop `gpu_percent` from the emergency predicate entirely and keep
   `gpu_temp_c`. Highest ratio of risk removed to effort.
2. **Record `/safety_stop` and `/mpc/goal_turn`; give the two `Stop` instances
   distinct `frame_id`s** (e.g. `base_link/emergency` vs `base_link/obstacle`), and
   publish a BT lane/condition status topic. One line in `_DEFAULT_TOPICS` plus a
   constructor argument makes the whole of Category 1 directly readable instead of
   inferred.
3. **Fix mission-completion accounting (run 14-35-50).** A mission that published no
   goal and never moved must not report COMPLETE — that is worse than an abort,
   because it silently corrupts every downstream success metric. Needs (3) above to
   diagnose, so land the BT telemetry first.
4. **Publish MPC solver diagnostics on a topic** (`solve_dt`, `success`, `status`,
   `cost`). Cheap, and it makes "is the MPC converging?" answerable from a bag alone.
   Consider throttling the 10 Hz INFO line while doing it.
5. **Add temporal smoothing / confidence gating to the semantic layer.** Jitter is
   depth-noise driven and confidence-correlated (low-confidence detections jitter
   2–7× more). Gating or down-weighting sub-0.5-confidence detections in the
   association step should cut the p95 (8–13 cm) substantially. Note the tracker is
   already helping — the fix belongs at its input, not in a new smoothing layer.
6. **Investigate the 35–45 % 3D-detection yield loss.** More impactful for perceived
   marker stability than the position noise: 2D detects at ~13 Hz, 3D emerges at
   ~8 Hz, with gaps up to 6.16 s and heavy track churn (9 tracks for ~2 objects).
   This is also the mechanism that would produce stale-latched-blocker stops — which
   did not fire in these runs but remains live.
7. **Widen the `safety_stop` mux lane timeout or raise the BT tick rate.** 100 ms
   tick against a 200 ms lane timeout is a 2× margin; one late tick actuates an MPC
   command mid-stop. Cheapest correct fix is raising `timeout` in `mux.yaml`.
8. **Install `ros-humble-rosbag2-storage-mcap`** and reduce recorded topic set or
   raise mission length floor — 361 MB for 32 s is dominated by
   `/camera/image_annotated` and `/camera/detection_masks`.

---

## Yield investigation (2026-09-02) — resolved; corrects Category 3, and found a real bug

The 35–45 % 2D→3D yield loss was investigated before the avoidance test, since
`/camera/detections_3d` is the sole input to the MPC avoidance path.

**Live measurement.** ZED 2i had dropped to a USB 2.0 link (enumerated at 480M
on Bus 01; the USB 3.0 bus sat empty) and the SDK refused to open it —
`ZED_Diagnostic -c`: *"Camera not detected. Make sure the camera is plugged in
or try another USB 3.0 port."* After replugging to USB 3.0 (5000M, Bus 02), the
perception stack was run with the same `segment` + `use_mask_depth=true` config
as yesterday's group-B runs:

```
YIELD | det2d_in=137 depth_in=300 mask_in=137 -> synced=137 (100%)
      | published=137 (100%) | rarest input=det2d (ceiling 100% of det2d)
      | frame drops: sync=0 no_camera_info=0 cv_bridge=0 tf2=0
      | detection drops: low_conf=0 bad_depth=0 mask_fallback=0
```

**22 consecutive 10 s windows at 100 % yield** (~14 Hz), zero sync drops, zero
bad-depth drops, zero mask fallbacks. Repeated with `yolo_model_task:=detect`
requested — ultralytics resolves the task from the `.pt` file and loaded
`segment` anyway, so that combination behaves identically (100 %), and the
seg-file/detect-task mismatch is **not** a pipeline hazard.

**Root cause, measured from the bags.** `yolo_detector_node` publishes a mask
image only when `result.masks is not None` (yolo_detector_node.py:525), which
is never true on a frame with zero detections. `detection_3d_node`'s 3-way
`ApproximateTimeSynchronizer` therefore cannot fire on those frames. Splitting
yesterday's frames by whether they contained any detection:

| | frames | of which produced a mask |
|---|---|---|
| **≥ 1 detection** | 1010 | **1002 (99.2 %)** |
| **0 detections** | 1107 | 33 (3.0 %) |

Per-run, the frames that *contained a detection* and still produced nothing:
0/378, 0/1, 0/9, 7/176, 0/20, 0/162, 1/216, 0/48 — **8 of 1010 (0.8 %)**.

So the yield metric was tracking **scene occupancy, not pipeline health**. Run
14-35-50 sat in front of a chair on 100 % of frames and scored 100 % yield; run
15-00-06 had exactly 1 non-empty frame out of 261 and scored 1.5 %. No
meaningful obstacle data was lost in either.

**Consequence for tomorrow: none for avoidance.** Obstacle-bearing frames reach
`/camera/detections_3d` — and therefore `obstacle_projector_node` →
`/perception/obstacles_2d` → `MPC_corr`'s `w_obs` term — at 99.2 % measured
under full-stack load, and 100 % measured live. The premise that a dropped
frame means "the MPC sees nothing for that obstacle on that tick" does not hold:
the dropped frames had no obstacle in them.

### Stale tracks on empty frames — found here, now fixed

`semantic_layer_node` documents a dependency on receiving one
`Detection3DArray` per frame *including empty ones*, because that is what ticks
its miss-streak lifecycle. Empty frames produced no message at all, so when an
object left the scene the frames went empty, no misses were ticked, and **its
track was never aged out**. Markers/costmap only — the MPC avoidance path reads
`obstacles_2d`, which correctly carries nothing when nothing is detected.

**Fix.** `yolo_detector_node` now publishes a **zero-size** mono8 mask alongside
every frame that has no instance masks, so the 3-way synchronizer still fires.
`_publish_masks()` returns a success flag, and its own error early-outs (shape
mismatch, encode failure) fall through to the same empty publish — the invariant
is per-*frame*, so a frame whose real mask could not be built must still emit
something or it becomes another silent gap.

Two platform quirks are pinned by tests, because both are the opposite of the
obvious guess (verified on this box):

- `cv2_to_imgmsg()` on a 0×0 array raises `ZeroDivisionError` computing `step`,
  so the empty message is built by hand rather than through CvBridge.
- `imgmsg_to_cv2(..., desired_encoding=...)` on a 0×0 image **returns `None`
  without raising**, so "no exception" does not imply a usable image;
  `detection_3d_node` guards on `height/width == 0` explicitly and also handles
  a `None` return.

**Verified — track aging.** Replaying the real frame occupancy of three runs
through `update_tracks_batch`, measuring frames between an object's last
sighting and its track being pruned:

| run | frames (empty / non-empty) | longest empty gap | staleness OLD | staleness NEW |
|---|---|---|---|---|
| 15-04-45 | 275 (93 / 182) | 24 | med 5, **max 38** | med 5, max 5 |
| 15-08-12 | 285 (124 / 161) | 91 | med 5, **max 98** | med 5, max 5 |
| 15-11-29 | 268 (218 / 50) | 141 | med 15, **max 43** | med 5, max 5 |

Under the old behaviour a track survived up to **98 frames (~7 s at 14 Hz)**
past its object's last sighting; run 15-11-29 pruned only 3 tracks where the
fixed version prunes 6, and left one alive at end of run that should have gone.
Under the fix, staleness is exactly `lost_miss_count` (5) in every case — no
earlier, no later.

**Verified — no yield regression.** Re-ran the Step-1 accounting live with the
fix in place: 12 consecutive windows at **100 %** (~15 Hz), zero drops in every
bucket. Then forced the empty path by raising `confidence_threshold` to 0.99 so
*every* frame had zero detections:

```
YIELD | det2d_in=208 depth_in=299 mask_in=208 -> synced=208 (100%)
      | published=208 (100%) | frame drops: sync=0 cv_bridge=0 tf2=0
      | detection drops: bad_depth=0 mask_fallback=0 published=0
```

Masks and `Detection3DArray`s published on **every** frame with **zero**
detections in the scene — the exact case that previously yielded 0 % — with no
errors or warnings logged. Regression tests added:
`f1tenth_perception/test/test_empty_mask_invariant.py` (4) and
`TestEmptyFrameAging` in `f1tenth_costmap/test/test_semantic_layer_uncertainty.py` (3).

---

## Follow-up implemented (obstacle-avoidance test config)

The recommendations above were implemented on this branch as the setup for the
next test run. Summary of what changed, keyed to the findings:

| Finding | Change | Where |
|---|---|---|
| GPU load spikes caused 3/9 stop episodes, all false positives | Load trip split from the thermal trip, disabled by default, debounced (N consecutive 1 Hz samples) when enabled. Temperature still trips on a single sample. | `is_system_overheated.py`, `enable_sys_obs_load_trip` |
| Camera corridor stop caused 42.5 s of stopped time and starved the mission subtree | `handle_obstacle` lane gated off by default; camera obstacles now reach the MPC as soft avoidance via the pre-existing `/perception/obstacles_2d` path | `behavior_executor_node.create_root()`, `enable_camera_obstacle_stop` |
| Lidar proximity stops in the band avoidance should own | Thresholds tightened 0.40/0.20 m → 0.15/0.15 m, now direct params rather than derived from `car_radius` + margins; lane kept ON as the floor | `proximity_front_threshold_m`, `proximity_side_threshold_m`, `enable_lidar_safety_stop` |
| Emergency vs obstacle stops indistinguishable in a bag | The two `Stop` instances stamp distinct `frame_id`s (`base_link/emergency`, `base_link/obstacle`) | `create_root()`, `stop.py` |
| `/safety_stop` not recorded (35 % of actuated samples) | Added, along with `/mpc/goal_turn`, `/perception/obstacles_2d`, and the two new topics below | `mission_logger_node._DEFAULT_TOPICS` |
| No MPC solver diagnostics on any topic | New `f1tenth_messages/MpcSolverStatus` published per tick on `/mpc/solver_status` | `MPC_corr.py` |
| No BT lane/condition telemetry | New `f1tenth_messages/BehaviorTreeStatus` published per tick on `/behavior/tree_status` | `behavior_executor_node.make_tree_status_publisher()` |
| Low-confidence detections jitter 2–7× more | Per-detection confidence- and range-scaled position covariance | `detection_3d_node._position_sigma()` |
| Track churn (9 ids for ~2 objects) | Confirmation grace window across missed frames; uncertainty-widened association gate; confidence-weighted EMA | `semantic_layer.py`, `semantic_layer_node.py` |
| 35–45 % 2D→3D yield loss, cause unmeasured | Per-frame yield accounting separating synchronizer drops from each early-return reason, logged every 10 s | `detection_3d_node._report_yield()` |

Two findings from this report are **not** addressed by that work and remain open:

- **Run 14-35-50's false COMPLETE.** Disabling the camera stop removes that
  particular starvation path, but the root Selector can still starve the
  mission subtree via the `emergency` lane, and the accounting bug that let a
  mission report success without ever publishing a goal is untouched. The new
  `/behavior/tree_status` topic is what makes it diagnosable next run.
- **Mux lane flicker** (100 ms BT tick vs 200 ms lane timeout, a 2× margin).

New tests: `f1tenth_behavior/test/test_safety_stop_config.py` (15),
`f1tenth_costmap/test/test_semantic_layer_uncertainty.py` (19),
`f1tenth_perception/test/test_detection_3d_uncertainty.py` (12).

---

### Reproducing

Extraction and analysis scripts:
`/tmp/claude-1004/-home-fabiocar-dev-ws-f1tenth-more/027935f6-1535-46b4-96cd-5c087195c12a/scratchpad/`
(`extract.py`, plus per-run `analysis/*.json`, `*.ack.json`, `*.pred.json`,
`*.sys.json`, `solve.json`, `stop_events_final.json`). All derived from the bags and
`~/.ros/log` only; no live ROS graph needed.
