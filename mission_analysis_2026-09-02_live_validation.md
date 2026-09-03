# Mission Analysis — 2026-09-02, runs 15:13–15:38 UTC (live validation)

Source: `~/.ros/mission_bags/` (13 runs, ~1.70 GB), read offline via `sqlite3` +
`rclpy.serialization`. Supplemented by the per-node ROS logs
`~/.ros/log/python3_<pid>_<epoch>.log` for the MPC's own `CFG` / `GOAL` /
`CORR/build` lines, which are still not published.

This run closes the loop the 2026-09-01 report left open ("confirming on the real
car still needs a live run"). Videos for all 13 runs are in `mission_videos/`,
rendered with `scripts/mission_replay_video.py`.

---

## Headline

**The single most important finding is about the runs themselves, not the car:
these 13 runs are not a validation of `w_psi = 1.0`. They are a `w_psi` sweep.**
The MPC node was restarted 14 times during the session and `w_psi` was edited
between restarts — 1.0, then 5.0, then 2.0, then 5.0, then 10.0. Five runs used
the modeled value; eight did not. `params_snapshot` cannot show this (`w_psi` is
a source constant in `MPC_corr.py`, not a ROS param), and every manifest in the
session is byte-identical.

With that established, the sweep is *more* informative than the flat validation
would have been, and it **refutes the modeled tuning result**:

| `w_psi` | runs | max lateral excursion (m) | horizon bends back toward `psiRef` |
|---|---|---|---|
| 1.0 (modeled choice) | 5 | 0.49 – **2.29** | 50–65 % of deflected ticks |
| 2.0 | 3 | 0.73 – 0.91 | 70–84 % |
| 5.0 | 4 | **0.25 – 0.69** | **82–100 %** |
| 10.0 | 1 | 0.63 | 53 % (regresses) |

The conservatism argument for 1.0 — "leaves room for the model error, solve
latency and detection jitter this idealised loop does not have" — pointed the
wrong way. Real-world effects made 1.0 *under-damped*, not *safe*. On this
evidence the useful range is **2.0–5.0, with 5.0 the best performer**, and the
`1.5` upper bound suggested in `MPC_corr.py`'s comment block is too low.

Secondary: the false-COMPLETE bug **recurred, and is now root-caused** (§8), and
a **new regression appeared** — 7.8 % primal-infeasible QP solves against a
0/1285 baseline (§9).

---

## Method note — what changed since 2026-09-01

Two new topics remove the log archaeology the last analysis needed:

- **`/behavior/tree_status`** — `active_lane`, `lane_names`/`lane_statuses`,
  `emergency_trip`, `safety_stop_active`, `stop_source`. Stop cause is now read
  directly instead of replayed offline.
- **`/mpc/solver_status`** — `success`/`status`/`solve_dt_sec` plus the predicted
  horizon (`pred_x/y/yaw/v`). Convergence and the optimizer's own geometry are
  now bag-answerable.

`/safety_stop` is still recorded as 0 messages, but the mux republish is now
self-describing: `Stop` stamps `header.frame_id = base_link/emergency` (was bare
`base_link`), so the lane is identifiable on `/ackermann_drive` alone.

One caveat on the new topic: **`active_lane` is `''` on 98 % of ticks** (1 235 of
1 259). The field is documented as "the root Selector child that returned
SUCCESS", and a `mission` lane that is `RUNNING` returns neither SUCCESS nor a
name. `lane_statuses` carries the real answer. As shipped, `active_lane` is
close to useless for its stated purpose — see Next steps.

---

## Runs Analyzed

All 13: branch `scene-graph`, commit `580a2af`, **`dirty: true`**. All share
`camera_source=zed`, `confidence_threshold=0.3`, `enable_slam=true`,
`localization_source=ekf`, `enable_nav2=false`, `enable_intelligence=true`,
`use_behavior_tree=true`, `enable_sys_obs=true`, `yolo_model=yolo26s-seg.pt`,
`yolo_model_task=segment`, `use_mask_depth=true`.

| # | run_id (15-…) | mission | outcome | rec | `w_psi` | drive | BT-stop | infeasible | goal m | traveled | displ. | lat max |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | `13-32` | bottle_then_person | ABORTED | 23.8 s | 1.0 | 100 % | 0 % | 36/238 | 7.0 | 5.913 | 5.921 | 0.586 |
| 1 | `15-50` | bottle_then_person | ABORTED | 26.3 s | 1.0 | 76 % | **29 %** | 43/262 | 6.0 | 5.036 | 5.036 | 0.490 |
| 2 | `17-49` | bottle_then_person | ABORTED | 25.2 s | 1.0 | 100 % | 0 % | 14/253 | 6.0 | 4.592 | 4.702 | **2.236** |
| 3 | `19-52` | bottle_then_person | ABORTED | 13.2 s | 1.0 | 96 % | 5 % | 7/133 | 6.0 | 5.564 | 5.710 | 1.282 |
| 4 | `21-57` | bottle_then_person | ABORTED | 13.0 s | 1.0 | 100 % | 0 % | 15/130 | 6.0 | 1.758 | 2.766 | **2.287** |
| 5 | `24-27` | bottle_then_person | ABORTED | 8.6 s | 5.0 | 100 % | 0 % | 0/87 | 6.0 | 4.533 | 4.557 | 0.465 |
| 6 | `26-23` | bottle_then_person | ABORTED | 10.0 s | 5.0 | 100 % | 0 % | 0/96 | 6.0 | 5.404 | 5.409 | 0.254 |
| 7 | `27-43` | bottle_then_person | ABORTED | 9.1 s | 5.0 | 100 % | 0 % | 0/92 | 6.0 | 5.207 | 5.243 | 0.611 |
| 8 | `29-24` | llm_plan_1788362958 | **COMPLETE** | 5.8 s | 2.0 | 98 % | 0 % | 0/58 | 2.0 | 1.804 | 1.946 | 0.730 |
| 9 | `31-05` | llm_plan_1788363065 | **COMPLETE** | 5.4 s | 2.0 | 98 % | 0 % | 0/55 | 2.0 | 1.777 | 2.037 | 0.996 |
| 10 | `34-45` | bottle_then_person | ABORTED | 15.4 s | 2.0 | 100 % | 0 % | 12/154 | 6.0 | 4.611 | 4.700 | 0.909 |
| 11 | `36-16` | bottle_then_person | ABORTED | 8.7 s | 5.0 | 100 % | 0 % | 0/88 | 6.0 | 4.263 | 4.318 | 0.686 |
| 12 | `37-53` | bottle_then_person | *(killed)* | 14.8 s | 10.0 | 100 % | 0 % | 13/144 | 6.0 | 4.761 | 4.768 | 0.633 |

`drive` = share of `/ackermann_drive` samples with non-zero speed. `BT-stop` =
share of `/behavior/tree_status` ticks with `safety_stop_active`. `traveled` is
the MPC's along-line projection, `displ.` the raw Euclidean displacement,
`lat max` the largest lateral offset from `psiRef` during the move.

Every `bottle_then_person` run is ABORTED on the 60 s move timeout or an operator
kill; none of them reached the 6.0 m goal. That is the mission's design (one
`goal_distance: 6.0` move, `on_timeout: abort`) plus a short test space, not a
failure signal on its own.

### Like-for-like warning — this is the important one

`params_snapshot` md5 is **identical across all 13 runs, and identical to the
2026-09-02 morning runs**. That is not evidence the config was constant. The
snapshot is a dump of launch-argument *declarations*; the parameters that
actually varied this session live elsewhere:

- `w_psi` — a literal in `MPC_corr.py`, recoverable only from the node's own
  `CFG | weights=...` log line at startup.
- `enable_camera_obstacle_stop` — a launch **override**, invisible in the
  snapshot (which records only the declared default).

Reconstructed from the 14 MPC startups in `~/.ros/log`:

| MPC start (UTC) | `w_psi` | covers run |
|---|---|---|
| 15:12:34 | 1.0 | `15-13-32` |
| 15:14:39 | 1.0 | `15-15-50` |
| 15:17:12 | 1.0 | `15-17-49` |
| 15:19:17 | 1.0 | `15-19-52` |
| 15:20:56 | 1.0 | `15-21-57` |
| 15:23:44 | 5.0 | `15-24-27` |
| 15:25:27 | 5.0 | `15-26-23` |
| 15:27:02 | 5.0 | `15-27-43` |
| 15:28:43 | 2.0 | `15-29-24` |
| 15:30:32 | 2.0 | `15-31-05` |
| 15:32:43 | 2.0 | *(no bag)* |
| 15:33:45 | 2.0 | `15-34-45` |
| 15:35:36 | 5.0 | `15-36-16` |
| 15:37:03 | **10.0** | `15-37-53` |

**Do not pool these runs.** Compare within a `w_psi` group only, and treat any
cross-group comparison as confounded by obstacle placement as well (see §3).

Against the 2026-09-01 baseline: that session was a different `params_snapshot`
(two groups, md5 `71b706…` and `114bb8…`), so no direct comparison is valid
except where a metric is config-independent — the `detections_3d` yield (§5) and
the solver-convergence rate (§9), both flagged as such below.

### Source-tree hygiene note

`MPC_corr.py` was edited again at 15:38 UTC, one minute after the last run
started. The tree now reads `"w_psi": 5.0`, while the 60-line comment block
directly above it still argues for and documents `1.0` as the chosen value. Code
and rationale disagree; neither matches the last run (10.0).

---

## Validation checklist

### 1 — Camera e-stop off: does the soft-cost path actually avoid?

**Confirmed off. Avoidance confirmed working. One residual risk found, and it is
not the one the sequencing plan anticipated.**

`enable_camera_obstacle_stop:=false` is directly verifiable from the bag:
`/behavior/tree_status.lane_names` is `['emergency', 'mission', 'navigation']` in
**all 13 runs** — `handle_obstacle` is structurally absent, exactly as the
parameter's docstring promises ("the lane is structurally absent when false, not
ticked-and-failing"). This is a clean win for the new topic: the distinction
"disabled" vs "not tripping" is now one field lookup.

The soft-cost path did the work. `/perception/obstacles_2d` carried 1–3 obstacles
(radii 0.05–1.49 m) through every run, and the car deflected around them: lateral
excursions from 0.25 m to 2.29 m, correlated tick-for-tick with obstacle presence
in the video overlays. **Zero collisions across 13 runs**, with the camera stop
lane not present at all.

**But the two closest approaches were not soft-cost failures — they were camera
blindness.** At both lidar-floor trips (§2), `/camera/detections_3d` reported
**0 detections** and `/perception/obstacles_2d` was **empty**:

```
15-15-50  t+16.02  DET3D n=0   obstacles_2d []   boundaries n=3
15-19-52  t+10.51  DET3D n=0   obstacles_2d []   boundaries n=3
```

The near returns were at −35° to −100° — the front-right shoulder and right
side, i.e. structure the car was scraping past, not a classified object ahead.
`/costmap/boundaries` *did* carry it (offsets 0.101 m and 0.170–0.229 m), but
after the documented tightening (`offset − car_radius − avoidance_margin` =
`offset − 0.32`) those halfspaces were already violated, which is precisely where
the infeasible solves cluster (§9).

So the accounting is: with camera e-stop off, the class of thing that can still
reach 0.11 m is **unclassified geometry the YOLO path never reports**, and the
only mechanisms left are the hard boundary constraints (which went infeasible)
and the 0.15 m lidar floor (which fired). The sequencing plan's premise — that
`enable_lidar_safety_stop` is the floor catching avoidance failures — held, but
the failure it caught was in the lidar/boundary path, not the camera soft-cost
path.

### 2 — Lidar floor at 0.15 m: did it trip, and was it right?

**Tripped in 2 of 13 runs. Both genuine. No false positives.**

| run | trip at | duration | min range | character |
|---|---|---|---|---|
| `15-15-50` | t+18.40 | 7.40 s (61 ticks) | **0.107 m** | one sustained 7.19 s sub-threshold episode |
| `15-19-52` | t+12.22, t+12.74 | 0.11 s, 0.29 s (5 ticks) | 0.146 m | four brief dips, all 0.146–0.149 m |

In `15-15-50` the scan crossed 0.15 m at t+18.14 and the BT tripped at t+18.40 —
**0.26 s latency**, consistent with the 10 Hz tick against a ~40 Hz scan. The car
was at 5.036 m of a 6.0 m goal with a lateral offset of 0.006 m (dead on the
reference line) and had been driving straight into structure on its right for
seven seconds. This is the floor doing exactly its job.

`15-19-52` is marginal by design: every dip bottomed at 0.146–0.149 m, within
3 mm of the threshold. It fired, but on the noise floor.

**One near-miss of the trip:** `15-34-45` had four sub-0.15 m scan frames
(0.148–0.149 m) and **no trip**. All four episodes lasted 0.00–0.05 s — single
scans between BT ticks. This is not a missed avoidance failure; it is
threshold-adjacent sensor noise that the 10 Hz latching tick never sampled. Worth
knowing, not worth acting on: raising the threshold to catch it would re-enter
the 0.15–0.40 m band the tightening deliberately handed to MPC avoidance.

Runs `15-26-23` and `15-36-16` had 21 and 35 frames below 0.20 m with no trip and
no incident — the tightened threshold correctly declining to compete with
avoidance in that band. That is the change working as intended.

### 3 — `w_psi = 1.0` direction recovery: real vs modeled

**Refuted. 1.0 does not reproduce its modeled behaviour on the real car.**

Because the session was a sweep, the comparison is richer than planned. Two
metrics, both computed from bag data only.

**(a) Settle distance** — distance travelled past the obstacle before
`|yaw − psiRef| < 0.05 rad` and stays, the same definition the modeled table
used. `psiRef` is taken from `/mpc/corridor_markers`' centerline far-end tangent
(§Step-1 check below confirms this equals the MPC's own `psiEnd`).

| run | `w_psi` | obstacle r (m) | settle after obstacle | modeled |
|---|---|---|---|---|
| `15-13-32` | 1.0 | 0.09 | 1.43 m | 1.34 m |
| `15-15-50` | 1.0 | 0.08–0.09 | **never settles** | 1.34 m |
| `15-17-49` | 1.0 | 0.60–0.64 | **7.20 m** | 1.34 m |
| `15-21-57` | 1.0 | 0.69 | **never settles** | 1.34 m |
| `15-24-27` | 5.0 | 0.09 | 1.47 m | – |
| `15-26-23` | 5.0 | 0.09 | 1.36 m | – |
| `15-31-05` | 2.0 | 0.17 | 1.86 m | – |
| `15-37-53` | 10.0 | 0.47 | 1.05 m | – |

The modeled 1.34 m is reproduced at `w_psi = 1.0` **only against a bottle-sized
obstacle** (r ≈ 0.09, run `15-13-32`: 1.43 m, within 7 % of model). Against a
person-sized obstacle (r ≈ 0.6–0.7) it degrades to 7.20 m or fails to settle at
all within the run. The modeled sweep used r = 0.15 and r = 0.35; **the real
obstacle set was up to 4× larger than either**, and that is where 1.0 breaks.

**(b) Horizon bend-back** — the observable version, and the one that is not
confounded by which obstacle happened to be in the way, because it is measured
per solve over 1 790 solves. For every tick with a deflection (`|yaw − psiRef| >
0.05`), does the *predicted horizon's* final yaw sit closer to `psiRef` than the
car's current yaw?

| run | `w_psi` | deflected ticks | bends back | median `|e₀|` | median `|e_N|` |
|---|---|---|---|---|---|
| `15-13-32` | 1.0 | 199 | 65.3 % | 0.621 | 0.457 |
| `15-15-50` | 1.0 | 238 | 53.4 % | 0.815 | 0.753 |
| `15-17-49` | 1.0 | 230 | 60.0 % | 1.010 | 0.861 |
| `15-19-52` | 1.0 | 79 | 63.3 % | 0.305 | 0.180 |
| `15-21-57` | 1.0 | 129 | 50.4 % | 1.557 | 1.334 |
| `15-29-24` | 2.0 | 56 | 83.9 % | 0.476 | 0.260 |
| `15-31-05` | 2.0 | 55 | 80.0 % | 0.471 | 0.258 |
| `15-34-45` | 2.0 | 111 | 70.3 % | 0.253 | 0.271 |
| `15-24-27` | 5.0 | 39 | **87.2 %** | 0.234 | **0.075** |
| `15-26-23` | 5.0 | 38 | **81.6 %** | 0.138 | **0.055** |
| `15-27-43` | 5.0 | 22 | **100.0 %** | 0.171 | **0.031** |
| `15-36-16` | 5.0 | 37 | **83.8 %** | 0.179 | **0.086** |
| `15-37-53` | 10.0 | 78 | 52.6 % | 0.396 | 0.423 |

At `w_psi = 1.0` the horizon bends back barely more often than a coin flip and
the median residual yaw error at the horizon end is 0.18–1.33 rad — **the
horizon is frequently staying parallel-offset, which is the exact failure mode
the Step-1 check was written to detect.** At 5.0 it bends back 82–100 % of the
time and lands within 0.03–0.09 rad of `psiRef` inside the 1 s horizon.

At `w_psi = 10.0` the metric regresses (52.6 %, and `|e_N| > |e₀|`), alongside 13
infeasible solves. One run, one obstacle set — suggestive that 10.0 is past the
useful range, not conclusive.

**Verdict on the original question** — "was that margin actually needed or overly
cautious": neither. The margin was chosen against the wrong risk. Model error,
solve latency and detection jitter did not require a *softer* weight; the real
obstacle sizes required a *stiffer* one. The clearance cost the modeled table
warned about (0.143 m at 1.0 → 0.093 m at 4.0) did not materialise as contact:
zero collisions at 5.0, and the two closest approaches in the whole session both
happened at `w_psi = 1.0`.

**Confound, stated plainly:** obstacle placement was not controlled across the
sweep. The `w_psi = 1.0` group happened to face the largest obstacles (r up to
1.49 in `15-21-57`), the 5.0 group the smallest (r ≤ 0.55 in `15-24-27`). Metric
(b) is per-solve and far less sensitive to this than metric (a), and both point
the same way, but a controlled re-run is the honest next step (see Next steps).

### 4 — Termination-check divergence: `traveled` vs raw displacement

**Confirmed, and substantially larger in the field than in the replayed leg.**

The single replayed leg showed 1.923 vs 2.056 — a 0.133 m divergence. Across the
runs with a real obstacle deflection:

| run | `w_psi` | traveled (along-line) | displacement | **divergence** | max divergence in run |
|---|---|---|---|---|---|
| `15-21-57` | 1.0 | 1.758 | 2.766 | **1.008 m** | 1.246 m |
| `15-31-05` | 2.0 | 1.777 | 2.037 | 0.260 m | 0.309 m |
| `15-19-52` | 1.0 | 5.564 | 5.710 | 0.146 m | 0.267 m |
| `15-29-24` | 2.0 | 1.804 | 1.946 | 0.142 m | 0.169 m |
| `15-17-49` | 1.0 | 4.592 | 4.702 | 0.110 m | 0.928 m |
| `15-34-45` | 2.0 | 4.611 | 4.700 | 0.089 m | 0.149 m |
| `15-36-16` | 5.0 | 4.263 | 4.318 | 0.055 m | 0.114 m |
| `15-27-43` | 5.0 | 5.207 | 5.243 | 0.036 m | 0.082 m |
| `15-26-23` | 5.0 | 5.404 | 5.409 | 0.006 m | 0.008 m |

The replayed 0.133 m was near the *low* end. `15-21-57` diverges by **1.008 m on
a 6 m move — 17 % of the commanded distance**. The divergence tracks `w_psi`
inversely, as it should: it is the integral of the lateral excursion, so the
under-damped runs pay for it directly.

This is not an academic distinction. §8 shows it terminating missions early.

### 5 — `detections_3d` yield

**Confirmed. No regression. The ZED USB fault has not recurred.**

Yield measured per-frame by pairing `/camera/detections` and
`/camera/detections_3d` on identical header stamps, over frames where the 2D
stage produced at least one detection:

| run | paired frames | frames w/ 2D dets | yield | frames fully lifted |
|---|---|---|---|---|
| `15-13-32` | 374 | 26 | 100.0 % | 26/26 |
| `15-15-50` | 386 | 58 | 98.6 % | 57/58 |
| `15-17-49` | 295 | 213 | 97.2 % | 206/213 |
| `15-19-52` | 182 | 9 | 100.0 % | 9/9 |
| `15-21-57` | 163 | 132 | 96.9 % | 126/132 |
| `15-24-27` | 120 | 4 | 100.0 % | 4/4 |
| `15-26-23` | 149 | 10 | 100.0 % | 10/10 |
| `15-27-43` | 130 | 5 | 100.0 % | 5/5 |
| `15-29-24` | 71 | 62 | 100.0 % | 62/62 |
| `15-31-05` | 68 | 62 | 100.0 % | 62/62 |
| `15-34-45` | 215 | 2 | 100.0 % | 2/2 |
| `15-36-16` | 127 | 6 | 100.0 % | 6/6 |
| `15-37-53` | 212 | 55 | 95.5 % | 52/55 |

Aggregate **98.6 %** (796 of 807 detections lifted). The three runs below 100 %
are the three with the heaviest detection load (213, 132, 55 frames with
detections) — consistent with occasional depth-lookup misses on cluttered
frames, not with a link fault. `/camera/detections_3d` message counts track
`/camera/detections` to within ±5 in every run, so nothing is being dropped
wholesale.

### 6 — Stale-track aging

**Confirmed, with direct evidence.**

`/costmap/semantic_markers` now emits explicit `action=2` (DELETE) markers.
Run `15-13-32` in full:

```
t+ 0.00   ADD    bottle#1
t+ 0.68   DELETE bottle#1      ← object left the scene
t+ 0.77   (array empty)
t+13.94   ADD    person#7
t+14.19   ADD    person#8
t+14.43   DELETE person#7
t+14.55   DELETE person#8
t+14.60   (array empty)
```

38 ADDs and 6 DELETEs in that run. The `bottle` track is deleted 0.68 s after
first sight and does **not** reappear for the remaining 23.11 s of the run — the
persistence that previously left a phantom marker in the costmap is gone. Across
all 13 runs, 68 of 82 tracks age out with more than 1 s of run remaining, and no
track survives past its object's disappearance.

### 7 — GPU load debounce

**No false-positive overheat stops. But the debounce itself is untested.**

`IsSystemOverheated` did not trip once across all 13 runs (0 of 1 259
`/behavior/tree_status` ticks). Against the 2026-09-01 baseline of 3 episodes /
4.0 s of stopped time from this cause, that is the failure mode gone.

**The caveat is important.** `params_snapshot` shows
`enable_sys_obs_load_trip = False` — the load half was **disabled outright**, not
merely debounced. `sys_obs_load_trip_consecutive_samples = 3` is configured but
was never exercised, because the code path it guards was switched off. So:

- "no false positives from momentary load spikes" — **confirmed**, but trivially:
  the lane could not fire.
- "the 3-sample debounce correctly rejects 1–2 sample spikes while admitting
  sustained load" — **untested**.
- "real thermal stops still fire" — **untested**. No thermal event occurred; the
  24 `/diagnostics/system_status` samples per run never approached the 100 °C
  limit, same as 2026-09-01.

The temperature path is unchanged code and was never the false-positive source,
so the risk here is low. But this checklist item cannot be marked green on this
evidence.

### 8 — False-COMPLETE bug — **recurred, and now root-caused**

The bug is back, and `BehaviorTreeStatus` plus `/mission/status` made it a
five-minute find instead of log archaeology.

Both `llm_plan` runs report `outcome: COMPLETE`. Neither finished its last move:

| run | final move | goal | MPC `traveled` | MPC `residuo` | raw displacement | `/mission/status` |
|---|---|---|---|---|---|---|
| `15-29-24` | `move_2` | 2.0 m | **1.804 m** | +0.196 m | 1.946 → >2.0 | COMPLETE @ t+5.47 |
| `15-31-05` | `move_2` | 2.0 m | **1.777 m** | +0.224 m | 2.037 | COMPLETE @ t+5.47 |

Neither log contains a `Goal raggiunto` line — the MPC never considered the move
finished. The BT declared it finished anyway.

**Root cause,** [condition_eval.py:131-135](src/f1tenth_behavior/f1tenth_behavior/mission/condition_eval.py#L131-L135):

```python
traveled = math.hypot(
    ctx.current_xy[0] - ctx.move_start_xy[0],
    ctx.current_xy[1] - ctx.move_start_xy[1],
)
return traveled >= float(target)
```

`distance_reached` uses **raw Euclidean displacement**. `MPC_corr` terminates on
the **along-line projection** ([MPC_corr.py:1275](src/f1tenth_control/mpc_controller/mpc_controller/MPC_corr.py#L1275),
`_project_onto_line`). The two disagree by exactly the divergence measured in
§4 — and in `15-31-05` that divergence is 0.260 m against a 2.0 m goal, so the BT
fires **13 % early**. The two checklist items are the same bug seen from two
ends: §4 measures the disagreement, §8 is what the disagreement costs.

This also explains why the bug is intermittent and why it did not show up on
straight runs: with no deflection the two measures agree to within millimetres
(`15-26-23`: 0.006 m). It needs a lateral excursion to manifest — which is why it
appeared on the `llm_plan` missions, whose `move_2` follows a 74.5° turn.

The parameter plumbing is fine: `p.get('distance', ctx.default_distance)` reads
the schema-2.0 `"distance"` key correctly. The defect is purely the metric.

### 9 — NEW: primal-infeasible QP solves (regression vs baseline)

Not on the checklist, but it is the largest new signal in the data.

**140 of 1 790 solves (7.8 %) returned `primal infeasible` or `primal infeasible
inaccurate`**, against the 2026-09-01 baseline of **0 of 1 285**.

| run | `w_psi` | infeasible | `n_boundary` / `n_obstacles` at failure |
|---|---|---|---|
| `15-13-32` | 1.0 | 36/238 | 3/0 (25), 2/0 (5), 3/1 (5) |
| `15-15-50` | 1.0 | 43/262 | 3/0 (39), 2/0 (4) |
| `15-17-49` | 1.0 | 14/253 | 3/1 (11), 3/2 (2) |
| `15-19-52` | 1.0 | 7/133 | 3/0 (7) |
| `15-21-57` | 1.0 | 15/130 | 3/0 (10), 3/1 (5) |
| `15-34-45` | 2.0 | 12/154 | 3/0 (12) |
| `15-37-53` | 10.0 | 13/144 | 3/0 (9), 3/1 (3) |
| all 5.0 runs + `15-29-24`, `15-31-05` | 5.0 / 2.0 | **0/476** | – |

Two things stand out. **It is the boundary constraints, not the obstacles**: 128
of 140 failures occurred with `n_obstacles = 0` and `n_boundary_constraints = 3`.
Given the tightening (`offset − 0.32 m`) and the observed offsets at the near-wall
episodes (0.101 m, 0.170 m), three simultaneously-tightened halfspaces with the
car already inside one of them has no feasible point — the QP is correctly
reporting an over-constrained geometry, not misbehaving.

**And it is not caused by `w_psi`**: failures appear at 1.0, 2.0 and 10.0 and are
absent at 5.0, which is not monotone. The zero-failure runs are also the short,
open ones (8.6–10.0 s, all ending well clear of walls). The likely driver is the
test space being tighter today than on 2026-09-01, not a code regression.

Two caveats before treating this as a hard regression: the 2026-09-01 count came
from log parsing rather than a published topic, and the two sessions ran
different `params_snapshot` configurations in a different physical space. It is
nonetheless worth chasing, because an infeasible solve is what preceded both
near-misses in §1–2.

Reassuringly, **zero missed control deadlines** in all 13 runs — max
`solve_dt_sec` 0.049 s against a 0.100 s budget, and the `rti` backend on every
one of 1 790 solves (no silent SLSQP fallback).

---

## Step-1 video checks

Both contract checks were run against the rendered videos and confirmed
numerically from the same bags.

**Corridor far end emits a straight-line-in-a-direction reference — CONFIRMED.**
`CORR/build` logs `psiStart` (live yaw, changing every tick) against `psiEnd`
(the direction captured at move start). Over run `15-37-53`:

```
psiStart=+0.0031  psiEnd=+0.0030
psiStart=-0.6604  psiEnd=+0.0030
psiStart=-0.2830  psiEnd=+0.0030
psiStart=+2.4820  psiEnd=+0.0030
```

`psiEnd` is constant to 4 decimals for the whole move while `psiStart` swings
2.5 rad, and `Pend` is recomputed from the live pose every tick. The
`psiRef` extracted independently from `/mpc/corridor_markers` has **standard
deviation 0.0000 rad** in every straight-move run. This is the voided-lateral-
pinning contract holding: a direction, not a fixed absolute line. The two
`llm_plan` runs show `psiRef` sweeping smoothly (−3.06 → 0.00 rad across 27
distinct values) through the commanded 74.5° turn, then re-latching — the turn
path re-captures the reference rather than inheriting a stale one.

**Predicted horizon bends back toward `psiRef` after deflection — CONFIRMED at
`w_psi ≥ 2.0`, NOT confirmed at 1.0.** See the table in §3(b). At 1.0 the horizon
stays parallel-offset on 35–50 % of deflected ticks. This is the regression the
check was designed to catch, and it is present — in the five runs that used the
value the last report recommended.

Videos (`mission_videos/`, one MP4 per run, established naming):

```
2026-09-02T15-13-32_mission-bottle_then_person.mp4     2026-09-02T15-27-43_…
2026-09-02T15-15-50_mission-bottle_then_person.mp4     2026-09-02T15-29-24_mission-llm_plan_1788362958.mp4
2026-09-02T15-17-49_mission-bottle_then_person.mp4     2026-09-02T15-31-05_mission-llm_plan_1788363065.mp4
2026-09-02T15-19-52_mission-bottle_then_person.mp4     2026-09-02T15-34-45_…
2026-09-02T15-21-57_mission-bottle_then_person.mp4     2026-09-02T15-36-16_…
2026-09-02T15-24-27_mission-bottle_then_person.mp4     2026-09-02T15-37-53_…
2026-09-02T15-26-23_mission-bottle_then_person.mp4
```

All 13 carry a predicted horizon on 100 % of solves and real corridor geometry
from the published markers.

---

## Scoreboard

| # | change | verdict |
|---|---|---|
| 1 | camera e-stop off, soft-cost avoidance | **Confirmed** — lane structurally absent, 0 collisions; residual risk is camera blindness, not soft-cost weakness |
| 2 | lidar floor at 0.15 m | **Confirmed** — 2 genuine trips, 0 false positives; 4 sub-tick dips missed (benign) |
| 3 | `w_psi = 1.0` | **Refuted** — modeled 1.34 m holds only for r≈0.09 obstacles; 5.0 clearly better |
| 4 | along-line vs raw termination metric | **Confirmed** — divergence up to 1.008 m, 7.5× the replayed example |
| 5 | `detections_3d` yield | **Confirmed** — 98.6 % aggregate, no regression |
| 6 | stale-track aging | **Confirmed** — explicit DELETE markers, no persistence |
| 7 | GPU load debounce | **Partially confirmed** — 0 false positives, but load lane was disabled outright; debounce and thermal path both untested |
| 8 | false-COMPLETE (was open) | **Recurred — now root-caused** at `condition_eval.py:131` |
| 9 | *(new)* QP feasibility | **Regression** — 7.8 % infeasible vs 0 % baseline |

---

## Open gaps

- **`/safety_stop` still records 0 messages.** Attribution now works via
  `frame_id = base_link/emergency` and `/behavior/tree_status`, so this is no
  longer blocking, but the topic in `mission_logger_node`'s list is dead weight.
- **`active_lane` is `''` on 98 % of ticks** — see Method note.
- **`w_psi` is invisible to the run manifest.** It is a source literal; a run's
  configuration is not reconstructible without the node's startup log, which
  survives only until log rotation. This session came within a rotation of being
  uninterpretable.
- **Obstacle placement was not controlled** across the `w_psi` sweep (§3).
- **No run reached its goal.** Every `bottle_then_person` run aborted on timeout;
  the 6.0 m move never completed in a space that supports ~5 m. Goal-completion
  behaviour at `w_psi ≥ 2.0` is therefore unobserved.
- **`enable_sys_obs_load_trip = False`** leaves §7 half-tested.

---

## Ranked next steps

1. **Fix the false-COMPLETE properly** — change `distance_reached` in
   [condition_eval.py:131](src/f1tenth_behavior/f1tenth_behavior/mission/condition_eval.py#L131)
   to the along-line projection, or better, have it consume `/mpc/goal_reached`
   so there is exactly one definition of "arrived" in the stack. This is a real
   bug with a measured 13 % early termination and a known one-line cause.
2. **Set `w_psi` to 5.0 and re-run a controlled comparison** — the sweep says 5.0,
   the source tree already says 5.0, and the comment block still says 1.0. Fix
   the comment to match the field data. Then re-run 1.0 vs 5.0 with the *same*
   obstacle in the *same* place, three runs each, to remove the §3 confound.
3. **Chase the 7.8 % infeasible solves** — instrument which halfspace is
   infeasible and by how much. If it is the tightening margin against a tight
   space, the fix is a soft/slack constraint rather than a hard one, so a wall
   at 0.10 m degrades the solution instead of destroying it. Both near-misses in
   §1–2 were preceded by infeasible solves.
4. **Publish `w_psi` (and the rest of `self.weights`) on `MpcSolverStatus`**, or
   promote them to ROS params so they land in `params_snapshot`. One field would
   have made this entire report's headline caveat unnecessary.
5. **Re-enable `enable_sys_obs_load_trip` with the 3-sample debounce** and run
   long enough to see a real GPU spike, to close §7 honestly.
6. **Run `bottle_then_person` in a space that fits it**, or shorten the move to
   4.0 m, so at least one run reaches `Goal raggiunto` and the completion path is
   exercised end-to-end.
7. **Fix `active_lane`** to report the RUNNING child's name, not only a SUCCESS
   child's.
