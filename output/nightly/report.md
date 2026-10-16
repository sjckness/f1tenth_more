# Overnight session — branch `campaign-final`, Orin, OFFLINE

Battery disconnected, ZED and Hokuyo absent. **No sensor driver and no command
path was launched at any point.** Everything below is the two recordings:
`corridor10m_20261015T171753` and the concurrent archive run
`2026-10-15T15-18-27_mission-llm_86cff518e42d`. 02:50 - 05:10,
`55e5f25` -> `fdd65e0`, 8 local commits, nothing pushed.

---

## MORNING SUMMARY

| point | state | the one line |
|---|---|---|
| **F2** | green | the aliased header falsified **nothing** in the column cross-check (six buckets bit-identical pre/post) but made `/perception/front_distance` **0.229 m short on 67.6 % of frames** of every run since the StereoColumns publish existed |
| **F1** | green | past 8 m the stereo is not evidence: the 15.85 m end wall is UNVERIFIED and FRONT again on 90/106 cycles, glass still VERIFIED glass, right walls VERIFIED, **nothing rejected anywhere** |
| **F3** | amber | tracker 46.76 -> **44.20 ms p50** / 53.63 p95 after three passes. Target was 43.59, so **0.61 ms over**, and I stopped there rather than take a riskier cut at 04:00 |
| **F4** | green | `ROLE_VALUE['other'] = 50`; nothing in the control path subscribes to that grid, its only runtime consumer is the recorder |
| **F6** | green | recording is opt-in, and the supervisor had to learn to forward the flag or the documented campaign command would have been **silently ignored** |
| **F5** | green | the second recorder was **typed by hand** -- nothing in the repo launches it -- so the conditional toggle does not apply. Both wrote ~5 MB/s, at the same time, 93 %/88 % of it images and grids |
| **G** | amber | intensity is populated and usable, but this run has **one** glass track after windowing and **zero** beams within 5° of any surface normal, so the headline feature is unmeasurable here |

**The three that need you.** (1) F1 moved the end wall from REJECTED to
UNVERIFIED, which is right, but the replay's WALL time is 22.51 s against the
recorded 35.91 s and the rest is the **publish cap on a cold tracker** -- 36
cycles still have a confirmed front wall hidden at rank 4. That cap is the next
real defect, not E2. (2) F3 is 0.61 ms over target and the tracker is still
44 ms a scan against a 25 ms scan period: the stack is **CPU-bound, not
accuracy-bound**, and that is the ceiling on everything. (3) G needs the run
described at the end of `output/glass_lidar/report.md` before any of it is
worth wiring; one glass track decides nothing.

**Decisions I took alone:** 5, all in `output/nightly/decisions.md`. The one
worth reading is F6's -- the toggle needed a change in `component_supervisor_node`
to work at all, because the supervisor forwards only `components.yaml`'s args.

---

## F2 — the header-aliasing bug: how much did it falsify?

Full detail and the live evidence in
[f2_header_aliasing.md](f2_header_aliasing.md). Measured from a temporary
worktree at `c44b503` (= `d355bbe^`) with `f1tenth_perception` and
`corridor_perception` rebuilt there and overlaid; the branch was not touched and
the worktree is removed.

### Column DISAGREE rate, before the fix vs after — bit-identical

| range [m] | columns with both | disagree | rate | resid med [m] | pre == post |
|---|---|---|---|---|---|
| 0–1 | 51 | 22 | 0.4314 | 0.041 | **yes** |
| 1–2 | 10 570 | 7 771 | 0.7352 | 0.2099 | **yes** |
| 2–3 | 6 192 | 5 914 | 0.9551 | 0.5848 | **yes** |
| 3–5 | 6 825 | 4 595 | 0.6733 | 0.4061 | **yes** |
| 5–8 | 7 147 | 3 957 | 0.5537 | 0.6671 | **yes** |
| 8+ | 6 037 | 4 689 | 0.7767 | 2.5499 | **yes** |

Every count, rate and residual identical over 146 cycles. `camera_frame` is
copied out of the header **before** the rewrite and the mask tick takes its own
transform at the top, so the columns, the class mask and the WALL cloud were all
computed with the right extrinsic. All 145 recorded `StereoColumns` messages
carry `camera_frame = zed2_left_camera_optical_frame`. No column tolerance was
touched.

### Contaminated consumers — all of them inside `front_clearance_node`

| consumer | what it got | consequence |
|---|---|---|
| `_front_offset(msg.header.frame_id)` | pose of `base_link` in `base_link` = 0, so offset 0.405 instead of 0.1756 | **`/perception/front_distance` 0.2294 m SHORT** on 163/241 frames (67.6 %) of the raw bag and 42/71 of the archive bag. Measured on the recording: median 2.851 m on contaminated frames against 3.080 m on clean ones. The mission wall guard reads this topic |
| `_ground_roi_mask(depth, frame_id, ...)` | identity rotation | the ROI ground-plane mask fitted as if the camera were axis-aligned with `base_link` |
| `_label_scene(depth, header, ...)` | identity rotation | the old scene_labels grid, `stereo_wall_distance`, `/perception/stereo_wall`. Negligible on this run: 133 messages, almost all with a null distance |
| `/perception/scene_labels` (Image) | `frame_id: base_link` on all 143 | a debug image in the wrong frame |
| `_publish_too_close(..., frame_id)` | `base_link` | a report field only |
| the class mask (E2) | identity camera transform | 18 245 samples, **0 in the image**, coverage 0.000 on every track — and indistinguishable from a wall the camera cannot see |

In the replay the same path fails harder (`/tf_static` has no
`base_link -> base_link` edge, so the pre-fix node published the no-reading
sentinel −1.0 on all 155 frames where the fixed one publishes 1.50 m). Live it
got 0.405 instead of nothing — a wrong number rather than no number, which is
the worse of the two.

---

## F1 — E2: stereo range and tolerance

Three changes: samples past `wall_rect_max_range_m` (8.0 m) are **not
informative**; the tolerance is the columns' own `base + quad·d²` with
`wall_obs_tol_base_m` and `wall_obs_tol_quad_per_m2` **themselves** (6 cm at
1 m, 82 cm at 8 m, against the old 15/40 cm) and the two rect-only tolerance
parameters are gone; REJECTED now needs `wall_rect_reject_cycles` (3)
consecutive **verifiable** cycles while VERIFIED stays immediate.

### The tracks the point asked about

| id | ρ_bl [m] | role | verify | glass cyc | FRONT cyc | cov max | expected | got |
|---|---|---|---|---|---|---|---|---|
| **44** (end wall) | 15.85 | front | **unverified** | 0 | **90/106** | 0.00 | UNVERIFIED | ✔ |
| **84** | 11.42 | other | **unverified** | 0 | 0 | 0.00 | UNVERIFIED | ✔ |
| **106** | 11.41 | other | **unverified** | 0 | 0 | 0.00 | UNVERIFIED | ✔ |
| 73, 109 | 13.5 | other | unverified | 0 | 0 | 0.00 | — | also past 8 m |
| **2, 10, 30, 64, 97** (left glass) | 1.9–2.2 | **left** | **verified** | **all** | 0 | 0.49–0.88 | VERIFIED glass | ✔ |
| **87, 113, 124** (right) | 1.28–1.31 | **right** | **verified** | 0 | 0 | 0.87–0.93 | VERIFIED | ✔ |
| 3, 4, 43, 102 (the ~2 m family) | 1.5–2.1 | other | unverified | 0 | 0 | 0–0.13 | not reference walls | ✔ |

**No track is REJECTED anywhere in the run** — the columns/rectangle contingency
has no `rect_reject` cell at all, against 90 rejection cycles before F1.

### Contract, and the error where it does report a wall

| state | recorded [s] | replay [s] | before F1 [s] |
|---|---|---|---|
| clear | 38.88 | 51.91 | 68.47 |
| wall | **35.91** | **22.51** | 5.95 |
| no_data | 0.37 | — | — |

End-wall error against the true strip range, replay: **median 0.010 m**, p95
0.019, max 0.148 — the recorded run's own figures are 0.011 / 0.019 / 0.151. So
when it reports a wall it is right to a centimetre; what is missing is 13 s of
WALL, and that is the **publish cap on a cold tracker**: the recorded run's
FRONT track was 435 s old with a hit count to match, and 36 replay cycles still
have a confirmed front wall the cap hides at rank 4 (median distance 15.48 m,
median length 3.13 m, 7 tracks held against a cap of 4). Not E2.

---

## F3 — E3 performance

Per-stage, during the replay, payload **off** (the car's configuration):

| stage | p50 [ms] | p95 [ms] |
|---|---|---|
| `verify_rectangles` (E2) | 2.75 | 4.42 |
| scene mask classify | 2.29 | 2.36 |
| inferred floor | 1.89 | 1.94 |
| mask tick (node) | 25.24 | 27.40 |
| depth callback | 62.50 | 65.52 |
| **whole tracker cycle** | **44.20** | **53.63** |
| scan callback | 51.13 | 61.81 |
| — pre-E baseline, same bag | 38.59 | 47.69 |
| — recorded, live on the car | 169.07 | 233.87 |

**44.20 against a 43.59 target: 0.61 ms over.** Progress: 49.79 (first E3
commit) -> 46.76 (F1) -> 44.93 -> 44.63 -> 44.20, across three passes — the
per-track payload behind `wall_debug_tracks` (default **false**), occlusion rays
only for pairs whose bearing sectors overlap (72 ray tests a cycle down to about
a dozen), one `base_link` segment per track per cycle shared by the roles and the
negative evidence, the beam span by two binary searches instead of a comparison
over all 1081 beams, and no second ranking of the published set. The payload
itself turned out to cost only ~0.4 ms; the remaining +5.6 ms over baseline is
2.75 ms of rectangle plus E1's gates and E3's two new rules. A test pins that
the sector gate changes no verdict.

---

## F4 — `other` in the semantic grid

`ROLE_VALUE = {front 10, left 20, right 30, back 40, **other 50**}`, and an
unknown role now falls back to `other` instead of `back`. Full consumer list in
[f4_semantic_grid_consumers.md](f4_semantic_grid_consumers.md): the only runtime
consumer of `/costmap/wall_cells` is `mission_logger`, which records bytes
without interpreting them; `VALUE_ROLE` is used only by a test; the costmap
renderer, boundary node, MPC, behaviour tree, nav2 and the foxglove relays do
not subscribe to it. **Nothing in the control path reads that grid**, so the new
value cannot be misread by anything that exists today. Two regression tests.

---

## F6 — `enable_mission_logger` default false

Launch description, built offline, every action whose condition evaluates true:

```
--- DEFAULT ARGS (enable_mission_logger=false)
    (nothing)
--- enable_mission_logger:=true
    Node topic_tools/throttle            <- the 5 Hz depth copy
    Node f1tenth_logger/mission_logger_node
```

With false the node does not exist and neither does the throttle that exists
only to feed its bags; with true the description is exactly what it was. **The
logger's own logic was not touched** — both actions were already gated on the
argument.

The load-bearing part: `component_supervisor_node` launches each component with
the args `components.yaml` gives it and forwards **none** of its own, so
`supervisor_bringup.launch.py enable_mission_logger:=true` would have been
silently ignored. `supervisor_bringup` now declares the argument and
`_apply_mission_logger_override()` writes the live value into the args of
whichever entry launches `mission_logger.launch.py`, found by launch_file, not
by position — mirroring `_apply_calibration_override()`. Verified both ways:

```
enable_mission_logger=False -> ros2 launch f1tenth_logger mission_logger.launch.py enable_mission_logger:=false
enable_mission_logger=True  -> ros2 launch f1tenth_logger mission_logger.launch.py enable_mission_logger:=true
```

Changed so campaign runs ask for it explicitly: `TEST_CAMPAIGN.md` (the stack
command, why, and two commands to check it took),
`docs/procedures/2026-09-18_go_to_floor.md`,
`docs/procedures/2026-09-17_floor_person_pass.md`, and the `README.md` Run row.

---

## F5 — the second recorder (report + proposal, nothing applied)

Full version in [f5_recorder.md](f5_recorder.md).

**Who: nobody in the repo — it was typed by hand.** No launch file, component or
script runs `ros2 bag record` against the live stack; `components.yaml` has no
recorder entry; the supervisor's log directory holds only `mission_logger`. The
bag says `-a` itself: 148 topics with **21 empty**, 16 `/viz/*` relays that
`mission_logger` is coded to drop, and `/events/write_split` — a topic the
recorder itself publishes, so only `-a` subscribes to it. `~/.bash_history` has
no `bag record` line, so that shell never flushed. **It is not part of the
bringup, so the conditional toggle does not apply** (F6 already made the one
that IS part of the bringup opt-in).

| | manual `-a` | `mission_logger` |
|---|---|---|
| file / span | 548.4 MB / 98.7 s | 71.3 MB / 13.9 s |
| **payload rate** | **5.51 MB/s** | **5.09 MB/s** |
| topics | 148 (21 empty) | 55 |
| Image/grid/cloud share | **93 %** | **88 %** (the 5 Hz depth alone is 65 %) |
| `/viz/*` | 16 topics, 3 % | excluded by code |
| measured loss | **24 % of `/scan`, 48 % of `/odometry/filtered`** | `/scan` complete at 40.0 Hz |

They ran at the same time: ~10.6 MB/s while 12 cores sat at 95–97 %.

**Proposal, not applied:** install `ros-humble-rosbag2-storage-mcap` (the default
already asks for mcap and already falls back loudly, so it is an apt command and
no code); zstd in **file** mode so compression happens on close and not on the
hot path; raw images out of the default list starting with
`mission_logger_record_depth` defaulting false; `/viz/*` never (already true);
write cache 100 → 512 MiB against the measured 288–521 ms recorder lag. And the
free one: never run a second `ros2 bag record` beside the mission logger.

---

## G — glass from LiDAR intensity

The ≤20-line verdict is [output/glass_lidar/report.md](../glass_lidar/report.md).
Tool: `tools/glass_intensity_probe.py` (no stack code). Plots:
`intensity_vs_incidence.png`, `intensity_vs_range.png`,
`passthrough_vs_incidence.png`; data in `beams.csv` (3 465 sampled beam rows),
`windows.csv` (52 feature windows), `features.json`.

**G1 green:** `/scan.intensities` populated on every scan of both bags — 1081
values, 0..4782, 86.5 % nonzero. `src/f1tenth_bringup/config/sensors.yaml`
already sets `publish_intensity: true`, against urg_node's shipped default of
false; the UST-10LX supports it. So G2–G5 ran.

**G2–G4, and why they decide nothing yet.** Ground truth from the rectangle's own
flags: 46 VERIFIED tracks, only **4 glass**, 3 of them carrying both flags over
their life, and after windowing at 10 scans exactly **one** glass track (id 33,
2.74 m) yields 3 windows against 49 from 34 normal tracks.

| feature | glass median | normal median | AUC | best cut | balanced acc |
|---|---|---|---|---|---|
| intensity × range² | 7 103 | 9 225 | 0.083 | `<= 7107` | 0.958 |
| pass-through ratio | 0.000 | 0.612 | 0.069 | `<= 0.018` | 0.948 |
| no-return fraction | 0.000 | 0.006 | 0.245 | `<= 0` | 0.755 |
| intensity-vs-incidence slope | +91 | −50 | 0.600 | `>= 70` | 0.800 |
| max intensity at incidence < 5° | — | — | — | — | **no samples** |

Two findings that hold regardless of the sample size: **no beam anywhere in the
run is within 5° of a surface normal** (minimum 12.0°; the glass is seen at
26–45°), so the proposed headline feature is unmeasurable here; and **at that
angle the LiDAR does not see through the pane at all** — pass-through 0.000, no
returns missing — so the glass label came from the stereo seeing the room behind
it, not from the LiDAR missing it. Best (weak) feature: range-normalised
intensity, lower on glass, which is the physically expected sign and also a line
drawn through one point. Proposal and the run to record next are in the report.

---

## Commits (local, nothing pushed)

| commit | point |
|---|---|
| `5e5adef` | F2 — what the aliased header falsified, measured against `c44b503` |
| `bb297fe` | F1 — past 8 m the stereo is not evidence, and rejection is not symmetric |
| `05c5898` | F3 — the debug payload is opt-in and the occlusion only tests what can occlude |
| `b5a97a3` | F4 — `other` is its own cell in the semantic grid, not `back` |
| `774b2e0` | F6 — recording is opt-in, and the toggle now actually reaches the logger |
| `9e8e56d` | F5 — the second recorder was typed by hand, not launched |
| `edfd9f8` | WIP: G1 green and the glass/intensity probe |
| `fdd65e0` | G — intensity is there, the glass examples are not |

## Housekeeping

- No ROS process left running; none was ever started (the replays construct the
  two nodes in-process on an isolated domain).
- The temporary worktree at `c44b503` is removed (`git worktree list` clean) and
  every scratch file lives under the session scratchpad, not the repo.
- `colcon build` of the seven touched packages: clean.
- Suites: `f1tenth_perception` + `corridor_perception` + `f1tenth_costmap`
  **1178 passed, 1 skipped, 1 xfailed**; `f1tenth_bringup` + `f1tenth_logger`
  node/launch tests 81 passed. The three `f1tenth_bringup` lint failures
  (`test_flake8`, `test_pep257`, `test_copyright`) are **pre-existing** —
  verified identical with my changes stashed; they walk the untracked
  `install_base573/` tree in the repo root.
