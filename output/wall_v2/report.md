# Far-wall regression fix — report

## Morning summary

1. **Main bag `corridor10m_20261016T170931` is unusable as specified: the car never
   moved and the bag has no depth.** Three independent checks (all three odometry
   topics at 0.00 m of path; ERPM 0 and every drive command exactly 0.000, mission
   IDLE; ZED odometry 6-9 cm and the forward laser range 19.41-19.44 m for all 325 s).
   No movement interval exists, so the analysis-window rule yields nothing.
2. P0 therefore ran on **`corridor10m_20261016T144537`**, window **56.57-152.60 s**
   (movements 58.57-70.92 and 125.94-150.60, built by the order's own rule), which is
   where the stated regression lives. The main bag was kept as a second, stationary
   far-wall probe (a wall at 19.4 m for 325 s).
3. **P0 culprit: `d578963` (R2 block 3) alone**, gate 3d. `9786bfc` (R2 block 2, the
   rate policy) is byte-identical to pre-merge — the rate-gate hypothesis is wrong, and
   the replay harness disables both rate gates anyway.
4. **Mechanism:** `wall_fusion.confidence` scores from an absolute `inlier_ref` (40
   returns) and `rms_ref` (0.03 m). LiDAR returns on a surface thin out as 1/range, so
   the same real wall scores 0.65 at 1 m and 0.23 at 21 m. 71 % of observations inside
   1.5 m reach the 0.5 FRONT gate; **1 % at 12-18 m and 0 % past 18 m**, with the
   12-18 m band's rms the *best* in the bag. 67 % of the window's cycles returned
   `front_reason = low_confidence` while the strip measured 21.16 m.
5. **WALL time / agreement, 144537 window: 2.88 s → 69.00 s** (the car itself: 68.61 s)
   and **0.124 → 0.808** raw, **0.948** over the cycles the car did not call NO_DATA.
   Wall-distance error p95 **1.217 m → 0.004 m**.
6. **Main bag, second witness: 0.00 s → 68.84 s of WALL** (car: 66.73 s), agreement
   **0.470 → 0.989** excluding the car's own NO_DATA, **zero phantoms**, `wall→clear`
   confusion 434 → 8.
7. **The 147-150 s phantom stays gone and NO_DATA stays 0.** One 4-cycle (0.85 s)
   episode remains in the 144537 window, reporting a wall at 18.6-18.9 m where the
   tool's strip truth has no return — it cannot stop the car early.
8. **Timing:** tracker p50 28.68 → 29.09 ms, p95 38.93 → 39.93 ms. Budget +3 ms, used
   +0.4 / +1.0 ms.
9. **P1 GATE: NOT PASSED, on the measurement.** Run on the two archive bags that do
   carry depth (335 frames stationary, 126 frames with 4.16 m of motion), with the mask
   RECOMPUTED from raw depth because the recorded label stream has zero wall pixels.
   Best foot residual at 1.5-3 m is **0.187 m** against a 0.10 m gate (point-level
   0.25-0.34 m); the foot also loses the spurious-segment test on one of the two bags.
   Two structural causes, not tuning: the mask's `wall` class is "not floor and not
   ceiling" so the foot traces the inferred-floor horizon at 3.0 m (39 % of points in
   2.75-3.25 m, **nothing at all between 0.75 and 2.50 m**), and at h = 0.22 m one pixel
   is 15.6 cm of range at 3 m, so the gate is sub-pixel at the top of its own bucket.
   Full write-up: `output/wall_v2/p1/report.md`.
10. **The radial filter is a win on its own**, on the source already in the stack:
    applied to `stereo_old` it drops 65-76 % of segments and cuts spurious segments per
    frame from 1.069 to 0.072 and from 0.762 to 0.119, residuals unchanged or better.
    The 45 deg fans disappear. It needs no foot — but it is a P2 change, and the gate
    says stop, so it is not made here.
11. P2, P3 and P4 are consequently not done. The regression replay that P4 defers to
    "if time allows" was folded into P0 and is in the tables below.
12. Open points: (a) the foot needs a real `wall` class, an inferred floor that reaches
    past 3 m, and a measured camera pitch (or a higher mount: h = 0.5 m would put the
    gate within reach) before it is worth revisiting; (b) the 18.9 m episode, i.e.
    whether FRONT should have a range cap; (c) `/sensors/core.distance_traveled` counts
    up 9807 ticks on a bag where ERPM is 0 throughout — one of the two is lying.

## The bisect

Same v2 replay tool for every point, same bag and window, only
`src/f1tenth_perception` shadowed on `PYTHONPATH` from a `git archive` export (D4).
Pass `replay_rec`; "agree" is agreement with the car's recorded contract.

| point | WALL s | CLEAR s | NO_DATA % | phantoms | 147-150 s phantom | agree | err p95 |
|---|---|---|---|---|---|---|---|
| `9c46add` pre-merge (V1-V4 base) | 69.61 | 21.26 | 5.40 | 1 | **YES** | 0.933 | 0.011 |
| `9786bfc` R2 block 2 (rate policy) | 69.61 | 21.26 | 5.40 | 1 | **YES** | 0.933 | 0.011 |
| `d578963` R2 block 3 (phantom fix) | **2.88** | **93.19** | 0.00 | 0 | no | **0.124** | 1.217 |
| `0c1cd417` HEAD (= merged) | 2.88 | 93.19 | 0.00 | 0 | no | 0.124 | 1.217 |
| `6cbda121` P0 fix | **69.00** | 27.06 | 0.00 | 1 | no | **0.808** | **0.004** |

`82c0afa` and the merge commit add nothing after `d578963`; the merge was shown inert
on the contract in `output/merge_cleanup/decisions.md` D0.

## Why the far wall was refused — the measurement

Every LiDAR/BOTH observation in the window, bucketed by the observation's own range
(`|WallLine.d|`), scored by the pre-fix model:

| range | n | inliers med | rms med | span med | conf med | conf p90 | % ≥ 0.5 |
|---|---|---|---|---|---|---|---|
| 0.0-1.5 m | 1222 | 32 | 0.0330 | 0.88 | 0.654 | 0.775 | 71.0 % |
| 1.5-3.0 m | 1722 | 42 | 0.0304 | 1.26 | 0.599 | 0.731 | 67.4 % |
| 3.0-5.0 m | 59 | 22 | 0.0848 | 0.71 | 0.341 | 0.591 | 11.9 % |
| 5.0-8.0 m | 697 | 26 | 0.0345 | 0.89 | 0.474 | 0.620 | 34.1 % |
| 8.0-12.0 m | 319 | 36 | 0.0332 | 2.09 | 0.570 | 0.659 | 65.2 % |
| 12.0-18.0 m | 203 | 19 | **0.0156** | 1.85 | 0.452 | 0.471 | **1.0 %** |
| 18.0-25.0 m | 249 | 14 | 0.1070 | 1.22 | 0.226 | 0.258 | **0.0 %** |

`front_reason` over the 374 cycles of the window, pre-fix: `low_confidence` 249,
`no_candidate` 102, `coast_expired` 12, `front` 11. On the `low_confidence` cycles
`strip_range_m` is 21.16 m and the car's own contract says `wall 21.19`.

And the phantom, probed at t=146-150 s: its candidates are refused `off_strip` (tracks
574, 586) and `role_other` (495) at confidence 0.51-0.78 — never `low_confidence`. The
geometry does that work, which is why relaxing confidence does not bring it back.

## The fix

`wall_fusion.confidence(..., range_m=...)` stretches both references by
`range / wall_conf_range_ref_m`:

| | before | after |
|---|---|---|
| 21 m, 14 inliers, rms 0.107 | 0.228 | **0.616** |
| 15 m, 19 inliers, rms 0.0156 | 0.453 | **0.741** |
| 3 m (the phantom band), 22 inliers, rms 0.0848 | 0.325 | 0.325 |
| 1 m, 32 inliers, rms 0.033 | 0.510 | 0.510 |
| 60 m, 3 inliers, rms 0.02 | 0.270 | 0.270 |

New keys (`stack_params.yaml`): `lidar_front_wall_wall_conf_range_ref_m` 5.0,
`lidar_front_wall_wall_conf_min_inlier_ref` 8.0,
`lidar_front_wall_wall_conf_max_rms_scale` 6.0. All SYNTHETIC, chosen from the table
above, wired through `lidar_front_wall.launch.py`.

## Contract, before and after

**144537, window 56.57-152.60 s** (703 cycles)

| | pre-fix | P0 fix | the car |
|---|---|---|---|
| WALL / CLEAR / NO_DATA [s] | 2.88 / 93.19 / 0 | **69.00** / 27.06 / 0 | 68.61 / 20.87 / 6.46 |
| confusion `wall→wall` / `wall→clear` | 6 / 509 | **493 / 22** | — |
| agreement, raw | 0.124 | 0.808 | — |
| agreement, excluding the car's NO_DATA | 0.124 | **0.948** | — |
| all-WALL err p95 / max | 1.217 / 2.174 m | **0.004** / 2.174 m | 0.010 / 4.96 m |
| phantom episodes (147-150 s one) | 0 (no) | 1 (no) | 1 (**yes**) |

**Main bag 170931, whole bag, stationary, wall at 19.4 m** (1661 cycles)

| | pre-fix | P0 fix | the car |
|---|---|---|---|
| WALL / CLEAR / NO_DATA [s] | 0 / 317.27 / 0.40 | **68.84** / 248.43 / 0.40 | 66.73 / 67.68 / 184.00 |
| confusion `wall→wall` / `wall→clear` | 0 / 434 | **426 / 8** | — |
| agreement, raw | 0.232 | 0.488 | — |
| agreement, excluding the car's NO_DATA | 0.470 | **0.989** | — |
| phantom episodes | 0 | **0** | — |

The car spent 184 s of this bag in NO_DATA — the old coast latch, stationary, staring at
a wall — which is what block 3e deliberately replaced; those 841 cycles are why the raw
agreement stays low and are not a disagreement about the wall.

## Timing

| | p50 | p95 | max |
|---|---|---|---|
| HEAD | 28.68 ms | 38.93 ms | 52.94 ms |
| P0 fix | 29.09 ms | 39.93 ms | 53.28 ms |

## P1 — evaluated on the archive depth bags; gate NOT passed

`tools/wall_foot_eval.py` written and run on the two bags that carry depth. Both gate
conditions fail, for reasons that are structural rather than tunable. The whole report,
with the range histogram, the per-pixel sensitivity table and the radial-filter
measurement, is in **`output/wall_v2/p1/report.md`**; the decision to re-point P1 at
those bags is D5 in `decisions.md`. Stopped before P2, as P1 instructs.

## Artefacts

| path | what |
|---|---|
| `output/wall_v2/baseline/b144537/` | HEAD on the window, the P0 baseline |
| `output/wall_v2/p0_fix/` | the same with the fix |
| `output/wall_v2/main_bag_prefix/`, `main_bag_p0fix/` | the main bag, before and after |
| `output/merge_cleanup/` | the merge session's three replays, still comparable |

No videos: `--no-video`, because the camera half cannot be drawn on bags with no depth
and no mask frames.

## Commits

| hash | point |
|---|---|
| `6cbda121` | P0 — the range-aware confidence model, 3 params, 14 tests |
| `6080fe0f` | P1 (first pass) — the blocker report and the P0 evidence base |
| this one | P1 — `tools/wall_foot_eval.py`, both runs, the verdict |

Tag `backup/pre-wall-v2-0c1cd417` marks the state before this session. Nothing pushed.
