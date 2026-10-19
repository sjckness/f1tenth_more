# P1 — wall-foot feasibility: the gate is NOT passed

> **Superseded by `output/wall_v2/q1/report.md`.** This pass ran on archive bags
> that predate the measured camera pitch and whose recorded class mask was unusable.
> Q1 repeats it on `corridor10m_20261016T144537` with the live-configuration mask
> and the real -2.083 deg pitch. The verdict does not change; several of the
> reasons below do. See the CORRECTED note under cause 2.

`tools/wall_foot_eval.py`, read-only, nothing wired into the stack.

## Verdict

| gate condition | required | stationary bag | moving bag | |
|---|---|---|---|---|
| foot median residual, 1.5-3 m, best variant, radial on | **≤ 0.10 m** | no segment matched at all; points 0.342 m | 0.187 m (n=4); points 0.250 m | **FAIL** |
| foot spurious segments/frame < stereo_old | — | 0.415 vs **0.072** | 0.040 vs 0.119 | **FAIL** (1 of 2) |

Condition 1 fails on both bags, condition 2 on one. **Do not proceed to P2.**

## The data, and why it is this data

The bags the work order named have no depth at all. Of 305 `.db3` files under
`~/bags` and `~/f1tenth_archive`, seven carry depth; all are archive mission runs.
Two were used:

| | frames | /scan | motion | note |
|---|---|---|---|---|
| `active/2026-10-14T12-05-34_mission-llm_2b356aac445f` | 335 depth, 804 masks | 2922 | **none** (0.00 m) | the most depth available |
| `active/2026-10-15T11-18-01_mission-llm_86cff518e42d` | 126 depth, 155 masks | 1126 | 4.16 m, v max 0.47 | exercises the odom-compensated accumulation |

The RECORDED `/perception/scene_labels` is useless for this: it is 160x90 and
contains **zero wall and zero floor_inferred pixels** in all 483 label frames of
all five depth bags checked — an older configuration. The mask is therefore
RECOMPUTED from the raw depth through `front_clearance_node`
(`corridor_wall_replay.MaskReplay`), which is what P2 would run live anyway. That
mask is 640x360 and rich: 45.9 % wall, 34.4 % floor_inferred, 6.5 % ceiling,
6.8 % self, 3.2 % floor.

## Why the foot fails — two causes, both structural

### 1. The "wall" class is not a wall; the foot traces the inferred-floor horizon

`camera_00.png` shows it directly: the green foot row runs along the boundary
between the cyan `floor_inferred` region and the olive `wall` region, a smooth
curve across the whole image. The olive region is everything that is neither
floor nor ceiling — open space included. The boundary is the edge of the
inferred-floor disc, which `front_clearance_inferred_floor_r_max_m` caps at
**3.0 m from the laser**.

The range histogram of the foot points (60 frames, 25 448 points) is that cap:

```
  0.50-0.75 m     351  ###
  2.50-2.75 m     337  ###
  2.75-3.00 m    5402  ############################################################
  3.00-3.25 m    4568  ##################################################
  3.25-3.50 m    4349  ################################################
  3.50-3.75 m    2711  ##############################
  ... tail to 5.00 m
```

39 % of all foot points land in 2.75-3.25 m and **nothing at all lands between
0.75 m and 2.50 m** — the band the gate is about. The corridor's side walls, which
the LiDAR puts at |y| = 1.6 m in the same frames, produce no foot points
whatsoever. A foot that cannot see a wall 1.6 m away is not a wall source.

### 2. The camera is 22 cm off the floor, so one pixel is 16 cm at 3 m

P1's own noise model, `sigma_r = d^2 / (h fy)`, with the measured h = 0.2200 m
and fy = 262.44 px:

| range | rows below the horizon | metres per pixel |
|---|---|---|
| 1.0 m | 57.7 | 0.017 |
| 1.5 m | 38.5 | 0.039 |
| 2.0 m | 28.9 | 0.069 |
| 3.0 m | 19.2 | **0.156** |
| 4.0 m | 14.4 | 0.277 |
| 5.0 m | 11.5 | 0.433 |

At 3 m the whole wall foot is 19 pixels below the horizon and one pixel of label
error is 15.6 cm. The gate asks for a **median** of 10 cm across 1.5-3 m, i.e.
sub-pixel row accuracy at the top of its own bucket. This camera cannot deliver
it from this height, whatever the mask does.

**CORRECTED 2026-10-18 (Q0).** This section first said the pitch "is not there".
That was true of THESE TWO BAGS and wrong about the stack. `camera.launch.py:76`
publishes `base_link -> zed2_camera_link` with pitch **-0.03635 rad (-2.083 deg)**,
measured 2026-10-15, and both 10-16 corridor bags carry it in `/tf_static`
(composed optical +Z in base_link = (0.99934, 0, 0.03634)). The two archive bags
used here are from 2026-10-14 and 2026-10-15 11:18 and **predate that
measurement**: their chain really is identity. `wall_foot_eval.py` composes the
full chain from whichever bag it is given, so it used the right frames for these
bags — but a pitch-less bag is the wrong era to judge the live foot on, which is
why Q1 re-runs the whole thing on `corridor10m_20261016T144537`. Scale of the
error: 2.083 deg is fy * 0.03635 = 9.5 px of foot row, 1.5 m of range at 3 m.
Everything below this line is therefore about the archive bags only; the Q1
report supersedes it.

## The radial filter earns its place anyway — on the EXISTING source

This is the one clear win of P1, and it needs no foot:

| | segments | dropped by the filter | spurious/frame | residual 1.5-3 m | residual 3-5 m |
|---|---|---|---|---|---|
| stereo_old, filter OFF (moving) | 129 | — | 0.762 | 0.116 | 0.071 |
| stereo_old, filter ON (moving) | 31 | 98 (76 %) | **0.119** | 0.114 | **0.065** |
| stereo_old, filter OFF (stationary) | 936 | — | 1.069 | 0.030 | 0.234 |
| stereo_old, filter ON (stationary) | 323 | 613 (65 %) | **0.072** | 0.030 | **0.184** |

A 6x to 15x cut in spurious segments, residuals unchanged or better. **The 45 deg
fans do disappear**: they are exactly the segments whose direction lies along the
viewing ray, and the 25 deg ray-angle test removes them without touching a wall
seen side-on. This is worth doing to `stereo_old` on its own, independently of
anything the foot was meant to add.

## Other measurements

Coverage — the fraction of frames with a side wall seen at 1.5-5 m — is the other
place the foot loses outright:

| source (moving bag, radial on) | left % | right % |
|---|---|---|
| foot_0.5s | 0.8 | 2.4 |
| foot_1.0s | 0.8 | 0.8 |
| stereo_old | 8.7 | 3.2 |
| merged | 4.8 | 2.4 |

Cost is not the problem: foot extraction p50 3.23 ms, the virtual scan + C
extraction p50 2.29 ms, 5.5 ms p50 per frame all in — inside a 5 Hz camera budget
with room to spare. The method is cheap and wrong, not slow.

Column rejections over the 335-frame bag: `too_far` 61 220, `no_transition` 8 992,
`reopened` 1 933, `occluded` 0, `self_mask` 0, `horizon` 0. The occlusion and
opening tests almost never fire, which is itself a symptom: the transition being
found is not a wall base, so there is no occluder in front of it and no doorway
above it.

## What would make the foot worth revisiting

1. A `wall` class that means a wall surface, not "not floor and not ceiling".
2. An inferred-floor cap beyond the band of interest, or a real floor segmentation
   that reaches the wall base.
3. A measured camera pitch (and a floor flat enough to assume z = 0), or the
   camera mounted higher: at h = 0.5 m, 3 m costs 6.9 cm per pixel instead of
   15.6 cm, which puts the gate within reach.

Without (1) and (3) the foot cannot be evaluated fairly, and nothing in this
session's data can supply them.

## Artefacts

`output/wall_v2/p1/stationary_12-05-34/` and `moving_11-18-01/`: `wall_foot_eval.md`,
`wall_foot_eval.json`, eight topdown frames (LiDAR grey, foot points by distance,
foot green, stereo_old red, merged blue, dashed = radial-rejected) and two mask
frames with the foot row drawn.
