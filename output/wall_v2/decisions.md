# Decisions — far-wall regression fix (P0) and the P1 gate, 2026-10-18

## D1. The main bag cannot carry the analysis window, so P0 runs on 144537

**Decision.** P0 is measured on `~/bags/corridor10m_20261016T144537` over the window
**56.57-152.60 s**, built by the work order's own rule. The designated main bag,
`~/bags/corridor10m_20261016T170931` (the newest, as instructed), is kept as a second,
stationary probe of the same question.

**Why.** The main bag has no movement at all. Three independent checks, each able to
fail on its own:

1. `/odometry/filtered`, `/odom` and `/ekf_global/odometry/filtered` all report 0.00 m
   of path and `twist.linear` identically 0 over 318 s. The window rule reads exactly
   this topic, so it yields no interval to pad.
2. `/sensors/core` ERPM max 0.0; `/commands/motor/speed`, `/drive` and
   `/ackermann_drive` are exactly 0.000 on every one of their 2361 / 2432 / 1992
   messages; `/mission/status` never leaves IDLE.
3. Independent sensors: ZED visual odometry moves 6-9 cm in total (jitter), and the
   forward ±10° median laser range is 19.41-19.44 m for all 325 s.

The same script on 144537 finds the two movements the order expects (58.57-70.92 s and
125.94-150.60 s) and the window 56.57-152.60 s, so the tooling is right and the bag is
stationary.

**Alternatives rejected.** (a) Use the whole main bag as the window — it would satisfy
the letter of "a window" but none of the order's metrics are defined off a moving car,
and the stated regression (509 wall→clear cycles) is a 144537 measurement that would
then have no baseline. (b) Fall back to the next-newest bag silently — 144537 *is* the
next-newest, and this makes that explicit rather than hiding it in a path.

**Consequence.** Every P0 number in report.md is over 56.57-152.60 s of 144537 and is
directly comparable with `output/merge_cleanup/`. The main bag appears separately, as a
325 s stationary wall at 19.4 m — the purest far-wall case available, and a second
witness that is not the bag the fix was tuned on.

## D2. The culprit, and why the fix is where it is

**Decision.** The regression is `d578963` (R2 block 3) alone, specifically gate **3d**
(`front_confidence >= front_min_confidence`), and the fix makes
`wall_fusion.confidence`'s two references range-aware rather than touching block 3's
geometry.

**Why not the alternatives the order suggested.**

* *The rate gate starving the contract under 1/4 decimation.* Ruled out twice. The
  bisect point `9786bfc` (R2 block 2, which IS the rate policy) is byte-identical to
  pre-merge on every metric. And the replay harness sets `node._tracker_due = lambda:
  True` and `virtual_wall_rate_hz = 0.0` (`corridor_wall_replay.py:831-840`), so no rate
  gate is even active in the numbers. The order's own requirement — "the rate gate must
  let a confirmed front track reach the contract at least once per tracker period" — is
  already satisfied on the car and is now pinned by a test: the cap is `0.75 / rate_hz`
  = 0.075 s at 10 Hz, shorter than the 0.1 s step period 40 Hz scans decimated by 4
  give, so every step publishes. No change was needed there.
* *Collinear pieces across an opening treated as a phantom even when a piece crosses
  the strip.* Measured, not assumed. Probing t=146-150 s, the candidate tracks come back
  `off_strip` (574, 586) and `role_other` (495) at confidence 0.51-0.78 — the geometry
  refuses them and the confidence gate never gets a say. Over the whole window
  `low_confidence` is 249 of 374 cycles while the strip reads 21.16 m. The failure is
  confidence, not geometry, so the geometry is left alone.

**The mechanism.** `wall_fusion.confidence` scores an observation from `inlier_ref` 40
returns and `rms_ref` 0.03 m, both absolute. A planar LiDAR's returns on a surface thin
out as 1 / range and its range noise grows with range, so the same real wall scores
0.65 at 1 m and 0.23 at 21 m:

| range of the observation | n | inliers med | rms med | conf med | % reaching 0.5 |
|---|---|---|---|---|---|
| 0.0-1.5 m | 1222 | 32 | 0.0330 | 0.654 | 71.0 % |
| 1.5-3.0 m | 1722 | 42 | 0.0304 | 0.599 | 67.4 % |
| 3.0-5.0 m | 59 | 22 | 0.0848 | 0.341 | 11.9 % |
| 5.0-8.0 m | 697 | 26 | 0.0345 | 0.474 | 34.1 % |
| 8.0-12.0 m | 319 | 36 | 0.0332 | 0.570 | 65.2 % |
| 12.0-18.0 m | 203 | 19 | **0.0156** | 0.452 | **1.0 %** |
| 18.0-25.0 m | 249 | 14 | 0.1070 | 0.226 | **0.0 %** |

The 12-18 m band has the **best** rms in the bag and still fails: it is the inlier count
that collapses. The far wall was refused for being far.

**The fix, and how both intents survive.** Both references are stretched by
`range / wall_conf_range_ref_m` (5 m) — fewer returns expected far away, a looser fit
tolerated, capped by `wall_conf_max_rms_scale`. Inside 5 m it is the identity, and the
phantom block 3 was written for lives at 1.9-3.3 m. Two independent guards therefore
still hold it: it is outside the scaled band, and it is refused on geometry anyway.

**One thing the first version got wrong.** With only a floor on the scaled `inlier_ref`,
a three-return sliver at 60 m scored 0.51 — the stretched `rms_ref` let a tiny residual
stand in for absent evidence. A test caught it. The scaling is now refused outright
below `wall_conf_min_inlier_ref` (8 returns): distance alone buys nothing.

## D3. P1 GATE — cannot be evaluated; stopping before P2

**Decision.** P1's gate is not met, because it cannot be measured on any available data.
Per P1's own instruction ("Otherwise: commit script + report and STOP"), work stops
before P2. No `tools/wall_foot_eval.py` was written.

**Why.** Every input P1 needs is absent from the designated bag:

| P1 needs | main bag 170931 | 144537 |
|---|---|---|
| depth frames (`depth_registered`) | **0** | **0** |
| `/zed2/zed_node/depth/camera_info` | 23 | present |
| `/camera/detection_masks` | 5 | **0** |
| `/perception/scene_labels` | 3 | 0 |
| `/perception/scene_labels/depth_mm` | 0 | 0 |
| `/perception/stereo_columns`, `/perception/wall_points` | 0, 0 | 0, 0 |

The wall foot is a mask-and-depth construction: with no depth image there is no pixel
ray to intersect with the floor plane, and with 5 mask frames in 325 s there is no
`floor → wall` transition to scan for. The two camera sources P1 compares
(`foot`, `stereo_old`) therefore have no data on either side, and a gate decided on
nothing is worse than no decision.

**Not written rather than written blind.** A few hundred lines of foot geometry, virtual
scan and radial filtering that cannot be executed once would be unreviewable and would
then gate P2 and P3. The work order's own gate exists to stop exactly this.

**What would unblock it.** 305 `.db3` files under `~/bags` and `~/f1tenth_archive` were
searched; **7 bags have depth**, all archive mission runs, none a corridor recording.
The best is `~/f1tenth_archive/active/2026-10-14T12-05-34_mission-llm_2b356aac445f/bag/`
— 335 depth frames, 804 `/camera/detection_masks`, 510 `/perception/scene_labels`. That
is enough to attempt P1, but it is a different scene, a different window and not
comparable with the P0 baseline, so **whether to re-point P1 at it is your call**, not
one to take unattended. The clean alternative is one new corridor recording with the
depth and mask topics enabled, moving, which would serve P1-P4 as written.

## D4. Temporary exports instead of worktrees for the bisect

The order asks for temporary worktrees. Each bisect point instead used
`git archive <commit> src/f1tenth_perception | tar -x` into the scratchpad and
`PYTHONPATH` shadowing. It is strictly weaker on the repository — no new worktree
state, no checkout, nothing to clean up — and it keeps `tools/` pinned at the v2 replay
for every point, which the order requires and which a worktree would not give (the tool
itself changed in `0abfd92`). Only `src/f1tenth_perception` differs between points, and
only `9786bfc` and `d578963` touch it.

## D5. P1 re-pointed at the archive depth bags, and the gate then failed on merit

**Decision.** On "go on", P1 was run on the two archive bags that carry depth —
`active/2026-10-14T12-05-34_mission-llm_2b356aac445f` (335 depth frames, stationary)
and `active/2026-10-15T11-18-01_mission-llm_86cff518e42d` (126 frames, 4.16 m of
motion) — rather than left blocked. `tools/wall_foot_eval.py` was written and the gate
was evaluated. **It fails, on the measurement, not on missing data.**

**Why this is a fair test and not a fudge.** D3 stopped because no bag had the inputs.
Two things changed that: the recorded label stream is not the only mask available, and
the recomputed one (what P2 would run live) is rich — 45.9 % wall, 34.4 % floor_inferred
at full 640x360. With a real mask, a real depth stream, a LiDAR reference in the same
frames and both a stationary and a moving bag, the question P1 asks is answerable. It is
a different scene from the P0 baseline, so **none of these numbers feed P4's comparison**
and none of them revise anything in the P0 report.

**The alternative rejected.** Tuning the foot until it passed. Three things were wrong
with the foot and only one is a parameter: the mask's `wall` class is "not floor and not
ceiling" rather than a wall surface; the inferred floor is capped at 3.0 m so the
transition found is its own horizon; and at h = 0.22 m one pixel is 15.6 cm of range at
3 m, so the gate's 10 cm median is sub-pixel at the top of its own bucket. The first and
third are not tunable from here, and tuning past them would have produced a number
rather than an answer.

**What came out of it anyway.** The radial-segment filter, which P1 specifies for every
camera source, is a clear win on the source already in the stack: applied to
`stereo_old` it drops 65-76 % of segments, cuts spurious segments per frame by 6x to
15x, and leaves the residuals unchanged or better. The 45 deg fans do disappear. That
is a P2-sized change with its evidence already measured, and it does not depend on the
foot at all — but it is still a P2 change, and P1's gate says to stop, so it is not
made here.

**Consequence.** P2, P3 and P4 remain not done, now for a measured reason rather than an
absent one. The three things that would make the foot worth revisiting are listed at the
end of `output/wall_v2/p1/report.md`.

## D6. Q0 — the camera pitch is real, and P1's claim about it was over-generalised

**The chain.** `front_clearance_node` looks up `base_frame <- frame_id` where `frame_id`
is the depth image's own optical frame (`_frame_transform`, front_clearance_node.py:945),
i.e. `base_link -> zed2_camera_link -> zed2_camera_center -> zed2_left_camera_frame ->
zed2_left_camera_optical_frame`.

**Where the pitch is.** `f1tenth_perception/launch/camera.launch.py:76` publishes the
first edge as `['0.2399', '0.0', '0.205', '0.0', '-0.03635', '0.0', 'base_link',
'zed2_camera_link']` — positional `x y z YAW PITCH ROLL`, so pitch = **-0.03635 rad =
-2.083 deg**, measured 2026-10-15 (comment at :59).

| source | base_link <- optical | pitch |
|---|---|---|
| HEAD launch | — | **-2.083 deg** |
| `/tf_static` of `corridor10m_20261016T144537` | (0.2294, 0.0600, 0.2196) | **-2.083 deg** |
| `/tf_static` of `corridor10m_20261016T170931` | (0.2294, 0.0600, 0.2196) | **-2.083 deg** |
| `/tf_static` of `archive/2026-10-14T12-05-34` | (0.2299, 0.0600, 0.2200) | 0.000 deg |
| `/tf_static` of `archive/2026-10-15T11-18-01` | (0.2299, 0.0600, 0.2200) | 0.000 deg |

**Verdict.** The pitch is present on HEAD and in every bag recorded after it was
measured. No STOP: `floor_inferred` is not invalidated.

**Did P1 use the right frames? Yes, and its conclusion was still over-stated.**
`wall_foot_eval.py` composes the whole chain from the `/tf_static` of whichever bag it
is handed (`scene_labels_replay.static_tf`), so it used exactly what the car used at
recording time. The error was in the write-up, not the code: the P1 report said "the
measured chain is identity apart from the optical convention" as a statement about the
stack, when it was a statement about two bags recorded before 2026-10-15. Corrected in
place, with a pointer to Q1. Scale: 2.083 deg is 9.5 px of foot row, 1.5 m of range at
3 m, so a pitch-less bag is simply the wrong era to judge the live foot on.
