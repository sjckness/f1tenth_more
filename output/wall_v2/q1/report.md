# Q1 — the wall foot on the right data: the new gate is NOT passed

Bag `~/bags/corridor10m_20261016T144537`, window **56.57-152.60 s** (367 mask frames).
Mask: the RECORDED `/perception/scene_labels/classes` — 640x360, live configuration,
34.5 % wall / 30.8 % floor_inferred. No depth is used or needed. TF from the bag's own
`/tf_static`, which carries the **-2.083 deg** pitch (Q0). Foot points within
2 pixel-equivalents of the `inferred_floor_r_max_m` = 3.0 m cap are discarded: 32 196
columns over the window, which is what the first P1 pass was measuring.

## Gate

| condition | required | foot (best) | stereo_old | |
|---|---|---|---|---|
| median residual, 0.75-1.5 m | **≤ 1.0 px** | **3.78 px** (n=21) | 1.52 px | FAIL |
| median residual, 1.5-2.5 m | **≤ 1.0 px** | **2.38 px** | 1.73 px | FAIL |
| coverage left / right, 1.5-4 m, radial on | > stereo_old | 0.8 % / 2.2 % | **14.2 % / 5.7 %** | FAIL |
| spurious segments per frame, radial on | ≤ stereo_old | 0.643 | **0.112** | FAIL |

**All four fail. No P2-foot.** The foot is not better than the source already in the
stack on any of the gate's axes, and it is 6x worse on spurious segments.

## What changed from the first pass, and what did not

The first pass was run on archive bags with no pitch and an unusable recorded mask. Two
of its three findings do not survive the better data; the verdict does.

* **"No foot points between 0.75 and 2.5 m" — WRONG on this bag.** With the real pitch
  and the live mask there are 18 575 of them. The histogram (window totals):

  | range [m] | points | | range [m] | points |
  |---|---|---|---|---|
  | 1.25-1.50 | 21 | | 2.75-3.00 | 1 860 |
  | 1.50-1.75 | 1 344 | | 3.25-3.50 | 307 |
  | 1.75-2.00 | 3 011 | | 3.50-3.75 | 7 990 |
  | 2.00-2.25 | 4 300 | | 3.75-4.00 | 18 911 |
  | 2.25-2.50 | 9 899 | | 4.00-4.25 | 13 408 |
  | 2.50-2.75 | 11 501 | | 4.25-4.50 | 8 793 |

  **0.75-1.5 m is near-empty for want of geometry, not of method.** The LiDAR puts this
  corridor's walls at y = -1.62 and y = +1.60 in the same frames; the camera sits at
  y = +0.06, so the nearest wall is 1.54 m away and nothing in the scene is closer than
  that. The gate's first bucket has n = 21 and cannot be decided on this bag either way.

* **The inferred-floor cap artefact — REAL, and now excluded.** 32 196 foot columns sat
  within 2 px of the 3.0 m cap and are discarded; the 3.00-3.25 m bin is empty as a
  result. What is left is a genuine wall foot: `camera_00.png` shows the green foot row
  sitting on the base of the olive wall regions on both sides, not on a floor horizon.

* **The pixel sensitivity — REAL and unchanged.** h = 0.2196 m, fy = 262.47 px, so one
  pixel of foot row is 3.9 cm at 1.5 m, 6.9 cm at 2 m and 15.6 cm at 3 m. This is why
  the gate is stated in pixels, and the foot still misses it by 2.4x to 3.8x.

## Why the foot is still 2-4 px off — the object band at the wall base

`camera_00.png` shows it: a brown `object` band runs along the base of both walls for
much of the image width. Where it is continuous the `floor -> object` test rejects the
column (part of the 36 854 `no_transition` and the 1 231 `reopened`); where it is
broken, the first wall run above the floor starts at the TOP of the object band, and the
foot lands behind the true base by the band's height. That is a systematic bias of the
right magnitude, and no accumulation window fixes it: `foot_0.5s` and `foot_1.0s` agree
to 2 mm (0.1664 vs 0.1649 m at 1.5-2.5 m).

## Everything measured

| source | radial | residual 0.75-1.5 m | 1.5-2.5 m | 2.5-4.0 m | angle med | spurious/frame | left % | right % | kept | dropped |
|---|---|---|---|---|---|---|---|---|---|---|
| foot_0.5s | on | -- | 0.1664/0.1934 (n=10) | 0.2537/0.4066 (n=3) | 2.91 | 0.643 | 0.8 | 2.2 | 282 | 11 |
| foot_0.5s | off | -- | 0.1664/0.1934 | 0.2094/0.3875 | 2.91 | 0.668 | 0.8 | 2.5 | 293 | 0 |
| foot_1.0s | on | -- | 0.1649/0.1872 (n=8) | 0.4046/0.4120 | 3.04 | 0.643 | 1.4 | 1.6 | 255 | 21 |
| stereo_old | on | -- | 0.1341/0.3519 (n=12) | 0.3229/0.3653 (n=43) | 2.46 | **0.112** | **14.2** | **5.7** | 99 | 437 |
| stereo_old | off | -- | 0.1305/0.3443 | 0.2995/0.3652 | 5.70 | 0.932 | 58.9 | 25.6 | 536 | 0 |
| merged | on | -- | 0.1502/0.1896 (n=18) | 0.2094/0.3875 | 2.92 | 0.643 | 1.1 | 2.5 | 292 | 20 |

Point residuals, metres then pixels:

| source | 0.75-1.5 m | 1.5-2.5 m | 2.5-4.0 m |
|---|---|---|---|
| foot [m] | 0.1451 (n=21) | 0.1911 (n=18 554) | 0.2019 (n=40 569) |
| stereo [m] | 0.0487 (n=4 594) | 0.1134 (n=56 304) | 0.2142 (n=78 089) |
| foot [px] | **3.78** | **2.38** | 0.96 |
| stereo [px] | 1.52 | 1.73 | 1.21 |

Cost: foot 3.35 ms p50, virtual scan + C extraction 1.85 ms, 5.22 ms p50 a frame.

## The one thing to carry into Q2, with a caveat

The radial filter on `stereo_old` repeats here: **437 of 536 segments dropped (82 %),
spurious per frame 0.932 -> 0.112 (8x)**, residual at 1.5-2.5 m unchanged (0.131 ->
0.134 m) and the median angle error halved (5.70 -> 2.46 deg).

**But it also cuts coverage 4x**: left 58.9 % -> 14.2 %, right 25.6 % -> 5.7 %. That is
not a bug in the filter, it is the geometry: a corridor side wall seen from a car driving
along it *is* nearly radial, so a 25 deg ray-angle test removes the very walls P3 wants
to confirm. Q2 has to be judged on both numbers, not the spurious one alone.
