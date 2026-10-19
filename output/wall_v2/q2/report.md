# Q2 — the camera gates and P3, in the stack

All of this is independent of the wall foot, which the Q1 gate refused.

## What was added

**The radial gate**, on every observation the camera had a hand in (STEREO and
BOTH; never LiDAR): the angle between the segment's direction and the ray from
the **camera** to its own midpoint must be at least
`camera_seg_min_ray_angle_deg` (25), and the bearing it subtends from the camera
at least `camera_seg_min_bearing_span_deg` (3). `MultiWallTracker.camera_xy` is
set by the node from `/tf_static`; without it the laser stands in.

**The stereo quality gates**, on STEREO-only observations: `stereo_max_rms_m`
0.05, `stereo_min_vertical_extent_m` 0.4 (only where the height was actually
measured), `stereo_min_length_m` 0.8.

**P3.1 — camera confirmation on fresh cycles only.** The tracker runs at the
scan rate and the camera at 5 Hz, so one profile is re-presented over several
cycles; without this, three cycles of one stale observation confirm a wall that
was seen once. A new `Track.fresh_hist` mirrors `history` except that a
camera-only hit on a stale cycle counts as a miss, and `_confirmed_by_history`
reads it for a track the LiDAR has never seen. The **state** machine still runs
on the ordinary hit, so a track does not start coasting because the camera was
late. `camera_confirm_fresh_only` false restores the old behaviour.

**P3.2 — the camera side zone.** A camera-only track is a side wall only within
`camera_side_x_max_m` 4.0, with its normal within `camera_side_max_normal_deg`
30 of the lateral axis (the LiDAR keeps 45), and at a lateral offset between
`camera_side_offset_min_m` 0.3 and `camera_side_offset_max_m` 3.0. The LiDAR
keeps its own wider zone untouched.

**P3.3 — LiDAR + camera into one track, labelled "both".** Already in the
tracker (`Track.has_lidar` / `has_both`, `source` ranked both > lidar > stereo,
`_absorb`), and "both" additionally requires the rectangle to have run
informatively on that track. Two tests now pin both halves of that rule rather
than leaving it implied.

**P3.4 — `/viz/walls` carries the source.** The TEXT marker is now
`"id role source state dist"`, source from `wall_fusion.SOURCE_NAMES`. No foot
points marker: there is no foot source (Q1).

## The replay — `corridor10m_20261016T144537`, window 56.57-152.60 s

### The contract is untouched, exactly

| | P0 level | Q2 |
|---|---|---|
| WALL / CLEAR / NO_DATA [s] | 69.00 / 27.06 / 0.00 | **69.00 / 27.06 / 0.00** |
| agreement with the car, raw / excl. its NO_DATA | 0.808 / 0.948 | **0.808 / 0.948** |
| confusion wall→wall / wall→clear | 493 / 22 | **493 / 22** |
| all-WALL error p95 | 0.004 m | **0.004 m** |
| phantom episodes | 1 | **1** |

Byte-identical. The requirement that agreement stay at or above the P0 level is
met with nothing to spare and nothing lost: on this bag the FRONT contract is
LiDAR-driven, and the camera gates do not touch LiDAR observations.

### The tracks do change, and one of the changes is a cost

| | before (9f4a9353) | after (Q2) |
|---|---|---|
| published tracks per cycle, mean | 2.03 | 1.84 |
| published track instances | 759 | 689 |
| role front / left / right | 273 / 360 / 126 | 273 / 360 / **56** |
| cycles with a LEFT wall | 89.0 % | **89.0 %** |
| cycles with a RIGHT wall | 30.5 % | **15.0 %** |
| camera-only published tracks | 0 | 0 |

**Right-wall coverage halves.** This is the price Q1 measured and predicted, and
it is not a surprise: a corridor wall seen from a car driving along it is nearly
radial by construction, so a 25 deg ray-angle test removes exactly those
segments. The 70 lost track instances were all `right`; every published track on
this bag is LiDAR-sourced either way, so what was lost is tracks that were
**born** from a stereo segment and later picked up LiDAR hits — the LiDAR line
alone was shorter than `min_span_birth_m` and could update a track but not start
one.

Left-wall coverage is unchanged at 89.0 %, and the FRONT role is unchanged at
273 cycles, so nothing the control path reads moved.

**The lever, if this trade is wrong for a mission:** lower
`camera_seg_min_ray_angle_deg`. 0 disables the test. The spurious-segment
benefit is on the camera source itself and was measured in Q1: 437 of 536
segments dropped, spurious per frame 0.932 -> 0.112 (8x), residual at
1.5-2.5 m unchanged (0.131 -> 0.134 m), median angle error 5.70 -> 2.46 deg.

### Timing — cheaper, not dearer

| | p50 | p95 | max |
|---|---|---|---|
| before | 29.09 ms | 39.93 ms | 53.28 ms |
| Q2 | **23.79 ms** | **32.99 ms** | **39.99 ms** |

5.3 ms off the median tracker cycle: the gates reject before the association
runs, so the Hungarian assignment has fewer columns.

## Tests

`test_camera_gates.py`, 18 new tests: a near-radial segment rejected and a real
oblique wall kept; the complementary case stated (an exactly radial line passes
0.23 m from base_link, so `stereo_min_rho_m` catches it first — the pair is not
a hole); the gate never touching a LiDAR observation; rms, flat and short
rejections, with the flat one only where the height was measured; camera-only
confirmation on fresh cycles, refused on stale ones, restored by the flag, and
a LiDAR-backed track unaffected; the camera side zone accepting a corridor
offset and refusing the far side of a hall; both-association and the rectangle
rule that governs it.

`test_wall_markers.py::test_the_label_says_id_role_source_state_dist` renamed
and updated in the same commit as the label change.

Suites: **1115 tests, 3 failures**, all pre-existing lint in `f1tenth_bringup`
(copyright, flake8, pep257 — unchanged since the session began).
