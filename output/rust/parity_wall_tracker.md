# Wall tracker parity: Python vs Rust (main-jazzy-rust R6)

lidar_front_wall_node's multi-wall tracker (`multi_wall_tracker.MultiWallTracker`) and its per-cycle
wall fusion (`wall_fusion`) ported to Rust (`src/rust/f1tenth_wall_tracker_core`, a ROS-free cdylib
with a stateful handle, loaded with ctypes by `f1tenth_perception/wall_tracker_native.py`, selected
by `wall_tracker_impl:=py|rust`, **default py**). linus (i7-9700K, x86_64, Ubuntu 24.04, Python
3.12.3, numpy 1.26.4 apt, scipy apt, glibc libm, system LAPACK 3.12, rustc 1.99.1), 2026-10-23.
Library under test: the installed
`install/f1tenth_wall_tracker_core/lib/f1tenth_wall_tracker_core/libf1tenth_wall_tracker_core.so`
(cargo profile debug, opt-level 3, debug_assertions on; 0 unresolved symbols with `ldd -r`).
Worktree `~/dev_ws/f1tenth_more_r6`, branch `rust-walltracker`, phase start b6909dc1.
Numbers: `output/rust/wall_tracker/*.json`; phase report `output/rust_main/r6.md`.

**Result: bit-identical.** On every bag, every cycle, every published message, every field of
every track and every tracker attribute is the same on both sides, floats compared by their bits
(max |difference| 0.0 everywhere; the 1e-9 tolerance was never needed). Track ids, states, roles,
FRONT choice and reasons, strip and rectangle verdicts and `/perception/virtual_wall` are
identical at every step. The numeric primitives and the fusion's branches are identical on random
sweeps too (0 mismatches). The test suites give the same outcome for every test ID on both
backends.

| Part | Cases | Differences |
|---|---:|---:|
| 8 car bags, the car's own steps (recorded inputs) | 2,859 tracker steps, 10,558 track states, 19,754 messages | **0** |
| corridor10m_20261015T171753, **every scan** (the 2292-scan replay) | 2,292 steps, 11,477 track states, 14,336 messages | **0** |
| 4 camera bags, camera recomputed from depth (`--stereo full`: fusion + rectangle check) | 913 steps, 2,736 track states, 6,386 messages, 218 rectangle cycles | **0** |
| 3 bags with door detection on | 1,277 steps, 4,713 track states, 10,221 messages | **0** |
| Numeric primitives (solve, inv, LSA, hypot, pow, wrap_pi, sum, arange, searchsorted, SVD) | 1,120,028 | **0** (1 expected, below) |
| Fusion and rectangle check on synthetic scenes | 3,000 robust_line + 3,000 fusion + 600 rectangle | **0** |
| colcon test f1tenth_perception + f1tenth_bringup + f1tenth_wall_tracker_core, py vs rust hook | 1,409 test IDs | **0** differ |

## How to run it

```bash
source output/rust_main/env.sh; cd $WS      # env_r6.sh for this worktree; isolated DDS, domain 95
O=output/rust/wall_tracker
# 1. primitives and synthetic fusion cases (~1 min)
python3 tools/rust_parity/wall_tracker/numerics.py --n 200000 --json $O/numerics.json
python3 tools/rust_parity/wall_tracker/fusion_cases.py --n 3000 --json $O/fusion_cases.json
# 2. bags in lockstep (~3 min)
P=tools/rust_parity/wall_tracker/parity_wall_tracker.py
python3 $P --json $O/parity_car.json c1015 c1016 c1019 obst1019 potted1435 person1455 goto1550 mission1021
python3 $P --cycles every --json $O/parity_every.json c1015
python3 $P --stereo full --json $O/parity_full.json potted1435 goto1550 person1455 mission1021
python3 $P --doors --json $O/parity_doors.json c1016 potted1435 goto1550
# 3. the test suites on both backends (~1 min); writes compare.txt + failing IDs vs the baseline
tools/rust_parity/wall_tracker/run_gate_tests.sh /tmp/r6_gate
# tables for this report
python3 tools/rust_parity/wall_tracker/report_tables.py all
```

The bag keys are `tools/rust_parity/wall_tracker/replay_common.py`'s `BAGS`: c1015 c1016 c1019
(`~/bags/corridor10m_*`), obst1019 (`~/bags/corridor_obstacles_20261019T135307`) and four
first_test_campaign missions of 2026-10-21/22 (potted1435, person1455, goto1550, mission1021).
Bags are read only.

## 1. Method

`parity_wall_tracker.py` builds two `LidarFrontWallNode`s in one process (corridor_wall_replay's
`WallReplay`, isolated domain): one with `wall_tracker_impl` py, one with rust. Every cycle feeds
both the same inputs in the same order (scan, odometry, camera profile / WALL cloud / class mask and
depth, as recorded or recomputed), then compares **bit for bit**:

- **messages**: every message the node published that cycle, field by field through
  `message_to_ordereddict`: `/perception/walls`, `/perception/walls/markers`,
  `/perception/wall_observations`, `/perception/lidar_front_wall`,
  `/perception/lidar_front_wall_virtual`, `/perception/virtual_wall`, `/perception/wall_planes`,
  `/perception/doors` (`--doors`). Not the CDR bytes: rmw leaves the alignment padding
  uninitialised, so one `WallObservationArray` serializes to different bytes twice in a row (found
  while building the harness).
- **debug**: `/perception/virtual_wall/debug` as JSON without its timing keys (tracker_ms, rect_ms,
  door_ms), with `wall_debug_tracks` on so the per-track payload (roles, verdicts, front reasons,
  confirm reasons) is in it.
- **state**: the whole tracker after the cycle: every field of every track (x, P, extent, deques,
  obs_ext, role, verify, strip verdict, rect statistics, occlusion, ...) and the tracker attributes
  (`_next_id`, stamps, yaw, odometer, all `n_*` counters, rejections, strip reading,
  `front_status`).

The rust node runs with `F1TENTH_WALL_TRACKER_VERIFY_SYNC=1`: every library call also checks that
the Python mirror (below) still equals the library state. A float is identical when its bits are
(NaN equals NaN; -0.0 differs from +0.0).

Cycle selections: **car** = the cycles the car's node actually ran (the recorded debug payloads),
**every** = every scan of the bag with odometry fed exactly up to the scan stamp
(`replay_common.Replay(exact_odom=True)`: `WallReplay._feed_odom` always feeds at least one sample,
right for the car's sparse steps, but at the scan rate it ran the odometry ahead and left 1560 of
2292 cycles without a pose), **rec** = the camera inputs as recorded, **full** = the camera profile,
WALL cloud and class/depth recomputed from the recorded depth, the only way these bags reach the
rectangle verification.

## 2. Bags

| bag | cycles | camera | doors | tracker steps | track states | messages | cycles differing | max abs diff | front steps | rect cycles |
|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|
| corridor10m_20261015T171753 (c1015) | car | rec | off | 206 | 724 | 1440 | 0 | 0.0 | 0 | 0 |
| corridor10m_20261016T144537 (c1016) | car | rec | off | 703 | 2469 | 4920 | 0 | 0.0 | 495 | 0 |
| corridor10m_20261019T131815 (c1019) | car | rec | off | 657 | 3285 | 4487 | 0 | 0.0 | 481 | 0 |
| corridor_obstacles_20261019T135307 (obst1019) | car | rec | off | 380 | 1254 | 2521 | 0 | 0.0 | 336 | 3 |
| 2026-10-22T14-35-30_mission-straight_until_potted_plant | car | rec | off | 286 | 784 | 1991 | 0 | 0.0 | 11 | 0 |
| 2026-10-22T14-55-23_mission-person_then_potted_plant | car | rec | off | 209 | 125 | 1460 | 0 | 0.0 | 0 | 0 |
| 2026-10-22T15-50-16_mission-go_to_person_1m | car | rec | off | 288 | 1460 | 2033 | 0 | 0.0 | 17 | 0 |
| 2026-10-21T12-25-22_mission-20261021T142038-bf7938.mission | car | rec | off | 130 | 457 | 902 | 0 | 0.0 | 0 | 0 |
| corridor10m_20261015T171753 (c1015) | every | rec | off | 2292 | 11477 | 14336 | 0 | 0.0 | 19 | 0 |
| 2026-10-22T14-35-30_mission-straight_until_potted_plant | car | full | off | 286 | 750 | 1991 | 0 | 0.0 | 11 | 56 |
| 2026-10-22T15-50-16_mission-go_to_person_1m | car | full | off | 288 | 1421 | 2033 | 0 | 0.0 | 0 | 86 |
| 2026-10-22T14-55-23_mission-person_then_potted_plant | car | full | off | 209 | 103 | 1460 | 0 | 0.0 | 0 | 4 |
| 2026-10-21T12-25-22_mission-20261021T142038-bf7938.mission | car | full | off | 130 | 462 | 902 | 0 | 0.0 | 0 | 72 |
| corridor10m_20261016T144537 (c1016) | car | rec | on | 703 | 2469 | 5623 | 0 | 0.0 | 495 | 0 |
| 2026-10-22T14-35-30_mission-straight_until_potted_plant | car | rec | on | 286 | 784 | 2277 | 0 | 0.0 | 11 | 0 |
| 2026-10-22T15-50-16_mission-go_to_person_1m | car | rec | on | 288 | 1460 | 2321 | 0 | 0.0 | 17 | 0 |

Columns: *tracker steps* = cycles with a pose (the tracker stepped); *track states* = tracks
compared field by field, summed over the steps; *front steps* = cycles with a FRONT track;
*rect cycles* = cycles with an informative rectangle check. Up to 9 simultaneous tracks; every
role (left, right, front, unknown) occurs on every corridor bag (role counts per run in the JSON).
With door detection on, `door_detection` ran on all 1,282 cycles but found no door in these bags:
the door runs prove that its inputs (the mirrored tracks, `is_host`, `_wall_bl`) are identical, not
the door logic itself, which stays Python.

## 3. Numeric primitives (`numerics.py`, `numerics.json`)

Each primitive the port reimplements is called through the library's debug exports on random
inputs built to hit its branches, and compared with the Python it replaces:

| primitive | Python reference | cases | mismatches |
|---|---|---:|---:|
| solve2 | `np.linalg.solve` 2x2 (dgesv: dgetrf2's 2x2 arithmetic is dgetf2's, then dgetrs) | 200,000 | 0 |
| inv2 | `np.linalg.inv` 2x2 (dgesv against the identity) | 200,000 | 0 |
| lsa | `scipy.optimize.linear_sum_assignment` (square, wide, tall; small integers, ties everywhere, the tracker's "big" filler) | 20,000 | 0 |
| hypot | `math.hypot` (CPython's own vector_norm with fma, not libm) | 200,009 | 0 |
| pow2 | Python `x ** 2` (libm `pow`, not `x * x`) | 200,009 | 0 (+1 expected: overflow) |
| wrap_pi | `corridor_perception.geometry.wrap_pi` (CPython float `%`) | 200,007 | 0 |
| sum | builtin `sum()` (CPython 3.12 Neumaier compensation) | 50,003 | 0 |
| arange | `np.arange` on the tracker's 5 cm grid | 20,000 | 0 |
| searchsorted | `np.searchsorted` on sorted and unsorted arrays | 10,000 | 0 |
| svd_vt | `np.linalg.svd(M x 2)[2]` (dgesdd path: dgeqr2 + dbdsqr on the 2x2 bidiagonal) | 20,000 | 0 |

What it took (all in `src/rust/f1tenth_wall_tracker_core/src/pyc.rs`, `linalg.rs`, `lsa.rs`,
`lapack.rs`):

- **scipy's `linear_sum_assignment`** (rectangular_lsap.cpp): the shortest augmenting path with the
  remaining columns scanned in scipy's reversed order, its tie rule (a column with `row4col == -1`
  wins a tie), the transpose for tall matrices and the infeasible / invalid checks.
- **numpy on linus calls the system LAPACK 3.12** (`_umath_linalg` links `liblapack.so.3`;
  `np.show_config()`'s "lapack: internal" is misleading). The robust line fit's SVD therefore needed
  LAPACK 3.12's arithmetic: dgeqr2 with BLAS 3.12's **Blue's-algorithm dnrm2** (its scaling
  constants as exact bit patterns), dlarfg / dlarf, dbdsqr's dlasv2 + drot, and gfortran's `SIGN`
  (= `copysign`). A first port after the old LAPACK (classic dnrm2, f2c `d_sign`) differed by up to
  1213 ulp in 3661 of 5000 cases; the 3.12 port is bit-identical on 10,000 + 20,000.
- **numpy's reductions**: `sum(axis=0)` of one contiguous column is pairwise (numpy's 8-way
  unrolled pairwise_sum), of several columns sequential row by row; `(K, 2).mean(axis=0)`
  sequential; netlib BLAS's two- and three-term dot orders for `@`; `argmin` first index;
  `searchsorted` with numpy's carried-over bounds.
- **float32**: the rectangle check runs on float32 images; a Python scalar meets a float32 array as
  float32 (comparisons included), a float32 array meets a float64 array in float64.
- **CPython**: float `%`, `max`/`min` argument order with NaN, `x ** 2` through libm `pow` (kept
  out of the optimizer with `black_box`), `math.atan2`, `math.hypot`, `sum`.

## 4. Fusion and rectangle check on synthetic scenes (`fusion_cases.py`, `fusion_cases.json`)

The bag replays compare the fusion end to end on what the car saw; this sweeps the branches a bag
reaches rarely, against the Python chain `_publish_observations` runs (compensation, projection,
`cross_check_profile`, `observations_from_lines`, `stereo_only_observations` over the WALL cloud),
written with the module functions themselves:

| part | cases | mismatches |
|---|---:|---:|
| `lidar_front_wall._robust_line` (noise, outliers, one kept hypothesis, duplicates, exact lines) | 3,000 | 0 |
| fusion cycle (random mount error; AGREE / DISAGREE / veto / STEREO_ONLY / LiDAR-only walls, profile gaps and heights, compensation on/off, stereo weight in [0, 1]) | 3,000 | 0 |
| `verify_rectangles` (random uint8 class / uint16 mm depth images, zeros, random segments in and out of the frame and range) | 600 | 0 |

Coverage of the 3,000 fusion cases: 69,904 columns AGREE, 29,418 LiDAR-only, 208,460
stereo-only, 37,727 DISAGREE; 4,582 observations (1,752 with both sensors, 451 stereo-only), 366
rejected; 3,326 rectangle segments, 939 informative.

## 5. The tests on both backends

`F1TENTH_WALL_TRACKER_IMPL=rust` is the test hook: `MultiWallTracker(...)` then returns the native
class and every `lidar_front_wall_node` in the tests builds the Rust tracker.
`run_gate_tests.sh` runs colcon test of f1tenth_perception, f1tenth_wall_tracker_core and
f1tenth_bringup once per backend:

| | py | rust |
|---|---|---|
| f1tenth_perception | 1319 passed | 1319 passed |
| f1tenth_bringup | 87 passed, 3 failed | 87 passed, 3 failed |
| f1tenth_wall_tracker_core (cargo_test.xml: unit, fmt) | 2 passed | 2 passed |
| test IDs with a different outcome | | **0** of 1,409 |
| failing IDs not in `output/jazzy_main/test_failing_ids.txt` | 0 | 0 |

The 3 bringup failures are baseline (test_copyright, test_flake8, test_pep257; the three changed
bringup files have the same ament_flake8 / ament_pep257 counts as at HEAD). Under the hook, **201
tests build a native tracker** (counted with a pytest plugin: test_multi_wall_tracker 76,
test_lidar_front_wall_node 64, test_wall_confirm_level 23, test_camera_gates 17,
test_door_detection 10, test_wall_tracker_backend 8, test_wall_confidence_range 3;
`output/rust_main/r6_native_test_counts.txt`). The wall_fusion function tests (test_wall_fusion,
test_wall_rectangle) call the Python functions directly; the native fusion is reached by 3 node
tests and covered by section 4 and the camera bags. The rest of the wall tests (door markers, wall
markers, front clearance, wall distance) read tracks or messages, not the tracker.

## 6. Expected differences (documented, none reachable on the car)

- An overflowing `x ** 2` (|x| > 1e154): CPython raises OverflowError, the port returns inf (the
  squared quantities are sigmas and turn rates). The 1 "expected" pow2 case above.
- A 4-tuple scan whose cut mask is None: the Python's negative evidence raises, the port reads it as
  no cut (as the strip and the open-gap tests already do in both). The node never passes None.
- Sorting with NaN keys: the port is a stable sort, identical for every NaN-free key; timsort's
  order with NaN keys is not reproduced (not observed: every bag replay and every test is identical on both).
- Python object identity only: a track pushed from Python whose deques were replaced by lists comes
  back as deques; a hand-made track with an id the tracker later issues is the same mirror object.
- The rust fusion path decodes the WALL cloud once per message (cached by message identity), the py
  path every cycle: `read_points_numpy` is deterministic, only the cost differs.
- **Platform**: bit parity is measured on linus x86_64 with numpy on the system reference LAPACK /
  netlib BLAS and glibc libm. On aarch64 (Thor, Orin) with another BLAS/LAPACK or libm the Python
  reference itself changes in the last bits; the Rust port keeps the reference arithmetic. Not
  measured there (R7).

## 7. Timing (linus, tracker_ms, ms)

The node's own `tracker_ms` (extraction to front_track, as the car logs it), uninstrumented,
`WALL_DEBUG_TRACKS=0` (the car default), each cycle's minimum over 3 identical replays
(`profile_tracker.py --repeat 3`; `timing_{py,rust}_{car,every,full}.json`). The rust number
includes the ctypes calls and the mirror update.

| bag | cycles | camera | steps | py p50 | py p95 | rust p50 | rust p95 | p95 py/rust |
|---|---|---|---:|---:|---:|---:|---:|---:|
| corridor10m_20261015T171753 (c1015) | car | rec | 206 | 7.74 | 9.53 | 2.66 | 3.41 | 2.8x |
| corridor10m_20261016T144537 (c1016) | car | rec | 703 | 8.73 | 12.44 | 3.15 | 4.08 | 3.1x |
| corridor10m_20261019T131815 (c1019) | car | rec | 657 | 5.01 | 7.96 | 1.50 | 1.84 | 4.3x |
| corridor_obstacles_20261019T135307 (obst1019) | car | rec | 380 | 3.59 | 4.16 | 1.62 | 1.83 | 2.3x |
| 2026-10-22T14-35-30_mission-straight_until_potted_plant | car | rec | 286 | 8.24 | 11.65 | 3.19 | 3.97 | 2.9x |
| 2026-10-22T14-55-23_mission-person_then_potted_plant | car | rec | 209 | 6.65 | 7.28 | 3.01 | 3.32 | 2.2x |
| 2026-10-22T15-50-16_mission-go_to_person_1m | car | rec | 288 | 9.61 | 12.38 | 3.38 | 4.15 | 3.0x |
| 2026-10-21T12-25-22_mission-20261021T142038-bf7938.mission | car | rec | 130 | 8.50 | 11.99 | 2.81 | 3.76 | 3.2x |
| corridor10m_20261015T171753 (c1015) | every | rec | 2292 | 8.39 | 10.07 | 2.57 | 3.31 | 3.0x |
| 2026-10-22T14-35-30_mission-straight_until_potted_plant | car | full | 286 | 8.40 | 11.72 | 3.41 | 4.19 | 2.8x |
| 2026-10-22T15-50-16_mission-go_to_person_1m | car | full | 288 | 10.44 | 12.80 | 3.76 | 4.45 | 2.9x |

The whole `/scan` callback (publishing included), p50 / p95: c1016 11.09 / 14.98 -> 5.42 / 6.58,
c1019 7.14 / 10.55 -> 3.80 / 4.50, goto1550 full 12.64 / 15.13 -> 5.96 / 7.01. The rust tracker_ms
in the parity runs (section 2's JSON) is higher than here because those runs verify the mirror on
every call and publish the debug track payload.
