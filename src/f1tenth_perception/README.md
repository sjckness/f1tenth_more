# f1tenth_perception

Nodes, and what each one's output actually is. Written for the `d_wall` pass;
not yet a complete package README — the older nodes carry their documentation in
their own module docstrings and this file does not duplicate them.

## The signals that are all called something like "clearance"

Getting these confused is a **documented recurring error** in this stack
(`front_clearance_node.py:36-56`, `check_stop_condition.py:25-47`). There are
now six, and only two of them are a distance to a wall.

| topic | node | what the number is |
|---|---|---|
| `/perception/d_wall` | `wall_distance_node` | **SIGNED perpendicular distance** [m] to one tracked wall, **left positive**. A lateral offset. |
| `/perception/d_wall/swept_arc` | `wall_distance_node` | **rear-axle ARC LENGTH** [m] before the body rectangle contacts the tracked wall. Metres of *travel*. Debug only. |
| `/perception/swept_clearance` | `swept_clearance_node` | the same arc-length quantity, from live LiDAR+ZED points. |
| `/perception/lidar_front_wall` | `lidar_front_wall_node` | **UNSIGNED** perpendicular distance, forward sector only. |
| `/perception/front_distance` | `front_clearance_node` | camera, straight ahead, **wall only** (objects masked out). |
| `/perception/front_clearance` | `front_clearance_node` | camera, straight ahead, min with obstacles. |

Not in this package but in the same confusion: `/mpc/wall_track`'s `d_wall` is
an **unsigned magnitude** (direction carried separately in `normal_yaw`) and is
MPC-internal; `/costmap/front_clearance` is SLAM-derived.

**An arc length is not a gap.** Straight ahead, `swept_arc` is the distance to
the front bumper (0.443 m ahead of the rear axle), not to the axle. A wall abeam
at 0.6 m reports the 5 m horizon, not 0.6 m. Anything reading either swept value
as a distance-to-wall is wrong.

---

# UNVALIDATED

**The LiDAR was powered down for the whole of the `d_wall` pass. Nothing below
has ever run on the car.** The code builds, and 72 unit tests plus an offline
replay against real archived scans pass. That makes it **not yet falsified**. It
does not make it working, and nothing in this section should be described as
verified until a fixture or a powered run has actually exercised it.

Work through `docs/bringup_checklist.md` in order. **Stage 2 — the physical
left/right sign check — gates everything downstream**, and it is the one check
no offline test can perform.

## 1. Sign conventions against physical left and right

The unit tests (`test_wall_distance.py::TestSignOnBothSides`) assert internal
consistency with the convention `WallDistance.msg` documents, on both sides,
including invariance to the fit's arbitrary normal flip. **They cannot catch the
convention itself being backwards.** If it is, the correction steers
*confidently into the wall* rather than away from it, and it does so smoothly
and within saturation, which is exactly what makes it hard to spot.

Two live hazards behind it:

- The workspace has **two `Line` classes with opposite signed-distance
  conventions**: `mpc_controller/wall_tracker.py:201` returns
  `c - n·p` ("positive before it") and `f1tenth_perception/glass_detect.py:443`
  returns `n·p - c`. Exact negations. `wall_distance.py` uses glass_detect's and
  negates deliberately; a future edit that switches source must re-derive it.
- `wall_tracker._gate` reorients `n` from the car toward the line before
  anything reads it. **`glass_detect` does no such normalisation** — `tls_line`
  returns whatever sign the SVD produced.

## 2. `swept_clearance_node` — registered, still never run

It had an entry point, a launch file and unit tests since it was written, but
appeared in **no** `components.yaml` entry and in **no** other launch file, so
it had never run on the car. The `d_wall` pass registered it as its own
`swept_clearance` component. **Registering it does not validate it**, and after
this pass it still has not run. Treat its first powered run as validation, not a
formality. Its geometry is unit-tested (`test_swept_corridor.py`); its ROS glue,
its TF chain to the ZED optical frame, its fusion staleness behaviour and its
CPU cost are not.

## 3. The phase machine against real `/mpc/wall_track`

**No bag in the archive contains `/mpc/wall_track`** — all 14 `wall_turn` and
`drive_turn_180` runs predate the wall tracker, and `/perception/front_distance`
and `/mpc/goal_drive` are absent from every one of them too.
`tools/wall_distance_replay.py` therefore *synthesises* the feed from the bag's
real `/scan` and `/odometry/filtered` at the real tick timing. That gives real
geometry, real odometry bias and the real 10 Hz cadence. It does **not** give:

- real turn-exit **silence** — the replay's silence is manufactured by its own
  stop rule, so it is clean by construction, which real silence is not;
- real dropped messages or real jitter on `/mpc/wall_track`;
- real `/mpc/hold` interleaving (the bags do carry `/mpc/hold` and the replay
  reads it, which is the nearest available proxy).

Capturing a bag **with** `/mpc/wall_track` is Stage 4, and is the highest-value
thing the first powered session produces.

## 4. OPEN DESIGN QUESTION: the coast cap can make `CORRIDOR` unreachable

This one is not a gap in testing, it is a conflict between two requirements, and
it was found in the replay rather than reasoned about in advance.

The phase machine enters `CORRIDOR` only on a clean `COMMITTED → silence`
transition **with a valid track held throughout** — deliberately strict, because
silence is overloaded five ways and a consumer that reads it as "turn finished"
fires on a safety hold mid-turn.

But the coast caps (`max_coast_distance` 0.5 m, `max_coast_yaw` 0.35 rad) are
set by the **measured** odometry bias, and at that quality a 90° turn genuinely
cannot be coasted end to end. Replayed on
`2026-09-10T15-33-46_mission-wall_turn`, one 90° turn produced **6 `track_id`s
at the shipping caps and 1 with the yaw cap lifted** — each cap firing sets
`valid=false` for a tick, which clears "held throughout".

**Consequence: on that turn, `CORRIDOR` could not have been reached even with a
move after the turn, so the correction would never have been applied.**

Three ways out, none chosen here, because weakening a safety condition is not a
call to make from replay data alone:

1. **Leave it.** The correction simply does not fire unless the wall is
   re-observed densely enough through the turn. Safe, possibly inert. Note the
   geometric candidate source already re-observed the wall on 39 of 180 ticks in
   that replay, so denser re-observation on the real car is plausible.
2. **Relax the condition to "same `track_id` at exit as at commit."** Strictly
   stronger than "valid at exit", strictly weaker than "valid at every tick",
   and it preserves the actual intent — the "throughout" rule exists to catch a
   track that re-associated to a different surface, and `track_id` already
   detects exactly that.
3. **Raise `max_coast_yaw`** — only legitimate after the VESC speed scale and
   gyro gain are fixed, which is the note attached to the parameter.

Stage 4's bag is what should settle it: it shows how often the wall is actually
re-observed through a real turn.

## 5. Coast behaviour through an actual turn

The caps are exercised offline against **synthetic** odometry paired with real
scans — and simulated drift is precisely the thing that is wrong about it. The
real bias (distance 17–20% short, gyro yaw gain 0.93–1.09) is what set the cap
values, but the caps have never been observed firing on live odometry.

## 6. Glass detection beyond ~1.1 m

Every glass measurement in `glass_detect.py`'s docstring comes from **one
session, 2026-09-14, parked 1.10 m from a clear pane**. That capture showed the
pane returning on essentially every forward beam out to 60° of incidence — which
contradicts the theory, and is credited to short range plus surface
contamination. The signal budget at 3–4 m is ~13× smaller. **It has not been
retested there, and the pane may well vanish**, which matters because the
`wall_turn` commit distance is around 4 m. Stage 3 answers it, with an open
doorway at 2/3/4 m as the discriminator control.

Parameters marked `# real, 1.1 m only` in `stack_params.yaml` are legitimate at
that range and nowhere else. Everything marked `# SYNTHETIC — retune on
hardware` was chosen by reasoning, not fitted to anything.

## 7. `glass_detect` alone tracks nothing on an opaque wall

Measured, and it changed the design: `glass_detect.detect()` returns a surface
only when it carries transparency evidence — it begins `if not voids: return []`
— so an ordinary opaque wall produces no candidate. On 40 real scans of an
office corridor it accepted **4** candidates against a confirmation bar of 6 in
8, and the node tracked nothing for the whole run. Hence
`wall_distance.geometric_candidates`, and hence `provenance` distinguishing
`GEOMETRIC` from `GLASS_CONFIRMED`.

What is unvalidated is the **gates** on that geometric source
(`wall_distance_geom_*`, all SYNTHETIC): whether `geom_min_inliers` 40,
`geom_min_span_m` 1.0 and `geom_min_distance_m` 0.5 select the wall a human
would point at, in a room that is not the one archive corridor they were checked
against. In particular, initial selection among several confirmed walls is
"nearest", which is arbitrary when the car has not yet turned.

## 8. CPU, and `taskset` contention on core 5

**Measured offline, on 300 real 1081-beam scans, on this Jetson, single core,
unpinned and with nothing else running:**

| | |
|---|---|
| full per-tick path (glass detect + geometric fit + track + observe + swept_arc) | **mean 26.8 ms**, median 26.2, p95 32.8, max 75.7 |
| budget at `publish_rate_hz` 10.0 | 100 ms → **26.8% of one core** |
| the same path at the 40 Hz `/scan` rate | 25 ms budget → **107% of one core** |

That last row is why `_scan_cb` only *stores* the newest scan and the fit runs on
the publish tick. Fitting per scan does not work on this hardware.

**Unvalidated:** this is a benchmark loop, not a running node — no DDS, no
executor, no `taskset`. `wall_distance_node`, `lidar_front_wall_node` and
`swept_clearance_node` now all pin to **core 5**, and the third of those has
never run at all. Core 5 is chosen because it is the one core no other pinned
node reserves, and specifically **not** core 4, which hosts
`behavior_executor_node` and the e-stop. Three nodes on one core, one of them
costing 27% on its own, is the number to check in Stage 0.

## 9. `d_wall`'s sign is undefined for a wall dead ahead

`signed_wall_offset` returns a magnitude equal to the perpendicular distance and
a sign from which side the perpendicular foot falls on. For a wall whose normal
lies along the heading, that sign is `copysign` of a zero — deterministic but
meaningless. `heading_rel` is published so a consumer can gate on it
(`|heading_rel|` near π/2 means the wall is in front, not beside). The
correction only runs in `CORRIDOR`, by which point a `wall_turn`'s front wall is
abeam, so this should not arise — **should not** being the unvalidated part.

---

## Other things found that are not about this pass

- **`glass_detector_node.py` does not exist.** `glass_detect.py`'s own docstring
  says "glass_detector_node.py is the ROS glue". There is no such file, no entry
  point, no launch file and no component. `wall_distance_node` is the module's
  first and only runtime consumer, in-process.
- **`f1tenth_perception` has no ament lint tests**, unlike `f1tenth_bringup`
  (whose `test_copyright`, `test_flake8` and `test_pep257` were already failing
  before this pass: 5 flake8 errors, 94 pep257 errors in
  `component_supervisor_node.py` alone, and no file carries a copyright notice).
