# Front-wall lock for "approach the wall": design

2026-10-21, branch `campaign-final`. Triggered by `P002-R029`/`R031` review
(see chat): the `front_clearance` stop_condition has no concept of "the wall I
was sent to approach" — it just asks a single live estimator "is something
close, dead ahead, right now." This proposes closing that gap by mirroring a
pattern the codebase already has for objects, not inventing a new one.

Sections 1-5 are the design as written before any code. What was built, and
where it departs, is in the implementation notes (2026-10-21) and the two
2026-10-22 sections at the end: the deceleration ramp now aims at the locked
wall, and every wall consumer shares one confirmation level.

## 1. What's wrong today

`front_clearance` (`condition_eval.py:336-416`) reads `ctx.front_clearance`,
filled by `check_stop_condition.py:310`'s subscription to
`get_wall_distance_topic()` (`param_defaults.py:87-113`), which today is
always `/perception/virtual_wall` — one number from `lidar_front_wall.py`'s
`WallEstimator`.

`WallEstimator.step()` (`lidar_front_wall.py:299-416`) already does more than
"instantaneous": `fit_forward_line` (`:186-221`) re-fits a line in the body-
frame forward sector every scan (so "front" rotates with the chassis, not
fixed to a heading), then `_predict()` (`:342-358`) dead-reckons the distance
from the last ACCEPTED fit's world-frame normal, and a "wrong-surface gate"
(`:367-379`, ±0.30 m) rejects a new fit that disagrees with that prediction.
A `Reacquisition` event (`:259-269`, emitted `:384-393`) is already logged
whenever a fit is accepted after a non-measured step.

The gap: the anchor that gate checks against is the **last accepted
measurement**, re-anchored forward on every acceptance ("snap, never blend").
Nothing anchors it to the wall that was actually in front when the mission
move *started*. A chain of individually-small, individually-accepted jumps
(each <0.30 m from its immediate predecessor) can walk the "front wall" meters
away from the original target with no single step ever tripping the gate —
confirmed in `P002-R029`, where the MPC's own `mpc_corr` target drifted ~2 m
over a 36° avoidance swerve. The stop condition doesn't care *which* wall it
fires on, only that *something* is close and roughly where the last accepted
fit said it'd be.

There's also no persistent per-wall ID anywhere in this path: `lidar_front_
wall.py` tracks exactly one scalar (distance, normal), not "wall #3".

## 2. The precedent already in this codebase

The object-following mission (`go_to_object.py` + `object_handler.py`) solves
the identical problem for objects, and it's tested and in production:

- A move locks onto one `track_id` (`go_to_object.py:92` `follow_gate_m=0.5`,
  passed through to the handler).
- If that ID is lost, `object_handler.py:200` re-acquires with
  `nearest_track(usable, p.target_class, self.target_xy, within=p.
  follow_gate_m)` — the nearest still-confirmed track of the right class
  **within a distance gate of where the old target was**, default 0.5 m
  (`object_handler.py:80`). Anything farther is not accepted as "the same
  thing", mission holds/treats it as absent instead.

This is exactly the "lose it, but only take a new one if it's at a sensible
distance" rule the chat proposed — already built, just for objects, not
walls.

## 3. Proposed mechanism for walls

Reuse the same shape instead of inventing a second one:

1. **Turn on an ID-bearing wall source.** `multi_wall_tracker.py` already
   maintains up to 4 Kalman-filtered wall tracks with IDs and a
   TENTATIVE → CONFIRMED → COASTING lifecycle (`:238-242`), and already
   assigns a geometric `FRONT` role with hysteresis
   (`role_with_hysteresis`, `:1007-1024`). It's wired but inert:
   `enable_scene_labels` is FALSE by default ([[scene-labels-state]]). This
   is the component to turn on, not a thing to build.
2. **At move start**, when `front_clearance`'s move is dispatched, read
   whichever CONFIRMED track currently holds the `FRONT` role and record its
   `track_id` and current `(distance, world_normal)` as the move's anchor —
   the same moment `go_to_object` records its `target_xy` lock.
3. **Each tick**, prefer that same `track_id`'s live distance if it's still
   CONFIRMED and still holds (or is eligible for) `FRONT`. This is the
   "tracking the chosen wall" half of the proposal.
4. **On loss** (COASTING past its grace period, or dropped), re-acquire: pick
   whichever CONFIRMED track now holds `FRONT`, but only accept it if its
   distance is within a `wall_follow_gate_m` of where the *locked* track's
   dead-reckoned prediction says the original wall should now be — not of
   the last accepted single-scan fit. Compute that prediction the same way
   `_predict()` already does (`lidar_front_wall.py:342-358`), just rooted at
   the move-start anchor instead of continuously re-anchored. Outside the
   gate: hold (same fail-safe direction `condition_eval.py:370-377` already
   argues for), don't stop, don't silently adopt a different wall.
5. **`condition_eval.py`'s `front_clearance` branch** then reads this locked
   track's distance instead of (or as a cross-check against) the raw
   `/perception/virtual_wall` scalar — same two-arm logic (`:382-416`)
   unchanged, new source.

`wall_follow_gate_m` plays the same role as `follow_gate_m` above; starting it
at the existing `lidar_front_wall.py` wrong-surface gate's 0.30 m (measured
from real reacquisition data, see that file's comment at `:370-375`) is a
reasonable default rather than picking a new number blind.

## 4. Why not just widen the existing single-estimator gate

Tried mentally and rejected: `WallEstimator` has no identity, only a scalar
anchor. Anchoring it to "move start" instead of "last accepted fit" would fix
the drift-by-small-steps problem but still can't express "this is specifically
wall #3" — if two walls' predicted distances cross during a maneuver, the
single-estimator design has no way to tell them apart, which is exactly the
lateral-wall-mistaken-for-front-wall scenario this was raised to prevent. The
multi-track path can.

## 5. Open questions / risks

- `enable_scene_labels` is off because the stereo half isn't validated on a
  depth-carrying bag yet ([[scene-labels-state]]). Turning it on for this
  feature inherits that open risk — needs a bag with depth before trusting it
  for a stop decision, not just a viz overlay.
- Need to confirm `multi_wall_tracker`'s `FRONT` role assignment behaves
  sanely mid-avoidance-swerve (large, fast yaw change) before trusting it as
  the sole re-acquisition source — not yet checked against real data.
- `condition_eval.py` is documented as deliberately pure/stateless
  (`:341-344`); the per-move lock state (track_id + anchor pose) has to live
  in `check_stop_condition.py` (which already owns move-scoped state) and get
  passed in via `EvalContext`, not inside `evaluate()` itself.
- Does not touch `mpc_controller/wall_tracker.py` (the separate tracker
  behind the MPC's `wall_turn` maneuver) — that's a different consumer of
  "where's the wall" and out of scope here unless it turns out to be the
  thing that actually produced R029's drifting target, which wasn't confirmed
  (see [[front-clearance-no-wall-lock]]).

---

## Implementation notes (2026-10-21)

Implemented in `f1tenth_behavior/mission/wall_lock.py` (the rules, no ROS) and
`behaviours/check_stop_condition.py` (FRONT-WALL LOCK: the inputs, the
per-move reset, the logging), behind `wall_lock_enabled` (default true) with
`wall_follow_gate_m` 0.30 and `wall_follow_gate_deg` 15 in `stack_params.yaml`.
`evaluate()` is unchanged: the caller passes the locked wall's distance, or
None for "not yet".

### What the code said that the design did not

- **The tracker is already on.** `enable_scene_labels` and `use_wall_tracker`
  have been `true` on this branch since 2026-10-14, and
  `/perception/virtual_wall` is already the multi-wall tracker's FRONT track
  (where the travel strip meets its measured segment, from the bumper;
  `lidar_front_wall_node._publish_virtual`), not `WallEstimator`'s scalar.
  Step 1 was a no-op, and the first risk in §5 is not new: the stop has read
  the tracker since 10-14. What was missing is only the identity.
- **The track id of the published value is on the wire.**
  `/perception/virtual_wall/debug` carries `state`, `distance_m` and
  `front_id` from one publish. The lock reads all three from there, so the
  value is known to belong to that track. Taking the id from
  `/perception/walls` and the value from the Float32 would pair two scans
  across a front switch, which is exactly the tick that matters.
- **The deployed `object_follow_gate_m` is 1.5 m**, not 0.5 (raised
  2026-09-23, P004-R003), and FOLLOW gates against the LAST target point, not
  the move-start one.

### Where the implementation departs from §3, and why

- **The distance read is the contract value, and only while the locked track
  is FRONT** (step 3's "or is eligible for" is not used). While the locked
  track is tracked but something else is FRONT, or nothing is, the stop reads
  "not yet". No second metric (perpendicular distance) is mixed in, so a move
  that never changes front wall behaves exactly as before, and the
  deceleration ramp lands on the same number the stop reads.
- **The lock is taken at the first LiDAR-backed FRONT of the move**, not on
  its first tick and not on any FRONT. A move that starts with the wall out of
  range (CLEAR) locks the first wall that becomes FRONT; until then the stop
  reads the raw topic, as before. A STEREO-only FRONT (`MultiWallTrack.source`,
  "nothing has ever cross-checked this wall") is never locked. That rule came
  from replaying the first version on a real bag (below): it locked a
  stereo-only phantom and would have refused the real wall. If the first
  lockable FRONT appears mid-swerve it is the wrong wall, and the lock cannot
  know.
- **The anchor follows the locked track's own CONFIRMED estimate**, not the
  move start. The drift §1 describes comes from re-anchoring on any accepted
  fit, with no identity; here only the tracker-associated track itself, or a
  candidate that has just passed the gate, moves the anchor. A frozen
  move-start anchor would refuse the right wall: `/odometry/filtered` reads
  distance ~17 % short, so a static wall's odom line moves ~0.17 m per metre
  driven and leaves a 0.30 m gate after ~1.8 m of approach. The gate is still
  evaluated as `_predict()` does: the anchor line, static in odom, seen from
  the current bumper.
- **An angle gate (15°) next to the distance gate.** A one-number gate cannot
  refuse a side wall at the same distance, which is §4's case. **15° is NOT
  MEASURED**: far above line-normal noise and one move's yaw drift, far below a
  side wall's 90°. See the parameter table below.
- **"Grace" is the tracker's own FRONT coast caps** (0.5 s / 0.3 m of path),
  read from the same keys.
- **"Hold" means the stop reads "not yet"** (`condition_eval.py`'s fail-safe
  for an absent reading), not `/mpc/hold`. The car keeps driving under the
  MPC, and the move stays bounded by its `timeout_sec`.

### Parameters

| key (`stack_params.yaml`) | value | where it comes from |
|---|---|---|
| `wall_lock_enabled` | true | switch. false: the stop and (since 2026-10-22) the ramp read `/perception/virtual_wall` alone, as before 10-21 |
| `wall_follow_gate_m` | 0.30 m | `lidar_front_wall_wrong_surface_gate_m`: the valley of WallEstimator's recorded innovation histogram (same-wall re-acquisitions -16/+6 cm p10/p90). Measured, but on the single-scalar estimator, not on tracker re-acquisitions |
| `wall_follow_gate_deg` | 15° | **NOT MEASURED.** Chosen between the line-normal noise (0.4-0.8° 1-sigma) plus one move's odometry yaw drift, and the 90° of a side wall. The two re-acquisitions on record are 1.7° and 1.1° (corridor10m_20261019T131815). To measure it: a bagged swerve (the M02 run with a person in the path), replayed with `tools/wall_lock_replay.py`, which prints the angle of every re-acquisition and refusal |
| coast caps | 0.5 s / 0.3 m | `lidar_front_wall_wall_tracker_front_max_coast_sec` / `_m`, the tracker's own FRONT caps, read from the same keys |

### Replays (`tools/wall_lock_replay.py`, 2026-10-21)

Every recording on linus with `/perception/walls` and a debug payload that
names its FRONT track (10-16 onward). None has an avoidance swerve: R029 and
R031 were not bagged (only the harness CSVs exist on the Orin, no wall data).

| recording | what it is | lock | 2.0 m stop, raw vs locked |
|---|---|---|---|
| `2026-10-21T12-25-22_mission-…bf7938` | M02 approach the wall, COMPLETE | 404 (LiDAR+stereo) at 2.31 m | same steps |
| `corridor10m_20261019T131815` | 20 m approach to the end wall, COMPLETE | 4 → 103 (0.01 m / 1.7°) → 128 (0.15 m / 1.1°) | same step, 1.734 m |
| `2026-10-18T13-12-20` / `13-28-17_mission-drive_test_straight_2m` | sim | 2, whole run | neither fires |
| `corridor_obstacles_20261019T135307` | car stationary | 976, whole run | neither fires |

**The first version locked any FRONT, and it failed on bf7938.** Track 321,
stereo-only and 23° oblique, was FRONT for two tracker steps at 4.6 m, then
nothing was FRONT for twelve seconds (`low_confidence`), then the real wall
came in as 404 at 2.31 m: 0.88 m / 25.7° off 321, refused. The mission, which
completed, would have run into its timeout. Hence the stereo-only rule.

The 10-14 / 10-15 mission bags predate `front_id` in the payload (the lock
never engages there) and their `/perception/walls` no longer decodes.

**Re-run 2026-10-22: the corridor10m row does not reproduce as "same step".**
The same tool, from `night-start/campaign-final`, prints raw 78.52 s /
1.734 m against locked 77.14 s / 1.98 m on that bag. The difference is the
recording, not the lock: the tool pairs `/perception/virtual_wall` and its
debug payload by bag time, and the bag holds 716 Float32 messages against
730 payloads, written in bursts (369 of the 730 payloads follow a Float32
that is not theirs). The Float32s carrying 1.98 m and 1.89 m are not in the
bag, so "raw" first reads under 2.0 m at 1.734. Live, the stop reads the
Float32 of the same publish, 1.98 m, on the same step as the locked value.
The tool's docstring now says so.

### Not done

- ~~**The MPC's deceleration ramp still ramps on the raw
  `/perception/virtual_wall`**~~ Done 2026-10-22: see "The ramp aims at the
  locked wall" below.
- **Not run on the car, and never on a swerve.** The case the lock exists for
  is covered by unit tests only (`test_wall_lock.py`,
  `test_wall_lock_behaviour.py`). The next M02 run with a person in the path
  should be bagged (the mission logger already records both topics) and
  replayed with the tool.
- §5's second point (FRONT behaviour mid-swerve on real data) is still open,
  for the same reason.
- **A different wall nearer than the locked one is refused too**, by the
  0.30 m gate, not only a side wall. A partition or a closed door across the
  path after the lock is taken ends the move on its timeout, where it used to
  end as "arrived". Since 2026-10-22 the ramp does not slow for it while the
  locked wall is still tracked behind it (see the next section); it does once
  the locked wall is lost.

---

## The ramp aims at the locked wall (2026-10-22, branch `night-walls`)

The R029 gap left above: the lock stopped the false "arrived", but the
deceleration ramp kept reading the raw `/perception/virtual_wall`, so in a
swerve it still braked the car to rest by the side wall and latched there.
Now, with `wall_lock_enabled`, the ramp aims at the locked wall.

**The wire.** The BT keeps every rule of the lock; the MPC only reads its
answer, so the two can never disagree on which wall is locked.

- `CheckStopCondition` publishes `/mpc/stop_wall_id` (`std_msgs/Int32`) on
  every tick it runs the lock: `wall_lock.ramp_wall_id(step)`, the locked
  track id in phases `locked`, `not_front` and `no_reading`, -1 in `unlocked`
  and `lost`. Nothing in the lock's decisions changed; the topic carries them.
- `MPC_corr` subscribes to it and to `/perception/virtual_wall/debug` (only
  with `wall_lock_enabled`) and asks `mpc_controller/ramp_wall_source.py`
  (pure, no ROS) for the distance on every tick of a wall stop
  (`_ramp_wall_distance`, read by `_wall_stop_d_rem`, so both the speed cap and
  the latched hand-over use it).
- The distance comes from the debug payload, not from the BT: `state`,
  `distance_m` and `front_id` of one publish, the same pairing the lock relies
  on, at the same latency as the raw Float32. The BT's own value would add up
  to one BT tick (0.1 s) on top.

**The cases** (`select_ramp_distance`; tested in
`mpc_controller/test/test_ramp_wall_source.py`):

| lock state | the ramp aims at |
|---|---|
| `wall_lock_enabled` false | `_fresh_front_distance()`, exactly as before; neither topic is subscribed |
| not locked yet (-1) | the raw wall distance, as before |
| locked, its wall FRONT | the payload's `distance_m`: the locked track's contract value, the number the stop reads. A move that never changes front wall ramps on the same value as before |
| locked, another wall (or none) FRONT | nothing: no wall ramp this tick (the swerve) |
| locked wall lost (-1) | the raw wall distance until the BT re-acquires under its gates. Conservative: a lost lock can only brake more |
| no fresh `/mpc/stop_wall_id` (> `wall_distance_max_age_s`), or locked without a usable payload | the raw wall distance (conservative) |

**Why "nothing" while another wall is FRONT.** The ramp is deceleration
toward the wall the move was sent to, and that wall is not in the strip.
Another wall or obstacle in the way stays with what already handles any wall
that is not FRONT: the solver's wall rows (a confirmed front/left/right wall
on `/perception/walls` is a hard row whether or not it is locked:
`wall_boundaries.admit_track`), the obstacle avoidance, the stop-and-reverse
logic and the 20 cm LiDAR stop. No new logic was added for it.

**Per move.** `stop_target_callback` forgets the lock id, so each move starts
on the raw value until the BT's first id of that move (at most one BT tick,
and the order of the two topics is not guaranteed either way, so either
race ends on the raw value). A BT that stops running the lock (holding the
car, another move type, dead) leaves the id stale within 0.5 s, and the
ramp goes back to the raw value.

**What the swerve looks like now** (R029, not run): side wall FRONT under the
target → no ramp, no latch, the stop reads "not yet", the car keeps its
mission speed under the wall rows; back on the target wall → the ramp resumes
on it. If the target wall is lost meanwhile (gone, coasted past 0.5 s / 0.3 m,
'unknown') and the side wall is refused, the ramp brakes for the side wall and
the move ends on its timeout, as all of it did before.

**Risk.** While the locked wall is still tracked behind a nearer surface
across the path (a partition, a closed door), the ramp does not slow for that
surface. Only the wall rows, the avoidance and the LiDAR stop act, until the
locked wall is lost (occluded tracks turn 'unknown' or coast out within the
0.5 s / 0.3 m caps), and from then the ramp brakes for it from wherever the
car is.

**Replay.** `tools/wall_lock_replay.py` now prints what the ramp aims at on
every step, from the payload alone. bf7938: 122 raw (unlocked), 9 locked;
corridor10m_20261019T131815: 88 raw (unlocked or lost), 636 locked, 6
not_front (all with the front CLEAR). On both, the ramp reads the raw
contract value of the same publish on every step (0 differ). That holds by
construction -- the tool takes both from the same payload, and they can only
differ on a not_front step with another wall FRONT -- so it says these bags
hold no swerve, not that the ramp is right. The one case the change exists for
is covered by unit tests only (`mpc_controller/test/test_ramp_wall_source.py`).

**Recorded.** The mission logger records `/mpc/stop_wall_id` next to
`/mpc/stop_target`.

---

## One wall confirmation level (2026-10-22, branch `night-walls`)

Not part of the lock, but it changes what the lock can see. The multi-wall
tracker now has one promotion rule (`multi_wall_tracker.py`, "THE CONFIRMATION
LEVEL"; keys `wall_confirm_*`): a track is CONFIRMED after
`wall_confirm_min_consecutive_hits` (5) cycles in a row with a LiDAR-backed
association, a measured length of at least `wall_confirm_min_length_m` (1.0 m),
and its line steady over those cycles within `wall_confirm_max_dist_jitter_m`
(5 cm) and `wall_confirm_max_angle_jitter_deg` (3°) in odom. The distance is
measured from the track's segment midpoint, not from the odom origin (review
fix, 2026-10-22): odom rho moved by the line's angle noise times the
along-wall distance from where the EKF started, so walls a few metres from the
start failed on `dist_jitter` (0.5° read as 8.7 cm 10 m along the wall).
`wall_confirm_require_lidar` (true) means a STEREO-only track never confirms.
Every consumer (FRONT, `/perception/walls`, door hosts, wall markers, the MPC
wall rows, the semantic layer, and this lock) reads that one level; the old
3-of-5 and stereo 5-of-7 counts are gone.

For the lock:

- **FRONT is confirmed by construction**, and with `require_lidar` true it is
  never STEREO-only, so the bf7938 phantom (a stereo-only FRONT for two steps)
  cannot happen any more. The lock's own "never lock a STEREO-only FRONT" rule
  stays as a second line, for `require_lidar` false.
- **Later, and much later for a far wall.** Confirmation takes at least
  0.5 s at the 10 Hz tracker rate (0.3 s before), but that is the floor: five
  extractions IN A ROW is the binding part for a far or partly seen wall.
  corridor10m_20261019T131815 re-tracked LiDAR-only (door_replay's
  Retracker, every 4th scan, stack params): the end wall first becomes FRONT
  15.7 m ahead, 12 s and 5.0 m after the car starts driving, against 20.4 m
  and 0.7 s / 0.15 m before. FRONT is held on 51 % of the bag's 796 cycles
  against 82 %: while standing at the start it is held on 0-40 % of the
  cycles of each 5 s (42-68 % before), and from 45 s, 5 m into the approach,
  on every cycle (from 35 s before). Birth to confirmation: median 4 cycles,
  p90 4 (median 2, p90 3 before). The lock and the ramp need a FRONT, so they
  engage later too.
- **Fewer side-wall rows.** The same replay: a confirmed RIGHT wall within
  2.5 m of the car (an MPC wall row) on 27 % of cycles against 34 % (13 %
  before the midpoint fix). The right wall there is extracted on only some
  cycles, so it rarely gets five in a row. Left: 85 %, unchanged. On the car, `confirm_reason=hits` on
  `/perception/virtual_wall/debug` (`wall_debug_tracks:=true`) is this case.
- **Odometry against the 5 cm bound.** `/odometry/filtered` reads distance
  ~17 % short, so a static wall drifts in odom while the car drives at it. In
  a synthetic head-on approach the spread before confirmation was 2.3-2.9 cm
  at 0.5 m/s and 3.3-4.0 cm at 0.7 m/s; at 1.0 m/s the front wall never
  confirmed. If FRONT shows up late on the car: `wall_debug_tracks:=true`,
  read `confirm_reason` on `/perception/virtual_wall/debug`, and if it says
  `dist_jitter` raise `wall_confirm_max_dist_jitter_m`.
