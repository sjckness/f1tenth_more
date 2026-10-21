# Front-wall lock for "approach the wall": design

2026-10-21, branch `campaign-final`. Triggered by `P002-R029`/`R031` review
(see chat): the `front_clearance` stop_condition has no concept of "the wall I
was sent to approach" — it just asks a single live estimator "is something
close, dead ahead, right now." This proposes closing that gap by mirroring a
pattern the codebase already has for objects, not inventing a new one.

Nothing here is implemented yet. Design only.

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
  refuse a side wall at the same distance, which is §4's case. 15° is not
  measured: far above line-normal noise and one move's yaw drift, far below a
  side wall's 90°.
- **"Grace" is the tracker's own FRONT coast caps** (0.5 s / 0.3 m of path),
  read from the same keys.
- **"Hold" means the stop reads "not yet"** (`condition_eval.py`'s fail-safe
  for an absent reading), not `/mpc/hold`. The car keeps driving under the
  MPC, and the move stays bounded by its `timeout_sec`.

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

### Not done

- **The MPC's deceleration ramp still ramps on the raw
  `/perception/virtual_wall`**, and its stop latches until the next move. In
  the R029 case (a side wall becomes FRONT nearer than the target) the ramp
  still stops the car by the side wall. The lock removes the false "arrived";
  the move then ends on its `timeout_sec`. Fixing the stop itself means
  feeding the ramp the locked distance (`MPC_corr._wall_stop_d_rem`), which is
  in the MPC speed path and was left out.
- **Not run on the car, and never on a swerve.** The case the lock exists for
  is covered by unit tests only (`test_wall_lock.py`,
  `test_wall_lock_behaviour.py`). The next M02 run with a person in the path
  should be bagged (the mission logger already records both topics) and
  replayed with the tool.
- §5's second point (FRONT behaviour mid-swerve on real data) is still open,
  for the same reason.
- **A different wall nearer than the locked one is refused too**, by the
  0.30 m gate, not only a side wall. A partition or a closed door across the
  path after the lock is taken ends the move on its timeout (the ramp still
  stops the car by it), where it used to end as "arrived".
