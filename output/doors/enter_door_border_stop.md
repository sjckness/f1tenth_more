# enter_door: arrives at the border of the doorway, not centered

2026-10-29, after `4010cb88` (enter_door: a left/right selector). Branch
`jazzy-doors` (this code does not exist on `main-jazzy-rust`, where this note
is being filed from).

## Symptom

On the live run right after the left/right selector fix landed: the car
resolved the correct door (confirms the fix -- no more coin-flip between two
doors facing each other across the corridor), drove up to it, and the
`door_entered` condition fired. But the car ended up at the edge of the
opening, not centered in it, so the doorway was not centered in the camera
frame. Reported as "again" -- this is a repeat of earlier runs, not a new
regression from today's commit.

## What was checked

- `src/f1tenth_behavior/f1tenth_behavior/behaviours/enter_door.py` (jazzy-doors)
- `src/f1tenth_costmap/f1tenth_costmap/semantic_doors.py` (jazzy-doors)
- `src/f1tenth_behavior/missions/enter_nearest_door.json` (jazzy-doors)

No run log, bag or mission-logger archive was available locally for the
actual run (nothing under `~/f1tenth_archive` newer than 2026-10-23; the run
was not captured anywhere this session could reach). The analysis below is
from reading the code, not from that run's numbers.

## Leading cause: `xy_tol` is one Euclidean radius, not forward/lateral

`EnterDoor.update()` measures arrival as:

```python
record.xy_error_m = math.hypot(door_xy[0] - car_xy[0], door_xy[1] - car_xy[1])
record.entered = (record.phase == PHASE_ENTER
                  and record.xy_error_m <= spec.xy_tol
                  and record.yaw_error_deg <= spec.yaw_tol_deg)
```

`enter_nearest_door.json` sets `xy_tol: 0.15` m. The physical slack either
side of center is `(door_width - car_width) / 2 = (0.9 - 0.57) / 2 =
0.165 m`. A 0.15 m tolerance is ~91% of that slack -- it does not distinguish
"0.15 m short of the opening, dead center" from "exactly at the opening, 0.15 m
toward one jamb". Either satisfies `entered`, and the second is visually "at
the border" exactly as reported. Since `xy_tol` is a plain radius with no
split between along-axis (how far through) and lateral (how far sideways)
error, this is consistent with a *repeat* failure rather than a one-off --
it is how arrival is scored, not a transient detector glitch.

## Checked and likely not the cause

`DoorTrack.centre` (the detector's estimate of the doorway's physical center)
is EMA-blended on every hit (`smooth_alpha = 0.3`) even after confirmation,
in `DoorTracker._hit()`:

```python
trk.centre = ((1.0 - a) * trk.centre[0] + a * cand.centre[0],
              (1.0 - a) * trk.centre[1] + a * cand.centre[1])
```

This keeps improving as the car gets closer and sees both jambs more
squarely -- it does not ratchet the estimate toward one side over an
approach. No asymmetry was found in how a CASE A (parallel-wall) doorway's
two jambs are averaged (`(jamb0 + jamb1) / 2`, equal weight).

## Open: EKF-vs-reality offset

Arrival is measured against the global EKF pose (`GLOBAL_XY_KEY` /
`GLOBAL_YAW_KEY`), not the camera. This is the same input the earlier
`go_to_object` wobble bug traced to a real offset (`/ekf_global` heading
lag vs. SLAM/camera truth, ~0.84x gain). If that offset is present here too,
the logged "entered, low error" could be a real-world position further off
than the number says. Not ruled in or out -- needs the actual run's
`xy_error_m` / `yaw_error_deg` at the moment `entered` fired, plus the
resolved door's `centre`/`yaw`, to tell apart from the tolerance explanation
above.

## What would confirm it

The `[enter_door]` log lines from the failing run (phase transitions,
resolved door id, and the goal_pose values `EnterDoor._publish_goal` prints),
or the mission's `DoorApproachRecord` as written into the mission summary /
extract. Not captured for this run.

## Recommended fix

Split `xy_tol` into an along-axis tolerance (depth through the opening,
can stay loose) and a lateral tolerance (sideways offset from the door's
own axis, should be tight -- well under the 0.165 m physical slack). A
single radius around the centre point structurally allows "entered" while
sitting at a jamb.
