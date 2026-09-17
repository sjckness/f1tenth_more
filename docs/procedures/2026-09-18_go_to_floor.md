# Floor procedure 2026-09-18: go_to at the 0.4 m/s floor

**Status: written 2026-09-17, not run.** Everything here is rig-verified only
(nominal model + measured braking). Nothing in this sheet has moved the car.

Order: **0** setup → **1** wheels-up bench → **2** chair ahead → **3** chair at
45° → **4** person ahead (gap 1.0) → **5** voice.
Stop the session at the first step whose abort criteria trip.

## Rules (all steps)

- **Nobody in the car's path on any non-go_to mission.** Only a go_to target
  may stand in front of the car, and only in steps 4 and 5.
- **Stay near the robot**, e-stop in hand (joystick deadman or
  `/mission/emergency_stop`), for every run including the bench.
- **Voice: single-phase commands only.** `vai dalla sedia`, `vai dalla
  persona`. No chains ("vai dalla sedia e poi …").
- **Voice: no selection commands** (`sinistra`, `destra`, `la seconda`, …).
  The handler takes the nearest track of the class; it cannot pick one.
- **Watcher in a second terminal for every step**
  (`python3 scripts/watch_objects.py`, see §0.4). For every approach, write the
  watcher's final gap **next to** the tape-measured gap.

## What changed since the last floor run (read before step 1)

- **The car never commands between 0 and 0.4 m/s** in a go_to move. It drives
  at 0.4 until mpc_corr's **stop latch** trips at live
  `r ≤ object_reach_tol_m + object_stop_distance_m = 0.10 + 0.14 = 0.24 m`,
  then commands 0 and stays stopped for the rest of the move.
- **It rests a little OUTSIDE the commanded gap**, by ~0.1 m (rig). Expected
  final gap: chair default (0.5) → **0.59–0.63 m**; person gap 1.0 →
  **1.08–1.11 m**; up to +0.17 more if the track jitters. **Never inside the
  commanded gap.** `mismatch_flagged` in the report will usually be set at
  gap 0.5 — expected, not a fault.
- **`object_reached`** fires on `r ≤ 0.10` **or** on `stop_latched` with
  measured speed ≤ 0.05 m/s.
- **Lost track (GRACE) is stop-and-wait**: speed 0, steering held, up to
  `lost_grace_sec` 1.5 s, then `target_lost`. It used to creep at 0.2 m/s.
- **`/mpc/drive_clamp` will fire during every approach.** The rig shows ~19
  clamp events per 4 m approach, early on, requests up to ~0.65 m/s cut to
  0.5. Cause known (the corridor lookahead is beyond the horizon's reach at
  0.4 m/s); not changed. So the car runs at 0.5 for part of each approach.
- **During a go_to approach the solver no longer slows for obstacles** (the
  floor overrides it). Only steering and the behaviour tree's safety stops
  remain. Hence the path rule above.
- `object_stop_distance_m` 0.14 is **measured** (6 archived stops, LiDAR,
  median 0.111 at 0.4 m/s, max 0.133). `object_reach_max_target_age_sec` 1.0
  is **still UNMEASURED** — record `target_age` on every run.

## 0. Setup

### 0.1 Clear the floor
Straight lane ≥ 6 m × 3 m, no glass. Tape a start mark and a centre line.
Car's front bumper on the start mark, pointing down the line. Measure where
the front of the bumper is: **every gap in this sheet is measured from the
front of the car to the near edge of the object**.

### 0.2 Restart the whole stack (one sequence)
```bash
cd ~/dev_ws/f1tenth_more
./scripts/kill_ros2.py -y
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch f1tenth_bringup supervisor_bringup.launch.py
```
In a second terminal:
```bash
cd ~/dev_ws/f1tenth_more && source install/setup.bash
./scripts/stackctl.py --settle 25 status
# need up: hardware, localization, perception, slam, navigation (mpc_corr),
#          behavior, diagnostics (mission_logger), intelligence (llama-server, for step 5)
```

### 0.3 Check the RUNNING values (read from the live nodes, not the files)
```bash
python3 scripts/check_floor_values.py
```
Must print `ALL PASS`:
`max_forward_speed_mps 0.5`, `min_moving_speed_mps 0.4`,
`object_stop_distance_m 0.14`, `object_reach_tol_m 0.1`, `object_a_dec` not
declared, the loader rejects a 0.3 m/s go_to and loads `go_to_person_floor.json`.
**Any FAIL → do not run.** A FAIL on the mpc_corr lines means an old node is
still running: `./scripts/kill_ros2.py -y` and restart.
(The loader check replaces the loaded mission; it starts nothing.)

### 0.4 Watcher (keep it running in its own terminal)
```bash
cd ~/dev_ws/f1tenth_more && source install/setup.bash
python3 scripts/watch_objects.py --csv /tmp/floor_$(date +%H%M).csv
# add --markers to see labels in RViz/Foxglove on /debug/semantic_track_labels
```
One line per confirmed track: `#id class score range bearing gap age`, and
during a go_to a `GO_TO …` line with `r gap alpha target_age speed_ref flags`.
`NO POSE` = no `/ekf_global/odometry/filtered` → **do not start** a run.
Give it ~10 s before trusting an empty list.

### 0.5 Mission start helper (paste once per terminal)
```bash
run_mission() {  # run_mission <mission.json path>
python3 - "$1" <<'PY'
import sys, rclpy
from f1tenth_messages.srv import LoadMission
from std_srvs.srv import Trigger
rclpy.init(); n = rclpy.create_node('floor_caller')
def call(cli, req):
    assert cli.wait_for_service(timeout_sec=10.0), cli.srv_name
    f = cli.call_async(req); rclpy.spin_until_future_complete(n, f, timeout_sec=10.0)
    print(cli.srv_name, f.result()); return f.result() is not None and f.result().success
if call(n.create_client(LoadMission, '/mission/load_mission'), LoadMission.Request(path=sys.argv[1])):
    call(n.create_client(Trigger, '/mission/start_mission'), Trigger.Request())
n.destroy_node(); rclpy.shutdown()
PY
}
M=$(ros2 pkg prefix f1tenth_behavior)/share/f1tenth_behavior/missions
```

### 0.6 Clamp events, live
```bash
tail -f ~/.ros/log/component_supervisor/f1tenth_navigation_navigation.launch.py.log | grep --line-buffered "DRIVE/clamp\|OBJECT/stop\|OBJECT/speed\|OBJECT/watchdog"
```
`DRIVE/clamp | requested +0.6x -> +0.500` early in an approach is expected.
`OBJECT/stop | latched at r=…` should appear once per approach, r ≈ 0.20–0.24.

### After every run
```bash
RUN=$(ls -td ~/f1tenth_archive/complete/*/ | head -1)
ros2 run f1tenth_logger mission_extract "$RUN/bag" --out-dir /tmp
```
Reads the newest archived bag and writes its extract to **/tmp** (never into
the archive). Prints per go_to move: `outcome`, final `r`, `alpha`,
`target_age`, `latched_r`, `max_speed`, and the drive clamp event count. Copy
them into the run log. (The bag stops at the mission's end: nothing after the
stop is recorded.)

---

## 1. Wheels-up bench

**Car on blocks, wheels off the ground. Nobody touches the wheels.** A chair
~2.5 m in front of the car, in camera view.

**What the bench can and cannot show.** It shows the SEQUENCE: acquire, drive
at the floor, stop-and-wait on a lost track, the latch holding, and
`object_reached`. It **cannot validate the stopping distance or any gap**: the
wheels spin in the air, odometry integrates that spin as motion, so `r` in
`/mpc/object_status` shrinks while nothing moves, and the latch will trip
from odometry alone after a few seconds. Ignore every distance on the bench.

1. `run_mission $M/go_to_chair_floor.json`
2. **Acquire.** Watcher: the chair line gets `TARGET`; a `GO_TO` line appears
   with `speed_ref 0.40`, flags `-`. Wheels spin up.
   *Write down:* `target_age` (typical value while tracking).
3. **Floor.** The wheels must never turn slowly: either stopped or at the
   0.4–0.5 m/s wheel speed. A slow crawl = FAIL.
4. **Stop-and-wait.** Cover the camera (or lift the chair out of view) for
   ~1 s, then uncover. Expect: wheels stop within ~0.5 s, steering does NOT
   return to centre, then wheels resume when the chair is back in view.
   Watcher `speed_ref 0.00` while covered.
5. **Lost.** Cover for > 2 s. Expect the move to end `target_lost` (behaviour
   log `failed: target_lost`), wheels stopped, steering ramps to centre.
6. Restart the mission (`run_mission $M/go_to_chair_floor.json`). Let it run
   until the watcher shows `flags stop_latched` (odometry-driven, see above).
   Expect: wheels stop, and **stay stopped** even after you move the chair
   farther away. Then `object_reached` → mission COMPLETE.

**Abort:** wheels crawl slowly at any time; wheels keep spinning > 1 s after
`stop_latched`; wheels restart after `stop_latched`; `goal_watchdog` flag
appears; `check_floor_values.py` was not ALL PASS.

---

## 2. Chair ahead (wheels on the ground)

Chair **3.0 m** ahead on the centre line, measured from the car's front to the
chair's **near edge**. Nothing else in the lane.

```bash
run_mission $M/go_to_chair_floor.json
```

**Watch (watcher):** chair `TARGET`; `GO_TO … speed_ref 0.40`; `gap` falling;
`flags stop_latched` appears around gap ≈ 0.74 (0.5 + 0.24); `target_age`
stays well under 1.0 s.
**Watch (0.6):** clamp events early, one `OBJECT/stop | latched`.

**Expect:** car stops with its front **0.59–0.63 m** from the chair's edge
(rig), `outcome reached`.

**Write down:** tape gap (front bumper → chair near edge), watcher final gap
(last `gap` on the chair line), `GO_TO` final `gap` and `target_age`,
`latched_r` and clamp count from mission_extract.

**Abort (e-stop):** gap on watcher < 0.45 while still moving; car still moving
1 s after `stop_latched`; car drives off the line toward something other than
the chair (wrong `TARGET`); `target_behind_terminal` or `goal_watchdog`;
reverse; anything unexpected.

Repeat 3 times. If the tape gap is **below 0.5** on any run, stop the session:
the measured stopping distance does not hold on this floor.

## 3. Chair at 45°

Chair at **3.0 m, 45° left** of the centre line (≈ 2.1 m ahead, 2.1 m left),
measured to its near edge. Same commands, watch list, write-down and abort as
step 2. Add: `alpha` at the end (arrival bearing), and whether
`inside_turn_radius` ever showed (advisory, expected at most early).

**Expect (rig):** final gap 0.59–0.63 m, `outcome reached`.
Repeat 2 times, then once at 45° right.

## 4. Person ahead, gap 1.0

A person stands **3.5 m** ahead on the centre line, still, facing the car,
spotter beside them. Mission gap is **1.0 m** (twice the default) on purpose.

```bash
run_mission $M/go_to_person_floor.json
```

**Watch:** person `TARGET` (and no other person in view); `flags stop_latched`
around gap ≈ 1.24; `target_age` < 1.0 s.
**Expect (rig):** car rests **1.08–1.11 m** from the person's near edge (up to
~1.28 if the track jitters), `outcome reached`.
**Write down:** as step 2 (tape gap to the front of the person's feet/shins).

**Abort:** watcher gap < 1.0 while moving; car still moving 1 s after
`stop_latched`; person says stop; `TARGET` jumps to a different track id
while approaching; anything unexpected. The person steps aside at will.

Repeat 3 times.

## 5. Voice (LLM planner)

Chair back at step 2's position, llama-server up (intelligence component).
```bash
cd ~/dev_ws/f1tenth_more && source install/setup.bash
ros2 run llm llm_planner_node --confirm "vai dalla sedia"
```
Read the printed plan: **one** `go_to_object` move, `target_class chair`,
speed 0.4, default gap 0.5. Answer the confirmation only if it matches.
Watch/expect/write-down/abort as step 2.

Then the person at step 4's position:
```bash
ros2 run llm llm_planner_node --confirm "vai dalla persona"
```
One `go_to_object` move, `target_class person`, gap **0.5** (default, not
1.0 — the person must be told the car will stop closer than in step 4).
Watch/expect as step 4 but expected rest **0.59–0.63 m**; abort as step 4.

**Refuse at the prompt** if the plan has more than one move, a class other
than the one asked for, or any selection wording.

---

## Run log

| # | step | mission / command | tape gap (m) | watcher final gap (m) | status gap (m) | target_age (s) | latched_r | clamp events | outcome | aborted? | notes |
|---|---|---|---|---|---|---|---|---|---|---|---|
