# Floor procedure: passing a standing person on a straight move

**Status: written, not run.** Work order Part 1.5, 2026-09-17.

## Read first: the rig predicts contact — use a dummy, not a person

Part 1 found **no person margin that is safe** (`obstacle_class_margin_m`
stays `{}`, no m\*). In the nominal-model rig
(`docs/analysis/2026-09-17_person_pass_margin.md`), passing a 0.50 m-wide
person 2 m ahead:

| mode | offset 0.0 | offset 0.3 |
|---|---|---|
| footprint, margin {} (shipped default) | body gap **−0.205 m** (contact) | **−0.032 m** (contact) |
| legacy | never passes (held 1.67 m short) | +0.236 m |

So the target for this procedure is a **person-sized dummy**: a life-size
printed photo of a standing person mounted on foam board or a light stand,
0.50 m wide × 1.75 m tall, light enough to be knocked over without damage.
It must be detected as `person` (checked in step 4). **No human stands in the
car's path.** Repeat with a human only after the dummy runs measure a body gap
≥ 0.25 m, and then only at offsets that did.

## What is measured

Per run: the car's minimum centre distance to the dummy, and the body gap.
Both come from **odometry plus the tape-measured dummy position**, not from
detections, which clip at the image edge (50.5° half field of view) before
the closest approach.

- Centre distance d = min over the run of |car(base_link) − dummy|, with the
  dummy's position tape-measured from the car's start pose.
- Body gap (disk, as in the rig) = d − 0.25 − 0.20.
- Odometry scale: `/odom` and the local EKF under-report distance by about 17%
  (vesc speed_to_erpm_gain, 2026-08-10). Use `/ekf_global/odometry/filtered`,
  and tape-measure the car's final stop position to check its scale per run.

## Setup

1. Clear straight floor, at least 7 m long × 3 m wide, no glass. Mark the
   start line and a straight reference line with tape.
2. Car at the start mark, base_link (centre between the axles) over the mark,
   heading along the tape line. Measure and record where base_link is on the
   chassis before the first run.
3. Dummy centre at **3.00 m along the line**, lateral **offset 0.30 m** (left)
   for the first runs, then **0.00 m**. Tape both positions; measure to the
   dummy's centre.
4. Overhead phone video of the pass region, if possible, as independent
   ground truth.
5. E-stop in hand (joystick deadman, or `/mission/emergency_stop`). One
   spotter beside the dummy, never in the path.

## Mission file (goal_distance 5 m)

```bash
cat > /tmp/person_pass_5m.json <<'JSON'
{
  "mission_id": "person_pass_5m",
  "moves": [
    {"id": "straight_5m", "goal_distance": 5.0,
     "stop_condition": {"type": "distance_reached"},
     "timeout_sec": 30, "on_timeout": "abort"}
  ]
}
JSON
```

It drives at mpc_corr's default 0.5 m/s. The `/drive` clamp (+1.0 / 0.0 m/s)
is active.

## Bring-up

```bash
cd ~/dev_ws/f1tenth_more && source install/setup.bash
ros2 launch f1tenth_bringup supervisor_bringup.launch.py
./scripts/stackctl.py --settle 25 status
# need: hardware, localization, perception (camera + detection), slam,
#       control, navigation (mpc_corr), behavior, diagnostics (mission_logger)
```

Mode check: `stack_params.yaml` must have `obstacle_radius_source: footprint`
and `obstacle_class_margin_m: {}` for the footprint runs.

Hand-started bag, in addition to mission_logger's automatic one:
```bash
ros2 bag record -o ~/f1tenth_archive/manual/$(date +%Y-%m-%dT%H-%M-%S)_person_pass_<mode>_<offset> \
  /odom /odometry/filtered /ekf_global/odometry/filtered /tf /tf_static /scan \
  /camera/detections /camera/detections_3d /perception/obstacles_2d \
  /costmap/boundaries /mpc/corridor_markers /mpc/solver_status \
  /drive /ackermann_drive /mpc/drive_clamp /mpc/hold /safety_stop \
  /mission/status /mission/move_outcome /behavior/tree_status
```

## Runs

For each run: start the bag, place the dummy, then
```bash
ros2 service call /mission/load_mission f1tenth_messages/srv/LoadMission "{path: /tmp/person_pass_5m.json}"
ros2 service call /mission/start_mission std_srvs/srv/Trigger {}
```
After the run: stop the bag, tape-measure the car's stop position, and record
the run in the log below.

**If `ros2 service call` hangs or reports no service.** The `ros2` CLI is
unreliable under this stack's Discovery Server (node/topic listing and
`param get` are known blind; service calls are untested here). The same two
calls through rclpy, which does discover the graph:
```bash
python3 - /tmp/person_pass_5m.json <<'PY'
import sys, rclpy
from f1tenth_messages.srv import LoadMission
from std_srvs.srv import Trigger
rclpy.init(); n = rclpy.create_node('person_pass_caller')
def call(cli, req):
    assert cli.wait_for_service(timeout_sec=10.0), cli.srv_name
    f = cli.call_async(req); rclpy.spin_until_future_complete(n, f, timeout_sec=10.0)
    print(cli.srv_name, f.result())
    return f.result() is not None and f.result().success
if call(n.create_client(LoadMission, '/mission/load_mission'), LoadMission.Request(path=sys.argv[1])):
    call(n.create_client(Trigger, '/mission/start_mission'), Trigger.Request())
n.destroy_node(); rclpy.shutdown()
PY
```
After the first run, check that the hand bag holds messages
(`ros2 bag info <bag>`: non-zero counts on /odom and /perception/obstacles_2d).
If it is empty, the recorder did not discover the graph; use
mission_logger's automatic bag for the analysis instead.

| # | mode | offset | notes |
|---|---|---|---|
| 1 | footprint {} | 0.30 | rig: −0.032 m |
| 2 | footprint {} | 0.30 | repeat |
| 3 | footprint {} | 0.00 | rig: −0.205 m, expect contact with the dummy |
| 4 | legacy | 0.30 | rig: +0.236 m |
| 5 | legacy | 0.00 | rig: held ~1.67 m short, mission times out (30 s abort) |

**Switching to legacy.** Edit `src/f1tenth_params/config/stack_params.yaml`:
`obstacle_radius_source: legacy`, then `./scripts/stackctl.py restart perception`
and confirm in `~/.ros/log/component_supervisor/perception*.log` that
obstacle_projector_node logged `radius_source=legacy`. Set it back to
`footprint` and restart perception after run 5.

## Pre-run check (every run)

With the dummy in view before start, `/perception/obstacles_2d` must contain
one disk near (3.0, offset) in base_link with r ≈ 0.25 (footprint) or ≈ 0.87
(legacy). If it is empty, the dummy is not detected as a person: **the run is
invalid**; do not start it.

## Abort (e-stop immediately)

- The car is heading into the dummy and the spotter judges the gap will be
  under ~0.15 m.
- `/drive` shows reverse or more than 1.0 m/s, or `/mpc/drive_clamp` publishes
  during the pass. The clamp should make this impossible; seeing it means the
  command path is not the one tested.
- `/safety_stop` trips (LiDAR contact backstop at 0.15 m).
- `/mpc/solver_status` reports repeated failures.
- The car leaves the 3 m-wide cleared lane.
- Anything unexpected: stop first, read the bag after.

## Analysis (after the session, from the hand bag)

```bash
python3 - <bag_dir> 3.0 <offset> <<'PY'
import math, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag, dummy = sys.argv[1], (float(sys.argv[2]), float(sys.argv[3]))  # dummy x, y in the start frame
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=['/ekf_global/odometry/filtered']))
start, d_min = None, math.inf
while r.has_next():
    topic, raw, _t = r.read_next()
    p = deserialize_message(raw, get_message(types[topic])).pose.pose
    if start is None:
        yaw = math.atan2(2*(p.orientation.w*p.orientation.z), 1-2*p.orientation.z**2)
        start = (p.position.x, p.position.y, yaw)
    dx, dy = p.position.x - start[0], p.position.y - start[1]
    x = dx*math.cos(start[2]) + dy*math.sin(start[2]); y = -dx*math.sin(start[2]) + dy*math.cos(start[2])
    d_min = min(d_min, math.hypot(dummy[0] - x, dummy[1] - y))
print(f'min centre distance {d_min:.3f} m, body gap {d_min - 0.45:+.3f} m')
PY
```
Arguments (after `python3 -`): bag directory, dummy x (3.0), dummy y (the
offset, positive left). Correct `d` by the
per-run odometry scale from the taped final position if they disagree by more
than 5 cm.

## Run log

| # | date/time | mode | offset | bag | min centre (odom) | body gap | taped stop | clamp events | aborted? | notes |
|---|---|---|---|---|---|---|---|---|---|---|
