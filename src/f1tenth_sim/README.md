# f1tenth_sim

Gazebo Harmonic simulation of the roboracer for the two-machine setup: this
package runs on the sim host (linus), the stack runs on the Thor. The
launch file is `launch/sim_bringup.launch.py` and its docstring lists every
argument. `scripts/sim/sim.sh` sets the network environment and starts the
launch file. Every `name:=value` argument is passed through to it.

## Test room (`world:=test_room_6x6`)

`worlds/test_room_6x6.sdf` replicates the real test room:

- 6.0 × 6.0 m inside, centred on the origin, so the inner wall faces are at
  x = ±3.0 and y = ±3.0;
- walls 0.10 m thick and 2.5 m high;
- matte walls and floor, lit by two ceiling lights.

The walls are solid both at the LiDAR scan plane (z 0.155) and across the
whole ZED view.

With this world, the launch file spawns:

| Entity | Default | Arguments |
|---|---|---|
| Car | (-2.5, 0), yaw 0 (facing +x, towards the person) | `car_x`, `car_y`, `car_yaw` |
| Person | (0, 0), static, always turned to face the car's spawn point | `person_enabled` (`auto` = on in this world), `person_x`, `person_y` |
| Obstacle | off | `obstacle_enabled`, `obstacle_type` (`plant_0.3` or `box_1.0`), `obstacle_x`, `obstacle_y` |

The models are in `models/`:

- `person_standing` is the Fuel "Standing person" (CC0, see its `LICENSE`).
- `plant_0.3` is a 0.3 m wide pot plant.
- `box_1.0` is 1.0 m across the path, 0.5 m deep and 0.8 m high.

The plant and the box are plain primitives. YOLO is not expected to classify
them, so they reach the stack as LiDAR or depth obstacles, not as detections.

**Ground truth** (for measuring only; the stack never reads it). Each topic is
a `geometry_msgs/PoseStamped` in the world frame:

- `/sim/ground_truth/car` is `base_link`, the rear axle.
- `/sim/ground_truth/person` is at the person's feet.
- `/sim/ground_truth/obstacle` is at the obstacle's base.

Use these for the real clearance and stop distance instead of the perceived
ones. The car's pose also stays on `/sim/ground_truth` as Odometry.

### Scenarios

Run each scenario on linus. Start the stack on the Thor as usual.

```bash
# 1. Person centred, straight approach (MPC dt)
scripts/sim/sim.sh world:=test_room_6x6

# 2. Person 0.3 m off-centre (corridor width)
scripts/sim/sim.sh world:=test_room_6x6 person_y:=0.3

# 3. Plant 0.3 m in the path, person behind it (corridor with a passable obstacle)
scripts/sim/sim.sh world:=test_room_6x6 obstacle_enabled:=true person_x:=2.0

# 4. Box 1.0 m in the path, person behind it (obstacle wider than the corridor)
scripts/sim/sim.sh world:=test_room_6x6 obstacle_enabled:=true obstacle_type:=box_1.0 person_x:=2.0

# 5. Empty room, drive at the +x wall (wall stop at 0.5 m)
scripts/sim/sim.sh world:=test_room_6x6 person_enabled:=false
```

What each scenario tests:

1. **MPC dt.** A 2.5 m straight run to a person with nothing in the way.
   Compare dt settings on the same run: speed profile, steering chatter, and
   the true stop distance (`/sim/ground_truth/car` against
   `/sim/ground_truth/person`).
2. **Corridor width.** The person is 0.3 m off the car's axis, so the car has
   to shift sideways inside the corridor. Check that the true gap from the
   car's outline to the corridor edges and to the person stays positive.
3. **Corridor with a passable obstacle.** The plant (0.3 m, at the room
   centre) stands in the corridor with the person 2 m behind it. Expected
   result: the car passes on one side, without flipping sides, and its true
   gap to the plant stays positive.
4. **Corridor with a blocking obstacle.** The 1.0 m box at the room centre
   is wider than the corridor leaves room for. Expected result: the move ends
   BLOCKED instead of crawling, matching the scenario harness on
   `test-campaign`.
5. **Wall stop at 0.5 m.** No person. The car faces the +x wall 5.5 m away.
   Check the true stop distance from the front bumper to the wall at x = 3.0.

   **Known limit:** the wall confirmation on `test-campaign` (`wall_confirm`)
   looks for a straight LiDAR segment of at least 0.8 m inside ±30°. The
   ±30° sector covers 2·d·tan 30° of the wall, so a 0.8 m segment fits only
   when the LiDAR is at least 0.69 m from the wall. At a 0.5 m stop, the
   LiDAR is about 0.42 m from the wall and sees only about 0.49 m of it, so
   the confirmation cannot succeed there. This is a limit of those
   parameters, not of the world.

The spawn pose can be moved for any scenario, for example
`car_x:=-1.5 car_yaw:=0.2`. Without the `world` argument the default office
world and its own spawn pose are used, unchanged.
