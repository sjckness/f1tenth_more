GOAL
Add a motorized pan (yaw about z) to the ZED 2i, ±15° range. Sim first (f1tenth_sim, Gazebo), then real hardware later.
For now one behaviour only: always point the camera where the car is GOING (an arc on the path ahead, not the steering angle).
The TF must follow the real camera angle so perception (YOLO + depth -> 3D -> costmap semantic layer) stays correct.
A future "scan" mode will sweep the camera around. Design the mode switch for it but do NOT implement scanning.

READ FIRST, BEFORE WRITING ANYTHING (report what you find)
- Working tree is dirty on branch `jazzy` (URDF refactor, sim config, perception changes not mine to commit).
  Do not commit, stage or revert those files. Stage only files you create or edit, and list them at the end.
- Camera frame chain today:
  - sim URDF: f1tenth_description/urdf/sensors.xacro + config/sim_sensor_mounts.yaml (single source of truth,
    zed2_camera_link xyz [0.36, 0, 0.25], base_link = rear axle on the ground)
  - stack in sim mode: f1tenth_perception/launch/sim_camera_tf.launch.py (the sim's own TF is on /sim/tf and never reaches the stack)
  - real car: camera.launch.py static base_link -> zed2_camera_link (0.12, 0, 0.15), then the ZED wrapper's robot_state_publisher below it
  - tests that encode these mounts: f1tenth_sim/test/test_sim_matches_car_config.py, f1tenth_bringup/test/test_sim_mode.py
- Find EVERY consumer that assumes the camera looks along base_link +x. grep for zed2, camera_link, optical, hfov/fov, and hardcoded yaw.
  Minimum list: obstacle_projector_node, front_depth_monitor_node (feeds MPC corridor length), the costmap semantic layer, swept_corridor.py,
  and anything using the ZED IMU (that IMU will now rotate with the camera). Tell me which ones break with a panned camera.
- Check how perception looks up TF. Every camera -> base_link / map lookup MUST use the image header stamp, never rclpy.time.Time() (latest).
  With a moving joint, latest-available is silently wrong. We have been burned by latest-available lookups before.
- Confirm the ZED wrapper on the car does not also publish a TF that would conflict with a dynamic pan joint (pos tracking publish_tf etc.).
- Find where wheelbase L, current steering angle and odometry live. Read L from the existing config, do not hardcode it.

DESIGN
1. Frames: base_link -> camera_pan_base (fixed, the pivot point) -> zed2_camera_link (revolute about z, the pan angle).
   Add the pivot xyz to sim_sensor_mounts.yaml (sim) and a matching real-car param (pivot placement on the car TBD, make it a param).
   Joint limits ±0.2618 rad. When pan = 0 everything must be bit-identical to today's TF tree (test this).
2. Aim law (pure function in its own module, unit-tested):
   - curvature κ: prefer measured κ = ω_z / v from the local EKF odom when |v| > v_min, otherwise κ = tan(δ)/L from the
     applied/measured steering angle. Blend smoothly between the two, no hard switch.
   - look-ahead arc length s = clamp(v * t_lookahead, s_min, s_max)
   - point on the arc in base_link (rear axle, bicycle model): P = (sin(κs)/κ, (1 - cos(κs))/κ), limit P = (s, 0) as κ -> 0 (use a series form, no divide-by-zero)
   - pan = atan2(P_y - pivot_y, P_x - pivot_x), then clamp to ±15°
   - reversing / stopped: return to 0 (param)
   - add a low-pass filter, rate limit (rad/s) and small deadband so the image doesn't jitter
   All params go in stack_params.yaml with descriptions. Keep descriptions consistent with the defaults.
3. Node `camera_pan_controller_node` (f1tenth_perception or a new f1tenth_camera_pan package, your call, justify it):
   - param `mode`: `track_heading` (now), `fixed` (hold `fixed_angle`, use for testing), `scan` reserved (log not-implemented, fall back to fixed 0)
   - publishes the command: std_msgs/Float64 rad on /camera_pan/command
   - hardware-agnostic. The sim and the future real driver both just consume this topic and report the MEASURED angle
     as sensor_msgs/JointState on /camera_pan/joint_state
4. TF publisher: the stack publishes base_link chain -> zed2_camera_link from the MEASURED angle (joint_state), stamped with the
   measurement time, never from the command. Publish rate >= 50 Hz so stamp-exact lookups interpolate cleanly.
   Replaces the static transform in sim_camera_tf.launch.py (sim) and camera.launch.py (car, behind a `camera_pan_enabled`
   param that defaults to false on the car until the hardware exists, so today's car keeps its static TF).
5. Sim: revolute joint in the URDF + position controller (use whatever fits the existing controllers.yaml / ros2_control or gz
   plugin setup, explain the choice). Bridge the command in and the joint state out through ros_gz_bridge config. Realistic
   servo speed limit (param, e.g. ~6 rad/s), so the TF-from-measured-angle path is really exercised.
6. Real hardware: OUT OF SCOPE. Only write down the interface a driver must satisfy (topics, units, rates, stamp semantics).

TESTS / VERIFICATION
- Unit tests for the aim law: straight -> 0, left/right turn sign correctness, clamp, κ->0 continuity, reverse -> 0.
- TF test: pan = 0 gives exactly today's transforms. Pan = ±15° rotates zed2_camera_link about the pivot.
- Extend test_sim_matches_car_config.py: the hokuyo housing must stay outside the ZED VFOV across the whole ±15° range (it currently checks only 0).
- Sim run: drive a constant-steering circle and a slalom. Show (a) the pan angle tracks the arc, (b) a static object's map-frame
  position from perception stays put while the camera pans (that is the real proof that the TF is right).
  Report the position spread with pan vs. fixed.
- Remember setup.py needs the `test` extra or colcon runs 0 tests (CLAUDE.md).

DELIVERABLE
Summary of what you changed, the consumer audit results, test output, sim numbers, open questions. Commit only your files on
`jazzy`, with a clear message. Don't push.
