# f1tenth_sim — local memory

Contracts/invariants only; narrative: `README.md` (test room, the 5 scenarios, ground truth) and
`output/sim_{port_report,clock_fanout,camera_compression}.md`.

## Identity
`ament_python`, 4 executables (`setup.py:46-55`): `drive_bridge`, `clock_throttle`,
`camera_pan_bridge`, `ground_truth_relay`; the tested logic is ROS-free (`kinematics.py`,
`ClockDecimator`/`clock_ns_from_cdr`). `setup.py` globs `launch/*.launch.py`, `worlds/*`,
`config/*` plus one `data_files` entry per `models/**` dir, so new files install themselves.
Not built on the Thor (`~/.colcon/defaults.yaml` `packages-skip: [f1tenth_sim, f1tenth_more]`);
the sim host is linus/TPad.

## Topology
One sim-side entry point, `sim_bringup.launch.py` (docstring = the arg reference). The stack's
half (`sim:=true`, `camera.launch.py`→`sim_camera_tf.launch.py`, `joint_state_publisher` owning
`/joint_states`) is `f1tenth_bringup/memory.md`'s, not restated here.
husarion gz_sim → private `sim_robot_state_publisher` → `ros_gz_bridge` (+ camera bridge) →
2 compressors → `clock_throttle` → `create` on `/sim/robot_description` → person/obstacle →
ground-truth bridge+relay → `drive_bridge` → pan spawner/bridge → foxglove. Spawners chain on
`OnProcessExit` (create→jsb→ackermann→camera_pan).

## Laws
- **No TF.** The private RSP remaps to `/sim/tf`, `/sim/tf_static`, `/sim/joint_states`; both
  bridge YAMLs are `GZ_TO_ROS`-only and `/tf`-free (`test_gz_bridge_never_publishes_tf`).
- **`/clock` is never bridged directly.** gz `/clock` → `/sim/clock_raw` → `clock_throttle` →
  `/clock` @250 Hz, because every world sets `max_step_size 0.004` (5 ms bins would alias the
  4 ms step). Forces `use_sim_time=False`; decimates on CDR bytes; forwards world-reset time.
- **Drop-in for the drivers, not for the stack.** `/ackermann_drive` → `TwistStamped` on
  `.../reference`, steering clamped to the real servo envelope (D2); controller odom → `/odom`;
  gz IMU → `/sensors/imu/raw` with `frame_id ""` + the `vesc.yaml` covariances (D3);
  `enable_odom_tf: false` — the Thor's EKF owns `odom→base_link`.
- **Raw frames never cross the LAN.** Staging `/sim/camera/{image_raw,depth_raw}` → compressed →
  decompressed on the Thor by `sim_camera_tf.launch.py`; `camera_info` keeps the canonical
  names. `republish` wants transports as *parameters*; only fully-qualified remaps apply.
- **Ground truth is measuring-only:** `/sim/ground_truth` plus
  `/sim/ground_truth/{car,person,obstacle}`; models carry `gz-sim-pose-publisher` @10 Hz.
- Limits: no pre-steer at standstill (a twist cannot carry steering at v=0); `wall_confirm`
  cannot succeed at a 0.5 m stop (`README.md` §5).

## Coupled constants (`test_sim_matches_car_config.py`)
wheelbase `0.325` identical in the `drive_bridge` default, launch params, `controllers.yaml`
and URDF; track/`wheel_radius` = URDF; `reference_timeout 0.2` = `mux.yaml` timeout;
pose/twist/IMU covariances = `vesc.yaml`; `STEERING_MIN/MAX_RAD` = `steering_calibration.yaml`
gains. The test reads the *sources*: the URDF side is the literal joint origins in
`core.xacro` (rear wheel x 0 / y ±0.1, steering hinge x 0.325 — there is no
`wheelbase`/`track` `<xacro:property>` block any more) plus the wheel collision cylinder in
`macros.xacro` (radius 0.05, length 0.045); the camera side is `camera_model: 'zed2'`
(`camera.launch.py`, wrapper v5.4.1), not `zed2i`.

## Do not "clean up"
The clock path, staging topics, private RSP remaps, `enable_odom_tf: false`, `frame_id ""`, and
the `''` `car_*` args (empty = the world's own default, resolved by `PythonExpression`).

## Trust level
`pytest test` is **59 passed at HEAD** (`test_flake8.py` excluded — it needs `ament_flake8`,
absent on this host). The suite reads sources directly and needs no built workspace, but
`test_urdf_sim_mounts_...` shells out to `xacro`, so run it with the ROS env sourced and
`PYTHONPATH` appended (`.../src/f1tenth_sim:$PYTHONPATH`) — overriding `PYTHONPATH` drops
`xacro`'s own package metadata and the xacro call fails with `PackageNotFoundError`. The five
2026-11-03 drifts are cleared: tests now read `core.xacro`/`macros.xacro` literals and `'zed2'`;
the launch's stray `wheelbase: 0.3302` is back to `0.325` (D5, the drive_bridge default and
`controllers.yaml`); `sim_port_report.md` D2 quotes the M02 gains (−0.30935/+0.31975) from
`kinematics.py`. D2's older §4.R/§9 measurements pre-date M02 and stay as history.
