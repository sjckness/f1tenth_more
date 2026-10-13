# Orin check report: campaign-final on Humble (2026-10-13)

Written on the Orin (Jetson, Ubuntu 22.04, ROS 2 Humble) by a Claude Code
session for linus. Facts linus needs come first. Later sections (campaign state,
build/test, bring-up, GO/NO-GO) are added as they finish.

## 1. Facts for linus (step 2)

### rosbag2: 0.15.16. Part B (407ff6d) works on it as written

| package | installed | apt candidate |
|---|---|---|
| ros-humble-rosbag2-py | 0.15.16-1jammy.20260426 | 0.15.17 |
| ros-humble-rosbag2 | 0.15.16-1jammy.20260426 | 0.15.17 |

- `/opt/ros/humble/share/rosbag2_py/package.xml`: `<version>0.15.16</version>`, so it is >= 0.15.7.
- On this install `rosbag2_py.Recorder` exposes only `record` and `cancel`
  (`dir(rosbag2_py.Recorder)` gives `['cancel', 'record']`). There is no subscription or readiness query.
- 407ff6d does **not** use a second concurrent `record()`. It creates one
  `rosbag2_py.Recorder()` per run and calls `record()` once on a thread
  (campaign-final `mission_logger_node.py:810-823`), then stops it with `cancel()`
  (`:887`). That is the same recorder API the campaign already used, so it is
  compatible with 0.15.16. **No switch to another design is needed.**
- What 407ff6d does is the "simple option" minus the readiness wait: it starts recording at LOADED
  and provides the pre-roll with a fixed 15 s `mission_countdown_sec`
  (`loader.py`), not a wait for all topics to be subscribed. Humble cannot report
  subscriptions, so this is the right call. Its weak point: if discovery ever
  takes longer than 15 s (campaign max was 14 s), the start of the drive is still lost.
  The bring-up dummy mission (step 5) checks this.

### Python stack

| item | version | where |
|---|---|---|
| python | 3.10.12 | system |
| osqp | 1.1.3 | ~/.local (pip) |
| numpy | 1.26.4 | ~/.local (pip) |
| opencv (cv2) | 4.11.0 | ~/.local (pip) |
| cv_bridge | 3.2.1 (ros-humble-cv-bridge) | /opt/ros/humble |
| ultralytics | 8.4.90 | ~/.local (pip) |
| scipy | 1.15.3 | ~/.local (pip) |
| torch | 2.11.0 | ~/.local (pip) |

### Vehicle geometry: base_front, lidar, ZED

The car runs `roboracer.urdf.xacro` through `f1tenth_description/launch/description.launch.py`, with
`enable_sensors` false on hardware. The sensor frames are therefore **not** the URDF's
`sensors.xacro` ones. They come from static_transform_publisher nodes:

| frame | parent | xyz [m] | yaw | source (campaign-final) |
|---|---|---|---|---|
| base_front (bumper) | base_link | 0.443, 0, 0 | 0 | `src/f1tenth_description/urdf/base_front.xacro` (robot_state_publisher) |
| laser | base_link | 0.12, 0, 0.20 | 0 | `description.launch.py` `static_baselink_to_laser` |
| zed2_camera_link | base_link | 0.12, 0, 0.15 | 0 | `f1tenth_perception/launch/camera.launch.py` `static_baselink_to_zed2` |
| imu | base_link | 0, 0, 0 | 0 | `f1tenth_hardware/launch/vesc.launch.py` |
| (sensors.xacro laser, sim only) | base_link | -0.12, 0, 0.15 | pi | stale, see backlog B10 |

On 0.443 m:
- It matches the chassis mesh. I recomputed the bounds from `meshes/chassis.stl`
  with the `chassis_geometry` transform in `macros.xacro:63-65`
  (origin x 0.46, rpy 1.57 0 3.14, x scale 0.00099). The mesh spans x = -0.082 .. **0.4427 m**
  in base_link. It also equals jazzy 30fa1c7's `base_front_x` and the old
  `swept_clearance_body_front_x_m`.
- In the URDF, base_link is at the **rear axle** (`core.xacro`: rear wheels at x 0,
  steering hinges at x 0.325). The 0.443 m is therefore 0.325 m wheelbase plus 0.118 m front overhang.
  **Contradiction:** the docstring in `description.launch.py` says "base_link is centered between the
  axles". If that is the physical truth, the bumper is about 0.28 m ahead of base_link, not 0.443 m.
- Implied bumper-to-sensor offset: the lidar and the ZED are each **0.323 m behind the bumper**.
  The lidar/ZED numbers are documented as ruler placeholders, and 0.443 is mesh-derived and
  never measured on the car. The number that matters for Part E is bumper minus lidar x. It
  needs a tape measurement (laser centre to bumper face) and is checked by the step 5 object test.
- The live `tf2_echo` check needs the stack up (step 5, waiting for the go-ahead).

## 2. Orin state before anything was touched (step 1)

- Branch `test-campaign`, HEAD `6ebbc8a` ("wall_turn: no front distance = no wall decision ...").
- HEAD is pushed: `6ebbc8a` is an ancestor of origin/test-campaign. After `git fetch`, origin/test-campaign is
  3 commits ahead at `349c881` (linus: 48829de, 6cae729, 349c881). The Orin's local branch was not pulled.
- `git log -5 --oneline`: 6ebbc8a, 714c299 (merge), f25e1e3, babc885, 426eabf.
- `git status`: modified `.claude/settings.json` (local permission additions only);
  untracked `pr.json` (57 KB, 2026-10-12 15:31, LLM planner old/new comparison);
  `src/f1tenth_hardware/vesc` has untracked content (`__pycache__` plus
  `vesc_tuning/speed_sweep_results/` with 2 CSVs from 2026-08-07). The submodule pointer is unchanged.
- Snapshot: branch **`orin-snapshot-20261013`** = `6ebbc8a` + one commit holding
  settings.json, pr.json and the 2 vesc CSVs (copied to
  `orin_snapshot_20261013/vesc_submodule_untracked/`, so nothing is pushed to the vesc repo).
  It was built from a temporary index (`git read-tree` / `commit-tree`), so the Orin's working tree,
  index and branch were never changed and no switch back was needed.
- Stashes (kept as they are). None of them touches campaign code paths:

| stash | date | base | content |
|---|---|---|---|
| @{0} "On scene-graph" | 2026-10-08 | e47e646 | `llm_planner_node.py`: legacy SYSTEM_PROMPT wall guard thresh 3.0 -> 2.0 (inactive on planner_path=v2) |
| @{1} "On vesc-tuner-test" | 2026-07-07 | 4eb017e | `track_mpc_opt_node.py`: v_ref 1.0->0.5, track_file default->random.csv; untracked copy of `.claude/settings.json` |
| @{2} WIP vesc-tuner-test | 2026-06-15 | 4eb017e | `stack_bringup_launch.py`: startup_sequence commented out |
| @{3} WIP vesc-tuner-test | 2026-06-15 | 0d7714f | vesc.yaml speed_to_erpm_gain 4614->5242.73, servo gain -1.2135->-1.0860; startup_sequence off; andre/track MPC test speeds |
| @{4} WIP stereocamera-test | 2026-06-12 | 29ad810 | llm_mpc_tuner reverse-driving bounds (v_min negative), parallel `ros2 param set`; andre_mpc CIRCLE_SPEED 1.0->0.5 |

(Sections 3-6 follow below as they are completed.)
