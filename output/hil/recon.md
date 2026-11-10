# HIL recon: Isaac on the workstation, the car stack on the Orin

Branch `hil-isaac`, worktree `~/dev_ws/f1tenth_more_hil`, base commit **`698faa14`** (tip of
`jazzy-doors`, 2026-10-29 12:00). Read-only recon done on linus on 2026-11-10; nothing was run
(no stack, no Isaac, no ROS). Every number below is either read from a file (file:line given) or
computed here by a script in this branch.

## 0. Branch choice

| candidate | tip | what it holds |
|---|---|---|
| **`jazzy-doors`** (chosen) | `698faa14` 2026-10-29 | the whole freespace door detector: scan-jump candidates (`b9b88093` "find them in the scan, not in the shape of the map"), the strong-door flag (`dc1dd62e`), one doorway one track (`7fa7d58f`), AB-cell scoring (`698faa14`) |
| `jazzy-doors-llm` | `0eb9d152` 2026-11-10 01:25 | `jazzy-doors` + 5 commits of LLM-planner work from tonight (`enter_door` schema 6.1, `llm_warmup`) |
| `doors-enter` | `2ba1497b` 2026-10-28 | older: semantic doors only, no freespace source |
| `door-fixes`, `tmp/door-fixes-*` | 2026-10-19/20 | the earlier Humble-line door work |

`jazzy-doors` is the tip of the door-DETECTOR line: the five newer commits on `jazzy-doors-llm`
touch the LLM planner, not the detector, and that branch is checked out in
`~/dev_ws/f1tenth_more/.claude/worktrees/doors-llm`, where another session was committing at
01:25 tonight. Basing on `698faa14` keeps the detector at its newest committed state and keeps my
history clear of commits that may still be amended. No merge, no rebase onto either branch.

Worktrees/branches left untouched: everything in `git worktree list`, in particular
`jazzy-doors-llm` (the other session) and the main checkout `~/dev_ws/f1tenth_more` (on `jazzy`).

## 1. What already exists

### 1.1 The Isaac sim (`~/isaac_ws`, branch `f1tenth`, `4d2de1e`, 2026-10-16)

A complete standalone simulation, not a toy: `sim/main.py` runs physics at 60 Hz, renders only at
the camera rate, and publishes every sensor from the loop with sim-time stamps.

| part | file | what it does |
|---|---|---|
| app | `sim/app.py` | SimulationApp, headless or GUI, render settings |
| car | `sim/car.py` | the NVIDIA Leatherback scaled 0.953 to the real 0.305 m wheelbase, Ackermann controller, first-order actuator lag + rate limit, wheel-torque limit (2.0 N m, found after PhysX went NaN against a wall) |
| sensors | `sim/sensors.py` | PhysX 2D lidar remapped onto the UST-10LX's exact beam angles, ZED2i-like stereo pair with the real `K`, IMU |
| noise | `sim/noise.py` | lidar range noise, depth floor-NaN + sigma = k z^2 + confidence map, IMU white noise, odometry scale error |
| ROS | `sim/ros_bridge.py` | rclpy publishers driven from the loop, `/clock`, the reset service |
| scenes | `sim/scene_builder.py` | `procedural_room` and `usd_environment`, colliders, semantic labels, person, plant, obstacle |
| launch | `launch.sh` | host side: `docker compose run`; container side: bridge libs, DDS profile, SIGTERM forwarding |

Isaac Sim **5.1.0**, run as a container (`nvcr.io/nvidia/isaac-sim:5.1.0` + a pip layer),
`network_mode: host`, `ipc: host`, persistent kit/ov/shader caches under `~/.cache/isaac-sim`.
The image carries Isaac's own Humble **and** Jazzy bridge libraries; `launch.sh --ros-distro`
picks one (Jazzy for this stack). Isaac is Python 3.11, so no host ROS ever enters the container.

Scenes that existed: `base_room` (6x6 procedural room with campaign layouts), `office` and
`hospital` (NVIDIA assets from the asset server), `gazebo_empty_room`.

### 1.2 The stack side (this repository, already written)

The HIL plumbing is largely in place and was built for the Thor, not the Orin:

- `supervisor_bringup.launch.py sim:=isaac` - **the same bringup as on the car**. It drops the
  device drivers (`vesc.launch.py`, `lidar.launch.py`), swaps `camera.launch.py` for
  `sim_camera_tf.launch.py` (ZED frames only), puts every component on `/clock` and loads
  `config/sim/isaac.yaml` after the normal parameters. No sim-specific node, no sim-specific
  topic: only parameters differ. `supervisor_bringup.launch.py:213-236`.
- `config/sim/isaac.yaml` - the only parameter differences: the mission archive goes to
  `~/f1tenth_sim_archive`, mission preflight expects Isaac's bridge node instead of
  `ackermann_to_vesc_node`, `/reset_all` zeroes wheel odometry through Isaac's `/sim/reset`.
- `config/sim/dds/{local.xml, isaac_local.xml, eth.xml.in}` - Fast DDS profiles with the stack's
  5 s participant lease; `eth.xml.in` is the two-machine template, rendered per host.
- `scripts/sim/sim_isaac.sh` - `isaac|stack|all|reset|shell|run`, `--net local|eth`, Discovery
  Server handling, `/clock` wait, clean shutdown. The two-machine mode assumes **the Thor**
  (`10.42.0.2`) runs the stack natively.
- `output/todo_for_andreas.md` section 5 in `isaac_ws` lists what was left for the Orin: same
  domain, same RMW, the Discovery Server question, `use_sim_time`, `publish_static_tf: false`.

So the HIL work is: bring the sim into the repository, add the worlds, and replace "Thor,
native" with "Orin, Jazzy container on host networking".

## 2. The office world

The addendum settled it: the office is the Gazebo world already in the `jazzy` branch.

| where | what |
|---|---|
| `src/husarion_gz_worlds` | submodule, `https://github.com/husarion/husarion_gz_worlds.git`, pinned at `87271ef` (0.1.0, 2026-06-22), **Apache-2.0** (`LICENSE`). 60 MB: models (Surfaces, Door, Kitchen, Tv, Laptop, Rosbot, ConferenceTable, WaterDispenser, HusarionLogo), worlds, maps. Added to `.gitmodules` on `jazzy` only, so this branch had to take the same entry. |
| `src/husarion_gz_worlds/worlds/husarion_office.sdf` | the upstream office, 8797 lines. Its wall and floor poses live in a `<state>` block. |
| `src/f1tenth_sim/worlds/husarion_office_2x.sdf` | **what the Gazebo sim actually loads** (`sim_bringup.launch.py:133` default). The office scaled 2x in x/y by `tools/scale_office_world.py` (walls/floors scaled, furniture moved, state baked, one 4 ms physics block). Brought onto this branch together with the generator. |

Measured from the SDF by `sim/isaac/tools/sdf_to_usd.py --plan` (this branch, runs without Isaac):

- 99 models, 157 visual and **148 collision** geometries, 47 unique meshes, **210 174 triangles**
- extent **28.53 x 27.11 x 2.60 m**, floor at z = 0, walls 2.5 m high
- every structural model has collision geometry: `Surfaces` (14 floor + 2 wall meshes),
  `OpenRobotics/Door`, `Glass_wall_1`, `Glass_wall_2` - `structure_without_collision: []`
- scaled geometry is metric; three meshes are authored in millimetres (Tv, Kitchen, Laptop,
  Rosbot parts) and the SDF compensates with `<scale>0.001</scale>` - the conversion must author
  that scale on the referencing prim, which it does
- no flipped STL facets (every facet normal agrees with its winding), no non-Z-up COLLADA
- 573 m2 of drivable floor at the lidar plane, measured per triangle

Why a conversion and not an import: **Isaac Sim 5.1 has no SDF importer.** It ships URDF, MJCF
and CAD importers plus `omni.kit.asset_converter` for meshes; SDF is Gazebo's format and nothing
in the 5.1 image reads it. So `tools/sdf_to_usd.py` converts the meshes with the asset converter
(cached by content, one USD each) and authors the world structure - model tree, poses, scales,
static flags, which geometry is visual and which is collision - from the SDF with pxr. The SDF
stays the single source of truth for both simulators, and no original file is touched.

Two findings that change how the world is used:

- **The modelled door is not a doorway.** `OpenRobotics/Door` is a closed leaf
  (x 10.03..10.27, y -7.07..-6.02) and a `Stairs` collision box stands 0.6 m in front of it
  (x 10.72..10.93, y -7.41..-6.41, z 0.17..0.33). Driving at it is driving at a wall and then up
  a step. `tools/pick_spawn_poses.py` therefore looks for real openings in the wall meshes - two
  jambs with a gap, which is also what the door detector is written for - and found ten; the best
  is **1.25 m wide at (8.064, -19.404)** with 6 m of free space on both sides.
- **The world has no people.** No `<actor>`, no character models. Nothing to map, so the walking
  people in `sim/people.py` are ours, off by default (see 4.3).

Spawn poses, computed from the SDF (not chosen by eye) and listed in `setup.md`:

| layout | pose | why |
|---|---|---|
| `corridor` | (16.814, 0.946), yaw -90 deg | 24.85 m clear ahead, 1.25 m to each side |
| `door` | (10.064, -19.404), yaw 180 deg | 2.0 m in front of the 1.25 m doorway, 1.48 m clearance, 0.65 m minimum on the way in |
| `open_space` | (15.364, -11.554), yaw 0 | the largest free circle on the floor: 2.80 m |

(The earlier search for a downloaded "humans2" office pack found nothing on linus: no
`*.usd/*.usda/*.usdz` outside the hashed Omniverse cache and one OceanSim demo, nothing matching
`/human/i`, no Isaac install, nothing in `~/Downloads`. The addendum replaced that search.)

## 3. The sim/stack boundary

From `isaac_ws/config/ros_interface.yaml`, which names its source for every entry, checked
against this branch.

### 3.1 Isaac publishes

| topic | type | rate | QoS | real car |
|---|---|---|---|---|
| `/clock` | Clock | 60 Hz (every physics step) | reliable d10 | - (sim only) |
| `/scan` | LaserScan, 1081 beams, -135..+135 deg, 0.25 deg | 40 Hz | reliable d5 | `urg_node`, same beams, 39.94 Hz measured |
| `/odom` | Odometry | 50 Hz | reliable d10 | `vesc_to_odom`, 50.0 Hz |
| `/sensors/imu/raw` | Imu | 50 Hz | reliable d5 | `vesc_driver`, 50 Hz (20 ms poll) |
| `/camera/image_raw` + `/camera/camera_info` | Image bgra8 640x360 | 15 Hz | reliable d5 | ZED wrapper remapped by `camera.launch.py:122-125`, 30 Hz configured |
| `/zed2/zed_node/depth/depth_registered` + `depth/camera_info` | Image 32FC1 640x360 | 15 Hz | reliable d5 | ZED wrapper, 30 Hz configured (~17 measured) |
| `/zed2/zed_node/left|right/image_rect_color` (+ info) | Image | 15 Hz | reliable d5 | the wrapper's own names; nothing in the stack subscribes |
| `/sim/ground_truth/{odom, person_pose, front_range, objects, start_pose}` | measuring only | 50/50/20/1 Hz | - | never on the car, never read by the stack |
| `/tf`, `/tf_static` | TFMessage | - | reliable, static transient_local | **off in HIL**: `publish_static_tf: false`, the stack owns TF |

### 3.2 Isaac subscribes

| topic | type | from |
|---|---|---|
| `/ackermann_drive` | AckermannDriveStamped, reliable d1 | the mux output - exactly what `ackermann_to_vesc` consumes on the car (`ackermann_mux.launch.py:29`) |
| `/sim/reset` | std_srvs/Trigger (service) | `reset_manager` through `config/sim/isaac.yaml` |

### 3.3 Mismatches against the real car

1. **ZED mount, 2.5 cm in x.** `car_spec.yaml` places the lens midpoint at the taped
   (0.255, 0, 0.22); the car's deployed `/tf_static` (bag 2026-10-15, `sim_camera_tf.launch.py:31-34`)
   puts `zed2_camera_link` at (0.2399, 0, 0.205), i.e. the lens midpoint at (0.2294, 0, 0.2196).
   In HIL the stack publishes the TF and Isaac only renders, so the sim must use the stack's
   numbers or every depth-to-lidar comparison carries a 2.5 cm bias. `sim_isaac.sh:263-267`
   already passes exactly these overrides; the HIL script keeps them.
2. **Camera rate 15 vs 30 Hz.** The sim's 15 Hz was chosen for render cost; the wrapper is
   configured for 30 Hz and measured ~13.7 Hz on `camera_info` in the reference bag. 15 Hz is
   therefore close to the car's effective rate and halves the link load. Kept, as a parameter.
3. **IMU units.** The real VESC IMU publishes gyro z in rad/s but gyro x/y in deg/s and
   acceleration in g (`vesc.yaml:107-188`); Isaac publishes SI throughout. Only vyaw is fused
   (`ekf.yaml`), so this does not change behaviour - but any new IMU consumer must know.
4. **Intensity.** The real `/scan` carries `intensities[]` (1081 values, `publish_intensity: true`,
   consumed by `glass_detector_node`); Isaac publishes none. **The glass detector cannot be
   exercised in HIL until the sim fills that field** - the open gap for the corridor glass world.
5. **Odometry covariance / IMU noise.** `car_spec.yaml` defaults differ from the car's
   `vesc.yaml`; `sim_isaac.sh:269-275` overrides them (pose_xy 0.2, pose_yaw 0.03, twist_vx 0.0,
   gyro stddev 0.0013217) so the EKF weighs odom against IMU as on the car. Kept.
6. **Depth conventions** already match (`-inf` too close, `+inf` too far, 32FC1 metres,
   min 0.3 / max 10.0 m from the wrapper's `zed2i.yaml`).

### 3.4 The real 6x6 test room

`src/f1tenth_sim/worlds/test_room_6x6.sdf` (on `jazzy`): 6.00 x 6.00 m inner, walls 0.10 m thick
and 2.5 m high, inner faces at +-3.0, two ceiling point lights at (+-1.5, 0, 2.4), car spawn
(-2.5, 0) facing +x, person spawned by the launch at (0, 0) facing the car. Reproduced 1:1 as
`scenes/room6x6.yaml` with the room corner at the origin (every coordinate +3, +3): car
(0.5, 3.0), person (3.0, 3.0), far wall 5.5 m ahead of `base_link`.

## 4. Decisions taken in this branch

### 4.1 Transport: Fast DDS, not Zenoh

`output/zenoh/recon.md` (on `main-jazzy-rust`) section 3.3: rmw_zenoh **cannot** be used with
Isaac Sim 5.1 - the bridge's internal Jazzy libraries ship Fast DDS and Cyclone only, Isaac is
Python 3.11 while the apt `rmw_zenoh` is 3.12, and NVIDIA documents Zenoh only from Isaac 6.x.
Its own recommendation list keeps HIL on Fast DDS. So: `rmw_fastrtps_cpp` on both sides, unicast
Discovery Server, no multicast anywhere.

### 4.2 Bandwidth

Measured/estimated in that same report for this exact scenario: `/camera/image_raw` and
`/zed2/.../depth_registered` are 0.92 MB each (640x360x4 B) at 15 Hz = 28 MB/s of payload, but
Fast DDS sends **one unicast copy per remote subscriber** (2 readers for the image, 3 for depth),
i.e. **~71 MB/s = ~570 Mbit/s** on the wire - past what a shared 1 GbE LAN gives, and the reason
Isaac's RTF fell from 1.00 to ~0.72 as soon as the Thor subscribed. Nothing may be changed in the
stack, so the knobs are all on the sim side (`config/hil.yaml -> sim.stereo`):
rate, resolution, and `right_camera: false`. The defaults for HIL are 640x360 at 10 Hz with the
right camera off: ~46 MB/s on the wire, which a dedicated gigabit link carries.

### 4.3 People

No actors in the SDF, so walking people are new code: `sim/people.py` moves characters along
waypoint polylines as kinematic rigid bodies with the person capsule collider, publishing their
live pose in `/sim/ground_truth/objects`. Off by default (`--layout people`). What it does **not**
do is a walk cycle - the mesh glides. Real walking needs `omni.anim.people` with a navmesh and a
command file; the same waypoints can drive it later. Paths were checked against the SDF occupancy
grid (minimum clearance 0.80 / 0.80 / 1.66 m).

### 4.4 Glossy floor and depth

The corridor world's point is that the real polished floor returns almost no ZED depth
(`stack_params.yaml:722-731`: on `corridor10m_20261015T171753` the wrapper's depth output
collapsed mid-run; the corridor's glass wall sits at a median 1.5 m). Isaac's depth is the
`distance_to_image_plane` annotator - geometric, and **not** degraded by a shiny material. So the
glossy layouts set the material (which is what RGB, YOLO and the scene mask see) **and** switch on
the floor-NaN depth model through the new `sensor_overrides` block. Saying "glossy floor" alone
would have changed nothing measurable.

## 5. Open gaps

1. `/scan` has no `intensities[]` in the sim: `glass_detector_node` cannot be exercised (3.3.4).
2. The office USD has not been generated or opened: `tools/sdf_to_usd.py --convert` needs Isaac,
   which linus does not have. The static plan passes; the visual check is tomorrow's first step.
3. `expected.wall_ahead_range_from_base` is unset for the office layouts - fill in from
   `tools/find_pose.py` on the workstation.
4. `Glass_wall_1/2` are not `<static>true</static>` in the SDF. The converter adds colliders and
   never a rigid body, so they stay put in Isaac; in Gazebo they are dynamic.
5. The Orin container has never run `sim:=isaac`: `scripts/sim/hil_check` exists to answer that
   question in one command, and has not been run.
