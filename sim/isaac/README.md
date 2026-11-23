# sim/isaac - Isaac Sim side of the HIL setup

The simulated F1Tenth car for hardware-in-the-loop runs: Isaac Sim 5.1 on the workstation, the
**unchanged** car stack on the Jetson Orin (`sim:=isaac`, see `output/hil/setup.md`). Everything
here is part of this repository: a plain `git clone` + `git submodule update --init` is enough.

    launch.sh                 start the container and the sim (host side and container side)
    docker/                   the Isaac Sim image (build.sh, Dockerfile, docker-compose.yaml)
    config/car_spec.yaml      the car: geometry, mass, limits, SENSOR POSES (every value has a source)
    config/sensors.yaml       lidar / stereo / IMU / odometry parameters and their noise models
    config/ros_interface.yaml topic names, frame ids, QoS - the contract with the stack
    config/hil.yaml           the ONE machine-specific file (IPs, paths, container names)
    config/scenes/*.yaml      the worlds (see below)
    sim/                      the simulation itself (scene, car, sensors, ROS bridge, people)
    tools/                    conversion, pose picking, probes, static checks
    usd/office_hil.usda       thin wrapper around the generated office stage

## Worlds

| `--world` | what it is | needs |
|---|---|---|
| `office` | the husarion office, converted from the Gazebo SDF the stack's Gazebo sim runs. Layouts `corridor`, `door`, `open_space`, `people`. | `tools/sdf_to_usd.py --convert` once, on a machine with Isaac |
| `room6x6` | the real 6x6 m test room, 1:1 with `f1tenth_sim/worlds/test_room_6x6.sdf`. Layouts `default`, `glossy`, `plant`, `obstacle`. | nothing |
| `corridor` | a 20 x 2.2 m corridor, glossy floor, optional glass wall. Layouts `glossy`, `matte`, `glass`, `person`. | nothing |
| `office_nvidia`, `hospital`, `base_room` | the earlier NVIDIA-asset scenes, kept as fallbacks | the NVIDIA asset server |

## Checks that run anywhere (no Isaac, no GPU)

    sim/isaac/tools/check_config.py --layouts     # every scene and every layout loads
    sim/isaac/tools/sdf_to_usd.py --plan          # the office SDF: meshes, colliders, scale, bbox
    sim/isaac/tools/pick_spawn_poses.py           # spawn poses computed from the SDF

## Provenance

Vendored from `~/isaac_ws` (branch `f1tenth`, commit `4d2de1e`, 2026-10-16) with the changes
listed in `output/hil/recon.md`. The office world comes from
[husarion_gz_worlds](https://github.com/husarion/husarion_gz_worlds) (Apache-2.0), pinned as the
submodule `src/husarion_gz_worlds`; its meshes are never modified.
