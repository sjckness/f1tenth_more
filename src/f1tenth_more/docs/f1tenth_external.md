# f1tenth_external (vendored submodules)

Five of the workspace's six git submodules (the sixth, `vesc`, is pinned
under `src/f1tenth_hardware/vesc` instead — see `f1tenth_hardware`'s doc,
since it's tightly coupled to that package). These are vendored/forked
third-party code, not owned by this project — brief by design; see each
submodule's own `README.md` for its real documentation.

| Submodule | Remote | Fork status | Role in this stack |
|---|---|---|---|
| `ackermann_mux` | `github.com/sjckness/ackermann_mux` (personal fork) | Forked from ROS's `twist_mux`, retargeted `Twist`→`AckermannDriveStamped`. Has local commits on top of the F1TENTH upstream lineage. | Priority-based arbitration between drive-command sources — see `f1tenth_control`'s doc. **Not launched via its own bundled launch file/config**; `f1tenth_control/ackermann_mux.launch.py` and `f1tenth_bringup/config/mux.yaml` are a separate, hand-written launch file + config with a different topic-naming scheme (deliberate non-reuse, documented in that launch file's own docstring). |
| `zed_ros2_wrapper` | `github.com/stereolabs/zed-ros2-wrapper` (clean upstream, tag `humble-v4.2.5`) | Zero local modification. | ZED2 stereo camera driver — `f1tenth_perception/camera.launch.py` includes its `zed_camera.launch.py` when `camera_source=zed`, with `camera.launch.py`/`zed2_perception.yaml` overriding several params (depth mode, publish topics, disabling position-tracking/mapping/object-detection). |
| `zed-ros2-interfaces` | `github.com/stereolabs/zed-ros2-interfaces` (clean upstream, tag `5.3.0`) | Exact match to upstream `master`. | Message/service types (`zed_msgs`) consumed internally by `zed_ros2_wrapper`. Not directly imported by any first-party package — `f1tenth_perception` uses `vision_msgs`' own `BoundingBox3D`, an unrelated same-named type; don't confuse the two. |
| `teleop_tools` | `github.com/f1tenth/teleop_tools` (F1TENTH org's vendoring of PAL Robotics' upstream) | Clean, no local fork history. | Only `joy_teleop` (of its 3 sibling nodes) is actually wired in, via `f1tenth_control/joy.launch.py` + `f1tenth_bringup/config/joy_teleop.yaml`. `key_teleop`/`mouse_teleop` are not referenced anywhere in this workspace. |
| `transport_drivers` | `github.com/ros-drivers/transport_drivers` (real upstream, tag `1.2.0`) | No local modification. | Only the `serial_driver`→`io_context`→`asio_cmake_module` chain is load-bearing, consumed as a **compiled C++ library** (not a node) by the `vesc` submodule's `vesc_driver` for real serial I/O to the VESC over `/dev/ttyACM0`. `udp_driver`'s 3 executables are unreferenced anywhere in this workspace. |

## Why vendored as submodules rather than `rosdep`/apt packages

`ackermann_mux` and `vesc` need real local patches (topic remaps, param
additions, sign-convention fixes) on top of upstream — a submodule pin makes
those changes trackable and reproducible. `zed_ros2_wrapper`/
`zed-ros2-interfaces`/`transport_drivers`/`teleop_tools` are vendored clean
(or near-clean) mainly to pin an exact known-working version rather than
floating on whatever `apt`/`rosdep` would resolve at build time.

## Known limitations

- Several nodes across these submodules are registered but never launched by
  anything in this workspace (`ackermann_mux`'s `joystick_relay.py` isn't
  even installed — commented out in its own `CMakeLists.txt`; `teleop_tools`'
  `key_teleop`/`mouse_teleop`; `transport_drivers`' `udp_driver` executables;
  `vesc_driver`'s `vesc_device_namer` udev-helper). None of this is a bug —
  they're just unused capability the vendored code happens to also ship —
  but worth knowing before assuming something is wired up because the code
  exists.
- Each submodule's pinned commit can, in principle, drift from what
  `f1tenth_bringup`'s config files assume about its param names/defaults if
  the pin is ever bumped without re-checking those assumptions — there is no
  automated check for this.
