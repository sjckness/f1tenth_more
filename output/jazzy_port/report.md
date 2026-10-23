# jazzy-campaign: campaign-final on Jazzy (linus) + Isaac sim mode: report

**Morning summary**
- 🟢 **P0** Source = origin/campaign-final **7b17e2f**. The car's `e3793e47` and `736d46f` were never pushed, so the port is older than the car (todo §1).
- 🟢 **P1** `plan.md`: 159 jazzy-only commits sorted into 19 cherry-picks, 8 partial, 19 proposals, 113 skipped. 20 proposals.
- 🟢 **P2** Branch `jazzy-campaign`: 25 port commits; colcon 27/27 packages, 0 errors. All test failures are pre-existing on campaign-final. Review: no logic, parameter or default changed.
- 🟢 **P3** `sim:=none|isaac` in `supervisor_bringup`. Default arguments: byte-identical launch description. sim: 54/54 nodes on /clock.
- 🟢 **P4** `tools/e2e_test.sh` on linus (base_room): **9/9 PASS**, clean stop.
  - Wall error 2.4 mm; person 3D error 0.10 m; 2 m mission (2.019 m ground truth) recorded by the logger; reset 1.6 cm.
  - RTF 0.99; VRAM 4.0/8 GB.
- Nothing pushed. Branches: `jazzy-campaign` (superproject + vesc + ackermann_mux). isaac_ws `f1tenth`: ba9c29b, f07f6c3, 54468a3 on top.
- **Open 1:** push the Orin's campaign-final, then bring `jazzy-campaign` forward to it (todo §1).
- **Open 2:** Thor: build with ZED v5.4.1 + VESC (skipped on linus), then the two-machine sim (todo §6).
- **Open 3:** Isaac fidelity: the parked car creeps (-6.5 mm/s, D20), and the T-posed person makes `go_to_person` stop 0.9 m early (D23).
- Commands:
  - `scripts/sim/sim_isaac.sh all --scene base_room` (one machine)
  - `… isaac --net eth` on linus + `… stack --net eth` on the Thor
  - `tools/e2e_test.sh`

---

## 1. Phase 0: what is campaign-final

| item | value |
|---|---|
| commit in the newest car bags | `e3793e47f595c501e0c9796e93791faf0cc9edef` (2026-10-15, dirty tree), `736d46fe14` (2026-10-14) |
| on any remote? | **no** (`git fetch origin e3793e47…` → `not our ref`; not on linus or /HDD2T either) |
| used as the source | **origin/campaign-final 7b17e2f** (2026-10-14 13:50) |
| what the car has that 7b17e2f lacks | params diff of 160 lines (`car_params_vs_7b17e2f.diff`): scene mask, multi-wall tracker, wall fusion, stage timing, depth recording, `enable_scene_labels` true; laser z 0.19 vs 0.20 |
| divergence | origin has 3 linus commits that the Orin lacks (`cdfd736`, `a8a61db`, `7b17e2f`: docs and campaign data), so a plain push from the Orin is rejected (todo §1 gives a merge without force) |

Submodule pins (all fetchable):

| submodule | pin | jazzy-campaign |
|---|---|---|
| vesc | 2848a9f | 2848a9f (= the fork's jazzy) |
| ackermann_mux | 42b7cd7 | 42b7cd7 (kept) |
| teleop_tools | 4337558 | same |
| transport_drivers | d3f510c | removed (apt serial_driver) |
| zed_ros2_wrapper | b5844a8 (humble-v4.2.5) | 86171bc (v5.4.1) |
| zed-ros2-interfaces | e970087 | same |

Details: `phase0.md`.

## 2. Packages (Phase 2, `phase2.md`)

| package | port action | linus build | tests (pass/total; failures) |
|---|---|---|---|
| f1tenth_messages | campaign-final | ✓ | 3/3 |
| f1tenth_params | campaign-final | ✓ | 54/54 |
| f1tenth_description | campaign-final | ✓ | 22/22 |
| corridor_perception | campaign-final | ✓ | 37/56 (19 skipped) |
| f1tenth_costmap | campaign-final (P1 = inflate_polytope proposal) | ✓ | 198/198 |
| f1tenth_logger | 6 rosbag2/Jazzy API commits | ✓ | 313/318 (3 lint, 2 checkout-name, P16) |
| mpc_controller | test_depend | ✓ | 727/741 (11 functional, pre-existing; 2 lint) |
| f1tenth_control | campaign-final | ✓ | 7/7 |
| f1tenth_navigation | slam_toolbox lifecycle + exec_depends | ✓ | 2/2 |
| f1tenth_localization | super-client env | ✓ | 14/14 |
| f1tenth_diagnostics | super-client env | ✓ | 149/153 (lint, pre-existing) |
| f1tenth_perception | ZED v5.4.1 remaps + pinned v4 defaults | ✓ | 614/614 |
| f1tenth_behavior | campaign-final (+ Phase 3 preflight parameter) | ✓ | 352/353 (known flake) + 3 new |
| llm | campaign-final | ✓ | 234/272 (12 stub tests, P16; 24 skipped) |
| f1tenth_hardware / vesc_ackermann | campaign-final | ✓ | – |
| vesc_driver, vesc | campaign-final | **skipped on linus** (apt serial_driver needs sudo) | Thor |
| zed_components, zed_wrapper, zed_debug, zed_ros2 | v5.4.1 | **skipped on linus** (no ZED SDK) | Thor |
| f1tenth_bringup | campaign-final + Phase 3 sim mode | ✓ | 37/40 (lint, pre-existing) + 15 sim-mode tests |
| f1tenth_sim (Gazebo) | `COLCON_IGNORE` | – | – |

Build: `colcon build --symlink-install --packages-ignore $(cat output/jazzy_port/linus_skip.txt)`, 27 packages, 0 errors, warnings only in upstream teleop_tools. Dependencies missing on linus come from user-space stand-ins:
- overlay `~/dev_ws/jc_deps_ws`: slam_toolbox 2.8.5, py_trees_ros, topic_tools;
- venv `~/dev_ws/jc_venv`: osqp, pyarrow, ultralytics, torch cu128.

Source `~/dev_ws/jc_env.sh` to use them (D6). The apt commands are in todo §3.

## 3. Conflicts and how they were resolved

| where | resolution |
|---|---|
| ackermann_mux pin (`b3c0b08` vs `42b7cd7`) | keep 42b7cd7: fetchable, differs only in an unused yaml (plan §0.3) |
| `52682de` vs `472f036` (TopicMetadata test fix) | `52682de` only; they are mutually exclusive |
| cherry-pick of `3d545db`: modify/delete of the transport_drivers gitlink | deletion kept (`git rm --cached`), leftover directory removed |
| `scripts/env/jazzy.sh` exports the lease profile (P3) | taken without the export |
| YOLO TensorRT guard, rclpy try_shutdown, inflate_polytope | proposals P10, P19, P1 (node logic) |
| CLAUDE.md sections | Rebuild policy + Orin Docker taken; the DDS lease and Thor sections left out |
| f1tenth_sim | `COLCON_IGNORE`; Isaac replaces Gazebo |
| jazzy's Gazebo `sim:=true` (P8) and preflight-in-sim (P9) | re-derived as the behaviour-neutral `sim:=isaac` (D5, D9, D13) |

## 4. linus skip list

`linus_skip.txt`: `zed_components zed_wrapper zed_debug zed_ros2 vesc_driver vesc`. What to check on the Thor is in `phase2.md` §"Skip list". Most important: the ZED v5 optical frame is renamed to `zed2_left_camera_frame_optical`, and `enable_image_validity_check: 0` is pinned.

## 5. Sim mode (Phase 3, `phase3.md`)

The sim stack is the car's stack. The only differences are parameters and which launch files run.
- `ros2 launch f1tenth_bringup supervisor_bringup.launch.py sim:=isaac` forces `use_sim_time:=true`. The supervisor (`apply_sim_mode`, a pure function):
  - drops `vesc.launch.py` and `lidar.launch.py`;
  - replaces the driver parts with TF-only launch files: `sim_hardware_tf` (base_link→imu) and `sim_camera_tf` (the deployed ZED mount plus the ZED frame tree, v4 names);
  - starts every component through `sim_component.launch.py`. That wrapper sets `use_sim_time` as a launch configuration and as a global parameter, and loads `config/sim/isaac.yaml` after the normal parameters.
- The overlay `config/sim/isaac.yaml` has 3 entries, each with its reason: sim archive dir, preflight actuation node `isaac_f1tenth_sim`, reset_manager wheel odom via `/sim/reset`. Host overlay `config/sim/hosts/linus.yaml`: CPU pinning 6,7 and 4,5, because the Orin's 10,11 and 8,9 do not exist on 8 cores.
- `use_sim_time:=true` alone only switches the clock. The supervisor itself stays on wall time (D10). `stack_bringup` (the fallback) is not extended (D11).
- **Back-port to campaign-final is pending** (D9, todo §5). It is API-neutral on Humble.
- Sim-side alignment is done only in `sim_isaac.sh` (Isaac `--set`), never in the stack (D15, D16):
  - `publish_static_tf=false`;
  - ZED lens at the deployed TF;
  - the car's odom covariances and gyro noise.
- IMU: Isaac keeps SI. The stack fuses only vyaw, which is rad/s on both the car and the sim (D17).
- `scripts/sim/sim_isaac.sh` subcommands: `isaac [--scene] [--gui]`, `stack [args]`, `all`, `reset`, with `--net local|eth` and `--domain`.
  - DDS profiles live in `config/sim/dds/`.
  - Pre-checks before start:
    - docker and the nvidia runtime;
    - the image is present (never pulled);
    - at least 25 GB free disk;
    - VRAM printed.
  - `stack` starts the Discovery Server and waits up to 30 s for `/clock`.
  - `--help` documents the two-machine sequence.
  - It is shellcheck 0.11.0 clean.
- isaac_ws `launch.sh --ros-distro {humble,jazzy}` (default jazzy) and `--dds-profile`: commit ba9c29b on `f1tenth`.

### 5.1 Default launch description: identical

`tools/sim/launch_snapshot.py` resolves the supervisor's registry and every Node of every component without starting anything. Before = d529557 (Phase 2), after = Phase 3. Results are in `launch_snapshots/`.

| arguments | result |
|---|---|
| (none) | **IDENTICAL**, byte-identical JSON, 0 diff lines (15 components, 38 processes) |
| `sim:=none use_sim_time:=false` | IDENTICAL |
| `calibration:=true` | IDENTICAL |
| `enable_intelligence:=false` | IDENTICAL |

Re-checked after the review fix of the supervisor: still identical. Limits: `camera.launch.py` and the hardware drivers' nodes resolve only partly off the car, the same way on both sides. Their sources are unchanged (phase3.md §3).

### 5.2 TF owners (live; e2e step 4)

| edge | owner |
|---|---|
| map → odom | `ekf_global/ekf_global_filter_node` |
| **odom → base_link** | **`ekf_filter_node` only.** Isaac `publish_odom_tf: false`; it matches the EKF output in 351/351 messages, given the 50 ms `transform_time_offset` |
| base_link → laser, base_front, chassis, wheels | description.launch.py (`static_baselink_to_laser`, robot_state_publisher, joint_state_publisher) |
| base_link → imu | `static_baselink_to_imu` (sim_hardware_tf; same arguments as vesc.launch.py) |
| base_link → zed2_camera_link → center → left/right → optical | `static_baselink_to_zed2` + `zed2/zed2_state_publisher` (deployed: 0.2399, 0, 0.205, pitch -2.083°) |
| anything from Isaac | nothing (`publish_static_tf=false`) |

### 5.3 Topic coverage (live; e2e step 2)

127 topics on 61 nodes, built from the graph API.
- The sim feeds `/scan`, `/odom`, `/sensors/imu/raw`, `/camera/image_raw`, depth + info, and `/clock`. The stack consumes all of them.
- 12 topics are subscribed but have no publisher. None is a sim gap: they are unpublished on the car too (`/imu`, `/model/virtual_robot/odometry`), device-only (`/sensors/core`: the battery lane stays "no data", never "low"), or manual and optional (teleop, set_pose, nav2 lane, ...). See phase3.md §6.

## 6. e2e on linus (Phase 4, `phase4.md`, raw data `e2e/`)

| step | result | key numbers |
|---|---|---|
| up | PASS | /clock 26 s, stack ready 61 s |
| graph | PASS | 0 unexpected missing topics |
| hz | PASS | scan 39.6, odom 49.5, imu 49.5, camera 14.9, depth 14.9, detections 14.9, EKF 45.9 Hz |
| tf | PASS | 19 edges, single owners, deployed poses |
| perception | PASS | person score 0.93, 3D error median 0.10 m; front wall 5.311 m, error 2.4 mm (tolerance 0.10) |
| mission | PASS | `drive_test_straight_2m` through the mission manager: COMPLETE; ground truth 2.019 m; 9 SLAM updates; logger bag closed in `~/f1tenth_sim_archive/complete/` |
| reset | PASS | `/reset_all` → `/sim/reset` in 5.3 s; car 1.6 cm from the start pose; EKFs restart at 0 |
| person | PASS* | `go_to_person` COMPLETE, but it stops 1.49 m from the body because of the T-posed sim person (D23) |
| perf | PASS | RTF 0.993 (min 0.96) |

## 7. Performance

linus: i7-9700K (8 cores), 32 GB, RTX 5050 8 GB.

| | mean | p95 / max |
|---|---:|---:|
| stack CPU (100 % = 1 core) | 287 % | 361 / 387 % |
| Isaac CPU | 129 % | 134 / 233 % |
| host CPU | 73 % | 86 / 94 % |
| stack RAM | 4.8 GB | 5.9 GB |
| Isaac RAM | 4.4 GB | 4.7 GB |
| host RAM | 16.7 GB | 17.5 GB of 32 |
| VRAM | 3.8 GB | 4.0 GB of 8.1 |
| GPU utilisation | 27 % | 40 % |
| RTF | 0.993 | min 0.96 |

MPC: 10 Hz, no failed solves. `solve_dt_sec` reads 0 in sim because it is measured on the node clock (P21). The mpc_corr process CPU while RUNNING is 49–55 % of a core, 59–76 ms per tick; that is an upper bound, since it includes all of the node's callbacks.

## 8. Proposals (logic changes NOT made)

- **P1–P20**: jazzy's logic fixes, all in `plan.md` §7.1. The most relevant:
  - P1 inflate_polytope hang on aarch64;
  - P3 DDS lease profile;
  - P5 health watchdog;
  - P6 wall-stop preflight;
  - P11 / P12 MPC None and steering clip;
  - P16 tests that fail on campaign-final itself;
  - P17 **do not adopt**.
- **P21**: time the MPC solve with a steady clock (behaviour-neutral on the car).
- **P22**: Nav2 servers on `use_sim_time` when `enable_nav2: true` in sim.
- **P23**: mission bag recorder on `/clock` in sim.

## 9. Isaac-side findings (isaac_ws `f1tenth`, commits on top only)

- **D25, creep:** the parked car moves backwards at -6.5 mm/s with yaw -0.005 rad/s; the wheels turn against zero targets. Three fixes failed: wheel kd, steering kd, position brake.
- **D26, T-posed person:** the arm span makes the 2D box 1.4 m wide.
- Committed: ba9c29b (distro and DDS options), f07f6c3 (gain log + D25), 54468a3 (D26).

## 10. To do (details in `todo_for_andreas.md`)

1. **Orin push** (§1): commit or keep the Orin's dirty tree, `git fetch origin`, `git merge origin/campaign-final`, `git push origin campaign-final` (fast-forward, never force). Then bring `jazzy-campaign` forward.
2. Stale lock `~/dev_ws/f1tenth_more/.git/modules/ackermann_mux/index.lock` (§2).
3. apt (sudo) for linus: slam-toolbox, py-trees-ros, topic-tools, serial-driver, ... (§3); and shellcheck (§4).
4. Back-port the sim mode to campaign-final as a new branch (§5).
5. **Thor** (§6):
   - build jazzy-campaign with ZED and VESC;
   - check the ZED v5 topics and frames;
   - run the two-machine sim (`isaac --net eth` on linus, `stack --net eth` on the Thor);
   - run the e2e checks with the eth environment;
   - check the camera bandwidth, about 28 MB/s.
6. Isaac: creep (§7) and person pose (§8).

## 11. Where everything is

| what | path |
|---|---|
| worktree / branch | `~/dev_ws/f1tenth_more_jc` / `jazzy-campaign` (no upstream set, nothing pushed) |
| docs | `output/jazzy_port/` |
| main checkout | `~/dev_ws/f1tenth_more` (campaign-final, untouched) |
| jazzy reference worktree | `~/dev_ws/f1tenth_more_jzref` (detached at origin/jazzy, read-only use) |
| user-space deps | `~/dev_ws/jc_deps_ws`, `~/dev_ws/jc_venv`, `~/dev_ws/jc_env.sh`; delete after apt (todo §3) |
| sim mission archive | `~/f1tenth_sim_archive/` (e2e runs) |
