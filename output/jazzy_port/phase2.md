# Phase 2: the jazzy-campaign branch

Worktree `~/dev_ws/f1tenth_more_jc`, branch `jazzy-campaign`, built from origin/campaign-final `7b17e2f` (Phase 0). The steps follow `plan.md` §5. Nothing was pushed.

## Commits (in order)

| Plan step | Commit | What | Source on jazzy |
|---|---|---|---|
| 5 | `825ed38` | `.gitmodules`: `branch = jazzy` for vesc (not for ackermann_mux, whose pin stays `42b7cd7`) | `6118239` (partial) |
| 6 | `a12fc04` | transport_drivers submodule removed; serial_driver/io_context come from apt | `3d545db` (modify/delete conflict on the gitlink resolved by keeping the deletion) |
| 10 | `cb23049` | `ROS_LOCALHOST_ONLY` → `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` | `a086f0e` |
| 10 | `47b30dc` | mcap install hints say `ros-jazzy-` | `130063c` |
| 10 | `cf3b28b` | `rosbag2_py.TopicMetadata(id=)` in the logger test | `52682de` |
| 10 | `9650483` | `RecordOptions.all` → `all_topics` (guarded) | `de1f8d8` |
| 10 | `6e64fce` | `read_bag` takes the storage plugin from the bag | `91d361e` |
| 10 | `0c6320f` | `ROS_SUPER_CLIENT` for the mission logger | `84a12f1` |
| 11 | `4a5e212` | mpc_controller `<test_depend>f1tenth_hardware</test_depend>` | `2ea8541` |
| 12 | `3954b6a` | slam_toolbox launched as a LifecycleNode with configure/activate | `4edcbbf` |
| 12 | `e02ba59` | f1tenth_navigation declares `slam_toolbox`, `lifecycle_msgs` | new (packaging) |
| 13 | `1fe8e43` | `ROS_SUPER_CLIENT` for ekf_cost_observer_node only | `a936c75` |
| 13 | `f7087b4` | `ROS_SUPER_CLIENT` for the steering calibration launch tree | `5b320a5` |
| 14 | `bddd014` | zed_ros2_wrapper `b5844a8` (humble-v4.2.5, `#error` on Jazzy) → `86171bc` (v5.4.1) **plus** the camera port: SetRemap sources follow the v5 topic rename, `zed2_perception.yaml` pins the humble-v4.2.5 zed2i defaults (grab 30 Hz, depth 0.3–10 m) | `9a66870` + new |
| 16 | `3d5ee89` | `src/f1tenth_sim/COLCON_IGNORE` (the Gazebo Fortress sim does not build on Jazzy; the Isaac sim replaces it) | `0607db8` |
| 19 | `ca1a2f9` | `scripts/stackctl.py` runs as a super client | `d29b3e0` |
| 22 | `1f24dff` | CLAUDE.md `## Rebuild policy` | `b0740ea` |
| 24 | `509e797` | `.gitignore` for bulk per-run artifacts under output/ | `5c82958` |
| 18 | `e19b487` | `scripts/env/jazzy.sh` without the lease-profile export (P3) | `d9a7f00`, `5b9ffdc`, `dac4bef`, `39b2b71` (partial) |
| 20 | `c9fd44d` | tools: discovery range, mcap-aware readers, `ros2 bag record --topics` | patterns of `a086f0e`, `91d361e` |
| 21 | `ffc7356` | `docker/orin` (Orin Jazzy image) without `spike/` | `f574e43`, `1b50dce`, `f7f06be`, `8ecc13b`, `af1f167` (partial) |
| 22 | `250bffc` | CLAUDE.md `## Orin Docker` | `f574e43` (partial) |
| 23 | `e04d02a` | `scripts/jazzy_parity/test_results_summary.py` | `cf53ed0` (partial) |
| 25 | `fdfc3cf` | README: Jazzy, transport_drivers from apt | new (docs) |

Steps 1–4, 7–9, 15, 17 are "keep campaign-final" (no commit). The guard grep for jazzy-only / test-campaign logic markers (plan step 2) prints the same single baseline hit after the last step (`static_frames.py:14`, a campaign-final docstring).

Total delta vs campaign-final (`git diff --stat 7b17e2f HEAD`, without output/): 56 files, +2918/−29; of that, `src/` is 16 files, +154/−21. Everything else is the new docker/orin infrastructure, the env script, tooling and docs.

### Submodules

| Submodule | Pin | Local branch `jazzy-campaign` | Re-applied migration |
|---|---|---|---|
| vesc (sjckness fork) | `2848a9f` | created at `2848a9f` (in the worktree's module store and fetched into `~/dev_ws/f1tenth_more/.git/modules/vesc`) | none needed: the pin IS the fork's `jazzy` branch tip |
| ackermann_mux (sjckness fork) | `42b7cd7` | created at `42b7cd7` (both module stores) | none: the fork's `jazzy` branch (`b3c0b08`) is the pin's parent; the only difference is `config/ackermann_mux_topics.yaml`, which the stack never loads |
| teleop_tools (upstream f1tenth) | `4337558` | not a fork: no branch | none (works on Jazzy; setup.cfg dash-key warnings only) |
| transport_drivers (upstream) | removed | – | replaced by apt `serial_driver`/`io_context` (`3d545db`) |
| zed_ros2_wrapper (upstream) | `86171bc` (tag v5.4.1) | not a fork | bump + camera port in `bddd014` |
| zed-ros2-interfaces (upstream) | `e970087` (5.3.0) | not a fork | none |

**No pushes.** The superproject commits reference only commits that are already on GitHub (`2848a9f` on sjckness/vesc `jazzy`, `42b7cd7` on sjckness/ackermann_mux `humble-jetson`, upstream tags). The local `jazzy-campaign` branches in the forks are a convenience for the user to push (`todo_for_andreas.md`).

## Build on linus

```
source ~/dev_ws/jc_env.sh      # /opt/ros/jazzy + user-space overlay (slam_toolbox, py_trees_ros, topic_tools) + venv site-packages
colcon build --symlink-install --packages-ignore $(cat output/jazzy_port/linus_skip.txt)
```

- `--packages-ignore`, not `--packages-skip`: with `--packages-skip`, colcon still sources the skipped packages' (non-existent) install as dependencies of f1tenth_perception and fails (first attempt, log kept in this file's history).
- **Result: 27 packages finished, 0 errors** (`build_linus.log`).
- Warnings: only `joy_teleop`, `key_teleop`, `mouse_teleop` (upstream teleop_tools: `SetuptoolsDeprecationWarning: Invalid dash-separated options 'script-dir'`, fixed upstream on `humble-devel` `163827a`). **None in our packages.**

### Skip list (`linus_skip.txt`) and what must be checked on the Thor

| Package | Why not on linus | Verify on the Thor |
|---|---|---|
| zed_components | needs the ZED SDK (`/usr/local/zed` absent) and CUDA | builds at v5.4.1 against ZED SDK 5.4.x, `ldd -r` clean; wrapper log: grab 30 Hz, no "depth.depth_mode contains a not valid string", no positional tracking, `rgb/color/rect/image` + `rgb/color/rect/camera_info` advertised |
| zed_wrapper | depends on zed_components | `camera.launch.py` up (apt `ros-jazzy-zed-description`); `/camera/image_raw`, `/camera/camera_info` ~30 Hz; depth published; TF base_link → zed2_camera_link → left optical resolves (**v5 renamed the optical frames to `zed2_left_camera_frame_optical`**: plan C20) |
| zed_debug | new in v5.4.1, depends on zed_components | builds, `ldd -r` clean |
| zed_ros2 | metapackage over the above | builds |
| vesc_driver | `find_package(serial_driver)`; apt `ros-jazzy-serial-driver/io-context/asio-cmake-module` not installed on linus (needs sudo) | builds against apt serial_driver 1.2.0; on the car: opens the port, `/sensors/core` and `/sensors/imu/raw` |
| vesc | metapackage over vesc_driver | builds |

## Tests (`colcon test`, linus, ROS_DOMAIN_ID 93, localhost discovery)

| Package | Tests | Pass | Fail | Err | Skip | Failures |
|---|---:|---:|---:|---:|---:|---|
| corridor_perception | 56 | 37 | 0 | 0 | 19 | |
| f1tenth_behavior | 353 | 352 | 1 | 0 | 0 | `test_grace_then_target_lost` (known float flake, plan P16) |
| f1tenth_bringup | 40 | 37 | 3 | 0 | 0 | copyright, flake8, pep257 (lint, pre-existing on campaign-final) |
| f1tenth_control | 7 | 7 | 0 | 0 | 0 | |
| f1tenth_costmap | 198 | 198 | 0 | 0 | 0 | |
| f1tenth_description | 22 | 22 | 0 | 0 | 0 | |
| f1tenth_diagnostics | 153 | 149 | 3 | 0 | 1 | lint (pre-existing) |
| f1tenth_localization | 14 | 14 | 0 | 0 | 0 | |
| f1tenth_logger | 318 | 313 | 5 | 0 | 0 | 3 lint; 2 `test_robot_logger` asserts that the checkout directory is named `f1tenth_more` (the worktree is `f1tenth_more_jc`: environment, P16) |
| f1tenth_messages | 3 | 3 | 0 | 0 | 0 | |
| f1tenth_navigation | 2 | 2 | 0 | 0 | 0 | |
| f1tenth_params | 54 | 54 | 0 | 0 | 0 | |
| f1tenth_perception | 614 | 614 | 0 | 0 | 0 | |
| llm | 272 | 234 | 14 | 0 | 24 | 12 `_note_llm_sent` stub methods missing in campaign-final's tests (P16), 2 lint; 24 skips need llama-server |
| mpc_controller | 741 | 727 | 13 | 0 | 1 | 2 lint; 11 functional failures in campaign-final's own tests (e.g. `test_prediction_step` expects 0.2 s, `stack_params.yaml` ships `mpc_prediction_dt_s` 0.1) |
| vesc_tuning | 16 | 16 | 0 | 0 | 0 | |
| external: ackermann_mux / vesc_ackermann / zed_msgs | – | – | 3 / 2 / 1 | | | cpplint, uncrustify, flake8, copyright (upstream code) |
| external: joy_teleop | 1 | 0 | 0 | 1 | 0 | collection error: `test_msgs` not installed (apt `ros-jazzy-test-msgs`) |

Why none of the functional failures comes from the port: the port changed **no code** in mpc_controller, llm or f1tenth_behavior (`git diff 7b17e2f HEAD -- src/f1tenth_control/mpc_controller src/f1tenth_intelligence src/f1tenth_behavior` touches only `mpc_controller/package.xml`). The failing assertions are campaign-final test/config disagreements (plan P16). The Phase 1 package analysts ran campaign-final's code on this host before the port and got the same failures (plan §5 steps 11, 15).

Raw data: `test_linus.log`, `test_result_all.txt`, `test_counts.json`, `test_failures.txt`.

## ros2 pkg list / interfaces

- `ros2 pkg list` lists all 27 built packages.
- `ros2 interface show` succeeds for **all 54** interfaces of f1tenth_messages (26), vesc_msgs, teleop_tools_msgs and zed_msgs.

## Humble leftovers (`humble_leftovers.txt`, plan §2 checks C1–C34)

- No hits: removed rosbag2 APIs, `TopicMetadata` without `id`, hardcoded sqlite3 readers through rosbag2_py, `ROS_LOCALHOST_ONLY`, Humble apt names, removed numpy aliases, Python 3.12 removed modules, invalid escape sequences (all files compile with `-W error::SyntaxWarning`), packaging rules (test extra, `tests_require`, `--` in XML comments), lease-profile exports.
- Remaining hits, all deliberate and listed in plan §7.2:
  - raw `.db3`/sqlite3 readers (F1): `corridor_perception/replay.py`, `f1tenth_logger/mission_debug_video.py`, `tools/measure_stop_distance.py`, `tools/scene_labels_replay.py`, `tools/test_campaign_backfill.py`; they read the archived Humble bags but not Jazzy's mcap bags;
  - v4 ZED frame names (`zed2_left_camera_optical_frame`) in docstrings, tests that set their own frame ids, `tools/scene_labels_replay.py`, and the Gazebo `sensors.xacro` (no stack node looks the frame up by name);
  - Gazebo Fortress names in `f1tenth_description` xacros (branches the car never enables) and the ignored `f1tenth_sim` (F3);
  - `test_foxglove_whitelists.py:39` keeps a v4 topic name as a sample (the test passes with either).

## Adversarial review of the port (`port_review.json`)

A second workflow (3 review lenses: behaviour neutrality, missed Jazzy breakages, runtime readiness; then one skeptic per finding trying to refute it) read every hunk of `git diff 7b17e2f HEAD`, loaded all 48 installed launch files (46 parse; `camera.launch.py` needs the skipped zed_wrapper and loads with a stub of the real v5.4.1 launch; `vesc_tuning/speed_tuning.launch.py` references a package `f1tenth_stack` that no longer exists: pre-existing in the vesc fork), dry-ran supervisor_bringup and stack_bringup on Jazzy's LaunchService (36 processes, rc 0), and imported every console-script module.

**Result: no hunk changes node logic, a parameter, a default, a topic, QoS or timing.** 15 findings, 13 confirmed by the skeptics, 2 refuted. Acted on:

| Finding | Action |
|---|---|
| ZED: v5.4.1 enables the SDK image validity check, which humble-v4.2.5 forced off on the car (ZED SDK 4.2.5) | pinned `general.enable_image_validity_check: 0` (`d9c381e`) |
| `stack_bringup.launch.py` (fallback path): foxglove's `ROS_SUPER_CLIENT=TRUE` leaks into every later process (unscoped include) | foxglove include wrapped in a scoped `GroupAction` (`d9c381e`), environment only |
| Orin image: `mpc_corr` crashes in its constructor writing `src/f1tenth_control/corridors_jsons/*.csv` into the root-owned source tree | Dockerfile creates that dir and `mission_reports/` owned by the runtime user (`d9c381e`); not buildable here |

Recorded, not changed (pre-existing on campaign-final or a user decision):
- **slam_toolbox activation under the Discovery Server** (medium): the Jazzy LifecycleNode needs the configure/activate service calls to match; with the default 20 s lease a stale participant can make that fail silently (jazzy fix batch 5 saw 8/30 bringups). Gate: `ros2cli lifecycle get /slam_toolbox` = active after every bringup; proposal P3 (lease profile) decides it.
- **Recorded data changes format on Jazzy**: mission bags become mcap (the logger's default `mission_bag_storage_id` was already 'mcap'; the Humble Orin lacked the plugin and fell back to sqlite3) and the ZED v5 optical frame is `zed2_left_camera_frame_optical`. The F1 readers (§ Humble leftovers) still expect sqlite3 / the v4 name.
- `f1tenth_more/package.xml` still `exec_depend`s the ignored `f1tenth_sim` (rosdep needs `--skip-keys f1tenth_sim`); `f1tenth_bringup` declares `rosbridge_server`, which nothing starts; `mpc_controller`'s `model_log` console script points at a missing `main`; several runtime deps are undeclared (osqp, torch, ultralytics, cv_bridge in the logger, ...) — all pre-existing (plan F7).
- Hardware-only dependencies (vesc_driver, urg_node, zed_wrapper) do not resolve on linus — expected (skip list).
