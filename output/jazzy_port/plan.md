# jazzy-campaign port plan

Written 2026-10-16 on linus, as the planning step of the jazzy-campaign workflow. This step was analysis only: nothing was committed, built or pushed, and no tracked content was changed. Git was used read-only, and patch dry-runs ran on scratch copies. The one exception is a single `git status` in this worktree, which can at most refresh the index stat cache. This file is the only file written here.

**Inputs.** The 159 classified jazzy-only commits, the two digests of jazzy's reports (`~/dev_ws/f1tenth_more_jzref/output/*.md`, jazzy's `CLAUDE.md`), the per-package and per-submodule analyses, and `output/jazzy_port/{phase0,decisions,log,todo_for_andreas}.md`. Every pin, line number and count below was re-checked with read-only git (`log`, `show`, `diff`, `grep`, `ls-tree`, `cat-file`, `rev-list`, `merge-base`) unless marked as taken from an analysis.

**Goal.** Branch `jazzy-campaign` = campaign-final's **logic, parameters and defaults** + jazzy's **API and packaging migration**. A port changes APIs, not logic, parameters or defaults. Any logic change found on jazzy, even a good fix, is a **proposal** (§7.1). It is not a port step.

Short shas throughout (7 characters). "CF" means `origin/campaign-final` (7b17e2f). "jazzy" means `origin/jazzy` (3dbe37f).

---

## 0. Base, conventions, and how the input conflicts were resolved

### 0.1 Base

- All port work happens in the worktree `~/dev_ws/f1tenth_more_jc`, on branch `jazzy-campaign` (decision D1). That branch is at `5b621a9`, which is CF `7b17e2f` plus the Phase 0 docs commit (`output/jazzy_port/*` only). Its upstream is unset (D3). The worktree's own submodule store already contains every pin this plan needs, including zed_ros2_wrapper `86171bc`.
- **The port base is older than the car.** The car's newest bags were recorded from `e3793e47` (2026-10-15, dirty tree) and `736d46f` (2026-10-14). Neither is on origin (D2, `phase0.md`, `todo_for_andreas.md` §1). This plan is written against 7b17e2f, so it must be re-applied once the Orin's campaign-final is pushed (risk R1). Each step is a cherry-pick or a small, line-addressed patch so that it can be replayed.

### 0.2 Conventions for whoever executes the plan

- Use `git cherry-pick -x <sha>` for every cherry-pick, so the jazzy sha is recorded. One step is one commit unless the step says otherwise.
- **Never take a whole jazzy file** for a path that CF changed: jazzy's versions carry test-campaign or jazzy-feature logic. **Never `git cherry-pick -m`** any of jazzy's four merges (`ab60fe4`, `39b2b71`, `714c299`, `6af1ddf`). Against its first parent, each of them carries a whole side line.
- After every step:
  - the guard grep of step 2 must still print only its single baseline hit;
  - the packages the step touched must build with the linus skip list (§6).

### 0.3 Where the inputs disagreed, and what this plan does

| Question | What the inputs said | Decision here | Evidence |
|---|---|---|---|
| ackermann_mux pin | Report digest 1: move to `b3c0b08`, because `42b7cd7` "cannot be fetched". Commit and submodule analyses: keep `42b7cd7`. | **Keep `42b7cd7`**: skip `8d99f78` and put no `branch` line in `.gitmodules` | `git branch -a --contains 42b7cd70` in the jc module store lists `remotes/origin/humble-jetson`, so the pin is fetchable. The two commits differ only in `config/ackermann_mux_topics.yaml` (+7/-3), and the stack never loads that file (`f1tenth_control/launch/ackermann_mux.launch.py` loads f1tenth_bringup's `config/mux.yaml`). CF `5c25ade` restored this pin on purpose. |
| Jazzy `TopicMetadata(id=)` test fix | Commit list: `52682de`, skip `472f036`. Logger analysis: `472f036`. | **`52682de`; never both** | `52682de` is on the jazzy-p0 migration line, and its result equals the jazzy-p0 file. Both commits rewrite the same loop in `test_object_run_summary.py`, so after one is applied the other's patch fails (verified). jazzy-campaign targets Jazzy only, so `472f036`'s Humble fallback is not needed. |
| rosbag2 `Recorder` reuse (from `ce36ebb`) | Report digest 1: port the single reused Recorder. Commit verification and logger analysis: not needed. | **Not ported**; runtime check instead (R4) | The "context is already initialized" error happens only while the previous `Recorder` object is still alive. CF's `_stop_recording` cancels it, joins for up to 15 s and drops it before the next run (`mission_logger_node.py:904-918`). A host reproduction of that pattern recorded 3 bags in a row. |
| `fd83d6e` (inflate_polytope loop) | Report digest 1: a real Jazzy break. Costmap analysis: apply. Commit list and report digest 2: proposal. | **Proposal P1**, recommended for acceptance before any Thor `colcon test` | It changes loop logic: it forces `keep[i_pt] = False` where CF trusts a recomputed dot product. Its authors report identical output, and the code path is off by default (`use_convex_polytope` false, `costmap_boundary_node.py:393`). The hang appears only on aarch64/OpenBLAS: CF's unpatched test passes 33/33 in 0.29 s on linus (x86). |
| Super-client launch environments (`84a12f1`, `a936c75`, `5b320a5`, `d29b3e0`) | Report digest 1: proposals. Commit list: cherry-pick. Package analyses: apply if jazzy's environment is adopted. | **Cherry-pick** | The port adopts jazzy's environment: `scripts/env/jazzy.sh` and `docker/orin/entrypoint.sh`, neither of which exports `ROS_SUPER_CLIENT`. The four commits change only the process environment, with no node logic, parameter or default. They do nothing where every node is already a super client (the Humble Orin's `~/.bashrc`). |
| YOLO TensorRT guard (`1b50dce`, the f1tenth_perception part) | Perception analysis: cherry-pick. Commit list: proposal. | **Proposal P10** | It is node logic. CF's default `yolo_model` is `yolo26s-seg.pt`, not an engine, and a foreign engine already falls through to passthrough via the existing `except` (jazzy `output/phase5d_image.md`). |
| f1tenth_sim | Sim analysis: a strict gz-rename port ("A") or an ignore ("A0"). Description analysis: apply the gz renames. Commit list: only `0607db8`. | **A0: `COLCON_IGNORE` (`0607db8`)**; the gz renames are deferred (F3) | The Isaac sim replaces Gazebo (log.md: `isaac_ws` `ba9c29b`, Jazzy bridge). Option A has never run on any branch. The renames have no effect on the car (`use_sim` and `enable_sensors` are never set there). |
| rclpy `try_shutdown()` (llm analysis: "migration fix") | Report digest 1: no rclpy change needed. Report digest 2: backlog item M13. | **Proposal P19** | jazzy `output/phase4/probes/sigint_probe_results.txt` shows that Humble 3.3.21 and Jazzy behave the same, so this is not a distro change. |
| `scripts/env/jazzy.sh` | `docker/orin` needs it, but jazzy's final version exports the lease profile, which is proposal P3. | **Take it without the export** (single-step file, step 18) | `docker/orin/stage_sources.sh:27` runs `git archive "$SHA" src scripts/env`, and the image sources the script (`Dockerfile:212,242`, `entrypoint.sh:8`). An exported profile path that does not exist makes every participant log an XML error (bringup analysis). |
| `CLAUDE.md` sections | Root analysis: merge jazzy's sections except the DDS lease section. | Take **Rebuild policy** (`b0740ea`) and **Orin Docker** (`f574e43`). Leave out DDS/lease (P3) and the Thor section (`8571558`). | The Thor section's premise is that a skipped f1tenth_sim breaks f1tenth_more. With `COLCON_IGNORE` the package is never discovered: jazzy's own Thor Phase 0 build under `0607db8` finished f1tenth_more (`output/thor_build.log`: "33 packages finished"). |

---

## 1. History and commit counts

- **Merge base** `573f313` (`git merge-base origin/campaign-final origin/jazzy`).
- **CF-only commits: 27** (`git rev-list --count 573f313..origin/campaign-final`). They are listed in Appendix B.
- **Jazzy-only commits: 159** (`573f313..origin/jazzy`), including 4 merges. The 159-commit input list matches `git rev-list` exactly (no commit missing, none extra).
- How jazzy's history is laid out (counted with `git rev-list`):

| Line | Commits | Range |
|---|---|---|
| jazzy first-parent: migration p0 to p5, fix batches 1 to 5, phase5d Docker, camera pan, sim follow-ups, test parity | 104 | from `bb54c8a` (parent `e47e646`) to `3dbe37f` |
| jazzy-sim side line 1 (Gazebo Harmonic port) | 23 | `63f46d4` to `feabbd2`, merged by `ab60fe4` |
| jazzy-sim side line 2 | 5 | `ab60fe4`, `5b9ffdc`, `f86f4a7`, `6a41f2b`, `70dc09f`, merged by `39b2b71` |
| test-campaign line (the M02 campaign fixes; campaign-final deliberately did **not** build on it) | 27 | `1d00853` to `6cae729`, including the merge `714c299`; merged into jazzy by `6af1ddf` |

- jazzy forked at `e47e646`, the parent of `573f313`. `573f313` adds data only (8 files, 0 text lines) and reaches jazzy only through `6af1ddf`.
- Tags: `jazzy-p0` = `3781f2e`, `jazzy-p1` = `9beea9b`, `jazzy-p3` = `fc1ae6f`, `jazzy-p4` = `fc990d3`, `jazzy-p5` = `4510559`. There is no `jazzy-p2` tag.
- **Actions over the 159 commits:** cherry-pick 19, partial 8, proposal 19, skip 113.

### 1.1 Jazzy-only commits by category

The primary category is the first category in each commit's classification. Many commits carry several categories; all of them are in Appendix A.

| Primary category | Commits | Action breakdown |
|---|---|---|
| SUBMODULE | 5 | **cherry-pick 2**: `3d545db`, `9a66870`<br>**partial 1**: `6118239`<br>**skip 2**: `bb54c8a`, `8d99f78` |
| DOCS | 40 | **cherry-pick 1**: `b0740ea`<br>**partial 1**: `cf53ed0`<br>**skip 38**: `2a57817`, `3781f2e`, `119cb4c`, `fc1ae6f`, `9f3d558`, `fc990d3`, `e4d6110`, `7d3303c`, `3978b81`, `49f7a2a`, `715907c`, `e248d33`, `34a43a6`, `ee9b0a0`, `4510559`, `898d7a6`, `8c0891d`, `39f7856`, `ca89326`, `2d10bc9`, `d94083f`, `503df8f`, `caefd05`, `4827536`, `feabbd2`, `6a41f2b`, `70dc09f`, `8571558`, `9765c4d`, `16b7b95`, `d2ef4b4`, `626c70b`, `f25e1e3`, `da79f37`, `f639ea6`, `27f84d3`, `5f19359`, `3dbe37f` |
| SIM_GZ | 31 | **cherry-pick 1**: `0607db8`<br>**skip 30**: `63f46d4`, `4f0a3d3`, `cc3addd`, `90cbfb1`, `ace37af`, `66d9ff3`, `bd49bfc`, `6739899`, `8b8a83a`, `0ef3500`, `06a0ca1`, `01abfe6`, `2e672a3`, `6529fcc`, `551fa1a`, `9ed3785`, `66ef8e8`, `f86f4a7`, `0d73ab9`, `04780ee`, `e67910c`, `0d233e2`, `083c8e8`, `a4488e5`, `0f93aae`, `2713326`, `69281f4`, `ff3df03`, `5bb7fa6`, `00b3b1c` |
| MIGRATION | 8 | **cherry-pick 7**: `a086f0e`, `130063c`, `2ea8541`, `52682de`, `4edcbbf`, `de1f8d8`, `91d361e`<br>**skip 1**: `472f036` |
| FEATURE | 22 | **proposal 14**: `fd83d6e`, `931a0eb`, `0397d6b`, `7155f79`, `af41013`, `b31b76c`, `7b301b2`, `0f151aa`, `96a5b12`, `88fbd42`, `725f256`, `f9e0fea`, `c85d9c9`, `279e3f4`<br>**skip 8**: `a637807`, `cb5dd7c`, `ff3ab10`, `a8e8d79`, `cb5735f`, `30fa1c7`, `740ccae`, `9e21beb` |
| HARNESS | 17 | **cherry-pick 1**: `5c82958`<br>**skip 16**: `9beea9b`, `5fec439`, `bd3bd3b`, `691e30d`, `19c8703`, `48108cd`, `210f380`, `684ef1f`, `3968af0`, `c437903`, `8cba2b3`, `42b416d`, `bb76c09`, `009813c`, `72c028b`, `ba58788` |
| JAZZY_INFRA | 12 | **cherry-pick 7**: `84a12f1`, `a936c75`, `5b320a5`, `d29b3e0`, `5b9ffdc`, `f7f06be`, `8ecc13b`<br>**partial 5**: `d9a7f00`, `dac4bef`, `f574e43`, `1b50dce`, `af1f167` |
| MERGE | 4 | **partial 1**: `39b2b71`<br>**skip 3**: `ab60fe4`, `714c299`, `6af1ddf` |
| TEST_CAMPAIGN | 20 | **proposal 5**: `8eb4c29`, `58f9864`, `20544fa`, `babc885`, `49de45d`<br>**skip 15**: `1d00853`, `2919e51`, `e1f8d6a`, `c0aa636`, `dab5787`, `6b4953f`, `75ccbe6`, `0b9e8ee`, `04facd9`, `959b40d`, `ce36ebb`, `426eabf`, `6ebbc8a`, `48829de`, `6cae729` |
| **Total** | **159** | |

The full per-commit table (sha, subject, categories, action, files to apply) is in **Appendix A**. Campaign-final's 27 commits, and what each means for Jazzy, are in **Appendix B**.

---

## 2. Migration patterns: checklist with detection commands

Run every command from `~/dev_ws/f1tenth_more_jc`. "CF" is the state of campaign-final 7b17e2f today, checked with `git grep <rev>`. "After" is the state expected on jazzy-campaign once §5 has been applied. Each item names its source: a jazzy commit, or a report digest ("digest 1" and "digest 2" are the two digests of jazzy's reports). In this section, R-numbers are not used; R1 to R19 are the risks in §7.3.

### rosbag2_py / logger

- [ ] **C1. `RecordOptions.all` was removed in Jazzy rosbag2_py 0.26.** Setting it raises AttributeError. The logger catches the error, logs "FAILED to start recording", and the mission runs unrecorded. Source: `de1f8d8`, digest 1.
  - Detect: `git grep -n -E 'record_options\.(all|all_topics)\b' -- src`
  - CF: `mission_logger_node.py:823` sets `record_options.all = False` with no guard.
  - After: a `hasattr(record_options, 'all_topics')` guard at about line 820 (step 10).
- [ ] **C2. `TopicMetadata(...)` requires `id=` on Jazzy.** Without it the call raises TypeError. Source: `52682de`.
  - Detect: `git grep -n -A2 'TopicMetadata(' -- src tools scripts ':!src/f1tenth_external'`
  - CF: `test_object_run_summary.py:63` has no `id` (6 test-setup errors on Jazzy).
  - After: `id=topic_id` (step 10).
- [ ] **C3. Bag readers must take the storage plugin from the bag.** Jazzy registers mcap, so the logger's default `storage_id` `'mcap'` (`mission_logger_node.py:660`, launch argument `mission_bag_storage_id`) now really records mcap. On the Humble Orin it fell back to sqlite3, because the plugin was not installed (`mission_logger_node.py:62-71`). Source: `91d361e`.
  - Detect rosbag2_py readers: `git grep -n -E "storage_id=['\"]sqlite3['\"]" -- src tools scripts ':!src/f1tenth_external' ':!*/test/*'`
    - CF: `mission_extract.py:175`, `tools/wall_track_replay.py:48`, `tools/wall_distance_replay.py:102`.
    - After: no hits (steps 10 and 20).
  - Detect raw-sqlite readers (deferred, F1): `git grep -n -E "glob\(.*\.db3|bag_0\.db3'|sqlite3\.connect\(" -- src tools ':!src/f1tenth_external' ':!*/test/*' ':!src/f1tenth_logger/f1tenth_logger/runs_db.py'`
    - CF and after: `corridor_perception/replay.py:119,128`, `f1tenth_logger/mission_debug_video.py:93,108`, `tools/measure_stop_distance.py:88,91`, `tools/scene_labels_replay.py:84,85`, `tools/test_campaign_backfill.py:170,173,237`.
- [ ] **C4. Only one live `rosbag2_py.Recorder()` per process.** While a first Recorder is alive, a second raises `RuntimeError: context is already initialized`. Source: digest 1; also `ce36ebb` (test-campaign, not ported).
  - Detect: `git grep -n 'rosbag2_py.Recorder()' -- src`
  - CF and after: `mission_logger_node.py:839` creates one Recorder per run. `_stop_recording` releases the previous one (`:904-918`: cancel, then `join(timeout=15.0)`).
  - Runtime: two missions in one logger process must both produce a non-empty bag (R4).
- [ ] **C5. Deprecation warnings only.** On rosbag2_py 0.26.10, `Recorder()`, `record(storage, record)` and `cancel()` print warnings but work. Leave them (F5).
- [ ] **C6. Humble apt names in user-facing hints.** Source: `130063c`.
  - Detect: `git grep -n 'ros-humble-rosbag2-storage-mcap' -- src`
  - CF: `mission_logger_node.py:68` and `:745`. After: no hits (step 10).
  - The other `ros-humble-` and `/opt/ros/humble` hits are comments (`git grep -n -E 'ros-humble-|/opt/ros/humble' -- src scripts tools ':!src/f1tenth_external'`). Leave them.

### Launch / lifecycle

- [ ] **C7. slam_toolbox 2.8.x `async_slam_toolbox_node` is a LifecycleNode.** Launched with a plain `Node()`, it stays "unconfigured" without any error and publishes neither `/slam/pose` nor `/slam/map`. `enable_slam` defaults to `true` (`stack_params.yaml:1991`). Source: `4edcbbf`.
  - Detect: `git grep -c TRANSITION_CONFIGURE -- src/f1tenth_navigation/launch/slam.launch.py`
  - CF: no hit (a plain `Node(` at `slam.launch.py:103`). After: 1, and the file equals jazzy's (step 12).
  - Runtime: `ros2cli lifecycle get /slam_toolbox` prints `active`.
- [ ] **C8. Launch parameters never use the strings `on`, `off`, `yes` or `no`.** YAML turns them into booleans. Source: jazzy `CLAUDE.md`.
  - Detect: `git grep -n -i -E "default_value=['\"](on|off|yes|no)['\"]" -- 'src/*.py'`
  - CF and after: no hits.

### Environment / Discovery Server

- [ ] **C9. `ROS_LOCALHOST_ONLY` is deprecated: use `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.** Source: `a086f0e`.
  - Detect: `git grep -n ROS_LOCALHOST_ONLY -- src scripts tools ':!src/f1tenth_external'`
  - CF: `scripts/replay_bag_through_mpc.sh:35`, `src/f1tenth_logger/test/test_campaign/conftest.py:20`, and `tools/scene_labels_replay.py:50` (CF-only, so no jazzy commit covers it).
  - After: no hits (steps 10 and 20).
- [ ] **C10. A process that reads the whole graph needs `ROS_SUPER_CLIENT=TRUE`, scoped to that process and never exported globally.** Source: `84a12f1`, `a936c75`, `5b320a5`, `d29b3e0`.
  - Detect: `git grep -n -E "SetEnvironmentVariable\('ROS_SUPER_CLIENT'|'ROS_SUPER_CLIENT': 'TRUE'|environ\['ROS_SUPER_CLIENT'\] = 'TRUE'|^export ROS_SUPER_CLIENT" -- src scripts ':!src/f1tenth_external'`
  - CF: `foxglove_bridge.launch.py:111` and `viz_relays.launch.py:112`.
  - After: those two, plus `mission_logger.launch.py`, `localization.launch.py` (for ekf_cost_observer only), `steering_offset_calibration.launch.py` and `scripts/stackctl.py`. There must be no `export ROS_SUPER_CLIENT` line anywhere.
- [ ] **C11. Every graph-reading call runs in a super-client process.**
  - Detect: `git grep -n -E 'get_topic_names_and_types|get_service_names_and_types|count_publishers\(|rosbag2_py\.Recorder\(' -- src scripts tools ':!src/f1tenth_external' ':!*/test/*'`
  - CF and after: `scripts/stackctl.py:72`, `ekf_cost_observer_node.py:221`, `steering_offset_calibration_node.py:682`, `mission_logger_node.py:839`. All four are covered once C10 holds.
  - `get_node_names()` is correct for plain clients (jazzy fix batch 3 audit), so preflight, diagnostics_server and the calibration's `:606` need nothing.
- [ ] **C12. Hand-started tools need the Discovery Server environment.** Source: `d9a7f00`, `5b9ffdc`, `dac4bef`, `39b2b71`.
  - Detect: `test -f scripts/env/jazzy.sh && grep -c FASTRTPS_DEFAULT_PROFILES_FILE scripts/env/jazzy.sh`
  - After: the file exists, and the count is 0 (step 18; the lease profile is proposal P3).
- [ ] **C13. Fast DDS lease profile: not ported (P3).**
  - Detect: `git grep -n FASTRTPS_DEFAULT_PROFILES_FILE -- src scripts ':!src/f1tenth_external'`
  - CF and after: no hits.

### C++ submodules / apt

- [ ] **C14. transport_drivers comes from apt.** Source: `3d545db`.
  - Detect: `git ls-tree HEAD src/f1tenth_external/transport_drivers` and `git config -f .gitmodules --get-regexp '^submodule\.transport_drivers'`
  - After: both print nothing (step 6).
  - Every Jazzy host needs: `ros-jazzy-serial-driver ros-jazzy-io-context ros-jazzy-asio-cmake-module ros-jazzy-udp-msgs`.
- [ ] **C15. Every submodule pin builds on Jazzy.**
  - Detect: `git ls-tree -r HEAD | awk '$2=="commit"'`
  - After: ackermann_mux `42b7cd7`, teleop_tools `4337558`, zed-ros2-interfaces `e970087`, zed_ros2_wrapper `86171bc`, vesc `2848a9f`.
  - CF's `b5844a8` (humble-v4.2.5) cannot build on Jazzy: `zed_camera_component.cpp:36-43` handles only FOUND_HUMBLE, FOUND_IRON and FOUND_FOXY, and anything else hits `#error Unsupported ROS2 distro`.
- [ ] **C16. ABI drift inside Jazzy.** `diagnostic_updater` 4.2.6 to 4.2.7 broke ackermann_mux, zed_components and zed_debug at runtime. Source: `b0740ea`.
  - Detect: after every `ros-jazzy-*` upgrade, run the `ldd -r` scan from `CLAUDE.md` "Rebuild policy" (step 22). It must print nothing.

### ZED wrapper v5.4.1 (forced by the bump in step 14; jazzy itself never adapted the code that uses the camera)

- [ ] **C17. RGB topics were renamed** (wrapper CHANGELOG v5.1.0: `~/rgb/image_rect_color` becomes `~/rgb/color/rect/image`). The camera_info topic comes from `image_transport::getCameraInfoTopic`, so it is `~/rgb/color/rect/camera_info` (`zed_camera_component_video_depth.cpp:361,376`).
  - Detect: `git grep -n -E 'rgb/image_rect_color|zed_node/rgb/camera_info' -- src ':!src/f1tenth_external' ':!src/f1tenth_sim'`
  - CF hits:
    - `camera.launch.py:99` and `:101` (the live remaps);
    - `zed2_perception.yaml:21` (a comment);
    - `test_foxglove_whitelists.py:39` (a sample in the "hidden" list; the test passes with either name);
    - `sensors.xacro:137` (Gazebo only).
  - After: only the test sample and `sensors.xacro:137`.
- [ ] **C18. The zed2i defaults changed**: `grab_frame_rate` 30 to 60, `min_depth` 0.3 to 0.01, `max_depth` 10.0 to 15.0 (`zed_wrapper/config/zed2i.yaml` at `b5844a8` and at `86171bc`). CF does not override them.
  - Detect: `grep -n -E 'grab_frame_rate|min_depth|max_depth' src/f1tenth_perception/config/zed2_perception.yaml`
  - CF: no hits. After: 30, 0.3 and 10.0 (step 14).
  - The other changed defaults have no effect here, because CF overrides them or disables the module: `pub_frame_rate` and `pub_resolution` are pinned by CF; `depth_stabilization` is pinned to 0; `point_cloud_freq` 30 to 10 (no subscriber); `pos_tracking_mode` GEN_1 to AUTO (tracking disabled); object detection and body tracking are disabled; `sensors_pub_rate` 200 to 100 (IMU not advertised).
- [ ] **C19. The v5 `depth_mode` list is NONE, NEURAL_LIGHT, NEURAL, NEURAL_PLUS.** CF sets `'PERFORMANCE'`.
  - Runtime check: the wrapper log must NOT contain "The parameter 'depth.depth_mode' contains a not valid string". That message means a silent switch to NEURAL (`video_depth.cpp:611-622`).
  - The SDK 5.4 warning "PERFORMANCE is deprecated" is expected. The mode still worked on the Thor and in the Orin spike (jazzy `output/phase5d_image.md:254`, `phase5d_spike.md:199`).
- [ ] **C20. The optical frames were renamed**: `zed2_left_camera_optical_frame` became `zed2_left_camera_frame_optical` (`zed_camera_component_main.cpp:2251`).
  - Detect: `git grep -n 'left_camera_optical_frame' -- src tools ':!src/f1tenth_external' ':!src/f1tenth_sim'`
  - CF and after: docstrings, test fixtures that set their own `frame_id`, `verify_seg_live_deploy.py` (sets its own), `sensors.xacro` (Gazebo), and `tools/scene_labels_replay.py:333,576`. That tool builds frames from the bag's `/tf_static`, so it breaks on bags recorded with v5 (F1). No stack node looks up this frame by name.
- [ ] **C21. The v5 advertise flags take effect.** CF's `zed2_perception.yaml` sets `publish_status`, `publish_stereo`, `publish_imu` and `publish_imu_raw` to false. These keys did not exist in humble-v4.2.5, so they were inert on the Humble car. On v5 those topics are not advertised.
  - Consumer check: `git grep -n -E 'zed_node/(imu|status|stereo)' -- src ':!src/f1tenth_external'` finds only a yaml comment.

### numpy / Python 3.12 / packaging

- [ ] **C22. inflate_polytope can loop forever with numpy 1.26 on aarch64 (P1).**
  - Detect: `grep -c 'keep\[i_pt\] = False' src/f1tenth_costmap/f1tenth_costmap/safe_corridor.py`
  - CF and after: 1 (`:422`). With P1: 3.
  - On the Thor: `timeout 600 python3 -m pytest src/f1tenth_costmap/test/test_safe_corridor.py`.
- [ ] **C23. Removed numpy aliases.**
  - Detect: `git grep -n -E 'np\.(float|int|bool|object)\b[^_0-9(]|np\.asscalar|np\.float_\b|np\.NaN\b|np\.alltrue|np\.product\b' -- src tools scripts ':!src/f1tenth_external'`
  - CF and after: no hits.
- [ ] **C24. Python 3.12 removals and invalid escapes.**
  - Detect removed modules and aliases: `git grep -n -E '^\s*(import|from) (distutils|imp|asynchat|asyncore)\b|pkg_resources|SafeConfigParser|assertEquals|assertRegexpMatches' -- src tools scripts ':!src/f1tenth_external'`. CF and after: no hits.
  - Detect invalid escape sequences: `find src tools scripts -name '*.py' -not -path 'src/f1tenth_external/*' -print0 | xargs -0 python3 -W error::SyntaxWarning -c 'import sys; [compile(open(f).read(), f, "exec") for f in sys.argv[1:]]'`. This must not raise; digest 1 found all 344 files clean.
- [ ] **C25. Packaging rules from `CLAUDE.md`.**
  - Every setup.py declares `extras_require={'test': ['pytest']}`: `git grep -L -E "'test'" -- 'src/**/setup.py' ':!src/f1tenth_external' ':!src/f1tenth_hardware/vesc'` must print nothing.
  - No `tests_require`: `git grep -n -E '^\s*tests_require' -- 'src/**/setup.py' ':!src/f1tenth_external'` must print nothing.
  - No `--` inside a single-line XML comment: `git ls-files '*package.xml' | grep -v -E 'f1tenth_external|/vesc/' | xargs grep -n -P '<!--(?:(?!-->).)*--(?!>)'` must print nothing. Check any multi-line comment you add by hand.
  - CF passes all three today.
- [ ] **C26. A test that reads another package's share directory needs a `<test_depend>`.** Source: `2ea8541`.
  - Detect: `git grep -n f1tenth_hardware -- src/f1tenth_control/mpc_controller/package.xml`
  - CF: no hit (`test_steering_limits.py:56` reads that share directory). After: the test_depend is there (step 11).
- [ ] **C27. rosdep keys.**
  - Detect: `rosdep check --from-paths src --ignore-src --rosdistro jazzy --skip-keys "ament_python python3-pyarrow f1tenth_sim"`
  - After this, only installable apt packages may remain.
  - `ament_python` is a bogus key in three package.xml files. `python3-pyarrow` has no Noble key; install it with pip. `f1tenth_sim` is a key the `f1tenth_more` metapackage still declares, while the package itself is ignored (catkin_pkg honours `COLCON_IGNORE`).
- [ ] **C28. Gazebo Fortress names: deferred (F3).**
  - Detect: `git grep -l -E 'ros_ign|ign_ros2_control|IgnitionSystem|ignition_frame_id|IGN_GAZEBO' -- src ':!src/f1tenth_external'`
  - After: `src/f1tenth_sim/*` (ignored), f1tenth_description's `urdf/*.xacro` (comments, plus the `use_sim` and `enable_sensors` branches the car never sets), and the `use_sim` text in `f1tenth_params`.
- [ ] **C29. `ros2 bag record` positional topics are deprecated on Jazzy** (`ros2bag/verb/record.py:68-71`: "(deprecated)"; use `--topics`).
  - Detect: `git grep -n 'ros2 bag record' -- scripts tools`
  - After: `--topics` is used (step 20).
- [ ] **C30. Old-style `static_transform_publisher` arguments: a deprecation warning only.**
  - Detect: `git grep -n "executable='static_transform_publisher'" -- 'src/*.py'` finds 4 call sites. Leave them (F6): `test_sensor_extrinsics.py` parses the positional form.
- [ ] **C31. `rclpy.shutdown()` after SIGINT raises RCLError.** This is the same on Humble and Jazzy (jazzy phase 4 probe).
  - Detect: `git grep -l 'rclpy.shutdown()' -- 'src/*.py' 'scripts/*.py' 'tools/*.py' ':!src/f1tenth_external'` lists 64 files. Leave them (P19).
- [ ] **C32. Nav2 Jazzy parameter names: dormant, because `enable_nav2` is false (F2).**
  - Detect: `grep -n -E '^\s*(progress_checker_plugin|plugin_lib_names|costmap_topic|footprint_topic):' src/f1tenth_navigation/config/nav2_params.yaml`
  - Hits: lines 23, 92, 93 and 121 (`global_frame: odom` of behavior_server is at 102). Leave them.
- [ ] **C33. Pip-only dependencies.** mpc_controller uses the osqp 1.x API (`mpc_solver.py:139-160`), and f1tenth_logger uses pyarrow.
  - Detect: `python3 -c 'import osqp, pyarrow, numpy; print(osqp.__version__, pyarrow.__version__, numpy.__version__)'`
  - Expected: `1.1.3 25.0.1 1.26.4`. pyarrow 26 requires numpy 2 or newer; keep numpy below 2 because cv_bridge needs it.
- [ ] **C34. f1tenth_sim is ignored, and the metapackage still builds.**
  - Detect: `test -f src/f1tenth_sim/COLCON_IGNORE && colcon list --names-only | grep -c -x f1tenth_sim`
  - Expected: 0. `colcon list` finds 33 packages (as in jazzy's Thor Phase 0 build).

---

## 3. Per-package table

★ = new on campaign-final and never ported to jazzy. The port carries these features from CF, so jazzy's Jazzy validation never covered them; see §3.1. Delta sizes are `git diff --shortstat 573f313 origin/campaign-final -- <path>`. Test numbers come from the package analyses, which ran CF code on this Jazzy host.

| Package | Jazzy status (jazzy delta) | Campaign-final delta | Port action | Risk |
|---|---|---|---|---|
| f1tenth_messages (ament_cmake) | no change needed (0 commits) | 1 commit `8b352d2`: `WallLineFit.REASON_SHORT_SPAN = 7` ★ | Take CF (step 3). Build it first and rebuild every consumer; linus's installed copy predates the constant. | low |
| f1tenth_params | no change needed. jazzy's 22 commits are features: test-campaign keys, `cb5735f` funnel, `camera_pan_*`, `fastdds_profile`, `health_watchdog`; `90cbfb1` is text only. | 12 commits, 6 files, +569/-67:<br>• 48 keys jazzy lacks: scene labels ★, debug image ★, `mpc_prediction_dt_s` 0.1 ★, `corr_half_width_start_m` 0.64 ★, `front_beside_body_margin_m` ★<br>• trapezoid walls in `corridor_geometry.py` ★<br>• `get_wall_distance_topic()` ★ | Take CF (step 4). Never merge jazzy's `stack_params.yaml` or `corridor_geometry.py`. | low (merge risk only) |
| f1tenth_description | ported, for Gazebo Harmonic only (8 commits) | 4 commits, 9 files, +339/-5: `base_front.xacro` at 0.405 ★, wheelbase 0.305 ★, measured laser x 0.305 ★, `vehicle_frames.front_side_limit_m` and `beside_body_behind_bumper` ★ | Take CF (step 8). The gz renames are deferred (F3). Never take jazzy's `base_front.xacro`, `vehicle_frames.py` or `sensors.xacro`. | medium: a whole-file take reverts the measured frames, and CF consumers need the helpers |
| corridor_perception | no change needed (jazzy `1d00853` is test-campaign logic) | none | Take CF (step 9). `replay.py` reads sqlite only (F1). | low |
| f1tenth_costmap | partial (`fd83d6e`) | 2 commits, 5 files, +157/-10: clearance measured from the bumper, and cells beside the car excluded ★ | Take CF (step 9). `fd83d6e` is P1. Do not take `c85d9c9` (P17), `a637807` or `30fa1c7`. | medium: test hang on aarch64 without P1 |
| f1tenth_logger | partial (6 migration commits; the rest is test-campaign or fix-batch logic) | 6 commits, 18 files, +706/-45: recording from LOADED ★, `mission_debug_video` ★ (reads sqlite only), 9 new default topics ★, `/viz` filter | Step 10: `a086f0e`, `130063c`, `52682de`, `de1f8d8`, `91d361e`, `84a12f1`. `mission_debug_video` goes to F1. | medium: without `de1f8d8` nothing is recorded; R4 |
| mpc_controller | no change needed except the test_depend | 8 commits, 16 files, +812/-145: trapezoid corridor ★, bumper distances ★, prediction step / `rate_weight_factor` ★, wall-truth subscription ★ | Step 11: `2ea8541`. pip `osqp==1.1.3`. P11, P12, P13 and P14 are proposals. | medium: merge risk (jazzy's MPC_corr.py carries BLOCKED, keep-outs and other key names); osqp |
| f1tenth_control | no change needed | 1 commit `30b0580`: `mpc_prediction_dt_s` launch argument ★ | Take CF (step 15); it must pair with CF's f1tenth_params, or the launch raises KeyError. | low |
| f1tenth_navigation | partial (`4edcbbf`) | none | Step 12: `4edcbbf`, plus `exec_depend`s on `slam_toolbox` and `lifecycle_msgs`. Nav2 keys are F2. | medium: without it slam_toolbox stays unconfigured, silently |
| f1tenth_localization | no change needed (`a936c75` is environment only) | none | Step 13: `a936c75`. | low |
| f1tenth_diagnostics | no change needed (`5b320a5` is environment only) | 1 commit, a comment | Step 13: `5b320a5`. P2 and P4 are proposals. apt `python3-ruamel.yaml`. | low |
| f1tenth_perception | partial: jazzy never adapted it to wrapper v5 | 9 commits, 31 files, +3640/-64: scene labels ★, debug image ★, floor RANSAC ★, virtual wall / wall planes ★, ZED 2i model ★, measured ZED x 0.2399 ★ | Step 14: the v5 remaps and the zed2i default pins, in the same commit as the bump. P10 is a proposal. | medium: an unfixed remap leaves `/camera/image_raw` silent; the camera is untested on v5 (R2) |
| f1tenth_behavior | no change needed | 6 commits, 19 files, +410/-76: 15 s countdown ★, bumper reference ★, consumer of `get_wall_distance_topic()` ★, `drive_test_straight_2m.json` ★ | Take CF (step 15). P6, P9 and P15 are proposals. | low: merge risk only |
| llm | no change needed | 5 commits, 17 files, +135/-56: wall-guard default 2.0 / 3.0 ★, speeds 0.5 ★, go-to gap from the bumper ★ | Take CF (step 15). P16 covers the test stubs; P19 covers try_shutdown. | low |
| f1tenth_hardware | no change needed (identical on both branches) | 1 commit `9b8f1b8`: servo gains (identical on jazzy) | Take CF (step 15). | low (hardware only: vesc_driver needs apt serial_driver) |
| f1tenth_bringup | no change needed | 3 commits, 14 files, +1196/-25: reset_manager, viz relays, foxglove allowlist. These are byte-identical on jazzy via `1d00853`, except CF's `86f249a` test fix. | Take CF (step 17). P3, P5 and P8 are proposals. | medium: Discovery Server isolation without P3 (R3); jazzy's f1tenth_bringup diverges from CF by about 2,400 lines (supervisor, watchdog, sim mode), so a merge would bring them back |
| f1tenth_sim | ported on jazzy (Harmonic rewrite, 21 commits) | 1 commit `d7b2770`: wheelbase 0.305 | Step 16: `COLCON_IGNORE` (`0607db8`). F3 if a Gazebo sim is ever wanted. | low for the car (never run there). CF's version cannot even generate its launch on Jazzy. |
| f1tenth_more | no change needed | none | Take CF (step 17). rosdep needs `--skip-keys f1tenth_sim`. | low |
| f1tenth_camera_pan | jazzy only | n/a | Do not port (P18). | low |
| scripts/ | partial | 1 commit (`watch_objects.py` from the bumper; identical on jazzy) | Steps 10 (`a086f0e`), 18 (`jazzy.sh`), 19 (`d29b3e0`), 20 (`--topics`). | low |
| tools/ | not started on jazzy | 2 commits: `scene_labels_replay.py` ★ (`3ffe245`) and the bumper reference | Step 20 (environment and storage); F1 for the raw-sqlite readers. | medium: offline tools cannot read Jazzy bags |
| docker/ | partial (`docker/orin` is new on jazzy; the Humble Dockerfiles were never migrated) | none | Step 21 (`docker/orin` without `spike/`). F4 for the Humble images. | medium: WIP, and it builds on L4T r36.5 only (R13) |
| CLAUDE.md, README.md, .gitignore, .gitmodules | partial | none | Steps 5, 6, 22, 24, 25. | medium (`.gitmodules` decides how serial_driver arrives) |
| .devcontainer/, src/scripts/, .github/ | not started (Humble-only), or n/a | none | Leave as they are (F4). | low |

### 3.1 Features new on campaign-final that jazzy never had

The port carries them as they are. Notes on how each meets Jazzy:

| Feature (CF commits) | Package | What it means on Jazzy |
|---|---|---|
| Recording from mission LOADED, with the countdown as pre-roll (`407ff6d`) | f1tenth_logger, f1tenth_behavior | It creates one `Recorder` per run. Prove two missions in one process on the Thor (R4). The logger's 315 unit tests stub the recorder, so they cannot catch this. |
| `mission_debug_video` host MP4 renderer (`0334b8c`) | f1tenth_logger | It reads `.db3` only and fails on Jazzy's mcap bags with "no .db3 found" (F1). |
| 5 Hz debug image, wall-estimator debug topic (`0334b8c`) | f1tenth_perception | Pure Python. Its input `/camera/image_raw` depends on the step 14 remap. |
| Scene labelling, virtual wall, wall planes, floor RANSAC, beside-the-car exclusion (`5c30b4d`, `8b352d2`, `c830202`, `7d033af`, `ca8e15b`, `d8609b5`) | f1tenth_perception, f1tenth_costmap, mpc_controller, f1tenth_behavior | Pure Python: perception 613/614 and costmap 198/198 on this host. The one perception failure is linus's stale f1tenth_messages install. `enable_scene_labels` defaults to false here, but the car runs true (R1). |
| Bumper frame and measured vehicle frames, wheelbase 0.305 (`10d3eb3`, `bdc7c93`, `d7b2770`) | f1tenth_description and its consumers | URDF and Python only; description tests 22/22. |
| Trapezoid corridor, prediction step, calibration (`a6dd23e`, `30b0580`, `9b8f1b8`) | mpc_controller, f1tenth_params, f1tenth_control | 734 tests pass with osqp 1.1.3. 11 failures exist on CF on any distro (P16). |
| LLM wall-guard default (`32bf769`) | llm | Pure Python. 12 tests fail on any distro because of missing stub methods (P16). |
| ZED wrapper launched as a ZED 2i (`1ff0644`) | f1tenth_perception | Coupled to the v5.4.1 bump: `zed2i.yaml`'s defaults changed, so step 14 pins them. |
| `tools/scene_labels_replay.py` (`3ffe245`) | tools | `ROS_LOCALHOST_ONLY` is fixed in step 20. Reading sqlite only, and v4 frame names, are F1. |
| `drive_test_straight_2m.json` (`1bb5ea2`) | f1tenth_behavior | Data only. |

---

## 4. Submodules

| Submodule (path) | Repo | CF pin | jazzy pin | Has a `jazzy` branch? | Builds on Jazzy? | Action on jazzy-campaign |
|---|---|---|---|---|---|---|
| vesc (`src/f1tenth_hardware/vesc`) | fork `sjckness/vesc` | `2848a9f` | `2848a9f` | yes: `origin/jazzy` = `2848a9f` (also `origin/scene-graph`) | yes. vesc_driver needs apt serial_driver, io_context and asio_cmake_module (1.2.0). vesc_msgs, vesc_ackermann and vesc_tuning need nothing extra. | Keep the pin. `.gitmodules` gets `branch = jazzy` (step 5). |
| ackermann_mux (`src/f1tenth_external/ackermann_mux`) | fork `sjckness/ackermann_mux` | `42b7cd7`, on `origin/humble-jetson` only | `b3c0b08` (= `origin/jazzy` = `origin/foxy-devel`) | yes, but `origin/jazzy` is the **parent** of the pin | yes: the C++ is identical to `b3c0b08`. Rebuild after any diagnostic_updater upgrade. | **Keep `42b7cd7`**, with no `branch` line (step 7). Optional user action: push a fork branch `jazzy-campaign` at `42b7cd7`, then set `branch = jazzy-campaign`. |
| teleop_tools (`src/f1tenth_external/teleop_tools`) | upstream `f1tenth/teleop_tools` | `4337558` | `4337558` | no. `origin/humble-devel` = pin + `163827a` (a setup.cfg underscore fix). | yes, with setup.cfg dash-key warnings | Keep the pin, no `branch` line (`3d545db` removes the one `6118239` adds). |
| transport_drivers (`src/f1tenth_external/transport_drivers`) | upstream `ros-drivers/transport_drivers` | `d3f510c` (tag 1.2.0) | **removed** (`3d545db`) | n/a (`origin/humble`, `origin/main`) | Only from source with libasio-dev, which linus lacks: io_context then fails with "Could NOT find ASIO". apt 1.2.0 is the same release. | **Remove** (step 6). Install apt `ros-jazzy-serial-driver ros-jazzy-io-context ros-jazzy-asio-cmake-module ros-jazzy-udp-msgs` on every Jazzy host. |
| zed_ros2_wrapper (`src/f1tenth_external/zed_ros2_wrapper`, named `src/zed_ros2_wrapper` in `.gitmodules`) | upstream `stereolabs/zed-ros2-wrapper` | `b5844a8` (humble-v4.2.5) | `86171bc` (tag v5.4.1) | n/a (upstream). `86171bc` is reachable through the tag only; it is not an ancestor of `origin/master` `b19902d`. | `b5844a8`: **no** (`#error Unsupported ROS2 distro`). `86171bc`: yes, with ZED SDK 5.4.x and CUDA. | **Bump to `86171bc`** (`9a66870`), in one commit with the camera-code adaptations (step 14). Apt: `ros-jazzy-zed-description`, `zstd-image-transport` (resolved by rosdep). |
| zed-ros2-interfaces (`src/f1tenth_external/zed-ros2-interfaces`) | upstream `stereolabs/zed-ros2-interfaces` | `e970087` (tag 5.3.0) | `e970087` | n/a | yes (rosidl only) | Keep the pin. |
| husarion_gz_worlds (`src/husarion_gz_worlds`) | upstream `husarion/husarion_gz_worlds` | none | `87271ef` (jazzy only, Gazebo) | n/a | yes | **Do not add** (`04780ee` is skipped). |

`.gitmodules` on jazzy-campaign after steps 5 and 6 (reproduced with `patch` on a scratch copy; the indentation is tabs):

```
[submodule "vesc"]
	path = src/f1tenth_hardware/vesc
	url = https://github.com/sjckness/vesc.git
	branch = jazzy
[submodule "ackermann_mux"]
	path = src/f1tenth_external/ackermann_mux
	url = https://github.com/sjckness/ackermann_mux.git
[submodule "teleop_tools"]
	path = src/f1tenth_external/teleop_tools
	url = https://github.com/f1tenth/teleop_tools.git
[submodule "src/zed_ros2_wrapper"]
	path = src/f1tenth_external/zed_ros2_wrapper
	url = https://github.com/stereolabs/zed-ros2-wrapper.git
[submodule "zed-ros2-interfaces"]
	path = src/f1tenth_external/zed-ros2-interfaces
	url = https://github.com/stereolabs/zed-ros2-interfaces.git
```

`docker/orin/jazzy_versions.manifest:112` documents the Thor's checkout and lists ackermann_mux `b3c0b083`. With the kept pin, `image_report.sh` will show "differs" for that row. That is expected and needs no edit.

---

## 5. Ordered port plan

The order is dependency order: rules and baseline first, then f1tenth_messages and f1tenth_params, the repository and driver layer (submodules), libraries, node packages, bringup, and finally environment, tooling, image and docs. Every text patch named here was dry-run with `patch --dry-run` against CF's copy of the file, and all of them apply; offsets are given where they occur. The two gitlink changes cannot be dry-run that way: step 6 describes the expected conflict and its resolution, and step 14 changes a gitlink nobody else touched (CF's net change to it is nil).

**Step 1: rules (skip-list).** Never apply any of the following.

- The 113 commits marked **skip** and the 19 marked **proposal** in Appendix A. Each proposal is decided separately (§7.1).
- Any part of the test-campaign line `1d00853..6cae729`.
- The four merges (`ab60fe4`, `39b2b71`, `714c299`, `6af1ddf`) with `-m`.
- `bb54c8a` and `8d99f78`.
- `472f036`: it duplicates `52682de`.
- `9ed3785`: keep the `COLCON_IGNORE`.
- `c85d9c9`: it changes the semantic score default from 0.5 to 0.35.
- The f1tenth_perception hunks of `1b50dce`.
- Jazzy's whole versions of these files:
  - `stack_params.yaml`, `corridor_geometry.py`;
  - `MPC_corr.py`, `mpc_solver.py`, `wall_turn.py`;
  - `component_supervisor_node.py`, `components.yaml`;
  - `front_clearance_node.py`, `yolo_detector_node.py`, `detection.launch.py`;
  - `mission_logger_node.py`, `loader.py`, `preflight.py`;
  - `base_front.xacro`, `vehicle_frames.py`, `sensors.xacro`, `description.launch.py`.

**Step 2: baseline (no commit).** On `jazzy-campaign` `5b621a9`, record two things.

- The guard grep. Its only hit must be `src/f1tenth_behavior/f1tenth_behavior/behaviours/static_frames.py:14`, a CF docstring; jazzy itself has hits in 84 files:

  ```
  git grep -n -E 'wall_confirm|front_wall_confirmed|wall_stop_require_lidar|mpc_blocked|/mpc/blocked|pass_side|prearm_recorder|mpc_prediction_dt_sec|mpc_control_period_sec|corr_width_ratio|corridor_adaptive|camera_pan|debug_confidence_threshold|input_status|health_watchdog|topic_watchdog|fastdds_profile|apply_sim_mode|f1tenth_sim_drive_bridge|wall_turn_distance_timeout_s|mission_translator_wall_default_distance|matlab_export|effective_params' -- src scripts tools docker ':!src/f1tenth_external' ':!src/f1tenth_sim'
  ```

- The C1 to C34 values. Optionally, a reference build and test of CF on Jazzy, with the baseline skip list `--packages-skip zed_components zed_wrapper zed_ros2 io_context serial_driver udp_driver vesc_driver vesc`.

Repeat the guard grep after every later step.

**Step 3: f1tenth_messages (no change).** Keep CF (`WallLineFit.REASON_SHORT_SPAN`). It is the first package in the build order, and its consumers rebuild after it.

**Step 4: f1tenth_params (no change).** Keep CF's `stack_params.yaml` (346 keys, including `mpc_prediction_dt_s` 0.1, `corr_half_width_start_m` 0.64 and `enable_scene_labels` false), the trapezoid in `corridor_geometry.py`, and `param_defaults.get_wall_distance_topic()`. Gate: the 54 params tests pass.

**Step 5: `.gitmodules`, `6118239` partial (submodule).**

```
git cherry-pick -n 6118239
```

Then delete the `branch = jazzy` line under `[submodule "ackermann_mux"]` and commit; the message names `6118239` and the dropped line. This gives vesc `branch = jazzy`. The teleop_tools and transport_drivers lines are only context for step 6, which removes them again.

**Step 6: drop the transport_drivers submodule, `3d545db` (submodule).**

```
git cherry-pick -x 3d545db
```

The cherry-pick stops with a modify/delete conflict on `src/f1tenth_external/transport_drivers`. The base side is `0257342` from the skipped `bb54c8a`; ours is `d3f510c`. Resolve it as follows:

1. `git rm --cached src/f1tenth_external/transport_drivers`
2. `git cherry-pick --continue`
3. Delete the leftover untracked checkout `src/f1tenth_external/transport_drivers/`, or colcon will still find io_context.

Do **not** run `git submodule deinit`, and do not edit `.git/config`: the config is shared with the main checkout, which still uses the submodule. Gate: `.gitmodules` equals the block in §4.

**Step 7: keep the other pins (submodule, no commit).** These stay: ackermann_mux `42b7cd7` (`8d99f78` is skipped), vesc `2848a9f`, teleop_tools `4337558`, zed-ros2-interfaces `e970087`. One optional action for the user, not the executor: push `42b7cd7` to the fork as branch `jazzy-campaign` (`git -C src/f1tenth_external/ackermann_mux push origin 42b7cd70:refs/heads/jazzy-campaign`), then add `branch = jazzy-campaign` under ackermann_mux. That protects the pin if `humble-jetson` is ever deleted (R15).

**Step 8: f1tenth_description (no change).** Keep CF: bumper 0.405, wheelbase 0.305, laser x 0.305, the beside-body helpers. The Gazebo renames are deferred with the sim (F3). Gate: the 22 description tests pass.

**Step 9: libraries, corridor_perception and f1tenth_costmap (no change).** Keep CF. `fd83d6e` is P1; if P1 is accepted it slots in here as `git cherry-pick -x fd83d6e`, which applies cleanly. Gate: 198 costmap tests and 37 corridor_perception tests pass on linus.

**Step 10: f1tenth_logger (cherry-pick), in jazzy order.**

```
git cherry-pick -x a086f0e 130063c 52682de de1f8d8 91d361e 84a12f1
```

| Commit | What it changes |
|---|---|
| `a086f0e` | `ROS_LOCALHOST_ONLY` in `conftest.py:20` and `scripts/replay_bag_through_mpc.sh:35` |
| `130063c` | the two mcap install hints (hunks at +15 and +98 lines: 65 and 742) |
| `52682de` | `TopicMetadata(id=)` |
| `de1f8d8` | the `all_topics` guard (+111 lines: 820) |
| `91d361e` | `read_bag` picks its storage from the bag |
| `84a12f1` | `ROS_SUPER_CLIENT` in `mission_logger.launch.py` |

After this step, `mission_extract.py`, `mission_logger.launch.py` and `conftest.py` are byte-identical to jazzy's (checked by applying the patches to scratch copies). Gate: `colcon test --packages-select f1tenth_logger` gives 315 passing tests with pyarrow 25.0.1, where it was 309 plus 6 errors before. The lint failures exist on both branches.

**Step 11: mpc_controller (cherry-pick).** `git cherry-pick -x 2ea8541` adds `<test_depend>f1tenth_hardware</test_depend>` (hunk at +4 lines). Gate, with `osqp==1.1.3`: 734 passed, 1 skipped, and the 11 failures that exist on CF on any distro (P16).

**Step 12: f1tenth_navigation (cherry-pick plus a packaging port).**

1. `git cherry-pick -x 4edcbbf` (slam_toolbox as a LifecycleNode, plus the configure and activate events). Afterwards `slam.launch.py` equals jazzy's.
2. In a second commit, add `<exec_depend>slam_toolbox</exec_depend>` and `<exec_depend>lifecycle_msgs</exec_depend>` to `src/f1tenth_navigation/package.xml`, with no `--` inside any comment. `lifecycle_msgs` is now imported by the launch file; `slam_toolbox` was already used but never declared.

Gate: 2 tests pass, and the launch file loads (`ros2 launch f1tenth_navigation slam.launch.py --show-args`).

**Step 13: launch environments for localization and diagnostics (cherry-pick).**

```
git cherry-pick -x a936c75 5b320a5
```

`a936c75` adds `additional_env={'ROS_SUPER_CLIENT': 'TRUE'}` on `ekf_cost_observer_node` only. `5b320a5` sets `SetEnvironmentVariable('ROS_SUPER_CLIENT','TRUE')` in `steering_offset_calibration.launch.py`. After this, `localization.launch.py` equals jazzy's. Gate: localization 14 tests pass; diagnostics 149 pass and 1 is skipped, with `python3-ruamel.yaml` installed.

**Step 14: f1tenth_perception and zed_ros2_wrapper v5.4.1. One commit (submodule plus a port-side manual edit).**

1. `git cherry-pick -n 9a66870` moves the gitlink from `b5844a8` to `86171bc`.
2. In `src/f1tenth_perception/launch/camera.launch.py:99-102`, change only the two `SetRemap` sources; the destinations stay as they are:
   - `src='/zed2/zed_node/rgb/image_rect_color'` becomes `src='/zed2/zed_node/rgb/color/rect/image'` (`dst='/camera/image_raw'` unchanged);
   - `src='/zed2/zed_node/rgb/camera_info'` becomes `src='/zed2/zed_node/rgb/color/rect/camera_info'` (`dst='/camera/camera_info'` unchanged).
3. In `src/f1tenth_perception/config/zed2_perception.yaml`, pin the humble-v4.2.5 `zed2i.yaml` values that campaign-final ran with:
   - under `general:`, `grab_frame_rate: 30`;
   - under `depth:`, `min_depth: 0.3` and `max_depth: 10.0`;
   - change the line 21 comment to `rgb/color/rect/image`.

   Keep `depth_mode: 'PERFORMANCE'` and verify it on the target (C19). Choosing a replacement mode would be a proposal, not part of the port.
4. Optional, test only: update the hidden-topic sample at `test_foxglove_whitelists.py:39` to `/zed2/zed_node/rgb/color/rect/image/compressed`.
5. Commit with a message naming `9a66870`. Then run `git submodule update --init src/f1tenth_external/zed_ros2_wrapper`; the worktree's module store already has `86171bc`.

Why it must be one commit: with the bump alone, `/camera/image_raw` and `/camera/camera_info` go silent, so YOLO, scene labels and the debug image get no input, and the ZED would grab at 60 Hz with depth valid from 0.01 m to 15 m. jazzy never fixed `camera.launch.py`; its `:88-91` still carries the v4 names.

Gate: perception 614/614 once f1tenth_messages has been rebuilt, and `test_zed_extrinsic.py` still passes (it reads the static TF and `camera_model`, not the remaps). Runtime checks are on the Thor (§6.2).

**Step 15: the other node packages (no change).** Keep CF's f1tenth_behavior, llm, f1tenth_control and f1tenth_hardware. Gates on linus:

| Package | Passed | Failed | Skipped | Notes |
|---|---|---|---|---|
| f1tenth_behavior | 352 | 1 | 0 | known float flake `test_grace_then_target_lost` (P16); needs apt `ros-jazzy-py-trees-ros` |
| llm | 233 | 14 | 25 | failures: 12 stub methods (P16) and pre-existing lint |
| f1tenth_control | 7 | 0 | 0 | |

**Step 16: f1tenth_sim (cherry-pick).** `git cherry-pick -x 0607db8` adds an empty `src/f1tenth_sim/COLCON_IGNORE`. Every SIM_GZ commit stays skipped, including `9ed3785`. Gate (C34): colcon no longer discovers f1tenth_sim and still builds f1tenth_more. Use `rosdep install --from-paths src --ignore-src --rosdistro jazzy -y --skip-keys "ament_python python3-pyarrow f1tenth_sim"`.

**Step 17: f1tenth_bringup and f1tenth_more (no change).** Keep CF's supervisor, `components.yaml` and launch files. Do not add sim mode, the watchdog or the lease profile; those are P8, P5 and P3. Gate: the 37 functional bringup tests pass. The 3 lint tests fail on both branches.

**Step 18: `scripts/env/jazzy.sh` (partial patch; one new file = `d9a7f00` + `5b9ffdc` + the `dac4bef` partial + the `39b2b71` partial).**

1. `mkdir -p scripts/env && git show origin/jazzy:scripts/env/jazzy.sh | sed '32,39d' > scripts/env/jazzy.sh`. Mode 100644; the file is sourced, not run. Lines 32-39 are the lease-profile comment and `export FASTRTPS_DEFAULT_PROFILES_FILE=...`, which belong to P3. Line 40, `unset _f1tenth_ws _f1tenth_sh`, must stay.
2. Replace jazzy's lines 100-102 (now 92-94) with the two lines they had before `dac4bef`, as in `5b9ffdc`:

   ```
   # old (3 lines)
   # The ros2 daemon keeps the environment it was started with. After changing
   # any of the above (the profile included), run `ros2 daemon stop` (or use
   # --no-daemon).
   # new (2 lines)
   # The ros2 daemon keeps the environment it was started with. After changing
   # any of the above, run `ros2 daemon stop` (or use --no-daemon).
   ```

3. Optional: comment-only rewording of the mentions of jazzy's Gazebo sim mode (jazzy's lines 2-3 and 53-56; lines 2-3 and 45-48 after the deletion). The result was checked on a scratch copy: 103 lines, `bash -n` and `zsh -n` pass, three exports, `unset` at line 32.

Do not take `d9a7f00`'s hunks in `scripts/jazzy_parity/phase5_bringup.sh`. Gates:

- `bash -n scripts/env/jazzy.sh` and `zsh -n scripts/env/jazzy.sh` pass;
- `grep -c FASTRTPS scripts/env/jazzy.sh` prints 0;
- `grep -n '^export' scripts/env/jazzy.sh` shows only `RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID` and `ROS_DISCOVERY_SERVER=${F1TENTH_DISCOVERY_SERVER:-127.0.0.1:11811}`. That matches CF's `discovery_server_address` 127.0.0.1 and port 11811.

**Step 19: `scripts/stackctl.py` (cherry-pick).** `git cherry-pick -x d29b3e0` sets `os.environ['ROS_SUPER_CLIENT'] = 'TRUE'` before `rclpy.init()`. Not `88fbd42`, which belongs to the P5 watchdog.

**Step 20: tools and scripts API adaptations (manual port; one commit, naming `a086f0e` and `91d361e` as the patterns).**

1. `tools/scene_labels_replay.py:50`: `os.environ['ROS_LOCALHOST_ONLY'] = '1'` becomes `os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'`.
2. `tools/wall_track_replay.py:48` and `tools/wall_distance_replay.py:102`: replace `storage_id='sqlite3'` with a storage id taken from the bag, using `91d361e`'s pattern. Both files already import `Path`:

   ```
   storage_id = 'mcap' if any(p.suffix == '.mcap' for p in Path(bag_dir).iterdir()) else 'sqlite3'
   ```

3. `scripts/replay_bag_through_mpc.sh:65-66`: pass the recorded topics with `--topics $REC_TOPICS`, as positional topics are deprecated on Jazzy. Keep `--storage sqlite3`.

Gates: C3 (rosbag2_py readers), C9 and C29.

**Step 21: `docker/orin` (partial patch = `f574e43` partial + `1b50dce` partial + `f7f06be` + `8ecc13b` + the `af1f167` partial).**

```
git checkout origin/jazzy -- docker/orin
git rm -r -q docker/orin/spike
```

The second command drops the 14 spike files, which jazzy marks THROWAWAY. This takes the final state of the Dockerfile, build.sh, stage_sources.sh, entrypoint.sh, the manifest and `thor_capture/` (13 files). It does **not** take `1b50dce`'s `yolo_detector_node.py` and test changes (P10), or anything under `output/`. Gates:

- `git diff --stat origin/jazzy -- docker/orin` lists only deletions under `docker/orin/spike/`;
- `docker/orin/stage_sources.sh` finds `scripts/env`, which step 18 provides;
- on the Orin (L4T r36.5 only), `docker/orin/build.sh <sha>` builds, and `build_checks.sh` passes. The manifest row for ackermann_mux shows "differs" (§4).

**Step 22: `CLAUDE.md` (cherry-pick plus a partial).**

1. `git cherry-pick -x b0740ea` adds `## Rebuild policy`. It applies cleanly; CF's `CLAUDE.md` equals `573f313`'s.
2. Add `## Orin Docker` from `origin/jazzy:CLAUDE.md` lines 171-195 (from `f574e43`). Reword line 192: `docker/orin/spike/spike.sh` is not ported and `scripts/docker_orin.sh` does not exist yet, so say that `docker/orin/build.sh` uses host networking.

Not the `## DDS / discovery` section (P3), not `## Thor` (`8571558`; with `COLCON_IGNORE` its premise is gone), and not `## Harness gotchas`. That last one is optional: `af1f167`'s two `rm -rf` and ssh bullets describe guards that docker/orin's scripts already implement.

**Step 23: test-parity tool (partial patch of `cf53ed0`, optional).**

```
git checkout origin/jazzy -- scripts/jazzy_parity/test_results_summary.py
```

It is a standalone, stdlib-only tool: per-package counts and failing test IDs from a colcon test-result tree, and `--compare` to diff two runs. Use it to diff the CF baseline (step 2) against jazzy-campaign (step 26).

**Step 24: `.gitignore` (cherry-pick, optional).** `git cherry-pick -x 5c82958` ignores `output/**/ros_log/`, `supervisor/`, `*.mcap`, `*.db3`, `*.mp4` and `phase*/reports/*/*.log` (hunk at -8 lines).

**Step 25: docs (manual, optional).** In `README.md`, line 3 changes "ROS 2 Humble workspace" to Jazzy, and line 68 changes `transport_drivers` from a submodule to apt packages. `docs/setup/JETSON.md` and the Humble Dockerfiles describe the old Humble container; leave them (F4).

**Step 26: verification gates (no commit).**

- linus: build with the §6.1 skip list. Tests against the numbers in steps 4 to 17. Guard grep and C1 to C34. `colcon list` shows 33 packages.
- Thor and Orin: §6.2 and §7.4.
- Then decide the proposals (§7.1).

---

## 6. linus colcon skip list

### 6.1 Build commands

After the port:

```
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-skip zed_components zed_wrapper zed_debug zed_ros2 vesc_driver vesc
```

colcon discovers 33 packages: 17 of ours, ackermann_mux, 5 teleop, 5 vesc, 4 zed and zed_msgs. f1tenth_sim is not among them, because of its `COLCON_IGNORE`. 27 packages are built. Naming a package in `--packages-skip` that does not exist only produces a warning.

On campaign-final as it is (the baseline in step 2), skip `zed_components zed_wrapper zed_ros2 io_context serial_driver udp_driver vesc_driver vesc` instead.

If `sudo apt install ros-jazzy-serial-driver ros-jazzy-io-context ros-jazzy-asio-cmake-module ros-jazzy-udp-msgs` is run on linus (the candidates exist), only the four zed packages remain skipped.

### 6.2 Skipped on linus, and what must be verified on the Thor

| Package | Why it is skipped on linus | What must be verified on the Thor (and on the car's Orin image) |
|---|---|---|
| `zed_components` | `find_package(ZED REQUIRED)` and CUDA: linus has no ZED SDK (`/usr/local/zed` is absent) and no CUDA toolkit | It builds at `86171bc` against ZED SDK 5.4.x, and `ldd -r` is clean (it links diagnostic_updater). With the ZED 2i, the wrapper log shows: grab at 30 Hz; no "depth.depth_mode contains a not valid string" (the SDK's "PERFORMANCE is deprecated" is fine); positional tracking not started, so no odom or map TF from the ZED; `rgb/color/rect/image` and `rgb/color/rect/camera_info` advertised. |
| `zed_wrapper` | depends on zed_components | `ros2 launch f1tenth_perception camera.launch.py` comes up. apt `ros-jazzy-zed-description` is present. `/camera/image_raw` and `/camera/camera_info` publish at about 30 Hz (`ros2cli topic hz`), and `/zed2/zed_node/depth/depth_registered` publishes. TF from `base_link` to `zed2_left_camera_frame_optical` resolves through `zed2_camera_link` (static 0.2399 / 0 / 0.15). YOLO, the scene labels and `/camera/debug_image/compressed` get frames. |
| `zed_debug` | new in v5.4.1; depends on zed_components | It builds, and `ldd -r` is clean. |
| `zed_ros2` | metapackage (exec_depends on zed_components, zed_wrapper, zed_msgs, zed_description) | It builds. |
| `vesc_driver` | `find_package(serial_driver)`: apt serial_driver, io_context and asio_cmake_module are not installed on linus (dpkg shows none), and step 6 removes the source | It builds against apt serial_driver 1.2.0 without libasio-dev, and `ldd -r` is clean. Wherever the VESC is wired (the car; the Thor only if wired): the driver opens the port and publishes `/sensors/core` and `/sensors/imu/raw`. |
| `vesc` | metapackage; depends on vesc_driver | It builds. |

Not on the skip list:

- f1tenth_sim is not discovered (`COLCON_IGNORE`).
- f1tenth_more builds: colcon does not check `exec_depend`s at build time, and jazzy's Thor Phase 0 build under `0607db8` finished it.
- No colcon package needs CUDA or TensorRT to build. YOLO in f1tenth_perception needs torch and ultralytics at runtime (Jetson pip wheels), and an `*.engine` model needs the device's TensorRT 10.x (P10).
- llm's live tests need llama-server, a host binary on port 8083. Their 23 tests skip on linus.

### 6.3 Test prerequisites on linus

These are not skips:

- **pip.** Use a venv, or `PIP_BREAK_SYSTEM_PACKAGES=1` because of PEP 668. Constrain `numpy<2` and `scipy<1.18`.
  - `osqp==1.1.3`: 197 mpc_controller tests fail without it, because `use_rti_solver` defaults to true.
  - `pyarrow==25.0.1`: f1tenth_logger needs it, and 26.0.0 requires numpy 2 or newer.
- **apt, needed for tests:**
  - `ros-jazzy-py-trees-ros` and `ros-jazzy-py-trees-ros-interfaces` (this upgrades py_trees from 2.4.0 to 2.6.0);
  - `python3-ruamel.yaml`.
- **apt, needed at runtime only:** `ros-jazzy-slam-toolbox`, `ros-jazzy-topic-tools`, `ros-jazzy-urg-node`, `ros-jazzy-rosbridge-server`, `ros-jazzy-v4l2-camera`.
- **Rebuild f1tenth_messages first.** linus's installed copy predates `REASON_SHORT_SPAN`, and one perception test fails until it is rebuilt.

---

## 7. Proposals, deferred API items, open risks

### 7.1 Proposals

These are logic changes found on jazzy. They are **not** part of the port; each needs its own decision.

| ID | jazzy source | What it changes | Why it matters for jazzy-campaign | Recommendation / where it would slot in |
|---|---|---|---|---|
| P1 | `fd83d6e` | `safe_corridor.inflate_polytope`: force `keep[i_pt] = False` in the merge branch (CF `:459`) and the new-face branch (CF `:470`) | On aarch64 with numpy 1.26 and OpenBLAS, the loop never ends: `test_safe_corridor` hung `colcon test` for more than 15 h on the Thor. linus passes either way. The path is off by default, and the output is identical according to its authors. | **Accept before any Thor `colcon test`.** Step 9; it applies cleanly. |
| P2 | `931a0eb` | `system_observer_node._read_thermal_zone_cpu_temp` also catches `TypeError` (CF `:108`), plus `test_system_observer_thermal_read.py` | On the Thor, the idle GPU thermal zone returns None (EAGAIN), and the node crashes on its first tick when jtop is absent. Reproduced on Python 3.10 and 3.12. | **Accept for Thor or the Orin container.** After step 13; it applies cleanly. |
| P3 | `7b301b2` (+ `dac4bef` lines 32-39, `39f7856` CLAUDE.md DDS section) | `f1tenth_bringup/config/fastdds_profile.xml` (5 s participant lease, 1 s announcement), `FASTRTPS_DEFAULT_PROFILES_FILE` in both bringup launch files plus a `fastdds_profile` launch argument and stack_params key, and the export in `jazzy.sh` | Under the Discovery Server, Jazzy bringups lost endpoint matching in 8 of 30 runs with the default 20 s lease (slam_toolbox never reached ACTIVE), and in 1 of 60 runs with the profile. Each participant announces its own lease. | **Decide before the first full-stack Jazzy run.** Otherwise wait at least 20 s between bringups, or start a fresh Discovery Server. Needs a hand merge into CF's `setup.py` and `stack_params.yaml`. After step 17. |
| P4 | `b31b76c` | Steering calibration's e-stop preflight polls the graph for up to `estop_graph_wait_sec` (default 5.0, also a launch argument) | With only `5b320a5`, a freshly started super client refuses 3 of 4 starts while the graph converges. | Accept if the calibration is used on Jazzy. After step 13. CF changed only a comment near `:314`. |
| P5 | `96a5b12`, `0f151aa`, `725f256`, `f9e0fea`, `88fbd42`, `49de45d` (one components.yaml line) | Supervisor topic-liveness watchdog (`/supervisor/health`, restart or alert, `health_watchdog` default `enforce`), mpc_corr `/mpc/input_status`, slam and EKF rules, once-settle, stackctl health, and `unwatched: reset_manager` | Silent discovery isolation was seen in about 1 of 320 node starts. Jazzy's backlog has an open "Logger severity cannot be changed between calls" crash in the health tick. | A separate feature port, with a hand merge into CF's `MPC_corr.py`, `components.yaml` and supervisor. Take it as one bundle, or not at all. |
| P6 | `0397d6b` (adapted) | Preflight requires the node that actually produces the wall-stop input | CF's `preflight.py:154,156` still names `costmap_boundary_node`. The producer is `lidar_front_wall_node` when `enable_scene_labels` is on, otherwise `front_clearance_node`. Jazzy's hard-coded `front_clearance_node` is also wrong for CF. | Port the idea, keyed on `get_wall_distance_topic()`. |
| P7 | `7155f79` | Add `/mpc/goal_drive`, `/sensors/imu/raw`, `/joint_states`, `/mpc/status` and `/perception/d_wall/psi_correction` to the logger's `_DEFAULT_TOPICS` | Jazzy found each of them missing from every bag. CF deliberately does not record `/perception/front_distance`. | User decision. |
| P8 | `af41013` (+ the ideas of `f86f4a7`, `0f93aae` and `e67910c`) | Supervisor `sim:=true`: drops the VESC, ZED and urg launches, a hardware-TF stand-in, every component on `use_sim_time`, and refuses calibration | Needed to run CF against the Isaac sim. CF's `supervisor_bringup.launch.py` declares no `sim` argument, and `ros2 launch` silently ignores undeclared arguments, so the real drivers would start. | Re-derive it against CF's `components.yaml`, `camera.launch.py` and measured frames. |
| P9 | `279e3f4` (+ the `49de45d` test stub) | Preflight swaps the `ackermann_to_vesc_node` requirement for the sim's actuation node when `use_sim_time` is set | Every Isaac mission fails preflight if ackermann_to_vesc does not run there. | Use the Isaac bridge's node name. |
| P10 | `1b50dce` (perception part) | `check_trt_engine` / `_strip_ultralytics_engine_metadata` in `yolo_detector_node` | A `*.engine` loads only in the TensorRT build that produced it (Orin image TRT 10.3.0.26 versus Humble 10.3.0.30). The default `.pt` model is unaffected. | Accept if engines are deployed. It applies to CF except hunk 2 (`import json` is already at `:163`). |
| P11 | `babc885` | `build_straight_corridor` handles a None `d_front`: it records NaN and logs "unknown" | CF's `MPC_corr.py:4402` sets `d_front = self._fresh_front_distance()`, which can be None. `:5204` `float(d_front)` and `:5223` `f'{d_front:.2f}'` then raise TypeError, which kills mpc_corr in the middle of a wall_turn. | **Recommended**: a latent crash on CF on any distro. |
| P12 | `20544fa` (the clip and cold start only) | Clip the published steering and `last_u` to `[delta_min, delta_max]` (CF `:4088`, `:4112`); a ramp-to-zero cold warm start when no plan exists (`:3962`) | The test harness saw OSQP steer past the box (0.326 to 0.393 against 0.319). | User decision. |
| P13 | `6ebbc8a` (the rule only) | wall_turn never commits without a fresh front distance | CF's `wall_turn.py:209` commits on the first rebuild even when the distance is unknown. | User decision. Its BLOCKED timeout is test-campaign logic and does not come with it. |
| P14 | `58f9864` (the decision only) | `corridor_heading_return` true or false | CF ships `true`, marked unresolved in `stack_params.yaml:1116-1118`. test-campaign chose `false` after the 2026-09-08 A/B (0.09-0.11 m against 0.75-0.99 m lateral). The `w_du_delta` 8.5714 value does not apply to CF. | User decision. |
| P15 | `c0aa636` (side fix) | Pass a reason to `MissionRuntimeState.abort()` and publish it on `/test/mission_event` | CF's `runtime.py:336` `abort()` takes no reason, so BT-internal aborts publish `''`. | Small, independent fix. |
| P16 | `8eb4c29` (test only) | Fix tests that fail on CF on any distro: `find_root` checkout-name asserts (they fail in `f1tenth_more_jc`), the `_note_llm_*` stubs (12 llm tests), the wall_turn table and the `drive_stop_2m` list, the 0.125 s target_lost tick, and the `mpc_model_check --demo` header | Keeps the test signal clean on jazzy-campaign. | A separate test-hygiene commit. |
| P17 | `c85d9c9` | `semantic_score_threshold` default from 0.5 to 0.35 | It is a default change on the car, made for Gazebo renders. | **Do not adopt.** If Isaac renders also score low, use a sim-only override. |
| P18 | `a637807`, `cb5dd7c`, `ff3ab10`, `a8e8d79`, `740ccae` (+ `ff3df03`, `5bb7fa6` sim) | f1tenth_camera_pan, and stamped camera TF lookups | Only relevant if pan hardware is adopted. | A separate feature, with its `camera_pan_enabled` default false. |
| P19 | `6529fcc` pattern | `except (KeyboardInterrupt, ExternalShutdownException)` and `rclpy.try_shutdown()` in stack nodes | 64 files call `rclpy.shutdown()` after a SIGINT has already shut the context: RCLError traceback and exit code 1 on launch stop. The same happens on Humble (jazzy backlog M13). | Clean-up, not part of the port. |
| P20 | `66ef8e8` (side observation) | mpc_corr's measured-steering joint names | CF's `MPC_corr.py:3231-3232` matches `car_1_*_steering_hinge_joint`, which nothing publishes, so `delta_real` stays None (jazzy backlog M14). | Review. |

### 7.2 Deferred API items

These are not in this port. Follow-ups only.

- **F1. Readers that cannot read mcap.**
  - Files: `f1tenth_logger/mission_debug_video.py:88-108` (new on CF), `corridor_perception/replay.py:113-128`, `tools/scene_labels_replay.py:84-85`, `tools/measure_stop_distance.py:88-91`, `tools/test_campaign_backfill.py:170-173`.
  - `scene_labels_replay.py:333,576` also uses the v4 name `zed2_left_camera_optical_frame`.
  - All of them keep working on the archived Humble `.db3` runs.
  - For Jazzy runs, choose one:
    - (a) add a `rosbag2_py.SequentialReader` path whose storage is taken from the bag (the `91d361e` pattern), with frame names as a parameter;
    - (b) record sqlite3 on the car, which needs a `components.yaml` argument override for `mission_bag_storage_id`. That is a config change and needs the user's decision.
- **F2. Nav2 Jazzy parameter names** (dormant while `enable_nav2` is false). In `nav2_params.yaml`:
  - `:23` `progress_checker_plugin` becomes `progress_checker_plugins: ["progress_checker"]`;
  - `:92-93` become `local_costmap_topic` and `local_footprint_topic`;
  - `:102` becomes `local_frame: odom`;
  - empty `plugin_lib_names` (`:121-168`), because Jazzy registers the built-in plugins itself.

  Not verified live.
- **F3. Gazebo Fortress to Harmonic**, only if a Gazebo sim on Jazzy is wanted.
  - f1tenth_sim: `package.xml:20-22`, the launch file, `ros_gz_bridge.yaml` type names, `use_stamped_vel` in `controllers.yaml`, `empty_room.sdf`, `drive_bridge` shutdown.
  - f1tenth_description: `ros2_control.xacro:23,69-70` and `sensors.xacro:48,94,138,158`.
  - Keep CF's car values (wheelbase 0.305 and so on). Jazzy's sim tests pin the car TFs at 0.12 and would have to be rebased.
- **F4. Humble-only images and docs.**
  - `docker/Dockerfile.cpu` (`ros:humble`), `Dockerfile.gpu`, `Dockerfile.jetson-thor`, `docker/entrypoint.jetson.sh`;
  - `.devcontainer/*` (`source /opt/ros/humble`);
  - `docs/setup/JETSON.md`;
  - `src/scripts/*.sh` (Foxy).
- **F5. rosbag2 deprecations.** `Recorder()` without arguments, `record(storage, record)` and `cancel()`: warnings only. jazzy left them as well (its backlog L8).
- **F6. Old-style `static_transform_publisher` arguments.** Four call sites; a deprecation warning only.
- **F7. Pre-existing undeclared dependencies** (not Jazzy-specific):
  - f1tenth_localization does not declare f1tenth_diagnostics or f1tenth_bringup;
  - f1tenth_bringup does not declare vesc_msgs or f1tenth_logger;
  - f1tenth_sim does not declare f1tenth_bringup;
  - corridor_perception's tests read f1tenth_perception's test data.

### 7.3 Open risks

| ID | Risk | Mitigation |
|---|---|---|
| R1 | The port base `7b17e2f` is older than the car (`e3793e47` and `736d46f` are not pushed). The car's params differ in 160 lines: `enable_scene_labels` true, laser z 0.19, ZED z 0.205 with pitch -2.083°, new logger keys `mission_logger_record_depth` and `_annotated_image`, multi-wall tracker, scene mask. | Re-run this plan on the pushed tip. Re-check the hunk positions in `mission_logger_node.py` (de1f8d8 at :820, 130063c at :65 and :742) and in `camera.launch.py` and `zed2_perception.yaml`. Re-run C3, C10, C11 and the guard grep over the new car code (new readers, graph calls, depth recording). |
| R2 | ZED v5.4.1 with this stack is unverified on a real camera, on both branches: the remap fix, `PERFORMANCE`, the now-active `publish_*` flags, `pos_tracking_mode` AUTO, the new optical frame names, the pinned rates and depth limits | The step 14 gates on the Thor, then on the Orin image (§6.2). |
| R3 | Discovery isolation with the default 20 s lease (8 of 30 Jazzy bringups) | P3, or at least 20 s between bringups, or a fresh Discovery Server. Watch slam_toolbox reach ACTIVE. |
| R4 | One `Recorder` per run: if a recorder thread does not exit within the 15 s join, the next `Recorder()` raises "context is already initialized" and that mission is silently unrecorded. Recording from LOADED has never run on Jazzy. | On the Thor, run at least 2 missions, plus a LOADED restart, in one logger process. Every bag must have messages and `extract.parquet` must be written. |
| R5 | The storage format changes from sqlite3 (as recorded on the Humble car) to mcap, which is CF's configured default | F1. Archived `.db3` runs still open everywhere. |
| R6 | Thor `colcon test` hangs in `test_safe_corridor` without P1 | P1, or run that file under `timeout`. |
| R7 | `system_observer_node` crashes on the Thor without P2 | P2, or provide jtop via `/run/jtop.sock`. |
| R8 | Steering calibration refuses about 3 of 4 starts under the Discovery Server even with `5b320a5` | P4. |
| R9 | ABI drift after `ros-jazzy-*` upgrades (diagnostic_updater 4.2.6 to 4.2.7) | Follow the Rebuild policy (step 22): `--cmake-clean-first` and the `ldd -r` scan. |
| R10 | robot_localization 3.8.3 and slam_toolbox 2.8.5 have only been validated on pre-campaign bags (jazzy Phases 1 and 2). The EKF configs are unchanged on CF, but the slam lifecycle path, together with CF's `reset_manager` restart, has never run on a car. | A bag replay plus a live run. Check `ros2cli lifecycle get /slam_toolbox`, and check that there is no map-to-odom TF. |
| R11 | topic_tools 1.3.4 `ThrottleNode` parameters (`lazy`, `msgs_per_sec`) in `viz_relays.launch.py`, and foxglove_bridge 3.2.6/3.6 `use_compression`, have not been checked live | First live run: `/viz/*` must appear in Foxglove. |
| R12 | Dependencies outside rosdep: osqp 1.x, pyarrow 25.0.1, numpy below 2, PEP 668. On the Thor, `~/.local` holds PyPI torch, nvidia-* wheels and a setuptools that shadow apt. | §6.3. On the Thor use `PYTHONNOUSERSITE=1` and expose osqp and pyarrow through `PYTHONPATH`. |
| R13 | `docker/orin` is work in progress (`af1f167`): there is no run script yet (`scripts/docker_orin.sh`), the build refuses any host but L4T r36.5, and it needs the pinned inputs under `/scratch/fabiocar`. The manifest is the Thor's. | Build and smoke-test on the Orin. Treat the manifest's "differs" row for ackermann_mux as expected. |
| R14 | Test-campaign or jazzy-feature logic comes back through a whole-file take or a merge | The step 1 rules, and the guard grep after every step. |
| R15 | ackermann_mux `42b7cd7` lives only on `origin/humble-jetson`; zed_ros2_wrapper `86171bc` is reachable only through tag `v5.4.1` | Optionally push a fork branch for ackermann_mux (step 7). Do not delete the tag. |
| R16 | The super-client commits assume plain-client hosts, as with `jazzy.sh` and the image. A host that still exports `ROS_SUPER_CLIENT=TRUE` globally (the Humble Orin's bashrc) makes every node a super client, which defeats the Discovery Server. | Source only `scripts/env/jazzy.sh` on Jazzy hosts. |
| R17 | Every Jazzy host needs the transport apt packages. The Thor lacked `ros-jazzy-asio-cmake-module` at first, and linus has none of them. | `rosdep install`, or the explicit apt line in §4. |
| R18 | MPC timing on the Orin: the Humble run already overran 100 ms (mean 112 ms, worst 309 ms). CPU pinning is tuned for the 12-core Orin (`cpu_affinity` `'10,11'` fails where there are fewer than 12 cores; linus has 8). | Re-measure in the Jazzy container. Pass `cpu_affinity:=` on small hosts. |
| R19 | The lint tests (flake8, pep257, copyright) fail on both branches | They existed before the port; no style fixes in the port. |

### 7.4 Thor and Orin verification list

This comes after the port and covers what linus cannot run.

1. Build all 33 packages on the Thor, then run the `ldd -r` scan; it must print nothing.
2. Run `colcon test`, with `test_safe_corridor` under `timeout` unless P1 is in. Compare with the step 2 baseline using `test_results_summary.py --compare`.
3. ZED and camera checks (§6.2).
4. Full stack under the Discovery Server:
   - the mission bag has messages (`84a12f1`);
   - `ekf_cost_observer` metrics are non-zero (`a936c75`);
   - `stackctl.py status` lists the supervisor services (`d29b3e0`);
   - slam_toolbox is active (`4edcbbf`), and nothing publishes a map-to-odom TF;
   - two missions in one logger process (R4);
   - mcap bags, and `extract.parquet` is written (`91d361e`).
5. `system_observer` on the Thor (P2), and the steering-calibration preflight (P4).
6. On the Orin: `docker/orin/build.sh <jazzy-campaign sha>`, `build_checks.sh`, `image_report.sh`. Then the same runtime checks inside the container: host networking, `/dev/shm`, the VESC and lidar devices.


---

## Appendix A: all 159 jazzy-only commits

The order is topological, as in `git log --reverse --topo-order 573f313..origin/jazzy`, so the jazzy-sim and test-campaign side lines appear just before the merges that bring them in. "Files to apply / note" names exactly what the port takes, with the step number from §5. The **proposal** rows are explained in §7.1.

| # | sha | date | subject | categories | action | files to apply / note |
|---|---|---|---|---|---|---|
| 1 | `bb54c8a` | 2026-09-30 | transport_drivers: point jazzy branch at COLCON_IGNORE commit (0257342) | SUBMODULE, JAZZY_INFRA | **skip** | none: pointer 0257342 is not on the remote; superseded by 3d545db |
| 2 | `8d99f78` | 2026-09-30 | ackermann_mux: correct superproject pointer to b3c0b083 | SUBMODULE, JAZZY_INFRA | **skip** | none: keep campaign pin 42b7cd7 (config-yaml-only delta, reachable on origin/humble-jetson) |
| 3 | `6118239` | 2026-09-30 | .gitmodules: set branch = jazzy for the four ported submodules | SUBMODULE, JAZZY_INFRA | **partial** | .gitmodules: `branch = jazzy` for vesc (teleop_tools/transport_drivers lines only as context, removed again by 3d545db); NOT for ackermann_mux (step 5) |
| 4 | `2a57817` | 2026-09-30 | output: Jazzy migration analysis, plan and C++ port report | DOCS | **skip** | none (output/ reports) |
| 5 | `3d545db` | 2026-09-30 | jazzy: drop transport_drivers submodule (replaced by Jazzy apt serial_driver/io_context/asio_cmake_module); teleop_tools stays on upstream pin without branch tracking | SUBMODULE, MIGRATION | **cherry-pick** | .gitmodules; delete gitlink src/f1tenth_external/transport_drivers (modify/delete conflict, resolve with git rm; step 6) |
| 6 | `0607db8` | 2026-09-30 | jazzy/phase0: f1tenth_sim: defer Gazebo Harmonic port — ign_ros2_control/ros_ign_gazebo have no Jazzy equivalent under those names | SIM_GZ, MIGRATION | **cherry-pick** | src/f1tenth_sim/COLCON_IGNORE (step 16) |
| 7 | `a086f0e` | 2026-09-30 | jazzy/phase0: ROS_LOCALHOST_ONLY -> ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST — deprecated in Jazzy | MIGRATION | **cherry-pick** | scripts/replay_bag_through_mpc.sh, src/f1tenth_logger/test/test_campaign/conftest.py (step 10; same rename port-side in tools/scene_labels_replay.py:50, step 20) |
| 8 | `130063c` | 2026-09-30 | jazzy/phase0: f1tenth_logger: mcap install hint says ros-jazzy-, not ros-humble- | MIGRATION | **cherry-pick** | src/f1tenth_logger/f1tenth_logger/mission_logger_node.py (2 strings; step 10) |
| 9 | `9a66870` | 2026-09-30 | jazzy/thor: bump zed_ros2_wrapper submodule humble-v4.2.5 -> v5.4.1 | SUBMODULE, MIGRATION | **cherry-pick** | gitlink zed_ros2_wrapper b5844a8 -> 86171bc, in ONE commit with port-side edits to camera.launch.py and zed2_perception.yaml (step 14) |
| 10 | `fd83d6e` | 2026-10-01 | jazzy/p0: f1tenth_costmap: fix numpy-1.26 infinite loop in inflate_polytope | FEATURE, MIGRATION | **proposal** | none (P1) |
| 11 | `2ea8541` | 2026-10-01 | jazzy/p0: mpc_controller: declare test_depend on f1tenth_hardware | MIGRATION | **cherry-pick** | src/f1tenth_control/mpc_controller/package.xml (step 11) |
| 12 | `52682de` | 2026-10-01 | jazzy/p0: f1tenth_logger: rosbag2_py.TopicMetadata requires id= in Jazzy | MIGRATION | **cherry-pick** | src/f1tenth_logger/test/test_object_run_summary.py (step 10; never together with 472f036) |
| 13 | `4edcbbf` | 2026-10-01 | jazzy/p0: f1tenth_navigation: slam.launch.py never activated slam_toolbox | MIGRATION | **cherry-pick** | src/f1tenth_navigation/launch/slam.launch.py (+ port-side package.xml exec_depends; step 12) |
| 14 | `3781f2e` | 2026-10-01 | jazzy/p0: Thor Phase 0 validation report, build/test logs, env snapshots | DOCS | **skip** | none (output/) |
| 15 | `9beea9b` | 2026-10-01 | jazzy/p1: localization parity harness — slam_pose_relay bit-exact, ekf_global within noise floor at steady state | HARNESS, DOCS | **skip** | none (parity harness) |
| 16 | `5fec439` | 2026-10-01 | jazzy/p2: SLAM + costmap parity harness — slam_toolbox TF-silent and within noise floor, semantic_layer bit-exact, costmap_boundary tight median agreement | HARNESS, DOCS | **skip** | none (parity harness) |
| 17 | `bd3bd3b` | 2026-10-01 | jazzy/p2: costmap_boundary function-level verification + SLAM map / commit-reference corrections | HARNESS, DOCS | **skip** | none (parity harness) |
| 18 | `119cb4c` | 2026-10-01 | jazzy/p2: fill in the addendum's own commit hash | DOCS | **skip** | none |
| 19 | `691e30d` | 2026-10-02 | jazzy/p2: frozen-input boundary parity test (Thor vs Orin) | HARNESS, DOCS | **skip** | none (parity harness) |
| 20 | `19c8703` | 2026-10-02 | jazzy/p3: goal injection for MPC replays — the bag never recorded /mpc/goal_drive | HARNESS | **skip** | none (parity harness, pre-campaign logic) |
| 21 | `48108cd` | 2026-10-02 | jazzy/p3: mpc and mpc_capture replay layers — Humble/Jazzy MPC parity needs the production launch under sim time | HARNESS | **skip** | none (parity harness) |
| 22 | `210f380` | 2026-10-02 | jazzy/p3: frozen-input MPC solver test (Thor vs Orin) — separate solver numerics from replay timing | HARNESS, DOCS | **skip** | none (frozen pre-campaign MPC inputs) |
| 23 | `684ef1f` | 2026-10-02 | jazzy/p3: MPC replay metrics, timing summary and plots — Jazzy noise floor and indicative timing | HARNESS, DOCS | **skip** | none (parity harness) |
| 24 | `fc1ae6f` | 2026-10-02 | jazzy/p3: Phase 3 report and Orin MPC instructions — GO-WITH-NOTES, Orin comparison pending | DOCS | **skip** | none |
| 25 | `de1f8d8` | 2026-10-02 | jazzy/p4: f1tenth_logger: RecordOptions.all -> all_topics — Jazzy rosbag2_py removed 'all', so no mission was ever recorded | MIGRATION | **cherry-pick** | src/f1tenth_logger/f1tenth_logger/mission_logger_node.py (all_topics guard; step 10) |
| 26 | `91d361e` | 2026-10-02 | jazzy/p4: f1tenth_logger: read_bag picks the storage plugin from the bag — mcap runs failed extraction with a hardcoded sqlite3 | MIGRATION | **cherry-pick** | src/f1tenth_logger/f1tenth_logger/mission_extract.py (step 10; same pattern port-side in tools/wall_track_replay.py:48, tools/wall_distance_replay.py:102, step 20) |
| 27 | `3968af0` | 2026-10-02 | jazzy/p4: bag_digest hashes message content — raw CDR bytes include uninitialised padding | HARNESS, DOCS | **skip** | none |
| 28 | `c437903` | 2026-10-02 | jazzy/p4: BT and ekf_cost_observer replay layers — Phase 4 needs both replayed identically on Jazzy and Humble | HARNESS | **skip** | none |
| 29 | `8cba2b3` | 2026-10-02 | jazzy/p4: logger, diagnostics and LLM check scripts — end-to-end evidence for Phase 4 Steps 2-4 | HARNESS | **skip** | none |
| 30 | `9f3d558` | 2026-10-02 | jazzy/p4: Phase 4 results, Orin BT instructions and report — GO-WITH-NOTES, Orin BT comparison pending | DOCS | **skip** | none |
| 31 | `fc990d3` | 2026-10-02 | jazzy/p4: fill in the report's own commit hash — the commit list said (next) | DOCS | **skip** | none |
| 32 | `931a0eb` | 2026-10-03 | fix/diagnostics: system_observer skips a thermal zone whose sysfs read returns None — Thor's idle GPU zone crashed the node on its first tick | FEATURE, HARNESS | **proposal** | none (P2) |
| 33 | `0397d6b` | 2026-10-03 | fix/behavior: preflight requires front_clearance_node for a front_clearance condition — it checked costmap_boundary_node, the producer before 63a6080 | FEATURE, HARNESS | **proposal** | none (P6) |
| 34 | `7155f79` | 2026-10-03 | fix/logger: record the drive goal, the front_clearance stop input, IMU, joint and MPC status topics — Phases 3-4 found each of them missing from every bag | FEATURE, HARNESS | **proposal** | none (P7) |
| 35 | `e4d6110` | 2026-10-03 | fix/batch1: report and evidence for the three non-migration fixes — soak, BT replay and logger-check data behind each commit | DOCS | **skip** | none |
| 36 | `7d3303c` | 2026-10-03 | fix/batch1: fill in the report's own commit hash — the commit list said (next commit) | DOCS | **skip** | none |
| 37 | `42b416d` | 2026-10-03 | jazzy/p5: full-stack bringup harness — Phase 5 needs the production supervisor bringup run, fed and measured, without hardware | HARNESS | **skip** | none |
| 38 | `af41013` | 2026-10-03 | jazzy/sim: supervisor_bringup sim:=true — the Gazebo phase needs the stack without hardware drivers, on simulated time | FEATURE, SIM_GZ | **proposal** | none (P8) |
| 39 | `3978b81` | 2026-10-03 | jazzy/p5: Phase 5 results, env proposal and report — GO-WITH-NOTES, logger needs a super client under the Discovery Server | DOCS | **skip** | none |
| 40 | `49f7a2a` | 2026-10-03 | jazzy/p5: fill in the report's own commit hash — the commit list said (next commit) | DOCS | **skip** | none |
| 41 | `d9a7f00` | 2026-10-03 | fix/env: scripts/env/jazzy.sh for interactive Jazzy shells — hand-started ROS tools need the Discovery Server settings the launch files give the stack | JAZZY_INFRA, HARNESS | **partial** | scripts/env/jazzy.sh (new file; realised by the single-step file of step 18); NOT the scripts/jazzy_parity/phase5_bringup.sh hunks |
| 42 | `84a12f1` | 2026-10-03 | fix/logger: mission_logger.launch.py sets ROS_SUPER_CLIENT for the logger — under the Discovery Server a plain client's recorder finds no topics and the bag is empty | JAZZY_INFRA | **cherry-pick** | src/f1tenth_logger/launch/mission_logger.launch.py (step 10) |
| 43 | `b0740ea` | 2026-10-03 | fix/docs: CLAUDE.md rebuild policy — an apt ABI change left ackermann_mux unloadable and only a full-stack run found it | DOCS, JAZZY_INFRA | **cherry-pick** | CLAUDE.md (## Rebuild policy; step 22) |
| 44 | `715907c` | 2026-10-03 | fix/docs: output/backlog.md, one list of post-migration items — they were spread over seven reports' decision sections | DOCS | **skip** | none |
| 45 | `e248d33` | 2026-10-03 | fix/batch2: report and evidence — env script, logger super client, rebuild policy, backlog | DOCS | **skip** | none |
| 46 | `34a43a6` | 2026-10-03 | fix/batch2: fill in the report's own commit hash — the commit list said (next commit) | DOCS | **skip** | none |
| 47 | `a936c75` | 2026-10-03 | fix/diagnostics: ekf_cost_observer_node runs as a Discovery Server super client — as a plain client it subscribed to nothing and half its metrics read 0 | JAZZY_INFRA | **cherry-pick** | src/f1tenth_localization/launch/localization.launch.py (step 13) |
| 48 | `5b320a5` | 2026-10-03 | fix/diagnostics: steering_offset_calibration.launch.py sets ROS_SUPER_CLIENT — as a plain client its e-stop preflight never sees the /safety_stop publisher and always refuses | JAZZY_INFRA | **cherry-pick** | src/f1tenth_diagnostics/launch/steering_offset_calibration.launch.py (step 13) |
| 49 | `d29b3e0` | 2026-10-03 | fix/tools: stackctl.py runs as a super client — as a plain Discovery Server client `status` reported every supervisor service MISSING on a running stack | JAZZY_INFRA | **cherry-pick** | scripts/stackctl.py (step 19) |
| 50 | `ee9b0a0` | 2026-10-03 | fix/batch3: report, live-check script and data, backlog update — plain-client graph audit | DOCS, HARNESS | **skip** | none |
| 51 | `4510559` | 2026-10-03 | fix/batch3: fill in the report's own commit hash — the commit list said (next commit) | DOCS | **skip** | none |
| 52 | `b31b76c` | 2026-10-03 | fix/diagnostics: steering calibration e-stop check waits up to 5 s for the graph — a single read under a new super client refused 3 of 4 starts with the mux and the BT both up | FEATURE | **proposal** | none (P4) |
| 53 | `bb76c09` | 2026-10-03 | fix/discovery: isolation investigation — scripts, short-lease profile and data for 150 full-stack bringups and 120 stack-free rounds | HARNESS, DOCS | **skip** | none |
| 54 | `898d7a6` | 2026-10-03 | fix/batch4: report, backlog, e-stop live data — H6 closed, H1 root-caused with a proposed lease fix and watchdog design | DOCS, HARNESS | **skip** | none |
| 55 | `8c0891d` | 2026-10-03 | fix/batch4: fill in the report's commit hashes — the commit list said (next) | DOCS | **skip** | none |
| 56 | `7b301b2` | 2026-10-04 | fix/discovery: 5 s participant lease profile for every participant of the stack, set by both bringup launch files — a killed node now leaves the Discovery Server in 4.5 s instead of 19.4 s | FEATURE | **proposal** | none (P3) |
| 57 | `dac4bef` | 2026-10-04 | fix/env: jazzy.sh exports the Fast DDS lease profile for hand-started participants, and works from zsh as well as bash | JAZZY_INFRA | **partial** | scripts/env/jazzy.sh: shell detection, setup.<sh> sourcing, `unset _f1tenth_ws _f1tenth_sh`; NOT the FASTRTPS_DEFAULT_PROFILES_FILE export or its comment (step 18) |
| 58 | `0f151aa` | 2026-10-04 | fix/control: mpc_corr publishes /mpc/input_status, an odometry-freshness heartbeat — /drive is published with or without odometry, so nothing showed an isolated mpc_corr | FEATURE | **proposal** | none (P5) |
| 59 | `96a5b12` | 2026-10-04 | fix/bringup: supervisor topic-liveness watchdog — each component's output topics, received by the supervisor in wall time, restart an isolated component within its restart budget | FEATURE | **proposal** | none (P5) |
| 60 | `88fbd42` | 2026-10-04 | fix/tools: stackctl.py status prints the supervisor's liveness watchdog, one line per watched component, and fails on any ERROR | FEATURE, HARNESS | **proposal** | none (P5) |
| 61 | `725f256` | 2026-10-04 | fix/bringup: liveness checks corrected by the live tests — slam judged by one map per start, the EKFs by their rate, not by age or header.stamp | FEATURE | **proposal** | none (P5) |
| 62 | `009813c` | 2026-10-04 | fix/discovery: fix batch 5 measurement harness — lease probe, discovery CPU/traffic per thread class, watchdog live tests, isolation and discovery-load batches | HARNESS | **skip** | none |
| 63 | `39f7856` | 2026-10-05 | fix/batch5: report, backlog, data — H1 closed by the lease profile and the liveness watchdog; GO-WITH-NOTES | DOCS | **skip** | none (CLAUDE.md DDS section goes with P3) |
| 64 | `ca89326` | 2026-10-05 | fix/batch5: fill in the report's own commit hash — the commit list said (next) | DOCS | **skip** | none |
| 65 | `63f46d4` | 2026-10-02 | jazzy/sim: sim_host_setup.sh — reproducible native Jazzy + ros_gz host install | SIM_GZ, JAZZY_INFRA | **skip** | none (Gazebo host installer) |
| 66 | `4f0a3d3` | 2026-10-02 | jazzy/sim: ros2_control.xacro on gz_ros2_control — CM reads /sim/robot_description, ground truth off TF | SIM_GZ, MIGRATION | **skip** | none (Gazebo; renames deferred F3) |
| 67 | `cc3addd` | 2026-10-02 | jazzy/sim: sim sensors match the real mounts — front-facing laser, 50 Hz IMU, camera mock off | SIM_GZ, MIGRATION | **skip** | none (Gazebo; renames deferred F3) |
| 68 | `90cbfb1` | 2026-10-02 | jazzy/sim: stack_params use_sim text — gz_ros2_control, and the sim no longer includes description.launch.py | SIM_GZ, DOCS | **skip** | none (stack_params text only) |
| 69 | `ace37af` | 2026-10-02 | jazzy/sim: empty_room.sdf system plugins renamed to gz-sim-* / gz::sim::systems::* | SIM_GZ, MIGRATION | **skip** | none (Gazebo world) |
| 70 | `66d9ff3` | 2026-10-02 | jazzy/sim: ros_gz_bridge.yaml on gz.msgs — IMU to /sim/imu_raw, ground truth added, ZED dropped | SIM_GZ, MIGRATION | **skip** | none (Gazebo bridge) |
| 71 | `bd49bfc` | 2026-10-02 | jazzy/sim: controllers.yaml for Jazzy — 50 Hz odom, real covariances, 0.2 s reference timeout | SIM_GZ, MIGRATION | **skip** | none (Gazebo controllers; f1tenth_sim ignored) |
| 72 | `6739899` | 2026-10-02 | jazzy/sim: drive_bridge takes /ackermann_drive, clamps to the servo envelope, relays the IMU as the car does | SIM_GZ, FEATURE | **skip** | none (sim bridge logic) |
| 73 | `8b8a83a` | 2026-10-02 | jazzy/sim: sim_bringup on ros_gz — private TF-silent RSP, no EKF/SLAM, driver drop-in only | SIM_GZ, MIGRATION | **skip** | none (sim launch rewrite) |
| 74 | `0ef3500` | 2026-10-02 | jazzy/sim: f1tenth_sim deps on ros_gz_sim/ros_gz_bridge/gz_ros2_control; robot_localization and slam_toolbox dropped | SIM_GZ, MIGRATION | **skip** | none (sim package deps) |
| 75 | `2d10bc9` | 2026-10-02 | jazzy/sim: sim port report — Steps 0–3 and 5 done, Step 4 checklist pending the ROS install | DOCS, SIM_GZ | **skip** | none |
| 76 | `d94083f` | 2026-10-02 | LOG: sim port to Jazzy/Harmonic and decisions D0–D5 | DOCS | **skip** | none |
| 77 | `503df8f` | 2026-10-02 | LOG: real hash for the sim port report | DOCS | **skip** | none |
| 78 | `06a0ca1` | 2026-10-03 | jazzy/sim: sim_host_setup.sh reads apt sources directly (deb822 + .list); runs from zsh | SIM_GZ, JAZZY_INFRA | **skip** | none |
| 79 | `01abfe6` | 2026-10-03 | jazzy/sim: import order per ament_flake8 (I100) — first colcon test run caught it | SIM_GZ | **skip** | none |
| 80 | `2e672a3` | 2026-10-03 | jazzy/sim: controller_manager /diagnostics → /sim/diagnostics | SIM_GZ | **skip** | none |
| 81 | `6529fcc` | 2026-10-03 | jazzy/sim: drive_bridge exits cleanly on launch shutdown | SIM_GZ, MIGRATION | **skip** | none (pattern recorded as P19) |
| 82 | `551fa1a` | 2026-10-03 | jazzy/sim: sim_host_setup.sh installs ros2controlcli | SIM_GZ | **skip** | none |
| 83 | `9ed3785` | 2026-10-03 | jazzy/sim: f1tenth_sim builds again — COLCON_IGNORE removed after Step 4 passed | SIM_GZ | **skip** | none: keep 0607db8's COLCON_IGNORE |
| 84 | `caefd05` | 2026-10-03 | jazzy/sim: report — Step 4 results (12/12 pass), RTF on Iris Xe vs MX450, §5 pending af41013 | DOCS, SIM_GZ | **skip** | none |
| 85 | `4827536` | 2026-10-03 | LOG: Jazzy sim Step 4 verification | DOCS | **skip** | none |
| 86 | `66ef8e8` | 2026-10-03 | jazzy/sim: sim joint states on /sim/joint_states — the car's joint_state_publisher owns /joint_states | SIM_GZ | **skip** | none (side observation recorded as P20) |
| 87 | `feabbd2` | 2026-10-03 | jazzy/sim: report + LOG — /sim/joint_states, merge (not rebase) plan for af41013 | DOCS, SIM_GZ | **skip** | none |
| 88 | `ab60fe4` | 2026-10-04 | Merge origin/jazzy into jazzy-sim — Thor Phase 5 sim:=true meets the Jazzy sim host (merge) | MERGE | **skip** | none (never cherry-pick -m) |
| 89 | `5b9ffdc` | 2026-10-04 | env: jazzy.sh works from zsh and takes the Discovery Server address from F1TENTH_DISCOVERY_SERVER | JAZZY_INFRA | **cherry-pick** | scripts/env/jazzy.sh (zsh support, F1TENTH_DISCOVERY_SERVER; realised by the single-step file of step 18) |
| 90 | `f86f4a7` | 2026-10-04 | jazzy/sim: supervisor sim-mode docstring — f1tenth_sim publishes no camera topics | SIM_GZ, DOCS | **skip** | none |
| 91 | `6a41f2b` | 2026-10-04 | jazzy/sim: §5 reconciled against Thor's sim:=true; backlog M14 (dead MPC measured steering), M15, L14; linus as Phase S sim host | DOCS, SIM_GZ | **skip** | none |
| 92 | `70dc09f` | 2026-10-04 | LOG: jazzy-sim merged with origin/jazzy, §5 reconciled | DOCS | **skip** | none |
| 93 | `39b2b71` | 2026-10-05 | Merge origin/jazzy into jazzy — the TPad's sim work (70dc09f) meets fix batch 5 (merge) | MERGE, JAZZY_INFRA, SIM_GZ, DOCS | **partial** | scripts/env/jazzy.sh merge-resolution hunk only (`_f1tenth_ws=${F1TENTH_WS:-$_f1tenth_ws}`; step 18); never cherry-pick -m 1 |
| 94 | `f9e0fea` | 2026-10-05 | fix/bringup: liveness watchdog `once` checks wait settle_sec after their input appears — a /scan that first came after slam's grace restarted a healthy slam | FEATURE | **proposal** | none (P5) |
| 95 | `72c028b` | 2026-10-05 | fix/batch5: once-settle live check, report and backlog — B6 fixed (M19 closed), Andreas's decisions of 2026-10-05 | HARNESS, DOCS | **skip** | none |
| 96 | `8571558` | 2026-10-05 | fix/docs: CLAUDE.md Thor section — ~/.colcon/defaults.yaml skips f1tenth_more with f1tenth_sim, or its exec_depend on the skipped sim fails the build | DOCS, JAZZY_INFRA | **skip** | none (Thor skip section not needed with COLCON_IGNORE) |
| 97 | `0d73ab9` | 2026-10-05 | f1tenth_sim: default to husarion_office world | SIM_GZ | **skip** | none |
| 98 | `04780ee` | 2026-10-06 | f1tenth_sim: default to 2x-scaled husarion office world | SIM_GZ, SUBMODULE | **skip** | none (no husarion_gz_worlds submodule) |
| 99 | `e67910c` | 2026-10-06 | f1tenth_sim: move simulated LiDAR/ZED mounts (car TFs unchanged) | SIM_GZ, FEATURE | **skip** | none |
| 100 | `0d233e2` | 2026-10-06 | f1tenth_sim: throttle /clock (Gazebo 1 kHz -> /sim/clock_raw -> clock_throttle -> /clock), clock_probe script, fan-out report | SIM_GZ, DOCS | **skip** | none (/clock lesson noted for Isaac) |
| 101 | `083c8e8` | 2026-10-06 | f1tenth_sim: 4 ms physics step, /clock 250 Hz | SIM_GZ, DOCS | **skip** | none |
| 102 | `a4488e5` | 2026-10-07 | f1tenth_sim+perception: compress the simulated ZED over the two-machine link | SIM_GZ, DOCS | **skip** | none |
| 103 | `0f93aae` | 2026-10-07 | f1tenth_bringup: sim mode swaps camera.launch.py -> sim_camera_tf.launch.py | SIM_GZ, FEATURE | **skip** | none |
| 104 | `c85d9c9` | 2026-10-07 | f1tenth_costmap: lower semantic_score_threshold default 0.5 -> 0.35 | FEATURE, SIM_GZ | **proposal** | none (P17, recommended NOT to adopt: car default change) |
| 105 | `2713326` | 2026-10-07 | scripts/sim: one-command launchers for the two-machine sim | SIM_GZ, JAZZY_INFRA | **skip** | none |
| 106 | `69281f4` | 2026-10-07 | scripts/sim: rename stack.sh -> sim_stack.sh | SIM_GZ | **skip** | none |
| 107 | `279e3f4` | 2026-10-07 | f1tenth_behavior: mission preflight requires the sim drive bridge in sim | FEATURE, SIM_GZ | **proposal** | none (P9) |
| 108 | `a637807` | 2026-10-08 | f1tenth_camera_pan: motorized ZED pan -- aim law + controller + measured-angle TF (stack side) | FEATURE | **skip** | none (camera pan; P18 bundle) |
| 109 | `cb5dd7c` | 2026-10-08 | f1tenth_camera_pan: tf node drops the pan TF when the measured angle is stale | FEATURE | **skip** | none (camera pan) |
| 110 | `ff3df03` | 2026-10-08 | f1tenth_sim: simulated camera pan joint (URDF + mount + VFOV test) | SIM_GZ, FEATURE | **skip** | none |
| 111 | `ff3ab10` | 2026-10-08 | f1tenth_camera_pan: steering-only mode while parked (track_when_stopped) | FEATURE | **skip** | none (camera pan) |
| 112 | `5bb7fa6` | 2026-10-08 | f1tenth_sim: wire the camera pan into the sim (controller, bridge, launch) | SIM_GZ, FEATURE | **skip** | none |
| 113 | `f574e43` | 2026-10-07 | phase5d D0 + D1a: Orin Jazzy container analysis, option C spike, Thor manifest | JAZZY_INFRA, DOCS | **partial** | docker/orin/jazzy_versions.manifest, docker/orin/thor_capture/* (13 files), CLAUDE.md `## Orin Docker`; NOT docker/orin/spike/*, output/* (steps 21-22) |
| 114 | `1b50dce` | 2026-10-07 | phase5d D1: Orin Jazzy image (option C) + detector TensorRT version guard | JAZZY_INFRA, FEATURE | **partial** | the 16 docker/orin image files; NOT docker/orin/spike/spike.sh, NOT yolo_detector_node.py + test_yolo_detector_node.py (P10) (step 21) |
| 115 | `f7f06be` | 2026-10-07 | phase5d D1: link llama.cpp against the CUDA driver stub | JAZZY_INFRA | **cherry-pick** | docker/orin/Dockerfile (realised by step 21) |
| 116 | `8ecc13b` | 2026-10-07 | phase5d D1: image sets NVIDIA_VISIBLE_DEVICES, USER by name | JAZZY_INFRA | **cherry-pick** | docker/orin/Dockerfile, docker/orin/image_versions.sh (realised by step 21) |
| 117 | `cf53ed0` | 2026-10-07 | phase5d D1: image report, runtime checks, Thor reference colcon test | DOCS, HARNESS, JAZZY_INFRA | **partial** | scripts/jazzy_parity/test_results_summary.py only (step 23) |
| 118 | `af1f167` | 2026-10-12 | Phase 5D docker: WIP | JAZZY_INFRA, DOCS | **partial** | docker/orin/build.sh, docker/orin/stage_sources.sh (step 21); CLAUDE.md harness bullets optional |
| 119 | `a8e8d79` | 2026-10-08 | f1tenth_camera_pan: forward pan args from the sim bringup; mode/fixed_angle/track_when_stopped dynamic | FEATURE, SIM_GZ | **skip** | none (camera pan / sim) |
| 120 | `5c82958` | 2026-10-12 | .gitignore: ignore bulk per-run harness artifacts under output/ | HARNESS | **cherry-pick** | .gitignore (step 24) |
| 121 | `00b3b1c` | 2026-10-12 | f1tenth_sim: test_room_6x6 world replicating the real test room | SIM_GZ | **skip** | none |
| 122 | `1d00853` | 2026-10-07 | 7/10 tests | TEST_CAMPAIGN | **skip** | none (adopted pieces already identical on campaign-final) |
| 123 | `9765c4d` | 2026-10-12 | M02 campaign data | DOCS, TEST_CAMPAIGN | **skip** | none (data already under first_test_campaign/) |
| 124 | `16b7b95` | 2026-10-09 | output: campaign fix plan for M02 R009-R011 (Step 0, code-level answers) | DOCS, TEST_CAMPAIGN | **skip** | none |
| 125 | `2919e51` | 2026-10-09 | mpc_corr: prediction step 0.2 s, control period 0.1 s (campaign fix A1) | TEST_CAMPAIGN, FEATURE, HARNESS | **skip** | none (superseded by campaign-final 30b0580) |
| 126 | `e1f8d6a` | 2026-10-09 | mpc_corr: wide fixed corridor, geometry as parameters (campaign fix A2) | TEST_CAMPAIGN, FEATURE | **skip** | none (superseded by a6dd23e) |
| 127 | `c0aa636` | 2026-10-09 | mpc_corr + mission: speed 0 or >= 0.4 on every move, keep-outs, BLOCKED (campaign fix A3) | TEST_CAMPAIGN, FEATURE | **skip** | none (abort-reason side fix recorded as P15) |
| 128 | `dab5787` | 2026-10-09 | wall stop default 0.5 m as a parameter, and who chose it (campaign fix A4, 1/2) | TEST_CAMPAIGN, FEATURE | **skip** | none (superseded by 32bf769) |
| 129 | `6b4953f` | 2026-10-09 | wall stop fires only when camera AND LiDAR agree (campaign fix A4, 2/2) | TEST_CAMPAIGN, FEATURE | **skip** | none (superseded by 8b352d2/ca8e15b) |
| 130 | `75ccbe6` | 2026-10-09 | logging + bag replay for the front distance, the solver and BLOCKED (campaign fix A5) | TEST_CAMPAIGN, FEATURE, HARNESS | **skip** | none |
| 131 | `d2ef4b4` | 2026-10-09 | output: Part A review -- params changed, numbers, R011 bag findings, Orin commands, test procedures | DOCS, TEST_CAMPAIGN | **skip** | none |
| 132 | `0b9e8ee` | 2026-10-09 | planner prompt: regression tool old vs new, and the prompt's hash in every run (review item 3) | TEST_CAMPAIGN, FEATURE, HARNESS | **skip** | none |
| 133 | `04facd9` | 2026-10-09 | BLOCKED is its own mission outcome (review item 4) | TEST_CAMPAIGN, FEATURE | **skip** | none |
| 134 | `959b40d` | 2026-10-09 | keep-outs on the car's outline, one committed pass side (review items 1, 2) | TEST_CAMPAIGN, FEATURE | **skip** | none |
| 135 | `ce36ebb` | 2026-10-09 | record from mission load, and every effective parameter (review item 5) | TEST_CAMPAIGN, FEATURE, MIGRATION | **skip** | none (its all_topics hunk is de1f8d8; Recorder reuse not needed) |
| 136 | `8eb4c29` | 2026-10-09 | pre-existing test failures: fix the stale, flaky and environment ones (review item 7) | TEST_CAMPAIGN, HARNESS | **proposal** | none (P16) |
| 137 | `ba58788` | 2026-10-09 | scenario harness: fake obstacles into the REAL mpc_corr, no perception (review item 6) | HARNESS, TEST_CAMPAIGN | **skip** | none |
| 138 | `472f036` | 2026-10-09 | logger test_object_run_summary: build the fixture bag on Jazzy's rosbag2_py too (review item 7) | MIGRATION | **skip** | none: duplicate of 52682de (same lines, conflicting form); never both |
| 139 | `626c70b` | 2026-10-09 | output: review items 1-7 summary | DOCS, TEST_CAMPAIGN | **skip** | none |
| 140 | `58f9864` | 2026-10-12 | decisions: w_du_delta 8.5714, corridor_heading_return false, reference missions restored | TEST_CAMPAIGN, FEATURE | **proposal** | none (P14) |
| 141 | `20544fa` | 2026-10-12 | mpc_corr: no BLOCKED cascade after a BLOCKED; steering clipped to its bounds | TEST_CAMPAIGN, FEATURE, HARNESS | **proposal** | none (P12) |
| 142 | `426eabf` | 2026-10-12 | mpc_solver: per-step rate weights follow the step they span (A1 review) | TEST_CAMPAIGN, FEATURE | **skip** | none (superseded by 30b0580) |
| 143 | `babc885` | 2026-10-12 | mpc_corr: wall_turn with a stale or missing front distance no longer crashes | TEST_CAMPAIGN, FEATURE | **proposal** | none (P11) |
| 144 | `f25e1e3` | 2026-10-12 | output: Part A review and review items brought to the final state; Part B backlog with the Orin's R011 findings | DOCS, TEST_CAMPAIGN | **skip** | none |
| 145 | `714c299` | 2026-10-12 | Merge branch 'test-campaign' of github.com:sjckness/f1tenth_more into test-campaign (merge) | MERGE, TEST_CAMPAIGN | **skip** | none |
| 146 | `6ebbc8a` | 2026-10-12 | wall_turn: no front distance = no wall decision, BLOCKED no_wall_distance after 2 s (B8) | TEST_CAMPAIGN, FEATURE | **skip** | none (its rule recorded as P13) |
| 147 | `48829de` | 2026-10-12 | wall_turn: rename wall_distance_timeout_s -> wall_turn_distance_timeout_s | TEST_CAMPAIGN, FEATURE | **skip** | none |
| 148 | `6cae729` | 2026-10-12 | wall_confirm: carry a confirmed wall down to the 0.5 m stop (latch) | TEST_CAMPAIGN, FEATURE | **skip** | none |
| 149 | `6af1ddf` | 2026-10-12 | Merge test-campaign (M02 campaign fixes) into jazzy (merge) | MERGE, TEST_CAMPAIGN | **skip** | none (never cherry-pick -m) |
| 150 | `da79f37` | 2026-10-12 | output: Part B backlog B9 (wall_turn on the raw front distance) and B10 (stale LiDAR pose in sensors.xacro) | DOCS | **skip** | none |
| 151 | `49de45d` | 2026-10-12 | sync fixes: sim steering clamp, reset_manager watchdog entry, test stub | TEST_CAMPAIGN, SIM_GZ | **proposal** | none now; components.yaml `unwatched: reset_manager` line only together with P5 |
| 152 | `f639ea6` | 2026-10-12 | snapshot before jazzy test parity (2026-10-12) | DOCS | **skip** | none (.claude/settings.json) |
| 153 | `27f84d3` | 2026-10-12 | claude settings: fold .claude/settings.json into the git-ignored settings.local.json | DOCS | **skip** | none (repo policy, separate decision) |
| 154 | `cb5735f` | 2026-10-12 | mpc_corr: corridor funnel with a fixed end/start ratio; adaptive widening, default off | FEATURE | **skip** | none (superseded by a6dd23e) |
| 155 | `30fa1c7` | 2026-10-13 | base_front: one URDF front-bumper frame; every front distance measured from it | FEATURE | **skip** | none (already ported as 10d3eb3) |
| 156 | `740ccae` | 2026-10-13 | camera_pan: camera_pan_enabled (default false) as a real parameter | FEATURE | **skip** | none (camera pan) |
| 157 | `9e21beb` | 2026-10-13 | yolo_detector: sub-threshold detections drawn grey on the annotated image | FEATURE | **skip** | none (superseded by 0334b8c) |
| 158 | `5f19359` | 2026-10-13 | output: jazzy test parity report (overnight run 2026-10-12/13) | DOCS | **skip** | none |
| 159 | `3dbe37f` | 2026-10-13 | LOG: jazzy test parity run (corridor funnel, base_front, pan off, grey boxes) | DOCS | **skip** | none |

---

## Appendix B: campaign-final's 27 commits after `573f313` and what each means for Jazzy

The port keeps all of them unchanged (jazzy-campaign is built on them). The last column says where Jazzy touches them.

| sha | Subject | Main paths | Jazzy relevance |
|---|---|---|---|
| `a6dd23e` | Part D: trapezoid corridor, 2.3/1.3 end/start ratio, start half-width 0.64 m | mpc_controller, f1tenth_params | Pure Python. Never take jazzy's `cb5735f` funnel. |
| `407ff6d` | Part B: record from mission LOADED, 15 s countdown as pre-roll | f1tenth_logger, f1tenth_behavior | One Recorder per run: R4. |
| `10d3eb3` | Part E: base_front frame; front distances from the bumper | description, behavior, mpc_controller, perception, costmap, llm | A re-port of jazzy's `30fa1c7`, so skip `30fa1c7`. |
| `0334b8c` | Part C: 5 Hz debug image, wall-estimator debug topic, host MP4 renderer | perception, logger | `mission_debug_video` reads sqlite only (F1). |
| `30b0580` | Part F: MPC prediction step 0.2 s, N = 20; loop stays 10 Hz | mpc_controller, f1tenth_control, params | Pure Python. The yaml value is 0.1 since `c830202`. Never take jazzy's `2919e51` key names. |
| `a940ed8` | output: campaign-final report | output/ | none |
| `f7df8a7` | output: report records that the branches are pushed (moved two gitlinks by accident) | output/, 2 gitlinks | Restored by `5c25ade`; the net effect is none. |
| `5c25ade` | Restore the ackermann_mux and zed_ros2_wrapper pointers to `573f313` | 2 gitlinks | The reason ackermann_mux stays at `42b7cd7`. The zed bump is deliberate (step 14). |
| `9b8f1b8` | The campaign's calibration: erpm gain, servo gains, MPC steering bounds | bringup, hardware, params, mpc_controller | Identical on jazzy (via `1d00853`). |
| `32bf769` | LLM wall-guard default | llm | Pure Python. Never take jazzy's `dab5787`. |
| `3f3cfba` | Bring-up: viz relays, foxglove allowlist, reset_manager | f1tenth_bringup, params, logger test | Byte-identical on jazzy. topic_tools 1.3.4 `ThrottleNode` parameters: R11. |
| `e9e5ade` | mission_logger_node: flake8 E305 | f1tenth_logger | none |
| `bdc7c93` | Vehicle frames from the 2026-10-13 tape measurement | perception, description, behavior, params | Pure Python and URDF. |
| `d8609b5` | Front clearance: a point beside the car is excluded | costmap, perception, description | Pure Python. |
| `86f249a` | test_foxglove_whitelists: allowlist is a list | bringup test | Jazzy's copy of this test is stale; keep CF's. |
| `d7b2770` | Wheelbase 0.305 in the URDF; bumper, LiDAR, ZED follow | description, perception, behavior, f1tenth_sim | The f1tenth_sim part becomes inert (`COLCON_IGNORE`). |
| `1ff0644` | Launch the ZED wrapper as a ZED 2i | f1tenth_perception | Couples to the v5.4.1 `zed2i.yaml` defaults: pinned in step 14. |
| `1bb5ea2` | missions: drive_test_straight_2m | f1tenth_behavior | Data only. |
| `c830202` | Snapshot of 2026-10-13 work in progress before scene labelling | llm, mpc_controller, perception, params | Pure Python. Sets `mpc_prediction_dt_s` back to 0.1 and walls to bezier. |
| `7d033af` | params: `enable_scene_labels` (default false) and the scene-label keys | f1tenth_params | The car runs `true` (R1). |
| `8b352d2` | lidar_front_wall: detection cut, wall planes, virtual wall | perception, f1tenth_messages | `REASON_SHORT_SPAN` requires rebuilding f1tenth_messages. |
| `5c30b4d` | Stereo scene labelling, OTHER-only object depth, debug overlay | f1tenth_perception | Pure Python. |
| `ca8e15b` | Consumers: wall stop and wall_turn read the one wall truth | behavior, mpc_controller, logger | `get_wall_distance_topic()`; P6 builds on it. |
| `3ffe245` | tools: scene_labels_replay and its report | tools | `ROS_LOCALHOST_ONLY` fixed in step 20. sqlite and v4 frame names: F1. |
| `cdfd736` | Full test-campaign data (first_test_campaing) | data | none; it supersedes jazzy's `9765c4d`. |
| `a8a61db` | campaign: fix folder name | logger, params tests, data | none |
| `7b17e2f` | docs: repo reorganisation plan | docs/ | Port base. |
