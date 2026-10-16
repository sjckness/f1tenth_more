# jazzy-campaign port + Isaac sim mode: running log

Times are linus local time (CEST).

- 2026-10-16 02:22 Start. Host `linus`, Ubuntu 24.04, ROS 2 Jazzy at /opt/ros/jazzy.
- 02:22 Main checkout `~/dev_ws/f1tenth_more` is on `campaign-final` (7b17e2f) with **local, uncommitted state**: submodule pointers moved (ackermann_mux b3c0b08, zed_ros2_wrapper cce25d3-dirty, zed-ros2-interfaces dirty) and untracked `docs/branches.md`, `src/husarion_gz_worlds/`. Not touched. All port work happens in a new worktree (decision D1).
- 02:23 `git fetch --all --prune`: origin has only `main`, `campaign-final` (7b17e2f), `jazzy` (3dbe37f) plus `archive/*`, `evidence/*` and `jazzy-p*` tags (the cleanup in docs/branches.md has been done on origin).
- 02:24 `e3793e47` (the commit in the 2026-10-15 bags) is **not** on origin: not in any ref (`git ls-remote`), and `git fetch origin e3793e47f595c501e0c9796e93791faf0cc9edef` fails with `upload-pack: not our ref`. Also missing: `736d46fe14` (2026-10-14 bags). No copy on linus or /HDD2T has them.
- 02:27 Submodules fetched (all remotes). Every submodule commit pinned by campaign-final 7b17e2f is on a remote branch or tag (Phase 0 table).
- 02:35 Worktree `~/dev_ws/f1tenth_more_jc` created on new branch `jazzy-campaign` at 7b17e2f (upstream unset, so no accidental push target); submodules initialised at campaign-final's pins.
- 02:40 Params evidence: the car's 2026-10-15 params snapshot differs from 7b17e2f's `stack_params.yaml` in 160 lines (scene mask, wall fusion, multi-wall tracker, `enable_scene_labels` true, stage timing, depth recording). Those features are on the Orin only.
- 02:30 Phase 0 committed (5b621a9).
- 02:49 isaac_ws f1tenth: --ros-distro (default jazzy) + --dds-profile, smoke test base_room 7/7 PASS with the Jazzy bridge (ba9c29b).
- 03:56 Phase 1: analysis workflow (23 agents, 0 errors) wrote plan.md: 159 jazzy-only commits = cherry-pick 19, partial 8, proposal 19, skip 113; 20 proposals; linus skip list zed_components zed_wrapper zed_debug zed_ros2 vesc_driver vesc.
- 03:56 linus lacks slam_toolbox, py_trees_ros, topic_tools, rosbridge_server (apt) and osqp, pyarrow, ultralytics, torch (pip): building an overlay (jc_deps_ws) and a venv (jc_venv) in user space (D6).
- 09:51 Phase 2: 25 port commits on jazzy-campaign (plan steps 5-25) + review fixes d9c381e; colcon build 27/27 packages, 0 errors (skip: zed_components zed_wrapper zed_debug zed_ros2 vesc_driver vesc); colcon test 2868 tests in our packages + externals: all functional failures pre-existing on campaign-final (mpc 11, llm 12, behavior 1 flake, logger 2 checkout-name), lint failures pre-existing; 54/54 interfaces show; adversarial review: no logic/param/default change.
- 10:05 Phase 3 code: supervisor sim mode (D9-D14), sim TF launch files, overlay, preflight actuation param, tests 28/28 (test_sim_mode 14, test_preflight 14). Launch snapshots: default, `sim:=none use_sim_time:=false`, `calibration:=true`, `enable_intelligence:=false` are byte-identical before/after; `sim:=isaac`: 54/54 nodes on sim time.
- 10:11 `scripts/sim/sim_isaac.sh` written (shellcheck 0.11.0 clean; shellcheck from pip `shellcheck-py` in a scratch venv, not installed system-wide).
- 10:14 First `sim_isaac.sh all --scene base_room`: /clock after 34 s, stack up, 61 nodes. Topic coverage: 12 subscribed-but-unpublished topics, all also unpublished on the car or optional (phase3.md §6). TF: one parent per frame, one /tf_static message per static edge.
- 10:20 Found: Isaac car creeps backwards when parked (-6.5 mm/s, yaw -0.005 rad/s; wheel joints turn although their targets are 0). Three fixes tried (wheel kd, steering kd, position brake), none worked: logged as isaac_ws D25 (f07f6c3), not fixed (D20).
- 10:22 Script bugs found in that run and fixed: the Discovery Server was started twice in `all` (one leaked), and the stack launch inherited an ignored SIGINT from the script's background job, so the cleanup could not stop it (now `env --default-signal=INT`); leftovers removed with `scripts/kill_ros2.py -y`.
