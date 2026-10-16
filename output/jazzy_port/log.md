# jazzy-campaign port + Isaac sim mode: running log

Times are linus local time (CEST).

- 2026-10-16 02:22 Start. Host `linus`, Ubuntu 24.04, ROS 2 Jazzy at /opt/ros/jazzy.
- 02:22 Main checkout `~/dev_ws/f1tenth_more` is on `campaign-final` (7b17e2f) with **local, uncommitted state**: submodule pointers moved (ackermann_mux b3c0b08, zed_ros2_wrapper cce25d3-dirty, zed-ros2-interfaces dirty) and untracked `docs/branches.md`, `src/husarion_gz_worlds/`. Not touched. All port work happens in a new worktree (decision D1).
- 02:23 `git fetch --all --prune`: origin has only `main`, `campaign-final` (7b17e2f), `jazzy` (3dbe37f) plus `archive/*`, `evidence/*` and `jazzy-p*` tags (the cleanup in docs/branches.md has been done on origin).
- 02:24 `e3793e47` (the commit in the 2026-10-15 bags) is **not** on origin: not in any ref (`git ls-remote`), and `git fetch origin e3793e47f595c501e0c9796e93791faf0cc9edef` fails with `upload-pack: not our ref`. Also missing: `736d46fe14` (2026-10-14 bags). No copy on linus or /HDD2T has them.
- 02:27 Submodules fetched (all remotes). Every submodule commit pinned by campaign-final 7b17e2f is on a remote branch or tag (Phase 0 table).
- 02:35 Worktree `~/dev_ws/f1tenth_more_jc` created on new branch `jazzy-campaign` at 7b17e2f (upstream unset, so no accidental push target); submodules initialised at campaign-final's pins.
- 02:40 Params evidence: the car's 2026-10-15 params snapshot differs from 7b17e2f's `stack_params.yaml` in 160 lines (scene mask, wall fusion, multi-wall tracker, `enable_scene_labels` true, stage timing, depth recording). Those features are on the Orin only.
