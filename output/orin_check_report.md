# Orin check report: campaign-final on Humble (2026-10-13)

**Status 2026-10-13 ~11:30:** campaign-final is now `e9e5ade`, which adds P1-P3 (campaign calibration, LLM
default, campaign bring-up) on Andreas's OK. Verdict: **CONDITIONAL GO**, see the last section.

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


## 3. What the campaign really ran (step 3)

### Verdict

**The 58 campaign runs (2026-09-21 .. 10-01) ran the committed HEAD of the day plus an uncommitted
tree. 1d00853 later committed that tree unchanged. For P004-R012..R019 (2026-10-01) the state was
`573f313` + the source part of `1d00853`.**
- Runtime files: **high** confidence.
- Exact bytes of the offline/tooling files: medium.

It is not `573f313` alone, and it is not `573f313` + `stack_params.yaml` alone (that is `7e5910e`).

Evidence, strongest first:

1. **Reflog (Orin).** The tree stayed on `test-logging` and was never committed during the campaign.
   - `573f313` committed 2026-09-30 18:25.
   - 2026-10-07 11:27:55: checkout test-logging -> test-campaign at 573f313.
   - 11:28:01: commit `1d00853` "7/10 tests", 6 s later, 861 files. That is a commit of the whole
     dirty tree, not new work.
   - No ROS launch dirs exist for 2026-10-02..10-07, so the car was not run in between.
2. **Manifests (`~/f1tenth_archive/complete/*/*.manifest.json`).**
   - All 58 runs say branch `test-logging`, `dirty: true`.
   - P004-R012..R019 = `573f313b1ed5…` (also in their `meta.json` `git_commit`).
   - No manifest or log captured a diff: the logger stores only `git status --porcelain` != "".
   - Caveat: "dirty" would also be set by untracked run output alone, so on its own it proves nothing.
3. **Params snapshots.** Every 09-30 and 10-01 `*.params.yaml` = blob `44f88f9` = `1d00853`'s
   `stack_params.yaml`. `573f313`/`e47e646` have `28ba26c`. Earlier days also differ from their commits:
   - 09-21/22: `961e7e6` vs `c009182`
   - 09-23..28: `fcc1337` vs `08398d6`
4. **Stack restarted for EVERY run, not just every day.**
   - `~/.ros/log` holds one full bring-up per run (discovery server + component_supervisor + 20
     component launches, 22 dirs within ~3 s):
     - 09-30: 7 bring-ups (17:26..17:39) for 7 runs;
     - 10-01: 9 bring-ups (11:14:58, 11:17:17, 11:20:41, 11:23:10, 11:28:02, 11:30:40, 11:32:09,
       11:33:57, 11:35:25) for 9 runs, each 1-2 min before its run, each ended by ctrl-c.
   - With symlink-install the nodes read src at launch, and the snapshot is copied at record time
     ~1-2 min later.
   - **So the 1d00853 yaml values were really loaded.** This is the condition linus put on `7e5910e`, and it is met.
5. **Code new in 1d00853 ran during the campaign** (10-01 node and launch logs):
   - `reset_manager` started 9x;
   - viz `throttle` relays 18x;
   - `viz_jpeg_node` ran;
   - `foxglove_bridge` advertised only the whitelist set (`/viz/scan`,
     `/viz/camera/image_annotated/compressed`, `/viz/costmap/visualization/compressed`, /tf, ...).
   - `reset_manager` appears in launch logs from 09-21 on (4/54/62/50/21/14/25 launch logs on
     09-21/22/23/24/28/30/10-01).
   - None of these files exist at `573f313`.
   - This settles linus's "bring-up work cannot be confirmed for 10-01": it was live.
6. **erpm gain 4516.9 live.** /odom displacement vs /slam/pose displacement in the archived bags:
   - 09-30 ratios 1.007, 1.005, 0.997, 0.995, 0.988, 0.993;
   - 10-01 ratios 0.958, 0.990, 0.996, 0.983.
   - With 5499.27 the ratio would be ~0.82 (5499.27/4516.9 = 1.2175, see vesc.yaml's own comment).
   - Script: odom_vs_slam.py, sqlite opened read-only.
7. **Steering gains -1.0552/-1.1133 live.** linus's servo-gain readout from the bags, plus steering
   saturating at +0.319/-0.309 (campaign-final `output/campaign_final_report.md`, Part A).
8. **LLM translator default live.** From 09-22 the model's raw output omits `thresh` on wall guards
   (P002-R001..R008, P005-R002..R009 `llm_raw`) and the missions still translated. That needs
   1d00853's `_default_wall_thresh` in `plan_translate.py`. On 10-08 at clean `e47e646` the model
   emitted `thresh: 3.0` again.

`origin/campaign-dirty-evidence` (`7e5910e`) is correct for what it covers (the yaml) but incomplete.
It leaves out the erpm gain, the steering gains and the LLM default.

### Files that differ

- Orin HEAD `6ebbc8a` + working tree vs `573f313`: 1026 files, +257913/-365.
  - 899 are `first_test_campaing/` run data.
  - Code: src/f1tenth_control 20, f1tenth_logger 18, f1tenth_bringup 14, f1tenth_behavior 14,
    f1tenth_intelligence 10, f1tenth_perception 5, corridor_perception 2, f1tenth_params 1,
    f1tenth_hardware 1.
  - Also tools 17, test_campaign 14, output 5, docs 4, scripts 2.
- Orin HEAD `6ebbc8a` vs `1d00853`: 182 files, +72291/-262. These are the 24 post-campaign
  commits on test-campaign (campaign fixes A1-A5, B8, M02 data). None of them is campaign state.
- The working tree adds only `.claude/settings.json` and `pr.json` (section 2). No source edits.
- `573f313..1d00853` restricted to runtime code and config, and what campaign-final has of it:

| file(s) | campaign (1d00853) | in campaign-final 5c25ade? | runtime evidence |
|---|---|---|---|
| `f1tenth_bringup/config/vesc.yaml` | speed_to_erpm_gain **4516.9** | **NO** (5499.27) | odom/SLAM ratio ~1.00 |
| `f1tenth_hardware/config/steering_calibration.yaml` | gain_left **-1.0552**, gain_right **-1.1133** | **NO** (-1.2135 both) | bag servo readout (linus) |
| `stack_params.yaml` steering bounds | -0.309 / +0.319 | **NO** (-0.283 / +0.278) | params snapshot, saturation |
| `llm/plan_translate.py`, prompts v2, `llm_planner_node.py` | wall thresh default 2.0/3.0 | NO (plan_translate differs only by B-F) | llm_raw without thresh |
| bring-up: components.yaml, reset_manager, viz_relays, viz_jpeg_node, foxglove launch, supervisor (4 lines) | present | NO | 10-01 launch logs |
| stack_params viz/foxglove keys | present | NO | snapshot |
| mission_logger_node.py (never record `/viz/`) | present | NO | none (inert for the default topic list) |
| missions/*.json, matlab_export, run_select, corridor_perception tracker, scripts | present | NO | offline / not launched |
| `models/yolo26_office.pt` (45 MB) | untracked file | NO | not configured (`yolo_model: yolo26s-seg.pt`) |

### stack_params.yaml (descriptions excluded; 49 more rows differ only in description text)

Orin now = `6ebbc8a`; the working tree has the same yaml. campaign-final added for reference.

| param | 573f313 | 1d00853 (= campaign snapshot) | Orin now 6ebbc8a | campaign-final 5c25ade |
|---|---|---|---|---|
| `corr_L_base` | – | – | 3.0 | – |
| `corr_half_width_start_m` | – | – | – | 0.64 |
| `corr_lookahead_frac` | – | – | 0.85 | – |
| `corr_width_scale_with_length` | – | – | – | False |
| `corr_wmax` | – | – | 0.6 | – |
| `corr_wmin` | – | – | 0.6 | – |
| `corridor_heading_return` | True | True | False | True |
| `debug_image_jpeg_quality` | – | – | – | 50 |
| `debug_image_min_confidence` | – | – | – | 0.1 |
| `debug_image_rate_hz` | – | – | – | 5.0 |
| `debug_image_width` | – | – | – | 480 |
| `enable_viz_relays` | – | True | True | – |
| `foxglove_client_topic_whitelist` | – | ['^/clicked_point$', '^/goal_pose$', '^/initialpos | ['^/clicked_point$', '^/goal_pose$', '^/initialpos | – |
| `foxglove_send_buffer_limit` | – | 2500000 | 2500000 | – |
| `foxglove_service_whitelist` | – | '^(?!/zed2/)(?!.*/(describe_parameters|get_paramet | '^(?!/zed2/)(?!.*/(describe_parameters|get_paramet | – |
| `foxglove_topic_whitelist` | – | ['^/viz/.*', '^/tf$', '^/tf_static$', '^/robot_des | ['^/viz/.*', '^/tf$', '^/tf_static$', '^/robot_des | – |
| `foxglove_use_compression` | – | False | False | – |
| `front_clearance_publish_debug_raw` | False | False | True | False |
| `front_clearance_publish_debug_wall` | – | – | – | True |
| `front_clearance_wall_confirm_enabled` | – | – | True | – |
| `front_clearance_wall_confirm_half_angle_deg` | – | – | 30.0 | – |
| `front_clearance_wall_confirm_min_extent_m` | – | – | 0.8 | – |
| `front_clearance_wall_confirm_min_inliers` | – | – | 10 | – |
| `front_clearance_wall_confirm_search_window_m` | – | – | 0.6 | – |
| `front_clearance_wall_confirm_tolerance_frac` | – | – | 0.1 | – |
| `front_clearance_wall_confirm_tolerance_m` | – | – | 0.3 | – |
| `mission_translator_wall_default_distance` | – | – | 0.5 | – |
| `mission_translator_wall_default_distance_before_turn` | – | – | 3.0 | – |
| `mpc_blocked_enabled` | – | – | True | – |
| `mpc_blocked_infeasible_sec` | – | – | 1.0 | – |
| `mpc_blocked_lock_fraction` | – | – | 0.95 | – |
| `mpc_blocked_lock_window_sec` | – | – | 2.0 | – |
| `mpc_blocked_min_progress_m` | – | – | 0.15 | – |
| `mpc_blocked_plan_check_sec` | – | – | 2.0 | – |
| `mpc_blocked_progress_window_sec` | – | – | 2.5 | – |
| `mpc_control_period_sec` | – | – | 0.1 | – |
| `mpc_horizon_steps` | – | – | 20 | – |
| `mpc_keepout_footprint_circles` | – | – | 3 | – |
| `mpc_min_speed_all_moves` | – | – | True | – |
| `mpc_obstacle_keepout_enabled` | – | – | True | – |
| `mpc_pass_side_forget_sec` | – | – | 2.0 | – |
| `mpc_pass_side_match_radius_m` | – | – | 0.5 | – |
| `mpc_prediction_dt_s` | – | – | – | 0.2 |
| `mpc_prediction_dt_sec` | – | – | 0.2 | – |
| `mpc_steering_angle_max_rad` | 0.278 | 0.319 | 0.319 | 0.278 |
| `mpc_steering_angle_min_rad` | -0.283 | -0.309 | -0.309 | -0.283 |
| `mpc_w_du_delta` | 8.33 | 8.33 | 8.5714 | 8.33 |
| `swept_clearance_body_front_x_m` | 0.443 | 0.443 | 0.443 | – |
| `viz_jpeg_quality` | – | 75 | 75 | – |
| `viz_rate_costmap` | – | 5.5 | 5.5 | – |
| `viz_rate_default` | – | 5.5 | 5.5 | – |
| `viz_rate_image_annotated` | – | 3.3 | 3.3 | – |
| `viz_rate_scan` | – | 5.5 | 5.5 | – |
| `wall_distance_timeout_s` | – | – | 2.0 | – |
| `wall_stop_require_lidar` | – | – | True | – |

### Cherry-picks onto campaign-final: P1-P3 APPLIED 2026-10-13 on Andreas's OK (see section 4b); P4 not applied

`1d00853` is one 861-file commit with run data, so it cannot be cherry-picked whole. Each proposal
applies the `573f313..1d00853` diff limited to the listed paths, as its own commit on campaign-final.
**All of it applies cleanly** (`git apply --check` against campaign-final).

```bash
# P1: vehicle calibration and steering bounds as driven. REQUIRED for a like-for-like test.
git diff 573f313 1d00853 -- \
  src/f1tenth_bringup/config/vesc.yaml \
  src/f1tenth_hardware/f1tenth_hardware/config/steering_calibration.yaml \
  src/f1tenth_params/config/stack_params.yaml \
  src/f1tenth_control/mpc_controller/test/test_steering_limits.py \
  src/f1tenth_control/mpc_controller/test/test_wall_turn_increment.py \
  src/f1tenth_control/mpc_controller/test/test_weight_set.py | git apply --index
# (stack_params here carries the steering bounds plus the inert viz/foxglove keys; supersedes 7e5910e)

# P2: LLM wall-guard default as run. RECOMMENDED (M02/M05-type prompts plan differently without it).
git diff 573f313 1d00853 -- src/f1tenth_intelligence/llm | git apply --index

# P3: campaign bring-up (viz relays, foxglove whitelist, reset_manager), same CPU load as the campaign. RECOMMENDED.
git diff 573f313 1d00853 -- src/f1tenth_bringup ':!src/f1tenth_bringup/config/vesc.yaml' \
  src/f1tenth_logger/f1tenth_logger/mission_logger_node.py \
  src/f1tenth_logger/test/test_viz_topics_not_recorded.py | git apply --index
# + one blank line before "# Snapshotted into the manifest" in mission_logger_node.py (flake8 E305 at the seam)

# P4: offline/tooling (missions json, matlab_export, corridor_perception Step 1, scripts). NOT recommended:
#     missions/go_to_person.json (gap_m 0.5) is rejected under Part E (gap_min 0.53 m from the bumper)
#     and breaks ~27 go_to tests; corridor_perception carries a stale strict xfail.
```

Verified on a **local, unpushed** branch `orin-verify-campaign-dirty`: campaign-final + P1..P4 as 4
commits (70d90d1, 703ad07, 6505420, 3ae2108). It builds, and the rest of its diff to 1d00853 is
81 files, +4238/-293, i.e. Parts B-F.

Tests on that branch vs campaign-final:
- **P1 and P2 add no failures.** `test_steering_limits` passes 8/8 with the campaign gains and bounds.
  `test_object_corridor_mode::test_the_straight_branch_limit_moves_with_the_ramp_span` goes from
  failing to passing.
- **P3** adds:
  - logger flake8 E305 (merge seam, the one-line fix above);
  - `test_foxglove_whitelists::test_topics_hide_the_zed_tree_and_full_rate_images_only` (TypeError,
    the test compiles a yaml *list* as a regex). That test was already wrong in the campaign tree;
    it is a test bug, not a runtime one.
- **P4** adds the go_to_person and corridor_perception failures named above.
- One more failure, `test_start_mission_go_to_object::test_start_with_every_dependency_live_runs`,
  appears with P4 and is not further investigated. The test file comes from 1d00853; it expects
  RUNNING immediately and probably meets Part B's 15 s countdown.

## 4. Build and test (step 4)

All tests: `colcon test` on the 19 workspace packages (`src/f1tenth_external` and the vesc C++
packages skipped), with an isolated `--test-result-base` per run.

| tree | build | tests | failures | skipped |
|---|---|---|---|---|
| baseline `573f313` (`build_base573`/`install_base573`, symlink) | 37 pkgs OK, 4 min 41 s | 2675 | 33 | 27 |
| **campaign-final `5c25ade`** (main build/install) | 37 pkgs OK, 48 s incremental | 2827 | 36 | 27 |
| local verify branch (campaign-final + P1..P4) | 37 pkgs OK | 2859 | 52 + 12 errors | 27 |

- **campaign-final introduces no new failing test.** The 33 baseline failures are the same 33 test
  IDs, in both directions:
  - lint/copyright in bringup/diagnostics/logger/llm/mpc_controller;
  - 12 llm stub `_note_llm_sent` AttributeErrors;
  - 4 stale-expectation tests in mpc_controller (wall_turn_increment x3, weight_set w_du_delta);
  - corridor_direction_recovery;
  - d_wall_correction;
  - model_log header;
  - object_corridor_mode ramp span;
  - go_to_object grace test.
- The 3 extra failures are `f1tenth_messages` cppcheck/cpplint/uncrustify "No files found". That is
  an environment artefact: the package is unchanged 573f313 -> campaign-final, but the main
  `build/f1tenth_messages` was configured 2026-09-07 with a different linter set. The fresh baseline
  configure registers only copyright/lint_cmake/xmllint.
- 152 new tests come with B-F and all pass, including f1tenth_description `test_base_front`.
- **No fix commit was needed. Nothing was pushed to campaign-final.**
- Build note: the first campaign-final build failed on dangling symlinks left in build/ and install/
  by files that the checkout removed (`models/yolo26_office.pt`, `viz_relays.*`,
  `reset_manager.*`, `missions/test.json`). Removing only the broken symlinks
  (`find build install -xtype l -delete`, 12 then 10 links) fixed it.

### 4b. campaign-final after P1-P3 (`e9e5ade`, pushed, fast-forward from 5c25ade, no force)

| commit | content |
|---|---|
| `9b8f1b8` | P1: the campaign's calibration, in one commit: `speed_to_erpm_gain` 4516.9, servo gains -1.0552/-1.1133, `mpc_steering_angle_min/max_rad` -0.309/+0.319 (only the 4 steering hunks of the stack_params diff), and the 3 mpc tests that encode them |
| `32bf769` | P2: LLM wall-guard default (`plan_translate.py`, v2 prompts, legacy prompt, tests and golden) |
| `3f3cfba` | P3: viz relays, foxglove whitelist, reset_manager, supervisor (4 lines), the stack_params `foxglove_*`/`viz_*` keys (hunk 5), and mission_logger's "never record /viz/" |
| `e9e5ade` | flake8 E305 fix at the P3 seam in `mission_logger_node.py` (one blank line) |

Each commit is `git diff 573f313 1d00853 -- <paths> | git apply --index`, so the content is exactly the
campaign tree's. The rebuild is clean (37 packages). `src/f1tenth_bringup/package.xml` has no `--`
inside an XML comment and parses; the package type is still `ros.ament_python`.

**Retest:** 2837 tests, 36 failures, 28 skipped.
- New vs 5c25ade: only
  `test_foxglove_whitelists::test_topics_hide_the_zed_tree_and_full_rate_images_only`.
  - The cause is a TypeError: the test compiles the yaml `foxglove_topic_whitelist` *list* as one
    regex.
  - That test was already broken in the campaign tree (1d00853). It is a test bug, not a runtime
    one: on 10-01 the bridge advertised exactly the whitelisted set.
  - Not fixed: outside the approved commits. The fix is one line, joining the list or looping over it.
- Fixed vs 5c25ade: `test_object_corridor_mode::test_the_straight_branch_limit_moves_with_the_ramp_span`
  (it needed the 0.319 steering bound).

## 5. Stationary bring-up (step 5), 2026-10-13 11:11-11:21, campaign-final `e9e5ade`

**Setup: no path to the motor.**
- The stack was launched the campaign way (`supervisor_bringup.launch.py`, 21 component launches,
  llama-server included). The `hardware` component was replaced by a scratchpad stand-in,
  `orin_standstill/hw_stub.launch.py`. It runs a zero `/odom` at 50 Hz, a do-nothing node named
  `ackermann_to_vesc_node` (`start_mission`'s preflight only checks that the name exists,
  `preflight.py:102-109`) and the identity base_link->imu TF.
- No `vesc_driver` process ran, and no VESC was connected (`/dev/ttyACM*` absent).
- `calibrate_hardware` and `startup_sequence` were removed from the config; both can move the servo.
- A components config without `hardware` is impossible as written: the supervisor auto-starts it
  unconditionally (`_ALWAYS_AUTO_START`) and dies with `KeyError: 'hardware'`
  (`component_supervisor_node.py:674` -> `:963`). That first attempt was torn down before the
  second launch.
- **The lidar was unpowered:** `eno1` NO-CARRIER and `urg_node` "could not open ethernet port
  192.168.0.10:10940". **Every lidar-dependent check below is therefore NOT DONE.**

| check | result |
|---|---|
| **C: debug image** `/camera/debug_image/compressed` | **4.99-5.01 Hz**, JPEG, **21.3 kB mean / 22.1 kB max**, 480 px. Content correct: boxes and masks, wall ROI, `WALL[stereo] 5.25@front raw 5.45@camera`, `CLEAR`/`BLOCKED`, state and stamp. |
| C: cost | Not measurable as 5 vs 0 without restarting perception (`debug_image_rate_hz` is read at init, `yolo_detector_node.py:347`). Proxy: `/camera/detections` **13.57 Hz** with the debug image on vs **12.3-13.5 Hz** in the 10-01 campaign bags, so no drop. `yolo_detector_node` uses 95 % of one core. |
| **C: wall debug** `/perception/debug/front_wall` | ~24 Hz (camera rate), JSON v1. Values are consistent: `stereo_raw_m` 5.456 from the lens, `front_offset_m` 0.333, `front_distance_m` 5.247 from the bumper; ROI 180x70 px; pixel classes add up; `source: stereo`. `lidar: null` because the lidar was off. |
| E: TF (`tf2` lookup from base_link) | `base_front` **0.443, 0, 0**; `laser` 0.12, 0, 0.20, yaw 0; `zed2_camera_link` 0.12, 0, 0.15; `zed2_left_camera_optical_frame` 0.110, 0.060, 0.1655; `imu` 0. All resolve, so front_clearance's new TF dependency is satisfied. |
| E: front distances vs a measured object | **Not done**: no tape values received (the numbers came through blank) and the lidar was off. Camera side only: `/perception/front_distance` 5.23 m median (wall through the glass); `/perception/front_clearance` **0.01** with BLOCKED. That is real: a person sat beside the car's nose inside \|y\| < 0.35 m and behind the bumper face, and Part E clamps such readings to 0.01. `/mpc/min_obstacle_distance(_forward)` 0.28 m (the same person). |
| **B: dummy mission bag** (`orin_standstill`: 20 s hold, `goal_distance` 0) | LOADED 11:17:57, start accepted ("starting in 15.0 s"), RUNNING 11:18:18, COMPLETE 11:18:38. Manifest: `record_start_time` = LOADED edge, `preroll_s` 21.1 (3 s between load and start). **The first bag message came 6.0 s after record start, i.e. 15.2 s before the mission start.** Every continuous topic had its first message more than 5 s before the start; only event topics (`/mpc/hold`, `/mpc/goal_distance`, `/mission/move_outcome`) start at or after it. 39 topics recorded, both debug topics included. Empty: the lidar-dependent topics, plus goal/solver topics a hold never uses. All 448 `/drive` messages were 0/0. Run: `~/f1tenth_archive/complete/2026-10-13T09-17-57_mission-orin_standstill`. |
| **F: MPC with dt 0.2** (`orin_solve_only`: 2 m goal at 0.4 m/s, stopped after 20 s; no VESC process or device, so `/drive` reached nothing) | Startup `CFG \| ts=0.1 ts_pred=0.2 N=20`. **200/200 solved, solve median 21.4 ms, p95 23.3 ms, max 32.5 ms.** Tick interval median 99.9 ms, p95 110 ms, max 117 ms: there is executor jitter, but no solve came near the 100 ms budget. |
| **CPU, 60 s, stack idle** (lidar off) | Per-core: 51.7 / 43.9 / 39.8 / 54.9 / **84.8** / 58.5 / 49.4 / 47.7 / 70.4 / 71.3 / 53.8 / 52.7 %, **mean 56.6 %**. Top processes (% of one core): yolo 94.6, mpc_corr 56.8, zed container 47.7, behavior_executor 46.0, **reset_manager 42.1**, detection_3d 41.2, ekf_cost_observer 31.7, front_clearance 30.5, costmap_boundary 28.9. |
| CPU, 15 s, while solving | mean 60.7 %; mpc_corr 79.5 %; mission_logger 28.9 % while recording. |
| lidar: `/scan`, `/obstacle_clearance`, swept/lidar, `lidar_front_wall`, SLAM, `/costmap/front_clearance`, e-stop front cone | **NOT DONE (lidar unpowered).** |

Observations (none of them changed):
- `reset_manager` uses ~40 % of a core while idle. It ran in the whole campaign (P3), so it does not
  hurt comparability, but it is a lot for a manager node.
- Readings beside the car's body but behind the bumper face clamp to 0.01 and report BLOCKED
  (Part E design). Expect a mission `front_clearance` stop to fire immediately when someone stands
  next to the nose at start.

## 6. Where base_link is (from the code, not the docstring)

- **Odometry: base_link is the rear axle.** `vesc_to_odom.cpp:300` uses omega = v*tan(delta)/L
  (L = 0.305, `vesc.yaml:207`). The filter's predict step integrates `x += v*cos(theta)*dt`,
  `y += v*sin(theta)*dt` (`vesc_to_odom.cpp:130-131`), with no slip angle. That is the kinematic
  bicycle referenced at the rear axle: only the rear-axle point moves along the heading. The EKF
  takes this /odom as `odom0` (`ekf.yaml:56`), and its `odom -> base_link` TF is what everything uses.
- The URDF agrees: `core.xacro` puts the rear wheels at x 0 and the steering hinges at x 0.325.
- **The MPC model assumes a different point:** `vehicle_model.py:28` uses
  beta = atan(lr/L * tan(delta)) with `lr = 0.17` (`MPC_corr.py:856`), i.e. a centre-of-gravity
  reference 0.17 m ahead of the rear axle. Yet it is fed the rear-axle pose. At full steer
  (0.319 rad) beta is about 10 degrees.
  - This mismatch was already in the campaign, so it does not affect comparability.
  - Not changed; flagged for later.
- The `description.launch.py` docstring ("base_link is centered between the axles") is wrong.
- Wheelbase: 0.305 in odometry and the MPC, 0.325 in the URDF. The tape value "rear axle -> bumper"
  checks `base_front_x` directly.
- **Tape comparison: pending.** The four values in the message arrived blank ("__ m"). Expected,
  if the URDF/TF is right:

  | | URDF/TF says |
  |---|---|
  | rear axle centre -> bumper face | 0.443 |
  | lidar centre -> bumper face | 0.443 - 0.12 = 0.323 |
  | ZED lens -> bumper face | 0.333 (left optical frame x 0.110; base_front - 0.110) |
  | test object -> bumper face | compare with `/perception/front_distance`, the lidar forward beam minus 0.323, `/perception/front_clearance`, `/costmap/front_clearance`, `/mpc/min_obstacle_distance_forward` |

  Anything off by more than 2 cm is reported before anything is changed, as asked.

## DECISIONS TAKEN ALONE

1. **Snapshot without switching branches.** The snapshot commit was built from a temporary index
   (`read-tree` + `commit-tree`). The Orin's working tree, index and branch were never changed, so
   no switch back was needed.
2. **vesc submodule untracked files.** The 2 CSVs (`vesc_tuning/speed_sweep_results`) were copied
   into the superproject snapshot under `orin_snapshot_20261013/vesc_submodule_untracked/`
   instead of committing inside the submodule, which would mean pushing to sjckness/vesc.
   `__pycache__` excluded.
3. **Pushed over SSH** (`git@github.com:sjckness/f1tenth_more.git`) because the Orin's https origin
   has no push credentials. Only `orin-snapshot-20261013` was pushed.
4. **Baseline built with `--symlink-install`**, the workspace default from `~/.colcon/defaults.yaml`, so
   it is comparable with the main build. Its tests ran while 573f313 was checked out.
   `build_base573/` and `install_base573/` are left on disk, untracked.
5. **Test scope:** the 19 workspace packages; the `src/f1tenth_external` submodules and the
   upstream vesc C++ packages were left out of both runs.
6. **Removed broken symlinks** in build/ and install/ (only `-xtype l`, 22 in total over two
   checkouts) to fix the symlink-install trap after branch switches. No other file under build/ or
   install/ was touched.
7. **Built and tested the cherry-pick proposal on a local branch `orin-verify-campaign-dirty`**
   (never pushed) so the GO/NO-GO rests on evidence. campaign-final was then checked out and rebuilt
   again. The car's install is campaign-final 5c25ade as pushed.
8. **Local branch `campaign-final`** created to track origin/campaign-final, for building and for
   any later fix commits.
9. Did not pull the 3 new linus commits on `test-campaign` (`349c881`) into the Orin's local branch.
10. **P1 stack_params split by hunk.** P1 carries the 4 steering-related hunks; P3 carries the
    foxglove/viz hunk. That way P1 is exactly "one consistent calibration" and P3 holds its own keys.
11. **Hardware stand-in for the bring-up** instead of removing `hardware` (the supervisor cannot run
    without it, see section 5). It lives only in the scratchpad (an `AMENT_PREFIX_PATH` overlay);
    nothing was added to the repo.
12. **Ran a solving mission (`orin_solve_only`)** for the MPC tick. A hold mission does not solve.
    No VESC driver process ran and no VESC device was connected, so the non-zero `/drive` (max
    0.15 m/s) could not reach the motor.
13. Both dummy runs are in the real archive (`~/f1tenth_archive/complete/2026-10-13T09-17-57_mission-orin_standstill`,
    `...09-19-48_mission-orin_solve_only`), named clearly; not deleted.
14. Stack torn down after the measurements (SIGINT to every launch, then SIGTERM for the top-level
    launch and discovery server); nothing left running. Lidar checks need a relaunch with the lidar
    powered.
15. Did not fix the foxglove whitelist test (outside the approved commits).

## GO / NO-GO for today's test (updated 11:30)

**CONDITIONAL GO on campaign-final `e9e5ade`.** It becomes a GO when both of these are done on the stand:
1. **Lidar powered and checked:** `/scan` ~40 Hz, `/obstacle_clearance`, lidar swept clearance,
   `lidar_front_wall`, SLAM pose. The e-stop front cone (IsProximityTooClose) runs on `/scan` and
   was re-referenced to the bumper by Part E. It is the safety layer and was not exercised today.
2. **Tape check:** the bumper offset and the sensor offsets agree with the URDF/TF to within 2 cm
   (section 6), and every front-distance topic agrees with the measured object distance.

Why the earlier NO-GO is lifted:
- campaign-final now carries the campaign's erpm gain, servo gains and MPC steering bounds (P1),
  the LLM default (P2) and the bring-up (P3). It builds, and the only new test failure is a test bug.
- Part B: the bag covers the whole mission on the car. Discovery took 6.0 s against the 15 s countdown.
- Part C: 5.0 Hz, 21 kB/frame, no detection-rate drop vs the campaign.
- Part F: dt 0.2, N 20, solve 21 ms median / 33 ms max against the 100 ms budget.
- Part E: the TF chain resolves on the car with the configured numbers.

Known differences from the campaign:
- The intended ones: B-F.
- The MPC's lr = 0.17 vs the rear-axle odometry, unchanged from the campaign.
- Today's measurements had the lidar off, so CPU under full load is ~5-10 % higher than measured.
