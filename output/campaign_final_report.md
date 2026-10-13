# campaign-final report (2026-10-13)

`campaign-final` = `573f313` (the code the car ran for the last campaign runs) plus one commit per part:

| order | commit | part |
|---|---|---|
| 1 | `a6dd23e` | D: trapezoid corridor, fixed 2.3/1.3 ratio, start half-width 0.64 m |
| 2 | `407ff6d` | B: record from mission LOADED, 15 s countdown as pre-roll |
| 3 | `10d3eb3` | E: `base_front` frame (x = 0.443 m), every front distance measured from the bumper |
| 4 | `0334b8c` | C: 5 Hz debug image, wall debug topic, host MP4 tool |
| 5 | `30b0580` | F: MPC prediction dt 0.2 s, N = 20 (loop stays 10 Hz) |
| - | - | G: no camera-pan code exists at 573f313, so nothing changed |
| - | this commit | report |

`git diff 573f313..campaign-final --stat`: 83 code/test/config files, +4242 / -295, plus this report and its appendices under `output/`. Nothing outside Parts B–F.

Appendices (same folder):
- `campaign_final_runs_table.md`: per-run commit and dirty flag (Part A).
- `campaign_final_dirty_diff.md`: `573f313..1d00853` stat, the `stack_params.yaml` table, and a per-file campaign/post-campaign classification.
- `campaign_final_virtual_wall_review.md`: Part H review (read-only).

## STEP 0 and git state

- **Snapshot:** the linus working tree as found is on its own local branch `snapshot/pre-campaign-final-20261012` = `8f345dd`.
  - Committing on `jazzy` was blocked by the permission classifier, so the snapshot is on its own branch.
  - `jazzy` was not touched.
  - Pushed to origin on 2026-10-13.
  - Submodule content can't be captured by a superproject commit and is still on disk untouched: zed wrappers emptied, 4 `husarion_gz_worlds` sdf edits.
- **Evidence branch (pushed):** `campaign-dirty-evidence` = `7e5910e`.
  - One commit on `573f313` that sets `stack_params.yaml` to the `1d00853` file. That file is byte-identical to every 2026-10-01 params snapshot.
  - This is an evidence-based guess. Cherry-pick it only if the stack was restarted each test day.
  - It will conflict with D/C/F in `stack_params.yaml` if cherry-picked onto `campaign-final`. Resolve by keeping both sides.
- **Push:** `campaign-final` pushed to origin on 2026-10-13, after `git ls-remote` showed no `origin/campaign-final` (no force).

## Part A: campaign commit

- **Every run is dirty.** All 57 runs with a manifest say `branch: test-logging`, `dirty: true`. P005-R001 has no manifest. Six commits were used over the campaign (table in the appendix).
- **Base: `573f313b1ed502a34584ea9cf89b09baa17837e4`** ("test: campaing: m01 with obstacle done", 2026-09-30 18:25).
  - It was used by P004-R012..R019 (2026-10-01 11:16–11:34 CEST), the last runs.
  - P004-R019 is the last run (aborted, empty bag); R018 is the last complete one.
  - It is on `origin/test-logging`.
- **Dirty flag: TRUE, no diff captured anywhere.**
  - The closest evidence is `1d00853` ("7/10 tests", 2026-10-07, single parent 573f313). It looks like a commit of the Orin tree.
  - Its `stack_params.yaml` equals every 2026-10-01 params snapshot.
  - Bags show the steering command saturating at exactly +0.319/−0.309, the `1d00853` bounds, not the 0.278/−0.283 at `573f313`.
  - The servo gains read from the bags (−1.0552/−1.1133) are those of `1d00853`'s `steering_calibration.yaml`.
  - **So at least the steering bounds and steering calibration on the car differed from `573f313`.** See the dirty-diff appendix and compare against the Orin tree (checklist step 1).
- **Caveat:** the `.mat` `meta.git_commit` is the export-time HEAD, not the run commit. Ignore it.

## Part B: recorder pre-roll (`407ff6d`)

- **Problem at `573f313`:** the bag started at RUNNING with a new recorder per run.
  - 17 of 58 bags are empty.
  - Non-empty bags start a median 8.2 s late (4.9–13.7 s); median coverage is 42 %.
- **Fix (the simple option):**
  - The logger starts recording at **LOADED**.
  - `mission_countdown_sec` default **3.0 → 15.0 s**, which is longer than the worst measured late start (13.7 s).
  - A readiness wait was not used: Humble's `rosbag2_py.Recorder` cannot report subscriptions, and 6 of 37 topics never publish.
- **Manifest:** `start_time` still means the RUNNING edge (the exporter match is unchanged). New fields `record_start_time`, `mission_start_time`, `preroll_s`.
- **Loaded timeout:** a mission left LOADED for 120 s closes as `NOT_STARTED`.
- **Recorded topic set:** unchanged by B.
- **Cost:** every mission now stands still 15 s. To change it live: `ros2 param set /behavior_executor_node mission_countdown_sec <s>`.

## Part C: debug video (`0334b8c`)

- **`/camera/debug_image/compressed`** (`sensor_msgs/CompressedImage`, JPEG, 480×341, about 16 KiB):
  - Drawn in `yolo_detector_node` only on 5 Hz ticks; no new image subscription.
  - Shows box, class, confidence, track ID and distance from the bumper.
  - Sub-threshold detections are drawn grey down to 0.1, with the threshold printed.
  - Also shows the wall ROI, wall distance and source, objects overlapping the ROI, front clearance, mission state and stamp.
  - On tick frames YOLO runs at conf 0.1 and the result is split at the threshold, so published detections are unchanged.
- **Params:** `debug_image_rate_hz` 5.0 (0 = off), `debug_image_width` 480, `debug_image_jpeg_quality` 50, `debug_image_min_confidence` 0.1.
- **`/perception/debug/front_wall`** (`std_msgs/String` JSON v1, one per depth frame, about 770 B):
  - raw stereo (lens) and bumper-referenced wall distance, ROI, pixel counts, objects in the ROI, latest LiDAR wall fit;
  - `source` is "stereo" and `fused_m` is null, because no fusion exists in the campaign stack.
  - Logging only.
- Both topics are added to the mission logger's recorded set.
- **Cost:**
  - 1.44 ms per frame on the i7-9700K.
  - Estimated about 5 % of one Orin core at 5 Hz, 8 % at 10 Hz.
  - The extra GPU cost of the 0.1-confidence tick frames is unmeasured: check it on the car (step 5).
- **Host MP4:** `PYTHONPATH=src/f1tenth_logger python3 -m f1tenth_logger.mission_debug_video <run_dir> --out-dir /tmp/v` (needs ffmpeg).

## Part D: corridor (`a6dd23e`)

- **Why it looked parallel:** the walls were Bezier curves whose end width was scaled down with corridor length, so short corridors had nearly equal ends.
- **New shape:** walls are a straight trapezoid, `corr_half_width_start_m` **0.64** → end 0.64·2.3/1.3 = **1.1323 m** for every length.
  - `corr_width_scale_with_length` (default false) restores the old scaling.
  - Adaptive widening near obstacles does not exist at 573f313, so no switch was added.
- **Sizing:** a person on the centreline 1 m ahead of the bumper sits at s = 0.443 + 1.0 = 1.443 m.
  - Needed half-width there: 0.25 person + 0.10 perception tolerance + 0.12 safety margin + 2·0.20 car radius = **0.870 m**.
  - With the 0.64 m start the corridor gives 0.877 m there. The old 0.4333 gave 0.594 m.
  - Your rough 0.64 / 1.14 is confirmed; 1.1323 keeps the ratio exact.
- **Feasibility:** a 1 m reaction is kinematically too late to keep all margins (details in the design notes). The corridor now has room for the pass; it does not make a 1 m reaction possible.
- **Constraint:** `corr_L_base` stays 3.0. Above about 3.09 m the trapezoid at s = 1.443 drops below 0.870 m.

## Part E: front reference (`10d3eb3`), port of jazzy `30fa1c7`, same numbers

- **One source:** `base_front` link at x = 0.443 m (`urdf/base_front.xacro`), read by `f1tenth_description.vehicle_frames.base_front_x()`.
- **Clamp:** readings at or behind the bumper publish 0.01 m.
- **Thresholds:** values are kept and now mean distance from the bumper, so safety stops move outward.

| consumer | old reference | new | threshold | bumper gap at stop, old → new |
|---|---|---|---|---|
| e-stop front cone | LiDAR origin | bumper | 0.15 | −0.17 (only after contact) → +0.15 |
| mission `front_clearance` stop | ZED lens | bumper | mission X | X − 0.333 → X |
| `wall_turn` trigger | lens, then bumper mid-turn | bumper | unchanged | trigger − 0.333 → trigger |
| `/perception/front_clearance` | lens / base_link | bumper | 0.5 / 0.7 | 0.17 / 0.06 → 0.5 |
| `/costmap/front_clearance` | base_link | bumper | – | value − 0.443 |
| steering-calibration abort | base_link | bumper | 0.9 | 0.457 → 0.9 |
| `obstacle_distance_below` (forward) | base_link | bumper | X | X − 0.443 → X |
| `IsObstacleDetected` (off by default) | camera | bumper | 1.0 | 0.667 → 1.0 |
| go_to gap | solver nose (0.5525) | bumper | default 0.5 → 0.6 | 0.6095 → 0.6 (**0.0095 m closer**, as in 30fa1c7; not a safety stop) |
| `front_depth_monitor` (not launched) | lens | bumper | – | – |

- **Not ported:** `30fa1c7` hunks for files absent at 573f313 were dropped.
- **TF dependency:** `front_clearance_node` gains a `/tf_static` buffer. If the camera TF doesn't resolve, `front_distance` stays −1 and `front_clearance` missions run to their timeout.

## Part F: MPC prediction dt (`30b0580`)

- **New param:** `mpc_prediction_dt_s` (yaml 0.2, launch arg for rollback). `0.1` reproduces the previous traces exactly.
- **Audit outcome:**
  - Model and rollout use dt.
  - The first rate row uses the 0.1 s control period, later rows dt.
  - Rate cost of stages k ≥ 1 is scaled by (Tc/dt)².
  - Warm start is interpolated at k + Tc/dt.
  - Lookahead floor is 2.5 m.
  - The wall_turn / object heading caps intentionally stay on the control period.
  - Obstacles are static, so obstacle prediction needed no change.
  - The horizon distance (2.0 m at 0.5 m/s) is less than `corr_L_base` 3.0.
- **Solve time (host):** median 5.25 ms vs 5.21 ms before, same QP size. The Orin estimate is about 25 ms median / 35 ms p95 vs the 100 ms budget.
- **Closed loop (nominal model):**
  - no corridor violations;
  - wall braking starts about 1 m earlier, and the car stops 0.55 m short of the wall instead of 0.38 m;
  - more steering reversals (0.45 vs 0.15 per m) at lower steering rates.
- **Expected oddities on the car:**
  - A `TGT/geometry ... Raise corr_L_base to at least 5.00` warning every 5 s. **Ignore it:** raising `corr_L_base` breaks Part D.
  - The replay video still labels the horizon "2.0 s".

## Part G: camera pan

There is no pan code, param, launch arg or URDF joint at 573f313. The pan only arrived on jazzy from 2026-10-08. Nothing was added.

## Part H: virtual wall (read-only)

**Verdict: the "virtual wall" as described does not exist; nothing was changed.**
- `/perception/lidar_front_wall_virtual` (on 573f313 since 09-14) uses LiDAR + odometry only, has no detection exclusion, and nothing consumes it.
- The real post-campaign fix is A4 `wall_confirm.py` on `test-campaign` (10-09/10-12). It is **sound with fixes**:
  - it can confirm two people standing side by side as a wall;
  - it can fail open near the stop distance;
  - it publishes the lagged camera value instead of the LiDAR intercept.

Full review in the appendix.

## Verification done here (Jazzy host, no Humble)

- py_compile on all touched files, and Python 3.10 grammar for C. YAML parses. The xacro expands; `check_urdf` passes with `base_front` at 0.443.
- **Lint:** flake8/pep257 counts on every touched file are equal to or below base; new files are 0/0.
- **Tests** (Jazzy rclpy from a scratch prefix; before → after, failure sets identical, the failures are environment-only):

| package | before | after |
|---|---|---|
| mpc_controller | 691 passed / 16 failed | 731 passed / 10 failed |
| f1tenth_params | 43 passed | 54 passed |
| perception | 540 passed | 604 passed |
| costmap | 190 passed | 193 passed |
| behavior | 343 passed / 1 failed | 352 passed / 1 failed |
| logger | 245 passed / 22 failed | 258 passed / 20 failed |
| f1tenth_control | – | 7 passed |
| description | – | 5 passed |

The mpc_controller "before" number was taken on the bare base; the 10 vs 16 failures differ only by environment (osqp/stubs).

- **Not verifiable without Humble or the car:**
  - Humble rclpy/tf2 behaviour.
  - Whether 15 s pre-roll suffices on the Orin.
  - The camera TF resolving on the car.
  - YOLO GPU cost at conf 0.1 on tick frames.
  - Real solve times and driving behaviour with the 4 s horizon and wider corridor.

## Orin morning checklist

```bash
cd ~/dev_ws/f1tenth_more
# 1. Is the tree clean? If dirty, save it on its own branch first.
git status --porcelain=v1 -uall; git stash list; git branch -vv | head; git log --oneline -3
git reflog --date=iso -n 50 | grep -E '2026-10-0[1-8]'      # expect 10-07 "commit: 7/10 tests"
# if dirty:
git switch -c orin-snapshot-20261013 && git add -A && git commit -m "Orin working tree as found 2026-10-13"
git push -u origin orin-snapshot-20261013
git diff 573f313b1ed502a34584ea9cf89b09baa17837e4 orin-snapshot-20261013 --stat
git diff 573f313 orin-snapshot-20261013 -- src/f1tenth_params/config/stack_params.yaml \
  src/f1tenth_hardware/f1tenth_hardware/config/steering_calibration.yaml src/f1tenth_bringup/config/vesc.yaml

# 2a. Clean tree: take campaign-final as is
git fetch origin && git switch -c campaign-final --track origin/campaign-final
# 2b. Dirty tree you want to keep: cherry-pick the parts onto it, IN THIS ORDER
#     (E needs D's test file, C needs E's helpers, F's yaml hunk sits next to C's)
git cherry-pick a6dd23e 407ff6d 10d3eb3 0334b8c 30b0580    # D B E C F
#     stack_params.yaml conflicts against a dirty yaml: keep both sides.
#     Leave F out of today's build: drop 30b0580 from the list.
# Optional evidence commit (only if the stack was restarted each test day):
#     git cherry-pick 7e5910e   (stack_params.yaml conflicts likely; keep both sides)

# 3. Build (every touched package; f1tenth_description must be rebuilt or imports fail)
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select f1tenth_description f1tenth_params \
  f1tenth_behavior f1tenth_logger f1tenth_perception f1tenth_costmap mpc_controller \
  f1tenth_control llm f1tenth_diagnostics
source install/setup.bash

# 4. Tests
colcon test --packages-select f1tenth_description f1tenth_params f1tenth_behavior \
  f1tenth_logger f1tenth_perception f1tenth_costmap mpc_controller f1tenth_control llm
colcon test-result --verbose | tail -40

# 5. Stationary bring-up (start the stack as for the campaign), then:
ros2 topic hz /camera/debug_image/compressed          # ~5 Hz
ros2 topic echo --once /perception/debug/front_wall   # JSON, "stereo_origin":"front", front_offset_m ~0.33
ros2 param get /mpc_corr corr_half_width_start_m      # 0.64
ros2 param get /behavior_executor_node mission_countdown_sec   # 15.0
#    logs: "base_link <- zed2_left_camera_optical_frame: x=0.110" (front_clearance),
#          mpc_corr startup line shows ts_pred 0.2, CORR halfWidth=[0.640..1.132]
#    cost: compare `ros2 topic hz /camera/detections` and `top -H -p <yolo pid>` with
#          debug_image_rate_hz:=0 vs 5; if detections drop > 0.5 Hz set debug_image_min_confidence:=0.3
# 6. Load a short mission: logger prints `recording "<run_id>"` at LOADED, ~15 s before STARTED,
#    then "mission started ... recording for 15.0s". After the run:
#    manifest preroll_s ~15; first bag message before start_time; both debug topics in `ros2 bag info`.
#    Host video: PYTHONPATH=src/f1tenth_logger python3 -m f1tenth_logger.mission_debug_video <run_dir> --out-dir /tmp/v
```

**Rollbacks:**
- F: `mpc_prediction_dt_s` 0.1.
- C: `debug_image_rate_hz:=0`.
- B: set the countdown back to 3 (late bags return).
