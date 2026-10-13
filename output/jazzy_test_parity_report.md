# Jazzy test parity: overnight run 2026-10-12/13 (Thor, `jazzy`)

Covers the corrected step list (steps 0 to 7). The run stopped at the session
limit in the middle of step 3 (2026-10-12 23:33) and was resumed and finished on
2026-10-13.

## Commits (all pushed to `origin/jazzy`)

| Step | Commit | What |
|---|---|---|
| snapshot | `f639ea62` | snapshot before jazzy test parity (2026-10-12) |
| 1 | `27f84d37` | `.claude/settings.json` folded into the git-ignored `settings.local.json` |
| 2 | `cb5735ff` | corridor funnel with a fixed end/start ratio; adaptive widening, default off |
| 3 | `30fa1c79` | `base_front` URDF frame; every front distance measured from it |
| 4 | branch `campaign-final` | created from `origin/test-campaign` at `349c881e` (it did not exist) |
| 5 | none | sim stack left alone; every test ran on `ROS_DOMAIN_ID=87`, localhost discovery, no discovery server |
| 6 | `740ccaec` | `camera_pan_enabled` (default false) as a real parameter |
| 7 | `9e21bebb` | sub-threshold detections drawn grey on `/camera/image_annotated` |

## Test diff vs baseline

| | tests | failures | skipped |
|---|---|---|---|
| baseline (before step 1) | 3231 | 107 | 106 |
| after step 7 | 3289 | 107 | 106 |

The failure set is identical except for one live-LLM eval: `test_go_to_prompt[vai verso il portatile]` passed and `[avvicinati al divano]` failed. These evals call the real model and vary between runs; the translator code they exercise was not touched in a way that affects them.

flake8 / pep257 counts are at or below the baseline in every package (mpc_controller flake8 1222 → 1221, f1tenth_behavior 455/530 → 454/529, llm flake8 166 → 165, all others equal).

### Pre-existing failures (baseline, not caused by this run)

- **ackermann_mux (40), zed_components (8), vesc_driver (4), vesc_ackermann (4), zed_debug (3), zed_msgs (1), joy_teleop (2)**: third-party lint (cpplint, uncrustify, flake8 quotes, copyright, lint_cmake). joy_teleop also has `test_no_buttons_or_axes`.
- **f1tenth_behavior (15)**: `test_mission_countdown` (11) and `test_mission_loader_hold_release` (3): the fake nodes have no `get_parameter`. One more countdown test asserts `'starting in 7.5 s'` but gets a preflight refusal.
- **f1tenth_bringup (6)**: lint, plus `test_foxglove_whitelists` (`unhashable type: 'list'`). `test_sim_mode` has two failures: it expects `mounts['zed2_camera_link']`, and the camera model is `zed2i`, not `zed2`.
- **mpc_controller (3)**: lint, plus `test_abort_holds_the_car` (`_input_status_last_sec` missing on the stand-in).
- **llm (6)**: lint, plus four live-model evals.
- **f1tenth_diagnostics (3)**: lint and copyright.

## Step 2: corridor

The campaign's A2 shipped `corr_wmin = corr_wmax = 0.60`, so every corridor was a rectangle. Now:

- The far end is `corr_wmin × corr_width_ratio`, with the ratio fixed at 2.3 / 1.3 (the original funnel's shape). `corr_wmax` is derived and is no longer a parameter.
- `corr_wmin` went from 0.60 to 0.65. The design case is a person on the centreline, 1 m ahead of the car front: r_person 0.25 + perception lateral tolerance 0.08 + car_radius 0.20 + safety margin 0.12 = 0.65 m. The far end is 1.15 m. At the person's station the funnel is 0.884 m, which still holds the whole car disk (0.85 m).
- Adaptive widening (`corridor_adaptive_enabled`, default **false**) rebuilds once with a wider start, capped at 0.90 m, when an obstacle's pass side needs it.
- `test_wide_corridor.py` asserts the start/end ratio at the default params.
- Closed loop: R011 and "1.0 m object seen from start" now pass instead of ending BLOCKED. "1.0 m object appearing at 1.4 m" still ends BLOCKED, with adaptive on or off.

## Step 3: `base_front` and the front-clearance table

`urdf/base_front.xacro` defines link `base_front`, fixed to `base_link` at x = 0.443 m. `roboracer.urdf.xacro` includes it (car and sim), and robot_state_publisher puts it on `/tf_static`. Python without TF reads the same file through `f1tenth_description.vehicle_frames.base_front_x()`. A test ties the parser to the expanded URDF.

Retired:
- the `swept_clearance_body_front_x_m` key;
- MPC_corr's `wall_bumper_x_m` parameter;
- every literal 0.443 in code: `BodyBox` default, `campaign_scenario`, the tools, launch files and node defaults.

A real reading at or behind the bumper is published as 0.01 m. It is never published as ≤ 0, which every consumer reads as "no reading" and which would switch the stop off.

Lens x in base_link is about 0.11 m on the car (`camera.launch.py` mount 0.12, plus the ZED's internal offset) and 0.35 m in the sim URDF. The code reads it from `/tf_static`, never from a literal. LiDAR is at 0.12 m on the car (`description.launch.py`).

| Consumer | Old reference | New reference | Threshold (kept) | Physical stopping point |
|---|---|---|---|---|
| Mission `front_clearance` stop and wall_turn (`/perception/front_distance`, front_clearance_node) | camera lens | bumper | mission values, e.g. 2.0 m | about 0.33 m further out on the car (0.09 m in sim) |
| `front_clearance` obstacle term (same node) | base_link | bumper | same | 0.443 m further out |
| Proximity e-stop front cone (IsProximityTooClose) | raw LiDAR range | bumper (laser pose from `/tf_static`) | 0.15 m | was 0.17 m **inside** the front face (after contact), now 0.15 m ahead of the bumper: 0.32 m further out. With no laser TF yet the laser is assumed at base_link, which can only trip early. |
| `obstacle_distance_below` (`/mpc/min_obstacle_distance`, `_forward`) | rear axle | bumper | e.g. 1.0 m | 0.443 m further out for obstacles ahead. On the omnidirectional topic, an obstacle beside or behind the car now reads farther. |
| `/costmap/front_clearance` (steering-calibration abort, LiDAR wall seed) | base_link | bumper (seed adds it back) | 0.4 m abort | 0.443 m further out (margin only grew) |
| IsObstacleDetected (off by default) | camera x | bumper | 1.0 m | further out |
| go_to_object gap (loader, LLM translator, handler, scoring, watch_objects) | solver nose (0.5525 m) | bumper (0.443 m) | gap_min adds the nose's 0.1095 m lead | **gap_min: unchanged** (+0.5 mm rounding). **Default gap 0.5 → 0.6:** 9.5 mm closer. **Explicit `gap_m` (e.g. 1.0 in `go_to_person_floor`): 0.11 m closer**, as the instruction "mission distances now mean from the bumper" implies. Still ≥ gap_min. |
| Swept / obstacle / wall-distance bodies, MPC body box, wall-tracker bumper | 0.443 literal or stack key | `base_front` | n/a | unchanged (same number, one source) |

The LLM golden fixtures were regenerated for the new gaps (default 0.6, clamp 0.53). The Italian clamp note now names the bumper and includes the nose term, so its terms add up to the applied gap.

## Step 6: camera pan

`camera_pan_enabled: false` in stack_params is the default for:
- a read-only `enabled` parameter on both pan nodes;
- `camera_pan.launch.py`'s `enabled` argument;
- the `camera_pan_enabled` arguments of `sim_camera_tf.launch.py` and `supervisor_bringup.launch.py`, forwarded by the supervisor like the other pan arguments.

When disabled, the controller creates no `/camera_pan/command` publisher and no timer. The TF node latches `base_link → camera_pan_base → zed2_camera_link` on `/tf_static` at pan 0 and does not subscribe to `/camera_pan/joint_state`.

Verified live on domain 87:
- no `/camera_pan/command` topic;
- `/tf_static` carries both edges;
- `tf2_echo base_link zed2_camera_link` gives translation (0.12, 0, 0.15) and the identity rotation.

There is no pan driver or serial port on this branch, so "the port is never opened" holds trivially. The car's `camera.launch.py` publishes its own static camera TF and never included the pan; the launch docstring that claimed otherwise is corrected.

## Step 7: grey sub-threshold boxes

- Inference runs at `debug_confidence_threshold` (new key, default 0.10) when it is below `confidence_threshold` (0.3). The result is split on the scores right after inference.
- Everything published (detections, the mask label image, coloured boxes/masks) comes from the part ≥ 0.3, exactly as before.
- The part below is only drawn: grey, with class and score, plus a corner label `conf >= 0.30 (grey: 0.10-0.30)`. Grey boxes get the LiDAR-exclusion box test.
- `debug_confidence_threshold >= confidence_threshold` switches the feature off.

Real-inference check (yolo26s-seg.pt and yolo26s.pt on the Ultralytics bus/zidane images, CUDA on the Thor): published detections and mask images are identical with the feature on and off.

Cost was measured A/B, interleaved in one process:
- +0 to +3.5 ms per frame, about +2 ms median on frames that have grey boxes, out of 50–60 ms per callback;
- the model itself costs +0.6 to 1.2 ms at conf 0.10;
- the first version split after the LiDAR exclusion and cost +8 ms, because the exclusion pulled the sub-threshold masks to the CPU. It now splits first.

## Decisions taken alone

1. **Default go_to gap.** I accepted the new default of 0.6 m (gap_min from the bumper, rounded up to 0.1 m). It sits 9.5 mm closer than the old 0.5 m from the nose, while gap_min, the safety floor, is physically unchanged. I did not re-tune it.
2. **Explicit mission gaps keep their values** and now mean bumper-to-edge, per the instruction. This puts them 0.11 m closer physically than before (table above).
3. **Gap closed-loop rig.** `test_object_gap_closed_loop.py` now models the bumper reference, since it simulates what GoToObject sends. Every case still passes.
4. **Disabled pan.** The TF node still runs and latches pan 0 statically, rather than not starting. That keeps one owner of `base_link → zed2_camera_link` in sim, and makes the chain resolvable by the `/tf_static`-only readers.
5. **`campaign-final`** was created from the `test-campaign` head (`349c881e`, 2026-10-12 15:35) as instructed. No campaign-final report existed, so nothing in this run was reconciled against one.

## Not done, and open issues

- **Before enabling the pan:** `front_clearance_node` resolves the camera pose from `/tf_static` only. With `camera_pan_enabled: true` the camera edge is dynamic, so after step 3 `/perception/front_distance` would publish -1 ("no reading") on every frame. That means no mission wall stop and no wall_turn distance. The same limitation already applied to the LiDAR wall confirmation. It needs a stamped `/tf` lookup at the depth image time.
- Not in the corrected list, so not done: the first prompt's 5 Hz CompressedImage debug image with full overlays, the wall-estimator debug topic, logger topics/MP4, and the TF audit with bag numbers.
- The sim URDF puts the LiDAR at x = 0.40, while the car TF says 0.12 (backlog B10). The proximity cone uses whichever TF is live.
- Nothing was run on the car or in Gazebo. Every check above is unit or closed-loop tests plus the two live/real-inference checks named.
