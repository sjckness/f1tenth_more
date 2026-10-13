## Part H: Virtual wall review (read-only)

**Ref reviewed:** `test-campaign` @ `349c881edadadb76b4860a46336695667bfc29b0`. All code was read through `git show` and `git grep`. Every virtual-wall file is byte-identical on `573f313` (the campaign commit), `jazzy` 49de45d and `snapshot/pre-campaign-final-20261012` 8f345dd.

### 1. Where it lives and what it does

**The task describes something that does not exist as one component.** No ref has an estimator written after the campaign that fuses LiDAR, stereo and odometry and leaves detected people or objects out of the wall fit. The search covered:

- all heads, remotes and tags;
- `ls-remote`;
- the reflog;
- `fsck` for unreachable commits and blobs;
- the worktree indexes.

`git log --all -S virtual_wall` and `-G 'virtual.?wall'` found nothing. Wall estimation is spread across three pieces instead:

| Piece | Where | When | Inputs | Who uses it |
|---|---|---|---|---|
| **Virtual wall** (`/perception/lidar_front_wall_virtual`, `WallEstimator`) | `src/f1tenth_perception/f1tenth_perception/lidar_front_wall.py` (pure logic), `lidar_front_wall_node.py` (ROS glue), `launch/lidar_front_wall.launch.py`, `f1tenth_messages/msg/WallEstimate.msg` + `WallLineFit.msg` | Added in `822c210` on 2026-09-14, **before the campaign**, never changed since. It ran during the campaign. | `/scan`, `/odometry/filtered`, `/costmap/front_clearance` (seed), `/tf_static`. **No stereo, no detections.** | **Nobody.** The docstring says "Nothing consumes either topic yet" (`test-campaign:src/f1tenth_perception/f1tenth_perception/lidar_front_wall_node.py:16-17`). The only subscriber is a lazy viz relay (`viz_relays.yaml:84-87`). |
| **A4 wall confirmation** (the actual post-campaign fix for R009) | `src/f1tenth_perception/f1tenth_perception/wall_confirm.py`, running inside `front_clearance_node.py` | `6b4953f` (2026-10-09), latch added in `6cae729` (2026-10-12). **Not on `573f313`.** | ZED stereo `front_distance` (YOLO objects already masked out) checked against a LiDAR line fit. **No odometry.** | The mission wall stop, through `/perception/front_wall_confirmed`, when `wall_stop_require_lidar: true` (`behavior_executor_node.py:529-536`, `:676-677`) |
| MPC `WallTracker` | `mpc_controller/wall_tracker.py` | `4c35975`, before the campaign | LiDAR line picked inside a window around the camera distance, then dead-reckoned on odometry | `wall_turn` increment (`MPC_corr.py:1112-1145`) |

**How the virtual wall works.** Everything runs per `/scan` at about 40 Hz:

- **Fit.** It takes the returns within ±10° of the heading, 81 beams on the 0.25° Hokuyo, and projects them into `base_link` (`lidar_front_wall.py:179-194`). It fits a line by MSAC over deterministic point pairs plus two total-least-squares refits (`:104-162`).
- **Single-scan checks.** At least 41 inliers, an inlier fraction of at least 0.6, and a normal angle of at most 45° (`:203-221`).
- **Prediction from odometry.** `d = d0 − (p − p0)·n_world` (`:321-337`). It needs a pose no older than 0.2 s and an anchor no older than 3 s.
- **Wrong-surface gate.** A fit more than 0.30 m from the prediction is rejected (`:346-358`).
- **Output priority** (`:360-390`): MEASURED, which snaps to the fit and re-anchors; then DEAD_RECKONED; then SEEDED (the SLAM-costmap cone value); then NONE.

**How A4 works.** Per depth frame, about 15-17 Hz:

- It needs a scan received within the last 0.3 s (`front_clearance_node.py:674-689`).
- It takes the LiDAR points within ±30° and within ±0.6 m of `d_cam` (`wall_confirm.py:195-203`) and fits a line.
- The line must have at least 10 inliers and a normal within 45°. Its extent must be at least 0.8 m, or a relaxed value while latched. It must cross the forward axis, and its intercept must lie within 0.30 + 0.10·d of the camera (`:210-241`).
- A latch holds for 0.5 s (`:244-269`).
- When confirmed, the node publishes the **camera's smoothed distance**; otherwise it publishes −1.0 (`front_clearance_node.py:998-1001`).

### 2. Verdict

**Not ready** as the "virtual wall" the task describes. The component with that name has no detection-based exclusion; its own docstring says the YOLO veto "is not built" (`lidar_front_wall_node.py:179-180`). It accepts a flat object as the wall whenever it has no prior fit to test against, and nothing consumes it. The real R009 fix (A4) does fix the single-person geometry for the mission stop. It is **sound with fixes** at best: it can still confirm two people as a wall, it can fail open near the stop distance, and it publishes the lagged camera value instead of the LiDAR intercept.

### 3. Analysis

#### Object exclusion

**Virtual wall: exclusion is implicit only.**

- The dominant-line rule leaves a minority object out of the fit.
- The 41-inlier and 0.6-fraction floors reject sectors made of legs or a torso.
- The 0.30 m gate rejects a surface that suddenly appears nearer than the predicted wall.

That gate exists only while an anchor with a pose is less than 3 s old (`lidar_front_wall.py:329-333`). The documented blind spots (`lidar_front_wall_node.py:169-177`) are:

- glass;
- a flat face within 0.3 m of the prediction;
- any surface that appears when there is no prediction: at startup, after age-out, while seeded, or while odometry is stale.

**A4: exclusion has two layers.**

1. Stereo pixels are masked using the YOLO label mask, or filled bounding boxes when the mask stamp does not match (`front_clearance_node.py:695-822`).
2. The LiDAR must agree with the camera, and the line must be at least 0.8 m long. A single person (about 0.27-0.37 m in the probes) fails that test.

The extent is measured as `s.max() − s.min()` with no gap rule (`wall_confirm.py:216-219`). `wall_tracker.py:135-142` already uses a longest-run rule with a 0.25 m maximum gap for exactly this reason.

#### Frames and front reference

| Signal | Reference point | Citation |
|---|---|---|
| Virtual wall `distance` | Perpendicular distance from the `base_link` origin to the fitted line; `normal_angle` 0 = ahead, positive = left | `WallEstimate.msg:11`, `lidar_front_wall.py:14-15`, `:198-200` |
| `/perception/front_distance`, `/perception/front_wall_confirmed` | From the camera lens, along the optical axis | `wall_confirm.py:185-187` |
| Conversion inside A4 | `d_cam = camera_x + front_distance`; `camera_x` comes from `/tf_static` | `wall_confirm.py:195`, `front_clearance_node.py:676`, `:689` |
| Mission stop comparison | The lens-referenced value | `condition_eval.py:372-374` |

The laser pose comes from `/tf_static`, not a hard-coded value. On the car it is (0.12, 0, yaw 0) (`description.launch.py:186-193`). Only yaw is used; roll and pitch are ignored (`lidar_front_wall_node.py:313-314`).

**Sim mismatch.** The sim laser on `573f313` and `test-campaign` faces backwards: `rpy="0 0 ${pi}"` at `sensors.xacro:42`, with a ±135° field of view at `:58-59`. The forward sector is therefore empty in sim. The snapshot fixed this in `cc3addd` (`snapshot/...:sensors.xacro:89`, `rpy="0 0 0"`).

#### Latency

- **Virtual wall.** All timing uses receipt time (`lidar_front_wall_node.py:291-292`); the scan stamp is only copied into the output header. The scan is paired with the latest pose received within 0.2 s, with no interpolation (`lidar_front_wall.py:303-306`, `:374-379`). The pure fit cost 0.72 ms per scan on x86 (measured), yet the Orin measurement showed 25.3 % CPU with 0 subscribers (`docs/topic_optimization/3_reconciliation.md:72`). No control path has any latency from it, because nothing uses it.
- **A4 stop path.** The chain is:
  1. depth frame;
  2. EMA with alpha 0.25 (`front_clearance_node.py:976-977`), which lags about 3 frames, roughly 0.18 s;
  3. LiDAR gate, using a scan up to 0.3 s old (`:680-682`);
  4. `Float32` with no stamp, stored on the blackboard with no receipt time (`check_stop_condition.py:418-419`);
  5. the 10 Hz behaviour-tree tick (`behavior_executor_node.py:257`).

  Probes at 0.5 m/s with a 0.5 m stop: an ideal camera fires at a true 0.50 m; with the EMA at 0.40 m; with the EMA plus an assumed 100 ms depth latency at 0.333 m; at 1.0 m/s with the same lags at 0.20 m.
- **MPC `wall_turn`.** It reads the raw `/perception/front_distance` with a 0.5 s age gate (`MPC_corr.py:1765-1770`, `:3052-3070`). It never sees the LiDAR confirmation.

#### Failure modes

| Condition | Virtual wall (`lidar_front_wall`) | A4 (`front_wall_confirmed`, the mission stop) | Consequence |
|---|---|---|---|
| Wall not visible (nothing in range, or outside the sector or window) | `too_few_returns`. It dead-reckons for up to 3 s, then uses the seed, then NONE (`lidar_front_wall.py:381-390`). | `too_few_returns`, publishes −1 (`wall_confirm.py:199-202`) | The stop never fires. The move is bounded only by `timeout_sec` and the proximity e-stop. |
| One person in the ROI, wall behind | Legs give `low_inlier_count`; a torso gives `low_inlier_fraction`. Rejected **if** a prior fit exists. | Not confirmed (`segment_too_short`, extent about 0.27-0.37) | **R009 fixed for the mission stop.** |
| Person present at startup or after age-out | A flat face becomes MEASURED. Torso probe with a 3.9 m seed: SEEDED, then **MEASURED 1.25 m**. | Not affected (one person still fails the extent test) | The virtual wall reports the person as the wall. No consumer today. |
| Two people, or a person plus a post, at the same depth | Not tested | **CONFIRMED** (extent 1.17-1.24 m) | **The stop fires at the people.** R009 again, through A4. |
| Person within about 0.3 + 0.1·d in front of a real wall | Rejected by the gate if a fit exists | CONFIRMED, publishing the person-biased camera value | Stops early, by at most about 0.35 m at a 0.5 m stop (the safe direction). |
| Glass or dark wall | Blind (documented, `lidar_front_wall_node.py:171`) | −1 forever, with no diagnostic | The stop never fires. Unverified on the real Hokuyo (`6b4953f`, `6cae729` messages). |
| Wall first seen nearer than about 0.69 m | — | Never confirmed (`wall_confirm.py:24-39`) | Only the e-stop is left. |
| Stale odometry (> 0.2 s) | Gate off; a fit is accepted and re-anchored **without a pose** (`:374-379`) | Not used | See issue 10. |
| Stale scan (> 0.3 s) or depth stall | — | `no_scan`, or no frame at all; the latch is not refreshed | Latch expires after 0.5 s and the stop fails open (issue 2). |
| Stereo dropout (fewer than 60 background pixels) | — | The EMA **holds** with no age limit (`front_clearance_node.py:290-292`), and the held value can still be confirmed | Late stop, or −1 (issue 3). |
| Detector miss or stall | No detection input at all | The detection cache never ages (`front_clearance_node.py:632-636`) | Raw `front_distance` leaks; `wall_turn` is exposed (issues 4 and 5). |
| `front_clearance_node` dies | — | The blackboard keeps the last value forever (`check_stop_condition.py:274`, `:418-419`) | A stale stop or a stale non-stop (issue 9). |
| Sim (on `573f313` or `test-campaign`) | Sector empty, so NONE or SEEDED | −1 forever | No wall stop ever fires in sim (issue 6). |

#### Tests

**Run on linus.** The pure modules were extracted from `test-campaign`. `test_lidar_front_wall.py` (45 tests) and `test_wall_confirm.py` (24 tests): **69 passed**. `f1tenth_perception/setup.py:39-41` declares `extras_require` test, so colcon does run these suites.

**Not run** (they need rclpy or launch_ros), reviewed by reading:

- `test_lidar_front_wall_node.py`
- `test_lidar_front_wall_launch.py`
- `f1tenth_bringup/test/test_lidar_front_wall_component.py`
- `f1tenth_behavior/test/test_wall_distance_source.py`

**Gaps:**

- No two-person or person-plus-clutter test for A4.
- No person-shaped scan run end to end through the estimator.
- No test of a fit accepted during an odometry gap.
- No test of the latch across dropped frames.
- Zero tests of the `front_clearance_node` A4 glue (`git grep` finds `wall_confirm` only in `test_wall_confirm.py`).
- Only identity quaternions in the node tests.
- No test pins the gate at its 0.30 m boundary.
- `REASON_NOT_AHEAD` is never asserted.

### 4. Ranked issues

Everything below was confirmed by the verifiers and by re-reading the code at `test-campaign`, unless it is marked **uncertain**. Paths are relative to `test-campaign:src/`.

| # | Sev | Location | Problem | Failure scenario | Suggested fix |
|---|---|---|---|---|---|
| 1 | **major** | `f1tenth_perception/f1tenth_perception/wall_confirm.py:216-219`, `:233` | The extent is `s.max() − s.min()` across gaps, and the crossing test uses the same min and max. Collinear separate objects therefore pass as a 0.8 m "wall", even with a free gap on the car's axis. | Two people side by side at ±0.45 m, 1.5 m ahead, 4 m wall behind, camera leaking: CONFIRMED, extent 1.17-1.24 m. A person plus a 3 cm post: CONFIRMED, 0.85-0.89 m. With the 3.0 m before-turn default (`stack_params.yaml:2459-2460`) the car stops at once, at the people. `test_wall_confirm.py:90-94` covers only one person. | Use the longest contiguous run with a gap of at most 0.25 m (as `wall_tracker.py:135-142` does), and require that run to contain the axis crossing. Add two-person and person-plus-post tests. |
| 2 | **major** | `wall_confirm.py:258-268`, `:224-232`; `front_clearance_node.py:680-682`, `:938-943` | The latch is the only way to confirm below about 0.69 m. It is refreshed only on a CONFIRMED frame and expires after 0.5 s wall-clock. Stale-scan frames, too-close frames (which return before the tracker runs) and depth stalls do not refresh it. | Approach at 0.5 m/s; the depth stream stalls for 0.55-0.6 s, starting at a camera reading of 0.75 m. Afterwards, re-confirmation needs a 0.8 m extent, which is geometrically impossible that close: `segment_too_short` on every frame, and the stop **never fires** (probe; a 0.40 s gap still fires). Bounded by the e-stop and `timeout_sec`. | Keep the latch across frames where no confirmation was attempted, or time it on odometry travel. Add a dropped-frame `_approach` test. |
| 3 | **major** | `front_clearance_node.py:290-292`, `:965-977`, `:998-1001`; `wall_confirm.py:221-222` | The value published when confirmed is the camera EMA, which holds with no age limit when stereo drops out. The fresh LiDAR intercept `x_int` is computed and then thrown away. | A plain wall where stereo drops out below about 0.76 m: probe shows `pub=0.76 ok` down to a true 0.35 m, then −1. A 0.5 m stop never fires. Even without dropout, the EMA lag alone makes the stop late (true 0.40 m at 0.5 m/s, 0.20 m at 1 m/s with an assumed 100 ms depth latency). This is inherited from before A4, but A4 had the LiDAR value available. | When confirmed, publish `x_int − camera_x` (or the minimum of that and the camera value). Age-gate the held EMA. |
| 4 | **major** | `front_clearance_node.py:632-636`, `:712-717`, `:821` | The detection and mask caches have no receipt time and are never compared with the depth stamp. A stalled detector leaves its last footprint frozen at full confidence. | YOLO hangs; a person then steps in 1.5 m ahead. `front_distance` leaks toward 1.5-2.6 m (`output/campaign_fix_plan.md:100-127`), and `wall_turn` commits at the person. Obstacles already have an age gate (`:874`); detections do not. | Stamp the caches on receipt and expire them after about 2× the detector period. While stale, flag `front_distance` as degraded or publish −1. Compare detection stamps with the depth stamp. |
| 5 | **major** (known, B9) | `f1tenth_control/mpc_controller/mpc_controller/MPC_corr.py:1765-1770`, `:3052-3070`; `output/campaign_partB_backlog.md:35-37` | `wall_turn` still commits on the raw camera `/perception/front_distance`. Neither A4 nor the virtual wall is on that path; `6cae729`'s commit message says A4 does not protect it. | A person 1.5 m ahead of a 4 m wall during a `wall_turn` move: the turn commits against a wall that is not there. | Gate the commit on `/perception/front_wall_confirmed` with the same freshness check (B9). |
| 6 | **major** (for sim validation) | `f1tenth_description/urdf/sensors.xacro:42` (same on `573f313`) | The sim laser has yaw π with a ±135° field of view, so neither the ±10° nor the ±30° forward sector contains any beam. | Thor sim validation of `campaign-final`: `front_wall_confirmed` is always −1, every wall stop drives past, and a sim run would wrongly suggest that A4 blocks real walls. | Cherry-pick the `sensors.xacro` part of `cc3addd` (front-facing laser) for sim runs. |
| 7 | minor | `wall_confirm.py:236`; `front_clearance_node.py:999-1001` | Agreement within 0.30 + 0.10·d, combined with publishing the camera value, means an undetected person standing in front of a real wall is published as "confirmed". | Person 0.3 m in front of the end wall, YOLO misses them: probe published a stop at 0.48 m with the real wall at 0.78 m. Early, which is the safe direction; the error is bounded by the tolerance. | Same fix as issue 3. Also reject when there are LiDAR returns on the axis nearer than `x_int`. |
| 8 | minor | `wall_confirm.py:24-39`, `:199-202`; `f1tenth_behavior/f1tenth_behavior/mission/condition_eval.py:372-373` | A LiDAR-invisible wall (glass, dark), or one first seen nearer than about 0.69 m, gives −1 forever with no "camera sees a wall, LiDAR never confirms" diagnostic. **Uncertain:** whether the camera-only stop would have fired on glass before A4. | A wall move toward a glass partition runs until `timeout_sec`. | Publish a diagnostic for "unconfirmed for more than N s". Consider a debounced reduced-speed fallback. |
| 9 | minor | `f1tenth_behavior/f1tenth_behavior/behaviours/check_stop_condition.py:274`, `:418-419`, `:466-482` | The wall-stop value has no receipt time and is not reset on a move change. | `front_clearance_node` crashes after a stop at 0.45: the next 0.5 m-stop move ends on its first tick. If the last value was −1, the stop is disabled for the whole move. Needs a crash or stall. | Store the receipt time and treat the value as None after 0.3-0.5 s. Reset it on a move change. |
| 10 | minor | `f1tenth_perception/f1tenth_perception/lidar_front_wall.py:374-379`, `:330` | A fit accepted while odometry is stale re-anchors with `pose=None`. That throws away the good posed anchor and disables the gate until the next accepted fit with fresh odometry, which is often one scan, about 25 ms. | Probe: with an odometry gap, a 1.0 m object is MEASURED. Without the gap it is `wrong_surface`. No consumer today; untested (`test_lidar_front_wall.py:319-323`, `:385-389` stop just before this case). | Never overwrite a posed anchor with an unposed one. Add the sequence test. |
| 11 | minor | `lidar_front_wall.py:329-333`, `:346-358`, `:362-383` | An object present at startup, after age-out, or while seeded is adopted as MEASURED. After it leaves, the real wall is rejected as `wrong_surface` and the object's distance is dead-reckoned for 3 s. | Board at 1.5 m for 1 s, wall at 4 m: MEASURED 1.5, then DEAD_RECKONED 1.5 until t = 4 s. Would reproduce R009 if the topic were ever wired to control. | Build the YOLO veto before any consumer uses the topic. Do not accept a first fit without corroboration. Re-acquire when the innovation stays stable for N scans. |
| 12 | minor | `f1tenth_perception/launch/lidar_front_wall.launch.py:34-37`, `:83`; `lidar_front_wall_node.py:82-83` | The node uses 25.3 % CPU with no consumer (`3_reconciliation.md:72`). Core 5 is claimed to be free, but `obstacle_clearance`, `swept_clearance` and `wall_distance` all default to core 5 (`git grep`). The docstring's 5 % CPU budget is stale. Any harm to the other core-5 nodes is inferred, not measured. | Load on an Orin running at 85-93 % CPU for no benefit. | Proposal P5a (`4_proposal.md:279-289`): stop auto-starting the node until something consumes it, and fix the docstrings. |
| 13 | minor | `lidar_front_wall.py:334-337`, `:381-383` | A dead-reckoned estimate has no sign check and no oblique check: it reports negative distances, and normals beyond 45° (after a turn), as `valid=True`. | Probes: −0.5 m valid; a wall at −90° valid. Matters only for a future consumer. | Invalidate the prediction when `d ≤ 0` or the normal exceeds `oblique_max`. |
| 14 | minor | `f1tenth_perception/test/test_front_clearance_node.py` (no A4 coverage); `front_clearance_node.py:486-515`, `:645-689` | The A4 glue is untested: the 0.3 s scan gate, the TF lookups, the −1 output, the wiring from ROS parameters to `WallConfirmConfig`, and `no_camera_transform` (which publishes −1 on every frame and silently disables the stop). | A broken parameter wiring, an inverted age check or a missing camera frame would leave all tests green. | Extend the mocked-node tests with confirmed, −1, `no_scan` and `no_camera_transform` cases, plus a parameter-wiring case. |
| 15 | minor | `f1tenth_perception/test/test_lidar_front_wall.py:208-437`; `test_lidar_front_wall_node.py:78-112` | No person-shaped scan runs end to end through fit, classify and estimator. The node tests use only identity TF and odometry quaternions. | A sign error in the yaw path, or the startup blind spot, goes unnoticed. | Add scan-level sequence tests, marked as known blind spots, and yawed TF and odometry node tests. |
| 16 | nit | `lidar_front_wall.py:303-306`, `:374-379` | The scan is paired with whatever pose is freshest within 0.2 s, with no interpolation, and stamped with the scan stamp. At the 0.2 s limit that is up to 0.1 m at 0.5 m/s, a third of the gate. | Spurious `wrong_surface` at speed. Nit while nothing consumes the topic. | Keep an odometry ring buffer and interpolate to the scan stamp. |
| 17 | nit | `lidar_front_wall_node.py:306-308`; `front_clearance_node.py:657-659` | Static laser and camera poses are cached forever after the first lookup. | A recalibrated `/tf_static` mid-session is ignored until restart. | Invalidate the cache in the `tf_static` callback. |
| 18 | nit | `lidar_front_wall_node.py:313-314`; `front_clearance_node.py:664-667` | Only yaw is taken from the quaternions; roll and pitch are ignored. An inverted Hokuyo (roll π) would mirror the bearings. | Hypothetical after a remount (`description.launch.py:53-55` calls the level mount an assumption). | Use the full rotation, or log an error when roll or pitch is not about 0. |
| 19 | nit | `f1tenth_description/urdf/base.xacro:5` vs `f1tenth_description/launch/description.launch.py:35`; `f1tenth_params/config/stack_params.yaml:598` | The `base_link` origin is ambiguous: rear axle or mid-axle. The EKF does not fuse `vy`, so its point behaves like the rear axle. | About 0.11 m of dead-reckoning error over a half-turn, if the topic were consumed. | Settle `base_link` by measurement. |
| 20 | nit | `wall_confirm.py:175-176`, `:261` | The latch normal comparison has no angle wrap, which is safe only while `max_normal_angle ≤ 45°`. The latch drops on a backwards time step (bag replay or sim clock reset). | Edge cases only. | Wrap the angle difference; handle `dt < 0`. |
| 21 | nit | `f1tenth_perception/test/test_lidar_front_wall_launch.py:96`; `test_lidar_front_wall.py:193`, `:385-389`; `test_wall_confirm.py:130-133` | Test names overstate what they check (the CLAUDE.md pattern): "off the behavior executor" is not checked; "filling the sector" covers ±7.1°; `test_off_while_odometry_is_stale` hides that an object is adopted. The gate's 0.30 m boundary and `REASON_NOT_AHEAD` are never asserted. | Regressions at the boundary or in the crossing check stay green. | Rename the tests and add boundary and not-ahead tests. |

Stale comments at `test-campaign`: `front_clearance_node.py:54-57`, `:72-77` and `check_stop_condition.py:19-29` still say the stop reads `/perception/front_distance`, and `front_clearance_node.py:1252-1264` still calls LiDAR fusion future work. `wall_confirm.py:41-45` explains why A4 does not use `/perception/lidar_front_wall`.

Not raised as issues: one finding said `no_camera_transform` could persist because `/tf_static` was published before the node started. That is **uncertain and probably wrong**, because the subscription is TRANSIENT_LOCAL with depth 100 (`front_clearance_node.py:597-603`), so a late joiner still receives it. The lack of a test for that path is kept under issue 14.

### 5. Changes

Nothing was changed, committed, merged or pushed in `/home/andreas/dev_ws/f1tenth_more` during this review. Code was read only through git refs. Probes ran on files extracted into the `partH` scratch area, which has been deleted.