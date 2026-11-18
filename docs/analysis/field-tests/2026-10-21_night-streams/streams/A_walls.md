## Stream A: walls (`night-walls`, worktree `~/dev_ws/f1tenth_more_walls`, head 120d0f4)

Six commits on `night-start/campaign-final` (6002cb0):

| commit | what | part |
|---|---|---|
| `4ecea87` | multi_wall_tracker: one wall confirmation level (`wall_confirm_*`) | A1 |
| `0063509` | mpc_corr: the deceleration ramp aims at the locked front wall | A2 |
| `53587fe` | output/front_wall_lock: design.md for the ramp and the confirmation level; 15° marked unmeasured | design |
| `6671431` | review fixes: confirmation distance jitter measured at the wall; replay tools get `wall_confirm_*` | fix |
| `480fb6c` | review fixes: every ramp-source transition is logged | fix |
| `120d0f4` | review fixes: measured confirmation latency, replay wording | fix |

Nothing was pushed, no sudo was used, and no other branch or worktree was touched.

### For tomorrow first
1. **A confirmed wall that is not locked is still an MPC constraint**, independent of virtual_wall and of the lock:
   - The tracker's `published()` emits it on `/perception/walls`.
   - `MPC_corr.wall_tracks_callback` passes it to `wb.build_wall_rows`.
   - `wall_boundaries.admit_track` (wall_boundaries.py:190) admits front/left/right walls that are not stereo-only, not TENTATIVE (:212) and not coasting longer than 1.0 s.
   - The rows reach the solver as `boundaries=live_boundaries`.

   Existing limits, unchanged: 'back' walls only become disks, 'unknown' walls are never published, and at most 4 tracks are published.
2. **Pre-existing bug found tonight, not fixed (out of scope):** `max_sigma_rho_m` (0.3 m, multi_wall_tracker.py:469) tests the sigma of the **odom** rho.
   - That sigma grows with the along-wall distance from the odom origin, so a wall about 20 m along its own direction from where the EKF started is deleted at birth. It is never FRONT and never an MPC row. The baseline does the same.
   - Shown in a synthetic test: one observation from (0, 20) gives no lasting track. In a 90° turn scene at (20, 0) there is no FRONT on either code; with the cap raised there is FRONT on cycle 4.
   - It only matters on missions that go more than ~20 m from the start (corridor10m stays within 16 m).
3. **FRONT comes later on real data** (A1 spec value, see Risks).

### A1: one confirmation level (4ecea87 + fix 6671431)
There is one promotion rule, "THE CONFIRMATION LEVEL" in multi_wall_tracker.py:
- parameters in `TrackerParams` (:439);
- the rule in `confirmation_reason` (:1119) and `line_jitter` (:1094), plus `track_midpoint` (:1145);
- the tracker applies it in `confirm_reason` (:1683) and `_promote` (:1697).

| parameter | value | meaning |
|---|---|---|
| wall_confirm_min_consecutive_hits | 5 | LiDAR-backed (LIDAR or BOTH) associations **in a row**; the birth cycle is hit 1 |
| wall_confirm_min_length_m | 1.0 | the track's measured extent |
| wall_confirm_max_dist_jitter_m | 0.05 | max − min of each observation's perpendicular distance **from the track's segment midpoint** (odom, ego motion compensated) |
| wall_confirm_max_angle_jitter_deg | 3.0 | max − min of the direction |
| wall_confirm_require_lidar | true | stereo-only tracks never confirm |

Removed rules (old name → old value → replacement):

| old | value | replaced by |
|---|---|---|
| `TrackerParams.confirm_hits` (a code default, never a launch parameter) | 3 | `wall_confirm_min_consecutive_hits` 5, now consecutive |
| `TrackerParams.confirm_window` | 5 | no window any more |
| `lidar_front_wall_stereo_confirm_hits` / `_window` | 5 / 7 | `wall_confirm_require_lidar` |
| `lidar_front_wall_camera_confirm_fresh_only` | true | no parameter: always on when camera hits count |
| tentative expiry "5 scans hold fewer than 3 hits" | | a tentative track with no association on a cycle is deleted |

**Hysteresis:** `ever_confirmed` is sticky, and `_merge` sets the merged track's state on the same cycle.

**Consumers.** All of them read this level and none has its own count:
- FRONT and virtual_wall;
- `published` and `/perception/walls`;
- strip and negative evidence, occlusion;
- door hosts (door_detection.py:492/565/876);
- wall markers;
- the BT lock (wall_lock.py);
- the MPC rows (wall_boundaries.py:212);
- the semantic layer (semantic_layer_node.py:668).

corridor_perception_c only extracts lines; it has no track lifecycle.

**Review fix (blocker, confirmed):** the jitter was the spread of odom rho, measured from the odom ORIGIN. A 0.5° line swing read as 8.7 cm 10 m along the wall, so whether a wall confirmed depended on where the EKF started.
- Now the spread is measured at the track's segment midpoint, where the fitted line pivots. `ref_xy` is a required argument, so no caller can fall back to the origin by omission.
- Also fixed: `tools/scene_labels_replay.py` never passed the unprefixed `wall_confirm_*` group to the replayed node (`UNPREFIXED` :115).

### A2: the ramp aims at the locked wall (0063509 + fix 480fb6c)
- `CheckStopCondition` publishes `wall_lock.ramp_wall_id(step)` on **`/mpc/stop_wall_id` (std_msgs/Int32)**. The BT's decisions are unchanged.
- MPC_corr subscribes to it and to `/perception/virtual_wall/debug` only when `wall_lock_enabled` is true (:1988).
- `_wall_stop_d_rem` reads `_ramp_wall_distance` (:3381), which uses the pure `ramp_wall_source.select_ramp_distance`.

| case | ramp aims at |
|---|---|
| wall_lock_enabled false | `_fresh_front_distance()`, exactly as before; nothing subscribed |
| not locked yet (-1) | raw wall distance |
| locked, its wall FRONT | the payload's `distance_m` |
| locked, another wall or none FRONT (swerve) | no wall ramp; other walls stay with the wall rows, avoidance, stop+reverse and the 20 cm LiDAR stop |
| locked wall lost (-1) | raw, until re-acquired (conservative) |
| stale id (> 0.5 s), new stop_target, unusable payload | raw |

Other A2 changes:
- mission_logger records `/mpc/stop_wall_id`.
- `wall_follow_gate_m` 0.30 and `wall_follow_gate_deg` 15 are unchanged; 15° is marked **NOT MEASURED** in design.md.

**Review fix:** the "RAMP | aiming at ..." transition line was throttled, so a transition within 2 s of the previous one was lost for good. It is now unthrottled (edge-triggered, :3406).

### Verified (linus, ROS_DOMAIN_ID 71, localhost; failures judged against `night-start/campaign-final`)
- **test_wall_confirm_level.py: 33 passed** (+4 from the fix). The same observations confirm on cycle N from three translated/rotated odom poses; with the origin as reference those tests fail.
- **Perception and params:**
  - pytest: perception 1262 passed + the 2 known env-only TestPlacement failures.
  - colcon test: perception 1264 / 2 env-only; params 80 / 0.
- **test_ramp_wall_source.py: 43 passed.** The new log test fails on the throttled code. Earlier A2 runs: 6/6 mutants caught; end to end with the real `FrontWallLock` and `MPCController`.
- **mpc_controller ramp/stop/wall files:** 255 passed + the 3 known `test_wall_turn_increment` failures. In A2's whole-suite run, all 21 failures also fail on the baseline: 0 new.
- **f1tenth_behavior:** 523 passed (**py_trees_ros stub run, not colcon test**; not touched by the fixes).
- **Logger and tools:** logger 271 passed + 5 that also fail on the baseline; tools tests 27 passed, 5 skipped.
- **Replay (corridor10m_20261019T131815, LiDAR-only re-track, 796 cycles):**

| | baseline | before fix | after fix |
|---|---|---|---|
| first FRONT after driving starts | 20.4 m ahead, 0.15 m driven | 15.7 m, 5.0 m driven | 15.7 m, 5.0 m driven |
| FRONT held, % of cycles | 82 | 50 | 51 |
| confirmed right wall within 2.5 m, % of cycles | 34 | 13 | 27 |
| birth → confirm, p90 cycles | 3 | 16 | 4 |

- **wall_lock_replay "0 differ":** this holds by construction. It only shows the bags hold no swerve; the swerve case is covered by unit tests only.
- **Not verified:** Humble or the Orin (all changed .py files parse as Python 3.10), the car, and any swerve recording.

### Risks for the car test
- **FRONT later for far walls.** About 4.5 m closer than before on corridor10m, and mostly absent while standing. Short or intermittent side walls give fewer MPC rows. Watch `confirm_reason` (`hits`, `dist_jitter`) on `/perception/virtual_wall/debug` with `wall_debug_tracks:=true`.
- **Odometry against the 5 cm bound.** /odometry/filtered reads ~17 % short. At 1.0 m/s a head-on wall never confirmed in simulation. If `confirm_reason` says `dist_jitter`, raise `wall_confirm_max_dist_jitter_m`.
- **Nearer surface while the lock holds.** The ramp does not slow for a surface in front of the locked wall until the lock is lost.

**Rebuild on the car:** f1tenth_perception, f1tenth_params, mpc_controller, f1tenth_behavior, f1tenth_logger.