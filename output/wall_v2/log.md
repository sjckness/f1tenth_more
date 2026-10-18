# Wall tracker v2 — log

Orin, `~/dev_ws/f1tenth_more`, branch campaign-final. Base HEAD `0c1cd417`.
Times are local (Europe/Rome), 2026-10-18.

| time | what |
|---|---|
| 13:46 | Premises checked: HEAD contains `0c1cd417` and `e9ab0a04`; `enable_wall_distance` default false; `ekf_cost_observer` absent from `src/` and `scripts/` (only `test_ekf_global_config.py::test_the_ekf_cost_observer_is_gone` names it); `ekf_frequency_hz` 20.0. All four hold. |
| 13:48 | `git tag backup/pre-wall-v2-0c1cd417` created (local only). |
| 13:50 | Main bag identified: `~/bags/corridor10m_20261016T170931` (newest, 860 MB, 325.6 s, 159 topics). |
| 13:52 | **No depth in the main bag.** `/zed2/zed_node/depth/depth_registered` has no messages at all (only the `/ffmpeg` and `/compressed` shells, 0 each); `/perception/scene_labels/depth_mm` 0; `/camera/detection_masks` 5; `/perception/scene_labels` 3; `/perception/stereo_columns` 0; `/perception/wall_points` 0. |
| 13:55 | **No movement in the main bag.** `/odometry/filtered`, `/odom` and `/ekf_global/odometry/filtered` all report 0.00 m of path and `twist.linear` identically 0 over 318 s. No movement interval exists, so the analysis window rule cannot be applied. |
| 13:58 | Attempt 2: `/sensors/core` ERPM max 0.0; `/commands/motor/speed`, `/drive`, `/ackermann_drive` all exactly 0.000 on every message; `/mission/status` never leaves IDLE. |
| 14:01 | Attempt 3 (independent sensors): ZED visual odometry moves 6-9 cm total (jitter), and the forward ±10° median laser range is **19.41-19.44 m for all 325 s**. The car did not move. (`/sensors/core.state.distance_traveled` does count up — 9807 ticks — which contradicts ERPM 0; logged as an open point, it does not change the verdict.) |
| 14:02 | Control: the same script on `corridor10m_20261016T144537` finds two movements (58.57-70.92 s, 125.94-150.60 s) and the window 56.57-152.60 s. The script is right; the bag is stationary. |
| 14:05 | Decision D1 (decisions.md): P0 is run on `corridor10m_20261016T144537` with window **56.57-152.60 s**, which is where the stated regression lives and which satisfies the window rule exactly. The main bag is kept as a second, stationary far-wall probe. |
| 14:10 | Baseline replay at HEAD on the window → `output/wall_v2/baseline/b144537/`. |
| 14:15 | Only two cpu-cleanup commits touch the tracker: `9786bfc` (R2 block 2) and `d578963` (R2 block 3). Bisect reduced to four points. |
| 14:25 | front_reason probe over the window: **`low_confidence` 249 / 374 cycles (67 %)**, `no_candidate` 102, `front` 11 — with `strip_range_m` 21.16 m and the car reporting 21.19 m on the same cycles. The wall is seen and then refused. |
| 14:35 | Confidence-vs-range table measured (report.md): 71 % of observations inside 1.5 m reach the 0.5 gate, 1 % at 12-18 m, 0 % past 18 m, and the 12-18 m band's rms is the best in the bag. |
| 14:40 | Probe of t=146-150 s: the phantom's candidate tracks are refused `off_strip` / `role_other` at confidence 0.51-0.78, never `low_confidence`. The phantom defence and the confidence gate are separable. |
| 14:50 | Fix written: range-aware references in `wall_fusion.confidence`, three new stack_params keys, launch wiring. |
| 15:05 | First replay with the fix: WALL 69.0 s, agreement 0.808, err p95 0.004 m, NO_DATA 0, 147-150 s phantom still gone. |
| 15:20 | `test_wall_confidence_range.py` written; two failures exposed a real weakness (a 3-return sliver at 60 m scored 0.51) → the stretch is now refused below `wall_conf_min_inlier_ref`. 14/14 green. |
| 15:30 | Bisect complete (table in report.md). Culprit: `d578963` alone. `9786bfc` is byte-identical to pre-merge. |
| 15:40 | Build clean; 1097 tests over f1tenth_perception / f1tenth_params / f1tenth_bringup, 3 failures, all pre-existing lint in f1tenth_bringup. Commit `6cbda121` (P0). |
| 15:45 | Workspace-wide search for a bag with depth: 305 `.db3` files under `~/bags` and `~/f1tenth_archive`; 7 have depth, all archive mission bags, best 335 frames (`active/2026-10-14T12-05-34_mission-llm_2b356aac445f`). None is a corridor bag. |
| 15:50 | **P1 GATE: cannot be evaluated.** See decisions.md D3. Stopping before P2, as P1 instructs. |
| 16:00 | Main bag replayed before and after the P0 fix as a stationary far-wall probe (`output/wall_v2/main_bag_prefix/`, `main_bag_p0fix/`). |
| 16:20 | "go on": P1 re-pointed at the depth-bearing archive bags (decisions.md D5). |
| 16:25 | The RECORDED `/perception/scene_labels` is 160x90 and has **zero wall and zero floor_inferred** pixels in all 483 label frames of all five depth bags. Unusable as the foot's mask. |
| 16:35 | The RECOMPUTED mask (`MaskReplay`, front_clearance_node over the raw depth) is 640x360 and 45.9 % wall / 34.4 % floor_inferred. P1 is runnable after all. |
| 16:40 | `tools/wall_foot_eval.py` written: foot per column (vectorised), odom-carried accumulation, virtual scan -> C extraction, stereo_old, merged, radial filter, LiDAR reference, metrics, topdown + mask frames. |
| 17:05 | Smoke run: the foot forms, but **no foot segment ever matches a LiDAR line**. Diagnosed rather than tuned. |
| 17:15 | Point-level residual added (nearest LiDAR return per camera point): foot 0.342 m median at 1.5-3 m against stereo's 0.067 m. |
| 17:25 | Cause 1 found: the foot range histogram spikes at 2.75-3.25 m (39 % of points) and is EMPTY between 0.75 and 2.50 m -- it is tracing the inferred-floor disc edge (`inferred_floor_r_max_m` 3.0), not a wall base. Confirmed visually in `camera_00.png`. |
| 17:30 | Cause 2 quantified: h = 0.2200 m, fy = 262.44 px, so at 3 m the foot is 19.2 rows below the horizon and one pixel is 15.6 cm. The gate's 10 cm median is below one pixel at the top of its own bucket. |
| 17:40 | Second witness: the moving bag (126 frames, 4.16 m) exercises the accumulation. Best foot residual 0.187 m at 1.5-3 m; coverage 0.8 %/2.4 % against stereo_old's 8.7 %/3.2 %. |
| 17:50 | The radial filter, measured on stereo_old: 65-76 % of segments dropped, spurious/frame 1.069 -> 0.072 and 0.762 -> 0.119, residuals unchanged or better. The fans are gone. |
| 17:55 | **GATE NOT PASSED** on both conditions. Stopping before P2, as P1 instructs. |
