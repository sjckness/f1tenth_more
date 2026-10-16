# Overnight session log — branch campaign-final, Orin, OFFLINE

Battery disconnected, ZED and Hokuyo absent. No sensor driver or command path was
launched at any point. Data: the bag of `corridor10m_20261015T171753` and the archive
run `2026-10-15T15-18-27_mission-llm_86cff518e42d`.

Start: 2026-10-16T02:50:53+02:00, at `55e5f25`.
Order: F2 -> F1 -> F3 -> F4 -> F6 -> F5 -> G.

| time | point | what |
|---|---|---|
| 02:50 | — | session start, `output/nightly/` and `output/glass_lidar/` created |
| 02:52 | F2 | worktree `c44b503` (= `d355bbe^`) created and f1tenth_perception/corridor_perception built there; F2 bucket script written |
| 03:00 | F2 | live evidence straight from both bags: `/perception/debug/front_wall` carries `frame_id: base_link` and `front_offset_m: 0.405` on 163/241 raw-bag messages (67.6 %) and 42/71 archive messages (59 %); the correct values are the optical frame and 0.1756 m |
| 03:10 | F2 | DONE. Column buckets BIT-IDENTICAL pre/post (6 buckets, 146 cycles): the aliasing never reached the cross-check. Contamination is in front_clearance_node's own ROI path: front_distance 0.2294 m short on 163/241 frames. `output/nightly/f2_header_aliasing.md` |
| 03:12 | F1 | implemented: max_range_m 8.0 (samples past it not informative), the columns' own quadratic tolerance (base 0.05 + quad 0.012 d^2, same two params), REJECTED needs 3 verifiable cycles. 947 tests green (12 new) |
| 03:25 | F1 | replay: track 44 (15.85 m end wall) UNVERIFIED and FRONT on 90/106 cycles; 84/106 (11.4 m) and 73/109 (13.5 m) UNVERIFIED; glass walls 2/10/30/64/97 VERIFIED+glass; right walls 87/113/124 VERIFIED; NO track rejected anywhere. Contract WALL 22.51 s (recorded 35.91), end-wall error median 0.010 m |
| 03:35 | F3 | implemented: per-track payload behind `wall_debug_tracks` (default FALSE, replay forces it on), occlusion rays only for overlapping bearing sectors. Tracker p50 46.76 -> 44.93 ms with the payload ON; measuring with it off |
| 03:40 | F4 | `ROLE_VALUE['other'] = 50`, unknown roles fall back to `other`; every consumer checked (only mission_logger records the grid, nothing in the control path reads it). 229 costmap tests green |
| 03:55 | F3 | after three optimisation passes: tracker 44.20 ms p50 / 53.63 p95 with the payload off (the car's configuration). Target was 38.59 + 5 = 43.59 -> 0.61 ms OVER. Breakdown in the report |
| 04:05 | F6 | DONE: default false; the supervisor now declares and injects the argument (it forwards no CLI args of its own, so the documented campaign command would have been silently ignored); TEST_CAMPAIGN.md, two procedures and README updated. Launch-description evidence captured. 81 bringup/logger tests green; 3 pre-existing lint failures confirmed pre-existing |
| 04:20 | F5 | DONE (report + proposal, nothing applied): the concurrent recorder was a MANUAL `ros2 bag record -a` -- no launch file, component or script in the repo starts one, and the bag's own 21 empty topics plus `/events/write_split` prove `-a`. Not part of the bringup, so the conditional toggle does not apply. Both recorders ~5 MB/s, concurrent, 93 %/88 % of it images and grids |
