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
