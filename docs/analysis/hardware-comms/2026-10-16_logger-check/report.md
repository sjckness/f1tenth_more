---
id: 2026-10-16_logger-check
title: "Il bag mente o lo stack rallenta? corridor10m_20261015T171753"
area: hardware-comms
date: 2026-10-16
tags: [logger, ros2-bag, perdita-scan, wall-tracker, cpu-saturo, qos]
modules: {f1tenth_logger: "0.0.1", f1tenth_perception: "0.0.1"}
branch: "orin/orin-wip-2026-10-22"
commit: "02cd0f2084da"
status: done
hypothesis: "Stabilire se i buchi nel bag derivano dal registratore o da un rallentamento reale dello stack."
result: "Verdetto (c) entrambi: nel bag /scan a 30.4 Hz contro 40.0 prodotti (24 % perso, 722 mancanti); wall tracker 155-178 ms per scan con CPU 95-97 % su 12 core, 4.2 osservazioni/s per 40 scan/s."
decision: "Mai un secondo ros2 bag record -a accanto a mission_logger; per clip raw lista topic senza Image/grid, --max-cache-size 536870912 e QoS RELIABLE depth 200 per /scan e /odometry/filtered."
raw_data: "nessun dato grezzo su git. Esterni: corridor10m_20261015T171753; bag di controllo archivio 2026-10-15T15-18-27_mission-llm_86cff518e42d"
migrated_from: "orin/orin-wip-2026-10-22:output/logger_check.md"
migrated_on: 2026-11-18
---
# L - does the bag lie, or did the stack slow down? `corridor10m_20261015T171753`
t = 0 is the first `/scan` stamp (1792077497.844); moving = |v| > 0.05 m/s, t = 26.1-41.7 s (max 0.47 m/s). This clip is NOT a mission_logger bag: plain `ros2 bag record` defaults (sqlite3, no compression, 100 MiB cache, no QoS overrides), `/viz/*` present, 5.5 MB/s of which 73 % is Image/grid topics - and mission_logger's recorder ran CONCURRENTLY over t = 28-41 s into archive run `2026-10-15T15-18-27_mission-llm_86cff518e42d`, whose bag is the control.

| window | /scan lat med/p95/max [ms] | mask lat med/max | /scan Hz here (40.0 produced) | tracker ms p50/p95 | mask tick p50/p95 | mpc solve p50/p95 | CPU x12 |
|---|---|---|---|---|---|---|---|
| 0-25 s still | 288 / 553 / 754 | 1792 / 6800 | 35.0 | 155 / 202 | 75 / 127 | -- | 95-96 % |
| 25-43 s moving | 521 / 723 / 1287 | 3139 / 4629 | 29.0 | 178 / 284 | 85 / 158 | 66 / 146 | 96-97 % |
| 43 s+ still | 433 / 623 / 784 | 1932 / 6855 | 30.4 | 174 / 222 | 91 / 154 | 91 / 117 | 95-97 % |

**Verdict: (c) both.** RECORDER: urg_node's own diagnostic reports 39.95 Hz and the concurrent archive bag holds `/scan` **complete at 40.0 Hz** over t = 28-41 s, while this bag holds 30.4 Hz - **24 % of scans never reached it** (722 missing; gap/period 155x2, 52x3, 37x4...), `/odometry/filtered` 26 Hz against the EKF's self-reported 50.000 Hz, latency to 9.5 s on `/perception/stereo_columns`. STACK: on 12 cores at 95-97 % the wall tracker spends 155-178 ms per scan against a 25 ms period, so the node emits 4.2 observations/s for 40 scans/s (archive bag) - dropping ~90 % of scans at its own subscription - and the 5 Hz mask tick lands at 3.4 Hz. Both halves worsen while moving (+15-20 % internal, +80 % latency).

Configuration fix, the recorder half:
1. Never run a second `ros2 bag record -a` beside mission_logger - the archive bag for the same seconds is complete.
2. For a raw clip pass an explicit topic list without the Image/grid topics (`/perception/scene_labels/image`, `/costmap/visualization`, `/bt/tree_visualization`, full-rate `depth_registered`, `/camera/image_raw`: 73 % of the bytes).
3. Add `--max-cache-size 536870912` plus a QoS override file making `/scan` and `/odometry/filtered` RELIABLE depth 200 (mission_logger's `_reliable_deep_qos` precedent for `/tf`).
