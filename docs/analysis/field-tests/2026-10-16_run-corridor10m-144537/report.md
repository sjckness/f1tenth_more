---
id: 2026-10-16_run-corridor10m-144537
title: "Analisi offline run corridor10m_20261016T144537"
area: field-tests
date: 2026-10-16
tags: [corridor10m, wall-tracker, phantom-wall, replay, scene-mask, front-wall]
modules: {f1tenth_perception: "0.0.1", f1tenth_costmap: "0.0.1"}
branch: "orin/orin-wip-2026-10-22"
commit: "02cd0f2084da"
status: done
hypothesis: "Verificare con replay offline (tools/corridor_wall_replay.py) qualita' del bag, maschera camera e tracker del muro dopo le modifiche rispetto al 10-15."
result: "Bag pulito (/scan 39.93 Hz, nessun buco); maschera migliorata (invalid 0.24->0.06, mask tick 83->33 ms p50). Muro fantasma di 3.24 s (t=147.0-150.24) con errore -4.905 m: la macchina molto probabilmente si e' fermata ~5 m prima."
decision: "n/d (solo diagnosi); il replay non riproduce il fantasma ma questo non prova una correzione (tracker parte a freddo)."
raw_data: "git exp/2026-10-16_run-corridor10m-144537:output/run_corridor10m_20261016T144537/corridor10m_20261016T144537.camera.mp4; exp/2026-10-16_run-corridor10m-144537:output/run_corridor10m_20261016T144537/corridor10m_20261016T144537.info.txt; exp/2026-10-16_run-corridor10m-144537:output/run_corridor10m_20261016T144537/corridor10m_20261016T144537.topdown.mp4; exp/2026-10-16_run-corridor10m-144537:output/run_corridor10m_20261016T144537/replay_stdout.log. Esterni: ~/bags/corridor10m_20261016T144537/; topdown.mp4 (163.5 s) e camera.mp4 (94.4 s) in output/run_corridor10m_20261016T144537/"
migrated_from: "orin/orin-wip-2026-10-22:output/run_corridor10m_20261016T144537/report.md"
migrated_on: 2026-11-18
---
# corridor10m_20261016T144537 — offline analysis (campaign-final, tools/corridor_wall_replay.py)

**Bag is clean, the camera mask got much better, and the run ends on a 3.2 s phantom wall.**
/scan 39.93 Hz mean, 81 of 82 two-second buckets at 39.5–40 Hz, max gap 43 ms; the only sub-35 bucket is the truncated final 1.35 s (27 Hz) — **no bag holes**. Improved vs 10-15: invalid 0.24→0.06, reflection 0.10→0.006, self 0.15→0.08, mask tick 83→33 ms p50, tracker 169→117 ms p50, end-wall |err| p95 still ~1–4 cm over 1990 cycles.
Still wrong: NO_DATA jumped 0.5 %→13.1 % of the run, 23 of 36 recorded tracks are throwaway stereo "right" walls at ρ≈0.20 m (12–38 cm from base_link, each <1.5 s, births t=64.6–134.6 s), and tracker p50 117 ms on-car vs 28 ms for the same code in the isolated replay — the cost is contention, not the algorithm.
**Top problem — topdown.mp4 t=147.0–150.24 s (also in camera.mp4):** front track 955 (13.67 m long, confidence 0.24–0.37, confirmed at 5 hits, height never measured) holds the FRONT role with **no LiDAR fit at all** (lidar_m None, reason 2, from t=145.1) and is dead-reckoned from 3.27 m down to **1.87 m while the laser strip shows 6.75–6.96 m**. The replay reports 6.72–6.93 m on the same stamps. It clears only on front_coast_expired at 150.26 s; mission COMPLETE lands 0.2 s later, so the car very likely stopped ~5 m early.
Caveat: the replay not reproducing it is **not** evidence of a fix — the tracker starts cold, and the recorded FRONT track was already 106.9 s old.

Missing topics (logged, those parts skipped, tool untouched): REQUIRED `/camera/detection_masks`, `/perception/wall_observations`; PREFERRED `depth_registered_5hz`, `/camera/image_raw`. So: camera half not replayed (0 frames, columns.csv header-only), step-3 observation diff n=0, and **rect verify never ran — every track is UNVERIFIED and no G/glass markers appear in the videos.** Class fractions below are therefore the *recorded* `/scene_labels/stats`, not a recomputation.

| metric (recorded unless noted) | 10-16 (this bag) | 10-15 (replay_e) |
|---|---|---|
| mask frames / wall cycles | 625 / 2830 | 145 / 264 |
| frac floor / floor_inferred | 0.0289 / **0.3094** | 0.0132 / — |
| frac invalid / reflection | 0.0579 / 0.0056 | 0.2405 / 0.1008 |
| frac wall / ceiling | 0.3364 / 0.1559 | 0.3151 / 0.1687 |
| frac self / object | 0.0784 / 0.0097 | 0.1478 / 0.0138 |
| mask tick ms p50 / p95 | 33.19 / 53.85 | 82.56 / 149.13 |
| health max age depth / tick / profile [s] | 0.429 / 0.493 / 0.548 (limit 1.0) | topic absent |
| watchdog fired | **no** — 0 of 326 | n/a |
| contract wall / clear / no_data [s] | 121.78 / 20.87 / **21.61** | 33.81 / 38.64 / 0.37 |
| end-wall err median / p95 / max abs [m] | -0.014 / 0.009 / **5.004** | +0.011 / 0.019 / 0.151 |
| same, replay pass | -0.011 / 0.008 / 0.191 | +0.011 / 0.091 / 0.148 |
| phantom episodes | **1** (t 147.0–150.24, 3.24 s, 60 cyc, err -4.905 m) | 0 |
| true strip while CLEAR: min / <5 m / <2 m | 8.671 m / 0 / 0 | 14.638 m / 0 / 0 |
| tracker_ms p50 / p95 | 117.46 / 179.48 | 169.07 / 233.87 |
| rect_ms p50 / p95 | 16.47 / 43.25 | field absent |
| tracker_skipped (decimation 4) | 4245, nonzero in 75.0 % of cycles | no decimation |
| stereo used (stereo_weight > 0) | 97.0 % of cycles, weight p50 0.734 | field absent |
| track ids: total / stereo-only / <1.5 s | 36 / 23 / 23 | 10 / 1 / 0 |
| tracks with ρ_bl < 0.5 m | **23** (ρ med 0.203 m, 89 cyc) | 2 |
| replay track ids (published) | 130 | 21 |
| replay tracks held: total / glass / verified / rejected | 1054 / **0** / **0** / 0 | 104 / 13 / 41 / 8 |

Tracks that ever held FRONT (recorded): id 2 — 499 cyc, ρ_bl 21.46 m, len 2.17 m, confirmed, lidar; id **955** — 15 cyc, ρ_bl 2.57 m, len 13.67 m (the phantom); id 889 — 5 cyc, ρ_bl 10.40 m. Full 36-row and 1054-row tables: `*.summary.md`. Mission timeline: LOADED -1.6 s, ABORTED +42.5 s, RUNNING 57.7→COMPLETE 70.8 s, RUNNING 125.6→COMPLETE 150.5 s.

- top-down, whole run, 10 Hz: `output/run_corridor10m_20261016T144537/corridor10m_20261016T144537.topdown.mp4` (1280x1020, 1635 f, 163.5 s)
- camera, 5 Hz, t=57.5–151.6 s: `output/run_corridor10m_20261016T144537/corridor10m_20261016T144537.camera.mp4` (1920x828, 472 f, 94.4 s)
