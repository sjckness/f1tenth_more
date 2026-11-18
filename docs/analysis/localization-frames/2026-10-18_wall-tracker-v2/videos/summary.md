# Wall pipeline variants, run by run

Generated 2026-10-19 09:18:50 by `tools/wall_variant_videos.py` at HEAD, through `tools/corridor_wall_replay.py`. Every number is the REPLAY's, on the current code, not what the car published at the time.

## The variants

| variant | parameters | file |
|---|---|---|
| **v1** | HEAD: ray angle 25 deg, birth span 1.0 m | `tools/wall_variants/v1_head.yaml` |
| **v2** | D7 alternative: ray angle OFF (0), birth span 0.5 m | `tools/wall_variants/v2_d7_alternative.yaml` |
| **v3** | birth span 0.5 m alone, ray angle still 25 deg | `tools/wall_variants/v3_birth_span_only.yaml` |

No default was changed: each variant is a `--param-file` override on top of the installed `stack_params.yaml`.

## `corridor10m_20261016T144537`

164.5 s, 14.93 m driven, 22 % of samples moving, camera pass `rec`, camera pitch -0.03635 rad.

| | v1 | v2 | v3 |
|---|---|---|---|
| cycles measured | 703 | 703 | 703 |
| cycles with a FRONT wall | 77.4 % | 78.1 % | 78.1 % |
| cycles with a LEFT wall | 93.9 % | 93.9 % | 93.9 % |
| cycles with a RIGHT wall | 8 % | 87.9 % | 87.9 % |
| cycles with an 'other' wall | 0 % | 0 % | 0 % |
| published track instances | 1287 | 2207 | 1989 |
| published tracks per cycle | 1.83 | 3.14 | 2.83 |
| distinct published ids | 11 | 21 | 25 |
| births per minute | 4.01 | 7.66 | 9.12 |
| role switches (one id, two roles) | 0 | 0 | 0 |
| tracks held (published or not) | 97 | 379 | 242 |
| of those, alive under 1 s | 63 | 167 | 153 |
| published tracks never seen by the LiDAR | 0 | 1 | 1 |
| published tracks within 0.5 m of the car | 0 | 0 | 0 |
| phantom FRONT episodes | 1 | 0 | 0 |
| WALL episodes | 14 | 13 | 13 |
| NO_DATA share of contract time | 0 % | 0 % | 0 % |
| tracker p50 | 23.65 ms | 32.46 ms | 29.73 ms |
| tracker p95 | 32.79 ms | 42.21 ms | 38.48 ms |

## `2026-10-15T15-18-27_mission-llm_86cff518e42d`

13.9 s, 4.88 m driven, 100 % of samples moving, camera pass `rec`, camera pitch -0.03635 rad.

| | v1 | v2 | v3 |
|---|---|---|---|
| cycles measured | 51 | 51 | 51 |
| cycles with a FRONT wall | 43.1 % | 43.1 % | 43.1 % |
| cycles with a LEFT wall | 80.4 % | 84.3 % | 84.3 % |
| cycles with a RIGHT wall | 0 % | 64.7 % | 60.8 % |
| cycles with an 'other' wall | 0 % | 0 % | 0 % |
| published track instances | 63 | 106 | 104 |
| published tracks per cycle | 1.24 | 2.08 | 2.04 |
| distinct published ids | 3 | 5 | 5 |
| births per minute | 12.98 | 21.63 | 21.63 |
| role switches (one id, two roles) | 0 | 0 | 0 |
| tracks held (published or not) | 15 | 61 | 39 |
| of those, alive under 1 s | 6 | 25 | 22 |
| published tracks never seen by the LiDAR | 0 | 0 | 0 |
| published tracks within 0.5 m of the car | 0 | 0 | 0 |
| phantom FRONT episodes | 1 | 1 | 1 |
| WALL episodes | 5 | 5 | 5 |
| NO_DATA share of contract time | 0 % | 0 % | 0 % |
| tracker p50 | 25.05 ms | 35.44 ms | 30.14 ms |
| tracker p95 | 32.51 ms | 48.08 ms | 39.63 ms |
