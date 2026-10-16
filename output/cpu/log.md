# Session log — H1..H5, branch campaign-final, Orin, OFFLINE

Battery disconnected, ZED and Hokuyo absent. No sensor driver and no command path
launched at any point. Files staged by name, never `git add -A`.

| time | point | what |
|---|---|---|
| 09:56 | — | session start at 70307ae |
| 10:07 | H1 | DONE: corridor_perception_c built, ctypes wrapper, backend param (default c). Equivalence over ALL 2292 scans: 0 count mismatches, 0 gate-stat mismatches, max diff 7.1e-15 m. extract_lines 24.618 -> 1.093 ms p50 (22.5x), target was 2 ms. 967 tests green (22 new) |
| 10:12 | H2 | implemented: wall_tracker_scan_decimation (default 4, counter), scan subscription depth 1 best-effort, skipped scans hold the last verdict. 8 decimation tests + 3 association tests at 10 Hz. Counts kept, time equivalents in the report |
| 10:16 | H4 | DONE: window a92e133 (2026-10-15 15:17) -> d355bbe (2026-10-16 02:31), 17 commits. Exactly ONE archive run affected: 2026-10-15T15-18-27_mission-llm_86cff518e42d, 42/71 frames (59.2%), median 2.6158 m contaminated vs 2.8698 clean. CSV + correction note written |
