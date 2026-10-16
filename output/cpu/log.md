# Session log — H1..H5, branch campaign-final, Orin, OFFLINE

Battery disconnected, ZED and Hokuyo absent. No sensor driver and no command path
launched at any point. Files staged by name, never `git add -A`.

| time | point | what |
|---|---|---|
| 09:56 | — | session start at 70307ae |
| 10:07 | H1 | DONE: corridor_perception_c built, ctypes wrapper, backend param (default c). Equivalence over ALL 2292 scans: 0 count mismatches, 0 gate-stat mismatches, max diff 7.1e-15 m. extract_lines 24.618 -> 1.093 ms p50 (22.5x), target was 2 ms. 967 tests green (22 new) |
| 10:12 | H2 | implemented: wall_tracker_scan_decimation (default 4, counter), scan subscription depth 1 best-effort, skipped scans hold the last verdict. 8 decimation tests + 3 association tests at 10 Hz. Counts kept, time equivalents in the report |
| 10:16 | H4 | DONE: window a92e133 (2026-10-15 15:17) -> d355bbe (2026-10-16 02:31), 17 commits. Exactly ONE archive run affected: 2026-10-15T15-18-27_mission-llm_86cff518e42d, 42/71 frames (59.2%), median 2.6158 m contaminated vs 2.8698 clean. CSV + correction note written |
| 10:23 | H5 | tools/cpu_test.sh + cpu_test_probe.py (pidstat is NOT installed; /proc fallback) + cpu_test_summary.py written; dry run of phases a and b under way |
| 10:25 | H5 | first dry run HUNG on `ros2 topic pub -1` in phase b (it blocks with no subscriber matched); the trap cleaned up correctly. Rate-limited both publishes under a timeout and fixed the rate counter, which had measured 0 Hz on every topic because it built its subscriptions from the graph before discovery |
| 10:28 | H5 | third dry run after unsetting ROS_DISCOVERY_SERVER (set in the env, server only runs with the stack: the second dry run counted 0 messages on all 11 topics with the bag demonstrably playing) |
| 10:30 | H5 | dry run GREEN end to end: phases a+b, summary.md with the per-phase tables and the a->b diff. lidar_front_wall_node 24.9%/28.4% CPU on core 5; /perception/walls 3.69 Hz against virtual_wall/debug 16.34 Hz = the decimation measured on real data |
| 10:32 | — | housekeeping: colcon build of 8 packages clean in 15.2 s, suites 1214 passed / 1 skipped / 1 xfailed, no ROS processes left. Session end |
| 10:33 | H5 | FIX: two orphaned depth_throttle_5hz nodes survived the trap (ros2 launch killed before forwarding). Each launch child now runs under setsid and cleanup() kills the process GROUP; a pkill by name was rejected because on the car it would reach the live stack |
| 10:36 | H5 | per-phase teardown ALSO fixed: it was killing bare pids and reporting "0 groups" at the trap, which is where the orphans actually came from. One `stop_owned` now used by both paths; the verification run reports "stopping 5 groups" per phase |
