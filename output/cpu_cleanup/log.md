# cpu-cleanup log (linus, static analysis only)

- 2026-10-16T14:19:37+02:00 step 0: remote orin = fabiocar@jetson-orin:dev_ws/f1tenth_more; fetch orin campaign-final OK; orin/campaign-final == 9c46add (ancestor check OK, 0 commits ahead).
- 2026-10-16T14:19:37+02:00 step 0: branch cpu-cleanup created at 9c46add in a separate worktree ~/dev_ws/f1tenth_more_cpu_cleanup (main checkout left on jazzy-campaign with its uncommitted work; see decisions.md D0).
- 2026-10-16T14:48:04+02:00 baseline pytest f1tenth_behavior (Jazzy env, source on PYTHONPATH): 368 passed, 1 FAILED pre-existing: test_go_to_object_behaviour.py::TestFailureOutcomes::test_grace_then_target_lost (fails at 9c46add too, untouched by this work).
- 2026-10-16T14:48:04+02:00 point 1: bench on linus (synthetic Orin-rate inputs): BT base 20.1-25.9% (mean ~23%) of one core -> new 11.9-13.9% (mean ~12.6%). Tick p50 0.38 ms, p95 0.96 ms at 10.00 Hz measured (/diagnostics).
- 2026-10-16T14:48:04+02:00 point 1: note: another session ran colcon build in ~/dev_ws/f1tenth_more (jazzy-campaign) at ~14:32; benches taken after it finished; numbers are noisy (+-2 points) because linus is a desktop under load.
- 2026-10-16T14:48:04+02:00 point 1: pytest f1tenth_behavior: 417 passed (+50 new), same 1 pre-existing failure; flake8 E/W/F: no new errors on touched files.
- 2026-10-16T14:53:08+02:00 point 2: wall_distance gated (enable_wall_distance=false). Tests: perception wall_distance launch/unit, bringup tests: all pass except pre-existing ament lint tests (test_copyright/test_flake8/test_pep257 fail at 9c46add too; no new findings vs base). Pre-existing failure also seen: mpc_controller test_d_wall_correction.py::test_every_other_wall_turn_mission_is_still_terminal_on_the_turn (fails at 9c46add; drive_stop_2m_from_wall.json).
- 2026-10-16T14:53:50+02:00 point 3: enable_ekf_cost_observer default false; localization + diagnostics tests 24 passed; no new lint findings.
- 2026-10-16T15:03:34+02:00 point 4: costmap tests (with local msg stand-in for MultiWallTrack) 232 passed; bringup 42 passed + 3 pre-existing ament lint failures (no new findings). Benches: costmap_boundary 23.3->11.5%, reset_manager 10.9->0.0% (linus, % of one core).
- 2026-10-16T15:13:28+02:00 follow-up commit 9dde6ee: tree.timer = ticker.timer (shutdown parity with tick_tock on any py_trees_ros version). Bench after: 12.1%.
- 2026-10-16T15:13:28+02:00 point 5: pinning table written (proposal only, nothing applied).
- 2026-10-16T15:13:28+02:00 point 6: py_compile 23 files OK; flake8 E/W/F no new errors; ament lint no new findings; pytest new vs base: identical failure sets except base-only test_watch_objects errors (tools/ missing from the base extract). Launch diffs in report.md.
- 2026-10-16T15:13:28+02:00 report written; nothing pushed.
