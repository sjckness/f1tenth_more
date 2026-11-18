# Campaign runs: code version per run (Part A)

Source: each run's `<run_id>.manifest.json` `git` block in `~/f1tenth_archive/mission_row/<run_id>/`, mapped via `~/matlab_ws/f1tenth_more/data/index/runs.csv` (`archive_run_id`). Re-derived independently by a second script; no discrepancies. Do NOT use the .mat `meta.git_commit` (it is the export-time HEAD, 573f313, for all 58 files).

| test_id | archive run | branch | commit | dirty |
|---|---|---|---|---|
| P001-R001-20260921T145508 | 2026-09-21T12-55-11_mission-llm_fd5d8fd4f171 | test-logging | 8e0d48c5fb | true |
| P001-R002-20260921T145711 | 2026-09-21T12-57-14_mission-llm_fd5d8fd4f171 | test-logging | 8e0d48c5fb | true |
| P001-R003-20260921T145854 | 2026-09-21T12-58-58_mission-llm_fd5d8fd4f171 | test-logging | 8e0d48c5fb | true |
| P001-R004-20260921T150023 | 2026-09-21T13-00-26_mission-llm_fd5d8fd4f171 | test-logging | 8e0d48c5fb | true |
| P001-R005-20260921T150157 | 2026-09-21T13-02-00_mission-llm_fd5d8fd4f171 | test-logging | 8e0d48c5fb | true |
| P001-R006-20260930T172816 | 2026-09-30T15-28-19_mission-llm_fd5d8fd4f171 | test-logging | e47e64683f | true |
| P001-R007-20260930T173000 | 2026-09-30T15-30-03_mission-llm_d467d7d7ba04 | test-logging | e47e64683f | true |
| P001-R008-20260930T173125 | 2026-09-30T15-31-28_mission-llm_d467d7d7ba04 | test-logging | e47e64683f | true |
| P001-R009-20260930T173424 | 2026-09-30T15-34-28_mission-llm_d467d7d7ba04 | test-logging | e47e64683f | true |
| P001-R010-20260930T173632 | 2026-09-30T15-36-35_mission-llm_d467d7d7ba04 | test-logging | e47e64683f | true |
| P001-R011-20260930T173814 | 2026-09-30T15-38-17_mission-llm_d467d7d7ba04 | test-logging | e47e64683f | true |
| P001-R012-20260930T173951 | 2026-09-30T15-39-54_mission-llm_d467d7d7ba04 | test-logging | e47e64683f | true |
| P002-R001-20260922T142524 | 2026-09-22T12-25-27_mission-llm_2b356aac445f | test-logging | 119f239b76 | true |
| P002-R002-20260922T142712 | 2026-09-22T12-27-15_mission-llm_2b356aac445f | test-logging | 119f239b76 | true |
| P002-R003-20260922T142859 | 2026-09-22T12-29-02_mission-llm_2b356aac445f | test-logging | 119f239b76 | true |
| P002-R004-20260922T143058 | 2026-09-22T12-31-01_mission-llm_2b356aac445f | test-logging | 119f239b76 | true |
| P002-R005-20260922T143244 | 2026-09-22T12-32-48_mission-llm_2b356aac445f | test-logging | 119f239b76 | true |
| P002-R006-20260923T170928 | 2026-09-23T15-09-31_mission-llm_2b356aac445f | test-logging | bc3b049092 | true |
| P002-R007-20260923T171225 | 2026-09-23T15-12-28_mission-llm_2b356aac445f | test-logging | bc3b049092 | true |
| P002-R008-20260923T171756 | 2026-09-23T15-17-59_mission-llm_2b356aac445f | test-logging | bc3b049092 | true |
| P003-R001-20260928T164818 | 2026-09-28T14-48-21_mission-llm_a1198dffdcdc | test-logging | bc3b049092 | true |
| P003-R002-20260928T165040 | 2026-09-28T14-50-43_mission-llm_a1198dffdcdc | test-logging | bc3b049092 | true |
| P003-R003-20260928T165238 | 2026-09-28T14-52-41_mission-llm_a1198dffdcdc | test-logging | bc3b049092 | true |
| P003-R004-20260928T165409 | 2026-09-28T14-54-12_mission-llm_a1198dffdcdc | test-logging | bc3b049092 | true |
| P003-R005-20260928T165545 | 2026-09-28T14-55-51_mission-llm_a1198dffdcdc | test-logging | bc3b049092 | true |
| P003-R006-20260928T165817 | 2026-09-28T14-58-20_mission-llm_a1198dffdcdc | test-logging | bc3b049092 | true |
| P003-R007-20260928T165947 | 2026-09-28T14-59-50_mission-llm_a1198dffdcdc | test-logging | bc3b049092 | true |
| P003-R008-20260928T170036 | 2026-09-28T15-00-39_mission-llm_a1198dffdcdc | test-logging | bc3b049092 | true |
| P003-R009-20260928T170138 | 2026-09-28T15-01-41_mission-llm_a1198dffdcdc | test-logging | bc3b049092 | true |
| P003-R010-20260928T170337 | 2026-09-28T15-03-40_mission-llm_a1198dffdcdc | test-logging | bc3b049092 | true |
| P004-R001-20260922T143740 | 2026-09-22T12-37-43_mission-llm_536cba700065 | test-logging | 119f239b76 | true |
| P004-R002-20260922T143855 | 2026-09-22T12-38-59_mission-llm_536cba700065 | test-logging | 119f239b76 | true |
| P004-R003-20260923T145652 | 2026-09-23T12-56-55_mission-llm_536cba700065 | test-logging | fa9f99e4dc | true |
| P004-R004-20260923T165958 | 2026-09-23T15-00-02_mission-llm_536cba700065 | test-logging | bc3b049092 | true |
| P004-R005-20260923T170639 | 2026-09-23T15-06-42_mission-llm_536cba700065 | test-logging | bc3b049092 | true |
| P004-R006-20260924T134512 | 2026-09-24T11-45-15_mission-llm_536cba700065 | test-logging | bc3b049092 | true |
| P004-R007-20260924T134658 | 2026-09-24T11-47-02_mission-llm_536cba700065 | test-logging | bc3b049092 | true |
| P004-R008-20260924T134911 | 2026-09-24T11-49-14_mission-llm_536cba700065 | test-logging | bc3b049092 | true |
| P004-R009-20260924T135136 | 2026-09-24T11-51-44_mission-llm_536cba700065 | test-logging | bc3b049092 | true |
| P004-R010-20260924T135341 | 2026-09-24T11-53-44_mission-llm_536cba700065 | test-logging | bc3b049092 | true |
| P004-R011-20260924T135544 | 2026-09-24T11-55-47_mission-llm_536cba700065 | test-logging | bc3b049092 | true |
| P004-R012-20261001T111634 | 2026-10-01T09-16-37_mission-llm_536cba700065 | test-logging | 573f313b1e | true |
| P004-R013-20261001T111911 | 2026-10-01T09-19-14_mission-llm_536cba700065 | test-logging | 573f313b1e | true |
| P004-R014-20261001T112202 | 2026-10-01T09-22-05_mission-llm_536cba700065 | test-logging | 573f313b1e | true |
| P004-R015-20261001T112447 | 2026-10-01T09-24-50_mission-llm_536cba700065 | test-logging | 573f313b1e | true |
| P004-R016-20261001T112933 | 2026-10-01T09-29-36_mission-llm_536cba700065 | test-logging | 573f313b1e | true |
| P004-R017-20261001T113120 | 2026-10-01T09-31-28_mission-llm_536cba700065 | test-logging | 573f313b1e | true |
| P004-R018-20261001T113259 | 2026-10-01T09-33-02_mission-llm_536cba700065 | test-logging | 573f313b1e | true |
| P004-R019-20261001T113450 | 2026-10-01T09-34-53_mission-llm_536cba700065 | test-logging | 573f313b1e | true |
| P005-R001-20260924T135827 | 2026-09-24T11-58-31_mission-llm_38d747d91edf | unknown | bc3b049092 | unknown (no run-time record) |
| P005-R002-20260924T140159 | 2026-09-24T12-02-02_mission-llm_e8f5ca47b036 | test-logging | bc3b049092 | true |
| P005-R003-20260924T140430 | 2026-09-24T12-04-33_mission-llm_e8f5ca47b036 | test-logging | bc3b049092 | true |
| P005-R004-20260924T140952 | 2026-09-24T12-09-55_mission-llm_e8f5ca47b036 | test-logging | bc3b049092 | true |
| P005-R005-20260924T141303 | 2026-09-24T12-13-06_mission-llm_e8f5ca47b036 | test-logging | bc3b049092 | true |
| P005-R006-20260924T141529 | 2026-09-24T12-15-33_mission-llm_e8f5ca47b036 | test-logging | bc3b049092 | true |
| P005-R007-20260924T141924 | 2026-09-24T12-19-28_mission-llm_e8f5ca47b036 | test-logging | bc3b049092 | true |
| P005-R008-20260924T142157 | 2026-09-24T12-22-01_mission-llm_e8f5ca47b036 | test-logging | bc3b049092 | true |
| P005-R009-20260924T142419 | 2026-09-24T12-24-22_mission-llm_e8f5ca47b036 | test-logging | bc3b049092 | true |

## Commits

| commit | date | runs | subject |
|---|---|---|---|
| 8e0d48c5fb | 2026-09-21T12:42:48+02:00 | P001-R001..R005 (2026-09-21 14:55-15:02 CEST). Next commit on test-logging: 6de6ac3 at 16: | f1tenth_logger: the test-campaign logger moves in, beside the mission  |
| 119f239b76 | 2026-09-22T12:37:32+02:00 | P002-R001..R005, P004-R001..R002 (2026-09-22 14:25-14:39 CEST). Next commit: 220dba3 on 20 | corridor_perception: Step 0 baseline, per-scan structure from real bag |
| fa9f99e4dc | 2026-09-23T12:50:07+02:00 | P004-R003 (2026-09-23 14:56 CEST). Next commit: bc3b049 at 16:39 | arc corridor: the target-line cost belongs to the arc, not to every ob |
| bc3b049092 | committer 2026-09-23T16:3 | P004-R004..R005, P002-R006..R008 (2026-09-23 17:00-17:18), P004-R006..R011, P005-R002..R00 | object corridor: a maximum length, an end on the target, and a run you |
| e47e64683f | 2026-09-30T09:42:23+02:00 | P001-R006..R012 (2026-09-30 17:28-17:39 CEST). Next commit: 573f313 at 18:25 the same day | repo root: move the loose docs under docs/, drop the stale ones |
| 573f313b1e | 2026-09-30T18:25:50+02:00 | P004-R012..R019 (2026-10-01 11:16-11:34 CEST). Next descendant: 1d00853 on 2026-10-07 (not | test: campaing: m01 with obstacle done |

## Discrepancies

- .mat meta.git_commit = 573f313b1e and meta.git_dirty = 1 in all 58 .mat files. These are export-time values: exporter.py runs `repo = git_commit(root)` once per export (l.333), with `git rev-parse HEAD` and `git status --porcelain --untracked-files=no` (l.130-139). They disagree with the run-time commit for 50 runs: P001-R001..R012, P002-R001..R008, P003-R001..R010, P004-R001..R011 and P005-R001..R009. They match only P004-R012..R019, by coincidence. Do not use meta.git_*. Use meta.campaign_meta_json.git_commit or the manifest.
- P005-R001-20260924T135827 has no manifest, params.yaml or parquet anywhere on linus. Its bag stayed in the car's active/ dir (.mat meta.bag_path=/home/fabiocar/f1tenth_archive/active/2026-09-24T11-58-31_mission-llm_38d747d91edf/bag); auto_outcome aborted, 'superseded by new test'. Its commit bc3b049 comes only from the test-campaign logger (no dirty flag); branch and dirty state are unknown.
- Every manifest says dirty=true, and it is real and functional. Each run's params.yaml differs from the committed stack_params.yaml at its own commit: MPC steering bounds -0.309/+0.319 vs committed -0.283/+0.278, plus foxglove/viz keys. That content first appears in git on 2026-10-07 (1d00853). No run is reproducible from its manifest commit alone.
- Steering gains: steering_calibration.yaml has gain_left/right -1.2135 at all six run commits. The car's params snapshot prose describes -1.0552/-1.1133 (2026-09-17), and these first enter git in 1d00853. The car's steering_calibration.yaml was never snapshotted, so the gains actually in effect are inferred, not verified.
- Dirty-flag semantics: the mission logger counts untracked files (git status --porcelain). The campaign output dir first_test_campaing/ sits inside the car's repo and is not ignored, except *.csv by .gitignore. So dirty=true would be true even with no source edits. The params.yaml diff is what shows real tracked edits.
- Test IDs are reused, and only test_id plus timestamp is unique. ~/f1tenth_archive/test_campaign/M02_approach_wall/P002-R001-20260921T150345 (8e0d48c, archive 2026-09-21T13-03-48_mission-llm_926f7913d041) is a different run from runs.csv P002-R001-20260922T142524 (119f239). 1d00853:first_test_campaing/results.csv has 89 rows, 31 of whose test_ids are not in runs.csv (e.g. four different P002-R001-*).
- The brief's 'last run P004-R018' is contradicted. P004-R019-20261001T113450 is in runs.csv and in 1d00853 results.csv (the last 20261001 row). It has a manifest (ABORTED, empty bag with 0 messages, 573f313, dirty). A further archived mission, 2026-10-01T09-36-57_mission-llm_536cba700065 (COMPLETE, 573f313, dirty), has no campaign test row and is not in runs.csv.
- No other disagreement. For all 57 runs with a manifest these agree: the manifest git block, the parquet-embedded manifest, the test-campaign logger's git_commit (.mat campaign_meta_json, 1d00853 meta.json and results.csv, plus linus test_campaign for P001-R001..R005), and the timeline check (each start_time falls after its commit and before the next first-parent commit of origin/test-logging).
