---
id: 2026-10-18_merge-cpu-cleanup
title: "Merge cpu-cleanup in campaign-final e replay di controllo"
area: platform
date: 2026-10-18
tags: [merge, cpu-cleanup, replay-v2, wall-tracker, regression-check]
modules: {f1tenth_perception: "0.0.1", f1tenth_params: "0.0.1", f1tenth_behavior: "0.0.1", f1tenth_bringup: "0.0.1", f1tenth_costmap: "0.0.1", f1tenth_localization: "0.0.0", f1tenth_logger: "0.0.1"}
branch: "orin/orin-wip-2026-10-22"
commit: "02cd0f2084da"
status: done
hypothesis: "Il merge di cpu-cleanup (13 commit) in campaign-final (V1-V4) è privo di conflitti e non altera il contratto WALL/CLEAR/NO_DATA?"
result: "Merge senza conflitti (61 file); 2228 test, 8 fallimenti tutti lint pre-esistenti; replay after e cpu-cleanup-solo identici (phantom 0, no_data 0.0 %). cpu-cleanup da solo: CLEAR 160.30 s / WALL 2.88 s, accordo con run 0.9331 -> 0.1238."
decision: "Merge e9ab0a04 senza conflitti; fix 0c1cd417: scan senza transform pubblica NO_DATA; non pushato; se il CLEAR oltre ~8 m è voluto va deciso sul branch cpu-cleanup"
raw_data: "git exp/2026-10-18_merge-cpu-cleanup:output/merge_cleanup/replay_after/ (2 file .txt/.log); exp/2026-10-18_merge-cpu-cleanup:output/merge_cleanup/replay_before_premerge/; exp/2026-10-18_merge-cpu-cleanup:output/merge_cleanup/replay_cpucleanup_only/. Esterni: ~/bags/corridor10m_20261016T144537"
migrated_from: "orin/orin-wip-2026-10-22:output/merge_cleanup/log.md"
migrated_on: 2026-11-18
---
# Merge cpu-cleanup into campaign-final — Orin, 2026-10-18

Operator: Claude (Opus 5), `~/dev_ws/f1tenth_more`. Not pushed, per the work order.
Rules honoured: no reset --hard, no rebase, no force, no branch -D, no clean.
Untouched: `.claude/settings.json`, `build_base573/`, `install_base573/`,
`first_test_campaing/`, and all of `output/` except `output/merge_cleanup/`.

## 0. Backup and premise check

    git tag backup/campaign-final-c6d8f99 c6d8f99      # local only
    git fetch $R cpu-cleanup:refs/remotes/gh/cpu-cleanup
    git fetch $R campaign-final:refs/remotes/gh/campaign-final

`gh/campaign-final` == `c6d8f990` == local HEAD, 0 ahead / 0 behind: the work order's
"V1-V4 already pushed by me" is confirmed. `gh/cpu-cleanup` = `82c0afa3`, 13 commits.
merge-base = `9c46add` ("H5: one teardown for both paths"), so H1-H5 are on BOTH
sides already and only V1-V4 (9 local commits incl. the earlier remote merge) face
cpu-cleanup's 13.

## 1. The merge — no conflicts

    git merge --no-ff gh/cpu-cleanup -m "Merge cpu-cleanup into campaign-final"
    -> e9ab0a04,  61 files, +6225 -1321

**Zero conflicts, trivial or otherwise.** Only three files were touched by both sides:

| file | hunks kept from c6d8f99 | hunks kept from 82c0afa |
|---|---|---|
| `src/f1tenth_params/config/stack_params.yaml` | +134 -11 | +2 -1 |
| `src/f1tenth_perception/.../lidar_front_wall_node.py` | +161 -61 | +98 -5 |
| `src/f1tenth_perception/test/test_lidar_front_wall_node.py` | +120 -31 | +164 -2 |

(read as: the merge result differs from that parent by this much, i.e. the other
parent's whole contribution survived). The heavy cpu-cleanup files
(`multi_wall_tracker.py` +771, `corridor_wall_replay.py` +1214, the behaviour-tree
`qos_events.py`/`tick_timing.py`, the `ekf_cost_observer_node.py` deletion) were not
touched by V1-V4 at all, and V4's `wall_markers.py` is new on the local side only.

No decision of the "same logic, different behaviour" kind had to be made in the merge
itself. One was needed afterwards — see `decisions.md`.

## 2. Static checks on the merged tree

| check | expected | found |
|---|---|---|
| `enable_wall_distance` | default false | `stack_params.yaml:1026` **false**; `wall_distance.launch.py` gates on `AndCondition(use_lidar, enable_wall_distance)` |
| `ekf_cost_observer` | gone from the launches | gone from all of `src/` and `scripts/`; the only mention left is `test_ekf_global_config.py::test_the_ekf_cost_observer_is_gone` |
| EKF frequency | 20 Hz | `ekf_frequency_hz` default **20.0**; the two yaml files still read 50.0 and are **overridden** by `ekf*.launch.py` from stack_params (comment says so) |
| decimation / backend | 4 / "c" | `wall_tracker_scan_decimation` **4**, `wall_extract_backend` **"c"** |
| `/viz/walls` lifetime | >= 3x the 5 Hz wall-layer period | `wall_marker_lifetime_s` **0.6 s**, `viz_walls_max_hz` **5.0** -> 3x0.2 = 0.6, met exactly |
| semantic wall layer | 5 Hz, whitelist consistent | `semantic_layer_rate_hz` 5.0 (launch) == `wall_layer_rate_hz` 5.0 (node timer); `costmap_boundary_extraction_rate_hz` 10.0; `/viz/walls` is covered by the existing `^/viz/.*` bridge rule, `test_foxglove_whitelists.py` green |

## 3. Build

    colcon build --symlink-install --packages-select f1tenth_behavior f1tenth_bringup \
      f1tenth_costmap f1tenth_diagnostics f1tenth_localization f1tenth_logger \
      f1tenth_more f1tenth_params f1tenth_perception corridor_perception_c

10 packages finished in 24.5 s, **clean** — no warnings, and the `ekf_cost_observer_node.py`
deletion left no dangling symlink in `build/`.

## 4. Tests

`colcon test --test-result-base <scratch>` over the same 10 packages (isolated result
base, so no stale xml).

* First run: **2228 tests, 0 errors, 9 failures**.
* After the fix in `decisions.md`: **2228 tests, 0 errors, 8 failures, 1 skipped**.

The 8 remaining are all lint, and all **pre-existing at c6d8f99** — verified by running
`ament_copyright` / `ament_pep257` / `ament_flake8` against a `git archive` export of
c6d8f990:

| package | failing lint | pre-merge exit code |
|---|---|---|
| f1tenth_bringup | copyright, pep257, flake8 | 1 / 1 / 1 |
| f1tenth_diagnostics | copyright, pep257, flake8 | 1 / 1 / 1 |
| f1tenth_logger | copyright, pep257 | 1 / 1 |

Neither file the flake8 runs name (`scripts/ensure_discovery_server.py`,
`launch/calibration.launch.py`) is touched by this merge. The pep257 complaints are the
project-wide D205/D213/D400/D415 docstring style, whole files at a time.

The tests the work order named, all green:

| area | tests | failures |
|---|---|---|
| event-driven e-stop | 76 | 0 |
| BT tick / tick_timing | 123 | 0 |
| wall markers + /viz/walls | 62 | 0 |
| cap by role | 20 | 0 |
| scan decimation | 9 | 0 |
| virtual wall rate | 5 | 0 |

## 5. Replay — `~/bags/corridor10m_20261016T144537`

cpu-cleanup rewrote `tools/corridor_wall_replay.py` (+1214, "wall replay v2") in the
same merge, so the existing `output/run_corridor10m_20261016T144537/` baseline is a
**v1** artefact and is not comparable: the tool's own `compare.md` marks nearly every
row "before: pre-v2 definition / pre-v2 replay / pre-v2 time base". Three runs of the
**same v2 tool** were therefore made, shadowing only `f1tenth_perception` on
`PYTHONPATH` from a `git archive` export, so the only variable is the node code:

| run | perception code | output |
|---|---|---|
| before | `c6d8f990` (pre-merge) | `output/merge_cleanup/replay_before_premerge/` |
| after | merged HEAD | `output/merge_cleanup/replay_after/` |
| control | `82c0afa3` (cpu-cleanup alone) | `output/merge_cleanup/replay_cpucleanup_only/` |

**Result 1 — the merge changes nothing.** "after" and "control" are identical on every
headline metric (phantom episodes 0, no_data 0.0 %, state time clear 160.30 s / wall
2.88 s, contract match 0.1238). V1-V4 are observably inert on the contract: they add a
viewer topic and nothing else.

**Result 2 — cpu-cleanup itself moves the contract a long way, and this needs your
eyes.** Same tool, same bag, pre-merge vs merged:

| metric, pass `replay_rec` | before (c6d8f99) | after (merged) |
|---|---|---|
| phantom_front_episodes | 1 (t=147.00-147.88, moving) | **0** |
| no_data_pct / _moving | 10.21 / 7.42 | **0.0 / 0.0** |
| state time WALL | 124.28 s | **2.88 s** |
| state time CLEAR | 22.24 s | **160.30 s** |
| agreement with the recorded car run | 0.9331 | **0.1238** (509 cycles wall -> clear) |
| confirmed tracks with rho_bl < 0.5 m | 22 | **0** |

The good half is exactly what R2 block 3 promised: the 147.0-150.24 s phantom that
`output/run_corridor10m_20261016T144537/report.md` calls "the top problem" — front
track 955 dead-reckoned from 3.27 m down to 1.87 m while the strip showed 6.75-6.96 m —
is gone, with no new phantom anywhere, and the junk near-field tracks (rho_bl < 0.5 m,
23 of them on the car) drop to zero.

The half to decide on: the merged pipeline also reports CLEAR for 160 of the 163 s of
this recording, where the car reported WALL for 121.78 s and measured it to |err| p95
6 mm over 499 cycles. The run never gets within 3 m of anything (`end_wall_error`
cycles = 0, `clear_state.true_strip_min` 8.671 m), so no cycle of it exercises a real
stop; whether "no FRONT track at all beyond ~8 m" is the behaviour you want on the
approach is a judgement this bag cannot settle. **It is cpu-cleanup's behaviour, not the
merge's** — the control run proves that — so it is a question for the branch, not a
blocker for the merge.

## 6. State

    0c1cd417 post-merge: a no-transform scan publishes its NO_DATA
    e9ab0a04 Merge cpu-cleanup into campaign-final   (c6d8f990 + 82c0afa3)

Not pushed. To push:

    git push git@github.com:sjckness/f1tenth_more.git campaign-final

To drop the follow-up commit and keep only the merge:

    git revert --no-edit 0c1cd417        # test_a_scan_with_no_laser_transform_is_NO_DATA goes red again
