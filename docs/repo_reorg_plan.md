# Repo reorganisation plan (campaign-final)

Status: **plan only, nothing moved yet** (2026-10-14). Phase 2 applies it as
separate commits (logs, output/, tools/, path fixes) after Andreas's go.
Branch: `campaign-final` at `a8a61db` ("campaign: fix folder name").

Decisions that need an answer before Phase 2 are marked **DECIDE**.

---

## 0. Step 0 results

### Cleanup status
- Branches on origin: **`main`, `campaign-final`, `jazzy`** only.
  `llm` and `scene-graph` are **gone from origin** too. Both pointed at
  `e47e646` (= `main`), so no commit was lost. If they should exist again:
  `git push origin e47e646:refs/heads/llm e47e646:refs/heads/scene-graph`.
- `origin/main` → `e47e646` (not moved; step 4 was skipped on purpose).
- Tags on origin: `archive/pre-campaign-final-20261012`,
  `archive/test-campaign`, `archive/mpc-corridor-split`,
  `archive/orin-snapshot-20261013`, `evidence/campaign-dirty-p004-r012-r019`.
- Local branches: `main`, `campaign-final`, `jazzy` (8 behind origin).
- Worktrees: only `~/dev_ws/f1tenth_more`.

### Size of cdfd736 (campaign data import)
- `git diff --shortstat`: 898 files changed, 243,403 insertions.
- `du -sh first_test_campaign/`: **123 MB** (whole folder, 918 files).
- By type: 354 `.jsonl`, 335 `.csv`, 136 `.json`, 70 `.png`, 2 `.txt`, 1 `.md`.
- **No bags and no MP4s** (no `.db3`, `.mcap`, `.bag`, `.mp4`).
- One file over 10 MB, and it is a `.jsonl`, not a bag/MP4.

| MB | file (under `first_test_campaign/`) |
|---:|---|
| 10.5 | `M02_approach_wall/P002-R002-20261008T133423/corridor_debug.jsonl` |
| 6.1 | `M02_approach_wall/P002-R008-20260923T171756/corridor_debug.jsonl` |
| 5.0 | `M02_approach_wall/P002-R011-20261008T142941/corridor_debug.jsonl` |
| 4.0 | `M01_forward_3m/P001-R008-20260930T173125/corridor_debug.jsonl` |
| 2.8 | `M05_turn_left_wall/P005-R002-20260924T140159/corridor_debug.jsonl` |
| 2.7 | `M05_turn_left_wall/P005-R007-20260924T141924/corridor_debug.jsonl` |
| 2.7 | `M05_turn_left_wall/P005-R003-20260924T140430/corridor_debug.jsonl` |
| 2.7 | `M05_turn_left_wall/P005-R009-20260924T142419/corridor_debug.jsonl` |
| 2.7 | `M05_turn_left_wall/P005-R006-20260924T141529/corridor_debug.jsonl` |
| 2.6 | `M05_turn_left_wall/P005-R005-20260924T141303/corridor_debug.jsonl` |

History not rewritten.

### Folder rename (done, `a8a61db`, pushed)
`first_test_campaing/` → `first_test_campaign/` with `git mv` (918 renames).
Live references fixed in 14 files: the logger's `DEFAULT_CAMPAIGN`,
`config/test_campaign_logger.yaml`, `launch/test_campaign_logger.launch.py`,
`TEST_CAMPAIGN.md`, the test_campaign modules and tests, and the
`f1tenth_params` corridor test. Left as written: the recorded run data
(69 `meta.json` files hold `campaign_dir: .../first_test_campaing`) and the
historical reports in `output/`. Tests: `f1tenth_logger/test/test_campaign`
203 passed, `f1tenth_params/test/test_corridor_geometry.py` 38 passed.

**On the Orin:** any run recorded there after the pull lands in
`first_test_campaign/`. Unpushed runs still sitting in a local
`first_test_campaing/` on the Orin must be moved by hand.

---

## 1. Logs

### Today: four roots, no single setting

| Writer | Writes | Where today | Knob |
|---|---|---|---|
| `f1tenth_logger/mission_logger_node.py:483-491,652-659` | bag (mcap), manifest, params.yaml, parquet extract, snapshots; `active/` → `complete/` / `incomplete/` | `~/f1tenth_archive` | stack_params `mission_logger_runs_dir` (`stack_params.yaml:2032`), launch arg `mission_logger_runs_dir`, node param `runs_dir`; hardcoded fallback |
| `f1tenth_logger/runs_db.py:74-87`, `runs_cli.py:180,194` | `runs.db` | **`~/f1tenth_runs`** (env `F1TENTH_RUNS_DIR`) | disagrees with the logger, which keeps its `runs.db` in `~/f1tenth_archive` |
| `component_supervisor_node.py:498,552,692` | `<component>.log`, `tracked_pgids.json` | `~/.ros/log/component_supervisor` (stack_params `:2265`) | `log_dir` |
| `mpc_controller/model_log.py:46` via `MPC_corr.py:1288` | `mpc_log.csv` (truncated each start) | **CWD** | `model_log_path` / `mpc_model_log_path` (`:1428`) |
| `MPC_corr.py:83-136,1241-1253,1555-1590` | `mpc_odom_error.csv`, `mpc_control_compare.csv`, `corridor_debug_<stamp>.jsonl` | **`<ws>/src/f1tenth_control/corridors_jsons/`** (in the repo) | hardcoded; `save_corridor_debug = True` hardcoded |
| `MPC_corr.py:5720-5745` | `corridor_debug.jsonl` per test | `<campaign>/<mission>/<test_id>/` | only while `/test_campaign/logger_status` reports an open test |
| `f1tenth_behavior/mission/move_scoring.py:373-388,465` | mission report JSON | **`<ws>/src/f1tenth_behavior/mission_reports/`** (in the repo) | hardcoded |
| `llm/llm_planner_node.py:1070-1100` | generated mission JSON | install tree `share/f1tenth_behavior/missions/llm_generated/` | hardcoded |
| `f1tenth_logger/mission_render.py`, `mission_replay_video.py`, `mission_debug_video.py` | `.mp4`, `.debug.mp4` | **`<ws>/mission_videos/`** (in the repo) | `--out-dir`; legacy bag root `~/.ros/mission_bags` |
| `vesc_tuning/speed_sweep_diagnostic_node.py:52-75` | `speed_sweep_<ts>.csv` | **inside the vesc submodule** | hardcoded |
| `f1tenth_diagnostics/steering_offset_calibration_node.py:376` | raw dumps | `~/.ros/steering_offset_calibration` | `raw_dump_dir` |
| `f1tenth_perception/scripts/verify_seg_live_deploy.py:164,205` | PNGs | `/tmp/` | hardcoded |
| test-campaign logger `robot_logger.py:695-713` | per-test CSV/JSONL, `meta.json`, `campaign.json` | `<repo>/first_test_campaign/` | params `root` (env `F1TENTH_MORE_ROOT`) and `campaign`; `DEFAULT_CAMPAIGN` |

`~/f1tenth_archive` exists on linus (mission_bags, records, runs.db,
test_campaign, …). `~/f1tenth_runs` does not exist.

### Proposal

1. **One runs root: `~/f1tenth_runs`.** Rename the stack param
   `mission_logger_runs_dir` → `runs_root` (keep the old key as a deprecated
   alias for one release), default `"~/f1tenth_runs"`, and expose it as a
   bringup launch arg `runs_root`. Everything else derives from it:

   | Output | New default |
   |---|---|
   | mission bags + manifests | `<runs_root>/{active,complete,incomplete}/` (unchanged layout) |
   | `runs.db` | `<runs_root>/runs.db` (`runs_db.default_runs_dir()` reads `runs_root`; `F1TENTH_RUNS_DIR` stays as override) |
   | MPC debug CSV/JSONL (`corridors_jsons`) | `<runs_root>/debug/mpc/` |
   | `mpc_log.csv` | `<runs_root>/debug/mpc/mpc_log.csv` (no more CWD) |
   | mission reports | `<runs_root>/mission_reports/` |
   | rendered videos | `<runs_root>/videos/` |
   | speed sweep CSVs | `<runs_root>/debug/vesc/` |

   Component supervisor logs stay in `~/.ros/log` (ROS convention).
   The generated LLM missions stay in the install tree (they are inputs,
   not logs).

   **DECIDE — existing archive.** The car and linus have real data in
   `~/f1tenth_archive`. Options: (a) `mv ~/f1tenth_archive ~/f1tenth_runs`
   on each machine once; (b) set `runs_root: ~/f1tenth_archive` in the
   Orin's local params and only change the default; (c) a symlink.
   Recommendation: (a), with `runs_cli` keeping `--archive-dir` able to read
   the old path. Note `tools/test_campaign_backfill.py` and `runs_cli`
   hardcode `~/f1tenth_archive/complete`; they switch to `runs_root`.

2. **Campaign profile.** Campaign output already only appears when the
   test-campaign logger runs (started by hand with
   `test_campaign_logger.launch.py`). Make that explicit: a bringup launch
   arg `profile:=normal|campaign` (default `normal`). `campaign` includes
   `test_campaign_logger.launch.py` with `campaign:=first_test_campaign`;
   `normal` does not start it, so nothing writes into the repo.
   `DEFAULT_CAMPAIGN` stays as the code-level default of the campaign logger.

3. **`.gitignore`.** Add the old in-repo log locations so a stale build
   cannot dirty the tree:
   ```
   src/f1tenth_control/corridors_jsons/   # already present (line 58)
   src/f1tenth_behavior/mission_reports/  # already present (line 76)
   mission_videos/                        # already present (line 53)
   mpc_log.csv                            # new; today only *.csv covers it
   output/                                # new; folder emptied in step 2
   ```
   **DECIDE — `first_test_campaign/`.** You asked for it on jazzy's
   `.gitignore`. On campaign-final the folder is tracked data: ignoring it
   keeps the 918 tracked files but makes every *new* run need `git add -f`.
   Recommendation: on campaign-final ignore only the bulk per-test files
   (`first_test_campaign/**/corridor_debug.jsonl`, `horizon.jsonl`) and keep
   CSV/JSON/PNG committable; on jazzy ignore the whole folder as asked.
   Also fix the stale comment at `.gitignore:51`
   (cites `scripts/mission_replay_video.py`, now an `f1tenth_logger` CLI).

### References to update (logs commit)
`stack_params.yaml:2032-2034,1426-1430,2265`; `mission_logger.launch.py:44-48,77`;
`mission_logger_node.py:483-491`; `runs_db.py:74-87`; `runs_cli.py:180,194`;
`runs_migrate.py`; `MPC_corr.py:83-136,1241-1253,1288,1555-1590`;
`mpc_corr.launch.py:249-252,308`; `model_log.py`; `move_scoring.py:373-388,465`;
`mission_render.py:61,80-93`, `mission_replay_video.py:108`,
`mission_debug_video.py:180,207`; `speed_sweep_diagnostic_node.py:52-75`
(submodule: needs a commit in `vesc` too — **DECIDE** whether to touch the
submodule or leave it); `tools/test_campaign_backfill.py:87`;
`scripts/watch_objects.py:84` (its "refuses archive paths" guard);
bringup launch for `profile`; `TEST_CAMPAIGN.md`; tests asserting these
defaults (CLAUDE.md: a default in stack_params is a coupling — update the
test in the same commit).

---

## 2. output/

On campaign-final `output/` holds 5 tracked Markdown reports, 134,517 B.
No scripts, no data, nothing untracked or ignored. No code, launch file,
doc or README outside `output/` references any of them.

| path | size | last commit | referenced by | class | target / reason |
|---|---:|---|---|---|---|
| `output/campaign_final_report.md` | 15.4 KB | f7df8a7 2026-10-13 | — (it is the index of the 3 appendices) | MOVE | `docs/reports/`; fix its "appendices under output/" wording |
| `output/campaign_final_dirty_diff.md` | 79.0 KB | a940ed8 2026-10-13 | campaign_final_report.md:19 | MOVE | `docs/reports/`; appendix of record |
| `output/campaign_final_runs_table.md` | 11.4 KB | a940ed8 2026-10-13 | campaign_final_report.md:18 | MOVE | `docs/reports/`; appendix A |
| `output/campaign_final_virtual_wall_review.md` | 24.5 KB | a940ed8 2026-10-13 | campaign_final_report.md:20 | MOVE | `docs/reports/`; appendix H; its link to `campaign_partB_backlog.md` dangles today (fixed by the import below) |
| `output/scene_labels_replay.md` | 4.2 KB | 3ffe245 2026-10-13 | — | MOVE | `docs/reports/`; hand-written validation report |
| `orin_check_report.md` (tag `archive/orin-snapshot-20261013`) | 45.7 KB | 8c9e29b | docs/branches.md | MOVE (fixed) | `docs/reports/` via `git show archive/orin-snapshot-20261013:output/orin_check_report.md` |
| `jazzy_migration_analysis.md` (branch `jazzy`) | 40.5 KB | 2a57817 2026-09-30 | docs/branches.md | MOVE (fixed) | `docs/reports/` via `git show origin/jazzy:output/jazzy_migration_analysis.md` |

**DECIDE — more reports from the Orin tag.** `archive/orin-snapshot-20261013`
also has `campaign_fix_plan.md`, `campaign_partA_review.md`,
`campaign_partB_backlog.md`, `campaign_review_items.md`,
`mission_recorder_late_start.md` (37 KB total). Two are cited by reports
above (`campaign_partB_backlog.md`, `mission_recorder_late_start.md`).
Recommendation: import all five into `docs/reports/` so the links resolve.

After the moves `output/` is empty and is ignored (see logs).
`docs/branches.md` (still untracked on linus) also goes into `docs/` with
the next docs commit, its `output/…` paths updated.

**DELETE: none.**

The jazzy branch's `output/` (9,903 files, ~72.5 MB: build logs, `.npz`,
mcap bags under `fix_batch_*`) is out of scope here and needs its own pass.

---

## 3. tools/

### Layout (as requested, with three adjustments)

```
tools/README.md            one line per script + needs (car / sim / none) + old→new path table
tools/nodes/<pkg>/         per-package helpers; real package names (f1tenth_perception, not perception)
tools/tests/code/          code-only checks; no car, no sim
tools/tests/sim/           needs gz sim (empty on campaign-final; jazzy's scripts/sim/* would land here)
tools/tests/car/           on-vehicle checks
tools/analysis/            plots, replays, run reports, campaign metrics
tools/analysis/common/     shared loaders: bags.py, frames.py, rundb.py
docker/                    container build/run scripts (not tools/)
```

Adjustments:
1. **Installed CLIs stay in their packages.** The MP4/debug-image renderer
   is `f1tenth_logger.mission_debug_video` (console script, tested,
   imported by `scene_labels_replay`). Same for `mission_render`,
   `mission_replay_video`, `mission_extract`, `test_campaign_analyze`,
   `test_campaign_corridor_plot`. Moving them breaks `ros2 run`, tests and
   imports. They get a "installed CLIs" section in `tools/README.md`.
2. **`src/f1tenth_bringup/scripts/ensure_discovery_server.py` does not move.**
   It is installed by `setup.py:25` and executed by
   `stack_bringup.launch.py:108`, `supervisor_bringup.launch.py:97` and
   `steering_offset_calibration_node.py:596`.
3. **`tools/tests/code/` is not run by `colcon test`** (it only tests `src/`
   packages — see CLAUDE.md "Ran 0 tests"). Real unit tests stay in the
   packages' `test/`. `tools/tests/code/` holds standalone check scripts,
   run with `pytest tools/tests/code` (documented in the README). Phase 2
   runs it explicitly.

### Mapping

needs: car = running stack on the vehicle; sim = gz sim; none = offline (+ws = sourced workspace).

| current | needs | target | references to update |
|---|---|---|---|
| `scripts/check_cpu_pinning.py` | car | `tools/tests/car/check_cpu_pinning.py` | `viz_relays.launch.py:23`, `f1tenth_bringup/setup.py:21` (comment), `stack_params.yaml:2226`, `docs/history/LOG.md:334` (historical, leave), self-usage |
| `scripts/check_ekf_update_rate.py` | car | `tools/tests/car/check_ekf_update_rate.py` (or DELETE, see merges G) | `ekf.launch.py:27,45`, `ekf_global.launch.py:33`, `f1tenth_bringup/setup.py:20` (comment), self-usage |
| `scripts/check_floor_values.py` | car +ws | `tools/tests/car/check_floor_values.py` | `docs/procedures/2026-09-18_go_to_floor.md:75,167`, self-usage |
| `scripts/ekf_cost_report.py` | none +ws | `tools/analysis/ekf_cost_report.py` | self-usage |
| `scripts/kill_ros2.py` | none | `tools/nodes/f1tenth_bringup/kill_ros2.py` | `docs/procedures/2026-09-18_go_to_floor.md:61,82`, self-usage |
| `scripts/stackctl.py` | car/sim | `tools/nodes/f1tenth_bringup/stackctl.py` | **runtime string** `f1tenth_logger/test_campaign/logger_node.py:531`; `TEST_CAMPAIGN.md:60,68,396`; `obstacle_clearance.launch.py:8`; `docs/procedures/2026-09-17_floor_person_pass.md:76,138`, `…2026-09-18_go_to_floor.md:68`; self-usage |
| `scripts/watch_objects.py` | car/sim | `tools/nodes/f1tenth_behavior/watch_objects.py` | **test** `f1tenth_behavior/test/test_watch_objects.py:29` (`parents[3]/'scripts'`); `docs/procedures/2026-09-18_go_to_floor.md:21,88`; prose in `object_handler.py:107`, `test_object_handler.py:249`, `test_go_to_object_behaviour.py:186`, `ObjectApproachStatus.msg:68` |
| `scripts/replay_bag_through_mpc.sh` | none +ws | `tools/nodes/mpc_controller/replay_bag_through_mpc.sh` | **path-relative** `WS=$(dirname BASH_SOURCE)/..` → `/../../..` |
| `scripts/docker_build.sh` | none | `docker/build.sh` | `docs/setup/README_DOCKER.md:43-45`; path-relative `cd`; **already broken**: compose file is `.devcontainer/docker-compose.yml` → add `-f` |
| `scripts/docker_run.sh` | none | `docker/run.sh` | `README_DOCKER.md:48,61,84`; `docker_build.sh:25`; same compose fix |
| `tools/gen_intent_target_enum.py` | none (torch) | `tools/nodes/llm/gen_intent_target_enum.py` | **path-relative** `parents[1]` → `parents[3]`; **test** `llm/test/test_intent_go_to.py:171-189` (skips without torch — run on a torch box); generated header in `object_classes.py:3`; `mission_config.py:607`; `intent_v1.json:66` |
| `tools/measure_stop_distance.py` | none +ws | `tools/analysis/measure_stop_distance.py` | `stack_params.yaml:1335`; `test_object_approach_closed_loop.py:104`; self-usage |
| `tools/mpc_model_check.py` | none | `tools/nodes/mpc_controller/mpc_model_check.py` | **test** `mpc_controller/test/test_model_log.py:16` (`parents[4]/'tools'`); `README.md:34`; `docs/DIAGNOSTICS.md:8,136,141-148`; comments `mpc_corr.launch.py:245`, `MPC_corr.py:1259`, `model_log.py:4`, `swept_clearance.py:11`; `stack_params.yaml:739,1426,1430` |
| `tools/requirements.txt` | – | stays `tools/requirements.txt` (one file for all tools) | `docs/DIAGNOSTICS.md:136` unchanged |
| `tools/plot_corridor.py` | none | `tools/nodes/mpc_controller/plot_corridor.py` | **path-relative** `Path(__file__).parent.parent` → repo-root finder; default path moves with the logs change; `corridor_plot.py:7-8` prose |
| `tools/scene_labels_replay.py` | none +ws | `tools/analysis/scene_labels_replay.py` | `docs/reports/scene_labels_replay.md:3` |
| `tools/test_campaign_backfill.py` | none +ws | `tools/analysis/campaign_backfill.py` (**renamed**: `test_*.py` is collected by a bare `pytest` and errors on its module-level rclpy import) | `TEST_CAMPAIGN.md:192,244`; `export_campaign_csv.py:30`; `first_test_campaign/backfill.json:3` is recorded provenance — leave |
| `tools/wall_distance_replay.py` | none +ws | `tools/analysis/wall_distance_replay.py` | `docs/bringup_checklist.md:231`; `f1tenth_perception/README.md:81`; self-usage |
| `tools/wall_track_replay.py` | none +ws | `tools/analysis/wall_track_replay.py` | `docs/wall_turn_investigation.md:41,213,584,590`; `wall_distance_replay.py:11,22,54` |
| `docs/analysis/chase_overspeed_ablation.py` | none | `tools/analysis/chase_overspeed_ablation.py` | `parents[2]` keeps its depth; `docs/analysis/2026-09-16_chase_overspeed.md:5` |
| `docs/analysis/person_pass_margin.py` | none | `tools/analysis/person_pass_margin.py` | same depth; `docs/analysis/2026-09-17_person_pass_margin.md:8` |
| `docs/analysis/go_to_live_eval.py` | none (llama-server) | `tools/nodes/llm/go_to_live_eval.py` | **path-relative** `parents[2]` → `parents[3]`; `docs/analysis/2026-09-17_go_to_live_eval.md:6`; `llm/test/test_intent_evals_live.py:24` |
| `src/f1tenth_perception/scripts/benchmark_yolo_latency.py` | GPU | `tools/nodes/f1tenth_perception/benchmark_yolo_latency.py` | **path-relative** models dir → `src/f1tenth_perception/models`; `verify_seg_live_deploy.py:5-36`; `test_detection_launch_config.py:25` |
| `src/f1tenth_perception/scripts/verify_seg_live_deploy.py` | GPU | `tools/tests/car/verify_seg_live_deploy.py` | **path-relative** `rsplit('/scripts/')` silently breaks; `test_detection_launch_config.py:18,23`; `/tmp` PNG output → `--out` |
| `src/scripts/run_container.sh` | – | **DELETE** | none (upstream foxy leftover, wrong image and workspace) |
| `src/scripts/resume_container.sh` | – | **DELETE** | none |
| `src/f1tenth_bringup/scripts/ensure_discovery_server.py` | – | stays | – |
| `docker/entrypoint.jetson.sh` | – | stays | COPY'd by `Dockerfile.jetson-thor:265` |

Path-relative scripts get one shared idiom instead of `parents[N]`:
`REPO = next(p for p in Path(__file__).resolve().parents if (p / 'src').is_dir())`.

### Scripts from earlier Claude Code sessions
No stray scripts in `output/`, `first_test_campaign/`, `docs/` (other than
the three `docs/analysis/*.py` above) or ignored dirs on campaign-final.
- `output/campaign_final_dirty_diff.md` cites `dirtydiff/steer_minmax.py`
  and `partA/dirty/servo_gain_readout.py`. They only exist in an old session
  scratchpad under `/tmp/claude-1000/…/scratchpad/`. Recover them into
  `tools/analysis/` if that scratchpad still exists at Phase 2, otherwise
  note them as lost in `tools/README.md`.
- `first_test_campaign/analysis/M02_20261008_R009-R011_why_it_stopped.png`
  and `M02_20261008_R011_camera_roi.png` have no generator in the repo.
- Every moved script gets a docstring with inputs, outputs and one example
  command (most have one; `wall_track_replay.py`, `go_to_live_eval.py`,
  `docker_*.sh` are partial).

### Merges (near-duplicates)
- **A. Bag readers → `tools/analysis/common/bags.py`.** Seven readers:
  sqlite in `measure_stop_distance` (all splits, `mode=ro`),
  `scene_labels_replay` (**first `.db3` only** — split bags silently
  truncated), `test_campaign_backfill` (hardcoded `bag_0.db3`);
  rosbag2_py in `wall_track_replay` / `wall_distance_replay` (near-copies,
  hardcoded `sqlite3`, fail on mcap); `ekf_cost_report` (run-or-bag dir,
  storage from `metadata.yaml`). Survivor: ekf_cost_report's resolution +
  storage detection, plus measure_stop_distance's all-splits read-only
  sqlite path (keep the read-only invariant). jazzy has
  `tools/campaign_bag.py` doing the same — reconcile when jazzy merges.
  Package-side readers in `f1tenth_logger` stay.
- **B. Frames → `common/frames.py`:** `yaw_of`, laser pose from
  `/tf_static`, the wall replays' `dead_ahead`. **Not**
  `measure_stop_distance._dead_ahead` (different filter, source of a
  deployed parameter).
- **C. Simulated commit rule** shared by both wall replays →
  `common/frames.simulated_commit()` so they cannot drift.
- **D. Run lookup → `common/rundb.py`**, a thin wrapper over
  `f1tenth_logger.runs_db` (replaces `find_archive_run`'s own glob).
- **E. `plot_corridor.py`** imports drawing/style from the package
  `corridor_plot` instead of copying the palette.
- **G. `check_ekf_update_rate.py` vs `ekf_cost_report.py`:** ekf_cost_report
  survives; **DECIDE** delete check_ekf_update_rate or keep it under
  `tests/car` as legacy.
- Not merged: the three replay harnesses (share `common` only), the two
  mpc rig-table scripts (each pins a dated report), the two perception GPU
  scripts (share one models-dir resolver).

`common` import: scripts directly in `tools/analysis/` import
`common.bags` as-is; scripts elsewhere in `tools/` use a 2-line
`sys.path` bootstrap to `tools/analysis`. **DECIDE** if you'd rather put
`common` into `f1tenth_logger` (which already owns `runs_db`).

---

## 4. Phase 2 order

1. **logs** — `runs_root`, `profile`, in-repo writers out, `.gitignore`, tests.
2. **output/** — `git mv output/*.md docs/reports/`, import the Orin and
   jazzy reports, fix links, add `docs/branches.md`.
3. **tools/** — `git mv` per the mapping, `common/`, merges, docstrings,
   `tools/README.md`, delete the two container leftovers and the
   `__pycache__` dirs.
4. **path fixes** — every reference above.

Then `colcon build`, `colcon test` for the touched packages,
`pytest tools/tests/code`, a grep for every old path (only historical docs
may still match), push `campaign-final`.

**jazzy:** only the `first_test_campaign/` ignore and the `runs_root`
default. The tools move is **not** ported. Known conflicts it would cause:
jazzy has 94 scripts in `scripts/`/`tools/` that campaign-final does not
(`scripts/jazzy_parity/*` ~55, `scripts/fix_batch_*/*`,
`scripts/sim/*.sh`, `scripts/env/jazzy.sh`, `tools/campaign_bag.py`,
`tools/run_scenarios.sh`, …), 4 modified in both, 1 deleted. The exact
list goes into the Phase 2 report.

---

## Full DELETE list

| item | reason |
|---|---|
| `src/scripts/run_container.sh` | upstream F1TENTH leftover; `f1tenth/focal-l4t-foxy` image, `~/f1tenth_ws`; nothing references it |
| `src/scripts/resume_container.sh` | same leftover; nothing references it |
| `scripts/__pycache__/`, `tools/__pycache__/` | untracked bytecode |
| `output/` (directory) | empty after its 5 reports move to `docs/reports/`; then ignored |
| `scripts/check_ekf_update_rate.py` | **only if you choose so** (merge G) |

Nothing in `output/` and nothing in `first_test_campaign/` is deleted.
