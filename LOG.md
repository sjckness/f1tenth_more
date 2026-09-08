# Project Log — f1tenth_more

Running "what happened and why" history. Reverse-chronological, one entry per event.

**No `CLAUDE.md` exists in this repo yet** (checked 2026-09-02). If one is added later
it is the "current conventions" doc — this file stays the history and the two should
not be merged.

**Reconciliation status.** Every entry below was checked against real git history on
2026-09-02. Commit hashes and dates are real (`git log`, `git show`, `git blame`).
Entries marked *uncommitted* exist only in the working tree — HEAD is `580a2af`
(2026-08-31) on branch `scene-graph`, and the tree carries **50 modified files
(+3321/−563), 2 staged deletions, and ~18 untracked files** on top of it. Dates for
uncommitted entries come from file mtimes and are marked as such; they are weaker
evidence than a commit date. Entries marked *not found in git history* could not be
grounded at all — see the end of this file.

`origin` is `github.com/sjckness/f1tenth_more`, last fetched 2026-08-11 and **14
commits behind** this workspace. This workspace is authoritative; the mirror is stale.

---

### 2026-09-08 — `transform_time_offset` goes to 0.08 on BOTH edges, not a 0.12/0.08 split [decision]
tags: decision
- Context: the 50Hz -> 20Hz EKF retarget widens each filter's publish period from 20ms
  to 50ms, so `transform_time_offset` (0.05 in both `ekf.yaml` and `ekf_global.yaml`)
  has to be re-derived or scans start landing ahead of the newest TF stamp. The
  proposal on the table was to split the edges: 0.12 for map -> odom (which moves at
  correction rates, so a long post-date costs almost nothing in position) and 0.08 for
  odom -> base_link (which carries vehicle motion, where 120ms is 36cm at 3 m/s).
- Measured first, across 5 healthy archived runs (`12-48-40` excluded: its D values run
  to thousands of ms, independently confirming that run's known 95% /tf loss).
  `D = scan_stamp - newest_TF_stamp`, so `D > 0` means the scan leads the transform:
  map -> odom is positive for **3.2-50.0%** of scans, odom -> base_link for 0.3-4.9%.
  The margin is therefore ALREADY violated today at 50Hz, which the "0.05 clears the
  28ms need by 2ms" reasoning did not predict.
- There is also no single ~28ms scan lead. It is ~13-14ms on odom -> base_link and
  ~27-34ms on map -> odom, and the difference is publish-period JITTER, not sensor
  stamping.
- The deciding question was whether positive D costs loss or latency. It costs
  **latency**:
  - `slam_toolbox_params.yaml:73` sets `transform_timeout: 0.2` — 200ms of tolerance
    against a worst measured D of +64.8ms, so 135ms spare. A scan that leads the
    newest transform waits one publish period and then succeeds.
  - Zero `extrapolation into the future` in any component log. The only three greps
    that hit were `static_transform_publisher` `list_parameters (timeout)` service
    warnings, unrelated to TF lookups.
  - slam_toolbox dropped exactly **10** scans in the session, ALL of them for
    `'the timestamp on the message is earlier than all the data in the transform
    cache'` — extrapolation into the PAST, the opposite direction. ~0.1% of scans,
    clustered at startup and just after the perception restart.
  - costmap and navigation logs contain zero transform complaints of any kind.
- Consequence: 0.08 on both edges, 0.12 dropped. Raising the post-date further would
  make the one drop mode actually observed slightly WORSE, since post-dating shifts the
  whole transform cache forward in time and "earlier than all the data in the cache" is
  precisely a too-old-stamp failure.
- **This is JITTER MITIGATION, not a fix.** The real defect is the global EKF's p99
  publish period of 36-55ms against a 20ms nominal. At 20Hz the nominal becomes 50ms
  with jitter on top, so the offset must be revisited against `ekf_cost_observer_node`'s
  own `period_ms_p90`/`period_ms_max` once a baseline run exists.
- status: open — decided and evidenced, NOT yet applied. Lands with the frequency
  change, which is itself gated on capturing the 50Hz cost baseline first.
- commit: measurement instrument landed as `1635aed`; the offset change itself is pending

### 2026-09-02 — Live `w_psi` sweep on hardware refutes the modeled tuning [decision]
tags: decision
- 13 mission bags recorded 15:13–15:38 UTC. The MPC node was restarted 14 times and
  `w_psi` edited between restarts (1.0 → 5.0 → 2.0 → 5.0 → 10.0), so the session is a
  sweep, not the flat validation of `w_psi = 1.0` it was planned as. Reconstructed from
  each node's `CFG | weights=` startup log; `params_snapshot` cannot show it because
  `w_psi` is a source literal in `MPC_corr.py`, not a ROS param.
- Result: 1.0 is under-damped on the real car (lateral excursions to 2.29 m, horizon
  bends back toward `psiRef` on only 50–65 % of deflected ticks). 5.0 is the best
  performer (0.25–0.69 m, 82–100 %). The source tree now reads `"w_psi": 5.0` while the
  comment block above it still argues for 1.0.
- Written up in `mission_analysis_2026-09-02_live_validation.md` (untracked).
- status: open — comment block and code disagree; controlled re-run still owed
- commit: uncommitted (`MPC_corr.py` mtime 2026-09-02 17:38; analysis .md untracked)

### 2026-09-02 — False-COMPLETE bug recurred, now root-caused [bug]
tags: bug
- Both `llm_plan` runs reported `outcome: COMPLETE` without finishing their last move
  (`traveled` 1.804 / 1.777 of a 2.0 m goal, no `Goal raggiunto` in either log).
- Root cause: `mission/condition_eval.py` `distance_reached` uses raw Euclidean
  `math.hypot(current − move_start)`, while `MPC_corr._project_onto_line` terminates on
  the along-line projection. The two disagree by the lateral excursion — up to 1.008 m
  on a 6 m move in these runs, 13 % early on the 2.0 m move.
- Fix proposed but not applied: use the along-line projection, or consume
  `/mpc/goal_reached`, so there is one definition of "arrived".
- status: open
- commit: uncommitted (diagnosis only; no code change made)

### 2026-09-02 — Instrumentation for the intermittent stop-then-resume thread landed [fix]
tags: fix
- Built as two new messages plus a recorder, **not** as the `/mpc/diagnostics` topic the
  original plan called for: `MpcSolverStatus.msg` (solver success/status/`solve_dt_sec`
  + predicted horizon `pred_x/y/yaw/v`), `BehaviorTreeStatus.msg` (`active_lane`,
  `lane_names`/`lane_statuses`, `emergency_trip`, `safety_stop_active`, `stop_source`),
  and `f1tenth_diagnostics/mission_logger_node.py` with `mission_logger.launch.py`.
- Both replace log archaeology: stop cause and solver convergence are now readable from
  a bag alone. Confirmed working across 13 runs.
- The planned **portable CSV/Parquet export is absent** — no `to_csv`/`parquet` anywhere
  in `f1tenth_diagnostics`. Raw bags only.
- Known defect found in use: `active_lane` is `''` on 98 % of ticks, because it only
  reports a child that returned SUCCESS and a `RUNNING` mission lane returns neither.
- status: open (export missing; `active_lane` defective)
- commit: uncommitted (`MpcSolverStatus.msg` mtime 2026-09-02 13:02,
  `mission_logger_node.py` 13:03, `BehaviorTreeStatus.msg` 2026-09-01 18:11)

### 2026-09-02 — CLAUDE.md creation dispatched — did not land [plan-change]
tags: plan-change
- A task to distill repo conventions/gotchas into a root `CLAUDE.md` was dispatched.
- No `CLAUDE.md` exists at repo root, in any commit, or in the working tree.
- status: open — not started, or started and lost
- commit: not found in git history

### 2026-09-02 — Second "no images" report, same root cause [bug]
tags: bug
- Reported again, with a new symptom alongside it (semantic markers also missing). A
  combined debug prompt (discovery-server state → node liveness → full chain debug) was
  dispatched.
- No artifact of that investigation exists in the repo — no notes, no script, no commit.
  Whether it ran and what it found is unrecorded.
- status: open
- commit: not found in git history

### 2026-09-01 — Lidar floor tightened 0.40 m → 0.15 m; camera e-stop lane made optional [decision]
tags: decision
- `IsProximityTooClose` front and side thresholds both dropped to **0.15 m** (HEAD still
  has 0.40 / 0.20). Rationale in the code: hand the 0.40–0.15 m band to MPC avoidance so
  the raw-sensor floor stops competing with it, and only fires once avoidance has failed.
- New params: `enable_camera_obstacle_stop` (default **false** — camera detections now
  influence the car through MPC soft cost, not by stopping) and `enable_lidar_safety_stop`
  (default true, deliberately kept as the floor).
- Validated on hardware 2026-09-02: 2 genuine trips in 13 runs, 0 false positives.
- Note: the brief's "0.40 m threshold (0.20 + 0.12 + 0.08)" describes the **committed**
  state, which this change supersedes.
- status: resolved
- commit: uncommitted (`is_proximity_too_close.py` mtime 2026-09-01 18:18)

### 2026-09-01 — GPU-load false-positive e-stops fixed by splitting load from temperature [fix]
tags: fix
- The 2026-09-01 bag analysis found `IsSystemOverheated` was the third-largest cause of
  unexpected stops — 3 of 9 episodes — every one a false positive from the *load* half
  (gpu_percent 96.8–99.1 for 1–2 samples at 1 Hz, while temperatures sat at 55 °C CPU /
  49 °C GPU against a 100 °C limit).
- Fix: temperature still trips on a single sample with no debounce; load now requires
  `load_trip_consecutive_samples` consecutive over-threshold samples (default 3) and can
  be disabled independently via `enable_load_trip`.
- Caveat found 2026-09-02: `enable_sys_obs_load_trip` is set **false**, so the lane was
  off entirely during validation — the debounce itself is still untested, and no thermal
  event occurred either.
- status: open (fix in place, unverified)
- commit: uncommitted (`is_system_overheated.py` mtime 2026-09-01 18:03)

### 2026-09-01 — Discovery server absent entirely = total visibility loss [bug]
tags: bug
- Distinct from the known partial-visibility DDS race: if the discovery server process
  isn't running at all, every participant is invisible to every other. Diagnosed as the
  cause of two "no images" reports (2026-09-01, 2026-09-02) — not a code regression.
- Confirmed in the tree: `src/f1tenth_bringup/scripts/ensure_discovery_server.py` exists
  and is referenced by exactly two launch files — `stack_bringup.launch.py` and
  `supervisor_bringup.launch.py`. The other **33 launch files have no guard**, so any
  per-component launch can come up into a dead discovery domain.
- status: open — guard not extended to per-component launches
- commit: uncommitted (guard predates HEAD; the gap is unactioned)

### 2026-09-01 — stack_bringup.launch.py found NOT retired; deletion dispatched [plan-change]
tags: plan-change
- The 2026-08-19 decision declared `supervisor_bringup.launch.py` the only bringup path.
  Both files turned out to be independently live and correct. A full deletion was
  dispatched (parity audit → migrate gaps → fix all refs incl. docs/tests → delete →
  verify build+tests+zero grep hits).
- **It did not land.** `stack_bringup.launch.py` still exists (15741 bytes, modified
  2026-08-31), and ~20 files across `src/` still reference it.
- status: open — superseded in intent, not in fact
- commit: not found in git history

### 2026-09-01 — Semantic tracks now age out with explicit DELETE markers [fix]
tags: fix
- `/costmap/semantic_markers` emits `action=2` (DELETE) when a track leaves the scene,
  ending the phantom-marker persistence. Verified on hardware 2026-09-02: 38 ADDs / 6
  DELETEs in one run, no track surviving its object.
- status: resolved
- commit: uncommitted (`semantic_layer.py` mtime 2026-09-02 10:53)

### 2026-08-31 — LLM planner integrated end-to-end [fix]
tags: fix
- `llm_planner_node`: manual CLI tool (never auto-launched persistent), natural language
  → mission JSON via a local LLM, calling the abort/load/start mission services.
- Backend: llama-server, stock Qwen2.5-3B-Instruct q5_k_m, port 8083, raw
  `POST /completion`, no ChatML, no grammar constraint. JSON parsing hardened with
  `raw_decode` to tolerate trailing garbage after the closing brace.
- The old `llm_mpc_tuner_node` removal is **staged but not committed** (−426 lines,
  plus `mpc_tuner_params.yaml`) — the brief's "confirmed complete" is premature.
- Brief correction: `enable_intelligence` defaults to **true**, not false. The key does
  not exist in HEAD's `stack_params.yaml` at all; it is new and uncommitted.
- status: resolved
- commit: 580a2af (2026-08-31)

### 2026-08-31 — LLM cold-start and llm_url both fixed (brief was stale) [fix]
tags: fix
- Brief lists both as open. Both are done in the working tree:
  - `LLAMA_TIMEOUT` raised 60.0 → **90.0**, and `_wait_for_llama_server()` added as a
    real readiness + warm-up pass, directly addressing the 58 s-against-60 s cold start.
  - `llm_url` is a declared ROS param overridden at launch from `models.yaml`'s own
    resolved port — no longer hardcoded, no longer desynced if the port changes. The
    `--llama-url` CLI override proposed in the brief was not the shape adopted.
- status: resolved
- commit: uncommitted (`llm_planner_node.py` mtime 2026-08-31 15:22; +216 lines unstaged)

### 2026-08-31 → 2026-09-02 — YOLO switched to segmentation with mask-based depth fusion [decision]
tags: decision
- Per-pixel instance mask selects ZED depth pixels (median over in-mask valid pixels,
  box-method fallback when a mask has zero valid pixels). Ultralytics `.pt` runtime
  chosen over a hand-rolled TensorRT mask decode; the TensorRT port is deferred.
- Measured GPU latency drove the call: TensorRT box engine 25.4 ms, `yolo26s.pt` detect
  40.7 ms, `yolo26s-seg.pt` segment ~52.3 ms — deploy seg as-is.
- **The default detector HAS been flipped** (brief says it hadn't): `stack_params.yaml`
  HEAD = `yolo26s.engine`, working tree = `yolo26s-seg.pt` with `yolo_model_task: segment`
  and `use_mask_depth` auto-derived from the `-seg` filename so it is a single switch.
  `yolo26s-seg.pt` is present but untracked.
- Annotated-image mask overlay (`_draw_mask_overlay`) done, with tests.
- status: resolved (deployment), deferred (TensorRT seg port)
- commit: uncommitted (`yolo_detector_node.py` / `detection_3d_node.py` mtime
  2026-09-02 11:09, +525/+366 lines; model file and `test/`, `scripts/` dirs untracked)

### 2026-08-21 — Localization reaches its third form: one EKF → two (local + global) [decision]
tags: decision
- The final step of the three-stage localization evolution (see 2026-06-12 and
  2026-06-15). `76804f5` adds `ekf_global.yaml`, `ekf_global.launch.py`,
  `test_ekf_global_config.py` and `scripts/check_ekf_update_rate.py` — the first time
  two EKF instances exist at once.
- Split: **local** EKF in the odom frame (feeds `mpc_corr`), **global** EKF in the map
  frame (fuses local + `/slam/pose`, owns `map->odom`).
- Correcting a plausible misreading: `1099733` (2026-08-10) is *not* where the second
  EKF appeared. It only changed the single EKF's `world_frame` to `map` so it published
  `map->odom` instead of `odom->base_link`, with `odom->base_link` becoming a fixed
  static transform from `localization.launch.py`. One instance, new frame — the split
  into two came eleven days later.
- status: resolved
- commit: 76804f5 (2026-08-21)

### 2026-08-21 — CPU pinning switched from self-pinning to taskset prefix [fix]
tags: fix
- The `os.sched_setaffinity`-at-node-startup mechanism introduced on 2026-08-13 was
  replaced by a `taskset` prefix applied at launch, across **7 launch files**:
  `mpc_corr`, `detection`, `behavior_bringup`, `ekf`, `ekf_global`, `slam`,
  `foxglove_bridge`. This is the commit that actually fixed the self-pinning bug.
- `cpu_affinity.py`, the original self-pinning helper, was never deleted — it still
  exists in `f1tenth_perception`, and `behavior_executor_node.py` / `MPC_corr.py` still
  carry `sched_setaffinity` references in comments. Dead-ish code worth a look.
- status: resolved
- commit: 76804f5 (2026-08-21)

### 2026-08-21 — slam_toolbox promoted to its own package and config [fix]
tags: fix
- `slam.launch.py`, `slam_toolbox_params.yaml`, `test_slam_toolbox_params.py`,
  `slam_pose_relay_node.py` and `slam_pose_covariance_calibration_node.py` all first
  appear here. Note this is a *promotion*, not an adoption — slam_toolbox had been in
  the stack since 2026-05-30 (see that entry).
- status: resolved
- commit: 76804f5 (2026-08-21)

### 2026-08-21 — Straight-after-turn reference frame and 180° spin fixed [fix]
tags: fix
- Fixes the 2026-08-20 live-test failures directly: the 1 m straight after a 90° turn
  had used the old world x-axis instead of the new heading, and the 180° bounce turn
  didn't work (orientation_delta wraparound ambiguity at exactly 180).
- Touched `check_stop_condition.py`, `condition_eval.py`, `MPC_corr.py`, plus
  `test_condition_eval.py` (+63 lines).
- status: resolved
- commit: 2bd6420 (2026-08-21)

### 2026-08-21 — Abort-recovery bug fixed: `/mpc/hold` released on start_mission [fix]
tags: fix
- After `abort_mission`, a subsequent `load_mission` + `start_mission` reported success
  and reached RUNNING but the car never moved. Traced end-to-end: the mission state
  machine was **not** the blocker (`_load()` has no precondition on prior state); the
  blocker was `mpc_corr`'s own `self.hold` flag in a different process, set True by
  abort/COMPLETE/`on_timeout='stop'` and cleared only by `HandleObjectAction._resume()`.
- Fixed by releasing hold in start_mission's LOADED→RUNNING transition.
- Brief's hash and description verified exact.
- status: resolved
- commit: 6930c3c (2026-08-21)

### 2026-08-21 — Mission robustness pass [fix]
tags: fix
- Preflight liveness check, closed-loop per-move scoring, geometric edge-case regression
  tests, dependency audit. +1608 lines across 13 files, incl. `test_move_scoring.py`,
  `test_preflight.py`, `test_corridor_heading_reference.py`.
- These are the "robustness ideas accepted" from the PACEd-NAV thread, landed.
- status: resolved
- commit: f960cf2 (2026-08-21)

### 2026-08-21 — MPC corridor visualization; rebuild rate 1 Hz → 10 Hz [fix]
tags: fix
- Directly addresses one of the intermittent-stop candidate causes (corridor going stale
  against hard constraints at ~1 Hz).
- status: resolved
- commit: f3fa196 (2026-08-21)

### 2026-08-21 — VESC gyro_z 57.3× unit bug fixed [fix]
tags: fix
- Vendored `vesc_driver`'s `gyr_z()` fed deg/s raw into a rad/s field. Fixed by a
  `gyro_scale_z` gain applied after bias subtraction; `vesc.yaml` sets
  **`gyro_scale_z: 0.0174533`** (= π/180). `gyro_scale_x/y` left at 1.0 — same-source
  suspicion only, not measured.
- Retroactively explains most earlier drift/instability reports.
- **Andreas: ~2 weeks of work to find this.** His recollection, recorded as his account
  — nothing in git dates the investigation, only the fix.
- Caveat: the fix is committed **in the submodule** (`1e6a734`, 2026-08-21). The parent
  repo's submodule pin (`6ab1dde`, 2026-08-13) predates it, and
  `src/f1tenth_hardware/vesc` shows dirty in `git status` — the pointer bump is not
  committed, so a fresh clone of the parent does not get this fix.
- status: resolved (submodule) / open (parent pin not bumped)
- commit: 1e6a734 in `src/f1tenth_hardware/vesc` (2026-08-21); parent bump uncommitted

### 2026-08-21 — Session checkpoint: dual-EKF, costmap boundaries, mission logic, calibration gates [fix]
tags: fix
- Large consolidating commit. Confirmed to contain, among others:
  - **Dual-EKF architecture** — local EKF (odom frame, feeds `mpc_corr`) + global EKF
    (map frame, fuses local + `/slam/pose`, owns `map->odom`). `ekf.yaml` +
    `ekf_global.yaml`.
  - **`mission_config._parse_move()` type-check fix** — `isinstance` guards, so a
    malformed `goal_distance` no longer raises an uncaught `TypeError` that killed the
    whole `behavior_executor_node` process (taking the emergency-stop and obstacle-stop
    safety lanes down with it). Catch-all `except Exception` net in `loader.py._load()`
    confirmed present in HEAD.
  - **`battery_voltage_check_node` startup-race fix** — `max_wait_for_first_sample_sec`
    (default 10.0 s) added. Brief says "real fix not yet applied"; it **was** applied here.
  - **`wall_detector_node` / `lidar_boundary_node` retired** for `costmap_boundary_node`
    (only `.pyc` remain).
  - `scripts/check_cpu_pinning.py` added.
- status: resolved
- commit: 76804f5 (2026-08-21)

### 2026-08-20 — Live test: map good, 90° turn OK, straight-after-turn and 180° bounce failed [bug]
tags: bug
- Map built correctly; object-detection duplication improved but not resolved; the 90°
  turn executed, but the following 1 m straight used the old/world x-axis instead of the
  new heading; the 180° bounce turn didn't work (suspected orientation_delta wraparound
  ambiguity at exactly 180).
- No commit on this date. The failures are fixed the next day by `2bd6420`; the
  detection-duplication item has no follow-up commit and is unresolved.
- status: resolved (turn/straight) / open (detection duplication)
- commit: not found in git history (test event, no commit; fix is 2bd6420)

### 2026-08-19 — supervisor_bringup declared the only bringup path [decision]
tags: decision
- Superseded 2026-09-01 when `stack_bringup.launch.py` was found still live and correct.
- status: superseded
- commit: 8dcee01 (2026-08-19, calibration safety gates Phase A — the bringup decision
  itself has no dedicated commit)

### 2026-08-13 to 2026-08-19 — Auto-calibration analyzed, hardened, then dropped [decision]
tags: decision
- Hand-calibrated IMU values kept; `calibration` default **false** in
  `stack_params.yaml`. Setting it true reintroduces a separate TF-race bug.
- Five `vesc.yaml.bak.20260819T*` backups on disk are the artifacts of the calibration
  runs that led to the decision.
- status: deferred (TF race under `calibration:=true` still open)
- commit: 8dcee01 (2026-08-19) + 76804f5 (2026-08-21)

### 2026-08-13 — Turn move type: TurnGoal.msg + goal_turn_callback [fix]
tags: fix
- `goal_turn_callback` in `mpc_corr` converts a signed `heading_delta_deg` into a
  synthetic `goal_pose` via the Ackermann turn-radius relationship. New `TurnGoal.msg`.
- **Brief is stale**: it says "not committed as of last note". Both are committed —
  `TurnGoal.msg` in `e2dbe78`, `goal_turn_callback` in `7f893d4`, both 2026-08-13. It was
  also live-tested on 2026-08-20 and fixed on 2026-08-21.
- status: resolved
- commit: e2dbe78 + 7f893d4 (2026-08-13)

### 2026-08-13 — CPU pinning's actual trigger: /camera/detections lagging by ~278 ms [bug]
tags: bug
- The symptom that motivated all the pinning work is recorded, but **only inside the fix
  commit itself** — there is no earlier commit or issue capturing it. `cd0e2f4`'s body:
  the pinning followed *"a latency audit that found /camera/detections lagging
  /camera/image_raw by ~278 ms under CPU contention."*
- That audit produced `cpu_affinity.py` — a shared helper calling `os.sched_setaffinity`
  **self-applied at node startup** for `yolo_detector_node`, `detection_3d_node` and
  `obstacle_projector_node`. This is the mechanism that later turned out to be the
  self-pinning bug, replaced by the taskset prefix on 2026-08-21.
- **The MPC 93 ms → 12.3 ms figure does not belong to this work at all.** See the
  2026-08-10 solver-swap entry: `0f24e66`'s own body attributes the 93 ms to *SLSQP*
  and names RTI/OSQP as the fix, three days before any pinning existed. The pinning
  entries in an earlier draft of this log implied otherwise; corrected.
- Of the perception numbers, only the ~278 ms detection lag is recorded in git. The
  340 ms → 107 ms pair appears in no commit message and remains unverified.
- status: resolved
- commit: cd0e2f4 (2026-08-13) — symptom and first fix in one commit

### 2026-08-13 — Hard boundary constraints + OSQP infeasibility fix; CPU pinning [fix]
tags: fix
- MPC hard boundary constraints and an OSQP infeasibility fix (`7f893d4`); wall-detection
  hardening, lidar boundary node, CPU pinning (`cd0e2f4`); `mpc_corr`'s `cpu_affinity`
  default fixed — it had never actually been wired (`5200d2d`).
- **Correction to an earlier draft of this log:** this commit did *not* introduce the
  taskset prefix. It introduced the `os.sched_setaffinity` self-pinning that later turned
  out to be the bug; the taskset-prefix conversion across 7 launch files is `76804f5`
  (2026-08-21) — see that entry. Brief says 5 nodes; it is 7.
- **Misattribution corrected:** the MPC 93 ms → 12.3 ms speedup was *not* produced by
  this commit or by pinning. It was the SLSQP → RTI/OSQP solver swap of 2026-08-10
  (`0f24e66`) — see that entry for the commit text that says so. Perception's
  340 ms → 107 ms is unverified; only the ~278 ms detection lag is recorded in git.
- The specific core assignment (YOLO 8,9; detection+projection 6,7) is **not** in
  `stack_params.yaml` — that file now says `cpu_affinity`/`nice` are "deliberately"
  absent. The numbers could not be confirmed from the repo.
- status: resolved
- commit: 7f893d4, cd0e2f4, 5200d2d, e2f5fed, e2dbe78 (all 2026-08-13)

### 2026-08-10 — The real 93 ms fix: SLSQP → RTI/OSQP solver swap [decision]
tags: decision
- Recorded separately because this is the change the MPC speedup actually came from, and
  it had been credited to CPU pinning. `0f24e66`'s own commit body, verbatim:
  > *"the real-time-iteration OSQP solve path added by the MPC optimization pass
  > following the frequency/bottleneck audit (**SLSQP's solve_dt averaged 93 ms of a
  > 112.6 ms loop against a 100 ms/10 Hz budget; RTI is the fix**, kept alongside the
  > original SLSQP path behind one `solve_mpc_step()` entry point for rollback via
  > `use_rti_solver:=false`)."*
- So git itself puts the 93 ms on **SLSQP** and names **RTI as the fix**, dated
  **2026-08-10 — three days before any CPU pinning existed** (`cd0e2f4`, 2026-08-13).
  The two landed close together and got conflated; they are independent changes.
- Andreas's notes carry the controlled benchmark that settles it, and it is unambiguous:

| configuration | solve_dt mean | over 100 ms budget |
|---|---|---|
| baseline — no pin, SLSQP | 93.0 ms | — |
| **SLSQP + pin** (same solver, pinned) | **93.1 ms** | **75.2 % of ticks** |
| RTI (OSQP) + pin | **12.3 ms** | — |

  Pinning alone moved the mean by **+0.1 ms**. The solver swap is the entire ~7.5×.
- Those three numbers are **not in git** — the "12.3" string in history is a battery
  voltage in `a18ac0f`, unrelated. They are attributed to Andreas's project notes. The
  93 ms and the 112.6 ms loop time *are* in `0f24e66`'s message and are verified.
- `use_rti_solver` defaults **true** from introduction, with the SLSQP path deliberately
  retained behind the same entry point for rollback and comparison.
- status: resolved
- commit: 0f24e66 (2026-08-10); param declared in ed098e3 / 8ee3a84 (same day)

### 2026-08-10 — f1tenth_llm renamed into f1tenth_intelligence/llm [fix]
tags: fix
- Not a from-scratch package creation. `f3250a8`'s own body calls it *"renamed/
  consolidated from the old f1tenth_llm + llm_mpc_tuner packages, ROS package name now
  just 'llm'"*, and `15d75a9` deletes the old tree 11 seconds later.
- **Proof it is a true move, not a rewrite:** `llm_mpc_tuner_node.py`'s blob hash is
  byte-identical on both sides — `85bed0c34c53e0fd732bc79c0f182379982f6cad` at
  `src/f1tenth_llm/llm_mpc_tuner/llm_mpc_tuner/` and at
  `src/f1tenth_intelligence/llm/llm/`.
- **The git consequence, which is the reason this matters:** the add and the delete are
  in **two separate commits**, so git's rename detection has nothing to pair. `-M` finds
  the rename only when you diff the two commits directly; `git log --follow` on the
  current path **stops at `f3250a8`** and will not show you the node's real history.
  That history is: repo root `llm_mpc_tuner/` (`621f076`, 2026-05-30) → `src/llm_mpc_tuner/`
  (`9997d58`) → `src/f1tenth_llm/llm_mpc_tuner/` (`6797b9f`, 2026-07-10) →
  `src/f1tenth_intelligence/llm/llm/` (`f3250a8`).
- **One part of the recalled timeline is not supported.** There was never an `llm`
  sub-package *inside* `f1tenth_llm`: at `15d75a9^` that tree contains exactly two
  subdirectories, `f1tenth_llm/` and `llm_mpc_tuner/`. The name `llm` appears for the
  first time directly under `f1tenth_intelligence/`.
- What genuinely was new in `f3250a8` (no predecessor in `f1tenth_llm`):
  `config/models.yaml`, `config/interrogations.yaml`, `config/mpc_tuner_params.yaml`, and
  `llm.launch.py` (+143 lines) with the ExecuteProcess-based llama-server bringup. So:
  the tuner node moved verbatim, the llama-server bringup and config were built here.
- status: resolved
- commit: f3250a8 + 15d75a9 (2026-08-10)

### 2026-08-10 — MPC_corr introduced and made the deployed controller; 6 variants dropped [decision]
tags: decision
- `MPC_corr.py`, `mpc_solver.py` and `vehicle_model.py` are **genuinely new** in
  `0f24e66` — `5b69f47`'s tree (2026-07-13) contains no `MPC_corr` under any path, so
  this is not a rename.
- The same commit deletes six confirmed-dead MPC variants that no bringup path ever
  wired: `mpc_node.py`, `trajectory_mpc_node.py`, `kinematic_mpc_node.py`,
  `frenet_mpc_node.py`, `andre_mpc_opt_node.py`, `track_mpc_opt_node.py`, plus their
  `tracks/` CSVs and console_script entries. `andre_mpc_node.py` survives as a
  non-default entry point.
- Answering the "what drove the car before MPC" question directly: **nothing else ever
  did.** There is no pure-pursuit, PID or Stanley controller anywhere in this repo's
  history. The only `pure_pursuit` hits are Nav2's
  `nav2_regulated_pure_pursuit_controller`, plus a `_pure_pursuit_ff` *helper method
  inside* `track_mpc_opt_node.py` (added `2188832`, deleted here) — a feedforward term in
  an MPC node, not a standalone controller. `PID` hits are all process IDs.
- status: resolved
- commit: 0f24e66 (2026-08-10)

### 2026-08-10 — BT grows from 2 lanes to 4; emergency and mission lanes added [fix]
tags: fix
- Distinct from the tree's introduction (2026-07-13). `37be92e` takes the root Selector
  from `[handle_obstacle, navigation]` to
  **`[emergency, handle_obstacle, mission, navigation]`**:
  - `emergency`: `IsBatteryLow` OR `IsEmergencyStopTriggered` OR `IsProximityTooClose`
    OR `IsSystemOverheated` → Stop. The first BT had no emergency branch at all.
  - `mission`: a scripted move sequence, deliberately placed **above** `navigation`.
- Brief correction: the structure is **four** lanes, not the three
  (`[emergency, handle_obstacle, navigation]`) the brief records. At runtime with
  `enable_camera_obstacle_stop:=false` the published `lane_names` is
  `['emergency', 'mission', 'navigation']` — `handle_obstacle` is structurally absent,
  which is why a 3-lane list is what you actually see in a bag.
- Also lands the BT visualization tool.
- status: resolved
- commit: 37be92e (2026-08-10)

### 2026-08-10 — Mission system begins: runtime.py, loader.py, the three services [fix]
tags: fix
- Where the mission system itself starts, as distinct from the 2026-08-13 turn step and
  the 2026-08-21 robustness pass. `37be92e` adds `mission/runtime.py` and
  `mission/loader.py` together with:
  - `/mission/load_mission` — typed `LoadMission` service
  - `/mission/start_mission` — `std_srvs/Trigger`, LOADED → RUNNING
  - `/mission/abort_mission` — `std_srvs/Trigger`
  - `/mission/emergency_stop` — `std_srvs/Trigger`, latched, cleared only by restart
- Two wrinkles worth recording: an `AbortMission.srv` was designed and then **dropped**
  per a simplification request (it never reached git), and `LoadMission.srv` itself
  wasn't committed until `e2dbe78` three days later — so `37be92e`'s docstring references
  an interface that was uncommitted at the time.
- status: resolved
- commit: 37be92e (2026-08-10); `LoadMission.srv` in e2dbe78 (2026-08-13)

### 2026-08-10 — component_supervisor replaces monolithic single-process bringup [decision]
tags: decision
- The before-state, from `1099733`'s own message: everything was launched by
  `stack_bringup.launch.py`, described there as a **"single-process orchestrator"** — one
  launch file including all the others, with no supervision layer and no way to restart
  a part of the stack without restarting all of it.
- What changed structurally: `component_supervisor_node.py` + `config/components.yaml`
  spawn **each component as its own subprocess**, individually restartable, exposing
  `/restart_component` and `~/control_component`. `supervisor_bringup.launch.py` is a thin
  launcher for it.
- The commit says the supervisor was added *"alongside"* stack_bringup, not replacing it.
  That word is the origin of the 2026-09-01 discovery that stack_bringup was never
  actually retired — it was never removed in the first place.
- Same commit also pulled `foxglove_bridge.launch.py` out of stack_bringup and deleted
  two dead nodes (`tf_publisher.py`, `throttle_interpolator.py`).
- status: resolved
- commit: 1099733 (2026-08-10)

### 2026-08-10 — Workspace reorg, package split, dead-code retirement [fix]
tags: fix
- Confirmed landed in a single 2026-08-10 burst:
  - `safety_stop_controller` package removed, retired in favour of BT `handle_obstacle`
    (`006451d`).
  - `f1tenth_params` added as single source of truth for launch-param defaults
    (`ed098e3`).
  - Launch files renamed to dot-separated `*.launch.py` (`8ee3a84`).
  - `MPC_corr` (RTI/OSQP) declared the deployed node, dead variants dropped (`0f24e66`).
  - BT emergency lane, mission subtree, BT visualization tool (`37be92e`).
  - `component_supervisor`, EKF map→odom, foxglove throttling (`1099733`).
  - `f1tenth_llm` **renamed/restructured** into `f1tenth_intelligence/llm` (`f3250a8`
    adds, `15d75a9` deletes) — see the dedicated entry below; "removed, relocated" as
    an earlier draft of this log put it understates it.
  - Per-package READMEs across all 12 packages, then a top-level docs index (`15621b1`,
    2026-08-11).
- The colcon-workspace restructure itself is much older: `9997d58` (2026-05-30) and
  `6797b9f` (2026-07-10). No commit series is labelled "Phases 0–10".
- status: resolved
- commit: 006451d, ed098e3, 8ee3a84, 0f24e66, 37be92e, 1099733, 15d75a9, f3250a8
  (2026-08-10); 15621b1 (2026-08-11)

### 2026-07-13 — Nav2 tried and the py_trees BT introduced, in the same commit [decision]
tags: decision
- `5b69f47` — commit message *"nav2 not working, odometry not bad"* — is both halves of
  this story at once, which is not how the brief remembers it (Nav2 first, BT later).
  It adds `nav2_params.yaml` + `nav2_bringup.launch.py` **and**
  `behavior_executor_node.py`, the first py_trees tree, together.
- First BT root Selector was only two lanes: `[handle_obstacle, navigation]`, where
  `navigation` was a `py_trees_ros` action client onto `NavigateThroughPoses` — i.e. the
  BT was initially a *wrapper around* Nav2, not a replacement for it.
- Nav2 was never actually dropped. `76804f5` (2026-08-21) deleted
  `nav2_bringup.launch.py` and replaced it with `nav2.launch.py`; today `enable_nav2`
  defaults **false**, and `navigation.launch.py` brings up `mpc_corr` instead. Nav2
  remains a supported, off-by-default alternative path with a `twist_to_ackermann_node`
  shim (Nav2 has no Ackermann-native controller plugin in this distro).
- status: resolved — Nav2 demoted, not removed
- commit: 5b69f47 (2026-07-13); demotion in 76804f5 (2026-08-21)

### 2026-07-02 — A segmentation model was committed and then never wired [idea]
tags: idea
- Answering the "was there an earlier segmentation attempt?" question: **there was no
  earlier attempt.** `16432ef` commits `yoloe-26s-seg.pt` alongside `yolo26s.pt` and
  `yolo26m.pt`, so a seg-capable checkpoint sat in `models/` from 2026-07-02 — but
  nothing ever referenced it. `use_mask_depth` has **zero** occurrences anywhere in git
  history, and the only surviving mention of `yoloe` is a fixture string in an untracked
  test.
- So the 2026-08-31 → 09-02 switch was the **first and only** segmentation attempt. The
  parked model is the closest thing to a precursor and it was never run.
- status: resolved (question answered — nothing was reverted or shelved)
- commit: 16432ef (2026-07-02) — model file only

### 2026-07-02 — yolo-test branch merged; source-agnostic camera + real YOLO inference [fix]
tags: fix
- `0d1c90d` source-agnostic camera (webcam/ZED) + real YOLO inference; `cef0ee1` merge of
  `yolo-test`; `16432ef` "yolo working with webcam".
- Brief's "branch promotion yolo-test → main": there is a `yolo-test` merge, but it is
  `yolo-test` into `yolo-test` from the remote, not a promotion to `main`. `main` and
  `test` both point at `15621b1` (2026-08-11) and the live branch is `scene-graph`.
- status: resolved
- commit: 0d1c90d, cef0ee1, 16432ef (2026-07-02)

### 2026-06-15 — First real EKF: robot_localization [fix]
tags: fix
- `a342621` — *"new EKF with robot_localization library"* — adds
  `src/f1tenth_stack/config/ekf.yaml` and wires the `robot_localization` EKF into
  `bringup_launch.py` / `stack_bringup_launch.py`. This is stage two of three.
- The file follows the reorg: `ekf.yaml` → `src/f1tenth_bringup/config/ekf.yaml`, and
  `ekf_launch.py` appears under `f1tenth_localization` in `5b69f47` (2026-07-13).
- status: resolved (superseded by the 2026-08-21 dual-EKF split)
- commit: a342621 (2026-06-15)

### 2026-06-12 → abandoned — Kalman odom+IMU fusion inside vesc_to_odom, now bypassed [bug]
tags: bug
- The predecessor to the dual-EKF, found. It is **not a standalone node and not in this
  repo** — no file with `kalman`, `fusion` or `fuse` in its name has ever existed here or
  in the vesc submodule. The fusion was added *inside* `vesc_ackermann`'s existing
  `vesc_to_odom` node (submodule commit `7169442`), with only launch wiring in the parent
  (`b29360d`). `bringup_launch.py`'s new comment:
  > *"vesc_to_odom_node now fuses the VESC bicycle-model odometry with the VESC IMU
  > (sensors/imu/raw) using a Kalman filter. KF tuning params (use_imu_yaw_rate,
  > use_imu_orientation, Q_*, R_*) default sensibly..."*
- **It was abandoned, and the evidence is live.** The same commit added a commented-out
  `vesc_to_odom_node_backup` — *"FALLBACK: original bicycle-model-only odometry (no IMU
  fusion)"*. Today
  [vesc.launch.py:205-208](src/f1tenth_hardware/f1tenth_hardware/launch/vesc.launch.py#L205-L208)
  runs `executable='vesc_to_odom_node_backup'` under the node *name* `vesc_to_odom_node`
  — so the fallback is what actually runs, wearing the fused node's name. Both sources
  still exist in the submodule (`vesc_to_odom.cpp`, `vesc_to_odom_backup.cpp`), and
  today's ROS logs contain `vesc_to_odom_node_backup_*.log`.
- That name reuse is worth flagging on its own: nothing at the topic or node-name level
  tells you which implementation is running. Only the `executable=` line does.
- The "new calibration nodes built to use raw IMU/odometry instead" are confirmed:
  `a18ac0f` (2026-08-10) adds `sensor_covariance_calibration_node.py` doing *"live
  IMU/odom covariance sampling"*, plus `gyro_bias_calibration`, both consolidated into
  `calibration.launch.py`.
- Superseded by `robot_localization` (2026-06-15) and then the dual-EKF (2026-08-21).
- status: superseded — but the dead fused code is still shipped and still buildable
- commit: vesc `7169442` (2026-06-12); parent wiring `b29360d`; bypass is in the current
  working tree

### 2026-06-12 — First filter attempt: hand-rolled Kalman in the vesc submodule [idea]
tags: idea
- `b29360d` — *"first attempt of IMU+eRPM +servo with a kalman filter"* — stage one of
  three. Note the parent commit is almost empty: 12 lines of launch wiring in
  `bringup_launch.py` plus a **submodule pointer bump**. The filter itself lived in
  `src/vesc` (submodule commit `7169442`, same day), not in this repo.
- Superseded three days later by `robot_localization`.
- status: superseded
- commit: b29360d (2026-06-12), parent wiring only; implementation in vesc `7169442`

### 2026-06-12 — ZED 2 stereo camera integrated (stereocamera-test) [fix]
tags: fix
- `88791fa` adds a `sensors_stack` metapackage with ZED 2 integration; `ee3e5af` pins
  `zed_ros2_wrapper` to `humble-v4.2.5`; `29ad810` / `52725aa` fix params and topic names
  to that wrapper's schema; `ba5f444` includes it in `stack_bringup_launch`.
- Recorded here because it is the closest thing in git to a "camera-first" ambition — but
  it is camera *integration*, not camera SLAM. See the standing entry below.
- status: resolved
- commit: 88791fa, ee3e5af, 29ad810, 52725aa, ba5f444 (all 2026-06-12)

### 2026-06-03 — andre_mpc_opt_node: the SLSQP predecessor to MPC_corr [idea]
tags: idea
- `c36e19f` adds `andre_mpc_opt_node.py`, committed as *"(not tested)"*. Its docstring is
  explicit about being an optimised drop-in for `andre_mpc_node.py` with the formulation
  held fixed: *"**SLSQP solver, horizon N=10, dt=0.1**"*, cost = radial + heading +
  steering-rate + lateral-accel. Optimisations were all implementation-level — numba
  `@njit` on the hot cost function, no per-evaluation array allocation, precomputed
  bounds, parameter caching, best-effort depth-1 QoS.
- Two things worth noting. It already carried *"optional Jetson CPU-affinity pinning +
  process nice level"* — so pinning as an idea predates the 2026-08-13 work by ten weeks.
  And it kept SLSQP, which is exactly why it could not fix the 93 ms solve: that took
  changing the solver, not optimising around it (2026-08-10 entry).
- Deleted in `0f24e66` as one of the six confirmed-dead variants no bringup path wired.
- status: superseded by MPC_corr
- commit: c36e19f (2026-06-03), deleted in 0f24e66 (2026-08-10)

### 2026-05-30 — Repo origin state: MPC and slam_toolbox from day one, no filter [decision]
tags: decision
- Worth pinning down because three of the "evolution" questions bottom out here.
  `621f076` / `9997d58`, the first commits in this history:
  - **Control was MPC from the start** — `mpc_controller` ships with five variants
    already (`andre_mpc_node`, `frenet_mpc_node`, `kinematic_mpc_node`, `mpc_node`,
    `trajectory_mpc_node`). No non-MPC controller ever existed here.
  - **slam_toolbox was already configured** — it appears as a `slam_toolbox:` config key
    in `621f076` and as a launch entry in `895d9ca` (2026-06-19): *"8. slam_toolbox,
    async mapping (reuses f1tenth_bringup/config/f1tenth_online_async.yaml)"*. Lidar SLAM
    was the plan from the beginning, not a fallback after something else failed.
  - **No localization filter at all** — the only odom/TF-related file is
    `f1tenth_stack/tf_publisher.py`. Raw VESC odometry, as the brief supposed.
  - `llm_mpc_tuner` also exists from day one, long before `f1tenth_intelligence/llm`.
- status: resolved (baseline, not an open item)
- commit: 621f076 + 9997d58 (2026-05-30)

---

## Standing open items (no single dated event)

### OPEN SAFETY FLAG — lidar static TF yaw never actually confirmed [open-question]
tags: open-question
- The repo **contradicts itself** on this. `is_proximity_too_close.py` (lines 27–28)
  claims the `base_link->laser` transform was "live-confirmed mounted at yaw=0.0 (was pi
  before the remount)". `f1tenth_description/launch/description.launch.py` says the
  opposite about the same transform: the post-remount values are "approximate,
  ruler-measured placeholders, **NOT a real calibration**", and only
  `zed2_camera_link` was confirmed live via `tf2_echo`.
- Actual args: `['0.12', '0.0', '0.20', '0.0', '0.0', '0.0', 'base_link', 'laser']` —
  x/y/z then yaw=0.0. The front e-stop's entire angular-coverage argument rests on that
  yaw.
- status: open — needs physical verification; the brief's concern is correct and the
  in-code "live-confirmed" claim should be treated as unsupported until then
- commit: uncommitted (contradiction present in the working tree)

### URGENT — costmap/map position drifts while the car is stationary [bug]
tags: bug
- After a mission stops with the car genuinely stationary, costmap/map position drifts
  far away despite zero real motion; missions also stop instantly regardless of lidar.
- Unverified hypothesis: `costmap_boundary_node`'s nearest-occupied-cell lookup may
  false-positive "occupied" on out-of-bounds queries once drifted outside the mapped area.
- What is in the tree: `costmap_boundary.py` has `nearest_occupied_in_window` with a
  `range_mask`, and `costmap_boundary_node.py:103` acknowledges drift ("has drifted,
  regardless of anything this node does. Gating THIS node's..."). **No investigation
  artifact, test, or fix exists.** The investigation prompt was written but not run.
- status: open — top priority, undiagnosed
- commit: not found in git history

### Fast-DDS Discovery Server partial-visibility race [open-question]
tags: open-question
- Adopted to fix a confirmed EKF-pair stall-on-component-churn bug. Known unresolved
  cost: an internal Fast-DDS race ("DISCOVERY_DATABASE Error: Matching unexisting
  participant", SEDP builtin-discovery-reader ~1.6 s after component launch) causing
  inconsistent per-participant topic visibility. Recurring as of 2026-08-31.
- Deliberately **not** reverting to SIMPLE discovery.
- Practical consequence for tooling: `ros2 node list` / `ros2 topic list` are unreliable
  here — probe with rclpy and read per-component supervisor logs instead.
- status: open — accepted cost, no fix
- commit: not found in git history (the error is runtime; only `ensure_discovery_server.py`
  exists as mitigation)

### Intermittent mid-mission stop-then-resume [open-question]
tags: open-question
- Symptom: mission drives, sometimes stops, then resumes on its own.
- Candidate causes still on the table: MPC solver non-convergence / constraint jitter,
  corridor staleness (partly addressed by `f3fa196`'s 1 Hz → 10 Hz), noisy pre-seg
  obstacle positions used as hard constraints, ackermann_mux lane flapping, proximity
  e-stop false-triggering, the costmap drift bug firing mid-motion, the DDS EDP race.
- Instrumentation now exists (see 2026-09-02 entry) and has produced a strong new lead:
  **140/1790 (7.8 %) primal-infeasible QP solves** in the 2026-09-02 runs against a
  0/1285 baseline, 128 of them with `n_obstacles=0, n_boundary_constraints=3` — i.e. the
  tightened lidar halfspaces, not obstacles.
- status: open — instrumented, not fixed
- commit: uncommitted

### Car's own LiDAR unit produces a phantom "chair" detection [open-question]
tags: open-question
- The LiDAR housing is visible in the ZED frame. A configurable exclusion-zone param is
  wanted. **Paused** pending confirmation that it really is the LiDAR housing and not a
  real chair, and that the reference screenshot came from the normal on-car mount.
- Partial evidence in the tree: `lidar_exclusion_x_min/x_max/y_min/y_max` and
  `lidar_exclusion_overlap_threshold` params exist in `stack_params.yaml` — so an
  exclusion-zone mechanism was built even though the diagnosis is still unconfirmed.
- status: open — mechanism exists, root cause unconfirmed
- commit: uncommitted (params present in working tree)

### Live wall test: car never stops at the intended 1 m front_clearance [bug]
tags: bug
- Car slows near a wall as expected but turns slightly left near the end and never
  actually stops at the intended 1 m `front_clearance` distance. Reported symptom only,
  never diagnosed.
- Not picked up anywhere in the repo. Note that the 2026-09-02 false-COMPLETE root cause
  (raw displacement vs along-line projection) is in the same `condition_eval.py` and may
  be related — `front_clearance` uses key `distance`, not `value`.
- status: open — undiagnosed
- commit: not found in git history

### Duplicate-service safety incident; production node still running stale code [bug]
tags: bug
- A test instance of `behavior_executor_node` registered under a different node name but
  answered the **same service names** as an already-running production instance (PID
  447653) — test mission calls were routed to and executed by production. Caught via
  duplicate-publisher count; car verified never to have moved (`/odom` linear.x = 0 over
  3 s); mission aborted cleanly.
- Unresolved at time of writing: production node PID 447653 was still running old
  crash-prone code in memory after the fix landed on disk, not restarted.
- **Cannot be checked from git** — it is a runtime/process fact. PID 447653 is not alive
  now (the current stack was restarted many times on 2026-09-02), so the specific stale
  process is gone, but nothing prevents a recurrence: service names are still not
  namespaced per instance.
- status: open (recurrence prevention); the specific stale process is gone
- commit: not found in git history

### `/slam/pose` silence while stationary is NOT a bug [decision]
tags: decision
- Root-caused: slam_toolbox 2.6.10's `shouldProcessScan` gate is XY-only, so pure
  rotation never crosses it. Expected behaviour, no fix warranted.
- status: resolved (no action)
- commit: not found in git history (analysis only)

### f1tenth_costmap loses ament_prefix_path hooks every colcon build [bug]
tags: bug
- Recurring infra annoyance, no permanent fix. Package declares
  `<exec_depend>ament_index_python</exec_depend>` and installs the standard
  `share/ament_index/resource_index/packages` marker, so the packaging looks correct —
  the failure mode is environmental, not a missing declaration.
- Related known gotcha: a literal `--` inside a `package.xml` XML comment silently
  downgrades colcon's package type to plain `python`, producing runtime-only "package not
  found" crashes.
- status: open — worked around, never fixed
- commit: not found in git history

### PACEd-NAV campaign — schema learned, mission files never committed [plan-change]
tags: plan-change
- 5 scenario types (S1 straight+turn, S2 double turn, S3 static obstacle avoidance,
  S4 ambiguous landmark, S5 long/out-of-vocab multi-step), 10 commands × 3 reps = 150
  runs planned. Metrics (Success, Violation rate, Min clearance, Feasibility,
  Translation OK) tracked in an external spreadsheet, not in mission JSON.
- The schema gotchas learned here are real and are now enforced in `mission_config.py`:
  top-level key is `moves` not `plan`; per-move `id` required; `fixed_distance` →
  `distance_reached`; `front_clearance` uses key `distance` not `value`; turn missions
  use `schema_version "2.0"`; a move is discriminated by exactly one of
  `goal_distance`/`goal_pose`/`turn`; `stop_condition` is a required sibling on every
  move including turn; `turn.speed` must be > 0 (default 0.5); a turn's own
  `stop_condition` type is `orientation_delta` and must equal `abs(heading_delta_deg)`.
- **No `PACED-NAV-*` mission file exists** in the working tree or in any commit. The
  missions directory holds `bottle_then_person`, `boundary_constraint_diag_05m`,
  `dock_approach_01`, `straight_left_straight`, `test_01`–`test_05`, `turn_90_left`.
- status: open — campaign infrastructure orphaned
- commit: not found in git history

### Camera-only SLAM attempt — NO evidence in git history [open-question]
tags: open-question
- Andreas recalls a deliberately ambitious camera-only SLAM approach tried before
  settling on lidar-primary slam_toolbox. **Searched hard and found nothing.** Across all
  branches, the reflog, and dangling objects: `rtabmap` 0 hits, `orb_slam` 0, `ORB_SLAM`
  0, `vslam` 0, `visual_slam` 0, `stella_vslam` 0. The 19 apparent "RTAB" hits are
  case-insensitive false positives on **"portable"** and **"restartable"**.
- The evidence actually points the other way: slam_toolbox is present from the **first
  commit** (2026-05-30, `621f076`) and running as an async-mapping launch entry by
  2026-06-19. There is no window in this repo's history where lidar SLAM was absent and
  something else was being tried.
- The nearest real thing is the `stereocamera-test` ZED 2 integration work of 2026-06-12
  — camera integration, no SLAM component.
- Three ways this could still be true and invisible here: it was never committed; it
  lived on a branch deleted before this repo's reflog begins (**the reflog only goes back
  to 2026-08-13**); or it lived in a different workspace entirely — note that
  `/home/fabiocar/roboracer_ws` appears in a tool-permission line in `5b69f47`, so a
  second workspace demonstrably existed.
- status: open — needs Andreas to confirm from memory or point at the other workspace
- commit: not found in git history (searched `--all --reflog`, `git fsck`, all refs)

### BT.CPP XML tree predecessor — referenced but never in this repo [open-question]
tags: open-question
- The py_trees tree introduced on 2026-07-13 describes its own priority structure as
  *"same as the old BT.CPP XML"*, and says of the missing emergency branch: *"the old
  BT.CPP XML never had one either."* So a BehaviorTree.CPP tree demonstrably existed and
  was the design ancestor of the current tree.
- **It is not in this repo.** No `.xml` behaviour tree was ever committed on any branch.
  The only `behaviortree_cpp` and `bt_xml` string hits are incidental: a tool-permission
  entry pointing at `/opt/ros/humble/include/behaviortree_cpp_v3/**`, a grep permission
  referencing `/home/fabiocar/roboracer_ws`, and a comment about
  `nav2_bt_navigator`'s own default XML.
- Best guess, flagged as a guess: it lived in `roboracer_ws`, the other workspace those
  permission lines point at.
- status: open — same question as the camera-SLAM entry; likely the same missing workspace
- commit: not found in git history

### go_straight move type — design stage only [idea]
tags: idea
- Closed-loop corridor centering using side-LiDAR to find and stay centered in a
  corridor, as a **new move-type discriminator**, not a flag on `goal_distance`.
- Open question, undecided: whether the corridor comes from a new costmap layer or a
  standalone side-scan line-fit node.
- Zero occurrences of `go_straight` anywhere in the repo. Nothing coded.
- status: open — design only
- commit: not found in git history

### Mocap test campaign infrastructure — planning only [idea]
tags: idea
- Qualisys motion capture via QTM on a Windows 10 VM on host `linus`, streaming to the
  Linux host; ROS 2 nodes on `linus` managing missions over the network to the car,
  prompting the LLM, verifying generated JSON, logging to a database, plus a GUI showing
  missions / launch button / scores.
- Zero occurrences of `qualisys`, `QTM`, or `mocap` anywhere in the repo.
- status: open — planning only, nothing built
- commit: not found in git history

### Abandoned parallel MPC branch reviewed, no action [decision]
tags: decision
- An uploaded `mpc_controller.py` / `mpc_solver.py` pair was reviewed and confirmed to be
  an abandoned parallel experimental branch, not a newer version. All useful features
  were already in the deployed `mpc_corr.py`. No edits made.
- Unverifiable by design: the correct outcome of "no edits made" is that nothing exists
  in the repo to find. Recorded on the brief's word alone.
- status: resolved (no action)
- commit: not found in git history

### cuBLAS shadowing bug from an unused pip package [fix]
tags: fix
- `pip nvidia-cudss-cu12` (installed 2026-07-08, unused in the repo) pulled a shadowing
  cuBLAS via `cuda-toolkit`, breaking torch. Fixed by uninstalling the chain and
  reinstalling `nvidia-cudss-cu12 --no-deps` only.
- Environment-level, outside the repo — nothing in git can confirm or refute it.
- status: resolved (per brief)
- commit: not found in git history (environment change, not a repo change)
