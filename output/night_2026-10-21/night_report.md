# Night 2026-10-21 → 22: report

Unattended run on linus, one lead agent and sub-agents. Every stream went
through an implementer, two independent read-only reviewers (one for the spec,
one for bugs and safety) and a fixer. **Nothing was pushed during the night.** No sudo was
needed. Detailed per-stream reports are next to this file in `streams/`. The LLM2 toggle and the push were finished the next morning (10-22): see §2 and §13.

## In short

- **`campaign-final`** (`4370897` + this report commit) = start (`6002cb0`) + walls (A) + run reports (D) + reverse (B) + the app (`pwa-control` with B3, B4, B5), each as a `--no-ff` merge, plus this report.
- **`main`** was fast-forwarded to `campaign-final` (it was an ancestor: 0 commits in `main ^campaign-final`).
- **`two-llm-arch`** (`b89a3bd` before its rebase onto this report) = that final `campaign-final` + stream C (LLM2 mock) + the LLM2 on/off (node, gateway, lamacchinina). It is NOT merged into `campaign-final` or `main`. Every `pwa-control` commit is in it.
- **Humble build on the Orin** (separate clone `~/dev_ws/f1tenth_more_buildcheck`, build only): `campaign-final` 41/41 packages, `two-llm-arch` 41/41 packages (`b89a3bd`). Every tracked `.py` compiles with the Orin's Python 3.10.12.
- **Reverse:** the VESC version is not the cause. The car reverses and odometry sees it; the 10-20 15:39 reverse ended on `timeout` because the manoeuvre counted the distance through the map anchor. Fixed (larger of the odom and map counts).
- **Read first for the car:** the wall confirmation level makes FRONT appear later (4.5 m closer on corridor10m), and with reverse on in the app a thumb resting low on the gas pad now reverses.

## 1. State at start and at end

**At the start of the night** (`two-llm-arch` did not exist yet):

```
### night-start/main
e47e646 repo root: move the loose docs under docs/, drop the stale ones
876a5a6 first_test_campaing: M03_bottle corridor plots and the R004-R010 overlay
bc3b049 object corridor: a maximum length, an end on the target, and a run you can read back
fa9f99e arc corridor: the target-line cost belongs to the arc, not to every object corridor
7966338 arc corridor: switch band on the clamp, and a tolerance per curve
51c6154 object corridor: the arc that gets the car to the target, behind a parameter
9d23592 stack_params: corridor_heading_return's prose describes the value that is NOT set
0e52526 test_campaign_corridor_plot: one figure per test, from the logs alone
b68f360 mpc_corr: stop destroying the corridor snapshots at every launch
90c51d6 corridor schema v2: the function definition, and one shared evaluator
220dba3 test_campaign: measure corridor clearance against the walls, not the end cap
119f239 corridor_perception: Step 0 baseline, per-scan structure from real bags
eeb4b38 supervisor: obstacle_clearance auto-starts with the stack
6de6ac3 test campaign: plan_result before load, plan provenance, logger config, backfill
8e0d48c f1tenth_logger: the test-campaign logger moves in, beside the mission logger

### night-start/campaign-final
6002cb0 front_clearance stop: lock onto the wall the move approaches (wall_lock_enabled)
1ee82c8 output/speed_gap: offset-free speed law -- design, identification, matrix, risks
17c54cc mission_logger: record the speed loop end to end by default
4701caa mpc_corr: offset-free speed command (lag model + disturbance observer)
f7e7791 output/speed_gap: why the car drives 0.23 m/s when the mission asks for 0.5
3ac5865 Mission start countdown 0 by default, and single-sourced in stack_params
ed2e4a4 stack_params: unknown_objects_mode off (the LiDAR-only objects blocked the car)
3feae47 tools/semantic_merge_replay.py + output/semantic_merge_replay.md: the merged semantic layer on 71 local bags
b1f9ffa output/mpc_known_failures.md: the 13 mpc_controller tests that already fail, one cause each
bfeb503 stack_params: obstacle_class_margin_m = {person: 0.5}
fb913d1 Door detection off by default (lidar_front_wall_door_detection_enabled: false)
b31155e hard_stop_node: direction-aware behind hard_stop_allow_reverse (default true)
e05816b Merge obstacles-v2 (d0028f2) into campaign-final: one semantic layer, both lifecycles
2ff621e Merge door-fixes (486a157) into campaign-final
247264e Merge mpc_corr_v2_reverse (4f53ab8) into campaign-final

### night-start/pwa-control
9c5371d PWA: a permanent MOCK badge when the gateway runs the pretend car
af0bb87 output/pwa_report.md + output/pwa_screens/: the PWA's final report and screenshots
1aa4a45 apps/pwa: Guida warns when the screen cannot be kept on
4586b99 f1tenth_gateway: README -- install on the Orin, token, systemd, tailscale serve, iPhone, tests
ce84001 f1tenth_gateway tests: skip the app tests per test when fastapi is missing
```

**At the end**, just before this report commit (`main` = `campaign-final`):

```
### main (before this report commit)
4370897 Merge pwa-control: the F1Tenth Lab Control app (gateway, f1tenth_ctl, PWA) with reverse, lamacchinina, CPU/GPU view
87e6f69 apps/pwa: review fixes - CPU view colours, chip names, 320 px tiles and tab
cf9e5ab f1tenth_gateway: review fixes - nvidia-smi hang, exiting processes, interpreter options
2368a2d apps/pwa: GPU tile and a per-core CPU view with the pinned processes
3accf86 f1tenth_gateway: CPU per core, GPU % and pinned processes in the status message
306782f apps/pwa: rename the Cockpit view to lamacchinina
ec288e7 Merge night-reverse: the reverse manoeuvre counts the distance backed up in odom
7bf99f9 Merge night-reports: run-report stills and video with the scene mask and the LiDAR
6777c61 Merge night-walls: one wall confirmation level (wall_confirm_*), MPC ramp on the locked front wall
120d0f4 output/front_wall_lock: review fixes - measured confirmation latency, replay wording
480fb6c mpc_corr: review fixes - every ramp-source transition is logged
6671431 multi_wall_tracker: review fixes - confirmation distance jitter measured at the wall
2ed5216 mission_overlay: review fixes - an unstamped message does not end the search
fb14a79 mission_logger: review fixes - mcap warning and report-overlay descriptions
1cc0a8a mission_overlay: review fixes - stamp search, still order, stack_params fallback

### campaign-final (before this report commit)
4370897 Merge pwa-control: the F1Tenth Lab Control app (gateway, f1tenth_ctl, PWA) with reverse, lamacchinina, CPU/GPU view
87e6f69 apps/pwa: review fixes - CPU view colours, chip names, 320 px tiles and tab
cf9e5ab f1tenth_gateway: review fixes - nvidia-smi hang, exiting processes, interpreter options
2368a2d apps/pwa: GPU tile and a per-core CPU view with the pinned processes
3accf86 f1tenth_gateway: CPU per core, GPU % and pinned processes in the status message
306782f apps/pwa: rename the Cockpit view to lamacchinina
ec288e7 Merge night-reverse: the reverse manoeuvre counts the distance backed up in odom
7bf99f9 Merge night-reports: run-report stills and video with the scene mask and the LiDAR
6777c61 Merge night-walls: one wall confirmation level (wall_confirm_*), MPC ramp on the locked front wall
120d0f4 output/front_wall_lock: review fixes - measured confirmation latency, replay wording
480fb6c mpc_corr: review fixes - every ramp-source transition is logged
6671431 multi_wall_tracker: review fixes - confirmation distance jitter measured at the wall
2ed5216 mission_overlay: review fixes - an unstamped message does not end the search
fb14a79 mission_logger: review fixes - mcap warning and report-overlay descriptions
1cc0a8a mission_overlay: review fixes - stamp search, still order, stack_params fallback

### two-llm-arch (before this report commit)
b89a3bd f1tenth_gateway: review fixes - LLM2 status parse cannot stop the ROS executor
c9a0859 apps/pwa: LLM2 toggle in lamacchinina (two-llm-arch)
5f5ff56 f1tenth_gateway: LLM2 state in the status and a /llm2/set_active endpoint (two-llm-arch)
eb9be96 two-llm: review fixes - /llm2/status state read at publish, service contract pinned
c51fd60 two-llm: LLM2 pause/resume (/llm2/set_active, start_active)
9dc045a two-llm: review fixes - tests for non-echo replies and the enable_llm2 gate, stale comments
7b4b088 two-llm: review fixes - read-only LLM2 params, reverse w_v limit, w_obs never tunable
1c34d91 two-llm: tests + docs for the LLM2 mock (C6, C7)
4f04969 two-llm: MPC_corr subscription + whitelist for LLM2 parameter updates
c6b1cba two-llm: llm2_mpc_param_node, the LLM2 mock MPC-parameter interrogation
96249f0 two-llm: MpcParamUpdate message, LLM2 server config, enable_llm2 launch args
4370897 Merge pwa-control: the F1Tenth Lab Control app (gateway, f1tenth_ctl, PWA) with reverse, lamacchinina, CPU/GPU view
87e6f69 apps/pwa: review fixes - CPU view colours, chip names, 320 px tiles and tab
cf9e5ab f1tenth_gateway: review fixes - nvidia-smi hang, exiting processes, interpreter options
2368a2d apps/pwa: GPU tile and a per-core CPU view with the pinned processes
```

+ this report commit on `campaign-final` (and `main`), and the rebase of `two-llm-arch` onto it.

## 2. Decisions taken

**Lead**
- **`pwa-control` was merged, not rebased.** It was published and had diverged from `origin/pwa-control` (local `9c5371d` vs the Orin's 7 commits `7f3be61..adb9143`). A rebase would rewrite published history and need a force push. B3 merged `origin/pwa-control`, then `campaign-final`, into it. It is now a fast-forward of `origin/pwa-control`.
- **Sub-agents committed on their own branches** in their own worktrees, as the plan says (this overrides the standing "sub-agents read-only" rule for this night only). Only the lead touched `campaign-final`, `main` and the `two-llm-arch` rebase.
- **ROS domains:** A 71, B1/B2 72, C 73, lead 74, D 75, and **76 for B3/B4/B5** (not 72), because B3 ran in parallel with B1/B2.
- **Extra backup tags** besides the four asked for: `night-start/pwa-control-origin` (`adb9143`), `night-start/mpc_corr_v2_reverse-origin` (`4f53ab8`), `night-pre-rebase/two-llm-arch` (`f297912`), and in the vesc repo `night-start/vesc-campaign-final` (`0e74a60`) and `night-start/vesc-main` (`2848a9f`).
- **Orin build check as a separate clone**, not a `git worktree` of `~/dev_ws/f1tenth_more`: the night commits are not pushed, and a worktree would write into the main checkout's `.git`. The clone was made locally from the main repo (read-only use), the night commits came in as git bundles (`/tmp/night_bundles/` on the Orin), and the submodules were cloned from the main repo's `.git/modules`. Only `/opt/ros/humble` was sourced. The main checkout is untouched (still `1ee82c86` with its own uncommitted files).
- **Order of the addendum work:** the LLM2 node pause/resume was done on `two-llm-arch` before the rebase (it touches only `llm`), the gateway + lamacchinina toggle after it (it needs the app). The D, B4/B5 addenda were done before the app merge, as asked.
- **Extra Humble check** on linus in the `f1tenth_more:cpu` image (Humble, Python 3.10): see §9.
- **The LLM2 toggle took three attempts.**
  - Its first workflow was cut by a session limit at ~03:00 and left ~285 lines of uncommitted, untested gateway work.
  - A second workflow resumed from that partial work at 09:14 and committed the gateway and the PWA by 09:37.
  - The lead session exited at 09:41, during that workflow's review, which killed it.
  - A later session ran the two reviews again at ~10:55 with the same lenses. They found two minor issues, and that session fixed both in `b89a3bd`.
- **The wall check** (§4, last subsection) was read-only, with two independent checks. Its third, adversarial step was killed at 09:41 too. No code was changed for it.

**Streams** (the full list is in each `streams/*.md`)
- A1: `wall_confirm_*` has no prefix; validated in `TrackerParams.__post_init__` (f1tenth_params has no validation layer). The birth cycle is hit 1. Jitter is measured at the track's segment **midpoint** (review blocker: measured from the odom origin, a wall far along its own direction never confirmed).
- A2: the BT publishes the locked id on `/mpc/stop_wall_id` (std_msgs/Int32, no message rebuild); MPC_corr reads distance and id from the same `/perception/virtual_wall/debug` payload; the pure `ramp_wall_source.select_ramp_distance` picks the source. In a swerve (lock held, another wall FRONT) there is **no wall ramp** at all: the ramp is only deceleration toward the target wall.
- B1/B2: vesc stays at `0e74a60` (already on GitHub, built on the Orin). James's `origin/jazzy` reverse commits `a596962`/`4b9b0d4` were **not** ported: they reverse through the forward solver, which `campaign-final` forbids on purpose. The forward `v ≥ 0` guard already existed (`a8a03ea`).
- B3: reverse in manual driving: linear to −0.4 m/s, through zero with a 0.3 s hold (longer while odometry shows motion the other way); no 0.4 floor backwards; `f1tenth_ctl` and Telegram no longer show a stale 15 s countdown.
- B4: only visible text renamed; route `#/cockpit`, `Cockpit.tsx` and test names kept.
- B5: kernel threads are counted apart, not as "unpinned" (221 on the idle Orin, 123 of them pinned per CPU); past 6 pinned processes the 6 colours repeat with a dashed/dotted border.
- C: **LLM2 is on port 8084**, not 8083: LLM1 (the planner) already listens on 8083 and starts by default. **`vdes` is not tunable** (per-move mission state, not a parameter); **`w_obs` is never tunable** (the only obstacle term, a soft cost). The LLM2 switches are read-only at runtime. With `allow_reverse` on, an update with `w_v` above 4.3 is rejected whole.
- C on/off: `/llm2/set_active` answers `active`/`paused`; a request in flight at the pause is finished and discarded; status keeps publishing while paused.
- D: overlay rendered **offline** from the bag; the masks are rebuilt from the recorded depth when the classes topic is not recorded. One small car-side addition (see §5).

## 3. Skipped, and why

- **Sim integration of LLM2** (feature on with the mock vs off): the plan excludes the stack and the sim on linus.
- **Anything on the car**: nothing was started, deployed or killed on the Orin; the Orin was only read (and built in the separate clone).
- **Push of the vesc commit**: not needed, `0e74a60` is on GitHub (`refs/heads/campaign-final` and `refs/heads/scene-graph` of sjckness/vesc).
- **tegrastats with both LLM servers**: commands for tomorrow in `output/two_llm/README.md` §4.
- **`npm run screens`**: it rewrites committed screenshots, so `output/pwa_screens/` still shows "Cockpit" and the 3-tile row.
- **Out-of-scope bugs found and left** (details in the stream reports): the tracker's `max_sigma_rho_m` deletion tests odom rho, so a wall ~20 m along its own direction from the EKF origin is deleted at birth (pre-existing, A); the app's AVVIA → motion still takes ~12-20 s, spent in Discovery-Server discovery by two fresh `f1tenth_ctl` participants, not in a countdown (B3); the overlay reads sqlite3 bags only (D).

## 4. Walls (stream A)

Merged as `6777c61` (`night-walls`, 6 commits). Details: `streams/A_walls.md`.

**First, the code check you asked for: a confirmed wall that is NOT locked is still an MPC avoidance constraint**, independent of `virtual_wall` and of the lock. Path: the tracker's `published()` emits it on `/perception/walls` → `MPC_corr.wall_tracks_callback` → `wb.build_wall_rows` → `wall_boundaries.admit_track` (`wall_boundaries.py:190`) admits front/left/right walls that are not stereo-only, not TENTATIVE (`:212`) and not coasting longer than 1.0 s → the rows go to the solver as `boundaries=live_boundaries`. Unchanged limits: 'back' walls only become disks, 'unknown' walls are never published, at most 4 tracks are published. No MPC change was needed.

**A1: one confirmation level** (`multi_wall_tracker.py`: `TrackerParams` :439, `confirmation_reason` :1119, `line_jitter` :1094, applied in `confirm_reason` :1683 and `_promote` :1697). New group in `stack_params.yaml`:

| param | value | meaning |
|---|---|---|
| `wall_confirm_min_consecutive_hits` | 5 | LiDAR-backed (LIDAR or BOTH) associations in a row; the birth cycle is hit 1 |
| `wall_confirm_min_length_m` | 1.0 | the track's measured extent |
| `wall_confirm_max_dist_jitter_m` | 0.05 | max − min of the perpendicular distance, measured at the track's segment midpoint (odom, ego-motion compensated) |
| `wall_confirm_max_angle_jitter_deg` | 3.0 | max − min of the wall direction |
| `wall_confirm_require_lidar` | true | stereo-only tracks never confirm |

Old → new:

| old name | old value | replaced by |
|---|---|---|
| `TrackerParams.confirm_hits` (code default, never a launch param) | 3 (of 5) | `wall_confirm_min_consecutive_hits` 5, now consecutive |
| `TrackerParams.confirm_window` | 5 | no window |
| `lidar_front_wall_stereo_confirm_hits` / `_window` | 5 / 7 | `wall_confirm_require_lidar` |
| `lidar_front_wall_camera_confirm_fresh_only` | true | always on when camera hits count |
| tentative expiry "5 scans hold fewer than 3 hits" | — | a tentative track with no association on a cycle is deleted |

Hysteresis: `ever_confirmed` is sticky; only the existing miss/deletion rules remove a confirmed track. **Consumers**, all on this one level with no thresholds of their own: FRONT and `virtual_wall`, `/perception/walls`, strip and negative evidence, door hosts (`door_detection.py:492/565/876`), wall markers, the BT lock (`wall_lock.py`), the MPC rows (`wall_boundaries.py:212`), the semantic layer (`semantic_layer_node.py:668`). `corridor_perception_c` has no track lifecycle. Not switched: the GlassTracker (separate node, off by default) and the semantic layer's raw `_observed_walls_map` (deliberately unconfirmed).

**A2: the ramp aims at the locked wall** (`MPC_corr._ramp_wall_distance` :3381, pure `ramp_wall_source.select_ramp_distance`):

| case | the deceleration ramp reads |
|---|---|
| `wall_lock_enabled: false` | `_fresh_front_distance()`, exactly as before; nothing new subscribed |
| not locked yet | raw `virtual_wall` distance |
| locked, its wall is FRONT | the payload's `distance_m` for that track |
| locked, another wall or none FRONT (swerve) | no wall ramp; other walls stay with the wall rows, avoidance, stop+reverse and the 20 cm LiDAR stop |
| locked wall lost | raw, until re-acquired under the existing gates |
| stale id (> 0.5 s), new stop target, unusable payload | raw |

`wall_follow_gate_m` 0.30 and `wall_follow_gate_deg` 15 stay params; 15° is marked NOT MEASURED in `output/front_wall_lock/design.md`, which also describes the ramp and the confirmation level. Every ramp-source change is logged (`RAMP | aiming at ...`), and `/mpc/stop_wall_id` is recorded by the mission logger.

**Measured on a real bag** (corridor10m_20261019T131815, LiDAR-only re-track, 796 cycles):

| | before tonight | now |
|---|---|---|
| first FRONT after driving starts | 20.4 m ahead, 0.15 m driven | 15.7 m ahead, 5.0 m driven |
| FRONT held, % of cycles | 82 | 51 |
| confirmed right wall within 2.5 m, % of cycles | 34 | 27 |
| birth → confirm, p90 cycles | 3 | 4 |

**Risks for the car:** FRONT (and so the lock and the ramp) engages later for far walls, mostly because of the 5-in-a-row rule; short or intermittent side walls give fewer MPC rows; odometry reads ~17 % short, which eats into the 5 cm jitter budget at speed (a head-on wall at 1.0 m/s never confirmed in simulation; 0.5 m/s is fine). While the lock holds, the ramp does not slow for a nearer surface in front of the locked wall.

### The wall check you asked for (10-22 morning)

You said: "the confirmation that makes the wall appear later is the one with the stereo, and for me that is OK. If there is no FRONT wall, the front distance must be 50 m; the car takes it into account and drives forward, then points to the wall once there is one. It should already be like that." Checked in the code of `4370897` with two independent read-only checks (perception; BT + MPC), the walls' unit tests (105 + 130 passed) and pure-module snippets:

- **No FRONT wall → 50 m: yes.** `/perception/virtual_wall` carries 50.0 (CLEAR, `wall_clear_distance_m`) at ~10 Hz, never silence and never 0. The −1 (NO_DATA) appears only with no pose or no laser transform, and that value holds the car.
  - The BT takes 50 as a fresh reading: no hold, the `front_clearance` stop says "not yet", and the move goes on.
  - The MPC takes 50 or more as "no wall": no ramp, so the car drives at the move's speed.
- **Wall confirmed → the car goes to it and stops: yes.** Once a LiDAR-backed FRONT is confirmed, the BT locks it and sends its id on `/mpc/stop_wall_id`. The MPC then ramps on that wall's distance. At 0.5 m/s the car cruises until ~2.45 m from the wall, latches zero at ~2.2 m and stops at ~2.0 m (stop `front_clearance` 2.0).
- **"Points to the wall": no, it keeps its heading.** A `straight` move holds the heading it had at its start, before and after a wall appears. The car never turns toward the wall.
  - Only a wall in the strip ahead of that heading (|y| ≤ 0.286 m) can be FRONT.
  - An oblique wall is approached at its angle; the car does not square up to it.
- **Not the stereo.** With `wall_confirm_require_lidar: true`, stereo hits never count toward confirmation. The later FRONT comes from the rule of 5 LiDAR-backed hits in a row (plus the length and jitter tests). The corridor10m replay above was LiDAR-only.
- **Gaps (no code changed, questions for you):**
  1. **Speed.** If the front wall never confirms, the topic stays at 50: no ramp, no BT stop, only the LiDAR proximity stop. The jitter budget is 5 cm, and a head-on wall at 1.0 m/s never confirmed in the synthetic test. Missions above ~0.7 m/s should be checked first.
  2. **Wrong lock** (by design, not new). If the first FRONT the move locks is the wrong wall and it stays tracked, the real end wall gives no ramp and no BT stop.
  3. **Log noise.** During the no-wall part of a wall-stop move, mpc_corr logs a WARN every 2 s ("the wall distance is absent, stale or CLEAR: no ramp this tick"). The command is unchanged.

## 5. Run reports with masks + LiDAR (stream D)

Merged as `7bf99f9` (`night-reports`, 6 commits). Details: `streams/D_reports.md`.

**Recon.** The app's Run view (`apps/pwa/src/screens/Run.tsx` → gateway `runs.py`) shows `<run_dir>/snapshots/*.jpg` and `<run_dir>/*.mp4` or `<ws>/mission_videos/<run_id>*.mp4`. The stills are written by `mission_logger_node._write_snapshots` at finalize; the video is made by hand with `mission_debug_video`. The default recording profile had **no camera frame without drawings** (only YOLO's annotated 5 Hz debug JPEG); the masks (`/perception/scene_labels/classes`) and the raw 5 Hz camera are recorded only in the campaign profile; the LiDAR-to-image projection existed only inside `front_clearance_node`'s live debug image, never recorded.

**Option taken: offline rendering**, with the mask rebuilt from the recorded 5 Hz depth (+ detection masks + the run's own params) when the classes topic is not in the bag. Recording the debug overlay instead would cost tens of ms per frame on the car.
- The renderer is **reused**: `scene_mask.overlay()` got keyword options (its defaults draw exactly what the live node drew), and `scan_to_base` moved there from `front_clearance_node`. Stream A touched neither file.
- New `f1tenth_logger/mission_overlay.py` (no ROS needed: reads sqlite3, decodes CDR) builds the stills; `mission_debug_video` uses it too (`--source auto|overlay|recorded`, `--alpha`, `--point-px`, `--no-legend`).
- Output names are unchanged, so **the app needs no change**.
- A run without camera frame, camera_info, camera pose or mask keeps exactly its old stills and video, with no error.

**Extra CPU on the car:** one small addition: `yolo_detector_node` JPEG-encodes its already-downscaled 480×270 debug-tick frame on `/camera/debug_image/clean/compressed`, **only while something subscribes** (the logger's recorder). 0.43 ms per frame on linus, estimated 1-1.5 ms per 5 Hz tick on the Orin (not measured), ~4 MB/min of bag. Nothing at all with the logger off (the default). Finalize: 10 stills in 0.2 s on linus.

**New params** (stack_params, read once per process, so a change shows at the next stack start or the next `mission_debug_video` run):

| param | default |
|---|---|
| `mission_report_overlay_alpha` | 0.3 |
| `mission_report_overlay_point_px` | 2 |
| `mission_report_overlay_legend` | true |

**Example images** (copied next to this report):
- `img/D_overlay_recorded_mask.jpg`: the car's own recorded mask + `/scan` + TF on the 10-19 obstacles bag.
- `img/D_overlay_rebuilt_mask.jpg`: mask rebuilt from depth on the 10-21 run (with a clean frame synthesized from the debug JPEG, so YOLO's drawings show).
- Also in scratch: `/tmp/claude-1000/-home-andreas-dev-ws-f1tenth-more/b4f59daa-6343-486c-9d59-6918832814b8/scratchpad/stream_D/fix/` (`cli_obst/`, `stills_new/`, `video_1021/…debug.mp4`).

**Not verified:** the default path (clean JPEG + rebuilt mask) on a real run (no run on linus has both); Orin timings. Keep the bag storage on sqlite3: with the mcap plugin installed, every run falls back to the plain stills.

## 6. Reverse (B1, B2, B3)

Merged as `ec288e7` (`night-reverse`, 4 commits) and, for the app, inside the `pwa-control` merge. Details: `streams/B12_reverse.md`, `streams/B3_app_reverse.md`.

**B1: which VESC the car runs, and why reverse "failed"**
- vesc gitlink: `0e74a60` on `campaign-final`, `pwa-control` and now `main`; it was `2848a9f` on the old `main`. `0e74a60` is on GitHub, so no push.
- The Orin runs `0e74a60`: `libvesc_ackermann.so` built 10-20 16:03 from that source; `vesc_driver` (09-07) is the same in both commits; `install/` is symlinks.
- `2848a9f..0e74a60` only removes two symmetric deadbands in `vesc_to_odom_backup`; the speed sign is preserved. The command chain passes the sign: gain 4516.9, offset 0, clipped to ±23250 ERPM (`vesc.yaml:96-97`), nothing publishes brake/current. **No VESC Tool or firmware value needs to change.**
- **The cause** (from the Orin's `~/.ros/log`, read only): the reverse manoeuvre counted `travelled` in the map frame, from a start pose re-read through map→odom every tick. In the 10-20 15:39:09 reverse, odom was 0.77 m back at the timeout tick while less than 0.5 m was counted, so map→odom moved with the car (inferred, map→odom is not logged). The other reverses of that day ended `distance_done` (12 of 13 in the evening), at −0.32..−0.36 m/s for a −0.40 command.

**B2: MPC**
- `git log campaign-final..mpc_corr_v2_reverse` is empty (local and origin).
- The forward solver cannot output v < 0 (already on `campaign-final`: `vMin` 0, speed-channel bound 0, and a final gate in `_publish_drive`).
- New in `reverse_manoeuvre.py`: `travelled = max(0, odom count, map count)`: either can end the reverse, neither can push it past the 0.5 m strip the precheck validated (an odom-only count would let frozen odometry run 0.9 m to the timeout: review finding, fixed). No flag: `dd129b9` + `6536877` can only be dropped together.
- `output/mpc_v2/STATUS.md` updated; its "Next run" bag command now includes `/tf`, `/tf_static`, `/ekf_global/odometry/filtered`, `/slam/pose_calibrated`.

**B3: reverse from the app** (gateway `teleop.py`, Guida)
- Negative throttle → negative speed, linear from the dead-zone edge (0.15) to `-min(max_reverse_speed_mps, limit)`; no 0.4 floor backwards; ramps up at `accel_mps2`, drops to 0 at once.
- Direction changes go through zero: 0 for at least `reverse_hold_s` (0.3 s), and longer while fresh odometry (< 0.5 s) shows motion the other way above `stop_speed_mps` (0.05).
- Same dead-man, 250 ms watchdog and release-to-zero as forward.
- Gateway params: `max_reverse_speed_mps` 0.4 (0 = reverse off; a test couples it to the MPC's `max_reverse_speed_mps`), `reverse_hold_s` 0.3, `stop_speed_mps` 0.05.
- Guida: amber **RETROMARCIA** pill (with "attesa arresto" during the hold), THR/CMD amber; with reverse on, the gas pad's ends read **AVANTI / RETRO**. The pad maps the absolute thumb position, so **a thumb resting low now reverses**.
- SPEC §5 and the gateway README replace "Retromarcia non abilitata nella v1".
- The "15 s": `adb9143` (discovery 3 → 15 s) is a ceiling, not a wait. The stale 15 s was `f1tenth_ctl`'s own copy of the countdown and Telegram's `countdown_s or 15`; both fixed (`04db46f`, `0b23bf1`).

## 7. App: lamacchinina (B4) and the CPU/GPU view (B5)

In the `pwa-control` merge `4370897`. Details: `streams/B45_app.md`.

**B4.** The view is shown as **lamacchinina** everywhere a user sees it: tab, h1, Guida's back link, SPEC, design mockups, gateway README. Route `#/cockpit`, `Cockpit.tsx`, `start_url` and test/screenshot names are unchanged. Below 360 px wide the tab bar uses 10 px labels so it fits.

**B5.** The gateway's existing 1 Hz status message got one field, `status.system` (no new endpoint), sampled only while a phone is connected:
- per-core CPU % from `/proc/stat` deltas, online/offline from sysfs;
- GPU % from the devfreq load file (per-mille → %) and its frequency. **On the Orin (L4T R36.5): load `/sys/devices/platform/bus@0/17000000.gpu/load`, frequency `/sys/class/devfreq/17000000.gpu/cur_freq`** (306 MHz idle; `max_freq` 816 MHz is the power mode's cap). On linus: `nvidia-smi` with a hard 1 s timeout (a wedged one is abandoned, never blocking the gateway);
- pinned processes every 5 s: affinity that leaves out an online core, with a friendly name (`__node:=`, launch file, script, executable), cores and own CPU %; zombies and processes vanishing mid-read are skipped; the rest is counted as "unpinned", kernel threads apart.
- PWA: CPU and GPU tiles in lamacchinina and Telemetria; tapping CPU opens `#/cpu`: one bar per core (grey < 60 %, amber 60-85 %, red ≥ 85 %, offline dashed), up to 4 per row, chips with the pinned processes above each core, a legend with one colour per process, "N processi non vincolati", Back to where you came from.
- Not verified with the stack up: tonight the GPU read 0 % (runtime-suspended) and no user process was pinned.

## 8. LLM2 (stream C) and its on/off (two-llm-arch only)

On `two-llm-arch` only. Details: `streams/C_llm2.md`, `streams/C_llm2_onoff_node.md`, `streams/C_llm2_onoff_app.md`; the user guide is `output/two_llm/README.md` (on that branch) and the recon `output/two_llm/recon.md`.

**What it is.** `llm2_mpc_param_node` (package `llm`) sends a fixed JSON of six MPC cost weights to its own llama-server every 2 s and asks for it back verbatim (temperature 0, a `json_schema` with exactly those keys, a cached fixed prompt prefix); it never overlaps requests (a busy tick is counted `skipped`); a valid reply goes out as `f1tenth_messages/MpcParamUpdate` on `/llm2/mpc_params`; `/llm2/status` (JSON string) reports every tick. `mpc_corr`, when `llm2_params_enabled`, parks an update in the callback and applies it at the top of the next control loop through `set_parameters_atomically` (so `ros2 param get` shows it), then echoes it on `/mpc/applied_params` with the same seq. The mission logger records all three topics.

**Switches, all off by default:** `enable_llm2` (stack_bringup: starts only the second llama-server and the node, via `llm/launch/llm2.launch.py`), `llm2_params_enabled` (mpc_corr: false = no subscription at all), `llm2_port` **8084**, `llm2_model` `qwen2.5-3b-instruct-q5km` (same `.gguf` as LLM1, run with `-ngl 999 -np 1 -c 4096`: 144 MiB of KV cache instead of 1152). On the supervisor path, run `ros2 launch llm llm2.launch.py` by hand and add `llm2_params_enabled: 'true'` to the navigation entry. With every flag at its default the launch starts exactly the same processes as `campaign-final` (§9).

**Whitelist** (declared units, before `scale_stage_weights`): `w_term` [2, 20], `w_psi` [1, 10], `w_psi_stage` [1, 12], `w_corr` [0.5, 9], `w_v` [1, 10], `w_du_delta` [2, 23.81]. One bad name, NaN/inf or out-of-range value rejects the whole message; nothing is clamped. Only `mpc_w_*` weights can ever be listed, never `w_obs`. With `allow_reverse` on, `w_v` above 4.3 is rejected (it would make reversing cheaper than standing still). Mock values = today's launch-path values: `w_term` 9.0, `w_psi` 4.5, `w_psi_stage` 4.7736, `w_corr` 3.5714, `w_v` 4.2857, `w_du_delta` 8.33.

**Model:** `Qwen/Qwen2.5-3B-Instruct-GGUF` Q5_K_M, 2 438 740 384 B, sha256 `2c63dde5f2c9ab1fd64d47dee2d34dade6ba9ff62442d1d20b5342310c982081`, identical on Hugging Face, linus and the Orin (nothing new to download on the car). On linus: llama.cpp b9148 Vulkan prebuilt (same build as the Orin's), user-level in `~/projects/llm/`, on the RTX 5050.

**Measured on linus** (5 min at 0.5 Hz against a real llama-server):

| | |
|---|---|
| replies | 151/151 ok, 0 invalid / timeout / error / skipped |
| echo_match | **100 %** (151/151; 161/161 over the whole run) |
| latency vs the 2 s period | min 854, **median 862**, p90 870, p99 976, **max 1062 ms** (slowest = 53 % of the period) |
| split | prefill 10.5 ms (prefix cached), generation 850 ms for 82 tokens |
| into the MPC | 161 `/llm2/mpc_params` → 161 `/mpc/applied_params`, median 18.6 ms, weights unchanged |
| first request after a server start | up to 26 s (Vulkan pipeline build); the node's 90 s warm-up timeout covers it |
| GPU | +2.6 GB for LLM2 (3655 → 6294 MiB max during requests) |
| both models loaded | they do not fit in linus's free VRAM, so LLM1's stand-in ran on the CPU: 6177 MiB GPU total (LLM2 2587), stand-in RSS 3.6 GB. Orin estimate ~6.6 of 62.8 GB, **not measured**: tegrastats commands in README §4 |

**What blocks real tuning later** (README §5): the weights are read once at startup and there is no parameter callback (a hand `ros2 param set mpc_w_*` changes nothing; only the LLM2 path re-reads them); declared and effective values differ by ×0.35 (`scale_stage_weights`) and `rate_weight_factor` scales `w_du_delta` again; `w_corr` is overridden on arc corridors; there is no lateral-offset reference; `vdes` is not a parameter; with reverse on `w_v` can only go down unless `w_rev` moves with it; the prompt has no observation input; there is no step limit and no revert to defaults when LLM2 goes silent; Orin latency is unmeasured.

**LLM2 on/off (your addendum).**
- Node: service `/llm2/set_active` (`std_srvs/SetBool`) answers `active` / `paused`; read-only param `start_active` (default true). Paused: the timer keeps running, no request goes to the server, nothing is published on `/llm2/mpc_params`, `/llm2/status` keeps coming every tick with `"state": "paused"`. A request in flight at the pause finishes and is discarded (counted `discarded`), even if you resume before it returns. **Pausing does not revert the MPC params** (the MPC keeps the last applied values; README §1), and llama-server keeps the model loaded.
- Gateway (`5f5ff56`): `POST /api/llm2 {"active": bool}` calls `/llm2/set_active`. It uses the same auth as the other commands, runs in the thread pool with a 2 s timeout and allows one call at a time (a second gets 409). The 1 Hz status carries `status.llm2` = `{state: active | paused | not_running, why, latency_ms, outcome, counters, age_s, pending}`. `not_running` means no `/llm2/status` for 5 s (gateway monotonic clock) or no service. The mock simulates every state (knobs `llm2_running`, `llm2_active`, `llm2_fail`, `llm2_delay_s`).
- lamacchinina (`c9a0859`): an "LLM2 · Parametri MPC" row below PROCESSI. Its switch shows the state LLM2 reports, not the last tap, plus the last latency and ok / invalid / timeout. It is greyed with "LLM2 non attivo" when LLM2 is not running or the gateway is older. No confirmation. On failure a red toast appears and the switch stays at the real state.
- Review fix (`b89a3bd`): a deeply nested `/llm2/status` could stop the gateway's whole ROS executor, teleop and the E-STOP answer included. It is now dropped like any unusable status. Review and tests: `streams/C_llm2_onoff_app.md`.

## 9. Verification after integration (lead)

Each stream's own tests, mutation checks and baseline comparisons are in its `streams/*.md`. On top of that, the lead after every integration step (linus, Jazzy, ROS domain 74, localhost only; failures judged against `night-start/campaign-final`):

| step | build | tests |
|---|---|---|
| + walls (A) `6777c61` | ok | perception wall/tracker/node files 261, mpc ramp + wall boundaries 95, behavior wall lock 59 (**py_trees_ros stub run**, it is not installed on linus), params 80: all passed |
| + reports (D) `7bf99f9` | ok | logger overlay + logger node 58, perception (yolo, scene_mask, front_clearance, wall confirm, tracker) 355, params 80: all passed |
| + reverse (B) `ec288e7` | ok | mpc reverse files + ramp + walls: 228 passed, 2 xfailed |
| + app `4370897` | ok | gateway 105 (its venv), f1tenth_ctl 113, telegram 85 + 18 skipped: all passed. The merge changes no launch file, `stack_params` or bringup; outside the app packages it only adds `tools/test_gateway_humble.sh` and a CLAUDE.md section |
| `two-llm-arch` rebased (`eb9be96`, no conflicts) | ok | mpc LLM2 + ramp + walls + reverse 203, llm2 51, bringup LLM2 gate 4, mpc_corr launch config 9, logger node 45, params 80, wall confirm 33: all passed |
| + LLM2 toggle `5f5ff56`, `c9a0859` and review fixes `b89a3bd` (`two-llm-arch`) | no colcon build needed (the gateway runs from source, the PWA is npm) | gateway 110 passed on Jazzy (its venv) and 110 on Humble / Python 3.10 (`tools/test_gateway_humble.sh`; f1tenth_ctl 98 + 15 skipped). PWA `tsc` + vite build ok. e2e webkit-portrait 17 passed (implementer, at `c9a0859`) |

**Default launch unchanged on `two-llm-arch`:** a dry walk of `stack_bringup.launch.py` (every include, condition and OpaqueFunction evaluated, nothing started) on `campaign-final` and on `two-llm-arch` gives the **same processes**. The only differences: three new launch arguments (`enable_llm2=false`, `llm2_model`, `llm2_port=8084`), the LLM2 include skipped by its condition, and one extra `mpc_corr` parameter `llm2_params_enabled=False` (= no subscription).

**Humble build on the Orin** (`~/dev_ws/f1tenth_more_buildcheck`, only `/opt/ros/humble` sourced, `colcon build --symlink-install --continue-on-error`, build only):

| commit | packages | Python 3.10 syntax check of every tracked `.py` |
|---|---|---|
| `ec288e7` (A + D + B) | 38/38 finished | 0 errors |
| `4370897` (final `campaign-final` before this report) | 41/41 finished (+ gateway, ctl, telegram) | 0 errors |
| `b89a3bd` (`two-llm-arch` with the LLM2 toggle and its review fixes; the rebase onto this report adds only the report files) | 41/41 finished | 0 errors |

Only external packages printed stderr warnings (teleop tools, `vesc_*`, `serial_driver`, `zed_components`).

**Humble unit tests on linus** (image `f1tenth_more:cpu`: Humble, Python 3.10.12, domain 161, sources exported read-only from `two-llm-arch`): llm2 node/protocol/launch 51 passed, bringup LLM2 gate 4, params 80, perception wall confirm + tracker 154 passed, mpc ramp + llm2 params + reverse: all of tonight's new tests pass. The remaining failures are the image's, identical on `night-start/campaign-final` in the same container: `osqp` not installed (the RTI solver tests), a numpy/cv2 mismatch (`scene_mask`, the logger overlay), no `py_trees`.

## 10. Packages to rebuild on the car

The Orin's workspace is `--symlink-install` (Python packages are egg-links), so most Python edits are live after a pull; new modules, entry points, data files and messages need the rebuild. Rebuild the whole list anyway, it is quick.

**`campaign-final` / `main`**
```bash
source /opt/ros/humble/setup.bash
cd ~/dev_ws/f1tenth_more && git pull   # after the push, campaign-final is a fast-forward
colcon build --symlink-install --packages-select \
  f1tenth_params f1tenth_perception f1tenth_behavior f1tenth_logger mpc_controller \
  f1tenth_gateway f1tenth_ctl f1tenth_telegram
systemctl --user restart f1tenth-gateway         # the gateway runs from source in its venv
```
- PWA: on linus, `npm run deploy` in `apps/pwa` (then reload the app on the iPhone; the service worker may keep the old UI until then).
- Telegram bot (runs from `~/dev_ws/f1tenth_more_telegram` on linus): update that checkout and `systemctl --user restart f1tenth-telegram` to get the "no countdown" wording.

**`two-llm-arch`**: all of the above, plus
```bash
colcon build --symlink-install --packages-select \
  f1tenth_messages llm f1tenth_control f1tenth_navigation f1tenth_bringup
systemctl --user restart f1tenth-gateway
```
and the PWA deploy from the `two-llm-arch` checkout (the LLM2 toggle). `f1tenth_messages` gains `MpcParamUpdate.msg`; `llm` gains the `llm2_mpc_param_node` entry point. Going back to `campaign-final` later needs no clean: the extra message is harmless.

## 11. Car checklist for tomorrow, in order

**1. Reverse from the app, wheels off the ground.**
- Guida, pull the gas pad down: amber **RETROMARCIA**, `ros2 topic echo /commands/motor/speed` goes negative (full stick ≈ −0.4 m/s × 4516.9 ≈ −1807 ERPM), the wheels turn backwards. Release: 0 at once. Forward → reverse: 0 for ≥ 0.3 s ("attesa arresto").
- A thumb resting low on the pad now reverses (the pad reads AVANTI / RETRO at its ends).
- If it misbehaves: gateway settings `max_reverse_speed_mps` (0.4; **0 turns reverse off**), `reverse_hold_s` (0.3), `stop_speed_mps` (0.05); defaults in `f1tenth_gateway/config.py:60-62`, overridable in the gateway's config yaml. If the wheels do not turn backwards at all: `speed_min` in `f1tenth_bringup/config/vesc.yaml:96` must stay −23250 (vesc_driver treats a missing `speed_min` as 0).

**2. MPC automatic reverse when stuck.**
- Drive a mission into a dead end. The reverse should end `distance_done` at 0.5 m (`rear_corridor_length_m`), counted as the larger of the odom and map distances. Measure one with a tape. Bag it with the "Next run" command in `output/mpc_v2/STATUS.md` (now with `/tf`, `/ekf_global/odometry/filtered`, `/slam/pose_calibrated`).
- If a reverse ends early: the map count over-read (jumps of up to 0.45 m in one tick were seen on 10-20); the mpc_corr log shows both counts. If it is cut after 0.5 s: `bt_disallowed` from the BT's `IsProximityTooClose` toggling `/bt/allow_reverse` (BT logic, not changed tonight). Knobs: `reverse_auto_enabled`, `reverse_timeout_factor` (2.0), `reverse_max_attempts`, `stuck_time_s`.

**3. Approach-the-wall mission with a side wall during a turn (the lock).**
- Run it with `enable_mission_logger:=true` (so it is bagged; R029/R031 never were). Watch the mpc_corr `RAMP | aiming at ...` lines and `/mpc/stop_wall_id`: during the swerve the ramp must not aim at the side wall; the move ends on the target wall, or on `timeout_sec`.
- If it misbehaves: `wall_lock_enabled: false` gives exactly the old behaviour (BT and ramp); re-acquisition gates `wall_follow_gate_m` (0.30) and `wall_follow_gate_deg` (15°, **never measured**). Replay the bag with `tools/wall_lock_replay.py`.

**4. Wall confirmation level: too slow or too eager.**
- Start with `wall_debug_tracks:=true` (stack_params `lidar_front_wall_wall_debug_tracks`) and read `confirm_reason` on `/perception/virtual_wall/debug`.
- Too slow: `hits` → lower `wall_confirm_min_consecutive_hits` (5 → 3); `dist_jitter` → raise `wall_confirm_max_dist_jitter_m` (0.05 → 0.08); `angle_jitter` → raise `wall_confirm_max_angle_jitter_deg` (3 → 5); `length` → lower `wall_confirm_min_length_m` (1.0).
- Too eager (phantom walls become FRONT or MPC rows): raise the hits, lower the jitter bounds. Glass walls never confirming: `wall_confirm_require_lidar: false` (then stereo hits count on fresh-camera cycles).

**5. CPU view and the new run reports.**
- With the stack up, tap CPU in lamacchinina: the pinned nodes should sit on their cores (ekf 0-1, slam 2, foxglove 3, behavior 4, detection/obstacle 6-7, yolo 8-9, mpc_corr 10-11), and the GPU tile should move off 0 %.
- After a mission with `enable_mission_logger:=true` (sqlite3 storage): `ros2 bag info` shows `/camera/debug_image/clean/compressed` at ~5 Hz; the finalize log says "N overlay snapshot(s)"; Run → Report shows light fills, small LiDAR dots, a small legend, readable image, stills in time order. Tune `mission_report_overlay_alpha` / `_point_px` / `_legend` (stack restart), or re-render with `ros2 run f1tenth_logger mission_debug_video <run_dir> --alpha … --point-px … --no-legend`.

**6. (two-llm-arch only) LLM2 with the mock: the drive must be unchanged.**
- Check port 8084 is free on the Orin. Launch with `enable_llm2:=true llm2_params_enabled:=true` (supervisor path: see §8). `/llm2/status`: outcome ok, `echo_match` true; `/mpc/applied_params` follows `/llm2/mpc_params`; drive the same mission with and without: same behaviour.
- **On two-llm-arch with enable_llm2: pause LLM2 from lamacchinina, check `/llm2/status` = paused and no new `/llm2/mpc_params`; resume.**
- tegrastats idle and during requests: commands in `output/two_llm/README.md` §4.

## 12. sudo

No step needed sudo, so there is no `sudo_tomorrow.sh`.

## 13. Push

Pushed on 2026-10-22 at 11:11 from linus, right after this report commit. Every push was a fast-forward of the remote; nothing was forced.
- **Before the push**, `git fetch origin` showed the remote tips unchanged since the night started: `origin/campaign-final` `1ee82c8`, `origin/pwa-control` `adb9143`, `origin/main` `e47e646`. `two-llm-arch` did not exist on origin.
- **The vesc submodule** needed no push: `0e74a60` is already on GitHub (`refs/heads/campaign-final`, `refs/heads/scene-graph`).

```bash
cd ~/dev_ws/f1tenth_more_campaign
git push origin campaign-final     # 1ee82c8 -> this report commit
git push origin pwa-control        # adb9143 -> 87e6f69
git push origin main               # e47e646 -> this report commit
git push -u origin two-llm-arch    # new: this report commit + stream C + the LLM2 on/off
```
Not pushed (local bookkeeping): the `night-walls`, `night-reverse` and `night-reports` branches, and the `night-*` tags.

## 14. Where things are

**Worktrees on linus** (all clean; `~/dev_ws/f1tenth_more` was not touched):

| worktree | branch |
|---|---|
| `~/dev_ws/f1tenth_more_campaign` | `campaign-final` |
| `~/dev_ws/f1tenth_more_walls` | `night-walls` (merged) |
| `~/dev_ws/f1tenth_more_reports` | `night-reports` (merged) |
| `~/dev_ws/f1tenth_more_reverse` | `night-reverse` (merged; was `mpc_corr_v2_reverse`, which is unchanged) |
| `~/dev_ws/f1tenth_more_pwa` | `pwa-control` (merged; the untracked `output/pwa_screens/browser_disconnected.png` is still there) |
| `~/dev_ws/f1tenth_more_two_llm` | `two-llm-arch` |

**Orin:** `~/dev_ws/f1tenth_more_buildcheck` (detached clone, safe to delete when you like; it now sits at `b89a3bd`) and `/tmp/night_bundles/`. Nothing else was written there.

**User-level installs on linus (stream C):** llama.cpp b9148 Vulkan in `~/projects/llm/llama-b9148/`, the model in `~/projects/llm/models/`. Not running.

**Still running, not touched:** the mock gateway from yesterday (PID 3095461, 127.0.0.1:8770) serves the `pwa-control` worktree's rebuilt `dist/` with yesterday's gateway code (so no reverse, no `status.system` until it is restarted).

**Scratch** (raw logs, test outputs, mutation scripts, the 5-minute LLM2 run, example reports): `/tmp/claude-1000/-home-andreas-dev-ws-f1tenth-more/b4f59daa-6343-486c-9d59-6918832814b8/scratchpad/` (`stream_*`, `lead/`, `results/`). The morning's toggle review (briefs, mutants, probes, logs): `/tmp/claude-1000/-home-andreas-dev-ws-f1tenth-more/f5bd9fe9-1efe-4113-b34c-300e74c86b36/scratchpad/toggle_review/`. Both live in `/tmp`, so they go at the next reboot.
