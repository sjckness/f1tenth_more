# Known bugs — debug backlog for main

State of `main-consolidation` as of 2026-10-22: campaign-final fa5471f plus
two-llm-arch, campaign-final-yolo-gpu and comms-base. **Nothing here was
fixed.**

Sources:
- the notes and commit logs on campaign-final, two-llm-arch, comms-base and
  campaign-final-yolo-gpu, read on linus;
- what I noticed myself while merging, marked **(merge)**;
- anything I inferred from code rather than read in a note, marked
  **(inferred)**.

Severity:
- **high**: can move the car wrongly, defeat a stop or e-stop, or kill a
  control path;
- **med**: a feature does not work or gives wrong data;
- **low**: cosmetic, logging, docs.

Status says whether the fix is on main. **A fix on main may not be on the car
today** (conflicts.md §2.2): the Orin runs fa5471f with 32 files reverted to
1ee82c86, including `MPC_corr.py`, `reverse_manoeuvre.py` and
`stack_params.yaml`.

Abbreviations used in the "Where" column:
- `NR` = `output/night_2026-10-21/night_report.md`;
- `B12` = `output/night_2026-10-21/streams/B12_reverse.md`;
- `ST` = `output/mpc_v2/STATUS.md`;
- `PR` = `output/pwa_report.md`;
- `TT` = `output/telegram/test.md`.

## A. Reverse, odometry, VESC (the four named items first)

| # | Symptom | Where | Sev | Status |
|---|---|---|---|---|
| A1 | **Reverse not seen by odometry.** **The car today runs the old version**: `reverse_manoeuvre.py` is at 1ee82c86, which lacks both fixes. 10-20 15:39:09: the reverse ended on `timeout` after backing 0.77 m (odom) on a 0.5 m strip. `travelled` was counted in map through a moving map→odom: the count sat at 0.000 for 1 s, then jumped 0.45 m in one tick. | `reverse_manoeuvre.py` `_reverse_tick` (~l.660); ST:48-64; B12:32-49 | high | Fixed on main: dd129b9 + 6536877, `travelled = max(odom, map)`. Never run on the car. Not on jazzy. |
| A2 | **VESC reverse ERPM polarity** was unknown: no negative sample before 10-20. | vesc `vesc_to_odom_backup.cpp`, `odom_speed.hpp`; ST:68-69; B12:20-30 | high | Resolved: negative, `/odometry/filtered` −0.36 m/s. Pinned by vesc 0e74a60 / 9cffbaf (`test_odom_speed.cpp`, `test_ekf_reverse_visible.py`). jazzy pins vesc 2848a9f: sign fix yes, pin and tests no. |
| A3 | **No forward v ≥ 0 guard.** Once `max_reverse_speed_mps` became 0.4, a forward braking undershoot commanded down to −0.153 m/s with forward steering: 64 of 568 forward ticks, 15:37 run. | `MPC_corr.py` `vMin`, final gate in `_publish_drive`; commit a8a03ea; ST:41-42; B12:53-56 | high | Fixed on main: a8a03ea. **origin/jazzy lacks it and adds James's a596962** (negative `vdes` through the forward solver). |
| A4 | **A reverse ended on `bt_disallowed`** after 0.5 s: `/bt/allow_reverse False: IsProximityTooClose blocks the reverse`. The log line does not say which trip fired: side/rear 0.15 m, or front under `front_hard_floor_m` 0.20 **(inferred)**. | `is_proximity_too_close.py` `blocks_reverse()`; `reverse_integration.py`; ST:65-67; NR:396 | med | Cause diagnosed. Behaviour unchanged. The log line should name the trip. |
| A5 | Odometry reads about 17-20 % short going forward. In a reverse, odom moved 0.81 m while the clearance ahead grew only 0.60 m. | `docs/bringup_checklist.md:294`; `docs/DIAGNOSTICS.md:56`; ST:90-91 | med | Open: tape-measure a reverse |
| A6 | `vesc_driver` `erpm_deadband` of 500 ERPM (about 0.11 m/s) zeroes speed in both directions. A slow crawl is invisible to odometry. | B12:27 | med | Mitigated by A1's max(odom, map) |
| A7 | The rear is blind: the LiDAR never scans the rear 90°, there is no rear sensor, and a reverse trusts the SLAM map plus memory. `car_rear_offset_m` 0.082 and reverse braking are unmeasured. | `output/reverse/recon.md:114-150`; `output/mpc_v2/reverse_and_memory.md:235-248` | high | Open |
| A8 | A fresh but wrong `reverse_active` from mpc_corr drops front protection to the 0.20 m floor. | `reverse_and_memory.md:704-707` | high | Accepted risk |
| A9 | `allow_reverse` (the multi-candidate reverse MPC) dithers or never enters doors. With SLSQP it raises on the first tick. Its `w_rev` invariant breaks above 0.5 m/s. | `output/reverse/report.md:9-22,100-140,211` | med | Off by default. See conflicts.md §2.1. |
| A10 | The sim LiDAR faces rearwards and the car's faces forwards, so sim reverse tests see the opposite side. | `output/reverse/recon.md:150` | low | Open |
| A11 | `/sensors/core` `distance_traveled` counts 9807 ticks while ERPM is 0. | `output/wall_v2/report.md:54-55` | low | Open |

## B. Control, MPC, speed

| # | Symptom | Where | Sev | Status |
|---|---|---|---|---|
| B1 | Speed gap: the car drives about 0.25 m/s when 0.5 is asked. The offset-free law tops out at about 0.43 m/s real and regulates the measured (short-reading) speed. The VESC delivers about 76 % of commanded ERPM. `velocity_correction` is empty. | `output/speed_gap/report.md:9-20,345-351`; `offset_free.md:17,380-385` | med | Mitigated (offset-free on by default). Never run on the car. |
| B2 | The threshold stop overshoots by 0.33, 0.68 and 1.27 m at 0.6, 1.0 and 1.5 m/s (nominal sim). | `output/stop_distance/report.md:5-9` | high | Mitigated: `enable_stop_ramp` true. Sim only. |
| B3 | The soft `w_obs` does not hold clearance for an obstacle within about 0.3 m of the straight path. Obstacle and boundary rows are soft. | `output/obstacles_v2/report.md:17-19` | high | Open, no fix |
| B4 | A wall more than 0.5 m inside the car's position makes the QP infeasible, and the fallback tiles `last_u`. | `output/avoidance/wall_boundaries.md:91-97` | high | Mitigated (`boundary_slack_max`) |
| B5 | While the front-wall lock holds, the deceleration ramp does not slow for a nearer surface in front of the locked wall. In a swerve (locked wall, another wall FRONT) there is no wall ramp at all: only the wall rows, avoidance, stop+reverse and the 20 cm LiDAR stop act. The older defect, where the ramp read raw `/perception/virtual_wall` and stopped the car by the side wall (P002-R029), is fixed on main by 00635090 (`ramp_wall_source.py`). | `ramp_wall_source.py`; NR:140,186-208,224 | high | Residual by design. The fix is not on the car (§2.2). |
| B6 | Old LOG items, open as of 09-02 and not re-checked since: map/costmap drift while standing still with instant stops; "never stops at 1 m front_clearance"; 7.8 % primal-infeasible QPs give stop-then-resume; service names are not namespaced, so a test ran on production. | `docs/history/LOG.md:726-802` | high | Open, maybe stale |
| B7 | 13 stale mpc_controller tests (21 with env failures). They hide config questions: corridor half-width 0.86 vs 1.37 needed to pass a person; wall-turn trigger 2.30 vs 3.71 m; `corridor_heading_return` doc vs value. | `output/mpc_known_failures.md:13-27` | med | Open |
| B8 | `ros2 run mpc_controller model_log` cannot work: setup.py registers `model_log = mpc_controller.model_log:main`, and `model_log.py` defines no `main`. **(merge)** static check, pre-existing on campaign-final. | `src/f1tenth_control/mpc_controller/setup.py`, `model_log.py` | low | Open |
| B9 | Dead MPC code on main: `andre_mpc_node` + `mpc.launch.py` (never launched) and `reverse_mpc.py` (off). **(merge)** | see conflicts.md §2.1 | low | Decision pending |

## C. Behaviour tree, e-stop, safety

| # | Symptom | Where | Sev | Status |
|---|---|---|---|---|
| C1 | `IsProximityTooClose` has no `/scan` freshness guard: a dead urg_node after a clear scan disarms the e-stop. RELIABLE depth 10 means a reading can be up to 250 ms stale. | `output/cpu_cleanup/report.md:453`; `rate_audit.md:362` | high | Open |
| C2 | `IsBatteryLow` and `IsSystemOverheated` check no message age after the first message. `CheckStopCondition`'s distance, goal and odom are reused forever, and `timeout_sec` is optional. | `src/f1tenth_behavior/README.md:314-317` | high | Open |
| C3 | E-stop latch is lost when `behavior` restarts (the respawned node reads clear), and there is no `/mission/reset_estop`. Resetting it means restarting `behavior`, which unmasks the lower mux lanes: a plain restart moved the car 1.53 m in sim. | `output/telegram/estop_proposal.md:26-29`; TT:266-285 | high | Open |
| C4 | Even while latched, the mux lets single lower-lane commands through under load (0.008 m in 3 s, sim). | TT:283-285; `src/f1tenth_ctl/README.md:217-220` | high | Open |
| C5 | hard_stop's self-mask is blind at ≤ 0.12 m and "cannot be relied on to fire at contact". | `output/avoidance/log.md:82-99` | med | By design |
| C6 | False COMPLETE: `distance_reached` uses `math.hypot` instead of the along-line projection, so a move can end up to 13 % early. | `condition_eval.py:292`; `docs/history/LOG.md:94-105` | med | Open |
| C7 | `reduce_speed` / `reduce_speed_for` and the move `vdes` are ignored stubs. A hold with `goal_reached` or a `forward_only` resume never auto-resumes. | `handle_object_action.py:183-188,221-244` | med | Open (no mission uses them) |
| C8 | If the camera TF does not resolve, `front_distance` stays −1 and front_clearance missions run to their timeout. | `output/campaign_final_report.md:117` | med | Known |
| C9 | `/costmap/front_clearance` read the floor down to 0.066 m for 9 s while driving, and the BT stop did not fire (run 2026-10-21T12-25-22, bf7938). | `output/speed_gap/report.md:453-477` | med | Open question |

## D. Perception: walls, doors, objects, YOLO

| # | Symptom | Where | Sev | Status |
|---|---|---|---|---|
| D1 | The laser's static TF yaw is "NOT a real calibration", and the e-stop's angular coverage rests on it. Wheelbase 0.305 is unmeasured. The base_link origin docs disagree by 0.1525 m. | `docs/bringup_checklist.md:283-292`; `docs/history/LOG.md:711-724` | high | Open |
| D2 | Tracker `max_sigma_rho_m` tests the odom rho, so a wall about 20 m along its own direction from the EKF origin is deleted at birth: no FRONT, no MPC row. | `output/night_2026-10-21/streams/A_walls.md:24-27`; NR:165 | high | Open (pre-existing) |
| D3 | Wall confirmation level (5 hits in a row, 5 cm jitter): a head-on wall at 1.0 m/s never confirmed (sim). On corridor10m_…T131815, FRONT now appears at 15.7 m vs 20.4 m and is held 51 % vs 82 %. | `output/front_wall_lock/design.md:362-385`; NR:217,232 | high above 0.7 m/s | Open |
| D4 | The wall lock can lock the wrong wall mid-swerve. A nearer surface in front of the locked wall is ignored by the ramp (B5). The 15° gate is not measured. | `design.md:173-175,185-188,242-254` | high | By design; `wall_lock_enabled: false` restores the old behaviour |
| D5 | The front wall was dead-reckoned for 8.5 s without the contract ever saying NO_DATA. The right wall is missing about 2/3 of the time. | `output/run_corridor10m_20261019T131815/report.md:30,331-341` | med | Open |
| D6 | ZED detections die after about 0.8 s, so recognised-object perception was dead about 98 % of two corridor runs. | `output/avoidance/log.md:147-162` | high | Open, no fix proposed |
| D7 | Unknown LiDAR objects "blocked the car". The semantic-layer LiDAR step takes an estimated 6-8 ms against a 3 ms budget. | commit ed2e4a4; `output/obstacles_v2/report.md:43-45` | med | Disabled (`unknown_objects_mode` off) |
| D8 | Doors: a closed glass door in a solid wall can confirm as open. `corridor_perception_c` fails on CMake ≥ 3.26 (`M_PI`), so the Python fallback runs (24.6 vs 1.1 ms). | `output/perception/doors.md:370-382,429-434` | med | Doors off by default |
| D9 | YOLO has no segmentation TensorRT engine: eager fp32 at about 87 % of a core, and detections about 278 ms behind the image. torch fails with `ImportError libcudss` unless `LD_LIBRARY_PATH` is set. | `output/perf/yolo_recon.md:24,53,232-233` | med | Open; env mitigated |
| D10 | The YOLO GPU branch changes four defaults (GPU pipeline on, 15 Hz cap, 2 threads, blocking sync). The pipeline was validated on linus only (48 frames identical). `test_yolo_detector_node.py` was not run on linus (needs ROS). **(merge)** | `yolo_detector_node.py`, `yolo_gpu.py`; conflicts.md §2.3 | med | Never run on the Orin |

## E. Logging, bringup, discovery

| # | Symptom | Where | Sev | Status |
|---|---|---|---|---|
| E1 | The recording silently lost the whole camera half (corridor10m_…T131815). A manual `-a` recorder lost 24 % of `/scan` and 48 % of odom. mcap is missing, so storage falls back to sqlite. The overlay reads sqlite3 only. | `output/nightly/f5_recorder.md:41,64-70`; NR:165 | med | Open |
| E2 | The Discovery-Server partial-visibility race is accepted. 33 launch files lack the DS guard. | `docs/history/LOG.md:170-180` | med | Open |
| E3 | Every stack node runs as a DS super client (`ctl.env` mirrors `~/.bashrc`). On linus this put the DS at 50 % of a core. | `src/f1tenth_ctl/config/ctl.env:37-43`; `output/telegram/discovery.md:148-192` | med | Open on Humble main. **Fixed on main-jazzy** (3ae4bef): ctl.env exports no ROS_SUPER_CLIENT; the probe, the monitor and the gateway set it for themselves. |
| E4 | Run provenance: every run is dirty with no diff captured, and `meta.git_commit` in the `.mat` is wrong for 50 runs. | `output/campaign_final_report.md:37-47` | low | Open |
| E6 | `f1tenth_ctl` (and through it the Telegram bot and the PWA gateway) always sources `$HOME/dev_ws/f1tenth_more/install/setup.bash`, which is hard-coded in `ctl.env`. Whichever worktree it runs from, it starts and inspects the **campaign** install, never `~/dev_ws/f1tenth_more_main`. **(merge)** | `src/f1tenth_ctl/config/ctl.env:1-4` | med | Open on Humble main. To drive main through the app, pass an env file via `F1T_CTL_ENV` (report.md §8). **Fixed on main-jazzy** (3ae4bef): ctl.env sources the install of its own checkout (`F1TENTH_WS` overrides it). |
| E7 | The run-report overlay (`mission_overlay.py:300-304`), `mission_debug_video.py:96-117` and the replay tools (`tools/door_replay.py`, `obstacles_v2_replay.py`, `corridor_wall_replay.py`, `extract_backends_equiv.py`, `calib_wall_check.py`, `front_phantom_replay.py`, `measure_stop_distance.py`, `scene_labels_replay.py`, `test_campaign_backfill.py`) read sqlite3 (`.db3`) only. On Jazzy the logger's old `mcap` default really recorded mcap. **(jazzy)** | `src/f1tenth_logger/`, `tools/` | med | main-jazzy records sqlite3 by default (as the car); format-agnostic readers (rosbag2_py `storage_id=''` or rosbags `AnyReader`) are backlog. |
| E5 | **The car runs an uncommitted tree:** fa5471f with 32 files at 1ee82c86, on branch `stack-campaign-final-2026-10-22`. Run provenance points at fa5471f, which is not what ran. **(merge)** | Orin `~/dev_ws/f1tenth_more`; conflicts.md §2.2 | med | Open |

## F. LLM planner (LLM1) and LLM2

| # | Symptom | Where | Sev | Status |
|---|---|---|---|---|
| F1 | LLM1 (qwen2.5-3b) does not reliably refuse selections: "the person on the left" is planned as go_to the nearest person. The llm suite has 14 failures with no documented cause. | `src/f1tenth_intelligence/llm/README.md:197-200`; `output/mpc_known_failures.md:43-45` | med | Open |
| F2 | With `enable_intelligence` now false on main, every planner path fails its warm-up until the stack is started with `enable_intelligence:=true` or `stackctl.py start intelligence`: `ros2 run llm llm_planner_node`, f1tenth_ctl's planner unit, and Telegram/PWA "plan". **(merge)** This is the intended consequence of the decision, listed so it is not debugged as a bug. | `ctl.env` F1T_PLANNER_CMD | low | By decision |
| F3 | LLM2 can only be started through `stack_bringup.launch.py enable_llm2:=true`. `supervisor_bringup.launch.py`, which the car and f1tenth_ctl use, has no `enable_llm2`: you need `ros2 launch llm llm2.launch.py` plus a machine-local components.yaml edit. So it is not the same pattern as LLM1. | `output/two_llm/README.md:57-72` | med | Open |
| F4 | LLM2 has no step limit and no revert when it goes silent. Pausing it keeps the last parameter set until mpc_corr restarts. The mock echo resets launch overrides. A manual `ros2 param set mpc_w_*` has no effect. | `output/two_llm/README.md:46-51,384,428-436` | med | Open; off by default |
| F5 | LLM2 was never run on Humble/the Orin. The app toggle was never run against the real node. Its trust window assumes `rate_hz` 0.5. | `output/two_llm/README.md:366-369`; `streams/C_llm2_onoff_app.md:54-56` | low | Untested |

## G. App: Telegram bot, PWA ("lamacchinina"), gateway, f1tenth_ctl

| # | Symptom | Where | Sev | Status |
|---|---|---|---|---|
| G1 | The gateway's ROS side can die silently: `RosBridge._spin` swallows any executor exception without logging. After that, teleop, telemetry and the E-STOP answer are dead while the app still looks alive. | `f1tenth_gateway/ros_bridge.py:137-141`; `streams/C_llm2_onoff_app.md:73,77` | high | "Left as is". The LLM2 trigger is fixed on two-llm-arch (175da11, now on main). |
| G2 | In Guida with reverse on, a thumb resting low on the gas pad backs the car up at up to −0.4 m/s, with no rear sensing (A7). | NR:15,277; `streams/B3_app_reverse.md:58` | high | Never tried on the car |
| G3 | Telegram 🛑 and the app's EMERGENZA are not the primary e-stop. Telegram takes 3-5 s from tap to zero drive (sim). The app latency is unknown. Neither was tested end to end against the real BT and mux. | `f1tenth_telegram/README.md:14-21`; `f1tenth_gateway/README.md:16-18`; PR:328-330 | high | Untested on the car |
| G4 | A plan made from the bot and one from the PWA at the same moment can swap their RESULT. The plan lock lives only in the gateway. A swapped plan could then be started **(inferred)**. | PR:390-392; `output/pwa_recon.md:381-383` | high | Open ("avoid by hand") |
| G5 | AVVIA → motion takes 12-20 s: each start opens two fresh f1tenth_ctl DS participants, and one needs 6-10 s to see `/mission/status` (measured on the car). | `streams/B3_app_reverse.md:20`; NR:156 | med | Open |
| G6 | App voice is off on the Orin: no faster-whisper in the gateway venv, so the mic is disabled ("usa la dettatura iOS"). | commit fc273b7; PR:326-327,393-394 | med | Open |
| G7 | Bot voice mishears commands: `medium` got 1 of 3 synthetic commands wrong, `small` 2 of 3. No human voice tested. | TT:494,499-503 | med | Open |
| G8 | The BT view in Telemetria stays empty if `py_trees_ros_interfaces` or `/behavior_executor_node/snapshot_streams/open` is missing on the Orin. | PR:385-387,509-510 | med | Unverified |
| G9 | Heavy subscriptions (odom, map, solver, BT) run whenever any phone is connected. Their Orin CPU cost was never measured. | PR:395-396 | low | Open |
| G10 | iOS behaviour is unverified: safe area, Wake Lock in standalone mode, MediaRecorder, multitouch. The wake-lock e2e test is flaky in headless WebKit. | PR:322-325,397 | low | Open |
| G11 | The QR code pairs Safari only. The Home Screen app has its own storage, so the token must be pasted there. | `f1tenth_gateway/README.md:237-241` | low | Open |
| G12 | UI rough edges: only the last toast shows; progress reads 0 % during the first move; the pill says READY during a mission; "no video" repeats on every Run card; battery shows V only; audio length is not capped on the client. | PR:398-407 | low | Open |
| G13 | CPU view: chip names are cut at 320 px, and it was never seen with the stack up (GPU read 0 %). | `streams/B45_app.md:89-94` | low | Open |
| G14 | Stale screenshots: `output/pwa_screens` still shows "Cockpit" and 3 tiles. The PR §10 checklist and the gateway README contradict each other on `journalctl`. The untracked `docs/pwa/` in the main checkout is the old SPEC (ce4c356). | NR:155; PR:467 | low | Open |
| G15 | Two pollers on one bot token get 409 Conflict and both lose updates. linus's IPv6 route is broken (`force_ipv4`). faster-whisper needs `av<17`. | `f1tenth_telegram/README.md:265-276,359`; TT:497 | low | Open |
| G16 | E-STOP availability fails open: if the graph cannot be queried, the button still says available. | `f1tenth_gateway/app.py:298-302`; `apps/pwa/src/store.ts:172` | low | By design |
| G17 | Never run on the car: the PR §10 checklist (the `/teleop` lane on the real mux, the watchdog, E-STOP end to end, the real `estop-reset`, auto-abort with the real BT, a mission started from the app, reverse, the CPU/GPU view); a bot-started mission; the real planner over the bot path. | PR §10; TT:181-200,320-324,372-375,426-428 | high (as a group) | Untested |

## H. Comms (comms-base, off by default)

| # | Symptom | Where | Sev | Status |
|---|---|---|---|---|
| H1 | link_manager's reconnect fails on the Orin with "Not authorized": `nmcli` network-control is `auth` until the polkit rule is installed (needs sudo; see report.md §7). | `output/comms/setup.md:108,330` | med | Open |
| H2 | The base AP's USB adapter (RTL8822BU) dropped off USB 31 times in 13 min while idle, and sent no beacons on 2.4 GHz. The base is "not dependable". | `output/comms/setup.md:264,272-299,594-598` | med | Open |
| H3 | `stackctl.py start comms` reads the yaml (`reconnect_dry_run: false`), so a "first run" through the component really runs `nmcli`. | `output/comms/setup.md:380-381` | med | Open |
| H4 | The PWA does not show the gateway's new `link` field. The Foxglove layout was never tried. The RSSI scale and thresholds are guesses. Nothing ran on the Orin. | commit 0b09632; `setup.md:10,269,473,581-604` | low | Open |

## Never validated on the car

Per the notes. Not bugs, but the first things to watch on main:
- **Reverse:** the count fix (A1), hard_stop's reverse gate (b31155e) and the real reverse distance. Steering in reverse is supported by one window only.
- **Speed and stopping:** the offset-free speed law; the stop ramp (nominal sim only).
- **Walls:** the wall lock and ramp ("never on a swerve"); the wall confirmation level (replay only); wall rows; the d_wall pass; wall_distance_node; swept_clearance.
- **Safety and CPU:** the event-driven e-stop (69d2e1c) and the Orin CPU checklist (all Orin CPU figures are estimates).
- **Perception:** doors; the obstacles-v2 upgrade path and self-mask; the YOLO seg TensorRT engine and the GPU pipeline.
- **LLM:** LLM2 on Humble; LLM1 structured output against a live model.
- **Tools and tuning:** the run-report overlay on a real run; the MPC weights closed-loop.
