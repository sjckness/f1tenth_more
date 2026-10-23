# main-jazzy, Phase 1: the port finished on linus (2026-10-23)

Plan: `~/.claude/plans/fuzzy-enchanting-teapot.md` (Humble `main` now; `main-jazzy` takes
everything in main, is ported and tested, and becomes `main` together with the Orin container).

## Phase 0 (done)
- `main` fast-forwarded to `main-consolidation` (ddd3adfc) and pushed: the consolidated Humble main.
- `main-consolidation` and `main-jazzy` pushed as branches. `campaign-final` (fa5471f) untouched.

## Phase 1 commits on main-jazzy (on top of a2fd3a86)
| commit | what | checklist |
|---|---|---|
| 48d0f80 | zed_ros2_wrapper back to v5.4.1 (86171bc), the validated pin | C15 |
| (vesc fork) | `sjckness/vesc` branch `jazzy` fast-forwarded 2848a9f -> 0e74a60, the pinned commit | C15 |
| 0f767f0 | inflate_polytope numpy-1.26 infinite loop (cherry-pick fd83d6e, P1) | C22 |
| 3ae4bef | ctl.env on Jazzy: scripts/env/jazzy.sh of its own checkout, no global ROS_SUPER_CLIENT, no python3.10 path; the gateway is its own super client | C6b, C10 |
| 0ecdd9b, af8bb90 | f1tenth_ctl sim profile for the Isaac sim (was the Gazebo `sim:=true`) | — |
| f3a2e9f | mission logger records sqlite3 by default (decision D4); mcap readers -> known_bugs E7 | C3b |
| 1c7a836, 447d61f | tools: ZED v5 remaps in replay_raw, discovery range, `ros2 bag record --topics`, typestore from the bag's distro | C9, C17, C29 |
| f9d3fa0 | YOLO scripts: no python3.10 path; Jazzy usage | C6b |
| 230f5cc | static_checks.py targets Python 3.12 | — |
| 974a845 | mpc_controller declares python3-numpy / python3-scipy | — |
| 04ef821 | Fast DDS 5 s participant lease (port of jazzy 7b301b2 + dac4bef), adapted to the Isaac sim profiles; CLAUDE.md DDS / harness / Thor sections | H1 |
| 586ad02 | system_observer: thermal zone read None (cherry-pick 931a0eb) | P2 |
| 3eb4e3f | steering calibration e-stop check waits for the graph (cherry-pick b31b76c) | P4 |

## Verification on linus (Jazzy native, `~/dev_ws/jc_env.sh` underlay)
- **Build:** `colcon build --symlink-install --packages-ignore $(cat output/jazzy_port/linus_skip.txt)`:
  32 packages, 0 errors (`build_linus.log`).
- **Tests:** `colcon test`, isolated DDS (domain 93, localhost): 5373 tests, 5226 pass, 45
  failures + 1 error, 101 skipped (`test_summary.txt`, `test_failing_ids.txt`).
  **No new failure:** every failing ID is in the port's own baseline
  (`output/jazzy_port/test_failing_ids.txt`): ament lint (copyright/flake8/pep257) in
  bringup/diagnostics/logger/llm/mpc_controller, the llm planner stub tests, 11 mpc_controller
  tests that pin older defaults, 2 logger tests that need the checkout to be named
  `f1tenth_more`, joy_teleop's dash keys. The extra skips are the gateway and Telegram tests
  (no fastapi / python-telegram-bot on linus) and 2 reverse scenarios. Upstream lint in the
  submodules (ackermann_mux, vesc_ackermann, zed_msgs) fails as before. The four new
  packages pass: f1tenth_comms 72/72, f1tenth_ctl 113/113, gateway 75 + 36 skipped,
  telegram 85 + 18 skipped.
- **Static checks:** 1 FAIL, the pre-existing model_log entry point (known_bugs B8).
- **YOLO smoke test** (`scripts/yolo_smoke_test.py`, old defaults vs GPU pipeline): PASS,
  identical detections and masks on 48 frames.
- **f1tenth_ctl `check`** with the sim profile: everything OK except the discovery server
  (no stack running).

## Not done (blocked)
- **`tools/e2e_test.sh` (Isaac, one machine):** `sim_isaac.sh` refuses to start Isaac with
  less than 25 GB free on `/`; linus has 22 GB (the first-test-campaign copy took 14 GB).
  Docker holds `f1tenth_more:gpu` (33.5 GB, 3 months old) and `f1tenth_more:cpu` (3.1 GB);
  `~/dev_ws/f1tenth_more_rust/build` is 2.5 GB. Freeing space is your call.
- **Phase 2 (Thor):** the Thor is unreachable (Tailscale offline, cable 10.42.0.2 times out).

## Decisions for you (old `jazzy` features, not brought: feature freeze)
Classified in the session; none is needed to run main-jazzy on Jazzy.
- Supervisor liveness watchdog (P5): would need re-deriving for main-jazzy (its EKF rule is
  30 Hz, main's EKFs run at 20 Hz; no `comms` component).
- Logger extra default topics (P7), camera pan (P18), semantic threshold 0.35 (P17, recommend no).
- The test-campaign MPC/mission line made only on jazzy (A1-A5, funnel, keep-outs, BLOCKED,
  wall stop latch, w_du_delta / corridor_heading_return, steering clip): main has its own
  version of each.
- 1d008532's test-campaign WIP (matlab export, run_select, corridor tracker, office YOLO model).
