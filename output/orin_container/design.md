# Orin Jazzy container: design notes for the implementing agents

Plan (approved): `/home/andreas/.claude/plans/fuzzy-enchanting-teapot.md` (phases P0–P12).
This file adds the facts and the interface contracts the parallel work packages share.
Decisions taken during the run go to `output/orin_container/decisions.md` (append: question,
decision, one-line reason). Everything unattended: never ask, never wait; decide and record.

## Ground rules (all packages)
- Agents never change git state (no commit/stash/checkout/merge/reset); the coordinator commits.
- No sudo, no apt, no ssh to any machine, no docker build/run on linus unless the package says so
  (W2 may run `docker build --check` and `docker compose ... config` locally; no image builds).
- Never `rm -rf` a variable path (`${VAR:?}` + prefix check); never `pkill -f` (self-match).
- Anything ROS in tests: isolated DDS — `unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
  FASTRTPS_DEFAULT_PROFILES_FILE; export ROS_DOMAIN_ID=<9x> ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.
- Never two colcon builds at once in the same workspace; build with `-j3`
  (`MAKEFLAGS=-j3`, `--parallel-workers 3`).
- The Bash tool shell is zsh: wrap commands in `bash -c '...'`.
- Python on the car image is 3.12 (Jazzy); keep code compatible with 3.12 and match the
  surrounding style (comment density, naming). New Python files: flake8 + pep257 clean.
- Every behaviour change on the native (Humble/systemd) path is forbidden: the native car must
  keep working exactly as today. New behaviour is selected only inside the container.
- linus may get its ROS apt packages moved to the 2026-09-11 snapshot (by the user, with sudo)
  at any time during the run. Check `ls /etc/apt/preferences.d/ros2-snapshot.pref`: if it
  appears while you work, rebuild the worktree's CMake packages clean before trusting a test
  (`colcon build --symlink-install --cmake-clean-first --packages-select <ament_cmake pkgs>`,
  CLAUDE.md "Rebuild policy") and say so in your report. Rust (ament_cargo) packages do not
  build after the move until the rclrs 0.8 port (P3): ignore them (`--packages-ignore`).

## Facts
- Orin: AGX Orin 64 GB, L4T R36.5 / JetPack 6.2.2, Ubuntu 22.04 host, kernel 5.15.185-tegra
  without bridge/iptables (host networking only; `docker build --network=host`), Docker 29.6,
  compose 5.1.4, nvidia-container-toolkit 1.16.2 (CSV mode), default runtime runc (always
  `runtime: nvidia`), docker data on the NVMe `/scratch` (540 GB free), eMMC `/` 15 GB free.
  User fabiocar uid/gid 1004; groups video 44, render **104** (image has 994: fix), dialout 20,
  input (resolve), zed 1005, jtop 1003, docker 994 (host). Device majors: ttyACM 166, ttyUSB 188,
  USB 189, video 81, input 13, hidraw dynamic (`/proc/devices`).
- Native stack on the Orin: Humble, `~/dev_ws/f1tenth_more` (branch
  stack-campaign-final-2026-10-22), host user units `f1tenth-gateway.service` (drop-in
  `two-llm.conf`, binds 127.0.0.1:8770, `tailscale serve` 443→8770) and
  `f1tenth-archive.timer`/`.service` (rsync to linus every 15 min). udev: `99-vesc.rules`
  (`/dev/sensors/vesc` → ttyACM on USB 1-4.4), `99-slabs.rules` (ZED hidraw 0666).
  Lidar Hokuyo 192.168.0.10:10940 on eno1 (192.168.0.15/24). ZED 2i S/N 36866199; host ZED SDK
  4.2.5 (the image has 5.4.0). GGUF: `~/projects/llm/models/qwen2.5-3b-instruct-q5_k_m.gguf`.
  Gateway token: `~/.config/f1tenth_gateway/env` (F1T_GATEWAY_TOKEN, mode 600).
- The car's profile differs from main-jazzy defaults: the car runs `enable_intelligence: true`
  and `enable_mission_logger: true` (main-jazzy: both false). The container reproduces the car.
- Existing recipe: `docker/orin/*` (see the plan's Context); main-jazzy-rust adds the RUST stage.
  Old D2 spec and findings: `git -C /home/andreas/dev_ws/f1tenth_more show jazzy:output/phase5d_d2_resume.md`,
  `jazzy:output/phase5d_analysis.md` (§6.2 compose sketch, §6.3 guard, §7 D3 tests),
  `jazzy:output/phase5d_image.md`, `jazzy:output/backlog.md`.

## Interface contracts between the packages (fixed; implement exactly these names)
Paths are repo-relative; in the image the repo is `/opt/f1tenth_ws`.
- `src/f1tenth_ctl/f1tenth_ctl/units.py`: facade, unchanged public functions; backend chosen by
  `F1T_PROCESS_BACKEND=systemd|proc|auto` (default `auto`: `proc` when `/.dockerenv` or
  `/run/.containerenv` exists or no user systemd manager answers, else `systemd`). New
  `procunits.py` + `runner.py` (run as `python3 -m f1tenth_ctl.runner ...`). State under
  `~/.local/state/f1tenth_ctl/` (existing dir). `F1T_WORKDIR` = cwd of managed processes
  (default `$HOME`).
- `src/f1tenth_ctl/config/ctl_container.env`: sources `ctl.env` from the same dir, then sets
  `F1T_PROCESS_BACKEND=proc`, `F1T_WORKDIR=$HOME/.local/state/f1tenth_ctl/cwd`,
  `F1TENTH_ARCHIVE_TRIGGER=$HOME/.local/state/f1tenth_archive_trigger/request`,
  `F1TENTH_IMAGE_SOURCES=/opt/f1tenth_ws/SOURCES`, `F1TENTH_CAR_SYNC=/opt/f1tenth_ws/CAR_SYNC.json`,
  the car profile launch args in the stack command, and appends the contents of
  `~/.config/f1tenth_car/stack_args` (one line, optional, logged) to the stack command.
- `docker/orin/container/f1tenth_container_main.py` (stdlib only; PID 1 is tini via compose
  `init: true`), `docker/orin/container/healthcheck.sh` (role `car`: UDP 11811 bound + TCP
  127.0.0.1:8770; any other `F1T_CONTAINER_ROLE`: exit 0), `docker/orin/container/gateway.yaml`
  (archive.workspace `/opt/f1tenth_ws`). Env read by the main: `F1T_CTL_ENV`,
  `F1T_GATEWAY_VENV` (`/opt/f1tenth_gateway/venv`), `F1T_GATEWAY_CONFIG`,
  `F1T_GATEWAY_ENV_FILE` (`~/.config/f1tenth_gateway/env`), `F1T_CONTAINER_ROLE` (`car` |
  `harness`), `F1T_AUTOSTART` (`none` | `stack`, default none).
- `docker/orin/set_installed_params.py` replaces `set_llama_paths.py`: checked edit of the
  installed `stack_params.yaml` (llama paths as before + `yolo_engine_path` →
  `/var/cache/f1tenth/trt/10.3.0.26/yolo26s-seg.trt10.3.0.fp16.engine`); refuses unless every old
  value matches.
- Mission logger: when `F1TENTH_ARCHIVE_TRIGGER` is set, `_spawn_sync` atomically writes that file
  (JSON: run_id, time) instead of calling systemctl; otherwise the systemctl call as today but the
  child is reaped and a missing user bus gives one WARN. `_git_state` without a git checkout falls
  back to `F1TENTH_IMAGE_SOURCES` / `F1TENTH_CAR_SYNC` (`source: image`).
- `scripts/orin/drift_check.sh` (+ `drift_check.py`): runs on the Orin host; args
  `--car-sync <json> [--ws ~/dev_ws/f1tenth_more]`; exit 0 clean, 3 drift (+ JSON on stdout).
- `docker/orin/compose.yaml` (service `car`, container `f1tenth_car`, project `f1tenth_car`),
  `docker/orin/compose.harness.yaml`, `scripts/docker_orin.sh`, `scripts/orin_stack_guard.sh`,
  `docker/orin/host/{f1tenth_ctl_car,stack_mode.sh,install.sh,uninstall.sh,bashrc_banner.sh}`,
  `docker/orin/host/systemd/{f1tenth-car-container.service,f1tenth-archive-trigger.path}`.
  Host state root: `/scratch/fabiocar/f1tenth_container/`. Image pin file:
  `~/.config/f1tenth/car_image` (tag + ID).
- D3 harness lives in `docker/orin/checks/` and `scripts/jazzy_parity/`.

## Work packages
- **W1 (main-jazzy worktree `/home/andreas/dev_ws/f1tenth_more_main_jazzy`)**: plan P4.1–P4.6 +
  P1.5 (drift check) + P1.6 (`tools/car_sync/params_delta.py` → `output/orin_container/params_delta.md`,
  comparing `git show 6c329992:src/f1tenth_params/config/stack_params.yaml` + the car's margin 0.7
  against HEAD; the car profile args come from it). Gate: ctl tests on both backends
  (`src/f1tenth_ctl/test`, new `test_procunits.py`), logger tests, new-file lint, JSON contract
  unchanged, a container-main mock run on linus (gateway `--mock`, dummy `F1T_*_CMD`), drift
  check fixture tests; colcon test of f1tenth_ctl, f1tenth_logger, f1tenth_gateway with no new
  failing IDs vs `output/jazzy_main/test_failing_ids.txt`.
- **W2 (main-jazzy-rust worktree `/home/andreas/dev_ws/f1tenth_more_rust`)**: plan P5 + P6.1–6.5 +
  P6.7 (telegram `ctl_command` default → `~/.local/bin/f1tenth_ctl_car`, only in the car config
  `src/f1tenth_telegram/config/telegram.yaml`). Uses W1's paths by contract (they arrive by merge).
  Gate: `docker build --check -f docker/orin/Dockerfile docker/orin` (lint only), shellcheck of
  every new/changed script, `docker compose -f docker/orin/compose.yaml [-f compose.harness.yaml]
  config` with fake variables, wrapper tests with a fake `docker` on PATH, `stack_mode.sh
  --dry-run`, python tests for any helper, `build_checks.sh` read through for every new check.
- **W3 (new worktree `/home/andreas/dev_ws/f1tenth_more_d3h`, branch d3-harness from main-jazzy)**:
  plan P6.6, port of the D3 harness from branch `jazzy` adapted to main-jazzy's node/topic names
  (`scripts/jazzy_parity/*`, `scripts/fix_batch_5/lease_probe.py` → `scripts/jazzy_parity/`,
  `docker/orin/spike/gates/{zed_grab.cpp,g2_trt.py,yolo_run.py,yolo_compare.py,dds_probe.py}` →
  `docker/orin/checks/`), each script with a header saying where it runs (Orin host, harness
  container, linus) and what it compares; a `docker/orin/checks/README.md` mapping the plan's P8/P9
  checks to scripts. Gate: syntax/lint, `--help` of each, unit tests for pure helpers, a dry
  smoke on linus against bag data where possible (no Isaac needed).
