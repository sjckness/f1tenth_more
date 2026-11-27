# f1tenth_bringup — local memory

Contracts/invariants only; narrative: `f1tenth_more/docs/f1tenth_bringup.md`.

## Identity
`ament_python`; exactly 4 executables (`setup.py:44`): `component_supervisor_node`,
`reset_manager` (= `reset_manager_node.py`), `stack_startup_sequence`, `viz_jpeg_node`.
`topic_watchdog.py` is a library. `setup.py` globs `config/*.yaml` + `config/*.xml`, so
new config files install themselves.

## Topology
- Exclusive entry points: `stack_bringup.launch.py` (one process tree, no recovery) or
  `supervisor_bringup.launch.py` (runs `ensure_discovery_server.py` **first**, then
  `component_supervisor_node`). Maintained separately; diverge on `intelligence`
  (supervisor-only).
- Supervisor runs each component as `ros2 launch <pkg> <file>` in its own process group;
  lock `/tmp/component_supervisor.lock`, pgid files under `log_dir`.
  `_HARDWARE_TEARDOWN_BUFFER_SEC = 2.0` mirrors `f1tenth_hardware/vesc.launch.py`
  (serial-port release before `calibrate_hardware`).

## Auto-start is code, not config
- `components.yaml` = *what* runs; *whether* it auto-starts comes from three hardcoded
  sets (`component_supervisor_node.py:410-430`): `_ALWAYS_AUTO_START` (13),
  `_NEVER_AUTO_START` (`calibrate_hardware`, `startup_sequence`),
  `_CONDITIONAL_AUTO_START` (`behavior`←`use_behavior_tree`,
  `intelligence`←`enable_intelligence`).
- ⚠️ **A component in `components.yaml` but in none of those sets never starts,
  silently, with no warning anywhere.** Always pair a `components.yaml` edit with a set
  edit in the same commit; tests assert membership.
- `dev_tools`/`navigation`/`lidar_front_wall` self-gate on
  `enable_foxglove`/`enable_nav2`/`use_lidar` — the supervisor does not gate them.

## Fast DDS
- `fastdds_profile.xml` (5 s lease) + `ROS_DISCOVERY_SERVER` /
  `FASTRTPS_DEFAULT_PROFILES_FILE` (`scripts/env/jazzy.sh`); every participant loads it
  itself, a server cannot impose it.
- ⚠️ **`/dev/shm/*fastrtps*` is purged at supervisor startup** (`_FASTRTPS_SHM_DIR`,
  `_FASTRTPS_SHM_GLOB_PATTERN`) — stale SHM segments break discovery; never delete that
  glob.

## Contracts
- Supervisor serves `/restart_component`, `~/control_component`, `~/run_calibration`;
  pubs `/supervisor/health` (**RELIABLE+TRANSIENT_LOCAL**, latched) + `/diagnostics`;
  subs `/calibration/in_progress` (Bool, TRANSIENT_LOCAL) when `defer_localization`;
  `/clock` **sim only**.
- `reset_manager` serves `/reset_all`, holds `/safety_stop` @20 Hz (mux timeout 0.2 s),
  restarts `hardware`/`slam` (no reset path), protects `intelligence`.
- `stack_startup_sequence` → `/teleop` @20 Hz (mux priority 100). `viz_jpeg_node`:
  parallel `input_topics`→`output_topics`, `/viz_internal/*`→`/viz/*`. Foxglove:
  `8765`@`0.0.0.0`, capabilities subset, `taskset`-pinned, 2 throttled image copies.

## Do not "clean up"
`service_whitelist: [[…]]` nested on purpose; `throttle` uses positional `arguments=`;
`camera_source` in `stack_bringup:137` is dead; the 4 stack-wide values are not
CLI-overridable; `vesc.yaml` `velocity_correction_*` must be **absent**, never `[]`; no
`--symlink-install` (rebuild + restart); health config is strict and
`fail_for_sec > clock_pause_sec`. This package also owns config consumed by
hardware/localization/control/perception/sim — never rename config files (external tests
read them by path).
