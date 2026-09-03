# f1tenth_more

Top-level metapackage for the F1TENTH (roboracer) autonomous racing stack —
aggregates every first-party package (no code of its own). This README is
the index into `docs/`, one file per package, written after the full
workspace reorg + optimization arc + code-analysis-and-fixes pass had all
landed, describing the actually-current state of each package (read fresh
from source, not carried over from any earlier phase's notes).

For the workspace-wide overview (architecture, the 4 stack-wide branching
args, full launch-parameter table, docker/deployment) see the
[top-level workspace README](../../README.md) — this index is one level
more detailed, package by package.

## Packages

| Package | One-liner | Docs |
|---|---|---|
| `f1tenth_params` | Single shared source of launch-parameter defaults (`stack_params.yaml` + `param_defaults.py`) every other package's launch files read from. | [docs/f1tenth_params.md](docs/f1tenth_params.md) |
| `f1tenth_messages` | Custom `.msg`/`.srv` interfaces shared across the stack (the only `ament_cmake` package). | [docs/f1tenth_messages.md](docs/f1tenth_messages.md) |
| `f1tenth_hardware` | VESC drive chain: battery pre-flight gate, optional live sensor covariance calibration, ackermann↔VESC conversion, odometry, the VESC driver itself. | [docs/f1tenth_hardware.md](docs/f1tenth_hardware.md) |
| `f1tenth_control` (+ `mpc_controller`) | Drive-command arbitration (`ackermann_mux`), the MPC controller (`mpc_corr`, the deployed node), manual joystick control. `safety_stop_controller` (retired) used to live here too. | [docs/f1tenth_control.md](docs/f1tenth_control.md) |
| `f1tenth_perception` | Camera bringup (ZED2 or webcam) + LiDAR bringup + YOLO 2D→3D detection fusion → ground-plane obstacles. | [docs/f1tenth_perception.md](docs/f1tenth_perception.md) |
| `f1tenth_localization` | `map → odom` via EKF or a raw unfiltered fallback (`localization_source`), plus the static `odom → base_link` TF the EKF composes against. | [docs/f1tenth_localization.md](docs/f1tenth_localization.md) |
| `f1tenth_navigation` | Static map server + the full Nav2 stack, one launch file per server plus an orchestrator. `enable_nav2` currently defaults `false` — `mpc_corr` drives by default instead. | [docs/f1tenth_navigation.md](docs/f1tenth_navigation.md) |
| `f1tenth_behavior` | py_trees behavior tree: emergency-stop, obstacle-corridor stop, a scripted mission subtree, and Nav2/direct-MPC goal navigation, one priority-ordered root `Selector`. | [docs/f1tenth_behavior.md](docs/f1tenth_behavior.md) (mission JSON schema: [src/f1tenth_behavior/README.md](../f1tenth_behavior/README.md)) |
| `f1tenth_bringup` | Both top-level entry points (single-process and per-component-restartable), Foxglove bridge, boot-time self-check, most shared hardware config. | [docs/f1tenth_bringup.md](docs/f1tenth_bringup.md) |
| `f1tenth_diagnostics` | Calibration/diagnostic tooling: gyro bias, sensor covariance, battery pre-flight gate, continuous battery monitoring, system (CPU/GPU) observability. | [docs/f1tenth_diagnostics.md](docs/f1tenth_diagnostics.md) (operational how-to: [src/f1tenth_diagnostics/README.md](../f1tenth_diagnostics/README.md)) |
| `llm` (`f1tenth_intelligence/llm`) | `llama-server` bringup + an LLM-driven natural-language mission planner. Auto-starts by default (`enable_intelligence`, the component supervisor's `intelligence` component; `:=false` to opt out). | [docs/llm.md](docs/llm.md) |
| `f1tenth_external` (+ `vesc` under `f1tenth_hardware/`) | 6 vendored/forked git submodules (ZED SDK wrapper, VESC driver, ackermann_mux, teleop_tools, transport_drivers, zed-ros2-interfaces) — brief, not owned by this project. | [docs/f1tenth_external.md](docs/f1tenth_external.md) |

**Not covered by this documentation pass** (outside the package list this
pass was scoped to): `f1tenth_description` (URDF/xacro robot model, shared
by real hardware and sim), `f1tenth_sim` (Gazebo simulation bringup). Both
exist and build; they just don't have a `docs/` page here yet.

## Where these docs live, and why

Centralized here in `f1tenth_more/docs/` (one file per package) rather than
duplicated inside each package's own directory, per this phase's explicit
brief — with two exceptions: `f1tenth_behavior` and `f1tenth_diagnostics`
already had their own in-package `README.md`, and in both cases it's a
*deep, actively-maintained reference* tied tightly to code in that same
package (`f1tenth_behavior/README.md` documents the mission JSON schema
enforced by `mission/mission_config.py`; `f1tenth_diagnostics/README.md` is
a node-by-node "how to run this calibration tool, what output to expect"
operational guide). Duplicating that content here would just create a
second copy to keep in sync, so those two docs stay in place and this
index's own `docs/f1tenth_behavior.md`/`docs/f1tenth_diagnostics.md` link
out to them instead of re-deriving the same material — each centralized
page covers everything else about its package (nodes, topics, launch files,
params) that the in-package doc doesn't.

## Verification approach

Every topic/service/param claim in these docs was checked against the
actual node source (`declare_parameter`/`create_subscription`/
`create_publisher`/`create_service` call sites) or launch-file source
(`DeclareLaunchArgument`/`Node(parameters=...)`), not written from memory of
what a node "should" do — this arc has repeatedly found real drift between
assumed and actual behavior (stale defaults in comments, retired packages
still mentioned in passing, launch files that were never actually wired in),
and a couple more instances turned up again while writing these docs (noted
in each affected page's own "Known limitations" section, e.g. a stale
`covariance_sample_duration_sec` name fixed in `f1tenth_diagnostics/README.md`,
and several files still saying "6 stack-wide branching args" where the
actual count is 5).
