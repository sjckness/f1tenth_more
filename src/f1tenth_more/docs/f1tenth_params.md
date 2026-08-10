# f1tenth_params

Single shared source of launch-parameter defaults for the whole workspace.
Every other package's launch files read their defaults from here instead of
hardcoding them, so there is exactly one place to look up (or change) what any
given launch argument actually defaults to.

## Why this package exists

`f1tenth_bringup/stack_bringup.launch.py` `include()`s almost every other
package's launch file, and nearly every launch file in the workspace imports
from `f1tenth_params`. If the shared defaults lived inside `f1tenth_bringup`
itself (as they originally did), every consumer package would need to depend
on `f1tenth_bringup` right back — a build-order cycle colcon refuses to
resolve. Keeping `f1tenth_params` as its own dependency-free leaf package
(`exec_depend` only on `ament_index_python` and `python3-yaml`) avoids that
entirely.

## Contents

- **`config/stack_params.yaml`** — one YAML entry per launch parameter across
  the whole stack (87 top-level keys as of this writing), each with a
  `default`, a `description` string, and a `# Consumed by:` comment above it
  naming every package/launch-file that actually reads it (audited against
  live `get_value()`/`get_default()`/`get_path_default()` call sites, not
  just "related" packages — treat these comments as the authoritative
  per-key consumer list; this doc doesn't re-derive them). Organized into
  named sections: Stack-Wide, Hardware, Localization/TF, Perception,
  Command/Control, Autonomy, Diagnostics & Intelligence, Simulation,
  Component Supervisor, Dev Tools/Visualization.
- **`f1tenth_params/param_defaults.py`** — the only code in this package:

  | Function | Use |
  |---|---|
  | `get_default(name)` → `(value, description)` | Normal case: pass straight through to a `DeclareLaunchArgument`'s `default_value`/`description`. CLI overrides (`name:=...`) still work normally afterward. |
  | `get_value(name)` | For the 5 stack-wide branching args (below) — this call *is* the value; there's no `DeclareLaunchArgument` backing it, so `name:=...` on the CLI is silently ignored for these. |
  | `get_path_default(name, package='f1tenth_bringup')` → `(absolute_path, description)` | For path-type entries (`default: config/vesc.yaml` etc.) — resolves against `package`'s install share dir. `map` is the one entry that overrides `package` (resolves against `f1tenth_navigation` instead). |
  | `get_odom_topic()` | Returns `/odometry/filtered` if `localization_source == 'ekf'` else `/odom` — single source of truth so every odometry consumer (MPC_corr, the BT's `CheckStopCondition`, Nav2's `bt_navigator`) stays in lockstep with whichever source `f1tenth_localization/localization.launch.py` actually brought up, instead of each hardcoding `/odom` independently. |

  `_load()` reads and `yaml.safe_load()`s `stack_params.yaml` once per
  process (`functools.lru_cache(maxsize=1)`) from the package's installed
  share directory.

## The 5 stack-wide branching args

`camera_source`, `localization_source`, `enable_llm`, `use_behavior_tree`,
`enable_nav2` are read via `get_value()` as plain Python values at launch
*parse* time, not `DeclareLaunchArgument`/`LaunchConfiguration`. This means
the only way to change one is editing `stack_params.yaml` directly — passing
`camera_source:=webcam` on a `ros2 launch` command line is silently ignored,
since no launch argument by that name exists anywhere to receive it. Every
other key in the file is a normal, CLI-overridable `DeclareLaunchArgument`
whose default/description just happens to be sourced from here.

(A sixth branching arg, `enable_safety_stop`, existed through Phase 1 of the
code-analysis/fixes arc and was deleted along with `safety_stop_controller`
— see `f1tenth_control`'s doc.)

## Nodes / launch files

None — this package has no nodes and no launch files of its own; it is a
pure config + Python-helper library imported by everyone else.

## Known limitations

- No schema validation on `stack_params.yaml` beyond "is it valid YAML" — a
  typo'd key name in a consumer's `get_default('typo_name')` call fails at
  launch time with a plain `KeyError`, not a more specific error pointing at
  the mismatch.
- The `# Consumed by:` comments are maintained by hand, audited against
  source at specific points in time (most recently alongside the Phase 1/2
  code-analysis-and-fixes pass) rather than mechanically kept in sync — they
  can drift if a consumer is added/removed without updating the comment.
