# f1tenth_navigation

Static map server + the full Nav2 stack (planner/controller/behavior
servers, `bt_navigator`, one shared lifecycle manager), split into one
launch file per node plus an orchestrator. No node source code of its own —
every node here is a standard Nav2 (or `nav2_map_server`) executable; this
package's own code is entirely launch-file composition.

**`enable_nav2` currently defaults to `false`** — by default, Nav2 is *not*
running and `mpc_corr` (`f1tenth_control`) drives directly instead. See
`navigation.launch.py` below.

## Launch files

| File | Purpose |
|---|---|
| `navigation.launch.py` | The actual entry point everything else includes (`stack_bringup.launch.py`, `vesc.launch.py`, `components.yaml`'s `navigation` component). Resolves `enable_nav2` (plain `stack_params.yaml` read): **true** → includes `nav2.launch.py`. **false** → includes `f1tenth_control/mpc_corr.launch.py` **and** `map_only.launch.py` (so `/map` still publishes for RViz/Foxglove even with the rest of Nav2 down). |
| `nav2.launch.py` | Orchestrator — replaces an older monolithic bringup file. Conditionally includes each of the 5 component files below based on its own `enable_nav2_<component>` value (all default `true`), builds one shared `lifecycle_manager_navigation` whose `node_names` list is built from exactly the components actually included (so e.g. `enable_nav2_map:=true` alone with everything else `false` still correctly lifecycle-manages just `map_server`). Also includes `f1tenth_perception/lidar.launch.py` — **this is the only path that ever launches the LiDAR** (see `f1tenth_perception`'s doc). Does *not* publish any static TF itself (moved to `f1tenth_localization/localization.launch.py`, see that doc's "Known limitations" note about the bug this fixed). |
| `map.launch.py` | `nav2_map_server`'s `map_server` alone (`/map`, latched/transient-local `nav_msgs/OccupancyGrid`). A Nav2 lifecycle node — stays `unconfigured` with no external `lifecycle_manager`, so launching this file standalone (bypassing `nav2.launch.py`/`map_only.launch.py`) leaves it inert. |
| `map_only.launch.py` | Pairs `map.launch.py` with its **own** single-node lifecycle manager, deliberately named `lifecycle_manager_map` (not `lifecycle_manager_navigation`) — so anything that specifically waits on the full 5-node Nav2 manager's readiness (`f1tenth_behavior`'s `wait_for_trigger_service_node`) can't mistake this smaller one-node manager for it. Used by `navigation.launch.py`'s `enable_nav2:=false` branch. |
| `controller.launch.py` | `nav2_controller`'s `controller_server` (`nav2_regulated_pure_pursuit_controller` per `package.xml`). |
| `planner.launch.py` | `nav2_planner`'s `planner_server` (`nav2_smac_planner` per `package.xml`). |
| `behavior_server.launch.py` | `nav2_behaviors`' `behavior_server`. |
| `bt_navigator.launch.py` | `nav2_bt_navigator`'s `bt_navigator` — serves the `navigate_through_poses` action `f1tenth_behavior`'s `NavigateThroughPosesClient` calls when `enable_nav2=true`. |

## Config

`config/nav2_params.yaml` — one shared params file for all 5 servers plus
`local_costmap`/`global_costmap`, loaded by each component launch file's own
`nav2_params` argument.

## Consumed `stack_params.yaml` keys

`map`, `autostart`, `nav2_params_config`, `enable_nav2_map`,
`enable_nav2_controller`, `enable_nav2_planner`,
`enable_nav2_behavior_server`, `enable_nav2_bt_navigator` — see each key's
own `# Consumed by:` comment in `stack_params.yaml`. (`enable_nav2` itself is
consumed by `f1tenth_navigation/navigation.launch.py` and
`f1tenth_bringup`/`component_supervisor_node.py`, not this package's other
files directly.)

## Known limitations

- **Splitting each Nav2 server into its own launch file trades away
  standalone lifecycle management.** Running e.g. `controller.launch.py` by
  itself (bypassing `nav2.launch.py`) leaves that node `unconfigured`
  forever — there's no per-file lifecycle manager, only the shared one in
  `nav2.launch.py`. This is an accepted tradeoff (the point of the split is
  selectability *through the orchestrator*, not N independent entry points),
  not an oversight, but worth knowing before trying to launch one component
  in isolation for debugging.
- With `enable_nav2` at its current default (`false`), most of this
  package's code (everything except `navigation.launch.py`'s `false`
  branch and `map_only.launch.py`) isn't exercised in the deployed
  configuration — worth keeping in mind when reasoning about what's actually
  running vs. what's built and available.
