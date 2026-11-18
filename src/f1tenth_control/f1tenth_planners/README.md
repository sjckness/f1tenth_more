# f1tenth_planners — the planner-controller modes

One parameter, `planner_controller_mode` (`f1tenth_params/config/stack_params.yaml`),
picks what turns mission moves into `/drive` when `enable_nav2` is false:

| mode | node | planner | controller |
|---|---|---|---|
| `corridor_mpc` (default) | `mpc_corr` | corridor (corridor_geometry) as soft reference | OSQP MPC |
| `corridor_rrt` | `corridor_rrt` | same corridor geometry, RRT* inside it around obstacles + LiDAR wall tracks | pure pursuit |
| `p2p_geometric` | `p2p_geometric` | no corridor: turn-then-straight / Dubins from the car to the target; obstacles slow and stop the car | pure pursuit |

All three take the same goal topics (`/mpc/goal_distance`, `/mpc/goal_pose`,
`/mpc/goal_turn`, `/mpc/goal_drive`, `/mpc/goal_object`, `/mpc/goal_object_end`,
`/mpc/hold`, `/mpc/stop_target`) and publish the same feedback (`/mpc/goal_reached`,
`/mpc/object_status`, `/mpc/min_obstacle_distance[_forward]`, `/drive`), so mission files
run unchanged. The two new nodes also publish `/planner/path`, `/planner/markers`
(path, corridor, RRT tree) and `/planner/status` (JSON per replan) for viewing.

## Selecting a mode for a test session

```bash
# whole stack (forwarded to the 'navigation' and 'behavior' components)
ros2 launch f1tenth_bringup supervisor_bringup.launch.py planner_controller_mode:=corridor_rrt
ros2 launch f1tenth_bringup stack_bringup.launch.py planner_controller_mode:=p2p_geometric
# navigation component alone
ros2 launch f1tenth_navigation navigation.launch.py planner_controller_mode:=corridor_rrt
```

or change `planner_controller_mode` in `stack_params.yaml`. The behaviour tree's mission
preflight requires the node of the selected mode, so a mission does not start against a
controller that is not up. A typo stops the bringup (`ValueError`), it does not start
without a controller.

Not a runtime switch: changing mode means restarting `navigation` and `behavior`
(`./scripts/stackctl.py restart ...` or a new bringup).

## What differs from mpc_corr (on purpose)

- **Turn moves** aim at the end of the requested steering's circle plus 0.5 m, on a heading
  5° past the requested one. mpc_corr's synthetic point (one chord along the new heading's
  ray) sits beside the car, which a forward-only planner can only reach with a loop; the
  overshoot makes the mission's `orientation_delta` (accumulated ≥ value) fire instead of
  grazing it.
- **No reverse**, no wall_turn distance model, no speed channel / LLM2 tuning. `wall_turn`
  commits the turn when the front wall is within R_min + `corr_wall_turn_safety_margin_m`.
- **goal_pose** coordinates are taken as odom, exactly as mpc_corr does.

## Tests

- `test/test_planners_pure.py` — geometry, Dubins, RRT*, pure pursuit, moves.
- `test/test_closed_loop_nodes.py` — both node classes in closed loop with a kinematic car
  (no DDS): distance, pose, ±90° turns, drive straight / wall_turn, hold, stale odometry,
  object approach, obstacle (RRT plans around it, p2p stops before it).
- `f1tenth_navigation/test/test_planner_controller_mode.py`, `f1tenth_params/test/test_planner_modes.py`,
  `f1tenth_behavior/test/test_preflight.py`, `f1tenth_bringup/test/test_sim_mode.py` — the selection plumbing.
