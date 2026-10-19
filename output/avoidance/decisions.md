# Avoidance pass — decisions

Branch `campaign-final`, Orin, Humble. Backup tag
`backup/pre-avoidance-69830ecf`. No push, no stack launched, no driving.

## Standing design decision (the user's, final)

The MPC avoids **only what perception recognises**: wall tracks become boundary
rows, recognised objects from `/camera/detections_3d` become obstacle
constraints. Unrecognised LiDAR obstacles are **not** avoided by the MPC — they
are handled by an independent last-resort stop (A2), outside the behaviour tree.

## D1 — `boundary_source` default is `walls` (A1)

The costmap nearest-occupied-cell rows are no longer the default: a single
occupied cell is enough to become a half-plane across the whole horizon, and the
extraction cannot tell a map artefact from a wall. `boundary_source:=costmap`
restores the previous behaviour exactly.

## D2 — launch arguments are forwarded (A1)

`navigation.launch.py` previously included `mpc_corr.launch.py` with no
`launch_arguments`, so every mpc_corr argument passed to `stack_bringup` was
silently dropped. The wall-row arguments are now forwarded explicitly through
`supervisor_bringup` → `navigation.launch.py` → `mpc_corr.launch.py`. Only the
keys this pass needs are forwarded, not a blanket re-export: the file's
"self-resolves from stack_params, needs no args passed in" design is kept for
everything else.

## D3 — `back` wall tracks never produce rows (A1)

The car never reverses on any mission in this campaign, so a half-plane from a
wall behind it can only ever constrain a manoeuvre that does not happen. This
settles the contradiction flagged in `wall_boundaries.md`: the work order's own
rule (roles front/left/right) wins over its illustrative "4-wall room → 4 rows".
A 6x6 room therefore gives 3 rows plus one `role_back` rejection, reported with
its reason.

## D4 — the last-resort stop is its own node on the safety lane (A2)

Not a behaviour-tree leaf, and not folded into an existing perception node:

- The BT is the thing it has to survive. A leaf cannot stop the car when the BT
  process is dead, which is exactly when a last resort is needed.
- It feeds the ackermann_mux **safety lane (priority 200)**, above navigation,
  so it wins regardless of what the MPC or the BT is asking for.
- It subscribes `/scan` at full rate with SensorData QoS and does nothing else,
  so its latency is one scan period (25 ms at the measured 40 Hz).

Trigger: any valid return whose range from the LiDAR origin is <= 0.20 m
(`base_link` x = 0.505 at 0 deg, i.e. 0.10 m ahead of the 0.405 m bumper),
excluding returns inside the car's own footprint polygon + 2 cm. Debounce 2
consecutive scans. Latched until the zone has been clear for 0.5 s. The existing
0.40 m BT proximity e-stop is untouched.

## D5 — A3 cannot be answered from the available bags, and is not invented

Measured, not assumed (see `log.md` for the numbers):

- `corridor_obstacles_20261019T135307`, the named obstacle run, is a
  **standstill**: path length 0.00 m, every `/drive` speed exactly 0.000,
  mission `IDLE`, `/mpc/solver_status` empty. Nothing was ever in the car's path
  — the forward strip is clear to 19.6 m and the nearest return in the whole
  45.7 s is 1.171 m, at -76 deg.
- `corridor10m_20261019T131815` is the only moving run from today (19.41 m, 312
  solves). Its **minimum body clearance over the whole run is 0.342 m**, against
  an intended standoff of `car_radius + obstacle_safety_margin_m` = 0.32 m. The
  car never came closer than the margin. The 518 scans under 0.40 m are all a
  wall 0.49 m off the car's **left side** between t=74 s and t=90 s, not an
  obstacle passage.

So there is no close passage to explain. What the bags DO show is why there
could not have been a recognised-object avoidance at all, and that is reported
in its place, with the two real defects found and a proposed fix for each.

## D6 — `boundary_slack_max` becomes a ROS parameter (A1)

Default 0.5, unchanged. It was a `solve_mpc_step` keyword that `MPC_corr` never
passed, so the cap was untunable; a wall the car is further than 0.5 m inside
still goes infeasible, and that is now a number that can be moved without a
code edit.
