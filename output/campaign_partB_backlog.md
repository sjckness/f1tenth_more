# Part B backlog (precision; adaptive corridor default OFF)

As of `test-campaign` after Part A and the review items, 2026-10-12. B1–B4 are as specified. B5–B8 were added from the Orin's R011 analysis and from Part A findings.

## Original items

| Item | Content |
|---|---|
| B1 | `front_distance` robustness: time-sync masks with depth (detection lag measured at 0.44 s p50 in R011); expand person masks to the ground; mask the column band at the image border; wall-model fit instead of the near-half percentile; publish "unknown", never a short distance |
| B2 | Fused object geometry (LiDAR–detection association, per-class clamps, centre = front surface + half depth, track-level size). **1.0 m objects only pass with true-centre reporting today: the front-surface disc inflates them toward the car.** Also: remove the double `car_radius` in the soft `w_obs` cost (front points already past the bumper) |
| B3 | Tracker (Mahalanobis + Hungarian, KF, lifecycle, track merging) replacing the 0.5 m nearest + EMA merge |
| B4 | Adaptive corridor (`corridor_adaptive_enabled`, default false). The pass side must come from `mpc_controller/pass_side.py` (`decide_sides`, `PassSideMemory`, `/mpc/pass_sides`): one source of truth with the keep-out rows |

## Added from the Orin's R011 analysis

**B5. The plant was counted twice: camera track + LiDAR map cell.**
- **Part A: not addressed.** The camera obstacle (`/perception/obstacles_2d`) drives the soft `w_obs` cost and the A3/A4 keep-out rows. The same plant is also an occupied cell in `/slam/map`, which `costmap_boundary_node` turns into a boundary row (`/costmap/boundaries`, soft, slack weight 1e4).
- **To do:** associate map cells with tracked objects (B3) and exclude those cells from the boundary extraction, or exclude boundary faces that coincide with a tracked object's footprint, so one obstacle is constrained once.
- **Test:** scenario harness with the same obstacle injected on both channels.

**B6. The front map limit plane pinned the plan.**
- **Part A: not addressed at the source.** The nearest occupied cell ahead becomes a front halfspace ("x ≤ offset − car_radius − margin") at every stage. A plant in the corridor therefore became a wall across the whole corridor, and the cheapest plan was to stop before it.
- **What Part A changes:** the outcome. With the 0.4 m/s floor in the QP and the BLOCKED detector, a pinned plan is now an infeasible tick, and the move ends BLOCKED within about 1 s instead of crawling for 33 s. This is inferred: the harness has no map, so the boundary rows were not exercised; the soft boundary rows absorb the slack and the plan-body check against the camera obstacle fails.
- **To do:** take obstacle-like occupied cells out of the front plane (B5), or limit the front plane to wall-like segments (the LiDAR wall-line test from A4 can be reused), so the car can pass beside the plant.
- **Test:** harness scenario with a map front plane at the obstacle.

## Added from Part A findings

**B7. Corridor limits apply to the rear axle only.** The keep-out rows already use 3 footprint circles. In the harness the outline never left the corridor by more than 0.05 m, but geometrically a car with its axle on the wall and 20° of heading puts its front corner 0.28 m outside. Consider applying the corridor rows to the front circle once B4 makes the corridor narrow again.

**B8. Unknown front distance on a wall_turn.** DONE (2026-10-12): no distance = no wall decision, also on the first rebuild; the car goes straight and the move ends BLOCKED `no_wall_distance` after `wall_turn_distance_timeout_s` (2.0 s). The turn is never committed without a valid distance.
