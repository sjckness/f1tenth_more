# Obstacles v2: walls → rows, every obstacle → the semantic layer

Branch `obstacles-v2` on linus (`~/dev_ws/f1tenth_more_obst`), cut from the Orin's
`campaign-final` `29dd3aa`. Nothing pushed. Nothing launched against a stack or
sensors. All timings are **linus** numbers unless marked Orin.

## SUMMARY

1. **A was skipped, as the rules say.** A1 (`d3671b1`, walls as the default
   boundary source) and A2 (`16c91de`, `hard_stop_node`) are committed on the Orin, and
   the `.git` cleanup was done there (it needed no commit). Their A3 and their island
   census were not done, and nor was `output/avoidance/report.md`. I did not redo them.
2. **A3, the cause of the close passes: neither bag has a close pass to explain.**
   - The obstacle bag is a standstill: path 0.00 m, 0 MPC solves.
   - The corridor bag's minimum body clearance is 0.342 m (Orin log).
   - The synthetic closed loop shows the likely mechanism. The soft `w_obs` term
     **does not hold clearance for an obstacle within about 0.3 m of the straight
     path** (table in §3). v2 does not change that term. Evidence only; no fix.
3. **Corridor bag (no obstacles were placed): 0 false speed caps and 0 live plans
   affected, but not "~0 unknown objects".**
   - It produces 13 confirmed unknown objects per frame (99 over the run).
   - They are **real side clutter at the LiDAR plane**, not wall leaks: only 11 of
     the 99 lie within 0.3 m of a wall line, 74 live longer than 2 s, and none is in
     the strip ahead when it is confirmed.
   - None comes within 0.48 m of any of the 312 live predicted trajectories, so the
     `w_obs` term would never have been active.
4. **Obstacle bag: unknown objects found, but no upgrades or passes to compare.**
   - It produces 15 unknown objects, each stable for the whole 39 s, with 2 folds.
   - There were **0 upgrades**: YOLO and depth died 0.66 s into the bag. That is the
     ZED fault the Orin's A3 notes found in both of today's bags.
   - **There is no passage to predict** — the car never moved — so "predicted vs
     live clearance" has nothing to measure.
5. **Duplicates and cell rows.**
   - **Duplicates:** 0 duplicate tracks survive any cycle, because the dedupe runs
     every cycle. The folds it performed were 106 on the corridor bag and 2 on the
     obstacle bag; they are mostly neighbouring clutter clusters merged, not one object
     tracked twice.
   - **Cell-derived MPC rows: 0** in v2, against 3 per solve live (that run predates A1).
6. **Timing.**
   - **MPC solve:** p95 with 8 obstacles is 9.37 ms, against today's p95 with 1 of
     9.44 ms (**0.99×**, budget 1.3×). This needed vectorising the obstacle
     linearisation, which is bit-identical.
   - **Added LiDAR step: about 2 ms p50 on linus (1.9 / 2.4), within the 3 ms
     budget. Estimated about 6–8 ms p50 on the Orin, over budget** (×3.2 from
     `extract_lines`' Python timings on both machines).
7. **Recommendation for the first live test: shadow mode.** Reasons:
   - the upgrade path has never run on real data, because the camera was dead in both bags;
   - the Orin cost is estimated, not measured;
   - a cluttered corridor will hand the MPC up to its cap of 8 side disks per cycle.

## 0. Recon: where each piece plugs in

| Piece | Where, before v2 |
|---|---|
| Object tracker | `f1tenth_costmap/semantic_layer.py` + `semantic_layer_node.py`. Map frame (global-EKF pose). Hungarian association, class-gated, gate `merge_distance_m` 0.5 m (+2σ). EMA α 0.3 on position, velocity and width. Lifecycle by detection **batches**: confirm 3 consecutive, grace 2, lost after 5 misses. `class_id` string. Out: `/costmap/semantic_tracks` (`Detection3DArray`) and markers. |
| MPC obstacles | **Not** from the semantic layer. `/perception/obstacles_2d` (`Obstacle2DArray` x/y/r, base_link) from `obstacle_projector_node`: r = footprint width/2 + `obstacle_class_margin_m`. `MPC_corr` overwrote its list on arrival and never aged it, with no cap. Soft only: `w_obs` (8.0) on softplus((car_radius 0.20 + avoidance_margin 0.12) − d_front), d_front = min over 3 nose points × all obstacles. |
| Costmap derived layers | `costmap_boundary_node`: nearest occupied `/slam/map` cell per direction → 3 rows (`boundary_source:=costmap`). The wall layer is rasterised from `/perception/walls`. |
| Rows | `boundary_source` = `walls` (A1): rows from confirmed front/left/right wall tracks. |
| Mux safety lane | `hard_stop_node` (A2), priority 200, unchanged. |
| Plug-in points | Unknown objects: **in `semantic_layer_node`** (new 10 Hz cycle, `unknown_objects.py`, same track list). MPC: **`mpc_obstacles_topic`** switched to the semantic layer's `/costmap/semantic_obstacles` (same message type), aged and capped (`obstacle_select.py`). Speed cap: `/costmap/unknown_speed_cap` applied to `vdes` in `MPC_corr`. |

## 1. What was built

- **B1 / B2 (`unknown_objects.py`).**
  - **Cell view:** the latest `/scan` on a local 0.05 m map-frame grid. Self-hits and
    anything beyond 4 m are dropped. `/slam/map` is never read (D1).
  - **Ownership:** each occupied cell is WALL, OBJECT or ISLAND.
    - WALL: within 0.10 m of a confirmed `/perception/walls` segment, **or of this
      cycle's LiDAR wall observation ≥ 1.0 m** (D5).
    - OBJECT: inside a fresh confirmed recognised track's footprint + 0.10 m (D2, D7).
    - ISLAND: everything else.
  - **Candidates:** islands clustered by single linkage at 0.20 m (D12). Each cluster
    gives a centroid, oriented extent and cell count, with ≥ 2 cells and within 4 m.
    Clusters seen through a gap in a wall are dropped (D6).
- **B3 (`semantic_layer.py`).** Unknown objects go into the **same** track list:
  - Class `unknown`, size = the measured extent, the same EMA.
  - **Confirmation:** consecutive sightings spanning 0.5 s.
  - **Decay:** 1.0 s after the last hit once the rays pass through its area; 3.0 s if
    the area cannot be seen; kept alive while its cells are still hit (D8).
  - **Upgrade:** a detection may match an unknown track. It keeps the **same id** and
    takes the detection's class.
  - **Absorption:** a LiDAR cluster matching a recognised track is absorbed. A second
    cluster of a track already held never spawns a new track.
  - **Lifecycles stay with their sensor** (D4).
  - **Dedupe:** runs every cycle.
- **B4.** The MPC takes obstacles only from `/costmap/semantic_obstacles`:
  - Every confirmed recognised track that is fresh (D7), plus unknown tracks in
    `active` mode.
  - r = disk radius + class margin, the projector's rule. An elongated unknown is a
    chain of half-width disks (D11).
  - The list is dropped after 1.0 s and capped at the 8 nearest. The w_obs weight and
    margins are unchanged.
  - The obstacle linearisation is vectorised (D9).
  - Second layer: a confirmed unknown in the travel strip within 1.0 m of the bumper
    caps `vdes` at 0.4 m/s.
- **B5.** `unknown_objects_mode` = `off` | `shadow` | `active` (default `active`), and
  `unknown_speed_cap_enabled`. Every threshold is in `stack_params.yaml`
  (`unknown_*`, `semantic_recognised_max_age_s`, `mpc_obstacles_*`).
- **No `.msg` / `.srv` changed.** Unknown objects use `class_id = "unknown"`.
  `/costmap/unknown_objects/debug` (JSON) and `/costmap/cell_owners` (OccupancyGrid)
  are new topics of existing types.

## 2. B6 tests (pytest, synthetic)

| Requested | Test |
|---|---|
| single noisy cell → nothing | `test_unknown_objects.py::TestTheTracker::test_a_single_noisy_cell_never_becomes_an_object` (+ flickering cluster) |
| persistent cluster → unknown | `…::test_a_persistent_cluster_becomes_an_unknown_object_after_persist_s` |
| YOLO over unknown → same ID, class, no dup | `…::test_a_yolo_detection_upgrades_the_unknown_in_place`; on the node: `test_unknown_objects_node.py::TestTheUpgradeOnTheWire` |
| recognised object's cells freed | `…::test_a_recognised_objects_own_cells_never_make_an_unknown`, `TestOwnership::test_a_tracked_objects_own_cells_are_freed` |
| person walking away → decay ≤ 1 s | `…::test_a_person_walking_away_leaves_no_unknown_behind_within_1_s` |
| wall cells → wall | `TestOwnership::test_cells_on_a_confirmed_wall_belong_to_the_wall`, `…::test_wall_cells_never_become_unknown_objects` |
| real OSQP solve plans around an unknown | `mpc_controller/test/test_obstacles_v2.py::TestTheSolve::test_a_real_osqp_solve_plans_around_an_unknown_obstacle` |
| shadow → MPC input unchanged | `test_unknown_objects_node.py::TestModes::test_shadow_computes_everything_but_the_mpc_input_is_unchanged` |
| 8 obstacles within budget | `test_obstacles_v2.py::TestTheSolve::test_eight_obstacles_solve_within_1_3x_of_one` |

**Results on linus.**
- `f1tenth_costmap`: 284 → **299 pass**. These include rclpy node tests, run on
  Jazzy rclpy against prebuilt messages that are byte-identical to the Orin's.
- `f1tenth_navigation`: 12 pass. `f1tenth_control`: 54 pass. `f1tenth_params`: 54 pass.
- `mpc_controller`: 788 pass. The **13 failures are the same set as HEAD on linus**
  (the Orin reports 11 pre-existing). `test_abort_holds_the_car`'s fake needed the
  new attributes.
- **Lint:** flake8 and pep257 are not worse than HEAD on any touched file; new files
  are clean.

## 3. Evidence

### Clearance of the soft `w_obs` term (closed loop, synthetic, `test_print_the_clearance_grid`)

The setup is a 4 m `goal_distance` move with one obstacle 2 m ahead at a lateral
offset, using the real `build_straight_corridor`, `compute_local_target` and OSQP RTI.
The body gap uses a disk model: rear-axle centre, `car_radius` 0.20.

| offset \ r | 0.10 | 0.18 | 0.30 |
|---|---|---|---|
| 0.0 | −0.123 | −0.155 | −0.206 |
| 0.1 | −0.058 | −0.094 | −0.140 |
| 0.2 | +0.003 | −0.037 | −0.087 |
| 0.3 | +0.062 | +0.019 | −0.037 |
| 0.5 | +0.209 | +0.144 | +0.069 |
| 0.7 | +0.400 | +0.321 | +0.210 |

An obstacle near the straight path is passed far too close, or overlapped in this
model, whether it is recognised or unknown. That is the likely mechanism behind
"close passes". v2 changes which obstacles reach the term, not the term itself.

### B7 replay (`tools/obstacles_v2_replay.py`, both bags)

Per-frame data is in `<bag>.frames.csv` (gitignored by `*.csv`; regenerate with the
tool). The summary for each bag is in `<bag>.summary.json`.

| | corridor10m_20261019T131815 | corridor_obstacles_20261019T135307 |
|---|---|---|
| window | 1st movement −2 s → its end +2 s (one movement only), 49.4 s, 408 cycles | no movement: whole bag, 39.1 s, 337 cycles |
| cells / frame (wall / object / island) | 87 / 0 / 137 | 59 / 0 / 149 |
| unknown objects confirmed (total; per frame mean / max) | 99; 13.3 / 23 | 15; 12.7 / 13 |
| lifetime p50 / max | 5.6 s / 25.2 s | 38.7 s (the whole bag) |
| near a wall line (≤ 0.3 m) / in the strip ahead | 11 / 0 | 0 / 0 |
| upgrades unknown → class | 0 (YOLO dead after 0.97 s) | 0 (YOLO dead after 0.66 s) |
| duplicates surviving a cycle / folds | 0 / 106 | 0 / 2 |
| cell-derived MPC rows | 0 (live: 3 per solve) | 0 (no solves) |
| MPC disks per cycle before the cap of 8 (mean / max) | 36.7 / 54 | 31.8 / 36 |
| speed-cap cycles | 0 | 0 |
| live plan → v2 disks, min / p5 (312 solves) | 0.48 m / 0.62 m: never inside 0.32 m | — |
| live solve p50 / p95 (Orin) | 27.1 / 54.2 ms, 1 obstacle (the frozen chair) | 0 solves |
| LiDAR step p50 / p95 (linus) | 2.36 / 3.19 ms | 1.88 / 2.16 ms |

**PNGs** (map frame; red island cells, blue wall cells and walls, grey fill = the MPC
disks, circles = tracks, magenta = the live MPC prediction):
- `output/obstacles_v2/corridor10m_20261019T131815_topdown_0.png`, `_1.png`, `_2.png`
- `output/obstacles_v2/corridor_obstacles_20261019T135307_topdown_0.png`, `_1.png`, `_2.png`

### MPC solve time (`test_print_the_timing`, linus, `solve_mpc_step` RTI)

| obstacles | before p50 / p95 (ms) | after p50 / p95 (ms) |
|---|---|---|
| 0 | 6.16 / 6.70 | 6.01 / 7.04 |
| 1 | 8.39 / 9.44 | 7.68 / 8.56 |
| 4 | 10.37 / 11.44 | 7.92 / 8.80 |
| 8 | 12.98 / 14.14 | 8.41 / 9.37 |
| 12 | 15.58 / 17.12 | 8.78 / 9.79 |

**Orin factor.**
- `extract_lines`' Python backend measures p50 24.6 ms a scan on the Orin under the
  audit's load (commit `c421578`), and p50 7.6 ms a scan on linus (200 scans of the
  corridor bag).
- That gives a factor of about **3.2**, so the LiDAR step is about 6–8 ms p50 on the
  Orin: over the 3 ms budget.
- It runs in `semantic_layer_node`, not in the MPC's loop.
- Mitigations, none applied: `unknown_rate_hz` 5, a smaller `unknown_range_m`, or a
  C port of the cell step.

## 4. Decisions

`output/obstacles_v2/decisions.md` has twelve decisions:

- **D1:** the cell view comes from the scan, not `/slam/map`.
- **D2:** objects own cells only if recognised.
- **D3:** a person's legs are one object.
- **D4:** each sensor keeps its own lifecycle.
- **D5:** wall observations own cells too.
- **D6:** a cluster seen through a gap in a wall is dropped (first-obstacle rule).
- **D7:** a stale recognised track leaves the MPC list. This fixes the Orin A3
  "latched chair" defect on the new path.
- **D8:** a split cluster refreshes its track, and a track whose cells are still hit
  stays alive.
- **D9:** solve vectorisation.
- **D10:** no cell rows outside `costmap` mode.
- **D11:** chain disks for elongated clusters.
- **D12:** single-linkage clustering.

**Deviation from the wording.** The rule "wall = within 0.10 m of a **confirmed** wall
segment" alone left the corridor's own walls as islands. D5 adds this cycle's long
LiDAR wall observations; `unknown_wall_obs_min_span_m := 0` restores the literal rule.

## 5. Commits (local, branch `obstacles-v2`)

| commit | what |
|---|---|
| `88cfc1c` | B1/B2: cell ownership and island candidates |
| `6ece508` | B3: one tracker for recognised and unknown objects |
| `79562b1` | B4/B5/B6: one obstacle path to the MPC, modes, speed cap, tests |
| `67beb26` | B7 findings: chain disks, single-linkage clustering (D11, D12) |
| (this one) | B7: the replay tool, its outputs, this report |

## 6. To run on the Orin

Build and test the touched packages: `f1tenth_costmap`, `mpc_controller`, `f1tenth_params`.

- The rclpy node tests in `test_unknown_objects_node.py` and `test_abort_holds_the_car.py`
  ran here on Jazzy, and need running on Humble.
- **First live test, shadow mode:** set `unknown_objects_mode: shadow` in
  `stack_params.yaml`, then watch
  `ROS_SUPER_CLIENT=TRUE ros2 topic echo /costmap/unknown_objects/debug`. That gives
  `lidar_ms` measured on the Orin, the counts per frame, and the upgrades with a
  working camera.
- **A2** "works with the BT process killed" was asserted on the import graph by the
  Orin session. To verify it live, kill the BT and put an object 0.15 m from the
  laser.
