# costmap_boundary_node: Python vs Rust parity

Branch `jazzy-rust` (jazzy-campaign + campaign-final 0c1cd41), linus, 2026-10-18.
Reference: `f1tenth_costmap/costmap_boundary_node.py` + `costmap_boundary.py` (cpu-cleanup
version: 10 Hz, strip search for the front clearance, TF listener only with the polytope).
Port: `src/rust/f1tenth_costmap_boundary_rs` (rclrs 0.7.0). Harness:
`tools/rust_parity/costmap_boundary/`.

**Result: 0 differences beyond the stated tolerances on 145,583 poses (841 recorded car maps and
144,772 recorded global-EKF poses; 804 synthetic; 7 malformed). 99.32 % of constraints and 100 %
of front clearances are bit-identical. The rest differ by one ulp, all traced to CPython's
`math.hypot`.**

## 1. Recon: what the Python node does

| | |
|---|---|
| Inputs | `/slam/map` (nav_msgs/OccupancyGrid; RELIABLE, TRANSIENT_LOCAL, depth 1) and `/ekf_global/odometry/filtered` (nav_msgs/Odometry; BEST_EFFORT, VOLATILE, depth 10), the GLOBAL EKF's map-frame pose, x, y and yaw from the quaternion. No TF (the TransformListener exists only with `use_convex_polytope`, cpu-cleanup). The Python also subscribes `/mpc/corridor_markers` (MarkerArray) for the polytope path, even when it is off |
| Rate | timer at `extraction_rate_hz` **10 Hz** (cpu-cleanup: 20 -> 10, mpc_corr's control rate), on the node clock (sim time under use_sim_time). Publishes every tick from whatever is cached, however old; no staleness gate ("periodic-publish pass") |
| Algorithm | crop a +-`max_range_m` (5 m) box around the robot, clipped to the grid; cell centres; map -> base_link rotation R(-yaw). **Constraints**: per window (front +-35 deg, left [45, 135] deg, right [-135, -45] deg, car-frame bearing) the single nearest cell with value >= `occupied_threshold` (65) within 5 m; constraint = radial half-plane, normal = unit vector to the cell, offset = its distance (base_link-relative, un-shrunk). **front_clearance**: nearest occupied cell in the car-width strip `car_dx > base_front_x (0.405)`, `abs(car_dy) <= front_side_limit_m (0.186)`, measured from the bumper, at least `MIN_FRONT_DISTANCE_M` (0.01); nothing found -> `max_range_m + 1` = 6.0 |
| Unknown cells | -1 is below every positive threshold: never occupied, never "confirmed free" |
| Output | `/costmap/boundaries` (f1tenth_messages/BoundaryConstraintArray: 0-3 constraints in the order front, left, right; header stamp = node clock now, frame_id `base_link`), `/costmap/front_clearance` (std_msgs/Float32). Both reliable, depth 10. Until a map AND a pose have arrived: empty array, clearance withheld (WARN every 5 s) |
| Params | `map_topic`, `pose_topic`, `output_topic`, `front_clearance_topic`, `robot_frame`, `front_facing_max_deg` 35, `side_window_min_deg` 45, `side_window_max_deg` 135, `occupied_threshold` 65 (int), `max_range_m` 5.0, `extraction_rate_hz` 10.0, `use_convex_polytope` false + 9 polytope keys. costmap.launch.py sets 9 of them from its `costmap_boundary_*` launch arguments. `base_front_x`, `front_side_limit_m` and `MIN_FRONT_DISTANCE_M` come from `f1tenth_description.vehicle_frames` (URDF, stack_params) |

### Out-of-bounds behaviour (the "false occupied when the car drifts outside the map" hypothesis)

**The hypothesis is false in both implementations: outside the map is permissive, never occupied.**

- **Python.** `_crop_indices` clips the search box to the grid. A box entirely outside gives
  `col_min >= col_max`, so `None` for every direction. With a partial overlap only the in-map
  cells are searched, and cells beyond the map edge do not exist. Result: no constraint from the
  unmapped side and `front_clearance` 6.0 m, "clear". The node's own module docstring already
  says this ("Out-of-bounds therefore fails PERMISSIVE"). Measured here: a car at (50, 50),
  (-50, 3) and (1e6, -1e6) next to an 8 m map gives 0 constraints and clearance 6.0. A car
  partly outside sees only the mapped cells within its 5 m box.
- **Rust.** Identical on all of those cases (`outside_far`, `outside_partial`: 9 poses, 0
  differences), and on the live node (section 4, phase 3).
- Consequence, not a bug in the port: a car that dead-reckons off the map has no wall
  constraints and a "clear" front, so the MPC loses its wall rows and any front_clearance
  consumer sees 6.0 m. This is fail-OPEN by design ("unknown is no data"). The instant-stop
  symptom it was blamed for needs a small clearance, which this cannot produce.

## 2. The port

`f1tenth_costmap_boundary_rs` is a node with the same name (`costmap_boundary_node`), the same
parameter names, defaults and types, and the same topics, types and QoS. It publishes the same
messages, stamped from the node clock with frame_id `robot_frame`.

- One pass over the cropped block computes the three windows and the strip. Each search's
  per-cell arithmetic is the Python's, in the same order: cell centre `origin + (i + 0.5) *
  resolution`, R(-yaw), `hypot`, `atan2`, the window tests. numpy's argmin tie-break (the first
  minimum in row-major order of the crop) comes from a row-then-column loop with a strict `<`.
- **Differences, all on inputs the Python cannot process** (it raises inside the timer
  callback and the node process exits): grid data length != width * height, resolution <= 0 or
  non-finite, non-finite origin, non-finite pose. The Rust publishes the same "empty array,
  clearance withheld" pair as "never received" and logs why.
- **NaN yaw: an intended difference** (section 3, bugs): the Python does not raise and publishes
  "no constraints, clear 6.0 m". The Rust withholds the clearance.
- **Not ported:** `use_convex_polytope:=true` (safe_corridor.py). The Rust node refuses to start
  with it ("run impl:=py"). The polytope's parameters are still declared, so the parameter
  interface is identical. `/mpc/corridor_markers` is not subscribed (the Python reads it only
  for the polytope). The `/costmap/safe_corridor_report` publisher exists and never publishes,
  as in the Python with the flag off.
- **Extra parameters**, set by costmap.launch.py from vehicle_frames: `base_front_x_m`,
  `front_side_limit_m`, `min_front_distance_m` (required, no defaults). Also
  `timing_report_period_s` (default 0: off).
- Switch: `impl:=py|rust` (supervisor_bringup / stack_bringup / sim_isaac.sh / e2e_test.sh) ->
  costmap.launch.py `costmap_boundary_impl`. Both Nodes are in the launch file; exactly one
  condition is true, none without `enable_slam` (`test_costmap_boundary_impl_launch.py`).
- Gates: `cargo fmt --check`, `clippy --all-targets -D warnings` (unsafe forbidden;
  unwrap/expect/panic denied outside tests), `cargo test` (9 tests), the colcon tests of the
  touched packages.

## 3. Parity

Python side: the real `CostmapBoundaryNode._extraction_tick`, with its two publishers replaced by
recorders and `_latest_grid` / `_latest_pose_xy_yaw` set per case. That is the node's own code
path, including `_publish_empty`. Rust side: `costmap_boundary_replay`, i.e.
`extraction::extract`, the function the node's timer calls. Parameters from the Python node
instance and vehicle_frames.

**Tolerances** (explicit, gate): normal components 1e-12, offset 1e-12 m, front_clearance
0 Float32 ulps, constraint count exact, published order exact. "Python raised" must pair with
"Rust: empty + withheld".

| Group | Maps | Poses | Constraints compared | Differences | Python raised |
|---|---:|---:|---:|---:|---:|
| Synthetic | 16 | 804 | 1,975 | **0** | 0 |
| Malformed | 6 | 7 | 0 | **0** (+1 intended) | 5 |
| Recorded (214 car bags with map + global pose; 841 map messages, <= 400 poses each) | 841 | 144,772 | 432,934 | **0** | 0 |

- Synthetic maps: empty; all unknown; all occupied; a walled room (incl. poses exactly on cell
  centres and yaw +-pi); corridor (unknown outside); corner; random clutter with values
  63/64/65/66/70/100/-1; values around the threshold; **exact ties** (8 equidistant cells around
  a robot on a cell centre, 6 yaws: the argmin tie-break); the car far outside, partly outside,
  at the edge; resolutions 0.1, 0.025, 0.05 and 0.0499999 with non-aligned origins (0.0123, -7.77);
  a non-square map; map updates (each case is a new grid; live in section 4).
- **Bit-exactness**: 431,964 of 434,909 constraints identical to the last bit (99.32 %); max
  difference 2.2e-16 in a normal component and 8.9e-16 m in an offset. All front clearances
  are identical (0 ulps). The residual is fully explained: `boundary_from_nearest_point`
  computes `k` with **CPython's `math.hypot`** (its own algorithm since 3.10), the Rust with
  libm `hypot`; they differ by 1 ulp on 0.59 % of random pairs (measured), against 0.68 % of
  constraints here. Every other operation, including numpy's `np.hypot` and `np.arctan2`, is
  glibc's libm on both sides (numpy 1.26.4, no AVX-512 on the i7-9700K).
- Malformed (Python raised; Rust published empty + withheld): data 99 or 101 values for 10x10
  (`ValueError: cannot reshape`), resolution 0 (`ZeroDivisionError`), x = NaN
  (`ValueError: cannot convert float NaN to integer`), x = inf (`OverflowError`). An empty 0x0
  grid is valid for both: empty array, clearance 6.0.
- Harness bug found and fixed along the way: serde_json's default float parser is not
  correctly rounded, which first showed up as ~1e-14 normal differences on cells next to
  the robot (catastrophic cancellation in `car_dx`). With `float_roundtrip` the input
  doubles are exact.

Raw: `output/rust/parity_costmap_boundary.json`. Re-run (about 10 min):
`tools/rust_parity/costmap_boundary/parity_costmap_boundary.py`.

### Bugs found in the Python (reported, not fixed)

1. **NaN yaw from the pose topic publishes "clear".** `cos(NaN)` makes every car-frame
   distance NaN, so no cell passes `dist <= max_range_m`. The node publishes an empty
   constraint array and `front_clearance = 6.0`: no walls, nothing ahead, from a pose that
   carries no information. A NaN x or y instead raises `ValueError` in the timer callback and the
   node process exits. The node has no `respawn` and costmap.launch.py keeps running with its
   other two nodes, so the supervisor sees nothing to restart: `/costmap/boundaries` and
   `/costmap/front_clearance` simply stop. A diverging EKF produces exactly these inputs.
   Rust: empty + withheld for all three, and the node keeps running.
2. **A malformed grid or a zero resolution kills the node** the same way (an exception in the
   timer callback, silent for the supervisor). Not seen in any recorded bag; slam_toolbox does
   not publish such grids.
3. **Permissive out-of-bounds** (section 1): known and documented in the node. Listed because
   the task asked for it explicitly.
4. **The front cone of the constraints** still has the "corridor phantom". With side walls at
   +-1 m, the front constraint is the side wall at 1.74 m, not the wall ahead. Only the
   front_clearance moved to the strip (2026-10-15). The MPC's front boundary row therefore
   follows the side walls in a narrow corridor. Behaviour, not a crash; ported as is (it is
   the parity reference).

## 4. Live check (both nodes at once, same inputs)

`tools/rust_parity/costmap_boundary/live_costmap_test.py`: both real nodes on an isolated
localhost domain, fed the same `/parity/map` (transient local) and `/parity/pose` (best effort,
50 Hz), each on its own output topics.

| Phase | Python | Rust |
|---|---|---|
| 0: no input (1.5 s) | 15 empty arrays, 0 clearances | 16 empty arrays, 0 clearances |
| 1: map A (160x160, 0.05 m, walled room + a block), static pose | constraints == Rust: **True**; clearance [3.4081037] | clearance [3.4081037] |
| 1: rate, frame_id, stamp | 10.0 Hz, ['base_link'], stamp within 0.4 ms of receipt | 10.0 Hz, ['base_link'], within 0.8 ms |
| 2: map B (220x120, 0.1 m, other origin) published mid-run | identical: **True**, output changed from map A: True | clearance [1.4317597] |
| 3: pose (40, 30), outside map B | identical: **True**: no constraint, clearance [6.0] | [6.0] |

Stamps: both nodes stamp with the node clock at the tick (wall time here; sim time under
use_sim_time, sim_component.launch.py sets it for every node, the Rust ones included).

## 5. CPU and callback time, py vs rust

Measured with both nodes running at once on the same inputs, linus idle otherwise (i7-9700K):
a recorded car map (240x215 cells at 0.05 m, the last /slam/map of
2026-10-08T12-29-44) and that bag's 1628 recorded global-EKF poses replayed at 50 Hz for
60 s; each node at its 10 Hz (600 and 600 ticks).
Tick time is each node's own measurement around the timer callback (`timed_py_node.py` wraps the
unchanged Python `_extraction_tick`; the Rust node's `timing_report_period_s`), reported every
5 s; the table gives the median of the per-report p50, the largest p95 and the largest max.

| | Python | Rust | ratio |
|---|---:|---:|---:|
| process CPU (100 % = one core): ticks + map + 50 Hz pose callbacks + DDS | 8.4 % | 1.0 % | 8.1x |
| tick p50 | 5.72 ms | 0.32 ms | 18x |
| tick p95 | 6.26 ms | 0.40 ms | 15x |
| tick max | 6.72 ms | 0.46 ms | |
| ticks measured | 560 | 559 | |

Offline, the Python `_extraction_tick` alone (the parity run, 145,583 ticks on all recorded
maps) is p50 3.58 ms, p95 4.73 ms, max 31.2 ms; the live tick adds message construction and
publishing. At 10 Hz the Python node uses ~6 % of a core in its tick alone; the Rust node
~0.3 %. Not measured on the Thor or the Orin.
