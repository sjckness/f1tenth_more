# Which occupancy grid the freespace door detector reads

`/slam/map` (`nav_msgs/OccupancyGrid`), published by `slam_toolbox`'s
`async_slam_toolbox_node` in the **`map`** frame at **0.05 m/cell**, re-published
every **`map_update_interval: 5.0` s** (~0.2 Hz) with RELIABLE / TRANSIENT_LOCAL
(latched) QoS -- `src/f1tenth_navigation/config/slam_toolbox_params.yaml`, where
`/map` is remapped to `/slam/map` by `launch/slam.launch.py`.

It is the stack's **only** live occupancy grid: Nav2's `map_server` (`/map`, via
`f1tenth_navigation/launch/map.launch.py`) serves a *static* saved map behind
`enable_nav2_map` and there is no Nav2 global costmap in the bringup; the
`f1tenth_costmap` nodes that already consume a grid (`costmap_boundary_node`,
`costmap_renderer_node`) all read `/slam/map` too, and the map-frame pose they
pair it with is the global EKF's `/ekf_global/odometry/filtered`, not
`/slam/pose`.

Consequence for this detector: **the grid geometry only changes every 5 s**, so
running faster than that re-measures the same cells. The cycle still runs at
2 Hz because what IS new on every tick is the `/scan` confirmation -- beams
reaching past the jamb line -- and that is what `freespace_doors_confirm_hits`
counts: 3 independent scan confirmations (~1.5 s) over a geometry the grid
holds still, rather than the semantic detector's 5 sightings of a wall pair
that moves with every scan.

## Measured, on linus (laptop, python3.12 / numpy 1.26 / scipy 1.11)

Median of 7 cycles per row, `tools`-free: the same `occupancy_window` ->
`skeleton` -> `candidate_doors` chain the node runs, on the synthetic scenes
`test_freespace_doors.py` builds.

| scene | `window_m` | cells | window | skeleton | candidates | total |
|---|---|---|---|---|---|---|
| room-door-room 0.90 | 4.0 | 161² | 2.2 | 16.5 | 0.5 | **19 ms** |
| room-door-room 0.90 | 6.0 | 241² | 5.6 | 32.3 | 0.8 | **39 ms** |
| room-door-room 0.90 | 8.0 | 321² | 9.4 | 55.1 | 1.1 | **66 ms** |
| cluttered room, 40 boxes | 8.0 | 321² | 9.5 | 22.2 | 12.1 | **44 ms** |

Against the 500 ms a 2 Hz tick allows, with the default 8 m window. The
medial-axis thinning dominates and is quadratic in `window_m` (one cell layer
is peeled per iteration, and there are more layers as well as more cells), so
`freespace_doors_window_m` is the knob with real leverage -- 8 -> 6 m is ~40 %
off the cycle. Clutter moves the cost the other way: the free space is
thinner, so thinning converges sooner, but there are more in-band pinches to
evaluate (`freespace_doors_max_candidates` caps that at 16).

The Orin has not been measured. Expect the same shape, a few times slower; if
2 Hz will not hold, lower `freespace_doors_window_m` before
`freespace_doors_rate_hz`, because a smaller window costs reach but a slower
rate costs confirmation latency on a detector that already waits 3 cycles.

The thinning was 284 ms at 8 m in its first form (an `(8, rows, cols)` stack
and an `np.roll` per sub-iteration). The 256-entry Zhang-Suen lookup table in
`_neighbour_code` / `_zhang_suen_tables` is what brought it to 55 ms;
`test_the_cycle_fits_the_two_hertz_budget` guards against losing that again.

## Replayed on real grids -- and what that changed

`tools/freespace_door_replay.py` runs the whole cycle offline from a bag
(`/slam/map` + `/ekf_global/odometry/filtered` + `/scan` + `/tf_static`), no
ROS, no car, no simulator. Two of the bags in `~/bags` carry all four:

| bag | grids | scans | cycles | ms med / p95 / max | doors |
|---|---|---|---|---|---|
| `corridor10m_20261019T131815` | 16 | 3223 | 180 | 59 / 74 / 78 | 0 |
| `corridor_obstacles_20261019T135307` | 9 | 1549 | 79 | 43 / 46 / 47 | 0 |

(`corridor10m_20261016T144537` has `/scan` but no `/slam/map` and no global
EKF pose, so this detector cannot be replayed on it at all.)

Zero doors is the right answer for both -- neither run has a doorway in it.
The cost matches the synthetic benchmark on the same machine, which is the
first thing the replay confirmed. **The other two things it found were bugs**,
and neither would have shown up on a hand-drawn grid:

**1. The candidate selection was not finding local minima.** The band was
applied first and each in-band cluster's narrowest cell taken. Wherever the
medial axis descends through 0.70 m on its way into a tight corner, that
cluster is cut off at exactly 0.70 and its argmin is the CUT, not a pinch. On
the last grid of `corridor10m_20261019T131815` all 38 candidates came back at
`2d = 0.700` exactly, every one rejected downstream -- and 22 of the 38 were
never evaluated at all, because `max_candidates` is 16 and the ordering is
narrowest-first, so a genuine 0.90 m doorway would have sorted *after* every
0.70 m artefact and been cut from the list before anything looked at it. The
accepted set was right; the recall under the cap was not. Fixed by filtering
to curve cells (degree 2) and to true local minima
(`freespace_doors_local_min_radius_m`) BEFORE the band: 38 candidates became
21, spread across the band (0.70, 0.73, 0.80, 0.85, 0.90, 1.00) instead of all
pinned at its lower edge. That real grid's skeleton is why the degree filter
earns its place too: 814 of its 2303 skeleton cells are branch points and only
1476 are curve cells, because the free-space boundary is ragged where it meets
unexplored space.

**2. A jamb could be a two-cell blob.** With the selection fixed, the replay
confirmed one 1.11 m "door" at (4.76, -5.38) -- and its far jamb was 2 cells
of occupancy (the near one was solid wall). It had passed the bottleneck test,
the rise test, the symmetry check against `2d` *and* a 12-beam scan
confirmation: nothing in the chain asked how much obstacle was behind a jamb,
because the march stops at the first non-free cell and one cell is enough to
stop it. A chair leg standing the right distance from a wall is a doorway
under those rules. Fixed with `freespace_doors_jamb_min_cells` (4 non-free
cells within `freespace_doors_jamb_radius_m`, 0.15 m), which rejected all 19
of that bag's shape-passing candidates, the three that were confirming the
false door among them, and left both bags at zero.

**What this does NOT establish.** Neither bag contains a real doorway, so the
replay has shown that the detector does not hallucinate one and costs what it
should -- it has NOT shown that it finds a real door on a real grid. The gate
to watch first on the car is `thin_jamb`: if real doorways come back with that
code, `freespace_doors_jamb_min_cells` is the knob, and
`test_the_jamb_gate_can_be_turned_off` exists so that it is provably the only
thing in the way.
