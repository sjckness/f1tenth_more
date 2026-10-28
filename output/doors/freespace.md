# The `freespace` door source: the grid, then the scan

## 1. Which occupancy grid the stack has

`/slam/map` (`nav_msgs/OccupancyGrid`), published by `slam_toolbox`'s
`async_slam_toolbox_node` in the **`map`** frame at **0.05 m/cell**, re-published
every **`map_update_interval: 5.0` s** (~0.2 Hz) with RELIABLE / TRANSIENT_LOCAL
(latched) QoS -- `src/f1tenth_navigation/config/slam_toolbox_params.yaml`, where
`/map` is remapped to `/slam/map` by `launch/slam.launch.py`.

It is the stack's **only** live occupancy grid: Nav2's `map_server` (`/map`)
serves a *static* saved map behind `enable_nav2_map` and there is no Nav2 global
costmap in the bringup. The map-frame pose it is paired with is the global EKF's
`/ekf_global/odometry/filtered`, not `/slam/pose`, matching
`costmap_boundary_node`.

## 2. Why the medial-axis detector found nothing

The first version cropped that grid, distance-transformed the free space,
thinned it to its medial axis and looked for a bottleneck whose clearance rose
on both sides. Replayed on real data it reported **zero doors**, and the reason
is structural, not a tuning miss:

> **A room behind a door is not a room in the grid.** It is a *ray fan* -- a
> handful of beams that got through the opening, each marking a thin line of
> free cells, with UNKNOWN between them, because nothing has ever driven in
> there to fill it.

So the far side never becomes a wide region, "clearance rises on both sides"
cannot hold, and the few skeleton cells that do reach through a doorway are
short branches off the corridor's axis which the spur pruning discards. The
detector was asking the map a question the map cannot answer until after you
have already driven through the door. Two real bugs were found and fixed on the
way to establishing that (see §5), but neither was the reason.

The code is still here and still tested, behind `freespace_doors_use_skeleton`
(default **false**): the geometry is sound wherever the map *is* filled in.

## 3. What runs instead: scan jumps

A doorway in a range profile is a **near-far-near** step: a jamb at A, a run of
beams that suddenly reach much further (into the room, or to nothing at all),
and a second jamb at B. That signature is there on the *first* scan that sees
the opening, needs no accumulated map, and is as strong for a door in the side
wall of a corridor as for one straight ahead -- which matters, because most of
ours are to the side.

The grid is demoted to a cross-check, and its UNKNOWN cells are read as
**unknown** rather than as obstacles. It is asked three cheap questions: is the
line A-B clear of real obstacle, is there **any** free cell behind it (one
ray's worth is enough -- there is deliberately no "wide room" requirement), and
is there occupancy at A and at B where the jambs should be. With no grid at all
the detector still runs and flags its candidates `no_grid`.

**Permissive on purpose**, because a missed door is a mission that cannot run
while a false door is a marker in Foxglove: a short occupied run across the
opening does not veto it (only a *continuous* `occupied_run_veto_m` does); a few
near beams inside it do not split it; a missing jamb keeps the candidate with a
`one_jamb` flag; and the detection width band (0.60-1.50 m) is wider than the
nominal one (0.70-1.40 m), outside which a door is published with a `marginal`
flag rather than dropped. **A flag is not a rejection** -- a flagged candidate
is a door, tracked and published like any other.

What stops a one-scan artefact is therefore the **confirmation**, which is
about where the car was standing rather than how many times it looked: two hits
from poses `confirm_pose_spread_m` apart, or `confirm_same_pose_hits` hits from
anywhere. A reflection or a beam that slipped past a corner is perfectly
repeatable from one standing position; a doorway is not.

## 4. Replayed on real bags

`tools/freespace_door_replay.py` runs the cycle offline from a bag
(`/scan` + `/ekf_global/odometry/filtered` + `/slam/map` + `/tf_static`), no
ROS, no car, no simulator. `--skeleton` runs the old detector for comparison.
Two bags in `~/bags` carry all four topics; `corridor10m_20261016T144537` has
no `/slam/map` and no global EKF pose and cannot be replayed at all.

| bag | cycles | ms med / p95 / max | doors (medial axis) | doors (scan jumps) |
|---|---|---|---|---|
| `corridor10m_20261019T131815` | 506 | 11.6 / 14.6 / 21.8 | **0** | **32** (18 strong) |
| `corridor_obstacles_20261019T135307` | 340 | 15.7 / 17.2 / 18.4 | **0** | **12** (0 strong) |

"Strong" means seen from at least 1 m of travel, no flags, at least 5 hits.
12 ms per cycle at 10 Hz, against the ~70 ms the medial-axis version cost at
2 Hz: there is no distance transform and no thinning on this path.

**The 32 is the `max_tracks` budget, not the detection count.** Uncapped the
same run confirms **98** doors. A 19 m drive through a cluttered room does not
contain 98 doorways, and that is the shape of the trade the permissive defaults
buy. Measured sweep on that bag, all doors / strong doors:

| change | doors | strong |
|---|---|---|
| shipped defaults | 98 | 30 |
| `confirm_same_pose_hits: 0` | 93 | 29 |
| ... and `confirm_pose_spread_m: 0.50` | 72 | 29 |
| ... and `confirm_pose_spread_m: 1.00` | **57** | **29** |
| `jump_min_m: 0.80` | 96 | 31 |
| `beyond_beams_min: 6` | 93 | 27 |
| detection band = nominal band | 81 | 36 |

The **confirmation** knobs are the only ones with real leverage, and they cost
nothing in recall: requiring a metre of travel between two sightings removes 41
doors and not one strong one. The geometry knobs barely move the count, which
is the honest signal that from a single scan a doorway and a gap between two
pieces of furniture are nearly the same thing.

`max_tracks` (32) exists because a confirmed door is never forgotten and each
one costs a TF and five markers every cycle -- 98 of them is about a thousand
transforms a second into the tree. It drops the weakest evidence first
(unconfirmed, then fewest hits, then least pose spread) and pins
`door_candidates` on the debug topic at the limit when it bites, so hitting it
is a tuning signal rather than silent corruption.

**What this does not establish.** Neither bag contains a known doorway, so none
of the 32 has been confirmed against ground truth. What the replay shows is
that the detector runs at 12 ms, finds openings where the skeleton version
found none, and that its false-positive rate on cluttered indoor data is high
by design and controlled by one knob.

## 5. Bugs the real data found

Each of these was invisible on a hand-drawn grid or a synthetic profile, and
each now has a test.

1. **The candidate selection was not finding local minima** (medial-axis path).
   The width band was applied first and each in-band cluster's narrowest cell
   taken, so wherever the axis descends *through* 0.70 m into a tight corner the
   cluster is cut at exactly 0.70 and its argmin is the cut. All 38 candidates
   on one grid came back at `2d = 0.700` exactly; 22 were never evaluated,
   because `max_candidates` is 16 and the order is narrowest-first, so a genuine
   0.90 m doorway would have been cut from the list first. Fixed by filtering to
   curve cells and true local minima before the band.
2. **A jamb could be a two-cell blob** (medial-axis path). One confirmed 1.11 m
   "door" had a solid wall on one side and 2 cells of occupancy on the other,
   after passing the bottleneck, rise, symmetry *and* a 12-beam scan check.
   Nothing asked how much obstacle was behind a jamb. Fixed with
   `jamb_min_cells`.
3. **`check_behind` sampled perpendicular to the opening.** Beams do not turn to
   face a doorway after passing it: seen at a shallow angle, the beams through a
   0.9 m opening 3 m away have fanned more than a metre sideways by the time
   they are 1 m past the threshold, so a perpendicular box lands where no beam
   went and reported `no_behind` for every good side door. Now samples **along
   the rays**.
4. **The `split` test counted beams that failed to reach `beyond_m`**, which
   rejected every side door into a small room: a beam clipping that room's far
   wall 0.3 m past the threshold read as "short" and a whole run of them split
   the opening. Now counts only beams that stopped *at or before* the threshold
   (`block_margin_m`) -- a beam past the threshold went through, however little
   further it got.
5. **The pose memory capped at the first 16 poses**, so a track born while the
   car was standing still reported `pose_spread 0.00` after 300 hits, every
   later pose 19 m up the corridor discarded by the cap. Now a running
   diameter.
6. **The surface-end gate was `or`, not `and`**, so one real jamb was enough.
   It rejected exactly 1 opening in ten thousand on a real bag -- a gate that
   never fires is not a gate. Now both ends.
7. **`xy or ()` on a numpy array** raised `ValueError: truth value of an array
   ...` inside the marker builder, which the node's cycle guard caught -- so it
   showed up as "the debug markers are silently never published" rather than as
   a crash.

## 6. Params to tune first

1. **`freespace_doors_confirm_pose_spread_m`** (0.15) with
   **`freespace_doors_confirm_same_pose_hits`** (3). The only knobs with real
   leverage on false positives, and they cost no recall. Raise the spread to
   0.5-1.0 m and set the same-pose fallback to 0 to require genuine pose
   diversity.
2. **`freespace_doors_jump_min_m`** (0.50). The first and bluntest gate: below
   it nothing downstream happens. If doors are missing *entirely*, check
   `/viz/doors_freespace_debug` for yellow points at the jambs before touching
   anything else.
3. **`freespace_doors_beyond_m`** (0.50) and
   **`freespace_doors_beyond_beams_min`** (2). The through-the-opening evidence.
   Loosen if real doorways seen edge-on come back as `few_beams`.

`freespace_doors_max_tracks` (32) is a TF budget, not a tuning knob -- raise it
only if a mission genuinely needs more than 32 doors held at once.
