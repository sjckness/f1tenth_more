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

| bag | cycles | ms med | doors (medial axis) | confirmed | **strong** |
|---|---|---|---|---|---|
| `corridor10m_20261019T131815` | 506 | 12.9 | **0** | 56 | **28** |
| `corridor_obstacles_20261019T135307` | 340 | 16.6 | **0** | 0 | **0** |

12 ms per cycle at 10 Hz, against the ~70 ms the medial-axis version cost at
2 Hz: there is no distance transform and no thinning on this path.

**`corridor_obstacles` finds nothing because the car never moves in it.** The
shipped confirmation is `confirm_pose_spread_m: 1.0` with
`confirm_same_pose_hits: 0` -- a metre of travel between two sightings, and no
same-pose fallback -- so a stationary car confirms no door at all. That is the
deliberate consequence of the sweep below, not an oversight: set the fallback
back to 3 if a mission has to resolve a door from a standstill, and expect the
false-positive count to climb.

### The sweep that chose those defaults

Measured on `corridor10m_20261019T131815`, all confirmed / strong:

| change | doors | strong |
|---|---|---|
| first, permissive defaults (`spread 0.15`, fallback 3) | 98 | 30 |
| `confirm_same_pose_hits: 0` | 93 | 29 |
| ... and `confirm_pose_spread_m: 0.50` | 72 | 29 |
| ... and **`confirm_pose_spread_m: 1.00`** (shipped) | **57** | **29** |
| `jump_min_m: 0.80` | 96 | 31 |
| `beyond_beams_min: 6` | 93 | 27 |
| detection band = nominal band | 81 | 36 |

The **confirmation** knobs are the only ones with real leverage, and they cost
nothing in recall: requiring a metre of travel between two sightings removes 41
doors and not one strong one. The geometry knobs barely move the count, which
is the honest signal that from a single scan a doorway and a gap between two
pieces of furniture are nearly the same thing -- the discrimination has to come
from *moving*, not from the gates.

### Confirmed vs strong

Confirmed means publishable; **strong** means a mission may drive at it
(`Door.strong`, `semantic_doors.DoorTracker.is_strong`): at least
`strong_min_hits` sightings, at least `strong_min_spread_m` between the two
robot positions furthest apart that saw it, and **no flags** -- so a
`marginal` width, a `one_jamb` opening or one found before any grid arrived
(`no_grid`) is published and drawn but never selected. `enter_door` resolves
both `"nearest"` and `"door_<id>"` over strong doors only and fails with
`no_strong_door` when a matching door exists but has not earned it, which is a
different answer from `door_not_found` on purpose. Both wall detectors publish
every confirmed door strong, so nothing changes for them -- and for the
profile detector that is not a shortcut, since its own confirmation already is
a viewpoint test.

In Foxglove the three tiers are the marker alpha: **strong solid**, merely
confirmed half-lit, candidate faint.

### The track budget

`max_tracks` (32) bounds TF and marker cost -- a confirmed door is never
forgotten and each costs a TF plus five markers per cycle. It drops the
**longest unseen** first, evidence breaking ties. Recency and not evidence,
measured: ranking by "confirmed first, then most hits" made a long-lived door
always outrank a young but genuine candidate, so the budget evicted candidates
*before* they could earn strength and a cap of 32 cost **10 of the 29 strong
doors** the uncapped run found. With recency first the same cap costs 1 of 29.

**What this does not establish.** Neither bag contains a known doorway, so none
of the 28 strong doors has been checked against ground truth. What the replay
shows is that the detector runs at 13 ms, finds openings where the skeleton
version found none, and that its false-positive rate on cluttered indoor data
is high by design and controlled by one knob.

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

## 6. One doorway, one track

The detector's first real outing published the same doorway several times.
On `corridor10m_20261019T131815`, 23 of its 30 strong doors were overlapping
pairs: ids 5 and 15 sitting 0.30 m apart with the same axis and 307 against 48
hits, and a cluster of four inside 0.6 m at four different angles, up to 97
deg between them.

The cause is that **nothing ever merged two tracks**. `assoc_max_m` only ever
folds a candidate into an existing track, so once both tracks existed they
never met again -- and one candidate landing a hair over 0.30 m from the
track that already had it was enough to create the second one.

Two tests now decide that two tracks are one opening, both in
`DoorTracker`, both 0 for the wall detectors:

- **`merge_tracks_overlap`** (1.0) -- centres closer than the mean of the two
  widths. *Two doorways cannot overlap in space*, so if two 1.1 m openings
  sit 0.8 m apart one of them is false whatever the axes say. Scale-free
  where a fixed radius is not: 1.2 m between two 1.2 m doorways, 0.7 m
  between two narrow ones. This is what the angle-spread clusters needed.
- **`merge_tracks_parallel_m`** (1.0) with **`merge_tracks_yaw_deg`** (25) --
  a duplicate of a *side* door drifts ALONG the wall, latching onto one real
  jamb and something else, so its copy sits further away than its own width
  (measured pairs at 0.89, 0.90, 0.91 m with 2-4 deg between their axes).
  The axis gate is what stops this from fusing two real openings a metre
  apart.

The survivor is the better-evidence track (confirmed, then strong, then hits,
then pose spread, then the older id) and keeps its own geometry, flags and
fixed direction. It absorbs only evidence: the loser's pose-diameter
endpoints, `max` of the hit counts (not the sum -- in a cycle where both
tracks took a hit from one scan, summing counts that scan twice), the earlier
`born_sec` and the later `last_seen_sec`. Clusters collapse closest-pair
first, repeatedly.

The same test also runs at **association** time, which is what keeps ids
stable: a candidate outside `assoc_max_m` but inside the same-door test
counts as a sighting of the existing track instead of becoming a new one, and
does *not* drag the track's geometry a third of the way towards itself the
way the EMA would. Letting those tracks be born and then merged away churned
1024 ids over 506 cycles for a final 19 doors.

Measured on `corridor10m_20261019T131815`, 506 cycles over 3223 scans:

| | before | after |
|---|---|---|
| tracks held | 98 | 25 |
| confirmed | 56 | 14 |
| strong | 30 | 11 |
| strong pairs under 1 m | 23 | **0** |
| ms/cycle median | 11.7 | 11.7 |
| ms/cycle p95 | 14.2 | 14.4 |
| ids burned | 386 | 647 |

`output/doors/corridor10m_doors.html` is that run on the map, before and
after, with the duplicate links drawn.

**What this does not fix.** 11 strong doors along one corridor is still more
than that corridor has, and the remaining ones are single hypotheses rather
than copies -- `door_7` and `door_580` face back along the corridor instead
of into a wall, which no amount of merging addresses. There is also no ground
truth in this bag: it measures duplication, not accuracy.

## 7. The grid check was not scoring the cells under AB

A confirmed strong door sat on a line whose cells were mostly occupied,
which `occupied_frac_max` 0.25 should have made impossible. Probing the
final `/slam/map` of `corridor10m_20261019T131815` under every strong door's
AB found **two separate causes**, both reproduced:

**1. The fraction was measured over the surviving runs, not over the
segment.** `check_segment` dropped every occupied run shorter than
`noise_max_m` and then computed the fraction from what was left -- so the two
tolerances multiplied instead of composing. A wall seen edge-on enters the
map as one- and two-cell specks, every speck is noise, `real` comes back
empty and the score is 0.00. Door 371 at (4.71, -1.47): **15 of 33 samples
occupied, runs of 7/4/4, score 0.21, verdict ok** -- and strong. The
fraction now counts every occupied sample; the noise allowance is kept for
the continuous-run veto, which is about shape, not quantity.

**2. A door was never re-checked after it left view.** The grid check runs
when a candidate is *seen*, and a confirmed door is deliberately kept after
it leaves view -- so a door confirmed while the space behind it was still
unknown held its strength after the map filled a solid wall in under it.
Door 350 at (1.04, 2.11) scores `blocked` on the final map, 12 consecutive
occupied samples across its opening. `recheck_tracks` now re-reads the grid
under every confirmed door whose **both jambs are inside the crop** -- the
caveat matters, because outside the crop every cell answers "not occupied"
and a segment out there scores a perfect clear. A door that fails carries
`grid_blocked`: still published, no longer drivable.

Neither was a transform or indexing error. The grid was read at the right
cells, in the right frame, with the right origin and no y flip; the sampled
cells are now drawn on the map (`/viz/doors_freespace_debug`, green free /
red occupied / grey unknown) so that can be seen rather than argued.

Two things were tightened while in there:

- **a rotated map origin is refused, not mis-indexed.** `occupancy_window`
  indexes the grid as axis-aligned, and `origin.orientation` was ignored
  outright, so a rotated map would have scored walls as free space
  silently. slam_toolbox and the ROS map servers publish identity; anything
  else now logs an error and drops the grid, which flags `no_grid`.
- **a stale grid bars strength** (`freespace_doors_strong_grid_max_age_s`).
  The threshold is 12.0 s and the number is measured, not chosen: the brief
  asked for 1 s, and with `map_update_interval: 5.0` on a latched topic that
  leaves **1 strong door of 25** and 8429 `grid_stale` flags -- the gate
  becomes the result. 6 s still demotes three doors on publish jitter alone.
  Two update intervals is the first value at which nothing is demoted by
  timing.

Measured on `corridor10m_20261019T131815`, defaults, 506 cycles:

| | before | after |
|---|---|---|
| tracks held | 98 | 25 |
| strong | 30 | **8** |
| strong pairs under 1 m | 23 | 0 |
| tracks carrying `grid_blocked` | n/a | 5 |
| `occupied_frac` rejections | 18 | 59 |
| ms/cycle median | 11.7 | 11.8 |

`recheck_tracks` and `mark_stale` live in `freespace_doors.py`, not in the
node, so `tools/freespace_door_replay.py` exercises the same code -- the
first version had them in the node and the replay measured 11 strong doors
where the stack would have published 8.

## 8. When there are no doors in Foxglove at all

`tools/doors_doctor.sh` answers this in ten checks, ordered so the first
failure is the cause: the executable in the install space, `Door.msg`'s
`strong` field, whether the installed `stack_params.yaml` is this branch, the
node in the node list, both gates (`door_source`, `freespace_doors_enabled`),
`/scan`, `base_link` -> the scan frame, the debug line, the markers, and
finally whether anything is confirmed.

The last one is the trap, and it is by design: with
`freespace_doors_confirm_same_pose_hits` 0 **a standing car never confirms a
door**, so a bench test looks exactly like a dead detector. The node now says
so itself after ten consecutive cycles with candidates and nothing confirmed,
naming how far apart its best two sightings have been. Unconfirmed candidates
are on `/viz/doors_semantic` the whole time, drawn faded.

## 9. Params to tune first

1. **`freespace_doors_confirm_pose_spread_m`** (1.0) with
   **`freespace_doors_confirm_same_pose_hits`** (0). The only knobs with real
   leverage on false positives, and they cost no recall. Already at the
   sweep's choice; lower the spread or set the fallback to 3 if doors must be
   resolvable from a standstill.
2. **`freespace_doors_strong_min_hits`** (5) and
   **`freespace_doors_strong_min_spread_m`** (1.0). What a mission is allowed
   to drive at, separately from what gets published. Tighten these rather than
   the detection gates when `enter_door` keeps picking doors you would not
   have picked.
3. **`freespace_doors_jump_min_m`** (0.50). The first and bluntest gate: below
   it nothing downstream happens. If doors are missing *entirely*, check
   `/viz/doors_freespace_debug` for yellow points at the jambs before touching
   anything else.

`freespace_doors_max_tracks` (32) is a TF budget, not a tuning knob.
