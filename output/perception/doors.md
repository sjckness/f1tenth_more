# Door detection on the multi-wall tracker (branch `door-detection`)

Branch `door-detection`, cut from `origin/campaign-final` at `868ec8a`. Target:
campaign-final on the Orin (Humble). Developed and unit-tested on linus (Jazzy).
No node was launched against real or simulated topics, and no bag was replayed.

**Scope.** Door detection in perception only, plus a fixed wall-marker height. Nothing
consumes `/perception/doors` or `/viz/doors`: not the MPC, the costmap boundaries,
`front_clearance`, the e-stop, the BT or the missions. None of those files changed.

---

## 0. Code audit (before writing)

### What the multi-wall tracker exposes per wall

`multi_wall_tracker.Track` (in-process, odom frame):

| Quantity | Where |
|---|---|
| id (never reused) | `Track.id` |
| line | `x = (rho, theta)`: `rho = x cos(theta) + y sin(theta)`, `rho >= 0`. `theta` is the normal from the odom origin toward the line. Covariance is `P` (2x2). |
| endpoints | the measured extent `t0..t1` along the tangent `(-sin, cos)`. `endpoints()` returns them in odom; `segment_in_base_link()` returns them in base_link. |
| normal / rho / d_ray | `theta` / `rho` (odom). `front_distances()` gives `d_perp_front` and `d_ray_front` from the bumper; `strip_distance()` is the FRONT distance. |
| side class | `role` (front/back/left/right/other, sticky, with hysteresis). `geometric_role()` decides it. |
| source | `source`: LIDAR / STEREO / BOTH (the best evidence the track has ever had) |
| state | `state` (TENTATIVE / CONFIRMED / COASTING) and `ever_confirmed` |
| coasting age | `last_seen` (stamp of the last association); the age is `tracker._last_stamp - last_seen`. Also `odo_at_last_seen` for metres coasted. |
| glass | `glass` (set by the rectangle verification), plus `verify` and `rect_*` |
| other | `occluded`, `contradicted`, `miss` (negative evidence), `strip_verdict`, `z_min/z_max/height_measured`, `confidence` |

`/perception/walls` (`MultiWallTrack`) carries id, label/role, state, rho/theta (odom
and base_link), p0/p1 (odom and base_link), cov, hits, age, d_perp_front, d_ray_front,
source, confidence, z_min/z_max/height_measured, d_side and is_front. Glass, verify,
occluded, contradicted and the coasting age are not on that message. They appear only
in `/perception/virtual_wall/debug` → `tracks`, and only when `wall_debug_tracks` is on.

### Where the LiDAR scan and YOLO detections reach the tracker

Both arrive in `lidar_front_wall_node._track_walls`, once per tracker cycle (one scan
in `wall_tracker_scan_decimation` = 4, about 10 Hz).

* **Scan.** `mwt.scan_in_base_link(..., sectors, with_cut=True)` returns
  `(ranges, base_link bearings, laser xy, cut)`. Beams cut for a detection become `inf`,
  and the `cut` mask tells them apart from no-returns. The tracker gets exactly this
  tuple, so the door detector uses the same beams.
* **YOLO.** `/camera/detections_3d` → `_detections_cb` stores `(x, y, half width)` in
  base_link and the header stamp. `_sectors()` carries each detection to the scan stamp
  on `/odometry/filtered` (`PoseHistory`, `propagate_objects`) and cuts its bearing
  sector. **The class id was discarded.** It is now kept in `_detection_classes`, in the
  same order, because a person needs to be told apart from other objects.

### Floor, inferred floor and wall masks

`front_clearance_node` publishes `/perception/scene_labels/classes` (mono8
`scene_mask.CLASS_*`: FLOOR 3, WALL 6, FLOOR_INFERRED 7, OBJECT 2, …) together with
`/perception/scene_labels/depth_mm`. Both carry one stamp. `lidar_front_wall_node`
subscribes to them only when `wall_rect_verify` is on (the default). `_mask_pair()`
returns the pair, with intrinsics from `/perception/stereo_columns` and the camera
transform from `/tf_static`. Until now the only user was the rectangle verification
(`wall_fusion.verify_rectangles`). The door's optional stereo support uses the same
pair, the same intrinsics and the same transform.

### Wall markers, and why their height varied

There are two builders:

1. **`/perception/walls/markers`** (`_publish_walls`). Each published track is drawn as a
   CUBE slab from `trk.z_min` to `trk.z_max`. If the stereo measured the height
   (`height_measured`), that is the height of the agreeing columns. If not, it is
   `wall_default_height_m` (2.5 m).
2. **`/viz/walls`** (`wall_markers.WallMarkerBuilder`). Each track is drawn as a prism
   using `z_span()`: the measured `(z_min, z_max)` if `height_measured`, otherwise
   `DEFAULT_Z_SPAN = (0, 2.0)`.

So heights differed **between walls**, depending on what the stereo reported for each
one, and **between the two topics** for the same unmeasured wall (2.5 m on one, 2.0 m on
the other).

**Fix (visualisation only).** Both builders now draw every wall from the floor to
`wall_viz_height_m` (2.5 m). `Track.z_min/z_max` and `MultiWallTrack.z_min/z_max` are
unchanged. On `/perception/walls/markers` the slab's alpha still shows whether the height
was measured (0.65) or assumed (0.35).

### DELETE / DELETEALL audit

Both wall topics already send a per-id DELETE for every dead track and give each marker
a lifetime (0.6 s). They deliberately send no DELETEALL, because sending one every cycle
was what made the walls flicker. I left that as it is. `/viz/doors` uses the same scheme:
a DELETE on each of a dead door's 4 slots, a lifetime on every marker, and no DELETEALL.
A stalled node or a restart therefore fades out after 0.6 s instead of leaving ghost
markers.

---

## 1. What a door candidate is (`door_detection.door_candidates`)

The extraction's collinear merge (`wall_merge_max_gap_m`, 1.5 m) reunifies the two faces
either side of a 0.7–1.0 m doorway into **one** observation. So a doorway is usually
*inside* one track's extent, not between two tracks. Two tracks either side of a doorway
also happen, when the tracker's own collinear gap keeps them apart. The detector handles
both cases the same way:

* Tracks that are confirmed or coasting are grouped when they lie on one line
  (collinear) and are at most a doorway apart.
* For each group, every beam that crosses the line inside the group's extent is
  classified against its wall's line:

| Class | Meaning |
|---|---|
| ON | within `wall_tol` = the tracker's `wall_negative_tol_m` (0.15 m) |
| BEYOND | at least `door_behind_margin_m` past the line |
| NONE | no return |
| SHORT | nearer than the line |
| SHALLOW | past the line, but by less than the margin |
| CUT | cut for a YOLO detection |

* **Pieces of wall** are runs of ON beams at least 0.10 m long. A **gap** is the stretch
  between two consecutive pieces. A **jamb** is the last ON return of a piece, projected
  onto its wall's line.

A gap is a door candidate only if all of these hold. The table lists the reject code for
each check, in the order the checks run:

| # | Rule | Reject code |
|---|---|---|
| 1 | Each jamb lies on a confirmed or coasting track (within 0.3 m of its measured extent) | `untracked_jamb` |
| 1 | If the jambs are on two different tracks, those tracks are collinear within `door_collinear_angle_deg` and `door_collinear_offset_m`, measured at the facing ends | `not_collinear` |
| 2 | Width (jamb to jamb) is within [`door_width_min_m`, `door_width_max_m`] | `width` |
| 3 | No ON return inside the gap (a pane or dark panel the LiDAR saw) | `on_line` |
| 4 | No SHORT return in the first or last 2 beams of the gap (a shadow edge, not a jamb) | `jamb_occluded` |
| 3 | No SHORT return elsewhere in the gap | `blocked` |
| 5 | No CUT beam in the gap: a detected object never makes a door | `object` |
| 3 | No return between 0.15 m and the margin past the line | `shallow` |
| 3 | Open beams (BEYOND + NONE) ≥ `max(door_min_beams, ceil(door_min_beam_fraction × beams the gap's angular width promises))` | `few_beams` |

Rejections go to `/perception/virtual_wall/debug` as `door_rejects`
(`[code, [ids], width]`, at most 16), next to `door_ms` and `doors`.

**6 — Stereo, optional** (`stereo_gap_support`). The detector samples the gap's
rectangle (0.3–1.8 m above the floor) and the floor 0.3–1.2 m beyond the line in the
class mask and depth. Support means: at most 20 % WALL pixels at the line's depth, and
at least 50 % FLOOR or FLOOR_INFERRED beyond the line. Support adds 0.2 to the confidence
(LiDAR alone gives 0.5–0.8). It is never required. If there is no mask pair, if the gap
is outside the image, or if the stereo is out of range, the result is "no opinion".

Closed doors and double doors are out of scope. A closed door is wall to the LiDAR, and
a double door fails the width check.

## 2. Tracking (`DoorTracker`)

* **Lifecycle:** tentative → confirmed (`door_confirm_hits` of `door_confirm_window`
  cycles) → coasting (unseen) → deleted after `door_delete_after_sec` unseen. A tentative
  door that cannot confirm within one window is dropped.
* **Identity:** doors are associated by centre (≤ 0.30 m), normal (≤ 15°, unoriented)
  and width (≤ 0.20 m) in the tracker's fixed frame, using greedy lowest-cost matching.
  Association **never** uses the parent wall ids, which are only reported. Jambs are
  smoothed with a running average (0.3). The normal stays perpendicular to the jamb line,
  with its sign fixed at birth: from the side the car first saw the door.
* **Occupied:** a person (`class_id == 'person'`) carried to the scan stamp, standing
  within the door's width (± their half width) and within 0.6 m of its line. This applies
  only to a confirmed door: it sets `occupied`, keeps the door alive and keeps its state
  CONFIRMED. It does not count as a sighting. A person in a gap that is not yet a door,
  or in a tentative door, does nothing.
* **Per door:** id, centre, normal, width, two jambs (odom and base_link), parent wall
  ids, state, occupied, confidence, stereo_support, hits, age, since_seen (the coasting
  age) and n_beams.

## 3. Parameters (stack_params.yaml, `lidar_front_wall_*`)

| Parameter | Default | Meaning |
|---|---|---|
| `door_detection_enabled` | `true` | Run door detection and publish `/perception/doors` + `/viz/doors`. Needs `use_wall_tracker`. |
| `door_collinear_angle_deg` | 5.0 | Two walls are one line within this angle… |
| `door_collinear_offset_m` | 0.10 | …and this offset at their facing ends |
| `door_width_min_m` | 0.70 | Narrowest doorway, jamb to jamb |
| `door_width_max_m` | 1.00 | Widest doorway |
| `door_behind_margin_m` | 0.30 | Every beam through the gap must return at least this far past the line, or not at all |
| `door_min_beams` | 8 | Floor on the open beams; below it the gap is undecidable |
| `door_min_beam_fraction` | 0.8 | The range scaling: this share of the beams the gap's angular width promises at its range |
| `door_confirm_hits` | 3 | Hits to confirm… |
| `door_confirm_window` | 5 | …within this many tracker cycles |
| `door_delete_after_sec` | 2.0 | Unseen this long → deleted (an occupied confirmed door is held) |
| `door_jamb_viz_diameter_m` | 0.10 | Jamb cylinder diameter on `/viz/doors` |
| `wall_viz_height_m` | 2.5 | Drawn height of every wall marker and every door jamb, from the floor |

These values are fixed in `door_detection.DoorParams` and are not parameters:

| Value | Default |
|---|---|
| wall tolerance | the tracker's `wall_negative_tol_m`, 0.15 m |
| minimum piece length | 0.10 m |
| jamb-to-extent tolerance | 0.30 m |
| edge beams | 2 |
| association | 0.30 m / 15° / 0.20 m |
| smoothing | 0.3 |
| person depth | 0.6 m |
| stereo thresholds | 0.2 wall / 0.5 floor / 0.25 coverage, 8 m range, +0.2 confidence |

At the Hokuyo's 0.25° spacing, a 0.85 m gap seen square-on has 8 beams at 24 m, and at
12 m when seen at 60° incidence. Near the car the 0.8 fraction dominates: a gap 3 m ahead
needs about 55 open beams.

## 4. Message definition (f1tenth_messages, new; no existing message changed)

`msg/Door.msg`:

```
uint32 id                       # never reused; NOT tied to the parent wall ids
uint8 state
uint8 STATE_TENTATIVE = 0
uint8 STATE_CONFIRMED = 1       # seen this cycle, or held by a person in it
uint8 STATE_COASTING = 2
geometry_msgs/Point center      # odom (DoorArray.header.frame_id), z = 0
geometry_msgs/Vector3 normal    # unit, crossing direction
float32 width
geometry_msgs/Point jamb0
geometry_msgs/Point jamb1
geometry_msgs/Point center_bl   # base_link at the scan stamp
geometry_msgs/Point jamb0_bl
geometry_msgs/Point jamb1_bl
uint32 parent_wall_id0          # MultiWallTrack ids at jamb0 / jamb1 (equal when
uint32 parent_wall_id1          # the doorway lies inside one track); informational
bool occupied
float32 confidence              # [0, 1]
bool stereo_support
uint32 hits
float32 age                     # [s]
float32 since_seen              # [s] the coasting age
uint16 n_beams
```

`msg/DoorArray.msg`: `std_msgs/Header header` + `Door[] doors`. The array is published
on `/perception/doors` on every tracker cycle, including when it is empty. Every live
door is in it, tentative and coasting included. `header.frame_id` is the odom frame and
`header.stamp` is the scan stamp.

## 5. Seeing it in Foxglove

* Open a **3D panel** with the display frame set to `odom` and enable `/viz/doors`.
  Enable `/viz/walls` or `/perception/walls/markers` too, to see the walls the doors are
  cut from. `/viz/doors` crosses the bridge without a whitelist change, because
  `'^/viz/.*'` already matches it. It is built only while something is subscribed.
* **Each door** is drawn as two vertical cylinders (one per jamb, 0.10 m diameter, 2.5 m
  tall, standing on the floor), a line between them on the floor, and the label
  `door <id> <width>m <state>[ OCCUPIED]` above.
  * Colour: green for an open door, orange when a person is standing in it.
  * Alpha shows the state: tentative 0.3 (faded), confirmed 1.0 (solid), coasting 0.5
    and darker (dimmed).
* **Walls** are all 2.5 m tall, standing on the floor.
* `/perception/doors` is not on the bridge allowlist. Use `foxglove_raw_debug:=true` to
  see it raw, or `ros2 topic echo /perception/doors` on the car.
* **Replay:** `tools/corridor_wall_replay.py` captures `/perception/doors` and its
  top-down video draws each door as a dot per jamb plus the threshold line, with the same
  label and colours.

## 6. Humble / Jazzy notes

* No Jazzy-only API. The node uses the same rclpy surface as before:
  `create_publisher` and `Publisher.get_subscription_count()`, both available since Foxy.
* The messages use only Humble rosidl features: constants with inline comments (as
  `MultiWallTrack.msg` already does), `geometry_msgs/Point` and `Vector3`, and an
  unbounded array of a package-local message (`Door[]`).
* `vision_msgs` `results[0].hypothesis.class_id` is the Humble vision_msgs 4.x layout,
  the same one `detection_3d_node` writes.
* No new dependency. Python is 3.10-safe (no 3.11+ syntax). numpy usage is 1.21-safe.
* **Frame.** "Map frame" in the request was implemented as the tracker's fixed frame,
  **odom**, which every wall track lives in. This node deliberately has no `/tf`
  subscription (it reads `/tf_static` only, to avoid deserialising every dynamic
  transform in Python), so it cannot place anything in `map`. A door lives for seconds
  past its last sighting, so odom drift over its lifetime is small. If doors are ever
  wanted in `map`, the consumer should transform them; this node should not.
* **Pre-existing build issue on linus.** `corridor_perception_c` does not compile with
  gcc 13 on Jazzy (`M_PI` undeclared under the package's C standard). It was skipped on
  linus, so the tracker ran on the Python extraction backend, which is the reference
  implementation. This is unrelated to this branch and does not affect the Orin build.

## 7. Tests (synthetic scans, no stack)

The `f1tenth_perception` suite went from 1013 tests (HEAD) to 1072. The only 2 failures
are the same in both runs: `colcon test` isolates the package's environment, so
`f1tenth_bringup` is not found (`test_*_launch.py::TestPlacement`). Both pass when run in
the full workspace environment. Across the 16 packages built up to `f1tenth_perception`
and `f1tenth_bringup`, the set of failing tests is identical before and after the change.
Those pre-existing failures on linus are mostly `mpc_controller` and `llm`.
`tools/test_corridor_wall_replay_helpers.py`: 13 passed.

| Requested test | Where |
|---|---|
| 1. 0.85 m gap, returns 3 m behind → door (inside one track, and between two) | `test_door_detection.py::TestTheDoorway::test_a_0_85_m_gap_seen_through_to_3_m_behind_is_a_door` |
| 2. 0.5 m and 1.4 m → no door | `TestTheDoorway::test_a_gap_of_the_wrong_width_is_not_a_door` |
| 3. Return on the wall line inside the gap → no door | `TestTheBeamsThroughTheGap::test_sparse_returns_on_the_wall_line_inside_the_gap_are_not_a_door`, `test_a_pane_the_lidar_sees_across_the_gap_is_wall`, `test_a_return_just_past_the_line_is_too_shallow_to_call` |
| 4. Box in front of the wall → no door | `test_a_box_in_front_of_the_wall_casts_a_gap_that_is_not_a_door`, `test_a_real_opening_with_one_jamb_hidden_by_a_box_is_not_a_door` |
| 5. Too few beams (far) → no candidate | `test_too_few_beams_through_a_far_gap_is_undecidable` (+ the range-scaling test) |
| 6. Person in confirmed door → kept, occupied; person in non-door gap → no door | `TestObjects` (4 tests) |
| 7. Parent wall re-birth → same door id | `TestIdentity::test_a_reborn_parent_wall_keeps_the_same_door_id` |
| 8. Corner → no door | `TestTheWalls::test_a_corner_is_not_a_door` (+ offset-parallel, tentative-wall, untracked-jamb) |
| 9. Wall markers 2.5 m regardless of input; dead doors → DELETE | `test_wall_markers.py::TestGeometry` (parametrised over measured heights), `test_lidar_front_wall_node.py::TestFixedWallHeight`, `test_door_markers.py::TestDeletion`, `TestDoors::test_a_door_that_dies_is_deleted_from_viz_doors` |
| End to end | `TestThroughTheRealTracker` (real extraction and `MultiWallTracker`), `test_lidar_front_wall_node.py::TestDoors` (the node, on synthetic `LaserScan`s) |

Two `test_wall_markers.py` tests encoded the old varying height: 2.0 m when assumed, the
measured span otherwise. They were replaced in the same commit by tests named for the new
behaviour.

**Lint (ament_flake8 / ament_pep257, per file, HEAD → branch).** No changed file got
worse:

| File | HEAD | Branch |
|---|---|---|
| `lidar_front_wall_node.py` | 17 / 23 | 17 / 23 |
| `wall_markers.py` | 1 / 9 | 1 / 9 |
| `test_lidar_front_wall_node.py` | 51 / 81 | 51 / 81 |
| `test_wall_markers.py` | 2 / 2 | 2 / 2 |
| `lidar_front_wall.launch.py` | 1 / 1 | 1 / 1 |
| `tools/corridor_wall_replay.py` | 33 / 44 | 33 / 44 |

The new files (`door_detection.py`, `door_markers.py`, `test_door_detection.py`,
`test_door_markers.py`) are 0 / 0.

**Cost.** `door_candidates` takes 3.7 ms per tracker cycle on linus with 12 wall tracks.
Expect a few times that on the Orin. It is reported every cycle as `door_ms` on
`/perception/virtual_wall/debug`, and `door_detection_enabled:=false` turns it off
completely.

## 8. Known limits, deviations, next steps

* **An open door leaf seen near its hinge** returns 0.15–0.30 m past the line, so the gap
  is rejected as `shallow`. This follows the spec ("every beam ≥ margin"). If real bags
  show it often, the fix is to treat SHALLOW beams *contiguous with a jamb* as jamb
  geometry (the reveal or the leaf), not to lower the margin.
* **Glass the LiDAR does not see at all** reads as an opening (all NONE). The stereo
  cannot tell either, because it also sees through glass. Real doorways almost always
  return the next room (BEYOND), so requiring some BEYOND returns is a possible later
  gate.
* `/perception/doors` is **not recorded** by the mission logger. The logger is outside
  "perception only". Adding it to the campaign profile is a one-line follow-up whenever
  bags with doors are wanted.
* **Not validated on recorded data.** Only synthetic scans were allowed here. The
  obvious first check is a replay of `corridor10m_20261016T144537` (the corridor mouth
  behind track 955) with `tools/corridor_wall_replay.py`.

## 9. Importing on the Orin

On the Orin, in the workspace (it is on campaign-final; check for local changes first):

```bash
cd ~/dev_ws/f1tenth_more
git status --short                      # must be clean (or commit / stash by path first)
git fetch origin door-detection
git switch -c door-detection --track origin/door-detection
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select f1tenth_messages f1tenth_params f1tenth_perception
source install/setup.bash
ros2 interface show f1tenth_messages/msg/DoorArray     # sanity: the new message is there
```

Then restart the `lidar_front_wall` supervisor component, which picks up the new
parameters from stack_params.yaml. Watch `/viz/doors` in Foxglove. To go back:
`git switch campaign-final`, then rebuild the same three packages.
