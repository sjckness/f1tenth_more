# Door detection on the multi-wall tracker (branch `door-fixes`)

Branch `door-fixes`, cut on 2026-10-19 from `origin/door-detection` (`c873247`) for
the live test of 2026-10-20. Target: the Orin (ROS 2 Humble), merged onto
`campaign-final-doors` (which must contain `904d86ed`). Developed and unit-tested
on linus (Jazzy); no node was launched against real or simulated topics. The
offline check on `corridor10m_20261016T144537` is in
[doors_phase1.md](doors_phase1.md).

**Scope: perception only.** Nothing consumes `/perception/doors` or `/viz/doors`:
not the MPC, the costmap boundaries, `front_clearance`, the e-stop, the BT or the
missions.

**THE OVERLAY RULE.** Doors are an overlay on the walls. Door detection never
modifies, splits or removes a wall track or any LiDAR data, and nothing may treat
a doorway's gap as free space because a door exists there.

What this branch adds on top of `origin/door-detection` (steps 1-3, §10):

| Commit | What |
|---|---|
| `1b2155d` | multi_wall_tracker: THE WALL'S SPAN -- a narrow wall first seen behind a detected person is born again (a regression of point 1, found by tonight's audit; §9) |
| `a74a628`, `1a6c7cc`, `d1fc7cc` | merge-rule tests: the code default pinned to the yaml, the 0.70 m boundary made honest, both extraction backends at 0.70 m |
| `0ee22d9` | tools/door_replay.py: the car's cadence and params, `--src`, `--walls retrack-*`, candidates CSV, door images (§11) |
| `1148636` | stack_params descriptions for THE WALL'S SPAN |
| `c07e7d8` | doors_phase1.md, part 1: the user's item 5 |
| `d811e58` | the wall role `other` renamed `unknown`; tentative tracks reported `unknown` (§4) |
| `ed6899a` | the addendum's door core: hosts, Manhattan prior, perpendicular gap tests, code classes, conservative lifecycle (§1, §2) |
| `95d3a77` | f1tenth_messages: `Door.jamb_height_unknown`, `STATE_CANDIDATE`; `/perception/doors` carries confirmed doors (§4) |
| `a49adca` | the node, markers, params and launch on the addendum's core: confirmed-only `/perception/doors`, the lazy ego-compensated stereo frame, the guarded cycle (§2, §3, §5) |
| `9df1976`, `affd305` | tools/door_replay.py on the addendum's core: every `DoorCandidate` a row, strikes, the Manhattan prior, `--summary-md`; a candidate drawn dashed and faded (§11) |
| `148d58e` | optional front-facing hosts (`door_host_front_facing`, default off; §1.1) |

---

## 1. What a door is (`door_detection.door_candidates`)

An OPEN doorway: a gap between two stretches of one confirmed wall line that the
LiDAR sees through. `door_candidates` evaluates every such gap of the cycle and
returns **all** of them, each with the code of the first test it failed (`''`
when it passed them all: a positive sighting).

### 1.1 Hosts

A **host** is a wall track that is **CONFIRMED** (not tentative, not coasting)
with role **front, left or right**. Only hosts form the collinear groups the gaps
are searched along; nothing else is ever a jamb. An **unknown** segment -- a
tentative track, a track with role `unknown`/`back`, a scan fragment no track
holds -- never hosts a door and is never evidence for or against one.

**Manhattan prior** (`door_manhattan_prior`, default on). Rooms are rectangular.
`ManhattanPrior` keeps the room's dominant direction: a length-weighted
histogram of the confirmed walls of at least 1.5 m (normal angle mod 90°, 2°
bins, circular), its peak refined by the weighted circular mean of the walls
within `door_manhattan_tol_deg` of it, smoothed over time (EMA with a 2 s time
constant on the 4-fold circle). It is reported every cycle on the debug topic
(`door_manhattan`). A host more than 10° off it is `off_manhattan`; the others
are **snapped** before the beam profile is built: copies of the group's lines
are rotated about their own midpoints onto the dominant axis, so a wall the
tracker fitted a few degrees off is judged on the room's line. The tracks
themselves are never touched. Long stable off-axis walls stay walls; they simply
cannot host doors. No dominant direction yet (no confirmed wall ≥ 1.5 m): no
check and no snap.

**Line and sides.** `short_line`: the union of the group's members along the
line (less this gap when it lies inside one member) is shorter than 2.0 m.
`short_side`: a jamb lies less than 0.5 m from the far end of the member it is
on.

### 1.2 The gap, on the beams

Since the collinear gap rule (§9) the tracker joins collinear pieces only across
gaps shorter than `door_width_min_m`, so a doorway normally lies **between** two
host tracks; the search still runs on the beams inside a track's extent as a
fallback (the in-track case). Along the group's (snapped) line every beam is
classified by the **perpendicular** depth of its return behind the line -- a
return `r` on a ray that meets the line at range `s` with incidence `cos φ` lies
`(r − s)·cos φ` behind it -- never by the distance along the ray:

| Class | Perpendicular depth behind the line |
|---|---|
| ON | from 0.15 m in front (the tracker's `wall_negative_tol_m`) to 0.05 m behind |
| SHALLOW | 0.05 m to `door_behind_margin_m` (1.0 m) |
| BEYOND | at least `door_behind_margin_m` |
| NONE | no return (and not cut) |
| SHORT | more than 0.15 m in front: something in front of the line |
| CUT | cut out for a YOLO detection: no opinion |

Pieces of wall are runs of ON returns at least 0.10 m long that lie on a host's
extent; a gap is the stretch between two consecutive pieces; a jamb is the foot
of a piece's last return on its member's line.

### 1.3 The codes

In the order they are checked. **Only a STRONG code on a VISIBLE gap is evidence
against a door**; WEAK and GEOMETRY codes, and STRONG ones on a gap that is not
visible, change nothing (§2).

| Class | Code | Meaning |
|---|---|---|
| GEOMETRY | `untracked_jamb` | a jamb more than 0.30 m off its host's measured extent |
| GEOMETRY | `not_collinear` | two hosts meet at a door-wide gap but are not one line (a corner) |
| GEOMETRY | `width` | jamb to jamb outside [`door_width_min_m`, `door_width_max_m`] = [0.70, 1.00] m |
| STRONG | `off_manhattan` | the host is more than `door_manhattan_tol_deg` (10°) off the room's dominant direction |
| STRONG | `short_line` | total collinear extent of the host line < `door_min_line_extent_m` (2.0 m) |
| STRONG | `short_side` | a side of the gap < `door_min_side_len_m` (0.5 m) |
| WEAK | `oblique_view` | the ray to the gap's centre is more than `door_max_view_angle_deg` (45°) off the normal |
| WEAK | `jamb_occluded` | a SHORT return among the first/last 2 beams of the gap: the wall end is a shadow edge |
| WEAK | `unknown_in_gap` | something unknown in the gap; `detail`: `blocked` (a SHORT return in it), `object` (a beam cut for a YOLO detection), `unknown_on_line` (ON returns in a gap BETWEEN two hosts: no host claims that stretch, so whatever returned there -- a closed leaf, a pane seen square-on -- is unknown) |
| WEAK | `few_beams` | fewer decided beams than required (§1.5), or an open leaf covers the whole core |
| STRONG | `on_line` | at least `door_on_line_min_beams` (2) ON returns in a gap INSIDE one host's extent: the tracker holds wall there |
| STRONG | `not_a_leaf` | shallow returns in an outer part that are neither an open leaf nor a reveal (§1.4) |
| STRONG | `behind` | in the central `door_core_fraction` (0.6), fewer than `door_behind_frac` (70 %) of the beams not on the leaf return ≥ `door_behind_margin_m` (1.0 m) behind the line, or not at all |
| STRONG | `not_boundary` | along both host segments (from the jamb to the member's far end, 0.10 m clear of both ends), more than `door_boundary_max_hits` (2) returns lie > `door_boundary_behind_min_m` (0.30 m) behind the line, not counting returns on another confirmed wall perpendicular to the host: the host is see-through (a glass partition's panes) |
| STRONG | `stereo_wall` | (stereo, `door_stereo_glass_veto`, default **off**) WALL pixels with valid depth ON the line over ≥ 50 % of the gap: a pane the LiDAR saw through. Mostly invalid depth: kept, `glass_suspect` |
| STRONG | `furniture` | (stereo, `door_jamb_height_check`, default on) a jamb's wall surface ends below `door_furniture_max_height_m` (1.3 m) while the camera saw above it (§1.6) |

`visible` = view within 45° and no `jamb_occluded`, no `unknown_in_gap`, at
least the required decided beams -- computed for every non-GEOMETRY gap, whatever
its code.

### 1.4 The leaf and the reveal (point 2)

Shallow returns in the **outer** parts of the gap (the 20 % next to each jamb),
plus ON-band returns past the line in an outer part that already holds shallow
ones (a leaf's returns straddle the ON band next to its hinge), are judged per
jamb side, grown with the shallow returns that continue contiguously from that
side into the core:

* **The reveal**: all within 0.05 m of the jamb along the line and at most
  `door_reveal_max_depth_m` (0.20 m) deep -- the jamb's own side face. Allowed at
  both jambs. Two faces perpendicular to the line at both jambs, deeper than that,
  are a thick wall's two reveals: a door with no leaf.
* **The leaf**: one straight segment (rms ≤ 3 cm, at least two returns) whose line
  passes within `door_leaf_hinge_tol_m` (0.10 m) of the jamb, which **starts at
  the jamb** (its near end within 0.10 m of it, no open beam between) and is at
  most `door_leaf_max_len_m` (1.0 m) long.
* Anything else -- shallow returns at both jambs, a box, a pole, a segment
  detached from the jamb -- is `not_a_leaf`.

Leaf and reveal returns count as decided beams and are **left out of the core**
when its through-share is taken, so a leaf opened 45° (which projects ~0.6 m
along a 0.85 m doorway, deep into the core) is a door. `Door.leaf_side` says
which jamb, left or right looking along the crossing normal, per sighting
(`NONE` also means "no leaf visible from here"), mirrored for a sighting from the
other side.

### 1.5 Beams

`n_required = max(door_min_beams, ceil(door_min_beam_fraction × n_expected))`,
where `n_expected` is the gap's angular width over the scan's angle increment.
Decided beams return beyond, shallow (leaf and reveal included) or not at all.
At 0.25° a 0.85 m gap seen square-on has 8 beams at 24 m; a gap 3 m ahead needs
about 52.

### 1.6 Stereo

Only for a gap that passed everything above, and only when the node has a class
mask / depth pair (`wall_rect_verify` and `enable_scene_labels` on, a
`StereoColumns` message, the camera TF). The node builds the camera frame lazily,
at most once per cycle, **carried from the mask's stamp to the scan's stamp on
odometry** (the pair is up to 0.6 s older than the scan); with no pose for one of
the stamps and a mask older than 0.1 s it gives no stereo opinion that cycle.

* **Jamb height** (`stereo_jamb_heights`): samples on the wall plane 0.05-0.30 m
  outward from each jamb, 0.1-2.4 m above the floor. A side is decided only with
  valid depth and WALL pixels on the plane. Both sides' wall reaching
  `door_jamb_min_height_m` (1.8 m): tall jambs. A side whose wall ends below
  1.3 m while the camera saw at least 0.3 m higher there (and it was not wall):
  `furniture`. No valid depth, no wall pixels, or a clipped view (the camera
  never saw up to 1.8 m there): no decision, `jamb_height_unknown`.
* **Glass veto** (`door_stereo_glass_veto`, default **off**): as in step 2 (§10.3).
  When the camera does not cover the gap the result is "no opinion" and an earlier
  `glass_suspect` is kept.
* **Support** (`stereo_gap_support`): +0.2 confidence when no WALL pixel sits at
  the line's depth in the gap and the floor carries on beyond it. Never required.

Closed doors are out of scope (wall to the LiDAR, or `unknown_in_gap`); double
doors fail the width.

## 2. Lifecycle (`DoorTracker`) and what is published

* **States: CANDIDATE → CONFIRMED.** A positive sighting associates with a door
  (centre ≤ 0.30 m, normal ≤ 15°, width ≤ 0.20 m, cheapest first) or spawns a
  CANDIDATE. Each positive adds the car's position (odom) and the observed width
  to the door's evidence (the last `door_confirm_window` = 20).
* **Confirm** only when the evidence holds positive, decidable sightings from at
  least `door_min_viewpoints` (2) car positions `door_viewpoint_sep_m` (0.3 m)
  apart (greedy clusters) **and** the widths' standard deviation is below
  `door_width_std_max_m` (0.05 m). A car standing still never confirms a door.
  (`door_square_looks` > 0 adds step 3's square-on looks as an extra requirement;
  default 0.)
* **Strikes.** A STRONG code on a VISIBLE gap that associates with a door not seen
  positive in the same cycle strikes it: evidence cleared, CONFIRMED back to
  CANDIDATE; a second strike deletes it. WEAK and GEOMETRY records, and STRONG
  ones that are not visible, have no effect at all.
* **Ages.** A CANDIDATE with no positive sighting for
  `door_candidate_max_age_sec` (10 s), a CONFIRMED door with none for
  `door_confirmed_max_age_sec` (60 s), is dropped -- unless a person detected
  inside a CONFIRMED door holds it (`occupied`).
* **Identity** never depends on the wall track ids (they re-spawn); the parent ids
  are only reported. Jambs are smoothed (running average 0.3); the normal keeps
  the sign it had at birth.

**Published** (`lidar_front_wall_node`, on every tracker cycle while door
detection is on):

* **`/perception/doors`** (`f1tenth_messages/DoorArray`, frame `odom`): the
  CONFIRMED doors only -- the CANDIDATES too with `door_publish_candidates`
  (`Door.state` tells them apart). A reader of the topic therefore never sees a
  door the lifecycle has not confirmed unless it asks for them. Each `Door`
  carries its odom pose and a base_link copy at the scan stamp.
* **`/viz/doors`** (`visualization_msgs/MarkerArray`): every live door, CONFIRMED
  and CANDIDATE alike, whatever `door_publish_candidates` says, on every cycle
  (subscribed or not, §5).
* Both go out on **every** door cycle, the empty array included, so a stall or a
  restart clears within the markers' lifetime.
* The per-gap diagnostics are on `/perception/virtual_wall/debug` (§5), not on
  either published topic.

## 3. Parameters (stack_params.yaml, `lidar_front_wall_*`)

All prefixed `lidar_front_wall_`. Every default below is also the node's
fallback and `DoorParams`' default; the three are pinned together by
`test_door_params_defaults_are_the_addendum_table` and the node coupling tests,
and the yaml carries the full prose for each. Degrees in the parameter, radians
in the `*_rad` field.

| Parameter | Default | What |
|---|---|---|
| `door_detection_enabled` | `true` | run the door cycle after the walls are published |
| `door_publish_candidates` | `false` | also publish CANDIDATE doors on `/perception/doors` |
| `door_collinear_angle_deg` | `5.0` | two hosts are one line within this angle |
| `door_collinear_offset_m` | `0.10` | ...and this perpendicular offset at the facing ends |
| `door_width_min_m` | `0.70` | narrowest doorway -- **and** THE COLLINEAR GAP RULE (§9) |
| `door_width_max_m` | `1.00` | widest doorway |
| `door_manhattan_prior` | `true` | keep the room's dominant direction, snap hosts onto it (§1.1) |
| `door_manhattan_tol_deg` | `10.0` | host off the axis by more → `off_manhattan`; peak-refine window |
| `door_manhattan_min_wall_len_m` | `1.5` | shortest confirmed wall that votes for the direction |
| `door_manhattan_tau_sec` | `2.0` | time constant of the direction's EMA |
| `door_min_line_extent_m` | `2.0` | `short_line` below this collinear host extent |
| `door_min_side_len_m` | `0.5` | `short_side` below this from a jamb to its track's far end |
| `door_host_front_facing` | `false` | optional: a doorway straight ahead may host (§1.1, `148d58e`) |
| `door_min_beams` | `8` | floor on the decided beams |
| `door_min_beam_fraction` | `0.8` | ...as a share of the beams the width promises at range |
| `door_max_view_angle_deg` | `45.0` | `oblique_view` beyond this off the normal |
| `door_behind_margin_m` | `1.0` | perpendicular depth from which a return is THROUGH (was 0.30) |
| `door_core_fraction` | `0.6` | the central share the `behind` test reads |
| `door_behind_frac` | `0.7` | `behind` below this share of the core through |
| `door_on_line_min_beams` | `2` | `on_line` at or above this many ON returns in an in-track gap |
| `door_boundary_behind_min_m` | `0.30` | depth a boundary hit must exceed |
| `door_boundary_max_hits` | `2` | `not_boundary` above this many hits over the host segments |
| `door_boundary_end_margin_m` | `0.10` | clear of each segment end for the boundary test |
| `door_leaf_max_len_m` | `1.0` | longest open leaf |
| `door_leaf_hinge_tol_m` | `0.10` | how near the jamb a leaf's line and near end must pass |
| `door_reveal_max_depth_m` | `0.20` | deepest a jamb's own side face may be |
| `door_stereo_glass_veto` | `false` | **off**: the stereo glass veto (`stereo_wall`) |
| `door_stereo_plane_tol_m` | `0.30` | a WALL pixel within this of the line counts for the veto |
| `door_stereo_wall_veto_frac` | `0.5` | veto above this share of the gap's valid samples on the line |
| `door_jamb_height_check` | `true` | on: the stereo furniture check (`furniture`, §1.6) |
| `door_jamb_min_height_m` | `1.8` | wall height that makes a jamb tall |
| `door_furniture_max_height_m` | `1.3` | wall ending below this, room above → `furniture` |
| `door_min_viewpoints` | `2` | car positions needed to confirm |
| `door_viewpoint_sep_m` | `0.3` | how far apart they count as separate |
| `door_width_std_max_m` | `0.05` | widths' std below this to confirm |
| `door_confirm_window` | `20` | the evidence window (last N positive sightings) |
| `door_candidate_max_age_sec` | `10.0` | drop a CANDIDATE unseen this long |
| `door_confirmed_max_age_sec` | `60.0` | drop a CONFIRMED door unseen this long |
| `door_square_looks` | `0` | optional extra: square-on looks to confirm (was 1, §8) |
| `door_square_incidence_deg` | `3.0` | how near the normal a beam counts for a square look (was 2.0) |
| `door_square_min_beams` | `8` | square-on beams a look needs |
| `door_jamb_viz_diameter_m` | `0.10` | jamb cylinder diameter on `/viz/doors` |

## 4. Messages

`f1tenth_messages/Door` and `DoorArray` (`95d3a77`, step 1 `3034c94`). The array
header carries the frame (`odom`) and the scan stamp the `*_bl` copies are in.

Each `Door`: `id` (never reused, **not** the wall track ids); `state`; `center`,
`normal` (the crossing direction, sign fixed at birth), `width`, `jamb0`,
`jamb1`, and a base_link copy of each (`center_bl`, `jamb0_bl`, `jamb1_bl`);
`parent_wall_id0/1` (the host wall ids at each jamb on the last positive
sighting, equal for an in-track door -- always wall ids, there are no frame
pieces any more); `occupied`; `confidence` ([0, 1]); `stereo_support`; `hits`
(positive sightings ever); `age`; `since_seen` (since the last **positive**
sighting); `n_beams` (decided beams on the last positive sighting); `leaf_side`
(`LEAF_NONE/LEFT/RIGHT`, step 2); `glass_suspect` (step 2); and
`jamb_height_unknown` (the addendum, `95d3a77`: no positive sighting has had
both jambs' wall height judged by the stereo).

* **States** (`95d3a77`): `STATE_CANDIDATE = 0`, `STATE_CONFIRMED = 1`.
  `STATE_TENTATIVE = 0` is kept as the legacy alias of `STATE_CANDIDATE`, and
  `STATE_COASTING = 2` as a legacy constant no longer published (doors do not
  coast). Humble rosidl accepts the duplicate constant value.
* **Wire-compatible.** `jamb_height_unknown` is appended last, after `leaf_side`
  and `glass_suspect`; every earlier field keeps its position and meaning, so
  bags recorded before the addendum deserialise unchanged.

### `MultiWallTrack.role`: `other` → `unknown` (addendum A)

The wall role meaning "real, tracked, drawn, an occluder, but a reference wall
for nothing" was renamed from `other` to `unknown` (`multi_wall_tracker.ROLE_UNKNOWN`).
Role and label values are now `front | back | left | right | unknown`; the tracker
never assigns `other` any more.

* **Wire type unchanged.** `MultiWallTrack.role` and `.label` stay `string`; no
  field was added, removed or reordered (comments only). Every bag recorded before
  and after deserialises as before, on Humble and Jazzy.
* **Readers accept both spellings.** `multi_wall_tracker.is_unknown_role(role)` is
  true for `unknown`, the legacy `other` and `''`. The costmap wall layer
  (`wall_raster.ROLE_VALUE`) maps both to 50, as before; the viz (`wall_markers`,
  the node's `_ROLE_COLOUR`, `tools/corridor_wall_replay.py`) keeps an `other` key
  in the same neutral grey as `unknown`. The FRONT gate refuses `other` exactly as
  it refuses `unknown`; its debug code stays `role_other`.
* **Tentative tracks are `unknown`** wherever tracks are published or drawn,
  whatever their geometric role: `/perception/walls` role/label (where a state-0
  track appears only on the cycle after a merge; `d_side` NaN), the marker colour
  and text on `/perception/walls/markers` and `/viz/walls`, and the debug tracks
  list. Inside the tracker `Track.role` keeps the geometric role, so the role
  hysteresis, `published()`, FRONT, the contract, the MPC, the costmap wall layer
  and the BT are unaffected.

## 5. Foxglove, the debug topic, recording

* **3D panel**, display frame `odom`: enable `/viz/doors` (and `/viz/walls` or
  `/perception/walls/markers` for the walls). `/viz/doors` crosses the bridge
  through the existing `'^/viz/.*'` allowlist entry (pinned in
  `test_foxglove_whitelists.py`) and is published on every tracker cycle while door
  detection is on, subscribed or not. No Foxglove layout is tracked in the repo, so
  there was no saved 3D panel to extend.
* **Each door**: two jamb cylinders (0.10 m × 2.5 m on the floor), the threshold
  line and a label. CONFIRMED doors solid, CANDIDATES faded, on all four
  markers. The label reads `door <id> <width>m <state>[ leaf <side>][ glass?][
  jamb?][ OCCUPIED]`; `jamb?` is `Door.jamb_height_unknown` (the stereo never
  measured both jambs), `glass?` is `glass_suspect`. An OCCUPIED door is orange,
  every other door green. The four markers sit at slots `4·id .. 4·id+3`; a dead
  door gets a DELETE on each slot, every marker a 0.6 s lifetime, and no
  DELETEALL (as the walls, §A).
* `/perception/doors` is not on the bridge allowlist: `ros2 topic echo
  /perception/doors` on the car, or `foxglove_raw_debug:=true`.
* **Debug** (`/perception/virtual_wall/debug`, JSON): `door_ms`, `doors`,
  `doors_confirmed`, `door_candidates` (every gap judged this cycle with its
  code / class / `visible` / `detail` / width / beams / view angle / leaf / ...,
  doors and STRONG records first, at most `DOOR_DEBUG_MAX_CANDIDATES` = 32),
  `door_candidates_total`, `door_manhattan` (the prior), `door_n_strikes`,
  `door_n_deleted_by_strikes`, `door_n_confirmed`, `door_errors`,
  `door_disabled`, `door_mask_age_s`, `door_stereo`. Every key is `null` while
  door detection is off.
* **Recording.** `/perception/doors` and `/viz/doors` are in the mission logger's
  default topic list (step 2); the logger stays disabled by default
  (`enable_mission_logger: false`) and only records what it was started with, so a
  run must start it after the build. The repo's manual `ros2 bag record` lists do
  not include the door topics (§12 has a command that does).

## 6. Humble / Jazzy notes

* No Jazzy-only API: the door path uses `create_publisher`/`publish` and
  `get_logger().error(..., throttle_duration_sec=)`, all on Humble. Python is
  3.10-safe and numpy usage is 1.21-safe (the Orin runs numpy 1.26.4).
* **Humble checks every message field's type on assignment** (Jazzy only with
  `ROS_PYTHON_CHECK_FIELDS=1`): every Door / Marker field is assigned an exact
  Python type, and the node tests were also run with `ROS_PYTHON_CHECK_FIELDS=1`.
* **An exception in the door cycle cannot take the node down**: it is caught,
  logged (throttled) and counted (`door_errors`); after 50 consecutive failures
  door detection disables itself for the node's life. Walls are published before
  the door cycle and are unaffected.
* **Frame.** Doors live in the tracker's fixed frame, **odom**. The node has no
  `/tf` subscription (it reads `/tf_static` only), so it cannot place doors in
  `map`; a consumer that wants them there should transform them.
* **Jazzy TODO (not fixed here):** `corridor_perception_c` does not build on
  Noble with CMake ≥ 3.26 (linus: CMake 3.28.3): `'M_PI' undeclared` at
  `src/extract_lines.c:31` (also 87, 146, 420). `CMakeLists.txt:11` forces
  `-std=c11` while `:17` (`C_STANDARD 11 C_EXTENSIONS ON`) adds `-std=gnu11`; since
  CMake 3.26 the standard flag goes *before* the compile options, so strict C11
  wins and glibc hides `M_PI`. **gcc 13 is not the cause** (it builds the file
  with `-std=gnu11`). CMake 3.22 on the Orin puts `gnu11` last, so it builds there.
  Later fix: drop `-std=c11` (keeps the Orin build exactly as it is), or
  `#define _DEFAULT_SOURCE` / `_XOPEN_SOURCE 700` before the first include
  (`_POSIX_C_SOURCE` is not enough; ISO C11 also turns off FMA contraction on
  aarch64, so re-run the backend equivalence check). Consequence on linus: the
  tracker falls back to the Python extraction after one startup WARN (24.6 vs
  1.1 ms a scan), and `test_extraction_backends` skips its C half.

## 7. Tests (synthetic scans, no stack)

All on synthetic `LaserScan`s and hand-built tracks; no stack is launched. The
door suites, all green on linus:

* **`test_door_detection.py`** (the core, 16 classes): `TestCodesAndParams`
  (every code has one class, the defaults ARE the addendum table, the removed
  params are gone, `tentative` is the old name of `candidate`), `TestTheDoorway`,
  `TestTheBeamsThroughTheGap` (perpendicular depth, `on_line` in- vs
  between-track, `jamb_occluded`, `few_beams` and the range-scaling),
  `TestTheHosts` (only a CONFIRMED front/left/right track hosts; corner →
  `not_collinear`; `untracked_jamb`; the door found from a moved, turned car),
  `TestTheHostRules` (the front-facing option: a doorway straight ahead has no
  host by default, is a door with the option on, and a rejected / occluded /
  contradicted / behind / beside piece never hosts), `TestTheAddendumF`
  (F1-F12, the addendum's worked examples), `TestBoundary` (`not_boundary`, the
  perpendicular-wall exception, the end margins), `TestTheLeaf` (the leaf and
  reveal, `not_a_leaf`, `leaf_side` and its mirror, the thick-wall two reveals),
  `TestLifecycle` (candidate → confirmed on two viewpoints and steady width,
  strikes, ageing, persons/occupied), `TestIdentity`, `TestStereoSupport`,
  `TestStereoJambHeight` (the furniture check and `jamb_height_unknown`, the
  frame fetched once), `TestGlassVeto` (off by default), `TestManhattanPrior`,
  `TestThroughTheRealTracker` and `TestTheCollinearGapRule` (end to end on the
  real extraction and `MultiWallTracker`: 0.5 / 0.85 / 1.4 m, the fallback).
* **`test_door_markers.py`** (`TestJambs`, `TestLabel`, `TestColour`,
  `TestDeletion`): the cylinders and threshold, the label text and flags,
  state → alpha, OCCUPIED → colour, the per-slot DELETE and lifetime, no
  DELETEALL, and that the label no longer says `frame` or `square`.
* **`test_lidar_front_wall_node.py::TestDoors`**: the node on synthetic scans --
  a doorway driven past is a CONFIRMED `Door`; a one-place sighting stays a
  candidate off `/perception/doors`; `door_publish_candidates`; every field
  clamped to its exact wire type; `/viz/doors` on every cycle with no door and no
  subscriber; the topics absent and every debug key `null` with detection off; an
  exception in the door cycle is logged, counted and the walls still publish.
* **`tools/test_door_replay.py`** (19): the merge rule old vs new on the
  retracker, the candidates CSV (one row per judged gap), the `width` row for a
  too-far pair, the cadence and pose interpolation, the capture/restore, the
  debug-frame-by-header-stamp.

The code defaults are pinned to `stack_params.yaml` by the coupling tests, so a
yaml edit that is not mirrored in the code (or the reverse) fails the suite --
the lesson of the stale launch-config test in CLAUDE.md.

## 8. Known limits

* **A closed glass door in a SOLID wall can be confirmed as open.** Glass returns
  the LiDAR only within ~2° of its normal; seen obliquely it looks through, and its
  solid host segments pass `not_boundary`. From two oblique viewpoints it confirms.
  Mitigations, both off by default: `door_square_looks:=1` (step 3's square-on look
  as an extra requirement) or `door_stereo_glass_veto:=true`. A glass **partition**
  (see-through hosts) is caught by `not_boundary`.
* **Glass corridors carry no doors**: their frames are unknown fragments, never
  hosts (step 3's frame jambs are gone, §10).
* **A leaf open less than about 37°** (a leaf as long as the doorway is wide then
  projects over the whole core: cos α ≥ 0.8) leaves no core beam to judge:
  `few_beams` (WEAK), never confirmed while it stays like that.
* **Doors out of view are aged out** (10 s candidate, 60 s confirmed); they are not
  a map.
* **The width is the gap between the last ON returns**: true width plus up to a
  beam spacing each side (1.3 cm at 3 m), and biased at long range.
* **Not validated on the car.** Offline, LiDAR-only: [doors_phase1.md](doors_phase1.md).

## 9. The collinear merge rule (point 1)

Every threshold that joined collinear LiDAR wall pieces before step 2:

| Where | Value | Since (git blame) | Status |
|---|---|---|---|
| `corridor_perception.extraction.ExtractionParams.merge_gap` (with `merge_alpha` 6°, `merge_rho` 0.05 m) | 1.5 m | `119f239` 2026-09-22 (Step 0 baseline) | library default, kept; the stack never uses it (`mwt.extraction_params()` always sets `merge_gap`) |
| `corridor_perception_c` `el_default_params.merge_gap` | 1.5 m | `c421578` 2026-10-16 (H1, C port) | C default, kept; the C backend receives `merge_gap` from the Python params on every call |
| `multi_wall_tracker.TrackerParams.merge_max_gap_m` → the extraction's `merge_gap` | 1.5 m | `193fe1d` 2026-10-15 (span gate) | removed |
| `lidar_front_wall_wall_merge_max_gap_m` (yaml, node, launch) | 1.5 m | `193fe1d` 2026-10-15 (`c44b503` 2026-10-16 only re-touched the node line) | removed |
| `TrackerParams.assoc_max_gap_m`: association gate, `_merge`, `_refresh_extent`, `_gap_between` | 0.5 m (inclusive) | `d578963` 2026-10-16 (block 3, andreas-linus) | replaced by `collinear_gap_m` |
| `lidar_front_wall_wall_assoc_max_gap_m` (yaml, node, launch) | 0.5 m | `d578963` 2026-10-16 | removed |
| `tools/extract_backends_equiv.py`: explicit merge gap | 1.5 m | `c421578` 2026-10-16 | now uses the stack's rule |

**The rule** (THE COLLINEAR GAP RULE, `multi_wall_tracker`): two collinear pieces
are one wall only across a gap **strictly shorter** than `door_width_min_m`
(0.70 m); at 0.70 m or more they stay two walls. Strict `<` at every site: the
extraction merge (`extraction.py:381`, `extract_lines.c:487`), the association
gate, the extent rebuild and the track merge (`_gap_between` and its callers).

* **Single source.** `lidar_front_wall_node` reads `door_width_min_m` once and feeds
  `TrackerParams.collinear_gap_m`, `extraction_params()` (both backends) and
  `DoorParams.width_min_m`. The code default is one constant,
  `multi_wall_tracker.DOOR_WIDTH_MIN_M = 0.70`, pinned to the yaml by
  `test_the_code_default_door_width_is_the_stack_params_value`. The rule applies
  with door detection off too.
* **How the gap is measured.** The extraction measures it end to end. The tracker
  counts only the part its beams saw **open**: a stretch hidden by a detected
  object's cut sector or by a nearer return is not an opening.
* **THE WALL'S SPAN** (`1b2155d`, tonight). Step 2 made the extraction split a wall
  at a person's cut sector ≥ 0.70 m, and in front of a narrow wall each piece is
  shorter than `wall_min_span_birth_m` (1.0 m): the wall was never born -- no FRONT,
  the contract CLEAR (a 2.5 m wall with a person 0.5 m in front: never FRONT on a
  3 m approach). The span gates now read the span of the wall a LiDAR piece belongs
  to: pieces of one line (the tracker's chi2 gate) with less than
  `door_width_min_m` of **open** gap between them are measured together. FRONT now
  comes from cycle 2 (the 3-hit confirmation's earliest). An open doorway still
  keeps the pieces either side apart. The price: two short collinear walls whose
  gap is mostly hidden (a person in a 1.0 m mouth) are born as one wall, as long
  walls already were. A stretch no beam looks at (behind the laser) counts as open
  for a birth.
* **Fallback.** The detector's in-track gap search is kept.
* **Other consumers.** The tracker bridges open gaps up to 0.70 m (wider than the
  0.572 m travel strip), so a 0.572-0.70 m gap between two collinear faces can be
  read as wall. On `campaign-final-doors` the MPC takes its boundaries from the
  walls (`boundary_source: walls`), so this rule shapes its wall rows too; its own
  `wall_row_merge_*` is a separate, consumer-side collinear merge and is unchanged.
* **Tests.** End to end on the real extraction and tracker: 0.5 m → one wall,
  0.85 m → two walls and one door between them, 1.4 m → two walls, no door; the
  extraction boundary 0.69 / 0.70 / 0.71 m on both backends (the C half runs only
  where `corridor_perception_c` loads: **run `tools/extract_backends_equiv.py` on
  the Orin**, §12).

## 10. History

* **Step 1** (`3034c94`, `5d75fa6`, 2026-10-19): `Door`/`DoorArray`, the detector
  (every beam through the gap past 0.30 m), tentative → confirmed (3 of 5) →
  coasting → deleted (2 s).
* **Step 2** (`35e9c91`, the user's points 1-4): the collinear gap rule (§9); the
  open leaf in the outer parts (core 0.6) and `Door.leaf_side`; the stereo glass
  veto (off) and `Door.glass_suspect`; `/viz/doors` on every cycle, in the bridge
  allowlist and the logger's default list.
* **Step 3** (`a335113`): FRAME JAMBS (doorways between the scan's short straight
  pieces in glass partitions), SEEN SQUARE-ON (no door confirmed before a square-on
  look) and PANES (a never-squared door returning ON twice was deleted and
  remembered 60 s). On `corridor10m_20261016T144537` it confirmed **no** door: no
  door ever got a square look ([doors_phase1.md](doors_phase1.md)).
* **Tonight's audit of points 1-6** (2026-10-19): point 1 correct as a rule, with
  the birth regression fixed by THE WALL'S SPAN; point 2: a leaf did not have to
  start at its jamb (13 % of random clutter in the outer part passed), a 45° leaf
  was never a door, near-hinge returns in the ON band made `on_line`, thick walls
  made `not_a_leaf`; point 3: no ego-motion compensation between the mask and the
  scan (an 8° yaw turned a real door into a veto), `glass_suspect` cleared when
  the camera did not cover the gap; point 4 complete (no Foxglove layout in the
  repo); point 5 not done; point 6 stale. All fixed on this branch.
* **The addendum** (2026-10-20): hosts must be confirmed front/left/right walls;
  the Manhattan prior and snap; every distance perpendicular; `not_boundary` and
  the stereo jamb height; STRONG / WEAK / GEOMETRY codes; the conservative
  lifecycle; the `unknown` role. **Removed**: frame jambs (frame lines are unknown
  fragments: they can never host), panes (they made unknown ON returns evidence
  against a door, and deleted doors without strikes). Seen square-on stays as an
  optional extra requirement (`door_square_looks`, default 0).

## 11. Offline replay (`tools/door_replay.py`)

```bash
python3 tools/door_replay.py [--walls recorded|retrack-old|retrack-new] \
    [--cadence steps|every] [--params stack|code] [--set FIELD=VALUE ...] \
    [--src CHECKOUT] [--door-images all|confirmed|none] [--out DIR] BAG
```

Pure Python (numpy, matplotlib, PyYAML, rosbags; no rclpy, nothing launched). The
door cycle of `lidar_front_wall_node` on a rosbag2 (sqlite) bag, at the car's own
cadence (one cycle per recorded `/perception/walls` message), with the replayed
checkout's `stack_params.yaml`. `--walls recorded` replays the car's own published
tracks (it cannot show a tracker change such as the merge rule); `retrack-*`
re-runs the tracker LiDAR-only with the pre-step-2 or the current rule. No camera:
no stereo, no YOLO cuts, no persons. Outputs: per-cycle and per-candidate CSVs, a
JSON summary, an overview PNG and `door_<id>.png` (the nearest
`/camera/debug_image/compressed` frame by header stamp, the nearest frame with the
door in view, the top-down view). Results: [doors_phase1.md](doors_phase1.md).

## 12. Importing on the Orin

On the Orin, in the workspace (check for local changes first; the install is a
copy, not a symlink, so a pull only takes effect after the rebuild):

```bash
cd ~/dev_ws/f1tenth_more
git status --short                      # must be clean (or commit / stash by path first)
git fetch origin door-fixes
git switch -c door-fixes --track origin/door-fixes   # first time
# or, once campaign-final-doors has it merged in:  git switch campaign-final-doors && git merge --ff-only origin/campaign-final-doors
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select f1tenth_messages f1tenth_params f1tenth_perception f1tenth_costmap
source install/setup.bash
ros2 interface show f1tenth_messages/msg/Door          # sanity: jamb_height_unknown, STATE_CANDIDATE present
```

Those are the packages this branch changes over `door-detection` with runtime
effect: `f1tenth_costmap` because the `other` → `unknown` role rename touches its
wall layer (`wall_raster.py`, §4); `corridor_perception` gained only a test, so it
needs no rebuild. Importing onto plain `campaign-final` (not `campaign-final-doors`)
also pulls in the door-detection packages themselves -- `f1tenth_logger` (step 2's
door topics in its default list) and `corridor_perception_c` (the C collinear merge
rule, §9) -- so rebuild those too in that case.

Then restart the `lidar_front_wall` supervisor component, which picks up the new
parameters from `stack_params.yaml`. Watch `/viz/doors` in Foxglove and
`ros2 topic echo /perception/doors` on the car. To record the door topics
(they are not in the manual `ros2 bag record` lists):

```bash
ros2 bag record /perception/doors /viz/doors /perception/walls \
    /perception/virtual_wall/debug /scan /odometry/filtered /tf /tf_static
```

Run `tools/extract_backends_equiv.py` on the Orin to confirm the C and Python
extraction agree on the 0.70 m collinear boundary (§9). To go back:
`git switch campaign-final-doors` (or `campaign-final`), then rebuild the same
packages.

---

## Appendix A. Step 1 code audit (before writing, 2026-10-19)

### What the multi-wall tracker exposes per wall

`multi_wall_tracker.Track` (in-process, odom frame):

| Quantity | Where |
|---|---|
| id (never reused) | `Track.id` |
| line | `x = (rho, theta)`: `rho = x cos(theta) + y sin(theta)`, `rho >= 0`. `theta` is the normal from the odom origin toward the line. Covariance is `P` (2x2). |
| endpoints | the measured extent `t0..t1` along the tangent `(-sin, cos)`. `endpoints()` returns them in odom; `segment_in_base_link()` returns them in base_link. |
| normal / rho / d_ray | `theta` / `rho` (odom). `front_distances()` gives `d_perp_front` and `d_ray_front` from the bumper; `strip_distance()` is the FRONT distance. |
| side class | `role` (front/back/left/right/unknown, sticky, with hysteresis; `other` before 2026-10-19). `geometric_role()` decides it. |
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
  sector. The class id is kept in `_detection_classes`, in the same order, because a
  person needs to be told apart from other objects.

### Floor, inferred floor and wall masks

`front_clearance_node` publishes `/perception/scene_labels/classes` (mono8
`scene_mask.CLASS_*`: FLOOR 3, WALL 6, FLOOR_INFERRED 7, OBJECT 2, …) together with
`/perception/scene_labels/depth_mm`. Both carry one stamp. `lidar_front_wall_node`
subscribes to them only when `wall_rect_verify` is on (the default). `_mask_pair()`
returns the pair, with intrinsics from `/perception/stereo_columns` and the camera
transform from `/tf_static`. The rectangle verification (`wall_fusion.verify_rectangles`)
and the door stereo checks use the same pair, intrinsics and transform.

### Wall markers, and why their height varied

Two builders drew walls: `/perception/walls/markers` (`_publish_walls`, a slab from
`z_min` to `z_max`: the stereo's measured height or `wall_default_height_m`) and
`/viz/walls` (`wall_markers`, the measured span or a fixed 2.0 m). Heights therefore
differed between walls and between the two topics. Since step 1 both draw every wall
from the floor to `wall_viz_height_m` (2.5 m); `z_min/z_max` on the message are
unchanged, and the slab's alpha still shows measured (0.65) or assumed (0.35).

### DELETE / DELETEALL

Both wall topics send a per-id DELETE for every dead track and give each marker a
lifetime (0.6 s), and deliberately no DELETEALL (one every cycle made the walls
flicker). `/viz/doors` uses the same scheme: a DELETE on each of a dead door's slots, a
lifetime on every marker, no DELETEALL, so a stalled node or a restart fades out after
0.6 s instead of leaving ghost markers.
