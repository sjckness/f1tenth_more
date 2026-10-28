# Doors from the semantic layer: recon

Branch `doors-enter`, worktree `~/dev_ws/f1tenth_more_doors`, base
`origin/campaign-final` (`c5a64177`). Humble target.

## 1. How a wall reaches the semantic layer

```
/scan ──> lidar_front_wall_node (f1tenth_perception)
            multi_wall_tracker.py  +  wall_fusion.py
          ──> /perception/walls   f1tenth_messages/MultiWallTrackArray
                                  header.frame_id = the ODOM frame
                                  header.stamp    = the scan stamp
                                  at most 4 confirmed tracks (+ coasting
                                  inside that cap)
          ──> /perception/wall_observations  WallObservationArray (raw, per scan)

/perception/walls ──> semantic_layer_node (f1tenth_costmap)
            _walls_msg_cb  -> self._latest_walls (newest message only)
            _confirmed_walls_map()   STATE_CONFIRMED tracks, p0_bl/p1_bl
                                     moved base_link -> MAP through the
                                     nearest global-EKF pose by stamp
                                     (semantic_layer.find_nearest_pose_by_stamp
                                      + wall_raster.transform_segment)
            + _observed_walls_map()  long LiDAR-backed raw observations
          ──> wall_raster.rasterize_walls
          ──> /costmap/wall_cells      OccupancyGrid, value = ROLE_VALUE[role]
              /costmap/wall_cell_ids   Int32MultiArray, the track id per cell
```

So "the semantic layer's walls" are the **CONFIRMED tracks of
`/perception/walls`, re-expressed into the map frame every cycle, never
accumulated**. The semantic layer adds nothing geometric of its own; it adds
the map frame, the ownership of LiDAR cells, and the role/id per cell.

### What one wall carries (`MultiWallTrack.msg`)

| field | meaning |
|---|---|
| `id` | never reused; re-spawns often in practice |
| `role` / `label` | `front \| back \| left \| right \| unknown`, with hysteresis. A TENTATIVE track is always `unknown`. |
| `state` | `STATE_TENTATIVE 0`, `STATE_CONFIRMED 1`, `STATE_COASTING 2` |
| `source` | `SOURCE_LIDAR 0`, `SOURCE_STEREO 1`, `SOURCE_BOTH 2` — the best evidence the track ever had |
| `rho`, `theta` | the line in odom (`rho = x cos t + y sin t`, `rho >= 0`) |
| `p0`, `p1` | fitted **extent endpoints**, odom, z = 0 |
| `p0_bl`, `p1_bl` | the same endpoints in base_link at the scan stamp |
| `confidence` | `[0, 1]` of the observation that last matched it |
| `z_min`, `z_max`, `height_measured` | vertical extent; assumed unless measured |
| `hits`, `age`, `cov` | lifecycle and the (rho, theta) covariance |

**Endpoints are `p0`/`p1`** (or `p0_bl`/`p1_bl`). There is no separate
"endpoint is a real wall end" flag anywhere: an extent end may equally be a
true corner, the edge of the LiDAR FOV, or an occlusion shadow. Deciding
which is the new detector's own job, and it needs `/scan` to do it.

### Confirmation levels, by name

Two independent axes, `state` and `source`. The new detector's
`door_min_wall_level` orders them into one scale:

| level | name | means |
|---|---|---|
| 0 | `tentative` | any live track, confirmed or not |
| 1 | `confirmed` | `state == CONFIRMED` (or COASTING), any evidence — a stereo-only wall counts |
| 2 | `lidar_confirmed` | `state == CONFIRMED` and `source in (LIDAR, BOTH)` — **the default** |
| 3 | `stereo_checked` | `state == CONFIRMED` and `source == BOTH` |

Default 2: tentative walls are excluded, and the stereo is never required.

## 2. Where the existing profile-based door detector is

| | |
|---|---|
| logic | `src/f1tenth_perception/f1tenth_perception/door_detection.py` (1811 lines, pure numpy) |
| markers | `src/f1tenth_perception/f1tenth_perception/door_markers.py` |
| host | `src/f1tenth_perception/f1tenth_perception/lidar_front_wall_node.py`, on the wall tracker's own cycle, behind `lidar_front_wall_door_detection_enabled` (**default false** since 2026-10-20) |
| out | `/perception/doors` (`f1tenth_messages/DoorArray`), `/viz/doors`, plus per-cycle counters on `/perception/virtual_wall/debug` |
| params | `src/f1tenth_params/config/stack_params.yaml`, ~40 keys under `lidar_front_wall_door_*` |
| tests | `src/f1tenth_perception/test/test_door_detection.py`, `test_door_markers.py` |
| replay | `tools/door_replay.py` |

It is **profile-based**: it takes the CONFIRMED wall tracks as *hosts*, snaps
them onto the room's Manhattan axis, and then judges each gap on a
**per-beam depth profile measured perpendicular to the host line** — through
/ shallow / on-line bands, an open-leaf fit, a reveal, a boundary test, and
an optional stereo veto. It needs the raw `/scan` and the stereo class mask,
and it lives next to both.

**The overlay rule** it documents holds for the new source too: a door never
modifies, splits or removes a wall or any LiDAR data, and nothing may treat
a doorway as free space because a door is published there.

## 3. What the new (semantic) source does differently

| | profile | semantic |
|---|---|---|
| input | wall tracks **+ raw scan profile + stereo mask** | **wall segments only**, from the semantic layer, in map; `/scan` used *only* for the two liveness tests below; no stereo |
| frame | the tracker's odom frame | **map** |
| host | one wall line, gaps inside/between collinear hosts | **case A** two collinear walls; **case B** one wall's end against a perpendicular wall |
| geometry | Manhattan snap, 10-code test ladder | angle/offset/width thresholds only |
| evidence | beam profile bands, leaf, reveal, boundary | beams pass through the gap; nothing on the gap segment |
| output | `/perception/doors`, `/viz/doors` | `/perception/doors_semantic`, `/viz/doors_semantic`, **TF `door_<id>`**, `/costmap/semantic_doors` |

Both are gated by `door_source` (`semantic` by default) and each keeps its
own enable flag, so turning one on never starts the other.

## 4. What the semantic layer already provides the new detector

All of it is in `semantic_layer_node`, which is why the new logic is hosted
there and not in a node of its own:

| need | already there |
|---|---|
| walls in map, with id/role/state/source | `self._latest_walls` + `find_nearest_pose_by_stamp` |
| the car's map pose | `self._pose_history` (global EKF) |
| the live scan and the laser's pose | `self._scan`, `_laser_pose_for(frame)` |
| beams, bearings, ranges | `unknown_objects.scan_points` |
| "do the beams pass through here" | `unknown_objects.rays_pass_through` |
| "is this behind a wall from the laser" | `unknown_objects.drop_behind_walls` |
| objects to veto a gap with | `self._tracks` (recognised **and** unknown), map frame |
| map <-> base_link | `unknown_objects.to_map` / `to_base_link` |

## 5. The one deliberate deviation from the task

`/costmap/semantic_objects` is **not** where the doors go.
`stack_params.yaml:3483` — mpc_corr reads that topic and adds its MEMORY
objects to the obstacle list and *all* of its objects to the reverse
manoeuvre's rear pre-check. Publishing a door there would hand the MPC a
phantom obstacle standing in the one place the car is being asked to drive
through.

The doors are published instead on **`/costmap/semantic_doors`**, same
message type (`SemanticObjectArray`), same frame (map), same node, same
cycle, `class_id = "door"` — a semantic-layer object list, just not the one
wired into the obstacle path.
