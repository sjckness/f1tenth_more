# Scene labels: offline replay, 2026-10-13

Tool: `tools/scene_labels_replay.py`. Per-scan CSVs, 0.5 s tables and MP4s are in
`mission_videos/scene_labels_replay/` (gitignored).

## What was and was not validated

The archived bags carry **no depth image, point cloud or camera_info** (checked across
every `metadata.yaml` in `~/f1tenth_archive` and `~/.ros/mission_bags`). The bags were
played through the real `LidarFrontWallNode` twice, `enable_scene_labels` false and true.
The nodes ran in-process on the recorded clock.

- **Validated:** the LiDAR half. That covers detection sectors, wall planes, the
  heading-wall selection and span check, and the virtual wall with odometry.
- **Not validated:** the stereo half. That covers ground, ceiling, IGNORE, the WALL pixels,
  OTHER-only object depth, the stereo wall distance, and front clearance on WALL and OTHER.
  Every ON value below has no stereo input.
- **Labelling CPU:** timed on **synthetic** 640 x 360 depth frames (floor, ceiling at
  2.6 m, the run's LiDAR walls, a 0.5 m person box). These frames measure cost only.
  They do not validate the labels.

OFF wall = the recorded `/perception/front_distance` (camera), which the stack actually
used. The LiDAR reference is chosen per tick:

- the deployed plain fit, when it measured;
- otherwise `g`, the longest uncut line that brackets the heading with a gap of at most
  1.5 m.

Neither reference uses the new code path.

## Runs

| | run | what |
|---|---|---|
| a | `complete/2026-10-13T11-33-21` (M02 P002-R005) | person + laptop between the car and the wall; the stop fired on the person |
| b | `complete/2026-10-13T11-10-47` (M02 P002-R001) | bare wall, nothing in front |
| c | `active/2026-09-11T16-14-50_mission-drive_stop_2m_from_wall` | person detections, doorway, end wall with a protrusion; no camera debug recorded |

## Summary

| | ON vs LiDAR ref, median / p95 / max [m] | person reported as wall (ON) | OFF wall > 0.5 m short with a person present | 2.0 m stop fires at, OFF / ON [s] | 90 deg wall_turn would commit at (trigger 2.68 m), OFF / ON [s] |
|---|---|---|---|---|---|
| a | 0.0014 / 0.0078 / 0.0144 | 0 ticks | 356 of 361 ticks | 8.51 (wall 4.2 m away) / never (wall 3.9 m at bag end) | 6.61 / never |
| b | 0.0025 / 0.0056 / 0.0080 | 0 | 0 | 18.13 (wall 2.4 m away) / never (2.49 m at bag end) | 13.13 / 16.33 |
| c | 0.171 / 0.176 / 0.195 | 0 | n/a | no camera record / 12.83 | no camera record / 11.53 |

In (c), the 0.17 m offset comes from the 32 s the car stood at the end wall, plus 12.3 to
14.8 s. During those times, the plain ±10° fit and the gap reference measure a narrow
protrusion, about 0.18 m wide, in front of the end wall. ON measures the wall behind it.
That object is OTHER by definition: it is narrower than 0.8 m. Up to 11.8 s, ON matches the
plain fit to within 0.01 m. The early versions of the selector took two collinear wall faces
either side of a 2.4 m doorway as "the wall ahead". `scene_labels.wall_ahead` now rejects
that case.

CPU, measured on the Orin while the stack was running, with one BLAS thread as deployed:

| | OFF | ON |
|---|---|---|
| `lidar_front_wall_node` scan callback, mean ms (p95) | 1.37-1.49 (1.6) | 1.54-1.84 (4.2-5.1) |
| at ~40 Hz, % of one core | 5.5-6.0 | 6.2-7.3 |
| `front_clearance_node` labelling tick (synthetic depth), mean / p95 ms | n/a | 6.9-8.2 / 7.2-8.7 (9.8 on the stereo-fallback path) |
| at 5 Hz, % of one core | n/a | 3.4-4.1 (4.9 fallback) |

On synthetic depth, 0 of 1144 person pixels were labelled WALL. The floor was fitted at
0.17° / -2 mm, the ceiling at 2.59 m against 2.60, and the stereo wall at 3.595 m against a
true 3.595.

## Acceptance (for `enable_scene_labels: true`)

- (a) Person never reported as the wall: **pass**. Wall within 0.10 m of the LiDAR:
  **pass** (max 0.014 m).
- (b) Wall within 0.05 m of the LiDAR: **pass** (max 0.008 m). Ground and ceiling never
  labelled wall: **NOT CHECKABLE**, because there is no depth in any recorded bag.
- CPU budget (< 5 %, < 15 ms per labelling tick): **pass on synthetic depth only**.
- `colcon build`: **clean**.

**Default stays `false`.** The ground/ceiling check of (b), and the stereo half as a whole,
cannot be validated from recorded runs.
