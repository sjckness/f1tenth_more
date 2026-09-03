# `<run_id>.extract.parquet` — schema

Everything `mission_render.py` draws, in one ROS-free file. Written by
`mission_extract.py` (needs ROS), read by `mission_render.py` (needs none).

**This document was derived from a real extract, not from intent.** Every
field name, type and example below was read back out of
`2026-09-02T15-27-43_mission-bottle_then_person.extract.parquet` with
`pyarrow.parquet.read_table`, and the units were traced to the ROS message
fields `read_bag()` actually reads. Where something the work order asked for is
*not* stored, that is stated rather than papered over — see **Padded
halfspaces** and **Stop events**.

## Table

Four columns, one row per record, zstd-compressed. Row kinds are distinguished
by `kind` rather than by separate files, so a run's extract is one artifact
that copies or syncs whole.

| column | arrow type | meaning |
|---|---|---|
| `kind` | `string` | `meta` \| `grid` \| `static_tf` \| `sample:<stream>` |
| `t` | `double` | sample time, seconds, bag receive clock. **NULL** for `meta`/`grid`/`static_tf` |
| `payload` | `string` | JSON; shape depends on `kind` |
| `blob` | `binary` | zlib-compressed raster bytes; **only** on `grid` rows, NULL elsewhere |

Reference run: 1971 rows, 0.63 MB, from a 97.3 MB bag.

## Timebase

`t` is the **bag receive timestamp** in seconds — one clock for every stream,
so nothing can desync because a publisher stamped its header from a different
source. It is absolute, not relative: subtract `meta.t0` for run-relative time.

Samples are stored **at full recorded rate**, deliberately *not* resampled onto
a fixed grid. Resampling is the renderer's job (zero-order hold with a
per-stream `max_age`, `--dt` selects the period). Baking one `dt` into the
extract would change output for every other `dt` and make the md5-equivalence
check meaningless.

## Why JSON payloads, honestly

Convenience for ragged nesting, **not** float exactness.

An earlier version of this document claimed JSON was what made byte-identical
re-rendering possible. That was wrong and worth correcting: Parquet's `double`
is IEEE-754 and round-trips bit-exactly on its own, so native columns would be
exactly as lossless. JSON is not buying precision here.

What it does buy is one uniform payload column for structures that are ragged
and heterogeneous in different ways per stream: a per-tick horizon of varying
length, a variable-length list of halfspaces, a dict of corridor polylines
keyed by namespace, tuples mixing strings, floats and `null`. Expressing those
natively means a wide schema of `list<double>` / `list<struct<...>>` columns,
mostly null on any given row, plus a migration whenever a stream's shape
changes.

The cost is real and worth naming: the file is larger and slower to read than
native columns would be, and **a non-Python consumer must parse JSON out of a
string column** rather than reading typed values directly. If a non-Python
reader ever matters, or the extract grows enough for size to bite, converting
the numeric streams (`pose`, `drive`, `clearance`, `map_to_odom`, and the
horizon inside `solver`) to native `double` / `list<double>` columns is the
obvious next step, and would not change what the renderer draws.


## `meta` (exactly one row)

```json
{"extract_version": 1, "t0": 1788434263.7, "t1": 1788434277.8,
 "pose_frame": "map", "pose_topic": "/ekf_global/odometry/filtered",
 "legacy_solver_msgs": 0, "dropped": {},
 "max_age": {"pose": 0.5, "map": null, ...},
 "manifest": {...}, "run_id": "2026-09-02T15-27-43_mission-bottle_then_person"}
```

`padding` is `{"car_radius": 0.2, "avoidance_margin": 0.12, "source": <path>}`,
read from **this run's own params snapshot** (`car_radius` and
`obstacle_safety_margin_m`, which is what feeds MPC_corr's `avoidance_margin`).
The renderer uses these unless `--car-radius`/`--avoidance-margin` is passed
explicitly, and announces the override when it is. `source` is `"fallback"`
when the run had no snapshot, so a guessed value is never mistaken for a
recorded one. See **Padded halfspaces** below.

`max_age` (seconds, `null` = never expires) is the renderer's staleness gate
per stream, carried in the file rather than hard-coded on the read side so the
two halves cannot silently desync. `manifest` is the run's whole
`manifest.json` inlined, so an extract is self-describing after it is copied
away from its folder. `dropped` maps topic → count of undecodable messages.

## `grid` rows — the deduplicated occupancy raster

`payload` `{"grid_id": int, "shape": [height, width], "sha256": hex}`;
`blob` is `zlib.compress(int8 raster bytes)`, C-order, `numpy.int8`, standard
ROS occupancy values (`-1` unknown, `0..100` probability).

Deduplicated **by content hash**: slam_toolbox republishes the whole map on
every update and the renderer only needs "the grid in force at time `t`", so
each unique raster is stored once and referenced by id. The reference run holds
3 map publishes and 3 distinct rasters (no repeats to collapse); a long run
that sits still collapses many publishes to one row.

## `static_tf` rows — one per child frame

```json
{"child": "zed2_left_camera_frame", "parent": "zed2_camera_center",
 "translation": [x, y, z], "matrix": [[3x3 rotation]]}
```
Metres, and a plain rotation matrix (the quaternion is already resolved).
Chained to `base_link` at render time by `static_chain_to_base()`.
14 rows in the reference run.

## `sample:<stream>` rows

JSON arrays are **positional tuples** — order is the contract. Frames noted per
stream.

| stream | payload | units / frame |
|---|---|---|
| `pose` | `[x, y, yaw, vx]` | m, m, rad, m/s. Frame = `meta.pose_frame` (`map` for `--pose-source global`, `odom` for `local`) |
| `boundaries` | `[[nx, ny, offset], ...]` | unit normal + m. Halfspace `n·p <= offset`, **base_link**. Non-finite offsets already dropped |
| `clearance` | `2.69` (bare float) | m, forward clearance |
| `obstacles` | `[[x, y, r], ...]` | m, **base_link**. Soft-cost disks |
| `detections` | `[frame_id, [[class_id, score, x, y, z, sigma], ...]]` | m in `frame_id`; `score` 0–1; `sigma` = 1σ position stddev in m, or `null` when the covariance was zero |
| `markers` | `[[kind, ns, x, y, r, text], ...]` | `kind` is `"disk"` or `"label"`; m, **map**. Accumulated live-track set, already diffed |
| `tree` | `[active_lane, [lane_names], [lane_statuses], emergency_trip, safety_stop_active, stop_source]` | strings; `safety_stop_active` bool. Statuses are `SUCCESS`/`FAILURE`/`RUNNING`/`INVALID` |
| `solver` | `[success, status, status_message, solve_dt_sec, control_period_sec, cost, solver, n_boundary_constraints, n_obstacles, pred_x[], pred_y[], pred_yaw[], pred_v[], prediction_frame_id]` | s, s, unitless cost; `solver` e.g. `"rti"`; `pred_*` are the horizon, m/m/rad/(m/s), **odom** |
| `stop` | `[source, speed]` | `source` is `"emergency"` or `"obstacle"` (last path segment of the frame_id); m/s |
| `drive` | `[speed, steering_angle]` | m/s, rad. Commanded |
| `map` | `{"grid_id": int, "extent": [xmin, xmax, ymin, ymax]}` | m, **map**. Raster in the matching `grid` row |
| `map_to_odom` | `[x, y, yaw]` | m, m, rad. Live TF, applied per frame to odom-frame layers |

A stream with no messages in the bag simply has no `sample:` rows; the reader
still creates an empty `Stream` for it from `meta.max_age`, so downstream code
never has to special-case a missing topic.

## Padded halfspaces — derived, not stored

Only the raw halfspace is stored; the padded line is
`offset - car_radius - avoidance_margin`, computed by `draw_frame`. Storing a
padded copy would freeze one render-time choice into the archive.

But the two terms **are** properties of the run, and are recorded as such in
`meta.padding` (above), read from the run's own params snapshot at extract
time. This corrects an earlier version in which they were hard-coded argparse
defaults of 0.20 / 0.12 — a third independent copy of numbers that live in
`stack_params.yaml`, agreeing with it only by coincidence. Under that version,
re-rendering a historical run after a config change would have silently drawn
padding that run never flew with.

## Stop events — the samples are the events

Also listed separately in the work order, but there is no separate stop-event
table: `build_stop_spans()` builds the scrubber's colouring from the
`sample:stop` rows plus `--dt`, exactly as it did when reading the bag. The
data is present; it just is not pre-aggregated.

## Reading one, without ROS

```python
from f1tenth_logger.mission_render import read_extract
bag = read_extract('<run_id>.extract.parquet')
bag['streams']['pose'].at(t)      # zero-order hold, honours max_age
bag['t0'], bag['t1'], bag['manifest']
```

`read_extract` returns exactly the dict the old ROS-only `read_bag()` returned,
which is why the drawing code needed no changes and why the output is
byte-identical.
