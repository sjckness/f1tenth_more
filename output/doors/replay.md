# Semantic doors: offline replay on corridor10m_20261016T144537

`python3 tools/semantic_door_replay.py --out output/doors ~/bags/corridor10m_20261016T144537`

Branch `doors-enter`. No stack was run: this is
`f1tenth_costmap/semantic_doors.py` driven over the bag's recorded
`/perception/walls`, `/scan` and `/odometry/filtered`, with
`stack_params.yaml`'s own `semantic_doors_*` values.

## Result

**No door, and no near miss.**

| | |
|---|---|
| cycles run | 6523 (one per `/scan`) |
| `/perception/walls` messages | 704 |
| gaps evaluated | 57 935 |
| **doors confirmed** | **0** |
| tracks held at the end | 0 (not one candidate was ever born) |
| cost per cycle (linus, not the Orin) | median **0.078 ms**, p95 0.125 ms |

Every one of the 57 935 gaps failed a GEOMETRY gate. Not one reached the
wall-end tests or the passage tests.

| code | gate | n |
|---|---|---|
| `foot_outside` | case B: the perpendicular's foot is not inside the other wall's extent | 26 012 |
| `angle` | the two walls are neither collinear (A) nor square (B) | 23 017 |
| `width` | the gap is outside 0.70-1.40 m | 5 096 |
| `offset` | case A: the two facing ends are more than 0.15 m apart across the line | 3 810 |

## Why zero is the right answer here

**No gap of doorway width ever existed between two tracked walls.** Of the
57 935 gaps, the number whose width fell in 0.70-1.40 m was **0** — in either
case, under any code. The distribution says where they were instead:

- case A, rejected on `angle`: 79 gaps under 0.5 m, 110 between 0.5 and
  0.7 m, then nothing until 2 m; 7430 of them over 10 m;
- case A, rejected on `offset`: all 3810 were 3 m or wider;
- case B, rejected on `width`: all 5096 were over 5 m — a wall end measured
  perpendicularly onto the far wall of a 10 m corridor.

That is the shape of a plain corridor: two long side walls, a wall across the
end, and the ~0.6 m stubs the tracker splits off at the ends. There is no
doorway in it to find, and the detector did not invent one.

## What this bag does NOT test

1. **The wall-end and passage gates were never reached**, so `end_fov`,
   `end_occluded`, `few_beams`, `not_through`, `on_segment` and `object` are
   covered by the unit tests
   (`src/f1tenth_costmap/test/test_semantic_doors.py`) and by nothing else
   yet. A bag recorded in front of a real open doorway is what would exercise
   them.
2. **The object veto had nothing to veto with.** The in-the-gap object test
   reads the semantic layer's own confirmed tracks, which exist only once
   that node has run; replaying it here would be a replay of a different node
   (`tools/semantic_merge_replay.py` is that one). `objects` was empty, so a
   door reported by this replay has passed three of the four gates, not four.
3. **The frame is `odom`, not `map`.** This bag carries `/odometry/filtered`
   and not `/ekf_global/odometry/filtered`, so the coordinates above are odom
   coordinates. The geometry is unaffected — a door is found in the same
   physical place — but on the car the same numbers are map coordinates, and
   that is what the `door_<id>` TF is published in.

## Files

- `output/doors/corridor10m_20261016T144537.candidates.csv.gz` — every gap on
  every cycle: stamp, case, the two wall ids, width, centre, code, code
  group, beams, through fraction. Gzipped because the raw file is 3.9 MB and
  nothing else in `output/` is over half a megabyte; the tool rewrites it
  uncompressed, so regenerate rather than edit.
- The per-cycle door CSV is not here: it was empty (no door was confirmed).
- `output/doors/corridor10m_20261016T144537.summary.json` — the table above,
  machine-readable.
