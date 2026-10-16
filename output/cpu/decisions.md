# Decisions taken unattended (H1..H5)

## H1 — the C refuses a non-uniform bearing grid, and the dispatcher sends it to the Python

The specified API takes `angle_min` and `angle_increment`, so the C can only
express a uniform grid -- which is exactly what a `LaserScan` is. But
`extract_segments` is also called with arbitrary bearings: `test_extraction.py
::test_merge_keeps_parallel_offset_surfaces_apart` synthesises points and sorts
their `arctan2`, and `corridor_perception.replay` re-derives bearings from a
cloud. Rebuilding a uniform grid from `a[0]` and a mean increment there is a
silently RESAMPLED scan, and it changed that test's answer from 2 segments to 3.

Decision: `extraction_c.uniform_angles()` checks the array against the
progression to 1e-12 rad; the dispatcher runs the reference when it fails, and
the C itself raises rather than guessing. Two tests pin both halves.

- Alternative A: pass the angle array to the C as well. Rejected: the point
  fixes the API, and two sources of truth for the same numbers is how the
  bit-exactness argument gets lost.
- Alternative B: let the C resample. Rejected outright -- it is wrong, and it
  was wrong silently.
- Closest to current behaviour: the reference's answer, which is what this is.

## H1 — both backends are compared on the same float32 values

The C's API takes float32 ranges. To measure the ALGORITHM rather than float32
rounding of the input, `extract_backends_equiv.py` and the backend tests feed
the Python the same float32 values widened back to float64. In production the C
sees the float32 of a float64 range, a ~1e-7 m difference on a range, four
orders of magnitude under the 1e-4 tolerance.
