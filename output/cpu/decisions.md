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
## H2 — the cycle-count thresholds KEEP their counts; the report states the time

The point allows either. Counts kept, for the reason the rules give: it is what
the car does today and what the tests pin, and a hits-in-window rule expressed
in seconds changes meaning when the cycle rate varies (which is exactly what a
CPU-bound node does).

And the arithmetic favours it. Today the node is not running at 40 Hz: the
nightly audit measured **4.2 observations a second** because it was CPU-bound.
Decimation 4 at the nominal 40 Hz scan rate is **10 Hz**, so every count-based
threshold becomes FASTER in wall-clock than it is today, not slower. The table
is in the report.

- Alternative: convert confirm_hits / stereo_confirm_hits /
  negative_miss_to_kill / rect_reject_cycles to seconds. Rejected for now: four
  new parameters, four changed semantics, and no measurement yet of the rate
  they would be set against. Revisit once H5 has measured the real cycle rate
  on the car with H1 in.

## H2 — a skipped scan HOLDS the tracker's last verdict

The node publishes its contract on every scan (its own docstring makes that a
hard contract). On a scan the tracker skips, `_front_d_ray`,
`_tracker_has_data` and `_tracked_wall` are left exactly as the last processed
scan left them, so the contract repeats a verdict at most 75 ms old.

- Alternative: publish NO_DATA on skipped scans. Rejected: it would flap the
  contract three scans in four, and NO_DATA means "the sensors are not telling
  us", which is false here.
- The debug payload carries `tracker_skipped` so a consumer can tell a held
  verdict from a fresh one.

## H2 — every replay forces decimation 1

A bag holds one recorded tracker CYCLE per `/perception/virtual_wall/debug`
message (~3.2 a second on this recording), not the 40 scans a second the car
sees. Decimating that again would compare a different set of scans against the
recording, and would leave three cycles in four reporting the previous cycle's
`tracker_ms`. `WALL_TRACKER_DECIMATION` overrides it for a run that wants to
see the decimation itself.
## H3 — 'other' takes no slot, so it never reaches the semantic grid

The point is explicit, and this is the consequence worth writing down: since
H3 an `other` track is not on `/perception/walls` at all, and
`semantic_layer_node` only rasterises what is published. So the `ROLE_VALUE
['other'] = 50` cell added last night (F4) is now unreachable through the
tracker path -- correct for an unknown role arriving from anywhere else, dead
for this one.

- Not changed: F4's value is still the right answer for a role this module has
  never heard of, and removing it would put `other` back on `back`.
- The `other` tracks are still visible: `/perception/virtual_wall/debug`'s
  `tracks` list carries every live track with its role when
  `wall_debug_tracks` is on, and the top-down video draws them.
- If the costmap should carry them as occupied-but-unlabelled surfaces, that is
  a decision about the costmap, not about the cap, and it needs its own point.

## H3 — FRONT takes every slot it needs, sides get one each

The order is: all FRONT tracks (nearest along the heading first), then the
nearest LEFT, then the nearest RIGHT, then BACK, then any remaining side
walls so the budget is never left unspent. "FRONT always (if present)" read as
"front is never the thing that drops off the end", which is the defect: a
front wall at rank 4 of 7 was being hidden on 36 of 206 cycles.
