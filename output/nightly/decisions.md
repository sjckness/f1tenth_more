# Decisions taken unattended

The rule for every one of these: take the option closest to the car's CURRENT
behaviour, write it down, continue.

## F1 — the tolerance params are the columns', not new ones of their own

The point says "the same quadratic model as the columns (base + quad*d², same
params)". Taken literally: the rectangle now reads
`lidar_front_wall_wall_obs_tol_base_m` and `..._quad_per_m2` directly, and the two
rect-only tolerance parameters (`wall_rect_tol_base_m`, `wall_rect_tol_frac`) are
GONE rather than left as dead aliases.

- Alternative considered: keep separate rect params defaulted to the column
  values. Rejected: two numbers for one sensor error can only ever drift apart,
  and the point says "same params", not "same defaults".
- Closest to current behaviour? Neither, strictly -- the parameter set changes
  either way. This is the reading that cannot silently diverge.

## F1 — what counts as a "verifiable cycle" for the rejection streak

A cycle with `coverage >= coverage_min`. Cycles below it neither advance nor
reset the streak, so a wall that simply leaves the camera's field of view does
not accumulate rejections while it is unseen.

- Alternative: count every cycle, resetting on low coverage. Rejected: that makes
  a wall at the edge of the frame unrejectable in practice, and the streak would
  measure how long the camera looked rather than what it saw.
