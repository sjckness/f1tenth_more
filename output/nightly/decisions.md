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
## F6 — the toggle had to be wired through the supervisor to work at all

`enable_mission_logger` already existed and `mission_logger.launch.py` already
gated both of its actions on it, so flipping the default was a one-line change.
But the supervisor launches each component with ONLY the args in
`components.yaml`, so the documented campaign command
(`supervisor_bringup.launch.py enable_mission_logger:=true`) would have been
silently ignored.

Decision: declare the argument on `supervisor_bringup.launch.py` and inject it
into the component's args in the supervisor, copying the existing
`_apply_calibration_override()` pattern exactly.

- Alternative A: put `args: {enable_mission_logger: 'true'}` in
  `components.yaml`. Rejected: that pins recording ON for every supervisor run,
  which is the opposite of the point.
- Alternative B: document `ros2 launch f1tenth_logger mission_logger.launch.py
  enable_mission_logger:=true` as a separate fourth terminal. Rejected: it takes
  the node out of the supervisor's restart/watchdog management, which IS the
  current behaviour for it.
- Closest to current behaviour: B is closest in code, A is closest in effect.
  The chosen option keeps the node exactly where it is today (a supervised
  component) and changes only whether it is asked for -- which is what the
  point asks for. The logger's own logic was not touched.

## F6 — three pre-existing lint failures in f1tenth_bringup are NOT mine

`test_flake8`, `test_pep257` and `test_copyright` fail in `f1tenth_bringup`.
Verified pre-existing: they fail identically with my changes stashed. The cause
is visible in the output -- flake8 is walking the untracked `install_base573/`
and `build_base573/` trees in the repo root, e.g.
`./install_base573/zed_wrapper/share/zed_wrapper/launch/zed_camera.launch.py:48:1:
W293`. Left alone, as instructed.
