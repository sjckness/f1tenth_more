# Wall boundaries: tracked walls as the MPC's half-plane rows

Branch `campaign-final`, Orin, Humble. Code and unit tests only — the stack was
never launched, no recorded data was analysed, nothing was pushed. Defaults are
unchanged: `boundary_source` is `costmap` and every costmap-cleanup switch is
off, so the shipped configuration behaves exactly as it did before.

---

## STEP 0 — static code audit

Read from the code on this branch. Three disagreements and one blocked premise
are flagged; none was silently fixed.

### Where `/costmap/boundaries` comes from

`f1tenth_costmap/costmap_boundary_node.py` runs a periodic timer
(`extraction_rate_hz`, 10.0 Hz since cpu-cleanup 2026-10-16) and on every tick
calls `costmap_boundary.extract_boundary_constraints` with the latched
`/slam/map` grid and the global EKF's map-frame pose
(`/ekf_global/odometry/filtered`). That function does **not** fit a line to
anything. For each of three car-frame angular windows — front `±35°`, left
`[45°, 135°]`, right `[-135°, -45°]` — it takes the **single nearest occupied
cell** (`>= occupied_threshold`, default 65) within `max_range_m` (5.0 m) and
emits a half-plane tangent to that one cell: normal pointing radially at it,
offset equal to the distance to it. Published as a `BoundaryConstraintArray`,
0–3 entries, `header.frame_id = base_link`.

**This is the mechanism the goal names.** One cell is enough. A map artefact, a
person slam_toolbox integrated once, one spurious return — each is
indistinguishable from a wall to that extraction, and each becomes a row across
the whole horizon. The gating is also permissive in the wrong direction:
`_extraction_tick` publishes on **every** tick from whatever grid and pose are
cached, however old, gated only on "has a message of each kind ever arrived"
(the periodic-publish pass removed the recency check deliberately; the node's
own docstring names the resulting tradeoff — a dead pose publisher is
indistinguishable from "no new correction yet").

Also live on this path, off by default: `use_convex_polytope` (`safe_corridor.py`),
which replaces the three half-planes with up to 8 polytope faces.

### How `mpc_solver` consumes them

- `MPC_corr.costmap_boundaries_callback` transforms each `(normal, offset)` from
  base_link into the MPC's odom world frame **on receipt**, via
  `_boundary_to_world(..., self.x, self.y, self.yaw)`, and caches the result plus
  a receipt timestamp.
- `_get_live_boundaries()` gates on `use_hard_boundary_constraints`, then on
  staleness via `_select_live_boundaries(..., odom_stale_timeout_sec)` (0.5 s).
- `solve_mpc_step` → `_solve_rti` calls
  `pad_boundary_constraints(boundaries, max_sources=boundary_max_sources)`, which
  pads with `BOUNDARY_DISABLED = (0.0, 0.0, inf)` **or truncates**, to exactly
  `boundary_max_sources` entries, so the QP's row count is identical every tick.
- One row per slot **per stage**: `normal · p_{k+1} <= offset - car_radius - margin`
  (`boundary_constraint_bounds`). **The margin is applied here and only here** —
  see "the margin is applied once" below.
- `boundary_hard=False` (the default) adds one slack variable **per slot, shared
  across stages**, boxed `0 <= s_j <= boundary_slack_max` (0.5 m) with cost
  `w·s² + w·s`. The penalty is therefore on each face's **worst** violation
  anywhere in the horizon, not the sum along it.
- Violations come back as `info["boundary_slack"]`, metres per slot.
- Infeasibility fallback: OSQP's infeasibility statuses are caught
  (`_OSQP_INFEASIBLE_STATUSES`, added after a live incident where a certificate
  of ~2e9 passed an `isfinite`-only check); `info["success"]` goes False, the
  warm start is dropped to `None`, and the caller tiles `last_u`. Before this
  pass **nothing logged which rows the solve had been given**.

### Effective values on the CAR launch path

`mpc_corr.launch.py` declares each as a `DeclareLaunchArgument` whose default is
`get_default(<key>)` from `stack_params.yaml`, and `MPC_corr.__init__` declares
the ROS parameter with `get_value(<key>)` as its in-code default. Both sides read
the same yaml, so in-code default == yaml == launch default for all four:

| key | code default | stack_params.yaml | launch | effective |
|---|---|---|---|---|
| `use_hard_boundary_constraints` | `get_value` | `true` | passes through | **true** |
| `boundary_hard` | `get_value` | `false` | passes through | **false** (soft) |
| `boundary_max_sources` | `get_value` | `3` | passes through | **3** |
| `boundary_slack_weight` | `get_value` | `10000.0` | passes through | **10000.0** |
| `use_convex_polytope` | `False` (node) | not present | `costmap_boundary_use_convex_polytope` = `'false'` | **false** |
| `car_radius` | `0.20` (literal) | `0.20` | passes through | **0.20** |
| `obstacle_safety_margin_m` / `avoidance_margin` | `0.12` (literal) | `0.12` | passes through | **0.12** |

Four things to flag rather than fix:

1. **`pad_boundary_constraints` is not a parameter.** It is the function in
   `mpc_solver.py`; the parameter that sets its `max_sources` is
   `boundary_max_sources`. The work order names the function. Everything below
   uses the real key.
2. **`boundary_slack_max` is unreachable from configuration.** It is a
   `solve_mpc_step` keyword defaulting to `BOUNDARY_SLACK_MAX_DEFAULT = 0.5` m,
   it is not a ROS parameter, it is not in `stack_params.yaml`, and `MPC_corr`
   never passes it. So the slack is hard-capped at 0.5 m on the vehicle. That
   matters: a wall the car is already more than 0.5 m inside cannot be softened
   and the QP goes infeasible. Left alone (defaults unchanged), but it is the
   one number in this path that cannot be tuned without a code edit.
3. **`boundary_slack_weight` 10000.0 is documented as "never validated on the
   vehicle"** in its own yaml description. The wall path therefore gets its own
   weight rather than inheriting it — see the derivation below.
4. **`navigation.launch.py` forwards no launch arguments.** It includes
   `mpc_corr.launch.py` with a bare `IncludeLaunchDescription` and no
   `launch_arguments`, deliberately ("needs no args passed in", its docstring).
   So `boundary_source:=walls` on a full-stack launch does **not** reach the MPC.
   See the launch section at the end for the two ways round it.

### What the multi-wall tracker publishes

`f1tenth_perception/multi_wall_tracker.py`, hosted in `lidar_front_wall_node`
behind `use_wall_tracker` (yaml default **true**, requires `enable_scene_labels`,
yaml default **true** since 2026-10-14 — so this topic **is** live on the car
launch path). Up to `max_published` = 4 tracks on `/perception/walls`
(`MultiWallTrackArray`).

- **Frame**: `header.frame_id` is the **odom** frame, `header.stamp` the scan
  stamp. Odom here is `lidar_front_wall_node`'s own `/odometry/filtered` — the
  same local EKF `MPC_corr`'s `x/y/yaw` come from.
- **Line**: `rho`, `theta` with `rho = x cos θ + y sin θ`, `rho >= 0`, θ the
  normal **from the frame origin** toward the line. **The origin is the odom
  origin, not the car** — this is the single most important fact for the
  geometry, and it is why the half-plane's orientation has to be re-decided
  against the car's position every cycle. `rho_bl`/`theta_bl` are the same line
  in base_link **at the scan stamp** (a frozen pose snapshot).
- **Endpoints**: `p0`/`p1` (odom) and `p0_bl`/`p1_bl` (base_link at the scan
  stamp) — the *measured* extent, rebuilt every step from the last
  `extent_decay_s` of observations, so an end nobody re-observes goes away.
- **Side class**: `role` (and `label`, the same value kept for compatibility) —
  `front | back | left | right | other`, the **geometric** role with hysteresis,
  not a bearing sector. `'other'` is a first-class answer: real, tracked, drawn,
  and a reference wall for nothing.
- **Source**: `source` — `SOURCE_LIDAR 0 | SOURCE_STEREO 1 | SOURCE_BOTH 2`, the
  **best evidence the track has ever had**.
- **State**: `state` — `STATE_TENTATIVE 0 | STATE_CONFIRMED 1 | STATE_COASTING 2`.
- **Distances**: `d_perp_front` (perpendicular, from `base_front`), `d_ray_front`
  (**along the heading ray from `base_front`** to the segment, NaN if it misses),
  `d_side` (from the car's side edge, left/right roles only), `is_front`.

**Two fields the work order asks for do not exist on the wire:**

- **No coasting age.** There is `age` (seconds since the track was *born*) and
  nothing else. The tracker knows the answer internally (`last_seen`,
  `odo_at_last_seen`) but does not publish it.
- **No glass flag.** `trk.glass` exists internally and goes out on
  `/perception/virtual_wall/debug` as free-form JSON, not on `/perception/walls`.

`lidar_front_wall_node._publish_walls` says explicitly why, and it is a hard
constraint, not an oversight: *"appending to a .msg makes every bag already
recorded with the old definition undeserialisable (CDR is positional), and these
recordings are the evidence base."* So this pass does **not** extend
`MultiWallTrack.msg`. Consequences:

- Coasting age is **derived on the consumer side** (`CoastClock`) from the states
  seen on `/perception/walls`, keyed on the track id — which *is* stable for one
  track's lifetime, because a coast is the same track going unmatched and only a
  re-birth changes the id. A track first seen already coasting has no
  last-confirmed stamp, so its clock starts at first sight: an under-estimate,
  which expires it *sooner* than the truth. That is the safe direction and it
  only happens when the MPC starts mid-coast.
- Glass needs no field here: the rule is "glass walls admitted like any other",
  so nothing branches on it.

### Blocked premise: STEP 4's map→odom

The work order asks for walls to be re-expressed in odom "through the current
map→odom (same pattern as the corridor anchor)", with a staleness gate on the TF
age. **There is no map→odom in this path and adding one would be wrong.** The
tracker is odom-native: `/perception/walls` is already in the same odom frame the
solver's rows are evaluated in. Pushing an odom quantity through map and back
would inject the global EKF's drift (measured at 1.88 m and 32° on the run that
motivated the corridor anchor) into a number that never had any.

What the work order is actually protecting against — a frozen snapshot — is
implemented, and more directly:

- the odom-frame `rho`/`theta`/`p0`/`p1` are used, **never** `rho_bl`/`p0_bl`
  (those are base_link at the scan stamp, i.e. exactly the stale snapshot);
- the half-plane's orientation is re-decided against the car's **current**
  `self.x/self.y` every cycle, in `_wall_boundaries`, at solve time;
- `wall_row_max_tf_age_s` gates the **wall message's own age**, and a
  `header.frame_id` that is not the MPC's odom frame withholds every row as a
  configuration error. Both withhold *all* rows with a throttled WARN.

The parameter keeps the name the work order gave it; its description in
`stack_params.yaml` says what it actually gates.

---

## What changed

### New: `mpc_controller/wall_boundaries.py` (pure, no rclpy, no f1tenth_messages)

`WallTrackView` / `WallRow` value types, and the whole rule as pure functions:
`admit_track`, `coast_margin_for`, `CoastClock`, `half_plane_from_track`,
`segment_span`, `row_is_active`, `clearance_to_path`, `merge_rows`,
`RowIdentity`, `other_wall_obstacles`, `build_wall_rows`, `check_row_capacity`,
`stale_reason`. `MPCController` is never constructed in tests (its `__init__`
opens log files and starts a timer), which is why the startup-capacity check and
the staleness gate are module-level functions rather than inline `if`s.

### `MPC_corr.py`

- `boundary_source` param, `'costmap' | 'walls' | 'none'`, validated at
  construction. `_get_live_boundaries(corridor)` dispatches on it; `'costmap'` is
  the previous code path untouched, `'none'` returns `[]`.
- `/perception/walls` subscription and the `/mpc/wall_constraints_diag`
  publisher exist **only** in `'walls'` mode — an unread subscription would
  deserialise four tracks at 10 Hz for nothing in the default configuration (the
  same reason `costmap_boundary_node`'s TF listener is now conditional).
- `wall_tracks_callback` caches plain odom views and updates the coast clock. It
  does **no** geometry: the clock must see every cycle's states including the
  ones the control loop skips, and the half-planes need the car's current pose
  and the corridor, neither of which exists in the callback.
- `_wall_boundaries(corridor)` builds the rows, publishes the diagnostic, and
  returns the `(normal, offset)` list. `_wall_path_points` is the corridor
  centreline plus the car's current position.
- Confirmed `'other'` tracks become soft `w_obs` disks, appended to a **new**
  list — never a mutation of `self.obstacles_global_live`, which also feeds
  `compute_robot_obstacle_distance` → `/mpc/min_obstacle_distance` → the mission
  `obstacle_distance_below` stop condition. A wall must not become a reason to
  stop a mission.
- `wall_row_slack_weight` replaces `boundary_slack_weight` in `'walls'` mode.
- `info['boundary_slack']` is now stored, and an infeasible solve logs the row
  ids it was given. The fallback itself is unchanged.
- Startup error (`RuntimeError`) when `boundary_source == 'walls'` and
  `max_wall_constraints > boundary_max_sources`.

### `costmap_boundary.py` / `costmap_boundary_node.py` (STEP 6, default off)

`occupied_components` (8-connected, so a diagonal wall stays one surface),
`ComponentPersistence`, `object_footprint_mask`, `filter_occupied_cells`. The
node computes components, updates persistence, builds a filtered **copy** of the
grid and feeds that to the **boundary extraction only**.
`/costmap/front_clearance` is recomputed from the raw grid on every tick,
whatever the switches say — it feeds a live mission-stopping BT condition, and
"would the car hit something going straight" is a question a single cell answers
perfectly well. A test asserts this.

### The margin is applied once

`boundary_constraint_bounds` already turns a row into
`normal · p <= offset - car_radius - avoidance_margin`. So the `offset` a wall row
carries is the **raw** odom-frame offset of the wall line, exactly as
`BoundaryConstraint.msg` requires ("offset is NOT pre-shrunk by any safety
margin"). The work order's STEP 3 asks for `… − (car half-width + safety margin)`
at the producer; doing that as well would double it and stop the car a whole
footprint early. The only producer-side shrink is `wall_row_coast_margin_m`,
which is extra conservatism for a surface nobody is currently looking at, not a
restatement of the car's footprint. `test_the_offset_is_raw_geometry_with_no_margin_baked_in`
pins this.

### One deviation from the work order's own test 1

STEP 2 says rows come only from roles `front | left | right`. STEP 7's test 1
expects a 4-wall room to give **4** rows. Those contradict: in a 6×6 room with
the car at the centre the tracker's `geometric_role` gives `front`, `left`,
`right` and **`back`** — three eligible roles and one that the stated rule
excludes. The rule was implemented as written, so a 6×6 room gives **3 rows plus
one `role_back` rejection**, reported with its reason rather than dropped
silently. The substance of test 1 (slack 0, trajectory inside the room) is
asserted. If `back` should produce a row too — it is harmless forward and the MPC
does allow `vMin = -1.0` m/s, so it would genuinely constrain a reversing
horizon — the one-line change is adding `'back'` to `ROW_ROLES`.

---

## New parameters

### `mpc_corr` (stack_params.yaml → `mpc_corr.launch.py` → node)

| key | default | what it does |
|---|---|---|
| `boundary_source` | `costmap` | `costmap` (current behaviour) / `walls` / `none` |
| `max_wall_constraints` | `4` | most wall rows shipped, nearest first; must be `<= boundary_max_sources` |
| `wall_row_max_coast_s` | `1.0` | a coasting track still rows for this long after last CONFIRMED |
| `wall_row_coast_margin_m` | `0.10` | extra standoff on a coasting row |
| `wall_row_allow_stereo_only` | `false` | `SOURCE_STEREO` tracks never row unless true |
| `other_walls_as_obstacles` | `true` | confirmed `'other'` tracks → soft `w_obs` disks |
| `wall_obstacle_spacing_m` | `0.25` | sample spacing along an `'other'` segment |
| `wall_obstacle_radius_m` | `0.0` | *additional* disk inflation; 0 = existing unified margin only |
| `wall_row_end_margin_m` | `0.3` | along-wall slack on the measured extent for the activity test |
| `wall_row_merge_angle_deg` | `10.0` | merge + cross-cycle identity tolerance (normals) |
| `wall_row_merge_offset_m` | `0.3` | merge + cross-cycle identity tolerance (offset) |
| `wall_row_slack_weight` | `2000000.0` | slack penalty on a wall row (see below) |
| `wall_row_max_tf_age_s` | `0.5` | wall-message age past which **every** row is withheld |

`boundary_source` is passed as `ParameterValue(..., value_type=str)`. An untyped
string `LaunchConfiguration` is re-parsed under YAML 1.1 rules on the way into
the node, which is how `'off'` becomes `False` and the node dies on
`InvalidParameterTypeException`. None of the three values is a YAML boolean
today; the typed form costs nothing and the next value added might be.

### `costmap_boundary_node` (node params → `costmap.launch.py`)

| key | launch arg | default |
|---|---|---|
| `component_filter` | `costmap_boundary_component_filter` | `false` |
| `min_component_length_m` | `costmap_boundary_min_component_length_m` | `0.5` |
| `min_component_scans` | `costmap_boundary_min_component_scans` | `2` |
| `exclude_tracked_object_cells` | `costmap_boundary_exclude_tracked_object_cells` | `false` |
| `object_footprint_margin_m` | `costmap_boundary_object_footprint_margin_m` | `0.12` |

`min_component_scans` counts **consecutive extraction ticks**, not scans: this
node reads the latched `/slam/map` on its own 10 Hz timer and never sees a scan.
The name is the work order's; the description says what it counts. The two
switches default false, so the thresholds carry their intended values while
being inert — turning the filter on takes one argument, not four.

---

## Slack-weight derivation: `wall_row_slack_weight = 2.0e6`

Requirement: a 0.05 m violation must dominate every other cost term by ≥ 100×.

The slack cost in `_solve_rti` is `P[s,s] += 2w` and `q[s] += w`, i.e. a term
`w·s² + w·s`. The linear part is deliberate — a pure quadratic has zero gradient
at `s = 0`, so the solver would take a free fraction of a millimetre. At
`s = 0.05`:

```
cost(0.05) = w · (0.05² + 0.05) = w · 0.0525
```

Bounding the rest of the objective at any operating point the controller sees,
from the shipped weight set (`stack_params.yaml`, after `scale_stage_weights`
with N = 20, factor 7/20):

| term | weight | effective | plausible worst error | contribution |
|---|---|---|---|---|
| `w_term` ‖x_N − pref‖² | 9.0 (terminal) | 9.0 | 3 m miss | 81 |
| `w_psi` (terminal) | 4.5 | 4.5 | 0.5 rad | 1.1 |
| `w_v` (v − vdes)², ×20 | 4.2857 | 1.50 | 2 m/s | 120 |
| `w_psi_stage`, ×20 | 4.7736 | 1.67 | 0.5 rad | 8.4 |
| `w_du_delta`, ×20 | 8.33 | 2.92 | 0.3 rad/step | 5.3 |
| `w_obs` softplus², ×20 | 8.0 | 2.80 | saturated | ~56 |
| `w_corr` | 3.5714 | 1.25 | — | small |

The largest **single** term is ~100; the **whole objective** is comfortably under
1.0e3 at any recoverable operating point. Taking the stricter reading (the total):

```
w · 0.0525 >= 100 · 1.0e3   =>   w >= 1.9e6   =>   w = 2.0e6
cost(0.05) = 2.0e6 · 0.0525 = 1.05e5   =  105 × the 1.0e3 bound   ✓
```

Why this stays numerically usable rather than just large:

- **It is bounded.** `boundary_slack_max = 0.5` m caps the term at
  `2.0e6 · (0.25 + 0.5) = 1.5e6`. It cannot run away.
- **OSQP equilibrates.** Ruiz scaling is on by default, so a ~1e6 spread between
  the slack diagonal and the single-digit stage weights is handled; the existing
  `1.0e4` already spans ~1e3.
- **Measured, not asserted.** `test_the_wall_slack_weight_keeps_the_solver_usable`
  and the whole of `TestAlreadyInsideTheMargin` solve at the shipped `2.0e6`:
  a satisfiable wall costs `< 1e-3` m of slack, and a wall the car is already
  0.07 m inside (`offset 0.25 < car_radius + margin = 0.32`) returns
  `success=True`, a finite in-bounds command, and a reported slack inside the
  0.5 m cap. If OSQP ever did degrade at this magnitude, `2.0e5` still clears the
  per-term reading (100 × the largest single term) by construction.

---

## Launch lines to try in Gazebo (6×6 room, person in the middle)

**`navigation.launch.py` forwards no launch arguments** (STEP 0, flag 4), so
`boundary_source:=…` on a full-stack launch is silently ignored. Two ways round
it, neither of which changes a default:

**A — the A/B itself: bring the sim+perception stack up, then run the MPC alone**

```bash
# terminal 1 -- sim + perception + BT, MPC included but about to be replaced
ros2 launch f1tenth_bringup stack_bringup.launch.py

# terminal 2 -- stop the stack's own mpc_corr, then one of:

# baseline today: nearest-occupied-cell rows from /slam/map
ros2 launch f1tenth_control mpc_corr.launch.py \
    boundary_source:=costmap

# the new path: half-planes from the tracked walls. boundary_max_sources MUST
# be raised to 4 or the node refuses to start (it would otherwise truncate one
# of the four rows silently).
ros2 launch f1tenth_control mpc_corr.launch.py \
    boundary_source:=walls boundary_max_sources:=4

# no rows at all: the clean A/B baseline, soft w_obs only
ros2 launch f1tenth_control mpc_corr.launch.py \
    boundary_source:=none
```

**B — a whole-stack run in one mode**: set `boundary_source` (and
`boundary_max_sources: 4` for `walls`) in `src/f1tenth_params/config/stack_params.yaml`
and launch `stack_bringup.launch.py` normally. This is how the stack's other
non-forwarded keys are set.

What to watch:

```bash
ros2 topic echo /mpc/wall_constraints_diag      # walls mode: rows, reasons, slack
ros2 topic echo /perception/walls               # the tracks behind them
ros2 topic echo /mpc/solver_status              # infeasible solves
ros2 topic echo /costmap/boundaries             # costmap mode: the three tangents
```

The per-row diagnostic carries `row_id`, `track_ids`, `role`, `state`, `normal`,
`offset`, `clearance_m`, `coast_age_s`, `active` and a `reason`
(`''` when active, otherwise `role_other` / `role_back` / `tentative` /
`stereo_only` / `coast_expired` / `path_past_segment_end` /
`over_max_wall_constraints`), plus `withheld` when the whole cycle was dropped.
`slack_m_last_solve` is from the **previous** solve — the rows have to exist
before the solve that prices them — and row ids are stable across cycles, so the
two still line up.

To try the costmap cleanup instead:

```bash
ros2 launch f1tenth_costmap costmap.launch.py \
    costmap_boundary_component_filter:=true \
    costmap_boundary_min_component_length_m:=0.5 \
    costmap_boundary_min_component_scans:=2 \
    costmap_boundary_exclude_tracked_object_cells:=true
```

---

## Tests

`src/f1tenth_control/mpc_controller/test/test_wall_boundaries.py` — **52 pass**.
`src/f1tenth_costmap/test/test_costmap_component_filter.py` — **32 pass**.
Synthetic throughout; three scenarios do real light OSQP solves at the shipped
slack weight.

| work order | where | result |
|---|---|---|
| 1. 4-wall room: rows, slack 0, inside | `TestFourWallRoom` | 3 rows + 1 `role_back` rejection (see deviation above); slack 0, trajectory inside every row |
| 2. short wall, open end stays feasible | `TestOpenEnd` | row inactive past the end, the gap solves, end margin decides the boundary case |
| 3. tentative → no row; confirmed `other` → soft obstacle | `TestAdmission` | both, plus tentative-`other` → nothing, and role tested before evidence |
| 4. coast 0.5 s → row + margin; 2 s → none | `TestCoasting` | both, plus the mid-coast-start clock |
| 5. two tracks one wall → one restrictive row | `TestMerge` | 0.67 survives, both ids reported, opposite walls stay apart |
| 6. id re-birth → same row id, no slack jump | `TestRowIdentity` | same `row_id`, identical geometry, identical solver slack |
| 7. stale map→odom → no rows + warn | `TestStaleness` | stale / never-received / wrong-frame each withhold all rows with a reason (gating the **wall message** age — see STEP 0) |
| 8. 4 walls, pad 3 → startup error | `TestRowCapacity` | `check_row_capacity` returns the error `MPCController.__init__` raises; the cap and the surplus reason are tested too |
| 9. inside the margin → solve returns, slack > 0 | `TestAlreadyInsideTheMargin` | `success=True`, finite in-bounds command, slack > 1e-3 and inside the 0.5 m cap |
| 10. costmap filter: island ignored, wall kept | `TestIslandVersusWall`, `TestNodeWiring` | pure **and** node-level, and `front_clearance` proven unfiltered |

Plus `test_mirrored_constants_match_the_message`, which keeps
`wall_boundaries.py`'s copies of `MultiWallTrack`'s enumerations honest (a
mirror nobody checks is a trap).

### Existing suites

- `mpc_controller`: **782 pass, 11 fail**. All 11 fail identically with HEAD's
  `stack_params.yaml` restored — verified by swapping the file back and re-running
  the eight affected modules. They are pre-existing config/test drift
  (`mpc_prediction_dt_s` is 0.1 against a test expecting 0.2; `mpc_w_du_delta`
  8.33 → effective 2.9155 against a note quoting 3.0; and six others). This pass
  adds **zero** new failures.
- `f1tenth_costmap`: **256 pass**, plus the 32 new.
- `f1tenth_control`: **7 pass**.
- Both launch files generate; all 13 + 5 new arguments are declared.
- `colcon build --symlink-install` clean for `f1tenth_params`, `f1tenth_costmap`,
  `mpc_controller`, `f1tenth_control`.

### Lint, measured against HEAD

- **flake8 (`ament_flake8`, mpc_controller): 77 → 77.** Unchanged. Every new and
  modified file is individually clean at the 99-column limit.
- **pep257 (`ament_pep257`, mpc_controller): 1197 → 1230 (+33).** Both numbers are
  failing; the test is red at HEAD. **Every one of the 33 is `D213`** ("multi-line
  docstring summary should start at the second line"), the package's single most
  common code already (266 at HEAD) and the one that is mutually exclusive with
  `D212`, the convention this codebase uses everywhere. The new files were written
  in the house style rather than as the only `D213`-compliant files in the
  package. If a flat number is wanted instead, it is a mechanical reflow of 33
  docstrings in `wall_boundaries.py`, `test_wall_boundaries.py` and the six new
  `MPC_corr` methods — say the word.

---

## Not touched

`/costmap/front_clearance` (asserted unfiltered by test), the proximity e-stop,
the BT stop logic, `MultiWallTrack.msg` (bag compatibility),
`boundary_slack_max`, `navigation.launch.py`'s include pattern, and every
existing default.
