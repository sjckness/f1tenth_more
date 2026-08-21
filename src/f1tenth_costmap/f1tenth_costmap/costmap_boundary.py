"""costmap_boundary.py -- nearest-occupied-cell hard boundary extraction from
slam_toolbox's own /slam/map (OccupancyGrid). Pure functions, no rclpy
dependency, independently unit-testable -- same "pure logic separate from
ROS glue" split this package's own costmap_renderer.py/semantic_layer.py
already use.

Part of the dual-EKF + costmap-derived-MPC-boundaries pass: retires
wall_detector_node.py (ZED RANSAC plane fit) and lidar_boundary_node.py
(raw /scan line fit) in favor of one source derived from the SLAM map
itself -- the map is, after all, already the accumulated record of every
lidar/ZED return the stack has ever integrated, so re-deriving "where is
the nearest wall" from raw sensor data a second time (as the two retired
nodes each independently did) is redundant once a real occupancy grid
exists. See costmap_boundary_node.py's own module docstring for why this
pass builds around /slam/pose's own currently-known-broken (never
publishes) state defensively rather than blocking on it.

DESIGN -- deliberately simpler than wall_detector_node's RANSAC plane fit
or lidar_boundary_node's total-least-squares line fit: per the task's own
spec ("nearest occupied cell within an angular window per direction"), this
does NOT fit a line/plane through multiple occupied cells at all. For each
of the three directions (front/left/right, car-frame angular windows), it
finds the SINGLE nearest occupied cell within that window and builds a
BoundaryConstraint whose normal points radially from the robot straight at
that cell and whose offset is exactly the distance to it -- i.e. a tangent
line perpendicular to that radial direction, at the nearest known obstacle
point. This is a conservative, correct hard boundary (the robot's own
position, at the origin, trivially satisfies normal.(0,0)=0 <= offset; any
point further along that same radial direction than the occupied cell
violates it) without the extra complexity of fitting a line through
multiple cells the way the two retired nodes did for their own raw-sensor-
point inputs -- appropriate here since a SINGLE nearest occupied cell is
exactly what the task asks for, not a full wall geometry reconstruction.

UNKNOWN CELLS (-1 in OccupancyGrid.data): never treated as occupied (the
occupied_threshold comparison excludes them structurally, since -1 is
below any positive threshold) and never treated as "confirmed clear"
either -- nearest_occupied_in_window simply never selects them, the same
way it never selects a below-threshold FREE cell. A window with zero
occupied cells (whether because the area is genuinely open or because it's
entirely unexplored) returns None either way -- "no data for this
direction", not a false-clear or false-tight assertion. This matches the
task's own explicit "unknown/unexplored cells: treat as 'no data', never
as false-clear or false-tight" requirement without needing a separate
unknown-cell code path at all.

SIGN CONVENTION -- matches f1tenth_messages/BoundaryConstraint.msg's own
documented convention exactly (verified against that message's own field
comments, same discipline wall_detector_node.py's own module docstring
followed for its analogous derivation): normal points AWAY from the robot,
TOWARD the obstacle; offset = perpendicular distance from the robot's own
frame origin to the boundary, un-shrunk by any safety margin (the
consumer's job, per that message's own note).

FRAME -- published base_link-relative, NOT map-relative, even though the
occupancy grid itself is map-frame and the fused pose driving this
extraction is the GLOBAL (map-frame) EKF's output. extract_boundary_
constraints does the map-frame -> car/base_link-frame rotation internally
(using the robot's own map-frame yaw) so its output already matches
wall_detector_node's/lidar_boundary_node's own base_link-relative
convention -- this is deliberate: mpc_corr's existing _boundary_to_world/
_boundary_callback_common machinery (which transforms an incoming base_
link-relative BoundaryConstraintArray into MPC's own world/odom frame using
its own ODOM-frame x/y/yaw) stays completely unchanged for the Part 5
source swap (see MPC_corr.py's own docstring for why MPC's internal state
is odom-frame, not map-frame, even post dual-EKF -- it never subscribed to
the global EKF directly and doesn't need to for this).
"""

import math

import numpy as np

# Sentinel returned by nearest_occupied_in_window when a window has zero
# occupied cells (genuinely clear so far as this map knows, OR entirely
# unexplored -- see module docstring's own "UNKNOWN CELLS" section for why
# both cases collapse to the same "no data" result here, not two distinct
# code paths).
NO_OCCUPIED_CELL = None


def yaw_from_quaternion(q) -> float:
    """Planar yaw (radians) from a geometry_msgs/Quaternion -- standard
    atan2 formula, valid for the roll=pitch=0 planar case this stack always
    operates in. Same formula as MPC_corr.py's own quaternion_to_yaw() /
    check_stop_condition.py's own _quaternion_to_yaw() -- reimplemented
    here rather than imported, same "two lines of math, not worth a cross-
    package dependency" precedent check_stop_condition.py's own module
    docstring already established for this exact function."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _crop_indices(
        width: int, height: int, resolution: float, origin_x: float, origin_y: float,
        robot_x: float, robot_y: float, max_range_m: float):
    """Axis-aligned (map-frame) column/row index bounds for a box of
    half-width `max_range_m` centered on the robot's own map-frame
    position, clipped to the grid's own extent. Cheap prefilter applied
    BEFORE any per-cell trig -- since nothing outside max_range_m can ever
    be selected anyway (see nearest_occupied_in_window's own range_mask),
    computing bearing/distance for the whole grid every extraction tick
    would be pure waste on a large, mostly-irrelevant-this-tick map.
    Returns (col_min, col_max, row_min, row_max), each end exclusive-safe
    for direct numpy slicing (col_max/row_max already +1'd and clipped)."""
    col_min = int(math.floor((robot_x - max_range_m - origin_x) / resolution))
    col_max = int(math.ceil((robot_x + max_range_m - origin_x) / resolution)) + 1
    row_min = int(math.floor((robot_y - max_range_m - origin_y) / resolution))
    row_max = int(math.ceil((robot_y + max_range_m - origin_y) / resolution)) + 1
    col_min = max(0, min(col_min, width))
    col_max = max(0, min(col_max, width))
    row_min = max(0, min(row_min, height))
    row_max = max(0, min(row_max, height))
    return col_min, col_max, row_min, row_max


def nearest_occupied_in_window(
        grid_data, width: int, height: int, resolution: float,
        origin_x: float, origin_y: float, robot_x: float, robot_y: float,
        robot_yaw: float, window_min_rad: float, window_max_rad: float,
        occupied_threshold: float, max_range_m: float):
    """Nearest occupied cell (arr value >= occupied_threshold) whose CAR-
    FRAME bearing (0 = front, + = left, matching every other node in this
    stack's convention) falls in [window_min_rad, window_max_rad] (signed --
    caller passes e.g. (-front_max, front_max) for a front cone or
    (side_min, side_max) for the left window, (-side_max, -side_min) for
    the mirrored right window, same convention lidar_boundary_node.py's own
    _classify_side already established) and whose distance is <=
    max_range_m.

    Returns (car_dx, car_dy, distance) for the winning cell -- ALREADY IN
    CAR/BASE_LINK FRAME (not map frame) -- or None if no cell in the window
    clears the occupied threshold within range (see module docstring's
    "UNKNOWN CELLS" section for why this is the same result whether the
    window is genuinely clear or simply unexplored).

    `grid_data` may be any int-convertible sequence (a real
    OccupancyGrid.data array/array.array, or a plain Python list in tests) --
    same flexibility costmap_renderer.py's own occupancy_to_rgb already
    provides, and cast via the same np.int16 (not int8, which would
    silently misinterpret slam_toolbox's own -1/0-100 convention on
    overflow) precedent that function already established.
    """
    col_min, col_max, row_min, row_max = _crop_indices(
        width, height, resolution, origin_x, origin_y, robot_x, robot_y, max_range_m)
    if col_min >= col_max or row_min >= row_max:
        return None  # robot's own max_range_m box doesn't overlap the grid at all.

    arr = np.asarray(grid_data, dtype=np.int16).reshape(height, width)
    sub = arr[row_min:row_max, col_min:col_max]

    cols = np.arange(col_min, col_max)
    rows = np.arange(row_min, row_max)
    # Cell CENTERS (+0.5), same convention costmap_renderer.py's own world_
    # to_pixel implicitly matches (nearest-cell rounding) -- not the cell's
    # own (col, row) corner.
    cell_x = origin_x + (cols.astype(np.float64) + 0.5) * resolution
    cell_y = origin_y + (rows.astype(np.float64) + 0.5) * resolution
    grid_x, grid_y = np.meshgrid(cell_x, cell_y)  # each (row_max-row_min, col_max-col_min)

    dx = grid_x - robot_x
    dy = grid_y - robot_y
    cos_yaw, sin_yaw = math.cos(robot_yaw), math.sin(robot_yaw)
    # R(-yaw): map-frame delta -> car/base_link-frame delta -- the inverse
    # of _boundary_to_world's own R(yaw) (MPC_corr.py), consistent with
    # that function's own derivation comment.
    car_dx = cos_yaw * dx + sin_yaw * dy
    car_dy = -sin_yaw * dx + cos_yaw * dy

    dist = np.hypot(car_dx, car_dy)
    car_angle = np.arctan2(car_dy, car_dx)

    occupied_mask = sub >= occupied_threshold
    range_mask = dist <= max_range_m
    if window_min_rad <= window_max_rad:
        window_mask = (car_angle >= window_min_rad) & (car_angle <= window_max_rad)
    else:
        # Wrap-around window (not currently used by costmap_boundary_node's
        # own front/left/right windows, all of which stay within a single
        # (-pi, pi] span -- supported here anyway so this function's own
        # contract doesn't silently assume otherwise for a future caller).
        window_mask = (car_angle >= window_min_rad) | (car_angle <= window_max_rad)

    combined = occupied_mask & range_mask & window_mask
    if not np.any(combined):
        return None

    # argmin over the masked distances only -- a naive full-array argmin
    # would happily return an out-of-window/unoccupied cell if it happened
    # to have a small raw distance value.
    masked_dist = np.where(combined, dist, np.inf)
    flat_idx = int(np.argmin(masked_dist))
    r, c = np.unravel_index(flat_idx, masked_dist.shape)
    return float(car_dx[r, c]), float(car_dy[r, c]), float(dist[r, c])


def boundary_from_nearest_point(car_dx: float, car_dy: float):
    """Convert one nearest-occupied-cell result (car-frame dx, dy) into a
    (normal_x, normal_y, offset) BoundaryConstraint tuple -- see module
    docstring's "SIGN CONVENTION" section. Returns (0.0, 0.0, math.inf) --
    the same naturally-inert sentinel wall_detector_node.py's own
    _wall_boundary_from_track uses under mpc_solver.py's pad_boundary_
    constraints convention -- for the practically-unreachable case of the
    nearest cell sitting exactly on the robot's own origin (distance=0,
    undefined direction), rather than dividing by zero."""
    k = math.hypot(car_dx, car_dy)
    if k < 1e-9:
        return 0.0, 0.0, math.inf
    return car_dx / k, car_dy / k, k


def extract_boundary_constraints(
        grid_data, width: int, height: int, resolution: float,
        origin_x: float, origin_y: float, robot_x: float, robot_y: float,
        robot_yaw: float, front_facing_max_rad: float,
        side_window_min_rad: float, side_window_max_rad: float,
        occupied_threshold: float, max_range_m: float) -> dict:
    """Full per-direction extraction -- {'front': ..., 'left': ..., 'right':
    ...}, each value either a (normal_x, normal_y, offset) BoundaryConstraint
    tuple (car/base_link-frame) or None ("no occupied cell in this
    direction's window within range" -- see module docstring). Front uses a
    SYMMETRIC cone (-front_facing_max_rad, +front_facing_max_rad) around the
    car's own forward axis, same front_facing_max_deg convention wall_
    detector_node.py's own front_clearance eligibility already established.
    Left/right reuse lidar_boundary_node.py's own side_window_min_rad/
    side_window_max_rad convention exactly (left: [min, max], right:
    [-max, -min]) -- the SAME lateral windows already matched to this rig's
    forward-facing lidar's own coverage, not re-derived independently."""
    results = {}
    windows = {
        'front': (-front_facing_max_rad, front_facing_max_rad),
        'left': (side_window_min_rad, side_window_max_rad),
        'right': (-side_window_max_rad, -side_window_min_rad),
    }
    for direction, (win_min, win_max) in windows.items():
        hit = nearest_occupied_in_window(
            grid_data, width, height, resolution, origin_x, origin_y,
            robot_x, robot_y, robot_yaw, win_min, win_max,
            occupied_threshold, max_range_m)
        if hit is None:
            results[direction] = None
            continue
        car_dx, car_dy, _dist = hit
        results[direction] = boundary_from_nearest_point(car_dx, car_dy)
    return results


def front_clearance_from_extraction(extraction: dict, max_range_m: float) -> float:
    """Scalar front_clearance (meters) from extract_boundary_constraints'
    own 'front' result -- the distance component of that same nearest-cell
    lookup, not a separate computation (single source of truth, matches
    wall_detector_node.py's own precedent of deriving front_clearance and
    front_wall_boundary from the SAME selected track rather than two
    independent lookups that could disagree).

    Returns max_range_m + 1.0 (a finite "clear at least this far, so far as
    this map knows" value -- NOT math.inf) when the front window has no
    occupied cell within range: unlike wall_detector_node's own +inf-for-
    genuinely-clear convention (that node had no search radius limit at
    all, so +inf was the honest answer), this extraction is deliberately
    range-limited (max_range_m) -- a bare +inf here would overstate
    confidence ("clear all the way to infinity") when in truth this only
    searched out to max_range_m. A value just past the search radius reads
    correctly on both sides: any real stop_condition threshold well under
    max_range_m still evaluates true-when-clear exactly as before, while
    the reported number stays an honest reflection of what was actually
    searched."""
    front = extraction.get('front')
    if front is None:
        return max_range_m + 1.0
    _nx, _ny, offset = front
    return offset
