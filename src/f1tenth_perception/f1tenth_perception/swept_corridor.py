"""Forward clearance along the steering arc -- pure geometry, no ROS, no I/O.

The single place clearance is decided. swept_clearance_node.py turns /scan and
ZED depth into points and calls clearance() for both; neither sensor path has
any clearance logic of its own. Everything here takes numpy arrays and floats
and is unit-tested in test/test_swept_corridor.py.

WHY THE ARC AND NOT A ROTATED CONE
----------------------------------
A cone rotated by the steering angle points along the tangent, and the tangent
leaves the arc at once: at delta 0.278 rad on L 0.305 m the radius is 1.07 m,
so one metre of travel turns the heading ~53 degrees. It misses obstacles on
the inside of the turn and flags ones the car curves away from. The corridor
is therefore the region the body actually sweeps.

FRAME
-----
base_link, x forward, y left, rear axle at the origin. That is where the URDF
puts base_link (f1tenth_description/urdf/base.xacro: "at the rear axle", rear
wheels at x = 0) and the point the odometry's no-sideslip kinematics track.
Some docstrings elsewhere (description.launch.py, sensors.xacro,
camera.launch.py) say base_link is "centered between the axles"; the URDF's
joints say otherwise. swept_clearance_node's rear_axle_x_m absorbs the
difference if a measurement ever proves those docstrings right.

FOOTPRINT
---------
The rectangle x in [BODY_REAR_X_M, BODY_FRONT_X_M], |y| <= BODY_HALF_WIDTH_M:
the axis-aligned bounds of the URDF chassis mesh (meshes/chassis.stl with
base.xacro's visual origin and scale), i.e. 0.525 m long and 0.272 m wide,
front bumper 0.118 m ahead of the 0.325 m front axle. Wheels (track 0.2 m,
width 0.045 m) sit inside it. Not a tape-measure calibration.

`margin` widens the corridor sideways only. The front face is not moved, so
straight-ahead clearance is the real gap to the bumper, not to a margin line.

THE TURN, IN CURVATURE
----------------------
kappa = tan(delta) / wheelbase, + = left, turn centre (0, 1/kappa). Every
formula below is written in kappa rather than R = 1/kappa, so nothing diverges
as delta -> 0 and the straight corridor is the exact kappa = 0 case of the
curved one, not a separately-coded approximation.

That is why there is no rectangular fallback at |delta| < 1e-2 rad. A switch
there cannot be continuous: at delta = 1e-2 the rear axle's arc has already
drifted kappa * x^2 / 2 = 0.41 m sideways at 5 m (6.6 cm at 2 m), so a point
near the corridor edge is inside the rectangle and outside the arc, and
clearance would jump as delta crosses the threshold. The only switch is at
|kappa| < STRAIGHT_CURVATURE, below which the curved formulas would divide
0 by 0 and the arc differs from the rectangle by under 1e-7 m within 5 m.

WHAT clearance() RETURNS
------------------------
The arc length the REAR AXLE travels before the body first touches a point --
the quantity a consumer converts to a stopping distance, since odometry speed
is rear-axle speed. Straight ahead that is x - BODY_FRONT_X_M.

For a point at radius d from the turn centre and angle phi, measured about the
centre from the rear axle in the direction of travel,

    phi = atan2(x, |R| - q),   q = sign(kappa) * y   (q: lateral toward the centre)

the leading edge of the (margin-widened) body at that radius sits at angle

    alpha_front = asin(front_x / d)                  front face leads
    alpha_side  = acos((|R| - half_width') / d)      inner side face leads
                                                     (d below the inner front
                                                      corner's radius)

and contact happens after the rear axle has turned phi - alpha, i.e.

    s = |R| * (phi - alpha).

With a zero-length body (front_x = rear_x = 0) alpha is 0 and this is exactly
the |R| * atan2(p.x, sign(R) * (R - p.y)) sketch of the work order, with band
|R| +- half_width'.

A point is reachable only if d lies in the band the leading half of the body
sweeps, [inner, outer] from corridor_bounds(). A rectangle whose length spans
the rear axle is closest to the turn centre at its inner flank level with
that axle, so inner = |R| - half_width' exactly. The inside rear corner is
FARTHER from the centre, not nearer: with the centre on the rear-axle line,
hypot(rear_x, |R| - half_width') > |R| - half_width'. Where the footprint
matters is the other side: the outer FRONT corner swings out to
hypot(front_x, |R| + half_width'), 0.08 m past |R| + half_width' at full lock.
test_swept_corridor.py checks both against a sampled footprint.

The leading half (x >= 0) is enough going forward as long as the nose is
longer than the tail (front_x >= -rear_x). Tail swing on the outer side is
then under 4 mm at full lock, and anything that close is caught by the floor.

WHAT IS NEVER COUNTED
---------------------
  * Behind the rear axle (x < 0): discarded, including the far side of the
    turn circle. The travel angle is taken on atan2's (-pi, pi], so no point
    is ever reached by wrapping around; the horizon is under half a turn.
  * Inside the body footprint (not the margin-widened one): the car itself --
    the LiDAR housing in the ZED image, a return off the chassis. Ignored by
    the corridor AND by the floor. An obstacle can only be there after contact.

THE FLOOR
---------
Any point within absolute_min_clearance of the body, measured as the straight
line to the footprint rectangle, returns that gap whatever the steering says --
sideways and alongside included. Points behind the rear bumper are excluded
from it too, so a wall behind the car never reads as forward clearance.
Measured to the body rather than to the sensor, the floor trips at or before
the old LiDAR cone check for every point outside the footprint.
"""

import math
from typing import NamedTuple

import numpy as np

BODY_FRONT_X_M = 0.443
BODY_REAR_X_M = -0.082
BODY_HALF_WIDTH_M = 0.136

# Below this |curvature| [1/m] the straight (kappa = 0) formulas are used.
# See the module docstring for why this is not the 1e-2 rad of the work order.
STRAIGHT_CURVATURE = 1e-9


class CorridorBounds(NamedTuple):
    """The band the leading half of the body sweeps.

    curvature: signed [1/m], + = left. radius: signed turn radius [m], +-inf
    when straight. inner/outer: radii [m] about the turn centre, both inf when
    straight, where the corridor is |y| <= half_width + margin instead.
    """
    curvature: float
    radius: float
    inner: float
    outer: float


def _curvature(delta, wheelbase, corridor_half_width):
    if not math.isfinite(delta):
        raise ValueError(f'steering angle must be finite, got {delta!r}')
    if wheelbase <= 0.0:
        raise ValueError(f'wheelbase must be positive, got {wheelbase!r}')
    kappa = math.tan(delta) / wheelbase
    if abs(kappa) < STRAIGHT_CURVATURE:
        return 0.0
    # A turn centre inside the corridor's own width is outside what any
    # steering on this car reaches (52 degrees at the default geometry) and
    # makes "inner flank" meaningless. Clamp to the flank rather than raise:
    # a clearance node must keep publishing.
    limit = (1.0 - 1e-9) / corridor_half_width
    return math.copysign(min(abs(kappa), limit), kappa)


def corridor_bounds(delta, *, wheelbase, half_width, margin, front_x=BODY_FRONT_X_M):
    hw = half_width + margin
    kappa = _curvature(delta, wheelbase, hw)
    if kappa == 0.0:
        return CorridorBounds(0.0, math.inf, math.inf, math.inf)
    rho = 1.0 / abs(kappa)
    # With q measured toward the centre (0, rho): the leading half
    # [0, front_x] x [-hw, hw] is nearest the centre at its inner flank level
    # with the rear axle, (0, hw) -- rho > hw by _curvature's clamp -- and
    # farthest at its outer front corner, (front_x, -hw).
    inner = rho - hw
    outer = math.hypot(front_x, rho + hw)
    return CorridorBounds(kappa, math.copysign(rho, kappa), inner, outer)


def first_contact_arc_length(x, y, delta, *, wheelbase, half_width, margin,
                             front_x=BODY_FRONT_X_M):
    """Rear-axle arc length [m] until the margin-widened body reaches each
    point, or -inf where it never does going forward (outside the swept band,
    behind the rear axle). Negative finite values mean the point is already
    inside the widened leading half. x, y: 1-D arrays in the rear-axle frame.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    hw = half_width + margin
    kappa = _curvature(delta, wheelbase, hw)

    if kappa == 0.0:
        return np.where(np.abs(y) <= hw, x - front_x, -np.inf)

    u = abs(kappa)
    q = math.copysign(1.0, kappa) * y

    # Each g_* is u * (difference of squared radii), rearranged so no term
    # of size 1/u is ever subtracted from another -- see the module docstring.
    g_inner = u * x * x + (hw - q) * (2.0 - u * (q + hw))            # d >= |R| - hw
    g_outer = u * (front_x ** 2 - x * x) + (hw + q) * (2.0 + u * (hw - q))  # d <= outer
    g_front = u * (x * x - front_x ** 2) + (hw - q) * (2.0 - u * (q + hw))  # front face leads

    phi = np.arctan2(u * x, 1.0 - u * q)
    ud_sq = (u * x) ** 2 + (1.0 - u * q) ** 2
    alpha_front = np.arctan2(u * front_x, np.sqrt(np.maximum(ud_sq - (u * front_x) ** 2, 0.0)))
    alpha_side = np.arctan2(np.sqrt(np.maximum(u * g_inner, 0.0)), 1.0 - u * hw)
    alpha = np.where(g_front >= 0.0, alpha_front, alpha_side)

    s = (phi - alpha) / u
    reachable = (g_inner >= 0.0) & (g_outer >= 0.0) & (x >= 0.0)
    return np.where(reachable, s, -np.inf)


def clearance(points_xy, delta, *, wheelbase, half_width, margin, max_range,
              absolute_min_clearance, front_x=BODY_FRONT_X_M, rear_x=BODY_REAR_X_M) -> float:
    """Forward clearance [m] along the arc steering angle `delta` [rad, + = left]
    traces, for an (N, 2) array of points in the rear-axle frame.

    min over reachable points of first_contact_arc_length, capped at
    `max_range`; lowered to the straight-line gap of any point within
    `absolute_min_clearance` of the body. Points inside the body footprint
    and non-finite points are ignored. See the module docstring.
    """
    pts = np.asarray(points_xy, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 2:
        if pts.size == 0:
            return float(max_range)
        raise ValueError(f'points_xy must be (N, 2), got shape {pts.shape}')
    finite = np.isfinite(pts).all(axis=1)
    x = pts[finite, 0]
    y = pts[finite, 1]
    if x.size == 0:
        return float(max_range)

    inside_body = (x > rear_x) & (x < front_x) & (np.abs(y) < half_width)
    result = float(max_range)

    s = first_contact_arc_length(x, y, delta, wheelbase=wheelbase, half_width=half_width,
                                 margin=margin, front_x=front_x)
    corridor = (s >= 0.0) & ~inside_body
    if corridor.any():
        result = min(result, float(s[corridor].min()))

    gap = np.hypot(np.maximum(x - front_x, 0.0), np.maximum(np.abs(y) - half_width, 0.0))
    near = (x >= rear_x) & ~inside_body & (gap < absolute_min_clearance)
    if near.any():
        result = min(result, float(gap[near].min()))

    return result
