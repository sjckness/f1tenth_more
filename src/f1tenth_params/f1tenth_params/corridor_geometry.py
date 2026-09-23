"""corridor_geometry.py -- the corridor's own geometry, as one pure function.

Pure: numpy and math only, no rclpy, no node state. Same "pure logic separate
from ROS glue" split campaign_status.py and wall_turn.py already use.

WHY THIS IS ITS OWN MODULE. The corridor is not a point list, it is three
curves, and until this commit the only place that knowledge existed was ~110
lines in the middle of MPCController.build_straight_corridor. Anything else
that wanted to draw or re-evaluate a corridor -- the test-campaign plotter,
an offline analysis -- had to reimplement it, and the day the planner's shape
changed the two would silently disagree. build_straight_corridor now calls
corridor_curves() for exactly the arrays it used to build inline, so there is
one implementation and a divergence is not expressible.

THE THREE CURVES
----------------
1. THE CENTRELINE, a heading ramp integrated numerically:

       tau(u)   = clip((u - u_start) / (u_end - u_start), 0, 1)
       shape(u) = 3 tau^2 - 2 tau^3            # smoothstep / cubic Hermite
       theta(u) = psi_start + dpsi * shape(u)
       xc, yc   = (x0, y0) + cumsum(cos theta * ds), cumsum(sin theta * ds)

   A smoothly ramped heading that is then integrated is a generalised
   clothoid (the clothoid proper is the LINEAR-heading case; this is the
   smoothstep case). It has no closed form, and -- this is the part that
   matters for anyone re-evaluating a logged corridor -- np.cumsum here is a
   RIGHT-ENDPOINT RIEMANN SUM, not the exact integral. It is first-order
   accurate, so its value depends on `n`: evaluating the same definition at a
   higher n gives a different, better curve, which is NOT what the planner
   drove. Re-evaluate at the stored n and the reproduction is bit-for-bit.
   (The gap is not large -- 7.6e-5 m at the endpoint for n=120, L=3.0 -- but
   "reproduces the planner" and "is more accurate than the planner" are
   different claims and only the first one is wanted.)

2 & 3. THE TWO WALLS, cubic Beziers, exact closed form, four control points
   each:

       P_L0 = C0 + w0 n0    CL0 = P_L0 + h L e0
       P_L1 = C1 + w1 n1    CL1 = P_L1 - h L e1
       P_R0 = C0 - w0 n0    CR0 = P_R0 + h L e0
       P_R1 = C1 - w1 n1    CR1 = P_R1 - h L e1

   with C0 = (x0, y0), C1 the END of the integrated centreline, n = (-sin psi,
   cos psi), e = (cos psi, sin psi) and h the handle fraction. The walls are
   therefore exactly evaluable at any density, but their control points still
   depend on C1 and so on `n` through it.

THE BOUNDARY is the two walls. It is NOT "centreline plus a half-width": w0
and w1 differ (the corridor is a widening funnel), and a Bezier is not a
constant offset of an integrated clothoid, so `half_width` below is a
DERIVED, per-sample quantity, not a parameter.
"""

import math

import numpy as np

__all__ = [
    'corridor_curves',
    'corridor_curves_to_pose',
    'POSE_HANDLE_FRAC',
    'bezier',
    'wrap_pi',
    'ramp_span_for',
    'CORRIDOR_HANDLE_FRAC',
    'SMOOTHSTEP_PEAK_SLOPE',
]

#: Bezier handle length as a fraction of the corridor length. A bare literal
#: in build_straight_corridor since that code was written; named here because
#: it now has to be logged for a corridor to be reproducible.
CORRIDOR_HANDLE_FRAC = 0.55

#: Handle length of the POSE-TO-POSE centreline, as a fraction of the straight
#: distance between the two poses. 0.40 from a search over the 24 archived
#: object rebuilds: peak curvature is flat between about 0.35 and 0.50 (the
#: per-rebuild optimum averages 0.45 leaving the car and 0.35 arriving, and
#: buys under 10% of kappa over a symmetric 0.40), so the symmetric value is
#: taken and the asymmetry is left as the tuning it would be.
POSE_HANDLE_FRAC = 0.40


def wrap_pi(angle):
    """Shortest-branch wrap to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def bezier(p0, c0, c1, p1, u):
    """Cubic Bezier at parameters ``u``, as an (len(u), 2) array."""
    uu = np.asarray(u, dtype=float)[:, None]
    p0 = np.asarray(p0, dtype=float)
    c0 = np.asarray(c0, dtype=float)
    c1 = np.asarray(c1, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    return ((1 - uu) ** 3 * p0
            + 3 * (1 - uu) ** 2 * uu * c0
            + 3 * (1 - uu) * uu ** 2 * c1
            + uu ** 3 * p1)


#: Peak of the smoothstep's derivative, 6 tau (1 - tau), at tau = 0.5. The
#: heading ramp's steepest point, and so the corridor's peak curvature:
#:     kappa_max = SMOOTHSTEP_PEAK_SLOPE * |dpsi| / ramp_span
SMOOTHSTEP_PEAK_SLOPE = 1.5


def ramp_span_for(dpsi, r_min):
    """Shortest ramp span [m] whose peak curvature still fits inside r_min.

    From kappa_max = 1.5 |dpsi| / span <= 1 / r_min.
    """
    return SMOOTHSTEP_PEAK_SLOPE * abs(float(dpsi)) * float(r_min)


def corridor_curves(x0, y0, psi_start, psi_end, length, n,
                    dpsi=None, u_start=0.0, u_end=0.40,
                    w0=0.4333, w1=0.7667,
                    handle_frac=CORRIDOR_HANDLE_FRAC,
                    r_min=None, length_ref=None):
    """Every array build_straight_corridor's corridor dict carries.

    ``dpsi`` is the SIGNED, possibly unwrapped rotation the corridor asks for.
    Pass it explicitly for a wall_turn, where the sign and any rotation past
    pi are not recoverable from ``psi_end`` alone; leave it None and it falls
    back to the shortest-branch wrap of ``psi_end - psi_start``, which is what
    every non-turn branch wants.

    ``r_min`` turns on the RAMP-SPAN CLAMP. The ramp span is (u_end - u_start)
    * length, i.e. a FRACTION of the corridor, so the same heading change is
    crammed into a shorter and shorter arc as the corridor shortens, and peak
    curvature runs away as 1/length. With r_min given, u_end is widened
    (never narrowed, never past 1.0) until the span is at least
    ramp_span_for(dpsi, r_min). Reported as ``u_end_eff`` /  ``ramp_clamped``.

    If even u_end = 1.0 is too short the geometry is simply not drivable from
    here, and that is REPORTED (``feasible`` False) rather than repaired:
    capping dpsi instead would silently move the corridor's end heading, and
    on the object branch that is the one thing holding the end cap on the goal.
    The reference is soft, so an infeasible ask costs steering saturation, not
    a constraint violation -- but nothing should have to infer that from the
    curvature.

    ``length_ref`` turns on the WIDTH RATE. Without it the funnel reaches w1 at
    u = 1 whatever the corridor's length, so a short corridor is wider than it
    is long -- 10 of the 24 archived object rebuilds are, the shortest being
    0.74 m long against a 1.53 m far-end width, which is a degenerate shape to
    hand a cross-track cost. With it, the funnel opens at a fixed rate per
    metre instead:

        w1_eff = w0 + (w1 - w0) * min(length / length_ref, 1)

    identical at length == length_ref (the design point), narrower below it.

    Returns a dict of numpy arrays plus the Bezier control points, so a caller
    that wants to log the definition does not have to re-derive them.
    """
    n = int(n)
    if n < 2:
        raise ValueError(f'a corridor needs at least 2 samples, got {n}')
    length = float(length)

    u = np.linspace(0.0, 1.0, n)
    s = length * u

    if dpsi is None:
        dpsi = wrap_pi(psi_end - psi_start)
    dpsi = float(dpsi)

    # ---- width rate ----------------------------------------------------
    w1_eff = float(w1)
    if length_ref is not None and float(length_ref) > 0.0:
        w1_eff = w0 + (w1 - w0) * min(length / float(length_ref), 1.0)

    # ---- ramp-span clamp -----------------------------------------------
    u_end_eff = float(u_end)
    ramp_clamped = False
    feasible = True
    if r_min is not None and length > 0.0:
        need = ramp_span_for(dpsi, r_min)
        u_need = u_start + need / length
        if u_need > u_end_eff:
            u_end_eff = min(1.0, u_need)
            ramp_clamped = True
        feasible = (u_end_eff - u_start) * length + 1e-12 >= need
    u_end = u_end_eff
    w1 = w1_eff

    tau = np.clip((u - u_start) / max(u_end - u_start, 1e-6), 0.0, 1.0)
    shape = 3.0 * tau ** 2 - 2.0 * tau ** 3
    theta = psi_start + dpsi * shape

    ds = np.zeros_like(s)
    ds[1:] = np.diff(s)

    xc = x0 + np.cumsum(np.cos(theta) * ds)
    yc = y0 + np.cumsum(np.sin(theta) * ds)

    c0_pt = np.array([xc[0], yc[0]], dtype=float)
    c1_pt = np.array([xc[-1], yc[-1]], dtype=float)

    n0 = np.array([-math.sin(psi_start), math.cos(psi_start)], dtype=float)
    n1 = np.array([-math.sin(psi_end), math.cos(psi_end)], dtype=float)
    e0 = np.array([math.cos(psi_start), math.sin(psi_start)], dtype=float)
    e1 = np.array([math.cos(psi_end), math.sin(psi_end)], dtype=float)

    p_l0 = c0_pt + w0 * n0
    p_r0 = c0_pt - w0 * n0
    p_l1 = c1_pt + w1 * n1
    p_r1 = c1_pt - w1 * n1

    k = handle_frac * length
    cl0 = p_l0 + k * e0
    cl1 = p_l1 - k * e1
    cr0 = p_r0 + k * e0
    cr1 = p_r1 - k * e1

    left = bezier(p_l0, cl0, cl1, p_l1, u)
    right = bezier(p_r0, cr0, cr1, p_r1, u)

    x_l, y_l = left[:, 0], left[:, 1]
    x_r, y_r = right[:, 0], right[:, 1]

    dx = np.gradient(xc)
    dy = np.gradient(yc)
    dn = np.maximum(np.sqrt(dx ** 2 + dy ** 2), 1e-9)
    tx = dx / dn
    ty = dy / dn

    return {
        'xc': xc, 'yc': yc,
        'xL': x_l, 'yL': y_l,
        'xR': x_r, 'yR': y_r,
        'tx': tx, 'ty': ty,
        'nx': -ty, 'ny': tx,
        'halfWidth': 0.5 * np.sqrt((x_l - x_r) ** 2 + (y_l - y_r) ** 2),
        'Pend': np.array([xc[-1], yc[-1]], dtype=float),
        'dpsi': dpsi,
        # what the two clamps actually did, so a log records the geometry that
        # was built rather than the one that was asked for
        'u_end_eff': float(u_end_eff),
        'w1_eff': float(w1_eff),
        'ramp_clamped': bool(ramp_clamped),
        'feasible': bool(feasible),
        'kappa_max': (SMOOTHSTEP_PEAK_SLOPE * abs(dpsi)
                      / max((u_end_eff - u_start) * length, 1e-9)),
        # the control points, so corridor_payload can log a definition that
        # reproduces these walls without re-deriving C1 from the centreline
        'ctrl_left': [p_l0.tolist(), cl0.tolist(), cl1.tolist(), p_l1.tolist()],
        'ctrl_right': [p_r0.tolist(), cr0.tolist(), cr1.tolist(), p_r1.tolist()],
    }


def corridor_curves_to_pose(x0, y0, psi_start, x1, y1, psi_end, n,
                            handle_a=POSE_HANDLE_FRAC,
                            handle_b=POSE_HANDLE_FRAC,
                            w0=0.4333, w1=0.7667,
                            handle_frac=CORRIDOR_HANDLE_FRAC,
                            r_min=None, length_ref=None):
    """The same corridor dict, for a centreline that ENDS ON A GIVEN POSE.

    corridor_curves integrates a heading ramp for a given LENGTH: where it
    ends is an output. This one is the boundary-value form -- start pose
    (x0, y0, psi_start), end pose (x1, y1, psi_end), both honoured exactly --
    which is what an approach to a target needs, because the corridor's end is
    the place the car is being sent to and "0.25 m past it on average" is a
    different instruction.

    WHY A CUBIC BEZIER AND NOT THE RAMP FAMILY. The ramp's heading is monotone
    from psi_start to psi_end, so its mean direction lies strictly between the
    two: it can reach a point in the psi_end direction from the start only by
    going straight. The target bearing IS psi_end here (that is what psi_c is),
    so the ask is exactly the case the ramp cannot express -- measured over the
    24 archived object rebuilds, 23 are unreachable by any (length, ramp
    placement) pair. The cubic Bezier through two poses always exists, and it
    is already the family this module's WALLS are built from.

    THE PRICE, and the reason ``feasible`` matters more here than in the ramp
    case. Landing on a pose whose tangent points back along the chord forces
    the curve out and back, so peak curvature grows as the heading error over
    the remaining distance -- kappa ~ |dpsi| / d. Over the same 24 rebuilds it
    fits inside the car's R_min on every rebuild beyond 2.0 m, on 8 of 9 beyond
    1.55 m, and on 9 of 12 beyond 1.25 m; inside a metre it is hopeless (up to
    12.8 1/m against a limit of 1.047). A caller must therefore be ready to
    fall back, and this function REPORTS rather than repairs: ``feasible`` is
    False and ``kappa_max`` says by how much.

    ``handle_a``/``handle_b`` are the Bezier handle lengths as fractions of the
    straight distance between the poses -- leaving the start and arriving at
    the end respectively. The curve is sampled by ARCLENGTH, not by the Bezier
    parameter, so the returned samples are evenly spaced the way the ramp
    family's are and anything advancing along s behaves identically.

    Returns the same keys as :func:`corridor_curves` plus ``length`` (the
    centreline's arclength, which here is an output) and ``handle_a`` /
    ``handle_b``. ``u_end_eff``/``ramp_clamped`` are reported as the ramp
    concepts they are: not applicable, so 1.0 and False.
    """
    n = int(n)
    if n < 2:
        raise ValueError(f'a corridor needs at least 2 samples, got {n}')

    p0 = np.array([float(x0), float(y0)])
    p3 = np.array([float(x1), float(y1)])
    chord = float(np.hypot(*(p3 - p0)))
    e_start = np.array([math.cos(psi_start), math.sin(psi_start)])
    e_end = np.array([math.cos(psi_end), math.sin(psi_end)])
    # A degenerate chord has no pose-to-pose curve; the handles would collapse
    # and every derivative below would be 0/0.
    a = max(float(handle_a) * chord, 1e-6)
    b = max(float(handle_b) * chord, 1e-6)
    p1 = p0 + a * e_start
    p2 = p3 - b * e_end

    # Dense in the Bezier parameter first: arclength sampling needs a curve to
    # measure before it can resample it, and curvature is read off the same
    # dense pass so a peak between two output samples is not missed.
    dense = max(8 * n, 512)
    t = np.linspace(0.0, 1.0, dense)[:, None]
    pts = ((1 - t) ** 3 * p0 + 3 * (1 - t) ** 2 * t * p1
           + 3 * (1 - t) * t ** 2 * p2 + t ** 3 * p3)
    d1 = (3 * (1 - t) ** 2 * (p1 - p0) + 6 * (1 - t) * t * (p2 - p1)
          + 3 * t ** 2 * (p3 - p2))
    d2 = 6 * (1 - t) * (p2 - 2 * p1 + p0) + 6 * t * (p3 - 2 * p2 + p1)
    speed = np.maximum(np.hypot(d1[:, 0], d1[:, 1]), 1e-12)
    kappa = np.abs(d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / speed ** 3

    seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
    s_dense = np.concatenate([[0.0], np.cumsum(seg)])
    length = float(s_dense[-1])

    s_out = np.linspace(0.0, length, n)
    xc = np.interp(s_out, s_dense, pts[:, 0])
    yc = np.interp(s_out, s_dense, pts[:, 1])
    # The ends are interpolation-exact only up to floating point; the whole
    # point of this function is that they are the poses asked for.
    xc[0], yc[0] = p0
    xc[-1], yc[-1] = p3

    kappa_max = float(kappa.max())
    feasible = True if r_min is None else kappa_max <= 1.0 / float(r_min)

    w1_eff = float(w1)
    if length_ref is not None and float(length_ref) > 0.0:
        w1_eff = w0 + (w1 - w0) * min(length / float(length_ref), 1.0)

    # The walls are the SAME construction as the ramp family's: they depend on
    # the two end poses, the two half-widths and the handle fraction, and on
    # the centreline only through its endpoint. So a corridor built here is
    # the same object as any other -- one centreline of a different kind
    # inside identical walls -- and nothing downstream has to know which.
    c0_pt = np.array([xc[0], yc[0]], dtype=float)
    c1_pt = np.array([xc[-1], yc[-1]], dtype=float)
    n0 = np.array([-math.sin(psi_start), math.cos(psi_start)])
    n1 = np.array([-math.sin(psi_end), math.cos(psi_end)])
    p_l0 = c0_pt + w0 * n0
    p_r0 = c0_pt - w0 * n0
    p_l1 = c1_pt + w1_eff * n1
    p_r1 = c1_pt - w1_eff * n1
    k = handle_frac * length
    cl0 = p_l0 + k * e_start
    cl1 = p_l1 - k * e_end
    cr0 = p_r0 + k * e_start
    cr1 = p_r1 - k * e_end
    u = np.linspace(0.0, 1.0, n)
    left = bezier(p_l0, cl0, cl1, p_l1, u)
    right = bezier(p_r0, cr0, cr1, p_r1, u)
    x_l, y_l = left[:, 0], left[:, 1]
    x_r, y_r = right[:, 0], right[:, 1]

    dx = np.gradient(xc)
    dy = np.gradient(yc)
    dn = np.maximum(np.sqrt(dx ** 2 + dy ** 2), 1e-9)
    tx = dx / dn
    ty = dy / dn

    return {
        'xc': xc, 'yc': yc,
        'xL': x_l, 'yL': y_l,
        'xR': x_r, 'yR': y_r,
        'tx': tx, 'ty': ty,
        'nx': -ty, 'ny': tx,
        'halfWidth': 0.5 * np.sqrt((x_l - x_r) ** 2 + (y_l - y_r) ** 2),
        'Pend': np.array([xc[-1], yc[-1]], dtype=float),
        'dpsi': wrap_pi(psi_end - psi_start),
        'length': length,
        'handle_a': float(handle_a),
        'handle_b': float(handle_b),
        'u_end_eff': 1.0,
        'w1_eff': float(w1_eff),
        'ramp_clamped': False,
        'feasible': bool(feasible),
        'kappa_max': kappa_max,
        'ctrl_centre': [p0.tolist(), p1.tolist(), p2.tolist(), p3.tolist()],
        'ctrl_left': [p_l0.tolist(), cl0.tolist(), cl1.tolist(), p_l1.tolist()],
        'ctrl_right': [p_r0.tolist(), cr0.tolist(), cr1.tolist(), p_r1.tolist()],
    }
