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
