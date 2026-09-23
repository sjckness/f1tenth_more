"""Payloads of mpc_corr's two test-campaign topics: /mpc/status and /corridor.

Pure: no rclpy. Both are std_msgs/String JSON in the shape
f1tenth_logger's test_campaign logger reads (see its logger_node.py):

    /mpc/status  after EVERY solve
                 {status, solve_time_ms, cost, iterations, success, solver,
                  status_code, status_message, horizon}
    /corridor    every time build_straight_corridor() produces a new corridor
                 {id, polygon: [[x, y], ...], source, frame_id, ...}

``status`` is a small closed vocabulary rather than the backend's own text, so
the logger can count feasibility with one rule for both backends. Only
``solved`` counts as solved there; ``solved_inaccurate`` is kept distinct
because mpc_corr itself treats it as usable (it keeps the warm start), which
the campaign may or may not want to count.

``horizon`` is the predicted state trajectory of THIS solve -- the same
info["x_pred"] that /mpc/solver_status already carries as pred_x/pred_y/...,
plus the control sequence that produced it. It rides on /mpc/status rather
than on a topic of its own because the campaign logger already subscribes
here, and because a horizon is only meaningful beside the solve that made it:
one message, one solve, one plan. See :func:`horizon_payload`.

Nothing here feeds back into control: MPC_corr builds these from values the
tick has already computed, after the solve, and publishing them is wrapped so
a failure can only lose a message.
"""

import math

__all__ = ['corridor_payload', 'horizon_payload', 'mpc_status_payload',
           'solve_status_label']

# osqp.SolverStatus values (OSQP 1.x, same numbering as 0.6's status_val).
# Literal ints so this module stays importable without osqp.
_OSQP_LABELS = {
    1: 'solved',
    2: 'solved_inaccurate',
    3: 'infeasible',            # primal infeasible
    4: 'infeasible',            # primal infeasible, inaccurate
    5: 'infeasible',            # dual infeasible
    6: 'infeasible',            # dual infeasible, inaccurate
    7: 'max_iter',
    8: 'timeout',
    9: 'non_convex',
    10: 'interrupted',
    11: 'unsolved',
}

# scipy.optimize SLSQP exit modes: 0 success, 9 iteration limit.
_SLSQP_MAX_ITER = 9


def _finite_or_none(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def solve_status_label(info, solver):
    """One word for how a solve ended, from solve_mpc_step's ``info``.

    A backend that reports success on a solution mpc_corr then rejects (the
    RTI result was non-finite) is ``failed``: the label follows what the node
    did with the solve, not what the backend claimed about it.
    """
    success = bool(info.get('success', False))
    try:
        code = int(info.get('status', -1))
    except (TypeError, ValueError):
        code = -1
    if solver == 'slsqp':
        if success:
            return 'solved'
        return 'max_iter' if code == _SLSQP_MAX_ITER else 'failed'
    if code == -1:
        return 'no_solve'
    label = _OSQP_LABELS.get(code, 'failed')
    if label in ('solved', 'solved_inaccurate') and not success:
        return 'failed'
    return label


#: Decimal places the horizon arrays are rounded to. 1e-4 m / 1e-4 rad is two
#: orders below the centimetre scale any prediction-error question is asked at,
#: and it keeps one solve near 700 bytes of JSON: this rides on /mpc/status at
#: the CONTROL rate (10 Hz at ts=0.1), not at the 1 Hz corridor rate, which is
#: why the arrays are rounded at all and why the inputs are the only extras.
_HORIZON_DP = 4


def _rounded(value):
    """Finite float rounded for the wire, or None."""
    value = _finite_or_none(value)
    return None if value is None else round(value, _HORIZON_DP)


def horizon_payload(info, ts, frame_id='odom'):
    """The predicted horizon of one solve, or None when it produced none.

    ``x``/``y``/``yaw``/``v`` are ``info["x_pred"]``: the horizon states rolled
    forward through the TRUE nonlinear model, identical to the pred_* arrays
    /mpc/solver_status already carries and for the same reason (see
    MpcSolverStatus.msg). One entry per step k = 1..N. The CURRENT state x0 is
    NOT included -- a consumer drawing a path starting at the car prepends its
    own pose, exactly as that message's readers do.

    ``steer``/``accel`` are ``info["zopt"]``, the control sequence that produced
    those states: the flat [delta_0, a_0, delta_1, a_1, ...] both backends
    return, split and aligned index-for-index with the states, so entry k is the
    input applied to REACH state k. They cost nothing (already computed, no
    second rollout) and they are what separates "the MPC planned a bad path"
    from "it planned a good one and the steering never got there".

    ``ts`` is the control period, carried because it is the one thing the arrays
    cannot be read without: step k happened at the solve time + (k + 1) * ts.
    A reader that has to guess this cannot line a horizon up against what the
    car then did, which is the entire point of logging it.

    ``frame_id`` is the MPC's own world frame -- 'odom', the same frame as
    /corridor's polygon and /mpc/corridor_markers, because x_pred is rolled
    forward from x0 and x0 is odom-frame. Carried per message rather than
    assumed, so a consumer that draws corridors and horizons together can
    assert they match instead of trusting that they do.

    ALL-OR-NOTHING on the states, matching MpcSolverStatus: a solve whose
    x_pred is missing, short or non-finite returns None rather than a partial
    or NaN-padded trajectory. That is the normal state on a failed solve, not
    an error. The INPUTS are treated more leniently -- a usable state
    trajectory with an unusable zopt still publishes, with steer/accel null --
    because the states are the deliverable and the inputs are the extra.
    """
    x_pred = info.get('x_pred')
    try:
        n = len(x_pred)
    except TypeError:
        return None
    if n == 0:
        return None

    xs, ys, yaws, vs = [], [], [], []
    for row in x_pred:
        try:
            state = [_rounded(row[i]) for i in range(4)]
        except (IndexError, KeyError, TypeError, ValueError):
            return None
        if any(value is None for value in state):
            return None
        xs.append(state[0])
        ys.append(state[1])
        yaws.append(state[2])
        vs.append(state[3])

    steer, accel = None, None
    zopt = info.get('zopt')
    try:
        usable = zopt is not None and len(zopt) >= 2 * n
    except TypeError:
        usable = False
    if usable:
        steer = [_rounded(zopt[2 * k]) for k in range(n)]
        accel = [_rounded(zopt[2 * k + 1]) for k in range(n)]
        if any(v is None for v in steer) or any(v is None for v in accel):
            steer, accel = None, None

    return {
        'frame_id': str(frame_id),
        'ts': _finite_or_none(ts),
        'n': n,
        'x': xs,
        'y': ys,
        'yaw': yaws,
        'v': vs,
        'steer': steer,
        'accel': accel,
    }


def mpc_status_payload(info, solve_dt_sec, solver, ts=None, frame_id='odom'):
    """The /mpc/status JSON object for one solve.

    ``cost`` and ``iterations`` are null when the backend reported none (a
    NaN cost is not a zero cost); ``solve_time_ms`` is the same wall time
    /mpc/solver_status carries as solve_dt_sec.

    ``horizon`` is :func:`horizon_payload`, and is null on a solve that
    produced no usable prediction -- which is most failed solves, so a reader
    must handle the null rather than assume every status line carries a plan.
    ``ts`` is the caller's control period; passing it is what makes the
    horizon's steps placeable in time, and leaving it out publishes the
    geometry with a null ``ts`` rather than dropping it.
    """
    iterations = info.get('iterations')
    try:
        iterations = int(iterations)
    except (TypeError, ValueError):
        iterations = None
    if iterations is not None and iterations < 0:
        iterations = None
    try:
        status_code = int(info.get('status', -1))
    except (TypeError, ValueError):
        status_code = -1
    return {
        'status': solve_status_label(info, solver),
        'solve_time_ms': _finite_or_none(solve_dt_sec * 1000.0),
        'cost': _finite_or_none(info.get('cost')),
        'iterations': iterations,
        'success': bool(info.get('success', False)),
        'solver': str(solver),
        'status_code': status_code,
        'status_message': str(info.get('status_message', info.get('message', ''))),
        'horizon': horizon_payload(info, ts, frame_id),
    }


#: Vertex rounding of the sampled boundary, in decimal places. 6 dp is 1e-6 m,
#: two orders below the 6.5e-5 m worst-case error the 120-sample polygon itself
#: carries against an exact evaluation of the walls (measured at the documented
#: 0.763 rad wall_turn ask, the tightest corridor the stack asks for). The
#: previous 4 dp put the rounding floor at 1e-4 m, i.e. LARGER than the
#: sampling error it sat on top of, which made the polygon the limiting term
#: for no reason. The function definition below is never rounded.
_POLYGON_DP = 6


def _xy_or_none(point):
    """[x, y] as finite floats, or None -- never a half-finite pair."""
    if point is None:
        return None
    try:
        x = _finite_or_none(point[0])
        y = _finite_or_none(point[1])
    except (IndexError, KeyError, TypeError):
        return None
    return None if x is None or y is None else [x, y]


def corridor_payload(corridor, corridor_id, frame_id, source, odom_topic=None):
    """The /corridor JSON object for one freshly built corridor.

    TWO DESCRIPTIONS OF THE SAME CORRIDOR, and the distinction is the whole
    point of the v2 schema:

    ``definition`` is the SOURCE OF TRUTH -- every number
    corridor_geometry.corridor_curves needs to reproduce the planner's own
    arrays exactly, at full float precision. A consumer that wants the curves
    evaluates them; it does not interpolate the samples.

    ``object_shape`` and ``ref_step`` are per-rebuild facts rather than
    geometry: which shape the object branch chose, and how far the reference
    moved since the previous corridor (centreline, tangent at the car, Pend).
    The reference step is a wobble candidate in its own right -- at 1 Hz with
    the car at ~0.45 m/s the solver is handed a visibly re-laid reference every
    second -- and nothing was measuring it.

    ``polygon`` is a SAMPLED boundary, kept because the clearance computation
    needs a polygon and because a v1 reader still works. It is the left wall
    walked forward then the right wall walked back, closed implicitly -- the
    same order robot_logger's corridor_from_centerline uses -- in the frame the
    walls were built in. ``sample_step_m`` records the arclength spacing it was
    taken at. Non-finite vertices are dropped rather than serialised as NaN.

    The centreline is NOT in the polygon and never was; it comes back only from
    the definition. That is what made the archived v1 runs unreproducible.
    """
    left = zip(corridor['xL'], corridor['yL'])
    right = list(zip(corridor['xR'], corridor['yR']))[::-1]
    polygon = []
    for x, y in list(left) + right:
        x, y = _finite_or_none(x), _finite_or_none(y)
        if x is not None and y is not None:
            polygon.append([round(x, _POLYGON_DP), round(y, _POLYGON_DP)])
    payload = {
        'id': int(corridor_id),
        'polygon': polygon,
        'source': str(source),
        'frame_id': str(frame_id),
        'psi_ref': _finite_or_none(corridor.get('psiRef')),
        'length_m': _finite_or_none(corridor.get('L')),
        'object_mode': bool(corridor.get('objectMode', False)),
    }

    # The object shape and the per-rebuild reference step ride at the top
    # level, not inside `definition`: they describe THIS rebuild's relationship
    # to the previous one and to the branch that built it, not the function the
    # definition evaluates. A reader wanting only the geometry can ignore them.
    payload['object_shape'] = str(corridor.get('objectShape', 'none'))
    # WHAT THE CORRIDOR WAS AIMED AT, beside the corridor. odom, like the
    # polygon: the tracked target as the planner held it at build time, and
    # the standoff point derived from it (the corridor's own end, except where
    # the length cap cut the corridor short of it). None off the object
    # branch. Without these a rebuilt corridor cannot be checked against the
    # thing it was built for -- which is how a target_lost abort ends up
    # undiagnosable after the fact.
    payload['object_target'] = _xy_or_none(corridor.get('objectTarget'))
    payload['object_goal'] = _xy_or_none(corridor.get('objectGoal'))
    step = corridor.get('refStep') or {}
    payload['ref_step'] = {
        'centreline_m': _finite_or_none(step.get('centreline_m')),
        'tangent_rad': _finite_or_none(step.get('tangent_rad')),
        'pend_m': _finite_or_none(step.get('pend_m')),
    }

    defn = corridor.get('defn')
    if defn:
        payload['schema'] = str(defn.get('type', 'mpc_corr/v2'))
        payload['definition'] = dict(defn)
        n = defn.get('corr_N')
        length = _finite_or_none(defn.get('L'))
        if n and length is not None and int(n) > 1:
            # arclength between consecutive boundary samples: what the
            # clearance polygon's discretisation error is governed by
            payload['sample_step_m'] = length / (int(n) - 1)

    if odom_topic:
        # the pose estimate the walls were built from: a logger measuring
        # clearance against a different one compares two frames
        payload['odom_topic'] = str(odom_topic)
    return payload
