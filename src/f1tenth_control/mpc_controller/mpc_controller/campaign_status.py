"""Payloads of mpc_corr's two test-campaign topics: /mpc/status and /corridor.

Pure: no rclpy. Both are std_msgs/String JSON in the shape
f1tenth_logger's test_campaign logger reads (see its logger_node.py):

    /mpc/status  after EVERY solve
                 {status, solve_time_ms, cost, iterations, success, solver,
                  status_code, status_message}
    /corridor    every time build_straight_corridor() produces a new corridor
                 {id, polygon: [[x, y], ...], source, frame_id, ...}

``status`` is a small closed vocabulary rather than the backend's own text, so
the logger can count feasibility with one rule for both backends. Only
``solved`` counts as solved there; ``solved_inaccurate`` is kept distinct
because mpc_corr itself treats it as usable (it keeps the warm start), which
the campaign may or may not want to count.

Nothing here feeds back into control: MPC_corr builds these from values the
tick has already computed, after the solve, and publishing them is wrapped so
a failure can only lose a message.
"""

import math

__all__ = ['corridor_payload', 'mpc_status_payload', 'solve_status_label']

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


def mpc_status_payload(info, solve_dt_sec, solver):
    """The /mpc/status JSON object for one solve.

    ``cost`` and ``iterations`` are null when the backend reported none (a
    NaN cost is not a zero cost); ``solve_time_ms`` is the same wall time
    /mpc/solver_status carries as solve_dt_sec.
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
    }


#: Vertex rounding of the sampled boundary, in decimal places. 6 dp is 1e-6 m,
#: two orders below the 6.5e-5 m worst-case error the 120-sample polygon itself
#: carries against an exact evaluation of the walls (measured at the documented
#: 0.763 rad wall_turn ask, the tightest corridor the stack asks for). The
#: previous 4 dp put the rounding floor at 1e-4 m, i.e. LARGER than the
#: sampling error it sat on top of, which made the polygon the limiting term
#: for no reason. The function definition below is never rounded.
_POLYGON_DP = 6


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
