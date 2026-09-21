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


def corridor_payload(corridor, corridor_id, frame_id, source, odom_topic=None):
    """The /corridor JSON object for one freshly built corridor.

    The polygon is the left wall walked forward then the right wall walked
    back, closed implicitly -- the same order robot_logger's
    corridor_from_centerline uses -- in the frame the walls were built in.
    Non-finite vertices are dropped rather than serialised as NaN.
    """
    left = zip(corridor['xL'], corridor['yL'])
    right = list(zip(corridor['xR'], corridor['yR']))[::-1]
    polygon = []
    for x, y in list(left) + right:
        x, y = _finite_or_none(x), _finite_or_none(y)
        if x is not None and y is not None:
            polygon.append([round(x, 4), round(y, 4)])
    payload = {
        'id': int(corridor_id),
        'polygon': polygon,
        'source': str(source),
        'frame_id': str(frame_id),
        'psi_ref': _finite_or_none(corridor.get('psiRef')),
        'length_m': _finite_or_none(corridor.get('L')),
        'object_mode': bool(corridor.get('objectMode', False)),
    }
    if odom_topic:
        # the pose estimate the walls were built from: a logger measuring
        # clearance against a different one compares two frames
        payload['odom_topic'] = str(odom_topic)
    return payload
