"""/mpc/status and /corridor: the payloads, and that publishing them is inert.

mpc_corr publishes both for the test-campaign logger (f1tenth_logger
test_campaign). Three things are pinned here:

1. the payloads -- the status vocabulary for every OSQP code and both
   backends, null for a cost or iteration count the backend did not report,
   and the corridor polygon's vertex order;
2. a real solve through solve_mpc_step reports its iteration count and comes
   out as ``solved``, so the field is not only covered by hand-built dicts;
3. the two publish methods never raise into control_loop, whatever the
   publisher or the corridor does -- a failure costs the message only.

Same duck-typed stand-in shape as test_corridor_marker_lifetime.py: the
methods under test are MPCController's own, bound to an object carrying only
what they read.

Run standalone: python3 -m pytest test/test_campaign_status.py -v
"""

import json
import math

import numpy as np
import pytest

from mpc_controller.campaign_status import (
    corridor_payload, horizon_payload, mpc_status_payload, solve_status_label)
from mpc_controller.MPC_corr import MPCController
from mpc_controller.mpc_solver import solve_mpc_step


# ---------------------------------------------------------------------------
# payloads
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('code, expected', [
    (1, 'solved'), (2, 'solved_inaccurate'), (3, 'infeasible'), (4, 'infeasible'),
    (5, 'infeasible'), (6, 'infeasible'), (7, 'max_iter'), (8, 'timeout'),
    (9, 'non_convex'), (10, 'interrupted'), (11, 'unsolved'), (42, 'failed'),
    (-1, 'no_solve'),
])
def test_every_osqp_code_has_a_label(code, expected):
    success = code in (1, 2)
    assert solve_status_label({'success': success, 'status': code}, 'rti') == expected


def test_a_solved_status_the_node_rejected_is_failed():
    # _solve_rti sets success False on a non-finite x even when OSQP said solved
    assert solve_status_label({'success': False, 'status': 1}, 'rti') == 'failed'


def test_slsqp_labels():
    assert solve_status_label({'success': True, 'status': 0}, 'slsqp') == 'solved'
    assert solve_status_label({'success': False, 'status': 9}, 'slsqp') == 'max_iter'
    assert solve_status_label({'success': False, 'status': 8}, 'slsqp') == 'failed'


def test_status_payload_fields():
    info = {'success': True, 'status': 1, 'status_message': 'solved',
            'cost': 1.25, 'iterations': 25}
    payload = mpc_status_payload(info, 0.0123, 'rti')
    assert payload == {
        'status': 'solved', 'solve_time_ms': pytest.approx(12.3),
        'cost': 1.25, 'iterations': 25, 'success': True, 'solver': 'rti',
        'status_code': 1, 'status_message': 'solved',
        # this info carries no x_pred, and a solve with no prediction
        # publishes no horizon rather than an empty one
        'horizon': None,
    }
    # strict JSON: no NaN anywhere
    json.loads(json.dumps(payload), parse_constant=pytest.fail)


def test_missing_cost_and_iterations_are_null_not_zero():
    payload = mpc_status_payload(
        {'success': False, 'status': -1, 'cost': float('nan')}, 0.004, 'rti')
    assert payload['cost'] is None
    assert payload['iterations'] is None
    assert payload['status'] == 'no_solve'
    json.loads(json.dumps(payload), parse_constant=pytest.fail)


def _rect_corridor(n=5, half=0.5, length=2.0):
    xc = np.linspace(0.0, length, n)
    return {
        'xL': xc, 'yL': np.full(n, half), 'xR': xc, 'yR': np.full(n, -half),
        'psiRef': 0.0, 'L': length, 'objectMode': False,
    }


def test_corridor_polygon_is_left_forward_then_right_back():
    payload = corridor_payload(_rect_corridor(n=3), 7, 'odom', 'mpc_corr',
                               odom_topic='/odometry/filtered')
    assert payload['id'] == 7
    assert payload['polygon'] == [
        [0.0, 0.5], [1.0, 0.5], [2.0, 0.5],
        [2.0, -0.5], [1.0, -0.5], [0.0, -0.5],
    ]
    assert payload['frame_id'] == 'odom'
    assert payload['source'] == 'mpc_corr'
    assert payload['odom_topic'] == '/odometry/filtered'


def test_corridor_polygon_spans_exactly_the_walls_it_was_built_from():
    # a 1.0 m wide, 2.0 m long corridor: the polygon must reach +-0.5 m and
    # 0..2 m and no further, or every clearance measured against it is off
    polygon = corridor_payload(_rect_corridor(n=21), 1, 'odom', 'mpc_corr')['polygon']
    ys = [p[1] for p in polygon]
    assert min(ys) == -0.5 and max(ys) == 0.5
    xs = [p[0] for p in polygon]
    assert min(xs) == 0.0 and max(xs) == 2.0


def test_the_corridor_carries_what_it_was_aimed_at():
    """The target and the standoff, odom, beside the geometry.

    A rebuilt corridor cannot be checked against the thing it was built for
    without these -- which is how a target_lost abort ends up undiagnosable.
    """
    corridor = _rect_corridor()
    corridor['objectTarget'] = [4.0, 1.0]
    corridor['objectGoal'] = [3.0, 0.75]
    payload = corridor_payload(corridor, 1, 'odom', 'mpc_corr')
    assert payload['object_target'] == [4.0, 1.0]
    assert payload['object_goal'] == [3.0, 0.75]


def test_no_target_is_null_and_a_half_finite_one_is_too():
    """None off the object branch; never [4.0, NaN], which would plot."""
    payload = corridor_payload(_rect_corridor(), 1, 'odom', 'mpc_corr')
    assert payload['object_target'] is None
    assert payload['object_goal'] is None

    corridor = _rect_corridor()
    corridor['objectTarget'] = [4.0, float('nan')]
    payload = corridor_payload(corridor, 1, 'odom', 'mpc_corr')
    assert payload['object_target'] is None
    json.loads(json.dumps(payload), parse_constant=pytest.fail)


def test_non_finite_vertices_are_dropped():
    corridor = _rect_corridor(n=3)
    corridor['xL'] = np.array([0.0, float('nan'), 2.0])
    payload = corridor_payload(corridor, 1, 'odom', 'mpc_corr')
    assert len(payload['polygon']) == 5
    json.loads(json.dumps(payload), parse_constant=pytest.fail)


# ---------------------------------------------------------------------------
# a real solve
# ---------------------------------------------------------------------------

def _solve_once(solver='rti', horizon=7):
    """One real solve down a straight 2.6 m wide corridor -> its info dict."""
    n = 60
    u = np.linspace(0.0, 1.0, n)
    xc, yc = 3.0 * u, np.zeros(n)
    corridor = {
        'xc': xc, 'yc': yc, 'xL': xc, 'yL': yc + 1.3, 'xR': xc, 'yR': yc - 1.3,
        'tx': np.ones(n), 'ty': np.zeros(n), 'nx': np.zeros(n), 'ny': np.ones(n),
        'halfWidth': np.full(n, 1.3), 'psiRef': 0.0, 't': 0.0,
        'Pend': np.array([3.0, 0.0]), 'dFront': 10.0, 'dpsi': 0.0,
        'obstacles_world': [], 'd_safe': 0.3, 'car_radius': 0.2,
        'avoidance_margin': 0.1,
    }
    limits = {'delta_min': -0.3, 'delta_max': 0.3, 'a_min': -2.0, 'a_max': 3.0,
              'dDeltaMin': -0.5, 'dDeltaMax': 0.5, 'dAMin': -2.0, 'dAMax': 2.0,
              'vMin': -1.0, 'vMax': 3.0}
    weights = {'w_term': 3.0, 'w_v': 8.0, 'w_psi': 0.0, 'w_u_a': 0.0,
               'w_du_delta': 15.0, 'w_du_a': 0.0, 'w_delta0': 0.0, 'w_obs': 8.0,
               'w_corr': 0.0}
    _, info = solve_mpc_step(
        x0=np.array([0.0, 0.0, 0.0, 0.3]), last_u=np.array([0.0, 0.0]),
        pref_nom=np.array([1.5, 0.0]), corridor=corridor, horizon=horizon,
        ts=0.1, params={'L': 0.305, 'lr': 0.17}, limits=limits,
        weights=weights, obstacles=[], dmin=0.3, vdes=0.5, solver=solver)
    return info


def test_a_real_solve_reports_iterations_and_is_solved():
    for solver in ('rti', 'slsqp'):
        info = _solve_once(solver)
        payload = mpc_status_payload(info, 0.01, solver)
        assert isinstance(payload['iterations'], int) and payload['iterations'] > 0, solver
        assert payload['status'] == 'solved', (solver, payload)
        assert payload['cost'] is not None and math.isfinite(payload['cost'])


# ---------------------------------------------------------------------------
# the node methods: publish once, never raise
# ---------------------------------------------------------------------------

class _Publisher:
    def __init__(self, fail=False):
        self.sent = []
        self.fail = fail

    def publish(self, msg):
        if self.fail:
            raise RuntimeError('publisher context is invalid')
        self.sent.append(json.loads(msg.data))


class _Log:
    def __init__(self):
        self.warnings = []

    def warn(self, text, **_):
        self.warnings.append(text)


class _Sub:
    def __init__(self, topic):
        self.topic_name = topic


class _FakeController:
    def __init__(self, fail=False):
        self.campaign_status_pub = _Publisher(fail)
        self.corridor_pub = _Publisher(fail)
        self._corridor_seq = 0
        self.active_odom_source = 'hardware'
        self.sub_odom_hw = _Sub('/odometry/filtered')
        self.sub_odom_sim = _Sub('/model/virtual_robot/odometry')
        # _publish_campaign_status carries the control period into the payload
        # so a logged horizon's steps can be placed in time.
        self.ts = 0.1
        self.log = _Log()

    def get_logger(self):
        return self.log

    _publish_campaign_status = MPCController._publish_campaign_status
    _publish_corridor_polygon = MPCController._publish_corridor_polygon


def test_one_status_message_per_solve():
    ctrl = _FakeController()
    info = {'success': True, 'status': 1, 'cost': 2.0, 'iterations': 12}
    ctrl._publish_campaign_status(info, 0.02, 'rti')
    ctrl._publish_campaign_status(info, 0.03, 'rti')
    assert [m['solve_time_ms'] for m in ctrl.campaign_status_pub.sent] == [
        pytest.approx(20.0), pytest.approx(30.0)]
    assert ctrl.log.warnings == []


def test_corridor_ids_count_rebuilds_and_name_the_pose_source():
    ctrl = _FakeController()
    ctrl._publish_corridor_polygon(_rect_corridor())
    ctrl.active_odom_source = 'sim'
    ctrl._publish_corridor_polygon(_rect_corridor())
    sent = ctrl.corridor_pub.sent
    assert [m['id'] for m in sent] == [1, 2]
    assert sent[0]['odom_topic'] == '/odometry/filtered'
    assert sent[1]['odom_topic'] == '/model/virtual_robot/odometry'


def test_a_failing_publisher_never_raises_into_the_control_loop():
    ctrl = _FakeController(fail=True)
    ctrl._publish_campaign_status({'success': True, 'status': 1}, 0.01, 'rti')
    ctrl._publish_corridor_polygon(_rect_corridor())
    assert len(ctrl.log.warnings) == 2


def test_a_malformed_corridor_never_raises_into_the_control_loop():
    ctrl = _FakeController()
    ctrl._publish_corridor_polygon({'xL': [0.0]})   # no right wall at all
    ctrl._publish_campaign_status(None, 0.01, 'rti')  # no info dict at all
    assert ctrl.corridor_pub.sent == [] and ctrl.campaign_status_pub.sent == []
    assert len(ctrl.log.warnings) == 2


# ---------------------------------------------------------------------------
# the predicted horizon
# ---------------------------------------------------------------------------

def _info(n=3, ok=True):
    """A solve's info dict with an N-step rollout and its control sequence."""
    x_pred = np.array([[float(k + 1), 0.5 * k, 0.01 * k, 0.45]
                       for k in range(n)])
    return {
        'success': ok, 'status': 1, 'cost': 1.0, 'iterations': 4,
        'x_pred': x_pred,
        'zopt': np.array([v for k in range(n) for v in (0.02 * k, 0.1)]),
    }


def test_the_horizon_carries_the_states_the_solver_optimized_over():
    payload = horizon_payload(_info(3), ts=0.1)
    assert payload['n'] == 3
    assert payload['x'] == pytest.approx([1.0, 2.0, 3.0])
    assert payload['y'] == pytest.approx([0.0, 0.5, 1.0])
    assert payload['v'] == pytest.approx([0.45, 0.45, 0.45])
    assert payload['frame_id'] == 'odom'
    assert payload['ts'] == pytest.approx(0.1)


def test_the_inputs_are_split_and_aligned_with_the_states():
    """zopt is flat [delta_0, a_0, delta_1, a_1, ...]; entry k reaches state k."""
    payload = horizon_payload(_info(3), ts=0.1)
    assert payload['steer'] == pytest.approx([0.0, 0.02, 0.04])
    assert payload['accel'] == pytest.approx([0.1, 0.1, 0.1])
    assert len(payload['steer']) == payload['n']


def test_a_solve_with_no_prediction_publishes_no_horizon():
    """The normal state on a failed solve, not an error."""
    assert horizon_payload({'x_pred': None}, ts=0.1) is None
    assert horizon_payload({'x_pred': []}, ts=0.1) is None
    assert horizon_payload({}, ts=0.1) is None


def test_a_non_finite_state_drops_the_whole_horizon():
    """All-or-nothing, matching MpcSolverStatus: never a NaN-padded path."""
    info = _info(3)
    info['x_pred'][1][0] = math.nan
    assert horizon_payload(info, ts=0.1) is None


def test_a_bad_input_sequence_still_publishes_the_states():
    """The states are the deliverable; the inputs are the extra."""
    info = _info(3)
    info['zopt'] = np.array([0.1, 0.2])          # too short for 3 steps
    payload = horizon_payload(info, ts=0.1)
    assert payload['n'] == 3
    assert payload['steer'] is None and payload['accel'] is None

    info = _info(3)
    info['zopt'] = None
    assert horizon_payload(info, ts=0.1)['steer'] is None


def test_the_status_payload_carries_the_horizon():
    payload = mpc_status_payload(_info(4), 0.02, 'rti', ts=0.1)
    assert payload['status'] == 'solved'
    assert payload['horizon']['n'] == 4
    assert payload['horizon']['ts'] == pytest.approx(0.1)


def test_the_status_payload_survives_a_solve_with_no_horizon():
    payload = mpc_status_payload({'success': False, 'status': 3}, 0.02, 'rti',
                                 ts=0.1)
    assert payload['status'] == 'infeasible'
    assert payload['horizon'] is None


def test_the_published_status_carries_the_nodes_control_period():
    """Without ts a reader cannot place step k in time, so it must ride along."""
    ctrl = _FakeController()
    ctrl.ts = 0.05
    ctrl._publish_campaign_status(_info(3), 0.02, 'rti')
    sent = ctrl.campaign_status_pub.sent[0]
    assert sent['horizon']['ts'] == pytest.approx(0.05)
    assert sent['horizon']['frame_id'] == 'odom'
    assert ctrl.log.warnings == []


def test_a_real_solve_publishes_a_horizon_of_the_configured_length():
    """Not only hand-built dicts: the real solver's info must carry x_pred."""
    info = _solve_once(horizon=7)
    payload = horizon_payload(info, ts=0.1)
    assert payload is not None
    assert payload['n'] == len(info['x_pred'])
    assert len(payload['x']) == payload['n']
