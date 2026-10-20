"""The behaviour-tree half of the deceleration ramp: arrival, and the target.

Two things the tree owns once mpc_corr ramps into a stopping point instead of
crossing it:

  THE ARRIVAL ARM. The ramp parks the car ON the wall target, and
  front_clearance's own guard is "distance < target" -- strict, so a car
  resting at exactly 2.00 m from a 2.00 m target never fires it and the move
  holds until timeout_sec. So the type also fires on "within
  stop_ramp_arrive_tol_m AND at rest", the same dual criterion object_reached
  already uses. Both halves are load-bearing and both are tested here.

  THE TARGET ITSELF. mpc_corr reads the wall DISTANCE but the TARGET lives in
  the mission JSON, so PublishMoveGoal sends it on /mpc/stop_target -- for
  EVERY move, because the -1.0 published for a move with no wall stop is the
  only thing that clears the previous move's target.

Run standalone: python3 -m pytest test/test_stop_ramp_arrival.py -v
"""

from types import SimpleNamespace

import pytest

py_trees = pytest.importorskip('py_trees')

import f1tenth_behavior.behaviours.check_stop_condition as check_module  # noqa: E402
from f1tenth_behavior.behaviours.check_stop_condition import (  # noqa: E402
    CheckStopCondition,
)
from f1tenth_behavior.behaviours.publish_move_goal import PublishMoveGoal  # noqa: E402
from f1tenth_behavior.mission.condition_eval import EvalContext, evaluate  # noqa: E402
from f1tenth_behavior.mission.detected_classes_bridge import (  # noqa: E402
    DETECTED_CLASSES_KEY,
)
from f1tenth_behavior.mission.mission_config import (  # noqa: E402
    StopCondition, parse_mission,
)
from f1tenth_behavior.mission.runtime import (  # noqa: E402
    FRONT_CLEARANCE_KEY,
    MIN_OBSTACLE_DISTANCE_FORWARD_KEY,
    MIN_OBSTACLE_DISTANCE_KEY,
    MISSION_KEY,
    MissionRuntimeState,
)
from f1tenth_params.param_defaults import get_value  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from std_msgs.msg import Float32  # noqa: E402

S = py_trees.common.Status
STOP_AT = 2.0
TOL = float(get_value('stop_ramp_arrive_tol_m'))
REST = float(get_value('stop_ramp_rest_speed_mps'))
V_MIN = float(get_value('min_moving_speed_mps'))


@pytest.fixture(autouse=True)
def _no_report_files(monkeypatch):
    monkeypatch.setattr(check_module, 'write_mission_summary', lambda *a, **k: None)


class _Clock:
    """time.monotonic() for check_stop_condition, set by the test."""

    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture
def clock(monkeypatch):
    c = _Clock()
    monkeypatch.setattr(check_module.time, 'monotonic', c)
    return c


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        # Float32/Bool keep their payload; the goal shapes (PoseStamped,
        # DriveCommand, TurnGoal) are recorded whole -- these tests only count
        # them, the goal shapes themselves are other files' business.
        self.msgs.append(getattr(msg, 'data', msg))


class _Logger:
    def __init__(self):
        self.warns = []

    def info(self, *a, **k):
        pass

    def warn(self, msg, **k):
        self.warns.append((msg, k))

    error = warn


def _mission(moves):
    moves = [dict(m) for m in moves]
    moves[-1]['terminal'] = True
    config = parse_mission({'mission_id': 't', 'schema_version': '4.0', 'moves': moves})
    state = MissionRuntimeState()
    state.load(config, check_module.time.monotonic())
    state.begin(check_module.time.monotonic())
    return state


def _wall_move(move_id='m', distance=STOP_AT, **extra):
    return dict({'id': move_id, 'drive': {'mode': 'straight', 'speed': 0.4},
                 'stop_condition': {'type': 'front_clearance', 'distance': distance},
                 'timeout_sec': 60, 'on_timeout': 'abort'}, **extra)


def _check(state):
    behaviour = CheckStopCondition()
    logger = _Logger()
    behaviour.node = SimpleNamespace(get_logger=lambda: logger)
    behaviour.logger_stub = logger
    behaviour.blackboard = SimpleNamespace()
    setattr(behaviour.blackboard, MISSION_KEY, state)
    for key in (MIN_OBSTACLE_DISTANCE_KEY, MIN_OBSTACLE_DISTANCE_FORWARD_KEY,
                FRONT_CLEARANCE_KEY):
        setattr(behaviour.blackboard, key, None)
    setattr(behaviour.blackboard, DETECTED_CLASSES_KEY, {})
    behaviour.hold_pub, behaviour.object_end_pub = _Pub(), _Pub()
    return behaviour


def _wall(behaviour, value):
    behaviour._front_clearance_cb(Float32(data=float(value)))


def _odom(behaviour, speed):
    """An odometry message arriving now, through the real callback."""
    msg = Odometry()
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.linear.x = float(speed)
    behaviour._odom_cb(msg)


# =====================================================================
# The arrival arm, as a property of evaluate() alone.
# =====================================================================
class TestTheArrivalArm:

    def condition(self, distance=STOP_AT):
        return StopCondition(type='front_clearance', params={'distance': distance})

    def ctx(self, front_clearance, speed):
        return EvalContext(
            now=1000.0, move_start_time=990.0, move_start_xy=(0.0, 0.0),
            current_xy=(1.0, 0.0), detected_classes={},
            min_obstacle_distance=None, front_clearance=front_clearance,
            default_distance=None, speed=speed, stop_ramp_arrive_tol_m=TOL,
            stop_ramp_rest_speed_mps=REST)

    def test_a_car_resting_on_the_target_has_arrived(self):
        """The case the strict guard cannot see, and the reason this arm exists."""
        assert evaluate(self.condition(), self.ctx(STOP_AT, 0.0)) is True

    def test_a_car_resting_just_inside_the_tolerance_has_arrived(self):
        assert evaluate(
            self.condition(), self.ctx(STOP_AT + TOL - 0.01, 0.0)) is True

    def test_a_car_resting_outside_the_tolerance_has_not(self):
        assert evaluate(
            self.condition(), self.ctx(STOP_AT + TOL + 0.05, 0.0)) is False

    def test_a_car_still_rolling_inside_the_tolerance_has_not(self):
        """ARRIVING is not ARRIVED -- the strict arm will fire a tick or two later.

        Without the speed half this would end the move while the car is still
        doing the operating floor, i.e. one whole stopping distance early,
        which is the bug the ramp exists to remove.
        """
        assert evaluate(self.condition(), self.ctx(STOP_AT + 0.05, V_MIN)) is False

    def test_the_speed_half_uses_the_magnitude(self):
        """Reverse is not rest, even though the stack forbids reverse."""
        assert evaluate(self.condition(), self.ctx(STOP_AT, -0.3)) is False

    def test_rest_is_the_configured_speed_not_exactly_zero(self):
        """VESC telemetry reads zero only below its own deadband."""
        assert evaluate(self.condition(), self.ctx(STOP_AT, REST)) is True
        assert evaluate(self.condition(), self.ctx(STOP_AT, REST + 0.01)) is False

    def test_no_odometry_yet_cannot_arrive_on_this_arm(self):
        assert evaluate(self.condition(), self.ctx(STOP_AT, None)) is False

    def test_the_strict_arm_still_fires_while_moving(self):
        """Unchanged behaviour: below the target at speed is still a stop."""
        assert evaluate(self.condition(), self.ctx(STOP_AT - 0.01, 1.0)) is True

    def test_an_absent_reading_is_still_never_a_distance(self):
        """-1.0 and None must not reach either arm. Pinned against regression."""
        for absent in (None, -1.0, 0.0):
            assert evaluate(self.condition(), self.ctx(absent, 0.0)) is False

    def test_the_arm_belongs_to_front_clearance_only(self):
        """A resting car does not satisfy somebody else's stop condition."""
        other = StopCondition(type='obstacle_distance_below',
                              params={'distance': STOP_AT})
        assert evaluate(other, self.ctx(STOP_AT, 0.0)) is False


# =====================================================================
# The same arm through the real behaviour, with the real odom callback.
# =====================================================================
class TestTheArmThroughTheBehaviour:

    def test_the_speed_comes_off_the_odometry_topic(self, clock):
        behaviour = _check(_mission([_wall_move()]))
        _odom(behaviour, 0.37)
        assert behaviour.speed == pytest.approx(0.37)

    def test_resting_on_the_target_ends_the_move(self, clock):
        state = _mission([_wall_move()])
        behaviour = _check(state)
        _wall(behaviour, STOP_AT)
        _odom(behaviour, 0.0)
        clock.t += 0.1
        assert behaviour.update() == S.SUCCESS
        assert state.last_stop_reason == 'stop_condition:front_clearance'

    def test_rolling_on_the_target_keeps_the_move_running(self, clock):
        behaviour = _check(_mission([_wall_move()]))
        _wall(behaviour, STOP_AT)
        _odom(behaviour, V_MIN)
        clock.t += 0.1
        assert behaviour.update() == S.RUNNING

    def test_a_stale_wall_distance_still_holds_rather_than_arriving(self, clock):
        """The freshness hold outranks the arrival arm, as it must.

        A car standing still next to a dead wall publisher has not arrived
        anywhere -- it is being held (check_stop_condition, WALL DISTANCE
        FRESHNESS), and the arm must not turn that into a completed move.
        """
        behaviour = _check(_mission([_wall_move()]))
        _wall(behaviour, STOP_AT)
        _odom(behaviour, 0.0)
        clock.t += float(get_value('wall_distance_max_age_s')) + 0.01
        assert behaviour.update() == S.RUNNING
        assert behaviour.hold_pub.msgs == [True]

    def test_the_defaults_come_from_stack_params(self, clock):
        behaviour = _check(_mission([_wall_move()]))
        assert behaviour.stop_ramp_arrive_tol_m == pytest.approx(TOL)
        assert behaviour.stop_ramp_rest_speed_mps == pytest.approx(REST)

    def test_the_tolerance_covers_the_ramps_own_hand_over(self):
        """stop_ramp_arrive_tol_m must be >= d_final or arrivals go unnoticed.

        d_final is where mpc_corr commands zero -- v_min * tau + v_min^2 /
        (2 a_stop), 0.133 m shipped. A car that stops a hair short of the
        target rests up to that far outside it, and a tolerance tighter than
        that would read a perfect stop as "never arrived".
        """
        d_final = (V_MIN * float(get_value('stop_ramp_tau_latency_s'))
                   + V_MIN ** 2 / (2.0 * float(get_value('stop_ramp_a_stop_mps2'))))
        assert TOL >= d_final


# =====================================================================
# /mpc/stop_target: published for EVERY move, or the ramp aims at the
# previous move's wall.
# =====================================================================
class TestTheStopTargetIsPublished:

    def behaviour(self, state):
        b = PublishMoveGoal()
        logger = _Logger()
        b.node = SimpleNamespace(get_logger=lambda: logger,
                                 get_clock=lambda: SimpleNamespace(
                                     now=lambda: SimpleNamespace(
                                         to_msg=Time)))
        b.blackboard = SimpleNamespace()
        setattr(b.blackboard, MISSION_KEY, state)
        b.goal_distance_pub = _Pub()
        b.goal_pose_pub = _Pub()
        b.goal_turn_pub = _Pub()
        b.goal_drive_pub = _Pub()
        b.stop_target_pub = _Pub()
        return b

    def test_a_front_clearance_move_publishes_its_target(self, clock):
        b = self.behaviour(_mission([_wall_move(distance=2.5)]))
        assert b.update() == S.SUCCESS
        assert b.stop_target_pub.msgs == [pytest.approx(2.5)]

    def test_a_move_with_another_stop_publishes_the_clear_sentinel(self, clock):
        state = _mission([{'id': 'm', 'goal_distance': 3.0,
                           'stop_condition': {'type': 'distance_reached'},
                           'timeout_sec': 60, 'on_timeout': 'abort'}])
        b = self.behaviour(state)
        assert b.update() == S.SUCCESS
        assert b.stop_target_pub.msgs == [pytest.approx(-1.0)]

    def test_the_sentinel_is_negative_not_zero(self, clock):
        """Zero is a legal target, so it cannot double as "no target"."""
        b = self.behaviour(_mission([_wall_move(distance=0.0)]))
        assert b.update() == S.SUCCESS
        assert b.stop_target_pub.msgs == [0.0]

    def test_it_goes_out_once_per_move_not_once_per_tick(self, clock):
        """Same goal_dirty guard as the goal shapes -- see that module."""
        state = _mission([_wall_move()])
        b = self.behaviour(state)
        b.update()
        b.update()
        b.update()
        assert len(b.stop_target_pub.msgs) == 1

    def test_every_goal_shape_publishes_one(self, clock):
        """The contract mpc_corr's "never cleared on a new goal" rests on."""
        shapes = (
            {'id': 'd', 'goal_distance': 2.0,
             'stop_condition': {'type': 'goal_reached'}},
            {'id': 'p', 'goal_pose': {'x': 1.0, 'y': 0.0, 'yaw': 0.0},
             'stop_condition': {'type': 'goal_reached'}},
            {'id': 't', 'turn': {'heading_delta_deg': 90.0, 'speed': 0.4,
                      'steering': 'full_lock'},
             'stop_condition': {'type': 'orientation_delta', 'value': 90.0}},
            {'id': 'v', 'drive': {'mode': 'straight', 'speed': 0.4},
             'stop_condition': {'type': 'time_elapsed', 'duration_sec': 5.0}},
        )
        for shape in shapes:
            move = dict(shape, timeout_sec=60, on_timeout='abort')
            b = self.behaviour(_mission([move]))
            assert b.update() == S.SUCCESS
            assert b.stop_target_pub.msgs == [pytest.approx(-1.0)], (
                f'goal shape {move["id"]!r} published no stop target')
