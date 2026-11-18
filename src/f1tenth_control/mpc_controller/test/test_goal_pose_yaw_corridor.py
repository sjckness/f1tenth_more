"""/mpc/goal_pose: the car arrives along the goal's yaw (goal_pose_use_yaw).

WHY THIS EXISTS. Until 2026-11-05 build_straight_corridor's goal_pose branch
pointed the corridor at the goal's BEARING and dropped its yaw. EnterDoor's
goal is a doorway whose yaw is the way through, so the car chased a point:
in 2026-11-05T14-35-46 straight_2m_then_left_door it reached the 0.8 m
pre-point 72 deg off the door axis and then drove past the door.

With goal_pose_honour_yaw set, the branch asks for the pose corridor
(corridor_curves_to_pose: ends ON the goal, tangent to its yaw) and psiRef
becomes the goal's yaw; when that curve is not drivable or longer than
corr_L_base the corridor is the bearing one, exactly as before.

The geometry below is that run's, from the bag: the car at the end of the 2 m
straight, door_130's centre and axis, and its 0.8 m pre-point.

Same stand-in shape as test_corridor_heading_reference.py: the REAL unbound
build_straight_corridor against a duck-typed object. No rclpy, no node.
"""
import inspect
import math

from mpc_controller.MPC_corr import MPCController

import pytest


class _FakeLogger:
    def __init__(self):
        self.infos = []

    def info(self, msg, *_a, **_k):
        self.infos.append(str(msg))

    def warn(self, msg, *_a, **_k):
        self.infos.append(str(msg))


DELTA_MAX = 0.319        # mpc_steering_angle_max_rad
WHEELBASE = 0.305        # mpc_wheelbase_m -> R_min 0.923 m

CAR = (1.95, 0.14, math.radians(5.3))          # end of the 2 m straight
DOOR = (4.45, 1.97, math.radians(88.2))        # door_130 centre and axis
PRE_POINT = (4.33, 1.14, math.radians(88.2))   # 0.8 m back on the axis


class _FakeMPC:
    def __init__(self, goal, honour):
        self._logger = _FakeLogger()
        self.front_distance = 10.0
        self.goal_pose_xy = (goal[0], goal[1])
        self.goal_pose_yaw = goal[2]
        self.goal_pose_honour_yaw = honour
        self.corr_L_base = 3.0
        self.corr_N = 120
        self.corr_wmin = 1.3
        self.corr_wmax = 2.3
        self.corr_turn_u_start = 0.10
        self.corr_turn_u_end = 0.70
        self.psi_init_corridor = 0.0
        self.goal_start_xy = None
        self.goal_anchor_odom = None
        self.limits = {'delta_max': DELTA_MAX, 'delta_min': -DELTA_MAX}
        self.params = {'L': WHEELBASE}

    def get_logger(self):
        return self._logger


def _build(car, goal, honour):
    fake = _FakeMPC(goal, honour)
    return MPCController.build_straight_corridor(fake, [car[0], car[1], car[2], 0.0]), fake


def _bearing(car, goal):
    return math.atan2(goal[1] - car[1], goal[0] - car[0])


class TestPoseCorridor:

    def test_the_door_is_reached_along_its_axis(self):
        # 3.1 m of chord: the pose curve is 3.4 m, past the cap, so the very
        # first rebuild is still the bearing one -- one step closer it is not.
        car = (2.76, 0.25, math.radians(17.9))     # 355.9 s in the bag
        corridor, _ = _build(car, DOOR, honour=True)
        assert corridor['objectShape'] == 'goal_pose_arc'
        assert corridor['psiRef'] == pytest.approx(DOOR[2])
        assert corridor['defn']['centreline'] == 'bezier'
        # It ENDS on the door, which the bearing corridor never did.
        assert corridor['xc'][-1] == pytest.approx(DOOR[0], abs=1e-6)
        assert corridor['yc'][-1] == pytest.approx(DOOR[1], abs=1e-6)
        assert corridor['defn']['feasible']

    def test_the_end_tangent_is_the_door_axis(self):
        car = (2.76, 0.25, math.radians(17.9))
        corridor, _ = _build(car, DOOR, honour=True)
        dx = corridor['xc'][-1] - corridor['xc'][-2]
        dy = corridor['yc'][-1] - corridor['yc'][-2]
        assert math.atan2(dy, dx) == pytest.approx(DOOR[2], abs=math.radians(2.0))

    def test_off_is_the_bearing_corridor_unchanged(self):
        car = (2.76, 0.25, math.radians(17.9))
        corridor, _ = _build(car, DOOR, honour=False)
        assert corridor['objectShape'] == 'none'
        assert corridor['psiRef'] == pytest.approx(_bearing(car, DOOR))
        assert corridor['defn']['centreline'] == 'ramp'


class TestFallBackToTheBearing:

    def _assert_same_as_off(self, car, goal):
        on, fake = _build(car, goal, honour=True)
        off, _ = _build(car, goal, honour=False)
        assert on['objectShape'] == 'none'
        assert on['psiRef'] == pytest.approx(off['psiRef'])
        assert on['defn']['centreline'] == 'ramp'
        for key in ('xc', 'yc', 'xL', 'yL', 'xR', 'yR'):
            assert list(on[key]) == pytest.approx(list(off[key])), key
        assert any('CORR/goal_pose' in m for m in fake._logger.infos)

    def test_an_undrivable_pose_falls_back(self):
        """The 0.8 m pre-point from the end of the straight: 83 deg of turn
        inside 1.0 m of lateral room needs R 0.71 m against R_min 0.92 m."""
        self._assert_same_as_off(CAR, PRE_POINT)

    def test_a_pose_past_the_cap_falls_back(self):
        self._assert_same_as_off(CAR, DOOR)

    def test_a_far_goal_falls_back(self):
        self._assert_same_as_off((0.0, 0.0, 0.0), (6.0, 1.0, math.radians(90.0)))


class TestOnlyGoalPoseHonoursTheYaw:
    """goal_turn's synthetic target reuses goal_pose_xy/yaw; it must not
    switch to the pose corridor, so only goal_pose_callback sets the flag."""

    def test_goal_pose_callback_sets_it_from_the_parameter(self):
        src = inspect.getsource(MPCController.goal_pose_callback)
        assert "self.goal_pose_honour_yaw = bool(getattr(self, 'goal_pose_use_yaw'" in src

    def test_goal_turn_callback_clears_it(self):
        src = inspect.getsource(MPCController.goal_turn_callback)
        assert 'self.goal_pose_honour_yaw = False' in src
