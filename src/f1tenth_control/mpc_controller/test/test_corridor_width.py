"""
The corridor's width: a trapezoid with a fixed end-to-start ratio.

Before campaign-final the far end reached corr_wmax only on a corridor
corr_L_base long (corridor_geometry's length_ref), so the object corridors the
car actually built -- 0.8 to 2.5 m -- ended 1.2 to 1.6 times as wide as they
started, and the Bezier walls did most of that widening in the middle half.
On the car that read as two parallel lines. These pin the replacement:
straight walls, CORRIDOR_WIDTH_RATIO at every length, the old per-metre
opening only behind corr_width_scale_with_length.

Same duck-typed stand-in as test_corridor_turn_shape.py.

Run standalone: python3 -m pytest test/test_corridor_width.py -v
"""

import unittest

from f1tenth_params.corridor_geometry import CORRIDOR_WIDTH_RATIO
from f1tenth_params.param_defaults import get_value
from mpc_controller.MPC_corr import MPCController
import numpy as np


class _FakeLogger:
    def info(self, *args, **kwargs):
        pass

    def warn(self, *args, **kwargs):
        pass


class _FakeMPC:
    """Just enough of MPCController for build_straight_corridor."""

    def __init__(self, goal_pose_xy=None, scale_with_length=None):
        self.front_distance = 10.0
        self.goal_pose_xy = goal_pose_xy
        self.corr_L_base = 3.0
        self.corr_N = 120
        self.corr_wmin = 0.65
        self.corr_wmax = 0.65 * CORRIDOR_WIDTH_RATIO
        self.psi_init_corridor = 0.0
        self.goal_start_xy = None
        self.goal_anchor_odom = None
        self.corr_turn_u_start = 0.0
        self.corr_turn_u_end = 0.40
        if scale_with_length is not None:
            self.corr_width_scale_with_length = scale_with_length

    def get_logger(self):
        return _FakeLogger()


def _build(fake):
    return MPCController.build_straight_corridor(fake, [0.0, 0.0, 0.0, 0.5])


class TestTheTrapezoid(unittest.TestCase):
    """build_straight_corridor's walls and half-width."""

    def test_the_end_to_start_ratio_is_fixed_at_every_length(self):
        for goal in (None, (2.0, 0.0), (1.2, 0.0)):
            with self.subTest(goal=goal):
                corridor = _build(_FakeMPC(goal_pose_xy=goal))
                hw = corridor['halfWidth']
                self.assertAlmostEqual(hw[-1] / hw[0], CORRIDOR_WIDTH_RATIO,
                                       places=9)
                self.assertAlmostEqual(corridor['defn']['w1_eff'],
                                       corridor['defn']['w1'], places=12)

    def test_the_half_width_grows_linearly_along_the_corridor(self):
        corridor = _build(_FakeMPC(goal_pose_xy=(1.2, 0.0)))
        u = np.linspace(0.0, 1.0, 120)
        expected = 0.65 + 0.65 * (CORRIDOR_WIDTH_RATIO - 1.0) * u
        self.assertTrue(np.allclose(corridor['halfWidth'], expected,
                                    atol=1e-12))

    def test_the_logged_definition_names_the_trapezoid_walls(self):
        defn = _build(_FakeMPC())['defn']
        self.assertEqual(defn['walls'], 'trapezoid')
        self.assertIsNone(defn['ctrl_left'])

    def test_scale_with_length_restores_the_first_campaign_funnel(self):
        """1.5 m of a 3.0 m design length opens half of the widening."""
        corridor = _build(_FakeMPC(goal_pose_xy=(1.5, 0.0),
                                   scale_with_length=True))
        w0, w1 = 0.65, 0.65 * CORRIDOR_WIDTH_RATIO
        self.assertAlmostEqual(corridor['defn']['w1_eff'],
                               w0 + 0.5 * (w1 - w0), places=9)
        self.assertAlmostEqual(corridor['halfWidth'][-1],
                               w0 + 0.5 * (w1 - w0), places=9)


class TestTheWidthParameters(unittest.TestCase):
    """What stack_params.yaml ships."""

    def test_the_length_scaling_ships_off(self):
        self.assertIs(get_value('corr_width_scale_with_length'), False)

    def test_the_end_half_width_is_derived_not_a_key(self):
        with self.assertRaises(KeyError):
            get_value('corr_half_width_end_m')

    def test_the_start_half_width_ships_as_a_positive_float(self):
        start = get_value('corr_half_width_start_m')
        self.assertIsInstance(start, float)
        self.assertGreater(start, 0.0)


if __name__ == '__main__':
    unittest.main()
