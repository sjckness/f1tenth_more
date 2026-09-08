"""
Regression coverage for the corridor-marker lifetime overflow.

_corridor_line_marker() used to pack the whole marker lifetime into
builtin_interfaces/Duration's nanosec field:

    m.lifetime.sec = 0
    m.lifetime.nanosec = int(3.0 * self.corridor_update_period * 1e9)

nanosec is a uint32 that the generated message class asserts on, so any
corridor_update_period above ~1.43s made that assignment raise
"AssertionError: The 'nanosec' field must be an unsigned integer in
[0, 4294967295]". stack_params.yaml's own default is 10.0, so this fired on
the very first _publish_corridor_markers() call of every run -- the
AssertionError escaped control_loop() through rclpy's executor and killed
mpc_corr, navigation crash-looped until the supervisor's restart budget was
exhausted, and /mission/start_mission then failed its preflight with
"node 'mpc_corr' not found in the ROS graph" (found live 2026-09-07).

Same "testable without constructing a real MPCController" shape as
test_corridor_turn_shape.py and friends: a duck-typed stand-in carrying only
the attribute the method under test actually reads.

Run standalone: python3 -m pytest test/test_corridor_marker_lifetime.py -v
"""

import unittest

from mpc_controller.MPC_corr import MPCController

_UINT32_LIMIT = 4294967296


class _FakeStamp:
    def to_msg(self):
        # builtin_interfaces/Time; the marker only forwards this, never reads it
        from builtin_interfaces.msg import Time
        return Time()


class _FakeController:
    """Only _corridor_line_marker's own reads: corridor_update_period."""

    def __init__(self, corridor_update_period):
        self.corridor_update_period = corridor_update_period

    _corridor_line_marker = MPCController._corridor_line_marker


def _marker_for(period):
    ctrl = _FakeController(period)
    return ctrl._corridor_line_marker(
        0, 'corridor', [0.0, 1.0], [0.0, 0.0], _FakeStamp(), (1.0, 1.0, 1.0, 1.0))


class TestCorridorMarkerLifetime(unittest.TestCase):

    def test_nanosec_stays_in_uint32_range_for_every_plausible_period(self):
        # 10.0 is the value that crashed live -- someone setting the PERIOD to
        # 10.0 meaning "10 Hz". It has never been stack_params.yaml's default
        # (an earlier version of this comment claimed it was; the yaml default
        # is 1.0, and is now the single source of truth -- MPC_corr.py reads it
        # through get_value rather than mirroring a literal).
        # 1.4316 straddles the old formula's overflow threshold exactly.
        for period in (0.05, 0.1, 0.5, 1.0, 1.4316, 1.5, 2.0, 5.0, 10.0, 60.0):
            with self.subTest(corridor_update_period=period):
                m = _marker_for(period)
                self.assertGreaterEqual(m.lifetime.nanosec, 0)
                self.assertLess(m.lifetime.nanosec, _UINT32_LIMIT)
                self.assertGreaterEqual(m.lifetime.sec, 0)

    def test_a_period_of_ten_seconds_does_not_raise(self):
        """The exact live failure: 3 * 10.0 s overflowed nanosec and crashed
        the node. Reachable by launch arg, not the default."""
        m = _marker_for(10.0)
        self.assertEqual(m.lifetime.sec, 30)
        self.assertEqual(m.lifetime.nanosec, 0)

    def test_total_lifetime_is_three_times_the_update_period(self):
        """The overflow fix must not change the intended duration -- the
        marker still has to outlive a normal rebuild by 3x (see the method's
        own comment for why it expires at all)."""
        for period in (0.05, 0.1, 1.0, 1.5, 10.0):
            with self.subTest(corridor_update_period=period):
                m = _marker_for(period)
                total = m.lifetime.sec + m.lifetime.nanosec / 1e9
                self.assertAlmostEqual(total, 3.0 * period, places=6)

    def test_sub_second_period_still_uses_nanosec(self):
        """A fast rebuild rate must not be rounded away to a 0s lifetime."""
        m = _marker_for(0.1)
        self.assertEqual(m.lifetime.sec, 0)
        self.assertEqual(m.lifetime.nanosec, 300000000)


if __name__ == '__main__':
    unittest.main()
