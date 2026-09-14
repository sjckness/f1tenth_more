"""The d_wall correction's mpc_corr half: where it lands, and every case in
which it must be exactly zero.

WHAT IS AND IS NOT TESTED HERE. This file covers the seam -- that the
correction reaches psi_base on the straight drive branch, that it reaches
nothing else, and that every degraded input produces 0.0 rather than a held
value. The control law itself (the sign, the gain, the deadband, the fade, the
rate limit) lives in f1tenth_perception's wall_distance.py and is covered by
test_wall_distance.py; duplicating it here would give the same constant two
spellings.

THE SEAM, in one line: docs/wall_turn_investigation.md establishes that the MPC
does not own a wall_turn's exit heading (f1tenth_behavior's orientation_delta
stop_condition ends the move at 90 degrees of accumulated yaw whatever the
corridor asked for), so the correction goes to the STRAIGHT corridor the car
drives afterwards and not to the turn's own dpsi_this -- which is bounded by
min(|dpsi_rem|, ...) and therefore has no authority left at exit.

Run standalone: python3 -m pytest test/test_d_wall_correction.py -v
"""

import json
import math
import os
import unittest


class _ClockFake:
    def __init__(self, now_sec):
        self._ns = int(now_sec * 1e9)

    def now(self):
        return self

    @property
    def nanoseconds(self):
        return self._ns


class _LoggerFake:
    def __init__(self):
        self.warns = []

    def warn(self, message, **_kwargs):
        self.warns.append(message)

    def info(self, message, **_kwargs):
        pass


class _Fake:
    """The smallest stand-in _fresh_d_wall_correction actually reads."""

    def __init__(self, *, value=0.1, stamp=100.0, now_sec=100.0,
                 enable=True, max_age=0.5):
        self.d_wall_correction = value
        self.d_wall_correction_stamp_sec = stamp
        self.corr_d_wall_correction_enable = enable
        self.corr_d_wall_max_age_sec = max_age
        self._clock = _ClockFake(now_sec)
        self._logger = _LoggerFake()

    def get_clock(self):
        return self._clock

    def get_logger(self):
        return self._logger


def _fresh(fake):
    from mpc_controller.MPC_corr import MPCController
    return MPCController._fresh_d_wall_correction(fake)


class TestZeroNeverLastKnown(unittest.TestCase):
    """Every degraded case is 0.0. The natural implementation holds the last
    value, and a held correction is a confident heading toward a position
    nothing can see any more."""

    def test_a_fresh_enabled_correction_passes_through(self):
        self.assertAlmostEqual(_fresh(_Fake(value=0.137)), 0.137, places=12)

    def test_a_fresh_negative_correction_passes_through_unchanged(self):
        """Sign is the perception node's business, not this accessor's. A
        clamp or an abs() here would silently break one side of the turn."""
        self.assertAlmostEqual(_fresh(_Fake(value=-0.137)), -0.137, places=12)

    def test_disabled_is_zero_even_with_a_fresh_message(self):
        self.assertEqual(_fresh(_Fake(value=0.2, enable=False)), 0.0)

    def test_no_message_yet_is_zero(self):
        self.assertEqual(_fresh(_Fake(value=0.2, stamp=None)), 0.0)

    def test_a_stale_correction_is_zero_and_warns(self):
        fake = _Fake(value=0.2, stamp=100.0, now_sec=101.0, max_age=0.5)
        self.assertEqual(_fresh(fake), 0.0)
        self.assertTrue(fake.get_logger().warns,
                        'a stale correction must say so: it means the node died')

    def test_exactly_at_the_age_limit_is_still_fresh(self):
        fake = _Fake(value=0.2, stamp=100.0, now_sec=100.5, max_age=0.5)
        self.assertAlmostEqual(_fresh(fake), 0.2, places=12)

    def test_a_nan_correction_is_zero(self):
        self.assertEqual(_fresh(_Fake(value=math.nan)), 0.0)

    def test_an_infinite_correction_is_zero(self):
        self.assertEqual(_fresh(_Fake(value=math.inf)), 0.0)
        self.assertEqual(_fresh(_Fake(value=-math.inf)), 0.0)


class TestItLandsOnPsiBaseAndNowhereElse(unittest.TestCase):

    def _source(self):
        path = os.path.join(os.path.dirname(__file__), '..', 'mpc_controller',
                            'MPC_corr.py')
        with open(path) as f:
            return f.read()

    def test_the_only_application_is_on_the_straight_branch(self):
        """One call site. A second would mean two authorities on the same
        correction, which is the failure the 'ONE source of truth' note on the
        wall_turn branch exists to prevent."""
        source = self._source()
        self.assertEqual(source.count('dpsi_d_wall = '), 1)
        self.assertIn('psiEnd = psi_base + dpsi_d_wall', source)

    def test_it_is_read_through_getattr_so_the_corridor_stand_ins_survive(self):
        """The corridor tests build duck-typed _FakeMPC objects that carry only
        the fields the geometry under test needs. A bare method call here makes
        every one of them raise AttributeError instead of selecting a shape --
        which is exactly what happened when this was first written."""
        source = self._source()
        self.assertIn("getattr(self, '_fresh_d_wall_correction', None)", source)

    def test_the_wall_turn_branch_does_not_touch_it(self):
        """dpsi_this stays the one source of truth for psiEnd, psiRefTurn and
        the S-curve on a wall_turn corridor."""
        source = self._source()
        wall_turn_branch = source.split("if drive_cmd['mode'] == 'wall_turn':")[1]
        wall_turn_branch = wall_turn_branch.split('            else:')[0]
        self.assertNotIn('dpsi_d_wall', wall_turn_branch)
        self.assertNotIn('d_wall_correction', wall_turn_branch)

    def test_the_subscription_is_unconditional(self):
        """Subscribed even when disabled, so Stage 4 of
        docs/bringup_checklist.md can record a real turn with the correction
        observed and not applied."""
        source = self._source()
        head, _, tail = source.partition('self.sub_d_wall_correction = self.create_subscription(')
        self.assertTrue(tail, 'the subscription is gone')
        # The enable flag must not appear as a guard immediately above it.
        preceding = head.rsplit('\n\n', 1)[-1]
        self.assertNotIn('if self.corr_d_wall_correction_enable', preceding)


class TestTheMissionThatExercisesIt(unittest.TestCase):
    """Option B has no effect at all unless a mission puts a move AFTER the
    wall_turn: on a terminal move AdvanceMove publishes /mpc/hold, and
    control_loop returns before the corridor rebuild, so no post-exit corridor
    is ever built. This asserts the one mission that does."""

    def _missions_dir(self):
        return os.path.join(os.path.dirname(__file__), '..', '..', '..',
                            'f1tenth_behavior', 'missions')

    def _load(self, name):
        with open(os.path.join(self._missions_dir(), name)) as f:
            return json.load(f)

    def test_wall_turn_then_straight_has_a_straight_move_after_the_turn(self):
        mission = self._load('wall_turn_then_straight.json')
        modes = [m['drive']['mode'] for m in mission['moves']]
        self.assertIn('wall_turn', modes)
        turn_at = modes.index('wall_turn')
        self.assertLess(turn_at, len(modes) - 1, 'the turn must not be the last move')
        self.assertEqual(modes[turn_at + 1], 'straight')

    def test_only_its_last_move_is_terminal(self):
        """schema 3.0 requires terminal on the last move and forbids it
        anywhere else -- and the whole point here is that the TURN is not
        terminal."""
        mission = self._load('wall_turn_then_straight.json')
        flags = [m.get('terminal', False) for m in mission['moves']]
        self.assertEqual(flags, [False] * (len(flags) - 1) + [True])

    def test_it_parses_against_the_real_mission_schema(self):
        import sys
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..',
                                        'f1tenth_behavior'))
        from f1tenth_behavior.mission.mission_config import parse_mission
        path = os.path.join(self._missions_dir(), 'wall_turn_then_straight.json')
        mission = parse_mission(self._load('wall_turn_then_straight.json'))
        self.assertEqual(mission.mission_id, 'wall_turn_then_straight')
        self.assertEqual(mission.schema_version, '3.0')
        self.assertTrue(os.path.exists(path))

    def test_every_other_wall_turn_mission_is_still_terminal_on_the_turn(self):
        """Not a thing to fix -- a deliberate record of why this new mission
        had to be written rather than an existing one reused."""
        for name in ('wall_turn.json', 'turn_90_left.json', 'drive_turn_180.json',
                     'drive_stop_2m_from_wall.json'):
            mission = self._load(name)
            last = mission['moves'][-1]
            self.assertEqual(last['drive']['mode'], 'wall_turn', name)
            self.assertTrue(last.get('terminal', False), name)


if __name__ == '__main__':
    unittest.main()
