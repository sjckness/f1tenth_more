"""condition_eval.py tests -- evaluate()/EvalContext are plain
functions/dataclasses with zero rclpy dependency, so this needs no Node
construction/live BT tree, same "pure logic, static/synthetic only"
convention this package's own test_is_proximity_too_close.py already
establishes.

Focus: the front_clearance stop_condition's missing-data (None) handling --
added by the dual-EKF + costmap-derived-MPC-boundaries pass's own explicit
"confirm/fix the BT's handling of missing front_clearance so it fails safe"
instruction. condition_eval.py's own front_clearance branch already
returned False (never satisfied, never a crash) for ctx.front_clearance is
None BEFORE this pass touched anything -- confirmed by reading, not
assumed -- so no functional fix was needed there; this test file is what
was actually missing: nothing previously exercised that path at all. Named
test_missing_front_clearance_fails_safe_not_satisfied per the task's own
"BT stop_condition: confirm safe behavior on missing front_clearance" ask.

Run standalone: python3 -m pytest test/test_condition_eval.py -v
"""

import pytest

from f1tenth_behavior.mission.condition_eval import EvalContext, evaluate
from f1tenth_behavior.mission.mission_config import StopCondition


def _ctx(**overrides):
    """EvalContext with every non-defaulted field filled with a harmless
    placeholder, overridable per test -- same idea as this package's own
    fixture-building convention elsewhere (see mission/runtime.py's own
    test fixtures), kept local here since no other test file in this
    package needs EvalContext yet."""
    base = dict(
        now=0.0,
        move_start_time=0.0,
        move_start_xy=(0.0, 0.0),
        current_xy=(0.0, 0.0),
        detected_classes={},
        min_obstacle_distance=None,
        front_clearance=None,
        default_distance=None,
    )
    base.update(overrides)
    return EvalContext(**base)


# ==============================================================================
# front_clearance -- fail-safe missing-data handling. THE most important
# case per this pass's own explicit instruction -- see module docstring.
# ==============================================================================

class TestFrontClearanceStopCondition:

    def test_missing_front_clearance_fails_safe_not_satisfied(self):
        """No /costmap/front_clearance message has arrived yet (costmap_
        boundary_node not yet publishing -- exactly the live state given
        /slam/pose's own currently-known-broken status, see that node's own
        module docstring) -- must evaluate to False (stop_condition never
        satisfied), never raise, never silently treat missing data as
        'clear'."""
        cond = StopCondition(type='front_clearance', params={'distance': 1.0})
        ctx = _ctx(front_clearance=None)
        assert evaluate(cond, ctx) is False

    def test_real_value_below_threshold_is_satisfied(self):
        cond = StopCondition(type='front_clearance', params={'distance': 1.0})
        ctx = _ctx(front_clearance=0.5)
        assert evaluate(cond, ctx) is True

    def test_real_value_at_or_above_threshold_is_not_satisfied(self):
        cond = StopCondition(type='front_clearance', params={'distance': 1.0})
        ctx = _ctx(front_clearance=1.0)
        assert evaluate(cond, ctx) is False

    def test_large_finite_clearance_value_is_not_satisfied(self):
        # costmap_boundary_node's own "clear at least this far" convention
        # (max_range_m + 1.0, see costmap_boundary.py's own front_
        # clearance_from_extraction docstring) -- a real, large, finite
        # number, not math.inf, but must still behave as "not satisfied"
        # for any sane threshold.
        cond = StopCondition(type='front_clearance', params={'distance': 1.0})
        ctx = _ctx(front_clearance=6.0)
        assert evaluate(cond, ctx) is False


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
