"""front_clearance's debounce_ticks, and obstacle_distance_below's
forward_only source selection.

Both are new optional fields whose DEFAULTS are the load-bearing part: every
mission written before either existed must behave bit-identically, which
means debounce_ticks defaults to 1 (fire on the first satisfied tick) and
forward_only defaults to false (the omnidirectional topic). Those two
defaults get as much coverage here as the new behaviour does.
"""

from f1tenth_behavior.mission.condition_eval import (
    DEFAULT_DEBOUNCE_TICKS, ConditionDebouncer, EvalContext, debounce_ticks_for, evaluate)
from f1tenth_behavior.mission.mission_config import StopCondition


def _ctx(**overrides):
    base = dict(
        now=0.0, move_start_time=0.0, move_start_xy=(0.0, 0.0),
        current_xy=(0.0, 0.0), detected_classes={},
        min_obstacle_distance=None, front_clearance=None, default_distance=None,
    )
    base.update(overrides)
    return EvalContext(**base)


def _feed(debouncer, condition, ctx_values):
    """Run one raw evaluate() + debounce fold per element of ctx_values,
    where each element is the live front_clearance for that tick."""
    return [debouncer.update(condition, evaluate(condition, _ctx(front_clearance=v)))
            for v in ctx_values]


class TestDebounceTicksDefault:

    def test_default_is_one_tick(self):
        assert DEFAULT_DEBOUNCE_TICKS == 1

    def test_front_clearance_without_the_field_needs_one_tick(self):
        cond = StopCondition(type='front_clearance', params={'distance': 1.0})
        assert debounce_ticks_for(cond) == 1
        assert _feed(ConditionDebouncer(), cond, [0.5]) == [True]

    def test_a_non_debounceable_type_ignores_the_field_entirely(self):
        # mission_config rejects this combination at load time; if one ever
        # reaches here anyway it must not silently change that type's timing.
        cond = StopCondition(type='distance_reached',
                             params={'distance': 1.0, 'debounce_ticks': 9})
        assert debounce_ticks_for(cond) == DEFAULT_DEBOUNCE_TICKS

    def test_values_below_one_are_clamped_up(self):
        cond = StopCondition(type='front_clearance',
                             params={'distance': 1.0, 'debounce_ticks': 0})
        assert debounce_ticks_for(cond) == 1


class TestDebounceStreak:

    def _cond(self, ticks=3):
        return StopCondition(type='front_clearance',
                             params={'distance': 1.0, 'debounce_ticks': ticks})

    def test_three_consecutive_satisfied_ticks_are_needed(self):
        cond = self._cond()
        assert _feed(ConditionDebouncer(), cond, [0.5, 0.5, 0.5]) == [False, False, True]

    def test_one_unsatisfied_tick_restarts_the_streak(self):
        cond = self._cond()
        # Two below, one above (the noise dip this exists for), then three
        # below again -- only the last of those six may fire.
        assert _feed(ConditionDebouncer(), cond, [0.5, 0.5, 2.0, 0.5, 0.5, 0.5]) == [
            False, False, False, False, False, True]

    def test_it_stays_satisfied_once_the_streak_is_met(self):
        cond = self._cond()
        assert _feed(ConditionDebouncer(), cond, [0.5, 0.5, 0.5, 0.5]) == [
            False, False, True, True]

    def test_reset_drops_a_partial_streak(self):
        # What happens on every move change -- a streak from the previous
        # move must not count toward this one.
        cond = self._cond()
        d = ConditionDebouncer()
        assert _feed(d, cond, [0.5, 0.5]) == [False, False]
        d.reset()
        assert _feed(d, cond, [0.5, 0.5]) == [False, False]
        assert _feed(d, cond, [0.5]) == [True]

    def test_no_message_yet_never_starts_a_streak(self):
        # front_clearance None means "nothing received", which evaluate()
        # reports as not-satisfied -- it must not accumulate.
        cond = self._cond()
        assert _feed(ConditionDebouncer(), cond, [None, None, None]) == [
            False, False, False]

    def test_none_passes_through_without_disturbing_the_streak(self):
        # A stub type evaluates to None: categorically "cannot be judged",
        # neither a hit nor a miss.
        d = ConditionDebouncer()
        cond = self._cond()
        assert _feed(d, cond, [0.5, 0.5]) == [False, False]
        assert d.update(StopCondition(type='manual', params={}), None) is None
        assert d.streak == 2
        assert _feed(d, cond, [0.5]) == [True]


class TestObstacleDistanceForwardOnly:

    def _cond(self, **extra):
        params = {'distance': 1.0}
        params.update(extra)
        return StopCondition(type='obstacle_distance_below', params=params)

    def test_default_reads_the_omnidirectional_value(self):
        cond = self._cond()
        # Something close BEHIND the car (the omnidirectional topic sees it,
        # the forward one does not). Default behaviour: this fires -- which
        # is exactly what every pre-existing mission expects.
        ctx = _ctx(min_obstacle_distance=0.5, min_obstacle_distance_forward=99.0)
        assert evaluate(cond, ctx) is True

    def test_forward_only_reads_the_filtered_value(self):
        cond = self._cond(forward_only=True)
        ctx = _ctx(min_obstacle_distance=0.5, min_obstacle_distance_forward=99.0)
        assert evaluate(cond, ctx) is False

    def test_forward_only_fires_on_something_actually_ahead(self):
        cond = self._cond(forward_only=True)
        ctx = _ctx(min_obstacle_distance=0.5, min_obstacle_distance_forward=0.5)
        assert evaluate(cond, ctx) is True

    def test_forward_only_with_no_message_yet_is_not_satisfied(self):
        cond = self._cond(forward_only=True)
        ctx = _ctx(min_obstacle_distance=0.5, min_obstacle_distance_forward=None)
        assert evaluate(cond, ctx) is False

    def test_a_non_true_forward_only_selects_the_default_source(self):
        # Explicit `is True`, not truthiness -- mission_config rejects a
        # non-boolean, so this only matters for a hand-built condition.
        cond = self._cond(forward_only='yes')
        ctx = _ctx(min_obstacle_distance=0.5, min_obstacle_distance_forward=99.0)
        assert evaluate(cond, ctx) is True
