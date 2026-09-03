"""Regression tests for the obstacle-avoidance test configuration added by the
2026-09-01 mission-analysis follow-up (see mission_analysis_2026-09-01.md).

Every assertion here corresponds to a specific finding in that analysis:

  - IsSystemOverheated caused 3 of 9 stop episodes, all false positives fired
    by single 1 Hz gpu_percent samples of 96.8-99.1% while temperatures sat at
    49-55 C against a 100 C limit. -> TestSystemOverheatedLoadDebounce
  - The camera obstacle lane starved the mission subtree for a whole run
    (root is a memory=False Selector, so lanes after a succeeding one are
    never ticked). -> TestCreateRootLaneGating
  - Both Stop instances stamped 'base_link', making emergency and obstacle
    stops indistinguishable in a bag. -> TestStopFrameIdsAreDistinct
"""

import py_trees
import pytest

from f1tenth_behavior.behaviours.is_system_overheated import IsSystemOverheated


class _Sample:
    """Minimal stand-in for f1tenth_messages/SystemStatus -- only the four
    fields the condition reads."""

    def __init__(self, cpu_percent=10.0, gpu_percent=10.0,
                 cpu_temp_c=50.0, gpu_temp_c=45.0):
        self.cpu_percent = cpu_percent
        self.gpu_percent = gpu_percent
        self.cpu_temp_c = cpu_temp_c
        self.gpu_temp_c = gpu_temp_c


def _cond(**kwargs):
    kwargs.setdefault('max_temp_c', 100.0)
    kwargs.setdefault('max_load_percent', 95.0)
    return IsSystemOverheated(**kwargs)


class TestSystemOverheatedLoadDebounce:

    def test_no_data_does_not_trip(self):
        assert _cond().update() == py_trees.common.Status.FAILURE

    def test_single_gpu_load_spike_does_not_trip(self):
        """THE regression this pass exists for. Run 15-09-43: one 96.8% GPU
        sample stopped the car for 1.0 s (onset 0.01 s after the sample) with
        2.65-4.79 m of clear space ahead. One sample must no longer be enough."""
        c = _cond(enable_load_trip=True, load_trip_consecutive_samples=3)
        c._callback(_Sample(gpu_percent=96.8))
        assert c.update() == py_trees.common.Status.FAILURE

    def test_two_consecutive_spikes_still_do_not_trip(self):
        """Run 15-08-12's was the longest observed false positive: 99.1% then
        95.1%, two consecutive samples, 2.11 s of stop. Still below default."""
        c = _cond(enable_load_trip=True, load_trip_consecutive_samples=3)
        c._callback(_Sample(gpu_percent=99.1))
        c._callback(_Sample(gpu_percent=95.1))
        assert c.update() == py_trees.common.Status.FAILURE

    def test_sustained_load_does_trip_when_enabled(self):
        """Debounced, not disabled: genuinely sustained load must still act."""
        c = _cond(enable_load_trip=True, load_trip_consecutive_samples=3)
        for _ in range(3):
            c._callback(_Sample(gpu_percent=99.0))
        assert c.update() == py_trees.common.Status.SUCCESS
        assert 'load' in c.tripped_reason

    def test_streak_resets_on_any_normal_sample(self):
        """gpu_percent ranged 3.0-99.1% between consecutive samples in run
        15-08-12, so the streak must be broken by a single normal sample --
        otherwise bursty load accumulates into a trip over time."""
        c = _cond(enable_load_trip=True, load_trip_consecutive_samples=3)
        c._callback(_Sample(gpu_percent=99.0))
        c._callback(_Sample(gpu_percent=99.0))
        c._callback(_Sample(gpu_percent=3.0))
        c._callback(_Sample(gpu_percent=99.0))
        assert c.update() == py_trees.common.Status.FAILURE

    def test_load_trip_disabled_by_default_never_trips(self):
        c = _cond(enable_load_trip=False)
        for _ in range(50):
            c._callback(_Sample(cpu_percent=99.0, gpu_percent=99.0))
        assert c.update() == py_trees.common.Status.FAILURE

    @pytest.mark.parametrize('field', ['cpu_temp_c', 'gpu_temp_c'])
    def test_temperature_still_trips_on_a_single_sample(self, field):
        """The thermal guard is NOT debounced and NOT gated -- it protects
        against hardware self-damage and never false-positived. It must keep
        working even with the load trip disabled entirely."""
        c = _cond(enable_load_trip=False)
        c._callback(_Sample(**{field: 101.0}))
        assert c.update() == py_trees.common.Status.SUCCESS
        assert field in c.tripped_reason

    def test_missing_jtop_reads_zero_and_fails_safe(self):
        """system_observer_node publishes 0.0 for GPU fields when jtop isn't
        connected -- that must never look like a trip."""
        c = _cond(enable_load_trip=True, load_trip_consecutive_samples=1)
        for _ in range(5):
            c._callback(_Sample(gpu_percent=0.0, gpu_temp_c=0.0))
        assert c.update() == py_trees.common.Status.FAILURE


class TestCreateRootLaneGating:
    """create_root() imports rclpy-dependent behaviours at module scope, so
    these are constructed without a running ROS graph -- fine, because nothing
    here calls setup()."""

    @staticmethod
    def _root(**kwargs):
        from f1tenth_behavior.behavior_executor_node import create_root
        return create_root(**kwargs)

    def _lane_names(self, **kwargs):
        return [c.name for c in self._root(**kwargs).children]

    def test_camera_obstacle_lane_absent_by_default(self):
        """Structurally absent, not ticked-and-failing: a lane that exists but
        never succeeds is indistinguishable in a BT snapshot from one whose
        condition simply isn't tripping."""
        assert 'handle_obstacle' not in self._lane_names()

    def test_camera_obstacle_lane_present_when_enabled(self):
        names = self._lane_names(enable_camera_obstacle_stop=True)
        assert 'handle_obstacle' in names
        # Ordering must be unchanged when it is restored.
        assert names.index('handle_obstacle') == names.index('emergency') + 1

    def test_mission_lane_always_present_and_after_emergency(self):
        for kwargs in ({}, {'enable_camera_obstacle_stop': True}):
            names = self._lane_names(**kwargs)
            assert names[0] == 'emergency'
            assert 'mission' in names and 'navigation' in names
            assert names.index('mission') < names.index('navigation')

    def test_proximity_condition_gated_by_its_own_flag(self):
        def emergency_children(**kw):
            root = self._root(**kw)
            emergency = next(c for c in root.children if c.name == 'emergency')
            cond = next(c for c in emergency.children
                        if c.name == 'emergency_condition')
            return [c.name for c in cond.children]

        assert 'IsProximityTooClose' in emergency_children()
        assert 'IsProximityTooClose' not in emergency_children(
            enable_lidar_safety_stop=False)

    def test_proximity_thresholds_are_passed_through_not_derived(self):
        """These are last-resort contact thresholds and must NOT pick up
        car_radius + margins -- that derivation expresses the opposite intent
        (a margin ON TOP of the car's radius)."""
        root = self._root(proximity_front_threshold_m=0.12,
                          proximity_side_threshold_m=0.11,
                          car_radius=0.20, obstacle_safety_margin_m=0.12)
        emergency = next(c for c in root.children if c.name == 'emergency')
        cond = next(c for c in emergency.children if c.name == 'emergency_condition')
        prox = next(c for c in cond.children if c.name == 'IsProximityTooClose')
        assert prox.front_distance_threshold == pytest.approx(0.12)
        assert prox.lidar_distance_threshold == pytest.approx(0.11)


class TestStopFrameIdsAreDistinct:
    """Both Stop instances publish byte-identical zero-speed commands on the
    same /safety_stop mux lane. frame_id is the ONLY thing that tells a bag
    which one fired -- during the 2026-09-01 analysis both stamped 'base_link',
    so every stop had to be re-bucketed by replaying predicates offline."""

    def test_emergency_and_obstacle_stops_differ(self):
        from f1tenth_behavior.behavior_executor_node import create_root
        root = create_root(enable_camera_obstacle_stop=True)
        frames = {}
        for lane in root.children:
            if lane.name not in ('emergency', 'handle_obstacle'):
                continue
            stop = next(c for c in lane.children if c.name == 'Stop')
            frames[lane.name] = stop.frame_id
        assert frames['emergency'] != frames['handle_obstacle']
        assert frames == {
            'emergency': 'base_link/emergency',
            'handle_obstacle': 'base_link/obstacle',
        }
