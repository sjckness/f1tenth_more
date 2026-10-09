"""
CorridorManager on its own: anchor, cache, rebuild gate, invalidation.

No node and no tf2: the map -> odom lookup is a fake callable with the node's
_lookup_map_odom(max_age_sec=None) signature, which records how it was called
and returns whatever the test says. The age guard's own arithmetic (how old a
transform is against the node's clock) is pinned in test_map_odom_age_guard.py;
this file pins what the manager does with the answer.

Run standalone: python3 -m pytest test/test_corridor_manager.py -v
"""

import math
import subprocess
import sys

from mpc_controller.corridor.geometry import pose_map_to_odom, pose_odom_to_map
from mpc_controller.corridor.manager import CorridorManager
import pytest

_TF = (1.25, -0.5, 0.3)
_ANCHOR_MAP = (3.0, -1.0, 0.4)
_HELD = (9.0, 9.0, 9.0)


class _Logger:
    def __init__(self):
        self.records = []

    def warn(self, text, **kwargs):
        self.records.append(('warn', text, kwargs))


class _Lookup:
    """Fake tf lookup: records each call's max_age_sec, returns `result`."""

    def __init__(self, result=_TF):
        self.result = result
        self.calls = []

    def __call__(self, max_age_sec=None):
        self.calls.append(max_age_sec)
        return self.result


def _manager(period=1.0, max_age=0.2):
    return CorridorManager(update_period=period, map_odom_max_age_sec=max_age,
                           logger=_Logger())


class TestCapture:

    def test_seeds_the_raw_odom_anchor_and_converts_it_to_map(self):
        m = _manager()
        lookup = _Lookup()
        assert m.capture_anchor(1.0, 2.0, 0.25, lookup) == _TF
        assert m.anchor_odom == (1.0, 2.0, 0.25)
        assert m.anchor_map == pose_odom_to_map(1.0, 2.0, 0.25, *_TF)

    def test_applies_no_age_guard(self):
        lookup = _Lookup()
        _manager().capture_anchor(1.0, 2.0, 0.25, lookup)
        assert lookup.calls == [None]

    def test_no_edge_keeps_the_raw_anchor_and_no_map_anchor(self):
        m = _manager()
        assert m.capture_anchor(1.0, 2.0, 0.25, _Lookup(None)) is None
        assert m.anchor_odom == (1.0, 2.0, 0.25)
        assert m.anchor_map is None

    def test_a_new_capture_replaces_the_previous_moves_anchor(self):
        m = _manager()
        m.capture_anchor(1.0, 2.0, 0.25, _Lookup())
        m.capture_anchor(5.0, 6.0, -1.0, _Lookup(None))
        assert m.anchor_odom == (5.0, 6.0, -1.0)
        assert m.anchor_map is None

    def test_the_yaw_is_stored_as_a_float(self):
        m = _manager()
        m.capture_anchor(1.0, 2.0, 1, _Lookup(None))
        assert type(m.anchor_odom[2]) is float

    def test_clear_drops_both(self):
        m = _manager()
        m.capture_anchor(1.0, 2.0, 0.25, _Lookup())
        m.clear_anchor()
        assert m.anchor_map is None and m.anchor_odom is None


class TestRefresh:

    def test_without_a_map_anchor_it_looks_nothing_up(self):
        m = _manager()
        m.anchor_odom = _HELD
        lookup = _Lookup()
        m.refresh_anchor(lookup)
        assert lookup.calls == []
        assert m.anchor_odom is _HELD

    def test_it_asks_with_the_configured_max_age(self):
        m = _manager(max_age=0.37)
        m.anchor_map = _ANCHOR_MAP
        lookup = _Lookup()
        m.refresh_anchor(lookup)
        assert lookup.calls == [0.37]

    def test_a_good_edge_reprojects(self):
        m = _manager()
        m.anchor_map = _ANCHOR_MAP
        m.anchor_odom = _HELD
        m.refresh_anchor(_Lookup())
        assert m.anchor_odom == pose_map_to_odom(*_ANCHOR_MAP, *_TF)
        assert m._logger.records == []

    def test_no_edge_holds_and_says_so_once_throttled(self):
        m = _manager()
        m.anchor_map = _ANCHOR_MAP
        m.anchor_odom = _HELD
        m.refresh_anchor(_Lookup(None))
        assert m.anchor_odom is _HELD
        assert m._logger.records == [(
            'warn',
            'ANCHOR/hold | map -> odom unavailable OR STALE this tick: holding '
            'last reprojected anchor (NOT reverting to the raw odom anchor)',
            {'throttle_duration_sec': 5.0})]

    def test_capture_then_refresh_through_the_same_edge_is_identity(self):
        m = _manager()
        m.capture_anchor(1.0, 2.0, 0.25, _Lookup())
        m.refresh_anchor(_Lookup())
        for got, want in zip(m.anchor_odom, (1.0, 2.0, 0.25)):
            assert got == pytest.approx(want, abs=1e-12)


def _warm(m):
    m.cached_corridor = {'corridor': 1}
    m.last_corridor_time = 10.0
    m.last_corridor_stamp = 'stamp'
    m.cached_pref_nom = 'pref'
    m.prev_corridor_ref = 'ref'
    m.object_arc_active = True
    m.anchor_map = _ANCHOR_MAP
    m.anchor_odom = _HELD
    return m


class TestInvalidate:

    def test_it_clears_the_cache_and_the_rebuild_memory(self):
        m = _warm(_manager())
        m.invalidate()
        assert m.cached_corridor is None
        assert m.last_corridor_time is None
        assert m.last_corridor_stamp is None
        assert m.cached_pref_nom is None
        assert m.prev_corridor_ref is None
        assert m.object_arc_active is False

    def test_it_leaves_the_anchor_and_the_configuration_alone(self):
        m = _warm(_manager(period=0.7, max_age=0.3))
        m.invalidate()
        assert m.anchor_map == _ANCHOR_MAP
        assert m.anchor_odom is _HELD
        assert (m.update_period, m.map_odom_max_age_sec) == (0.7, 0.3)


class _Moved:
    def __init__(self, value):
        self.value = value
        self.calls = 0

    def __call__(self):
        self.calls += 1
        return self.value


class TestRebuildGate:

    def test_nothing_cached_rebuilds_without_asking_about_the_target(self):
        m = _manager()
        moved = _Moved(False)
        assert m.needs_rebuild(0.0, moved) is True
        assert moved.calls == 0

    def test_a_cleared_time_alone_forces_a_rebuild(self):
        m = _warm(_manager())
        m.last_corridor_time = None
        assert m.needs_rebuild(10.1, _Moved(False)) is True

    def test_a_cleared_corridor_alone_forces_a_rebuild(self):
        m = _warm(_manager())
        m.cached_corridor = None
        assert m.needs_rebuild(10.1, _Moved(False)) is True

    def test_inside_the_period_it_holds(self):
        m = _warm(_manager(period=1.0))
        moved = _Moved(False)
        assert m.needs_rebuild(10.5, moved) is False
        assert moved.calls == 1

    def test_the_period_boundary_is_inclusive(self):
        m = _warm(_manager(period=1.0))
        moved = _Moved(False)
        assert m.needs_rebuild(11.0, moved) is True
        assert moved.calls == 0

    def test_target_motion_pre_empts_the_period(self):
        m = _warm(_manager(period=1.0))
        assert m.needs_rebuild(10.5, _Moved(True)) is True


class TestMaybeRebuild:

    def test_a_due_rebuild_builds_once_and_records_when(self):
        m = _manager()
        built = []

        def build():
            built.append(1)
            return {'new': True}

        assert m.maybe_rebuild(20.0, 'stamp-20', build, _Moved(False)) is True
        assert built == [1]
        assert m.cached_corridor == {'new': True}
        assert m.last_corridor_time == 20.0
        assert m.last_corridor_stamp == 'stamp-20'

    def test_a_held_rebuild_does_not_build_or_touch_the_cache(self):
        m = _warm(_manager(period=1.0))

        def build():
            raise AssertionError('built inside the period')

        assert m.maybe_rebuild(10.5, 'stamp', build, _Moved(False)) is False
        assert m.cached_corridor == {'corridor': 1}
        assert m.last_corridor_time == 10.0
        assert m.last_corridor_stamp == 'stamp'

    def test_the_cache_is_whatever_the_build_returned(self):
        corridor = {'xc': [0.0, 1.0]}
        m = _manager()
        m.maybe_rebuild(0.0, None, lambda: corridor, _Moved(False))
        assert m.cached_corridor is corridor


def test_the_corridor_package_does_not_import_rclpy():
    """geometry, manager and markers construct values; rclpy stays in the node."""
    probe = ('import sys\n'
             'import mpc_controller.corridor.geometry\n'
             'import mpc_controller.corridor.manager\n'
             'import mpc_controller.corridor.markers\n'
             'print("rclpy" in sys.modules)\n')
    out = subprocess.run([sys.executable, '-c', probe], capture_output=True,
                         text=True, check=True).stdout.strip()
    assert out == 'False'


def test_pose_helpers_round_trip():
    back = pose_map_to_odom(*pose_odom_to_map(2.0, -0.5, 0.9, *_TF), *_TF)
    for got, want in zip(back, (2.0, -0.5, 0.9)):
        assert got == pytest.approx(want, abs=1e-12)
    assert math.isfinite(back[2])
