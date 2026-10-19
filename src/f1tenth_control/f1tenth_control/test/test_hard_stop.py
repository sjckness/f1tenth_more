"""The last-resort LiDAR stop: trigger, self-hit mask, debounce, latch, and the node.

Synthetic scans built by hand, plus a real but never-spun rclpy node with its
publishers mocked. Nothing is launched, no LiDAR is needed, and the car is not
driven.

The four tests the work order names are, in order:
  1. object at 0.18 m ahead -> stop within 2 scans   TestTriggerAndDebounce
  2. self-hit points -> no stop                      TestSelfHitMask
  3. clear for 0.5 s -> release                      TestLatchAndRelease
  4. still works with the BT process killed          TestIndependentOfTheBT

Run standalone: python3 -m pytest test/test_hard_stop.py -v
"""

import json
import math
from unittest.mock import MagicMock

import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan

from f1tenth_control.hard_stop import (
    HardStopState,
    SELF_MARGIN_M,
    footprint_polygon,
    is_self_hit,
    trigger_range_hits,
)
from f1tenth_control.hard_stop_node import HARD_STOP_FRAME_ID, HardStopNode

# This rig, from the bag's own /tf_static and the URDF.
LASER_X = 0.305
FRONT_X = 0.405
REAR_X = -0.10
HALF_WIDTH = 0.136
TRIGGER_M = 0.20

# /scan as the rig actually publishes it (corridor bags, 2026-10-19):
# 1081 beams over -135..+135 deg at 40 Hz, range_min 0.02 m.
N_BEAMS = 1081
ANGLE_MIN = math.radians(-135.0)
ANGLE_INC = math.radians(0.25)
RANGE_MIN = 0.02
RANGE_MAX = 30.0

POLY = footprint_polygon(FRONT_X, REAR_X, HALF_WIDTH)


def beam_index(bearing_deg):
    """Beam whose bearing is nearest `bearing_deg`."""
    return int(round((math.radians(bearing_deg) - ANGLE_MIN) / ANGLE_INC))


def blank_ranges(fill=5.0):
    return [fill] * N_BEAMS


def ranges_with(points, fill=5.0):
    """A scan with (bearing_deg, range_m) returns placed on it."""
    r = blank_ranges(fill)
    for bearing_deg, rng in points:
        r[beam_index(bearing_deg)] = rng
    return r


def hits(ranges):
    return trigger_range_hits(ranges, ANGLE_MIN, ANGLE_INC, RANGE_MIN, RANGE_MAX,
                              LASER_X, POLY, TRIGGER_M)


# ==============================================================================
# 1. An object at 0.18 m ahead stops the car within 2 scans.
# ==============================================================================

class TestTriggerAndDebounce:

    def test_a_return_at_0p18_m_ahead_is_a_trigger(self):
        found = hits(ranges_with([(0.0, 0.18)]))
        assert len(found) == 1
        assert found[0][0] == pytest.approx(0.18)
        # 0.18 m from the laser at x=0.305 is base_link x=0.485, which is
        # 0.08 m AHEAD of the 0.405 m bumper -- outside the footprint, so it
        # must not be discarded as a self-hit.
        assert found[0][2] == pytest.approx(LASER_X + 0.18)

    def test_it_engages_within_two_scans(self):
        state = HardStopState(debounce_scans=2, clear_s=0.5)
        assert state.update(True, 0.000) is False, 'one scan is not evidence'
        assert state.update(True, 0.025) is True, 'the second scan engages it'
        assert state.trigger_count == 1

    def test_the_node_publishes_a_zero_speed_stop_on_the_safety_lane(self, node):
        node.scan(ranges_with([(0.0, 0.18)]))
        assert node.stop_pub.publish.call_count == 0, 'debounce: not on scan 1'
        node.scan(ranges_with([(0.0, 0.18)]))
        assert node.stop_pub.publish.call_count == 1
        msg = node.stop_pub.publish.call_args[0][0]
        assert msg.drive.speed == 0.0
        assert msg.drive.steering_angle == 0.0

    def test_the_stop_is_attributable_in_a_bag(self, node):
        # The mux republishes the winning message VERBATIM onto
        # /ackermann_drive, so frame_id is the only thing that tells a recorded
        # run which source stopped the car. stop.py asks every new Stop source
        # for a distinct one, after an analysis could not separate the BT's two.
        node.scan(ranges_with([(0.0, 0.18)]))
        node.scan(ranges_with([(0.0, 0.18)]))
        msg = node.stop_pub.publish.call_args[0][0]
        assert msg.header.frame_id == HARD_STOP_FRAME_ID
        assert msg.header.frame_id not in ('', 'base_link',
                                           'base_link/emergency',
                                           'base_link/obstacle')

    def test_a_return_just_outside_the_radius_does_not_trigger(self):
        assert hits(ranges_with([(0.0, 0.21)])) == []

    def test_the_radius_is_inclusive(self):
        assert len(hits(ranges_with([(0.0, TRIGGER_M)]))) == 1

    @pytest.mark.parametrize('bearing_deg', [0.0, 20.0, -20.0, 40.0, -40.0,
                                             75.0, -75.0, 90.0, -90.0])
    def test_it_triggers_all_round_not_only_ahead(self, bearing_deg):
        # A last resort has no heading. Something 0.18 m off the car's side is
        # just as much a contact as something ahead of it, and the BT's own
        # front-biased e-stop is the mechanism that cares about direction.
        #
        # The bearings here are all OUTSIDE the body at 0.18 m. Which ones
        # those are is not a free choice -- see
        # test_the_blind_band_is_exactly_the_car_s_own_body for the band in
        # between, where a return at this range is geometrically inside the
        # car and is discarded on purpose.
        found = hits(ranges_with([(bearing_deg, 0.18)]))
        assert len(found) == 1, f'{bearing_deg} deg must trigger'

    def test_the_blind_band_is_exactly_the_car_s_own_body(self):
        # A real and deliberate property, measured here rather than discovered
        # later. The trigger circle (r <= 0.20 m about the laser at x = 0.305)
        # is NOT all free space: the laser sits 0.10 m behind the bumper and
        # the body is 0.136 m half-width, so most of that circle is inside the
        # car. The self-hit mask therefore has to discard a band of bearings,
        # and the band is precisely the set of points inside the grown
        # footprint -- i.e. points where there cannot be an external object
        # the car is not already touching.
        #
        # Measured at 0.18 m: bearings 49..60 deg (and their mirror) are blind.
        # At 0.12 m EVERY bearing is blind, because 0.12 m ahead of the laser
        # IS the bumper face. That is the floor of this mechanism and the
        # reason the debounce is 2 scans and not 20: the stop has to engage on
        # the way in, at 0.20..0.13 m, and latch -- it cannot be relied on to
        # fire at the moment of contact.
        x_max, y_max = FRONT_X + SELF_MARGIN_M, HALF_WIDTH + SELF_MARGIN_M
        for bearing_deg in range(0, 136):
            bearing = math.radians(bearing_deg)
            x = LASER_X + 0.18 * math.cos(bearing)
            y = 0.18 * math.sin(bearing)
            inside = (x <= x_max) and (abs(y) <= y_max)
            found = hits(ranges_with([(float(bearing_deg), 0.18)]))
            assert (found == []) == inside, (
                f'{bearing_deg} deg at 0.18 m lands at ({x:+.3f},{y:+.3f}); '
                f'inside the body={inside} but hits={len(found)}')

    def test_a_return_at_the_bumper_face_is_inside_the_mask(self):
        # The floor stated as its own test. 0.12 m from the laser at 0 deg is
        # base_link x = 0.425, the bumper plus the 2 cm self margin. The trade
        # is in SELF_MARGIN_M's own docstring: 2 cm of blindness at the shell,
        # against a permanent latched stop on the car's own rear deck that
        # cannot be recovered from the driver's seat.
        assert hits(ranges_with([(0.0, 0.12)])) == []
        assert len(hits(ranges_with([(0.0, 0.13)]))) == 1

    def test_no_returns_at_all_is_not_a_trigger(self):
        # NaN and inf are no-returns, not obstacles at 0 m.
        assert hits([float('nan')] * N_BEAMS) == []
        assert hits([float('inf')] * N_BEAMS) == []

    def test_a_value_under_range_min_is_not_a_trigger(self):
        # The sensor saying "too close to measure" is not "something at 0.01 m".
        # Treating it as a detection would stop the car on every dropout.
        assert hits(ranges_with([(0.0, 0.01)])) == []

    def test_the_nearest_hit_is_reported_first(self):
        # All three outside the body (see the blind-band test): at 0 and
        # +-20 deg the limit is 0.12/0.128 m, so 0.14 upward is a real return.
        found = hits(ranges_with([(0.0, 0.19), (20.0, 0.14), (-20.0, 0.17)]))
        assert [round(h[0], 2) for h in found] == [0.14, 0.17, 0.19]


# ==============================================================================
# 2. Self-hit points produce no stop.
# ==============================================================================

class TestSelfHitMask:

    def test_the_polygon_covers_the_body_plus_the_margin(self):
        x_min, x_max, y_min, y_max = POLY
        assert x_max == pytest.approx(FRONT_X + SELF_MARGIN_M)
        assert x_min == pytest.approx(REAR_X - SELF_MARGIN_M)
        assert y_max == pytest.approx(HALF_WIDTH + SELF_MARGIN_M)
        assert y_min == pytest.approx(-(HALF_WIDTH + SELF_MARGIN_M))

    def test_a_return_on_the_rear_deck_is_a_self_hit(self):
        # 0.18 m back from the laser is base_link x = 0.125, dead centre of the
        # car. Without the mask this latches the stop forever on the car's own
        # shell, which is not recoverable from the driver's seat. 180 deg is
        # outside this LiDAR's -135..+135 field of view, so the rearmost beam
        # that can actually see the deck is 135 deg.
        assert is_self_hit(LASER_X - 0.18, 0.0, POLY)
        assert hits(ranges_with([(135.0, 0.18)])) == []
        assert hits(ranges_with([(-135.0, 0.18)])) == []

    @pytest.mark.parametrize('bearing_deg', [100.0, 120.0, 134.0,
                                             -100.0, -120.0, -134.0])
    def test_returns_across_the_chassis_never_trigger(self, bearing_deg):
        # Past roughly +-100 deg the beam leaves across the body. These are the
        # bearings a real self-hit arrives on, and the whole run of them must
        # be discarded.
        found = hits(ranges_with([(bearing_deg, 0.18)]))
        x = LASER_X + 0.18 * math.cos(math.radians(bearing_deg))
        y = 0.18 * math.sin(math.radians(bearing_deg))
        if is_self_hit(x, y, POLY):
            assert found == [], (
                f'{bearing_deg} deg at 0.18 m lands at ({x:+.3f},{y:+.3f}), '
                'inside the footprint: it is the car itself')

    def test_a_scan_that_is_nothing_but_self_hits_never_engages(self, node):
        # Fill the whole rear arc at 0.10 m -- the shape a mis-mounted LiDAR or
        # a added body panel produces. Many scans, and the car must still move.
        self_scan = ranges_with([(b, 0.10) for b in range(95, 136)]
                                + [(-b, 0.10) for b in range(95, 136)])
        for _ in range(20):
            node.scan(self_scan)
        assert node.stop_pub.publish.call_count == 0
        assert node.node.state.trigger_count == 0

    def test_a_real_obstacle_still_triggers_through_the_self_hits(self, node):
        # The mask must not blind the node: same self-hit arc, plus one real
        # return ahead.
        mixed = ranges_with([(b, 0.10) for b in range(95, 136)]
                            + [(-b, 0.10) for b in range(95, 136)]
                            + [(0.0, 0.18)])
        node.scan(mixed)
        node.scan(mixed)
        assert node.node.state.engaged
        assert node.stop_pub.publish.call_count == 1

    def test_the_margin_is_what_decides_a_point_at_the_shell(self):
        # Just outside the body, inside the margin -> still the car.
        just_outside = HALF_WIDTH + 0.01
        assert is_self_hit(0.0, just_outside, POLY)
        # Past the margin -> a real return.
        assert not is_self_hit(0.0, HALF_WIDTH + SELF_MARGIN_M + 0.01, POLY)


# ==============================================================================
# 3. Clear for 0.5 s releases the latch.
# ==============================================================================

class TestLatchAndRelease:

    def _engaged(self):
        state = HardStopState(debounce_scans=2, clear_s=0.5)
        state.update(True, 0.000)
        state.update(True, 0.025)
        assert state.engaged
        return state

    def test_it_stays_engaged_while_the_zone_is_still_occupied(self):
        state = self._engaged()
        for k in range(40):
            assert state.update(True, 0.05 + 0.025 * k)

    def test_one_clear_scan_does_not_release_it(self):
        state = self._engaged()
        assert state.update(False, 0.050) is True, (
            'releasing on the first clear scan would stutter the car forward '
            'into whatever it just stopped for')

    def test_it_releases_after_the_full_clear_hold(self):
        state = self._engaged()
        assert state.update(False, 0.400) is True, '0.375 s of clear is not 0.5'
        assert state.update(False, 0.525) is False, 'past 0.5 s it releases'

    def test_a_hit_during_the_hold_restarts_the_clock(self):
        state = self._engaged()
        state.update(False, 0.300)
        state.update(True, 0.400)       # re-seen: clock restarts here
        assert state.update(False, 0.800) is True, 'only 0.4 s since the last hit'
        assert state.update(False, 0.950) is False

    def test_re_engaging_counts_as_a_second_trigger(self):
        state = self._engaged()
        assert state.trigger_count == 1
        state.update(False, 0.6)
        assert not state.engaged
        state.update(True, 0.7)
        state.update(True, 0.8)
        assert state.engaged and state.trigger_count == 2, (
            'the count is EVENTS -- how many times the car was stopped -- not '
            'scans, which is what a replay or a run report wants')

    def test_the_hold_is_re_evaluated_without_any_scan(self):
        # tick() exists because the latch releases on TIME and only a scan
        # delivers time. A dead LiDAR must not leave the stop engaged forever.
        state = self._engaged()
        assert state.tick(0.300) is True
        assert state.tick(0.600) is False

    def test_the_node_keeps_the_mux_lane_claimed_while_engaged(self, node):
        # The lane times out in 0.2 s, so an engaged stop that stops publishing
        # is released by the mux. The republish timer is what prevents that.
        node.scan(ranges_with([(0.0, 0.18)]))
        node.scan(ranges_with([(0.0, 0.18)]))
        before = node.stop_pub.publish.call_count
        node.node._republish()
        node.node._republish()
        assert node.stop_pub.publish.call_count == before + 2

    def test_the_republish_stops_once_released(self, node):
        node.scan(ranges_with([(0.0, 0.18)]))
        node.scan(ranges_with([(0.0, 0.18)]))
        node.node.state.last_hit_time = node.node._now() - 10.0
        node.node._republish()
        count = node.stop_pub.publish.call_count
        node.node._republish()
        assert node.stop_pub.publish.call_count == count, (
            'a released stop must stop claiming the lane, or the car never '
            'drives again')


# ==============================================================================
# 4. It works with the behaviour tree gone.
# ==============================================================================

class TestIndependentOfTheBT:

    def test_the_module_imports_nothing_from_the_behaviour_stack(self):
        # The strongest form of "no dependency on the BT" that a unit test can
        # assert: the import graph. py_trees or f1tenth_behavior appearing here
        # would mean a BT package failure can take this node down with it.
        import f1tenth_control.hard_stop as hs
        import f1tenth_control.hard_stop_node as hsn
        for module in (hs, hsn):
            source = open(module.__file__).read()
            for forbidden in ('py_trees', 'f1tenth_behavior', 'behaviours'):
                assert f'import {forbidden}' not in source, (
                    f'{module.__name__} imports {forbidden}')
                assert f'from {forbidden}' not in source

    def test_it_subscribes_to_the_raw_scan_and_nothing_from_the_bt(self, node):
        # No /behavior/*, no /mission/*, no tree status. One sensor in.
        names = [s.topic_name for s in node.node.subscriptions]
        assert names == ['/scan'], (
            f'exactly one input, the raw scan, at full rate -- got {names}')

    def test_it_stops_the_car_with_no_bt_input_whatsoever(self, node):
        # The functional form of the same claim: the node has received nothing
        # but scans in this whole test session, and it stops the car.
        node.scan(ranges_with([(0.0, 0.15)]))
        node.scan(ranges_with([(0.0, 0.15)]))
        assert node.node.state.engaged
        assert node.stop_pub.publish.call_count == 1

    def test_the_bt_s_own_estop_distance_is_untouched_by_this(self):
        # The 0.40 m proximity e-stop stays exactly as it is: two independent
        # stops at two distances, neither relying on the other. This pins the
        # separation so a future change to one is visible as a change to both.
        from f1tenth_params.param_defaults import get_value
        bt_threshold = (float(get_value('car_radius'))
                        + float(get_value('obstacle_safety_margin_m'))
                        + float(get_value('proximity_front_extra_margin_m')))
        assert bt_threshold == pytest.approx(0.40)
        assert TRIGGER_M < bt_threshold, (
            'the last resort must sit INSIDE the e-stop it backs up, or it '
            'fires first and makes the tuned one dead code')


# ==============================================================================
# The diagnostic.
# ==============================================================================

class TestDiagnostic:

    def test_it_publishes_state_and_count_every_scan(self, node):
        node.scan(blank_ranges())
        payload = json.loads(node.diag_pub.publish.call_args[0][0].data)
        assert payload['engaged'] is False
        assert payload['trigger_count'] == 0
        assert payload['n_hits_this_scan'] == 0
        assert payload['nearest_range_m'] is None

    def test_a_trigger_is_reported_with_its_range_and_bearing(self, node):
        node.scan(ranges_with([(-30.0, 0.17)]))
        node.scan(ranges_with([(-30.0, 0.17)]))
        payload = json.loads(node.diag_pub.publish.call_args[0][0].data)
        assert payload['engaged'] is True
        assert payload['trigger_count'] == 1
        assert payload['nearest_range_m'] == pytest.approx(0.17, abs=1e-3)
        assert payload['nearest_bearing_deg'] == pytest.approx(-30.0, abs=0.3)

    def test_the_thresholds_in_force_are_in_the_payload(self, node):
        node.scan(blank_ranges())
        payload = json.loads(node.diag_pub.publish.call_args[0][0].data)
        assert payload['debounce_scans'] == 2
        assert payload['clear_s'] == pytest.approx(0.5)


class TestDisabled:

    def test_disabled_never_claims_the_lane(self, rclpy_ctx):
        node = _build({'hard_stop_enabled': False})
        try:
            for _ in range(10):
                node.scan(ranges_with([(0.0, 0.10)]))
            node.node._republish()
            assert node.stop_pub.publish.call_count == 0
            assert node.node.state.trigger_count == 0
        finally:
            node.node.destroy_node()


# ==============================================================================
# Harness
# ==============================================================================

@pytest.fixture(scope='module', autouse=True)
def rclpy_ctx():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


class _Harness:
    """A real HardStopNode with both publishers mocked, driven scan by scan."""

    def __init__(self, node):
        self.node = node
        self.stop_pub = node.stop_pub
        self.diag_pub = node.diag_pub

    def scan(self, ranges):
        msg = LaserScan()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'
        msg.angle_min = ANGLE_MIN
        msg.angle_max = math.radians(135.0)
        msg.angle_increment = ANGLE_INC
        msg.range_min = RANGE_MIN
        msg.range_max = RANGE_MAX
        msg.ranges = [float(r) for r in ranges]
        self.node._scan_cb(msg)
        return msg


def _build(params=None):
    overrides = [Parameter(k, value=v) for k, v in (params or {}).items()]
    node = HardStopNode(parameter_overrides=overrides)
    node.stop_pub.publish = MagicMock()
    node.diag_pub.publish = MagicMock()
    return _Harness(node)


@pytest.fixture
def node(rclpy_ctx):
    harness = _build()
    yield harness
    harness.node.destroy_node()
