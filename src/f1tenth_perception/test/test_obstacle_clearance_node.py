"""obstacle_clearance_node.py -- a real (never spun) rclpy Node whose callbacks
are called directly with publishers mocked, the test_swept_clearance_node.py
convention.

These defend the wiring; test_obstacle_clearance.py covers the geometry.
  * A wall at a known distance reaches /obstacle_clearance through the laser's
    static transform, once per scan, and invalid ranges are ignored -- including
    urg_node's finite 65.533 m no-return code.
  * No transform yet: the scan is skipped, nothing is published.
  * Contact: one /safety/event per episode, re-armed only after the clearance
    rises past the hysteresis band.
  * The /scan subscription cannot back-pressure urg_node.
  * The default footprint is swept_clearance's body.

Run standalone: python3 -m pytest test/test_obstacle_clearance_node.py -v
"""

import json
import math
from unittest.mock import MagicMock

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.parameter import Parameter
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from f1tenth_params.param_defaults import get_value
from f1tenth_perception.obstacle_clearance_node import ContactEdge, ObstacleClearanceNode
from f1tenth_perception.swept_corridor import BODY_FRONT_X_M, BODY_HALF_WIDTH_M, BODY_REAR_X_M

ANGLE_MIN = -math.pi
ANGLE_INCREMENT = math.radians(0.25)
N_BEAMS = 1440
LASER_X = 0.12
NO_RETURN = 65.533


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def make_node():
    created = []

    def make(**params):
        overrides = [Parameter(name, value=value) for name, value in params.items()]
        node = ObstacleClearanceNode(parameter_overrides=overrides)
        node.clearance_pub.publish = MagicMock()
        node.safety_pub.publish = MagicMock()
        created.append(node)
        return node

    yield make
    for node in created:
        node.destroy_node()


def _tf_static():
    t = TransformStamped()
    t.header.frame_id = 'base_link'
    t.child_frame_id = 'laser'
    t.transform.translation.x, t.transform.translation.z = LASER_X, 0.20
    t.transform.rotation.w = 1.0
    return TFMessage(transforms=[t])


def _scan(segments, background=math.inf):
    """360-degree scan in frame `laser` of line segments given in base_link;
    beams that hit nothing return `background`."""
    angles = ANGLE_MIN + np.arange(N_BEAMS) * ANGLE_INCREMENT
    d = np.stack([np.cos(angles), np.sin(angles)], 1)
    ranges = np.full(N_BEAMS, math.inf)
    origin = np.array([LASER_X, 0.0])
    for p0, p1 in segments:
        p0 = np.asarray(p0, dtype=float)
        e = np.asarray(p1, dtype=float) - p0
        w = p0 - origin
        denom = d[:, 0] * (-e[1]) - d[:, 1] * (-e[0])
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (w[0] * (-e[1]) - w[1] * (-e[0])) / denom
            u = (d[:, 0] * w[1] - d[:, 1] * w[0]) / denom
        hit = (np.abs(denom) > 1e-12) & (t > 0.0) & (u >= 0.0) & (u <= 1.0)
        ranges = np.where(hit, np.minimum(ranges, t), ranges)
    ranges = np.where(np.isinf(ranges), background, ranges)
    msg = LaserScan()
    msg.header.frame_id = 'laser'
    msg.angle_min = ANGLE_MIN
    msg.angle_max = ANGLE_MIN + (N_BEAMS - 1) * ANGLE_INCREMENT
    msg.angle_increment = ANGLE_INCREMENT
    msg.range_min = 0.02
    msg.range_max = 30.0
    msg.ranges = [float(r) for r in ranges]
    return msg


def _wall_ahead(gap):
    x = BODY_FRONT_X_M + gap
    return [((x, -5.0), (x, 5.0))]


def _published(publisher):
    return [call[0][0].data for call in publisher.publish.call_args_list]


class TestClearance:

    @pytest.mark.parametrize('gap', [0.3, 1.0, 2.5])
    def test_a_wall_ahead_reads_its_gap_to_the_front_of_the_footprint(self, make_node, gap):
        node = make_node()
        node._tf_static_cb(_tf_static())
        node._scan_cb(_scan(_wall_ahead(gap)))
        assert _published(node.clearance_pub) == [pytest.approx(gap, abs=1e-4)]

    def test_a_wall_beside_reads_its_gap_to_the_flank(self, make_node):
        node = make_node()
        node._tf_static_cb(_tf_static())
        y = BODY_HALF_WIDTH_M + 0.4
        node._scan_cb(_scan([((-3.0, y), (3.0, y))]))
        assert _published(node.clearance_pub) == [pytest.approx(0.4, abs=1e-4)]

    def test_one_message_per_scan(self, make_node):
        node = make_node()
        node._tf_static_cb(_tf_static())
        for gap in (2.0, 1.5, 1.0):
            node._scan_cb(_scan(_wall_ahead(gap)))
        assert _published(node.clearance_pub) == [
            pytest.approx(2.0, abs=1e-4), pytest.approx(1.5, abs=1e-4),
            pytest.approx(1.0, abs=1e-4)]

    def test_the_no_return_code_and_nan_are_not_obstacles(self, make_node):
        node = make_node()
        node._tf_static_cb(_tf_static())
        scan = _scan(_wall_ahead(1.2), background=NO_RETURN)
        scan.ranges[0] = float('nan')
        scan.ranges[1] = 0.001     # below range_min
        node._scan_cb(scan)
        assert _published(node.clearance_pub) == [pytest.approx(1.2, abs=1e-4)]

    def test_nothing_in_range_is_infinite_clearance(self, make_node):
        node = make_node()
        node._tf_static_cb(_tf_static())
        node._scan_cb(_scan([], background=NO_RETURN))
        assert _published(node.clearance_pub) == [math.inf]

    def test_without_the_laser_transform_the_scan_is_skipped(self, make_node):
        node = make_node()
        node._scan_cb(_scan(_wall_ahead(1.0)))
        node.clearance_pub.publish.assert_not_called()


class TestContact:

    def test_one_event_per_contact_episode(self, make_node):
        node = make_node()
        node._tf_static_cb(_tf_static())
        # approach, touch, stay in contact, back off inside the band, touch again
        for gap in (0.5, 0.1, -0.01, -0.02, 0.03, -0.01):
            node._scan_cb(_scan(_wall_ahead(gap)))
        events = [json.loads(m) for m in _published(node.safety_pub)]
        assert len(events) == 1
        assert events[0]['event'] == 'contact'
        assert events[0]['clearance_m'] == pytest.approx(-0.01, abs=1e-3)
        assert events[0]['source'] == 'obstacle_clearance_node'
        assert 'inside the footprint' in events[0]['cause']
        # leave the band, then touch again: a second episode
        for gap in (0.2, -0.005):
            node._scan_cb(_scan(_wall_ahead(gap)))
        assert len(_published(node.safety_pub)) == 2

    def test_contact_events_can_be_turned_off(self, make_node):
        node = make_node(publish_contact_events=False)
        node._tf_static_cb(_tf_static())
        node._scan_cb(_scan(_wall_ahead(-0.05)))
        node.safety_pub.publish.assert_not_called()
        assert _published(node.clearance_pub) == [pytest.approx(-0.05, abs=1e-4)]

    def test_the_edge_itself(self):
        edge = ContactEdge(threshold=0.0, rearm=0.05)
        fired = [edge.update(c) for c in (1.0, 0.0, -0.1, 0.04, -0.1, 0.06, -0.2, math.inf, 0.0)]
        assert fired == [False, True, False, False, False, False, True, False, True]

    def test_a_negative_rearm_is_refused(self):
        with pytest.raises(ValueError):
            ContactEdge(threshold=0.0, rearm=-0.01)


class TestDecouplingAndDefaults:

    def test_the_scan_subscription_is_best_effort(self, make_node):
        assert make_node().scan_sub.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT

    def test_the_default_footprint_is_the_swept_clearance_body(self, make_node):
        node = make_node()
        assert node.get_parameter('footprint_length_m').value == pytest.approx(
            BODY_FRONT_X_M - BODY_REAR_X_M)
        assert node.get_parameter('footprint_width_m').value == pytest.approx(
            2.0 * BODY_HALF_WIDTH_M)
        assert node.get_parameter('footprint_rear_x_m').value == pytest.approx(BODY_REAR_X_M)

    def test_the_module_body_agrees_with_stack_params(self):
        # the launch file reads stack_params, the node defaults read swept_corridor
        assert get_value('swept_clearance_body_front_x_m') == pytest.approx(BODY_FRONT_X_M)
        assert get_value('swept_clearance_body_rear_x_m') == pytest.approx(BODY_REAR_X_M)
        assert get_value('swept_clearance_body_half_width_m') == pytest.approx(BODY_HALF_WIDTH_M)

    def test_a_degenerate_footprint_is_refused_at_startup(self, make_node):
        with pytest.raises(ValueError):
            make_node(footprint_width_m=0.0)
