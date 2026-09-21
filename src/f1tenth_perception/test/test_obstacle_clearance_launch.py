"""obstacle_clearance.launch.py -- the LaunchDescription built and inspected
directly, no process spawned (the test_swept_clearance_launch.py convention).

Run standalone: python3 -m pytest test/test_obstacle_clearance_launch.py -v
"""

import importlib.util
import os

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from f1tenth_params.param_defaults import get_value

_PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))


def _launch_description():
    path = os.path.join(_PACKAGE_DIR, 'launch', 'obstacle_clearance.launch.py')
    spec = importlib.util.spec_from_file_location('obstacle_clearance_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _declared(ld):
    return {e.name: e for e in ld.entities if isinstance(e, DeclareLaunchArgument)}


def _default(ld, name):
    return ''.join(s.perform(LaunchContext()) for s in _declared(ld)[name].default_value)


def test_one_node_gated_on_use_lidar():
    ld = _launch_description()
    nodes = [e for e in ld.entities if isinstance(e, Node)]
    assert len(nodes) == 1
    assert nodes[0].condition is not None
    assert 'use_lidar' in _declared(ld)


def test_the_footprint_defaults_to_the_swept_clearance_body_in_stack_params():
    ld = _launch_description()
    front = get_value('swept_clearance_body_front_x_m')
    rear = get_value('swept_clearance_body_rear_x_m')
    half = get_value('swept_clearance_body_half_width_m')
    assert float(_default(ld, 'obstacle_clearance_footprint_length_m')) == pytest.approx(
        front - rear)
    assert float(_default(ld, 'obstacle_clearance_footprint_width_m')) == pytest.approx(2 * half)
    assert float(_default(ld, 'obstacle_clearance_footprint_rear_x_m')) == pytest.approx(rear)


def test_it_publishes_where_the_campaign_logger_listens():
    ld = _launch_description()
    assert _default(ld, 'obstacle_clearance_clearance_topic') == '/obstacle_clearance'
    assert _default(ld, 'obstacle_clearance_scan_topic') == '/scan'
    assert _default(ld, 'obstacle_clearance_publish_contact_events') == 'true'


def test_pinned_off_the_e_stop_core():
    assert _default(_launch_description(), 'obstacle_clearance_cpu_affinity') == '5'
