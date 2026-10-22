"""Isolation for the tests that start real rclpy nodes."""

import os

import pytest


@pytest.fixture
def ros_context():
    """
    Yield an rclpy context that can only see this host, on its own domain.

    The car's environment points every process at the stack's discovery
    server (ROS_DISCOVERY_SERVER=127.0.0.1:11811): a test node started there
    would join the live graph. Clear it, and pick a domain nothing else uses.
    """
    rclpy = pytest.importorskip('rclpy')
    saved = dict(os.environ)
    for k in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'ROS_STATIC_PEERS',
              'FASTRTPS_DEFAULT_PROFILES_FILE'):
        os.environ.pop(k, None)
    os.environ['ROS_DOMAIN_ID'] = '147'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'                     # Humble
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'  # Jazzy
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    try:
        yield ctx
    finally:
        rclpy.shutdown(context=ctx)
        os.environ.clear()
        os.environ.update(saved)
