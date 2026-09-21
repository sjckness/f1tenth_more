"""pytest wiring for the test-campaign suite (f1tenth_logger.test_campaign).

    colcon test --packages-select f1tenth_logger
    python3 -m pytest src/f1tenth_logger/test/test_campaign -q    # sourced

**ROS isolation.** The node round trip publishes real DDS traffic, so the
whole pytest process is moved onto its own domain with localhost-only
discovery and no discovery server, before rclpy is imported anywhere. It
cannot reach the car's stack and the car's stack cannot reach it. Override
the domain with ``F1TENTH_TEST_DOMAIN`` if 91 is taken.
"""

import os
import sys
from pathlib import Path

import pytest

os.environ["ROS_DOMAIN_ID"] = os.environ.get("F1TENTH_TEST_DOMAIN", "91")
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ.pop("ROS_DISCOVERY_SERVER", None)
os.environ.pop("ROS_SUPER_CLIENT", None)

sys.path.insert(0, str(Path(__file__).resolve().parent))

try:
    import rclpy  # noqa: F401
    HAVE_RCLPY = True
except ImportError:  # pragma: no cover - a machine without ROS
    HAVE_RCLPY = False

requires_rclpy = pytest.mark.skipif(
    not HAVE_RCLPY, reason="rclpy is not importable: source the ROS workspace"
)
