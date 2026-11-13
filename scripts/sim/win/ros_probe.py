#!/usr/bin/env python3
"""Look at the ROS 2 graph with the bridge's own rclpy (no host ROS on Windows).

    ros_probe.py topics [SECONDS]        list topics and types seen on the domain
    ros_probe.py hz TOPIC [SECONDS]      message count and rate (best-effort QoS)
    ros_probe.py reset                   call /sim/reset (std_srvs/Trigger)

Started by hil_win.py with the environment of a run (domain, Fast DDS profile, bridge DLLs on
PATH), so it sees the sim exactly as a node on the same domain would.
"""
import os
import sys
import time


def _bridge_rclpy() -> None:
    bd = os.environ.get("ISAAC_ROS_BRIDGE_DIR", "")
    path = os.path.join(bd, os.environ.get("ROS_DISTRO", "jazzy"), "rclpy")
    if os.path.isdir(path) and path not in sys.path:
        sys.path.append(path)
    lib = os.path.join(bd, os.environ.get("ROS_DISTRO", "jazzy"), "lib")
    if hasattr(os, "add_dll_directory") and os.path.isdir(lib):
        os.add_dll_directory(lib)


_bridge_rclpy()
import rclpy  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402


def _spin(node, seconds: float) -> None:
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)


def main() -> int:
    cmd = sys.argv[1] if len(sys.argv) > 1 else "topics"
    rclpy.init()
    node = rclpy.create_node("hil_win_probe")
    try:
        if cmd == "topics":
            _spin(node, float(sys.argv[2]) if len(sys.argv) > 2 else 6.0)
            for name, types in sorted(node.get_topic_names_and_types()):
                print(f"{name}  [{', '.join(types)}]")
            return 0
        if cmd == "hz":
            topic = sys.argv[2]
            secs = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
            _spin(node, 3.0)
            types = dict(node.get_topic_names_and_types()).get(topic)
            if not types:
                print(f"topic {topic} not found (is the sim running on domain {os.environ.get('ROS_DOMAIN_ID')}?)")
                return 1
            from rosidl_runtime_py.utilities import get_message
            count = [0]
            node.create_subscription(get_message(types[0]), topic, lambda _m: count.__setitem__(0, count[0] + 1),
                                     QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            t0 = time.time()
            _spin(node, secs)
            dt = time.time() - t0
            print(f"{topic}: {count[0]} messages in {dt:.1f} s = {count[0] / dt:.2f} Hz")
            return 0 if count[0] else 1
        if cmd == "reset":
            from std_srvs.srv import Trigger
            cli = node.create_client(Trigger, "/sim/reset")
            if not cli.wait_for_service(timeout_sec=10.0):
                print("/sim/reset not available")
                return 1
            fut = cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
            print(fut.result())
            return 0 if fut.result() is not None else 1
        print(f"unknown command {cmd}")
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
