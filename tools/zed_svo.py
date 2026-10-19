#!/usr/bin/env python3
"""Start or stop the ZED wrapper's SVO recording, and check the inputs of tools/record_raw.sh.

    zed_svo.py start /abs/path/clip.svo2   prints the compression mode used, exit 1 on failure
    zed_svo.py stop
    zed_svo.py check TOPIC [TOPIC ...]     exit 1 when a topic sends nothing within 5 s

Services (zed_components' zed_camera_component, srv_prefix '~/'):
  /zed2/zed_node/start_svo_rec  zed_msgs/srv/StartSvoRec
      bitrate (0 or 1000-60000 kbit/s), compression_mode (sl::SVO_COMPRESSION_MODE,
      checked only against LAST despite the "[0,2]" in its message), target_framerate,
      input_transcode, svo_filename
  /zed2/zed_node/stop_svo_rec   std_srvs/srv/Trigger

H265_LOSSLESS (4) is tried first. If the wrapper refuses it, H265 (2) at the
60000 kbit/s maximum is used. SDK 4.2 writes .svo2.
"""
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from std_srvs.srv import Trigger
from zed_msgs.srv import StartSvoRec

NODE = '/zed2/zed_node'
# /tf_static is latched: its publishers send once, TRANSIENT_LOCAL. A volatile
# best-effort reader joining later never sees a message.
LATCHED = {'/tf_static': QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)}
MODES = [('H265_LOSSLESS', 4, 0), ('H265', 2, 60000)]


def _call(node, client, request, timeout=15.0):
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.result()


def start(node, path):
    client = node.create_client(StartSvoRec, f'{NODE}/start_svo_rec')
    for name, mode, bitrate in MODES:
        req = StartSvoRec.Request(bitrate=bitrate, compression_mode=mode, target_framerate=0,
                                  input_transcode=False, svo_filename=path)
        res = _call(node, client, req)
        if res is None:
            print(f'{NODE}/start_svo_rec not available', file=sys.stderr)
            return 1
        if res.success:
            print(f'{name} bitrate={bitrate or "default"}')
            return 0
        print(f'{name} refused: {res.message}', file=sys.stderr)
    return 1


def stop(node):
    res = _call(node, node.create_client(Trigger, f'{NODE}/stop_svo_rec'), Trigger.Request())
    if res is None or not res.success:
        print(f'stop_svo_rec failed: {None if res is None else res.message}', file=sys.stderr)
        return 1
    return 0


def check(node, topics):
    time.sleep(3.0)   # discovery
    seen, subs = set(), []
    for topic in topics:
        types = dict(node.get_topic_names_and_types()).get(topic)
        if not types:
            continue
        subs.append(node.create_subscription(
            get_message(types[0]), topic, lambda _m, t=topic: seen.add(t),
            LATCHED.get(topic, qos_profile_sensor_data)))
    end = time.time() + 5.0
    while time.time() < end and len(seen) < len(topics):
        rclpy.spin_once(node, timeout_sec=0.1)
    missing = [t for t in topics if t not in seen]
    for t in missing:
        print(f'NO DATA: {t}', file=sys.stderr)
    return 1 if missing else 0


def main():
    rclpy.init()
    node = rclpy.create_node('zed_svo_ctl')
    try:
        cmd = sys.argv[1]
        if cmd == 'start':
            return start(node, sys.argv[2])
        if cmd == 'stop':
            return stop(node)
        return check(node, sys.argv[2:])
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
