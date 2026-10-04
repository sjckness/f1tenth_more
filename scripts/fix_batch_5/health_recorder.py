#!/usr/bin/env python3
"""Fix batch 5: record every /supervisor/health status change, until killed.

One JSON line per change of a component's (level, status), plus one
'heartbeat' line per 10 s with the full status set, so a run's timeline can
be read back: when did swept_clearance go FAILING, RESTARTING, STARTING, OK.

  health_recorder.py --out FILE.jsonl
"""
import argparse
import json
import signal
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('health_recorder')
    last = {}
    last_beat = [0.0]
    f = open(a.out, 'a', buffering=1)

    def cb(msg):
        now = time.time()
        full = {}
        for st in msg.status:
            name = st.name.rsplit('/', 1)[-1]
            level = st.level[0] if isinstance(st.level, (bytes, bytearray)) else int(st.level)
            values = {kv.key: kv.value for kv in st.values}
            full[name] = [level, st.message, values]
            key = (level, st.message.split(':', 1)[0])   # status, not the ages it quotes
            if last.get(name) != key:
                last[name] = key
                f.write(json.dumps({'t': round(now, 3), 'component': name, 'level': level,
                                    'message': st.message, 'values': values}) + '\n')
        if now - last_beat[0] >= 10.0:
            last_beat[0] = now
            f.write(json.dumps({'t': round(now, 3), 'heartbeat': full}) + '\n')

    qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(DiagnosticArray, '/supervisor/health', cb, qos)
    stop = [False]
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    while not stop[0]:
        rclpy.spin_once(node, timeout_sec=0.2)
    f.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
