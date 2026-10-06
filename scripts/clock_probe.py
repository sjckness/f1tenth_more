#!/usr/bin/env python3
"""Probe /clock delivery: rate, largest gap and sim-time advance per interval.

Acceptance test for the /clock throttle (output/sim_clock_fanout.md): run it on
the Thor for >= 2 minutes with the sim up. Healthy at clock_rate:=200 means
msgs/s ~= 200, max_gap < ~20-35 ms, sim_advance ~= wall interval, backwards=0.

    python3 scripts/clock_probe.py [--period 5] [--topic /clock]

Runs on wall time and subscribes BEST_EFFORT depth 1, like every node's
TimeSource does, so it sees what the stack sees.
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rosgraph_msgs.msg import Clock


class ClockProbe(Node):

    def __init__(self, topic, period):
        super().__init__('clock_probe')
        self.period = period
        self.reset(time.monotonic())
        self.last_rx = None
        self.last_sim = None
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, topic, self.on_clock, qos)
        self.create_timer(period, self.report)

    def reset(self, now):
        self.t0 = now
        self.n = 0
        self.max_gap = 0.0
        self.backwards = 0
        self.sim0 = None
        self.sim1 = None

    def on_clock(self, msg):
        now = time.monotonic()
        sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.last_rx is not None:
            self.max_gap = max(self.max_gap, now - self.last_rx)
        if self.last_sim is not None and sim < self.last_sim:
            self.backwards += 1
        if self.sim0 is None:
            self.sim0 = self.last_sim if self.last_sim is not None else sim
        self.sim1 = sim
        self.last_rx, self.last_sim = now, sim
        self.n += 1

    def report(self):
        now = time.monotonic()
        wall = now - self.t0
        adv = (self.sim1 - self.sim0) if self.n else 0.0
        gap = f'{self.max_gap * 1e3:6.0f} ms' if self.n > 1 else '     -   '
        print(f'msgs={self.n:<6d} ({self.n / wall:5.0f}/s)  max_gap={gap}  '
              f'sim_advance={adv:6.2f} s in {wall:4.1f} s wall  backwards={self.backwards}',
              flush=True)
        self.reset(now)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--topic', default='/clock')
    ap.add_argument('--period', type=float, default=5.0)
    a = ap.parse_args()
    rclpy.init()
    node = ClockProbe(a.topic, a.period)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
