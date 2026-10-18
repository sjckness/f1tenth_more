#!/usr/bin/env python3
"""The Python costmap_boundary_node, unchanged, with its tick timed.

CostmapBoundaryNode's timer calls self._extraction_tick; this subclass wraps
it with perf_counter and logs p50/p95/max every --report-s seconds (the same
statistic the Rust node logs with timing_report_period_s). Measurement only:
the published output is the stock node's.
"""

import sys
import time

import numpy as np
import rclpy

from f1tenth_costmap.costmap_boundary_node import CostmapBoundaryNode


class TimedCostmapBoundaryNode(CostmapBoundaryNode):

    def __init__(self, report_s):
        self._ticks_us = []
        self._report_s = report_s
        self._last_report = time.monotonic()
        super().__init__()

    def _extraction_tick(self):
        t0 = time.perf_counter()
        super()._extraction_tick()
        self._ticks_us.append((time.perf_counter() - t0) * 1e6)
        if time.monotonic() - self._last_report >= self._report_s:
            v = np.array(self._ticks_us)
            self.get_logger().info(
                f'tick timing over {len(v)} ticks: p50 {np.percentile(v, 50):.1f} us, '
                f'p95 {np.percentile(v, 95):.1f} us, max {v.max():.1f} us')
            self._ticks_us.clear()
            self._last_report = time.monotonic()


def main():
    report_s = 1e9
    args = list(sys.argv)
    if '--report-s' in args:
        i = args.index('--report-s')
        report_s = float(args[i + 1])
        del args[i:i + 2]
    rclpy.init(args=args)
    node = TimedCostmapBoundaryNode(report_s)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
