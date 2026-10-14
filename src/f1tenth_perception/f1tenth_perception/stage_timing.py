"""Per-stage latency timing for the camera pipeline (stage_timing param, default off).

Pure (no rclpy): yolo_detector_node and detection_3d_node feed it samples in
milliseconds and it hands back one report per period, which the node logs as
a single line:

    STAGE_TIMING {"node": ..., "period_s": ..., "stages": {name: {p50, p95, max, n}},
                  "counts": {name: n}}

A stage is any named series. The nodes record two kinds and name them so the
log says which is which: `age.<stage>` is now minus the frame's header.stamp
when the stage ended (so the ages of consecutive stages rise), and
`dur.<stage>` is how long the stage itself took. Counts are plain event
counters (frames in, frames processed, frames skipped).

Temporary measurement code for the 2026-10-14 detection-latency audit. With
the param off nothing here is constructed and the nodes take no timestamps.
"""

import json

import numpy as np


class StageTimer:
    """Collects samples per stage and reports p50/p95/max every period_s."""

    def __init__(self, node_name, period_s=30.0):
        self.node_name = str(node_name)
        self.period_s = float(period_s)
        self._samples = {}
        self._counts = {}
        self._start = None

    def add(self, stage, value_ms):
        """Record one sample [ms] for stage; non-finite values are ignored."""
        value = float(value_ms)
        if np.isfinite(value):
            self._samples.setdefault(stage, []).append(value)

    def count(self, name, n=1):
        """Add n to the counter name."""
        self._counts[name] = self._counts.get(name, 0) + int(n)

    def report(self, now_s):
        """Return the report dict once a full period has passed since the first call, else None.

        Resets the samples and counters after reporting. The first call only
        starts the period.
        """
        if self._start is None:
            self._start = float(now_s)
            return None
        elapsed = float(now_s) - self._start
        if elapsed < self.period_s:
            return None
        stages = {}
        for stage, values in self._samples.items():
            arr = np.asarray(values, dtype=float)
            stages[stage] = {
                'p50': round(float(np.percentile(arr, 50)), 2),
                'p95': round(float(np.percentile(arr, 95)), 2),
                'max': round(float(arr.max()), 2),
                'n': int(arr.size),
            }
        out = {'node': self.node_name, 'period_s': round(elapsed, 2),
               'stages': stages, 'counts': dict(self._counts)}
        self._samples = {}
        self._counts = {}
        self._start = float(now_s)
        return out

    @staticmethod
    def format(report):
        """Return the single log line for a report."""
        return 'STAGE_TIMING ' + json.dumps(report, sort_keys=True)
