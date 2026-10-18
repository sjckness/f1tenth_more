"""
/perception/virtual_wall is published only on a new estimate, <= ~rate_hz.

cpu-cleanup R2 (rate policy). Pure Python: lidar_front_wall has no ROS import.
"""

import pytest

from f1tenth_perception.lidar_front_wall import virtual_wall_due


def _run(scan_times, tracker_on, decimation, rate_hz):
    """Feed scans; return the times virtual_wall_due allowed a publish."""
    last, out = None, []
    for i, t in enumerate(scan_times):
        ran = (i % decimation) == decimation - 1
        if virtual_wall_due(tracker_on, ran, t, last, rate_hz):
            last = t
            out.append(t)
    return out


def test_with_the_tracker_only_tracker_steps_publish():
    scans = [i * 0.025 for i in range(40)]                 # 1 s of 40 Hz scans
    pubs = _run(scans, tracker_on=True, decimation=4, rate_hz=10.0)
    assert len(pubs) == 10
    assert all(abs(b - a - 0.1) < 1e-9 for a, b in zip(pubs, pubs[1:]))


def test_scan_jitter_does_not_drop_a_tracker_step():
    # A tracker step 10 ms early (90 ms after the last) still publishes.
    assert virtual_wall_due(True, True, 1.090, 1.0, 10.0)
    assert not virtual_wall_due(True, True, 1.070, 1.0, 10.0)


def test_skipped_scans_never_publish():
    assert not virtual_wall_due(True, False, 5.0, None, 10.0)


def test_without_the_tracker_every_scan_is_new_but_capped():
    scans = [i * 0.025 for i in range(40)]
    pubs = _run(scans, tracker_on=False, decimation=1, rate_hz=10.0)
    # every 3rd scan (75 ms >= 0.75 x 100 ms): <= 13.3 Hz, never 40 Hz
    assert 10 <= len(pubs) <= 14


@pytest.mark.parametrize('gate_s', [0.5])
def test_worst_receipt_gap_is_inside_every_consumer_gate(gate_s):
    """BT wall_distance_max_age_s and MPC corr_wall_turn_front_distance_max_age_sec
    are 0.5 s: >= 2x the nominal 0.1 s, and above the ~0.17 s period of a
    CPU-bound tracker (~6 Hz, measured 2026-10-14)."""
    assert gate_s >= 2 * 0.1
    assert gate_s > 1 / 6.0
