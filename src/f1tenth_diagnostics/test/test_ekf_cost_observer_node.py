"""
Unit tests for ekf_cost_observer_node's measurement arithmetic.

These cover the parts that decide whether the reported cost is CORRECT, not
whether the node starts: the PID discriminator (both EKF instances are the same
executable, so getting this wrong silently attributes one filter's CPU to the
other), the cost divisions, and the two-source tick reconciliation that exists
because a single self-counted rate has already produced one wrong answer in this
workspace.

No live ROS graph: _FilterProbe's arithmetic never touches its owner node, so
the tests drive it directly with a stub.
"""

import pytest

from f1tenth_diagnostics.ekf_cost_observer_node import (
    _FilterProbe,
    find_ekf_pid,
    summarize_periods,
)


class _StubProc:
    """Stands in for psutil.Process, returning scripted cumulative CPU times."""

    def __init__(self, cpu_seconds):
        self._cpu_seconds = list(cpu_seconds)
        self._calls = 0

    def is_running(self):
        return True

    def cpu_times(self):
        value = self._cpu_seconds[min(self._calls, len(self._cpu_seconds) - 1)]
        self._calls += 1
        return type('T', (), {'user': value, 'system': 0.0})()


def _probe(cpu_seconds=(0.0, 0.0)):
    probe = _FilterProbe(
        owner=None, label='global', node_name='ekf_global_filter_node',
        output_topic='/ekf_global/odometry/filtered', input_topics=[])
    probe.pid = 4242
    probe._proc = _StubProc(cpu_seconds)
    return probe


def test_find_ekf_pid_tells_the_two_identical_ekf_node_processes_apart(monkeypatch):
    """Only the `-r __node:=` remap distinguishes local from global."""
    class _P:
        def __init__(self, pid, cmdline):
            self.info = {'pid': pid, 'cmdline': cmdline}

    procs = [
        _P(100, ['/opt/ros/humble/lib/robot_localization/ekf_node',
                 '--ros-args', '-r', '__node:=ekf_filter_node']),
        _P(200, ['/opt/ros/humble/lib/robot_localization/ekf_node',
                 '--ros-args', '-r', '__node:=ekf_global_filter_node',
                 '-r', '__ns:=/ekf_global']),
    ]
    monkeypatch.setattr(
        'f1tenth_diagnostics.ekf_cost_observer_node.psutil.process_iter',
        lambda attrs=None: procs)

    assert find_ekf_pid('ekf_filter_node') == 100
    assert find_ekf_pid('ekf_global_filter_node') == 200
    assert find_ekf_pid('ekf_nonexistent_node') is None


def test_find_ekf_pid_does_not_match_global_when_asked_for_local(monkeypatch):
    """
    `__node:=ekf_filter_node` must not loosely match the global name.

    The guard is the full-token comparison including the `__node:=` prefix. A
    looser containment test would make the two instances indistinguishable, and
    the failure mode would be silent CPU misattribution between the two filters
    rather than an error anyone notices.
    """
    class _P:
        def __init__(self, pid, cmdline):
            self.info = {'pid': pid, 'cmdline': cmdline}

    monkeypatch.setattr(
        'f1tenth_diagnostics.ekf_cost_observer_node.psutil.process_iter',
        lambda attrs=None: [
            _P(200, ['ekf_node', '-r', '__node:=ekf_global_filter_node'])])

    assert find_ekf_pid('ekf_filter_node') is None


def test_cpu_per_tick_divides_by_the_in_process_count_not_the_self_count():
    """ticks_inproc is authoritative: our own subscriber can undercount."""
    probe = _probe(cpu_seconds=(1.0, 1.55))
    probe.sample()                      # priming call, establishes the baseline

    probe.ticks_selfcount = 10          # a lossy subscriber
    probe.note_inproc_events(20)        # the filter's own count
    probe.meas_count = 40

    metrics = probe.sample()

    # 0.55s of CPU over 20 ticks = 27.5ms/tick, the figure this metric exists
    # to keep visible once the 20Hz budget stops flagging it.
    assert metrics['cpu_ms_per_tick'] == pytest.approx(27.5)
    assert metrics['cpu_ms_per_meas'] == pytest.approx(13.75)
    assert metrics['meas_per_tick'] == pytest.approx(2.0)


def test_falls_back_to_self_count_when_print_diagnostics_is_off():
    """Without the filter's FrequencyStatus there is only one count left."""
    probe = _probe(cpu_seconds=(0.0, 0.4))
    probe.sample()

    probe.ticks_selfcount = 20          # no note_inproc_events call at all
    probe.meas_count = 20

    metrics = probe.sample()

    assert metrics['ticks_inproc'] == 0.0
    assert metrics['cpu_ms_per_tick'] == pytest.approx(20.0)
    # No second source to cross-check against, and the field says so rather
    # than reporting a fabricated 1.0 agreement.
    assert metrics['tick_count_agreement'] == 0.0


def test_tick_count_agreement_exposes_a_lying_subscriber():
    """The 8.4Hz-vs-29.7Hz artifact would show here as agreement far from 1."""
    probe = _probe(cpu_seconds=(0.0, 0.5))
    probe.sample()

    probe.ticks_selfcount = 8           # what a starved subscriber saw
    probe.note_inproc_events(30)        # what the filter actually did

    metrics = probe.sample()

    assert metrics['tick_count_agreement'] == pytest.approx(30 / 8)
    assert metrics['tick_count_agreement'] > 2.0


def test_sample_resets_counters_so_each_window_is_independent():
    """A leaked counter would make cost drift downward over a long run."""
    probe = _probe(cpu_seconds=(0.0, 0.1, 0.2))
    probe.sample()

    probe.ticks_selfcount = 5
    probe.note_inproc_events(5)
    probe.meas_count = 5
    probe.stamps = [1.0, 1.02]
    probe.sample()

    metrics = probe.sample()
    assert metrics['ticks_inproc'] == 0.0
    assert metrics['ticks_selfcount'] == 0.0
    assert metrics['meas_delivered'] == 0.0
    assert metrics['period_ms_p50'] == 0.0


def test_first_sample_reports_zero_cost_rather_than_a_whole_process_lifetime():
    """cpu_times() is cumulative since exec, not since the last window."""
    probe = _probe(cpu_seconds=(3600.0, 3600.02))
    probe.ticks_selfcount = 50

    first = probe.sample()
    assert first['cpu_ms_per_tick'] == 0.0

    probe.ticks_selfcount = 20
    second = probe.sample()
    assert second['cpu_ms_per_tick'] == pytest.approx(1.0)


def test_summarize_periods_uses_stamp_deltas_and_reports_the_tail():
    """p90/max are the wall-clock companion to the CPU-time cost figure."""
    stamps = [0.0, 0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.14, 0.16, 0.30]
    stats = summarize_periods(stamps)

    assert stats['period_ms_p50'] == pytest.approx(20.0)
    assert stats['period_ms_max'] == pytest.approx(140.0)


def test_summarize_periods_survives_a_window_with_nothing_in_it():
    """A filter that published nothing must not take the observer down."""
    assert summarize_periods([])['period_ms_p50'] == 0.0
    assert summarize_periods([1.0])['period_ms_max'] == 0.0
