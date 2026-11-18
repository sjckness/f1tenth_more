"""
system_observer_node's sysfs thermal fallback survives a None (EAGAIN) read.

On Thor, /sys/devices/virtual/thermal/thermal_zone1/temp (gpu-thermal) fails
with EAGAIN whenever the GPU is idle. The raw read then returns None, and the
text-mode f.read() the node uses turns that into
`TypeError: can't concat NoneType to bytes`. The handler used to catch only
(OSError, ValueError), so the node crashed on its first publish tick (Phase 4
report, Decision 3; output/phase4/probes/thermal_probe_results.txt).

These tests feed the node a real io.TextIOWrapper over a raw stream whose
read returns None -- the same CPython code path as the sysfs file, not a
mocked exception -- and check that the zone is skipped for that tick, the
next matching zone is used, and _publish_tick() still publishes.

Zone layout below is Thor's: tj-thermal, gpu-thermal, cpu-thermal.

Run standalone: python3 -m pytest test/test_system_observer_thermal_read.py -v
"""
import io

import f1tenth_diagnostics.system_observer_node as son
import pytest
import rclpy


class _EagainRaw(io.RawIOBase):
    """Raw stream that behaves like the idle gpu-thermal sysfs file: None."""

    def readable(self):
        return True

    def readinto(self, b):
        return None

    def readall(self):
        return None


_ZONE_ROOT = '/sys/devices/virtual/thermal'


def _install_zones(monkeypatch, zones):
    """Install zones: (type, temp) pairs; temp None is an EAGAIN read."""
    files = {}
    for i, (zone_type, temp) in enumerate(zones):
        files[f'{_ZONE_ROOT}/thermal_zone{i}/type'] = zone_type + '\n'
        files[f'{_ZONE_ROOT}/thermal_zone{i}/temp'] = temp

    def fake_glob(pattern):
        assert pattern == f'{_ZONE_ROOT}/thermal_zone*/type'
        return [p for p in files if p.endswith('/type')]

    def fake_open(path, *args, **kwargs):
        content = files[path]
        if content is None:
            return io.TextIOWrapper(io.BufferedReader(_EagainRaw()))
        return io.StringIO(content)

    monkeypatch.setattr(son.glob, 'glob', fake_glob)
    monkeypatch.setattr(son, 'open', fake_open, raising=False)


def test_eagain_stream_reproduces_the_thor_typeerror():
    # Guards the reproduction itself: if CPython ever stops raising here, the
    # tests below would no longer exercise the crash path.
    with pytest.raises(TypeError):
        io.TextIOWrapper(io.BufferedReader(_EagainRaw())).read()


def test_none_read_on_gpu_zone_is_skipped_and_cpu_zone_used(monkeypatch):
    _install_zones(monkeypatch, [
        ('tj-thermal', '34750\n'),
        ('gpu-thermal', None),
        ('cpu-thermal', '33468\n'),
    ])
    assert son.SystemObserverNode._read_thermal_zone_cpu_temp(None) == pytest.approx(33.468)


def test_none_read_on_cpu_zone_falls_back_to_first_readable_zone(monkeypatch):
    _install_zones(monkeypatch, [
        ('tj-thermal', '34750\n'),
        ('gpu-thermal', None),
        ('cpu-thermal', None),
    ])
    assert son.SystemObserverNode._read_thermal_zone_cpu_temp(None) == pytest.approx(34.75)


def test_every_zone_unreadable_returns_zero(monkeypatch):
    _install_zones(monkeypatch, [('gpu-thermal', None), ('cpu-thermal', None)])
    assert son.SystemObserverNode._read_thermal_zone_cpu_temp(None) == 0.0


def test_empty_read_is_skipped(monkeypatch):
    _install_zones(monkeypatch, [
        ('gpu-thermal', ''),
        ('cpu-thermal', '33468\n'),
    ])
    assert son.SystemObserverNode._read_thermal_zone_cpu_temp(None) == pytest.approx(33.468)


def test_publish_tick_survives_none_read_and_publishes(monkeypatch):
    _install_zones(monkeypatch, [
        ('tj-thermal', '34750\n'),
        ('gpu-thermal', None),
        ('cpu-thermal', '33468\n'),
    ])
    # Force the no-jtop fallback, as on Thor (and in a container without jtop).
    monkeypatch.setattr(son, 'jtop', None)
    rclpy.init()
    try:
        node = son.SystemObserverNode()
        published = []
        monkeypatch.setattr(node._pub, 'publish', published.append)
        node._publish_tick()
        node._publish_tick()
        node.destroy_node()
    finally:
        rclpy.shutdown()
    assert len(published) == 2
    for msg in published:
        assert msg.cpu_temp_c == pytest.approx(33.468)
        # Without jtop the GPU temperature is not read at all: 0.0, the
        # documented "not available" value IsSystemOverheated treats as no trip.
        assert msg.gpu_temp_c == 0.0
