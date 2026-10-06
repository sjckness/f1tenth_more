"""clock_throttle: sim-time decimation of Gazebo's clock, and the wiring.

Pure logic and source checks: needs no ROS and no built workspace.
Background: output/sim_clock_fanout.md.
"""
from pathlib import Path
import struct

from f1tenth_sim.clock_throttle import clock_ns_from_cdr, ClockDecimator
import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
MS = 1_000_000


def _forwarded(rate_hz, stamps_ns):
    d = ClockDecimator(rate_hz)
    return [t for t in stamps_ns if d.accept(t)]


def test_1ms_steps_at_200hz_forward_every_5ms_on_the_grid():
    out = _forwarded(200.0, [i * MS for i in range(10_000)])      # 10 s of sim
    assert len(out) == 2000                                       # 200 Hz
    assert all(t % (5 * MS) == 0 for t in out)                    # exact grid
    assert all(b - a == 5 * MS for a, b in zip(out, out[1:]))     # no jitter


def test_a_50hz_timer_tick_is_always_hit_exactly():
    out = set(_forwarded(200.0, [i * MS for i in range(1, 1001)]))
    assert all(k * 20 * MS in out for k in range(1, 51))


def test_4ms_steps_at_250hz_forward_every_step_on_the_grid():
    # The current setting (4 ms physics, 250 Hz): the bin IS the step, so every
    # step is forwarded exactly on a 4 ms grid, no jitter. 200 Hz (5 ms bins)
    # against 4 ms steps would alias, hence 250.
    out = _forwarded(250.0, [i * 4 * MS for i in range(2500)])     # 10 s of sim
    assert len(out) == 2500                                        # 250 Hz
    assert all(t % (4 * MS) == 0 for t in out)                     # exact grid
    assert all(b - a == 4 * MS for a, b in zip(out, out[1:]))      # no jitter


def test_coarser_steps_still_average_the_target_rate():
    # 4 ms physics at 200 Hz: 4 of every 5 bins get a step (the aliasing we avoid)
    out = _forwarded(200.0, [i * 4 * MS for i in range(2500)])     # 10 s
    assert len(out) == 2000
    # 4 ms steps faster than the target: every step goes through
    assert len(_forwarded(300.0, [i * 4 * MS for i in range(250)])) == 250


def test_paused_clock_forwards_nothing_new():
    d = ClockDecimator(200.0)
    assert d.accept(1000 * MS)
    assert not any(d.accept(1000 * MS) for _ in range(100))   # same value repeated
    assert d.accept(1005 * MS)


def test_time_going_backwards_is_forwarded_and_reanchors():
    d = ClockDecimator(200.0)
    for i in range(1, 101):
        d.accept(i * MS)
    assert d.accept(1 * MS)            # world reset
    assert not d.accept(2 * MS)        # same 5 ms bin as the reset value
    assert d.accept(5 * MS)


def test_rate_zero_is_passthrough():
    stamps = [i * MS for i in range(1, 101)] + [50 * MS, 50 * MS]
    assert _forwarded(0.0, stamps) == stamps


@pytest.mark.parametrize('sec,nsec', [(0, 0), (12, 345_678_901), (2**31 - 1, 999_999_999)])
def test_cdr_little_and_big_endian(sec, nsec):
    le = b'\x00\x01\x00\x00' + struct.pack('<iI', sec, nsec)
    be = b'\x00\x00\x00\x00' + struct.pack('>iI', sec, nsec)
    assert clock_ns_from_cdr(le) == clock_ns_from_cdr(be) == sec * 10**9 + nsec


def test_cdr_too_short_is_rejected():
    with pytest.raises(ValueError):
        clock_ns_from_cdr(b'\x00\x01\x00\x00\x01\x00')


def test_gazebo_clock_is_not_bridged_straight_onto_clock():
    with open(PKG / 'config' / 'ros_gz_bridge.yaml') as f:
        entries = yaml.safe_load(f)
    clock = [e for e in entries if e['gz_topic_name'] == '/clock']
    assert len(clock) == 1
    assert clock[0]['ros_topic_name'] == '/sim/clock_raw'
    assert all(e['ros_topic_name'] != '/clock' for e in entries)
    with open(PKG / 'config' / 'ros_gz_bridge_camera.yaml') as f:
        assert all(e['ros_topic_name'] != '/clock' for e in yaml.safe_load(f))


def test_launch_starts_the_throttle_between_raw_and_clock():
    text = (PKG / 'launch' / 'sim_bringup.launch.py').read_text()
    assert "executable='clock_throttle'" in text
    assert "'input_topic': '/sim/clock_raw'" in text
    assert "'output_topic': '/clock'" in text
    assert "'clock_rate', default_value='250.0'" in text
    assert "'clock_throttle = f1tenth_sim.clock_throttle:main'" in (PKG / 'setup.py').read_text()
