"""C3: offline replay must reproduce the live command sequence exactly."""

from math import cos, hypot, sin

import pytest

from go_to_object.mission_state import MissionState
from go_to_object.object_tracker import TrackerParams
from go_to_object.pursuit_geometry import wrap_pi
from go_to_object.replay import (
    DetectionEvent,
    OdomEvent,
    ReplayConfig,
    TickEvent,
    compare,
    replay,
)


def _events(*, obj=(18.5, 4.0), speed=1.5, dt=0.01, ticks=900, latency=0.05):
    """A plausible event stream in the order a node would have seen it."""
    events = []
    x = y = psi = 0.0
    pending = []
    for i in range(ticks):
        t = i * dt
        events.append(OdomEvent(t, x, y, psi))
        while pending and pending[0][0] <= t:
            _, capture, bx, by = pending.pop(0)
            events.append(DetectionEvent(capture, bx, by))
        if i % 10 == 0:
            c, s = cos(psi), sin(psi)
            rx, ry = obj[0] - x, obj[1] - y
            pending.append((t + latency, t, c * rx + s * ry, -s * rx + c * ry))
        events.append(TickEvent(t, last_odom_stamp=t))
        # Open loop: the vehicle path only has to be plausible, since replay
        # is judged against replay, not against a particular trajectory.
        x += speed * dt * cos(psi)
        y += speed * dt * sin(psi)
        psi = wrap_pi(psi + 0.02 * speed * dt)
    return events


def test_replay_is_deterministic():
    events = _events()

    assert replay(events) == replay(events)


def test_replay_reproduces_itself_bit_for_bit_through_compare():
    events = _events()
    first = replay(events)
    recorded = [dict(state=c.state, curvature=c.curvature,
                     curvature_raw=c.curvature_raw,
                     drive_enable=c.drive_enable) for c in first]

    assert compare(replay(events), recorded, tolerance=0.0) == []


def test_compare_reports_a_divergence_rather_than_absorbing_it():
    events = _events()
    recorded = [dict(state=c.state, curvature=c.curvature,
                     curvature_raw=c.curvature_raw,
                     drive_enable=c.drive_enable) for c in replay(events)]
    recorded[400]['curvature'] += 1e-9

    problems = compare(replay(events), recorded, tolerance=0.0)

    assert len(problems) == 1
    assert 'tick 400' in problems[0] and 'curvature' in problems[0]


def test_compare_notices_a_truncated_recording():
    events = _events()
    replayed = replay(events)
    recorded = [dict(state=c.state) for c in replayed[:-5]]

    problems = compare(replayed, recorded)

    assert any('tick count differs' in p for p in problems)


def test_event_order_matters_not_just_event_stamps():
    """Sorting by message stamp is not the same as arrival order.

    A detection is processed when it *arrives*, which is deliberately later
    than the moment it describes. A replay that re-sorted by stamp would fuse
    detections before the odometry they need, so this must not be equivalent.
    """
    events = _events()
    by_stamp = sorted(events, key=lambda e: e.stamp)

    assert replay(events) != replay(by_stamp)


def test_replay_actually_drives_the_mission_forward():
    """Guards against the comparison tests passing on an inert sequence."""
    commands = replay(_events())
    states = {c.state for c in commands}

    assert MissionState.APPROACH.value in states
    assert any(abs(c.curvature) > 0.0 for c in commands)


def test_shadow_mode_zeroes_the_command_and_changes_nothing_else():
    """C1: everything is computed; only the published number is suppressed."""
    events = _events()
    live = replay(events, ReplayConfig(enabled=True))
    shadow = replay(events, ReplayConfig(enabled=False))

    assert all(c.curvature == 0.0 for c in shadow), 'nothing reaches the servo'
    assert any(c.curvature != 0.0 for c in live), 'the live case must differ'
    assert [c.state for c in shadow] == [c.state for c in live]
    assert [c.curvature_raw for c in shadow] == [c.curvature_raw for c in live]


def test_replay_honours_a_non_default_configuration():
    events = _events()
    slow = replay(events, ReplayConfig(max_kappa_rate=0.02))
    fast = replay(events, ReplayConfig(max_kappa_rate=5.0))

    slow_steps = max(abs(b.curvature - a.curvature)
                     for a, b in zip(slow, slow[1:]))
    fast_steps = max(abs(b.curvature - a.curvature)
                     for a, b in zip(fast, fast[1:]))

    assert slow_steps <= 0.02 * 0.01 + 1e-12
    assert fast_steps > slow_steps


def test_an_unknown_event_type_is_refused_not_ignored():
    with pytest.raises(TypeError):
        replay([OdomEvent(0.0, 0.0, 0.0, 0.0), object()])


# -- Change 2: a gapped recording must fail loudly, not compare successfully --

from go_to_object.replay import (  # noqa: E402
    DEFAULT_CONTROL_PERIOD,
    ReplayGapError,
    check_tick_continuity,
)


def _clean(n=200, period=DEFAULT_CONTROL_PERIOD, t0=1789459839.0):
    return [t0 + i * period for i in range(n)]


def test_a_continuous_tick_sequence_passes():
    check_tick_continuity(_clean(), DEFAULT_CONTROL_PERIOD)


@pytest.mark.parametrize('jitter', [0.0, 0.2, -0.3, 0.49])
def test_ordinary_scheduling_jitter_is_tolerated(jitter):
    period = DEFAULT_CONTROL_PERIOD
    stamps = _clean()
    stamps = [t + (jitter * period if i % 2 else 0.0)
              for i, t in enumerate(stamps)]

    check_tick_continuity(stamps, period)


@pytest.mark.parametrize('stamps', [[], [1.0]])
def test_a_sequence_too_short_to_have_intervals_is_not_an_error(stamps):
    check_tick_continuity(stamps, DEFAULT_CONTROL_PERIOD)


def test_a_dropped_tick_fails_and_names_the_right_interval():
    stamps = _clean(n=50)
    dropped_at = 17
    del stamps[dropped_at]

    with pytest.raises(ReplayGapError) as excinfo:
        check_tick_continuity(stamps, DEFAULT_CONTROL_PERIOD)

    message = str(excinfo.value)
    assert f'tick {dropped_at - 1}' in message, 'names the tick before the gap'
    assert repr(stamps[dropped_at - 1]) in message, 'and its exact timestamp'
    assert repr(stamps[dropped_at]) in message, 'and where it resumes'
    assert '1.0 tick(s) missing' in message


def test_a_decimated_recording_fails_at_the_first_gap_not_a_later_one():
    """Throttled diagnostics: every 5th tick survives. Must name the first."""
    stamps = _clean(n=100)[::5]

    with pytest.raises(ReplayGapError) as excinfo:
        check_tick_continuity(stamps, DEFAULT_CONTROL_PERIOD)

    message = str(excinfo.value)
    assert 'tick 0' in message and 'tick 1' in message
    assert repr(stamps[0]) in message
    assert '4.0 tick(s) missing' in message


def test_a_burst_of_ticks_faster_than_the_period_also_fails():
    """Duplicated or re-published messages are as damaging as dropped ones."""
    stamps = _clean(n=30)
    stamps.insert(10, stamps[9] + DEFAULT_CONTROL_PERIOD * 0.1)

    with pytest.raises(ReplayGapError, match='tick 9'):
        check_tick_continuity(stamps, DEFAULT_CONTROL_PERIOD)


@pytest.mark.parametrize('offset', [0.0, -0.01])
def test_a_non_advancing_tick_is_reported_as_such(offset):
    stamps = _clean(n=10)
    stamps[5] = stamps[4] + offset

    with pytest.raises(ReplayGapError, match='does not advance'):
        check_tick_continuity(stamps, DEFAULT_CONTROL_PERIOD)


def test_the_check_respects_a_non_default_control_period():
    fifty_hz = _clean(n=40, period=0.02)

    check_tick_continuity(fifty_hz, 0.02)
    with pytest.raises(ReplayGapError):
        check_tick_continuity(fifty_hz, DEFAULT_CONTROL_PERIOD)


def test_an_invalid_control_period_is_refused():
    with pytest.raises(ValueError):
        check_tick_continuity(_clean(), 0.0)


def test_the_gap_message_says_why_it_matters():
    stamps = _clean(n=20)
    del stamps[5]

    with pytest.raises(ReplayGapError) as excinfo:
        check_tick_continuity(stamps, DEFAULT_CONTROL_PERIOD)

    assert 'meaningless' in str(excinfo.value)
    assert 'Re-record' in str(excinfo.value)
