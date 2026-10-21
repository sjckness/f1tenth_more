"""Manual driving's safety rules (SPEC section 5): mapping, clamps, watchdog, lease."""

import math
import os

from f1tenth_gateway.teleop import (
    HARD_MAX_SPEED_MPS, MEASURED_MAX_AGE_S, MIN_MOVING_SPEED_MPS, speed_for, Teleop,
    TeleopConfig)
import pytest
import yaml

STACK_PARAMS = os.path.join(os.path.dirname(__file__), '..', '..', 'f1tenth_params', 'config',
                            'stack_params.yaml')


def drive(t, client='a', now=0.0, seq=[0], **kw):
    seq[0] += 1
    msg = {'type': 'teleop', 'seq': seq[0], 'thr': 0.0, 'steer': 0.0, 'deadman': True}
    msg.update(kw)
    return t.on_input(client, msg, now)


@pytest.fixture
def tele():
    t = Teleop(TeleopConfig())
    assert t.acquire('a', 0.0) == (True, 'ok')
    return t


# ---- 5. never below 0.4 m/s forward, dead zone, reverse to 0.4 m/s -------------------------

def test_the_minimum_speed_is_the_stacks_own():
    """A coupling (CLAUDE.md): stack_params min_moving_speed_mps is the same 0.4."""
    with open(STACK_PARAMS) as f:
        stack = yaml.safe_load(f)
    assert stack['min_moving_speed_mps']['default'] == MIN_MOVING_SPEED_MPS == 0.4


def test_the_reverse_cap_is_the_mpcs_own():
    """A coupling: stack_params max_reverse_speed_mps is the gateway's default 0.4."""
    with open(STACK_PARAMS) as f:
        stack = yaml.safe_load(f)
    assert stack['max_reverse_speed_mps']['default'] == \
        TeleopConfig().max_reverse_speed_mps == 0.4


@pytest.mark.parametrize('thr, want', [
    (0.0, 0.0), (0.15, 0.0), (-0.15, 0.0),                     # dead zone, both ways
    (0.1500001, 0.4), (1.0, 0.6),                              # 0.4 .. limit
    (-0.8, -0.4 * 0.65 / 0.85), (-1.0, -0.4),                  # linear to -0.4, no floor
    (-0.1500001, 0.0),
])
def test_the_gas_mapping(thr, want):
    assert speed_for(thr, 0.6, 0.15, 0.4) == pytest.approx(want, abs=1e-5)


def test_reverse_is_capped_by_max_reverse_and_by_the_drivers_limit():
    assert speed_for(-1.0, 1.0, 0.15, 0.4) == pytest.approx(-0.4)     # limit 1.0: the cap
    assert speed_for(-1.0, 0.5, 0.15, 0.8) == pytest.approx(-0.5)     # cap 0.8: the limit
    assert TeleopConfig(max_reverse_speed_mps=5.0).max_reverse_speed_mps == HARD_MAX_SPEED_MPS


def test_max_reverse_0_forbids_reverse_as_in_v1():
    assert speed_for(-1.0, 0.6, 0.15, 0.0) == 0.0
    t = Teleop(TeleopConfig(max_reverse_speed_mps=0.0))
    t.acquire('a', 0.0)
    drive(t, thr=-1.0, now=0.0)
    assert t.tick(0.0).speed == 0.0


def test_forward_never_commands_between_0_and_0_4_and_reverse_stays_within_0_4(tele):
    for thr in [i / 100 for i in range(-100, 101)]:
        v = speed_for(thr, tele.limit, tele.cfg.deadzone, tele.cfg.max_reverse_speed_mps)
        assert v == 0.0 or v >= MIN_MOVING_SPEED_MPS or -0.4 <= v < 0.0


def test_the_first_moving_command_is_0_4_then_it_ramps(tele):
    drive(tele, thr=1.0, limit=1.0, now=0.0)
    first = tele.tick(0.0)
    assert first.speed == pytest.approx(0.4)
    drive(tele, thr=1.0, limit=1.0, now=0.02)
    second = tele.tick(0.02)
    assert second.speed == pytest.approx(0.4 + 1.0 * 0.02)         # accel_mps2 1.0


def test_reverse_ramps_from_0_with_no_floor(tele):
    drive(tele, thr=-1.0, now=0.0)
    assert tele.tick(0.0).speed == 0.0                              # dt 0 on the first tick
    drive(tele, thr=-1.0, now=0.02)
    assert tele.tick(0.02).speed == pytest.approx(-0.02)            # accel_mps2 1.0
    now = 0.02
    for _ in range(50):
        now += 0.02
        drive(tele, thr=-1.0, now=now)
        cmd = tele.tick(now)
    assert cmd.speed == pytest.approx(-0.4)                         # the cap
    drive(tele, thr=-0.5, now=now + 0.02)
    assert tele.tick(now + 0.02).speed == pytest.approx(speed_for(-0.5, 0.6, 0.15, 0.4))


def _run(t, thr, t0, t1, dt=0.025, v_meas=None):
    """
    Drive thr from t0 to t1; return [(now, speed)]. v_meas(now) feeds measured().

    40 inputs a second, under max_input_hz (50): a dropped input would keep
    the previous stick and blur what these tests look at.
    """
    out, now = [], t0
    while now < t1 - 1e-9:
        if v_meas is not None:
            t.measured(v_meas(now), now)
        drive(t, thr=thr, now=now)
        out.append((now, t.tick(now).speed))
        now += dt
    return out


def test_forward_to_reverse_holds_0_for_reverse_hold_s_without_odometry(tele):
    fwd = _run(tele, 1.0, 0.0, 1.0)
    assert fwd[-1][1] > 0.4
    rev = _run(tele, -1.0, 1.0, 2.0)
    zero = [n for n, v in rev if v == 0.0]
    moving = [n for n, v in rev if v != 0.0]
    assert all(v <= 0.0 for _, v in rev)                            # never forward again
    assert zero[0] == pytest.approx(1.0) and moving[0] - zero[0] >= 0.3 - 1e-9
    assert moving[0] - zero[0] < 0.3 + 0.03                         # and not longer
    assert tele.snapshot()['reverse'] and not tele.snapshot()['reverse_wait']


def test_the_direction_change_waits_for_the_measured_speed_to_reach_0(tele):
    _run(tele, 1.0, 0.0, 1.0)

    # The car coasts: 0.6 m/s at t=1.0 down to 0 at t=1.6 (still 0.12 at 1.48).
    def rolling(now):
        return max(0.0, 0.6 - (now - 1.0))
    rev = _run(tele, -1.0, 1.0, 2.0, v_meas=rolling)
    first = next(n for n, v in rev if v != 0.0)
    assert first >= 1.55 - 1e-9                                     # |v| <= 0.05 from 1.55
    assert all(v == 0.0 for n, v in rev if n < first)
    assert tele.snapshot()['reverse']


def test_reverse_to_forward_goes_through_0_as_well(tele):
    _run(tele, -1.0, 0.0, 1.0)
    fwd = _run(tele, 1.0, 1.0, 2.0)
    first = next(n for n, v in fwd if v != 0.0)
    assert first - 1.0 >= 0.3 - 1e-9
    assert dict(fwd)[first] == pytest.approx(0.4)                   # then the forward floor


def test_a_stale_measurement_does_not_block_the_direction_change(tele):
    _run(tele, 1.0, 0.0, 1.0)
    tele.measured(0.6, 1.0)                                         # last odometry, then none
    rev = _run(tele, -1.0, 1.0, 2.0)
    first = next(n for n, v in rev if v != 0.0)
    assert first - 1.0 < MEASURED_MAX_AGE_S + 0.05


def test_moving_against_the_stick_waits_even_without_a_previous_command(tele):
    """A car still rolling forward (after a mission, pushed) is stopped before reversing."""
    rev = _run(tele, -1.0, 0.0, 0.5, v_meas=lambda now: 0.3)
    assert all(v == 0.0 for _, v in rev)
    assert tele.snapshot()['reverse_wait']


def test_releasing_the_deadman_in_reverse_gives_a_zero_at_once(tele):
    _run(tele, -1.0, 0.0, 1.0)
    status, cmd = tele.on_input('a', {'deadman': False}, 1.0)
    assert status == 'released' and cmd.speed == 0.0


def test_the_watchdog_zeroes_reverse_too(tele):
    _run(tele, -1.0, 0.0, 1.0)
    assert tele.tick(1.0 + 0.2).speed < 0.0
    cmd = tele.tick(1.0 + 0.27)
    assert cmd.reason == 'zero' and cmd.speed == 0.0


# ---- 4. server-side clamps -------------------------------------------------------------------

def test_the_server_cap_is_1_m_s_whatever_the_config_or_the_client_says():
    t = Teleop(TeleopConfig(max_speed_mps=5.0))
    assert t.cfg.max_speed_mps == HARD_MAX_SPEED_MPS == 1.0
    t.acquire('a', 0.0)
    now = 0.0
    for _ in range(200):
        drive(t, thr=1.0, limit=9.0, now=now)
        cmd = t.tick(now)
        now += 0.02
    assert cmd.speed == pytest.approx(1.0) and t.limit == 1.0


def test_the_client_limit_is_clamped_to_0_4_1_0(tele):
    drive(tele, thr=1.0, limit=0.1)
    assert tele.limit == 0.4
    drive(tele, thr=1.0, limit=0.8)
    assert tele.limit == 0.8


def test_steering_is_clamped_and_rate_limited(tele):
    drive(tele, thr=0.0, steer=5.0, now=0.0)
    tele.tick(0.0)
    drive(tele, thr=0.0, steer=5.0, now=0.02)
    cmd = tele.tick(0.02)
    assert cmd.steer == pytest.approx(1.5 * 0.02)                  # steer_rate_radps
    now = 0.02
    for _ in range(50):
        now += 0.02
        drive(tele, thr=0.0, steer=5.0, now=now)
        cmd = tele.tick(now)
    assert cmd.steer == pytest.approx(0.30)                        # max_steer_rad


def test_speed_falls_at_once(tele):
    now = 0.0
    for _ in range(100):
        drive(tele, thr=1.0, limit=1.0, now=now)
        tele.tick(now)
        now += 0.02
    drive(tele, thr=0.16, limit=1.0, now=now)
    assert tele.tick(now).speed == pytest.approx(speed_for(0.16, 1.0, 0.15, 0.4))


def test_non_finite_input_is_a_zero_and_disarms(tele):
    drive(tele, thr=1.0)
    tele.tick(0.0)
    status, cmd = tele.on_input('a', {'seq': 99_999, 'thr': float('nan'), 'steer': 0.0,
                                      'deadman': True}, 0.01)
    assert status == 'invalid' and cmd.speed == 0.0 and not tele.armed


def test_input_rate_is_limited_but_a_release_never_is(tele):
    statuses = [drive(tele, thr=1.0, now=0.0)[0] for _ in range(60)]
    assert statuses.count('ok') == 50 and statuses.count('rate') == 10
    status, cmd = tele.on_input('a', {'deadman': False}, 0.0)
    assert status == 'released' and cmd.speed == 0.0


def test_stale_seq_is_dropped(tele):
    tele.on_input('a', {'seq': 50, 'thr': 1.0, 'steer': 0.0, 'deadman': True}, 0.0)
    assert tele.on_input('a', {'seq': 49, 'thr': 1.0, 'steer': 0.0, 'deadman': True},
                         0.0)[0] == 'stale'


# ---- 1. the 250 ms watchdog, 2. release = zero at once ----------------------------------------

def test_the_watchdog_zeroes_250_ms_after_the_last_input(tele):
    drive(tele, thr=1.0, now=0.0)
    assert tele.tick(0.24).speed > 0
    cmd = tele.tick(0.251)
    assert cmd.reason == 'zero' and cmd.speed == 0.0
    assert tele.stats['watchdog_trips'] == 1
    assert ('a', 'tripped', 'watchdog') in tele.pop_events()


def test_after_a_trip_the_thumb_must_lift_before_it_moves_again(tele):
    drive(tele, thr=1.0, now=0.0)
    tele.tick(0.3)
    assert drive(tele, thr=1.0, now=0.31)[0] == 'tripped'
    assert tele.tick(0.32).speed == 0.0
    tele.on_input('a', {'deadman': False}, 0.33)
    assert drive(tele, thr=1.0, now=0.34)[0] == 'ok'
    assert tele.tick(0.34).speed == pytest.approx(0.4)


def test_releasing_the_gas_gives_a_zero_to_publish_now(tele):
    drive(tele, thr=1.0, now=0.0)
    tele.tick(0.0)
    status, cmd = tele.on_input('a', {'deadman': False}, 0.01)
    assert status == 'released' and cmd.speed == 0.0 and cmd.steer == 0.0


def test_zeros_for_the_tail_then_silence_so_the_mux_lane_times_out(tele):
    drive(tele, thr=1.0, now=0.0)
    tele.tick(0.0)
    tele.on_input('a', {'deadman': False}, 0.1)
    assert tele.tick(0.2).speed == 0.0
    assert tele.tick(0.59).speed == 0.0
    assert tele.tick(0.61) is None


def test_nothing_is_published_without_a_driver(tele):
    assert tele.tick(0.0) is None


# ---- 6. one driver at a time ------------------------------------------------------------------

def test_a_second_client_gets_busy(tele):
    ok, why = tele.acquire('b', 0.0)
    assert not ok and 'occupata' in why
    assert drive(tele, client='b', thr=1.0)[0] == 'not_holder'


def test_closing_the_driver_releases_with_a_zero(tele):
    drive(tele, thr=1.0)
    tele.tick(0.0)
    cmd = tele.release('a', 0.01)
    assert cmd.speed == 0.0 and tele.holder is None
    assert tele.acquire('b', 0.02)[0]


def test_revoke_zeroes_and_tells_the_driver(tele):
    drive(tele, thr=1.0)
    tele.tick(0.0)
    cmd = tele.revoke('E-STOP', 0.01)
    assert cmd.speed == 0.0 and tele.holder is None
    assert tele.pop_events() == [('a', 'revoked', 'E-STOP')]


def test_config_rejects_nonsense():
    with pytest.raises(ValueError):
        TeleopConfig(max_speed_mps=0.2)
    with pytest.raises(ValueError):
        TeleopConfig(deadzone=1.0)
    with pytest.raises(ValueError):
        TeleopConfig(reverse_hold_s=-0.1)
    with pytest.raises(ValueError):
        TeleopConfig(stop_speed_mps=0.0)
    assert math.isclose(TeleopConfig(default_limit_mps=3.0).default_limit_mps, 1.0)
