"""heartbeat.py: packet format and the link state machine (no sockets, a fake clock)."""

from f1tenth_comms.heartbeat import (decode, encode, LinkMonitor, PACKET, STATE_DEGRADED,
                                     STATE_LOST, STATE_OK)
import pytest

TS, LOCAL = 0, 1


def monitor(**kw):
    # ok_after_s 0: most tests evaluate once, at the end; the dwell has its own test.
    args = {'start_t': 0.0, 'timeout_s': 2.0, 'window_s': 10.0, 'degraded_loss_pct': 20.0,
            'degraded_rtt_ms': 250.0, 'degraded_silence_s': 1.2, 'failback_s': 3.0,
            'ok_after_s': 0.0}
    args.update(kw)
    return LinkMonitor(['tailscale', 'local'], **args)


def run(m, t0, t1, period=0.5, rtt=None, start_seq=1):
    """Send a round every period from t0 to t1; rtt[path] = seconds or None (no reply)."""
    rtt = rtt or {}
    seq = start_seq
    t = t0
    while t < t1 - 1e-9:
        m.on_send(seq, t)
        for path, r in rtt.items():
            if r is not None:
                m.on_reply(path, seq, t + r)
        seq += 1
        t += period
    return seq


def test_packet_round_trip_and_rejects():
    data = encode(1, 0xDEADBEEF, 42, 123456789)
    assert len(data) == PACKET.size == 24
    p = decode(data)
    assert (p.path, p.nonce, p.seq, p.t_ns) == (1, 0xDEADBEEF, 42, 123456789)
    assert decode(b'XXXX' + data[4:]) is None          # magic
    assert decode(data[:4] + b'\x02' + data[5:]) is None   # version
    assert decode(data + b'\x00') is None               # length
    assert decode(b'') is None


def test_ok_on_tailscale_with_rtt_and_no_loss():
    m = monitor()
    run(m, 0.0, 5.0, rtt={TS: 0.020, LOCAL: 0.005})
    v = m.evaluate(5.0)
    assert v.state == STATE_OK and v.state_name == 'OK'
    assert v.active == TS                     # first in priority, not the fastest
    assert v.rtt_ms == pytest.approx(20.0)
    assert v.rtt_mean_ms == pytest.approx(20.0)
    assert v.loss_pct == 0.0
    assert v.paths[LOCAL].up and v.paths[LOCAL].rtt_mean_ms == pytest.approx(5.0)
    assert v.since_last_reply_s == pytest.approx(0.5 - 0.020)
    assert (v.rounds_sent, v.rounds_answered, v.rounds_lost) == (10, 10, 0)


def test_fallback_to_local_at_once_when_tailscale_stops():
    m = monitor()
    seq = run(m, 0.0, 5.0, rtt={TS: 0.020, LOCAL: 0.005})
    run(m, 5.0, 9.0, rtt={TS: None, LOCAL: 0.005}, start_seq=seq)
    # Tailscale's last reply was at 4.52 s: down 2 s later.
    v = m.evaluate(9.0)
    assert v.active == LOCAL and v.state == STATE_OK
    assert not v.paths[TS].up
    assert v.rtt_ms == pytest.approx(5.0)
    assert v.loss_pct == 0.0                  # the round was answered on the other path
    assert v.paths[TS].loss_pct > 0


def test_fallback_after_degraded_silence_not_after_the_timeout():
    m = monitor()
    seq = run(m, 0.0, 5.0, rtt={TS: 0.020, LOCAL: 0.005})
    assert m.evaluate(5.0).active == TS
    run(m, 5.0, 6.5, rtt={TS: None, LOCAL: 0.005}, start_seq=seq)
    # Tailscale's last reply 4.52 s: quiet for 1.48 s > 1.2 s, still UP (< 2 s).
    v = m.evaluate(6.0)
    assert v.paths[TS].up and v.active == LOCAL and v.state == STATE_OK


def test_failback_to_tailscale_only_after_failback_s():
    m = monitor()
    seq = run(m, 0.0, 4.0, rtt={TS: None, LOCAL: 0.005})
    assert m.evaluate(4.0).active == LOCAL
    seq = run(m, 4.0, 6.0, rtt={TS: 0.020, LOCAL: 0.005}, start_seq=seq)
    assert m.evaluate(6.0).active == LOCAL    # up for 2 s < failback_s
    run(m, 6.0, 7.5, rtt={TS: 0.020, LOCAL: 0.005}, start_seq=seq)
    assert m.evaluate(7.5).active == TS


def test_a_tailscale_that_went_quiet_must_be_fresh_failback_s_before_it_is_back():
    m = monitor()
    seq = run(m, 0.0, 4.0, rtt={TS: 0.010, LOCAL: 0.010})
    assert m.evaluate(4.0).active == TS
    seq = run(m, 4.0, 5.5, rtt={TS: None, LOCAL: 0.010}, start_seq=seq)
    assert m.evaluate(5.5).active == LOCAL        # quiet 1.99 s: still UP, not FRESH
    seq = run(m, 5.5, 6.0, rtt={TS: 0.010, LOCAL: 0.010}, start_seq=seq)
    assert m.evaluate(6.0).active == LOCAL        # fresh again, but only for 0.5 s
    run(m, 6.0, 9.0, rtt={TS: 0.010, LOCAL: 0.010}, start_seq=seq)
    assert m.evaluate(9.0).active == TS


def test_ok_only_after_ok_after_s_without_a_degraded_condition():
    m = monitor(ok_after_s=2.0)
    seq = 1
    states = []
    for i in range(40):
        t = i * 0.5
        m.on_send(seq, t)
        if not 4 <= i <= 6:                       # 1.5 s without a reply on any path
            m.on_reply(TS, seq, t + 0.01)
        seq += 1
        states.append(m.evaluate(t + 0.25).state_name)
    assert states[:4] == ['DEGRADED'] * 4         # fine from the first reply: OK 2 s later
    assert states[4] == 'OK'
    assert states[5] == 'DEGRADED'                # quiet for more than 1.2 s
    back = states.index('OK', 5)
    assert back - 5 >= 4                          # not OK again before 2 s clean
    assert states[back:] == ['OK'] * (40 - back)  # and no flicker after
    assert 'LOST' not in states


def test_lost_when_no_path_answers_for_timeout():
    m = monitor()
    seq = run(m, 0.0, 3.0, rtt={TS: 0.010, LOCAL: 0.010})
    run(m, 3.0, 8.0, rtt={}, start_seq=seq)
    assert m.evaluate(4.0).state == STATE_DEGRADED          # silent 1.5 s
    v = m.evaluate(4.6)
    assert v.state == STATE_LOST and v.active is None and v.rtt_ms is None
    assert v.since_last_reply_s == pytest.approx(4.6 - 2.51)
    v = m.evaluate(8.0)
    assert v.rounds_lost > 0 and v.rounds_sent == 16
    assert v.rounds_answered + v.rounds_lost <= v.rounds_sent


def test_degraded_not_lost_while_proving_at_start():
    m = monitor()
    m.on_send(1, 0.0)
    assert m.evaluate(0.1).state == STATE_DEGRADED
    m.on_send(2, 0.5)
    assert m.evaluate(2.5).state == STATE_LOST      # start + timeout passed, never a reply


def test_degraded_on_loss():
    m = monitor()
    seq = 1
    for i in range(20):                       # every 3rd round lost on both paths: 33 %
        m.on_send(seq, i * 0.5)
        if i % 3:
            m.on_reply(TS, seq, i * 0.5 + 0.01)
        seq += 1
    v = m.evaluate(10.0)
    assert v.active == TS and v.state == STATE_DEGRADED
    # 16 rounds past the timeout + 3 answered since; round 18 is still in flight.
    assert v.loss_pct == pytest.approx(100.0 * 6 / 19)


def test_degraded_on_rtt():
    m = monitor(degraded_rtt_ms=100.0)
    run(m, 0.0, 5.0, rtt={TS: 0.150})
    assert m.evaluate(5.0).state == STATE_DEGRADED


def test_duplicate_unknown_and_late_replies_are_ignored():
    m = monitor()
    m.on_send(1, 0.0)
    assert m.on_reply(TS, 1, 0.010) == pytest.approx(0.010)
    assert m.on_reply(TS, 1, 0.020) is None      # duplicate
    assert m.on_reply(TS, 99, 0.020) is None     # unknown round
    assert m.on_reply(7, 1, 0.020) is None       # unknown path
    m.on_send(2, 1.0)
    assert m.on_reply(TS, 2, 3.5) is None        # 2.5 s > timeout: late
    v = m.evaluate(4.0)
    assert (v.rounds_answered, v.rounds_lost, v.late_replies) == (1, 1, 1)


def test_old_rounds_are_pruned():
    m = monitor()
    run(m, 0.0, 100.0, rtt={TS: 0.01})
    m.evaluate(100.0)
    assert len(m.rounds) <= (10.0 + 2.0 + 1.0) / 0.5 + 1


def test_bad_config():
    with pytest.raises(ValueError):
        LinkMonitor([], start_t=0.0)
    with pytest.raises(ValueError):
        LinkMonitor(['a'], start_t=0.0, timeout_s=0.0)
