"""Whitelist, the stale-command filter and confirmations."""

from f1tenth_telegram import guard


def test_whitelist():
    assert guard.allowed(123, [123, 456])
    assert guard.allowed('456', [123, 456])
    assert not guard.allowed(789, [123, 456])
    assert not guard.allowed(None, [123])
    assert not guard.allowed(123, [])


def test_stale_filter_at_ten_seconds():
    assert not guard.is_stale(100.0, 109.9, 10.0)
    assert not guard.is_stale(100.0, 110.0, 10.0)
    assert guard.is_stale(100.0, 110.1, 10.0)


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def test_confirmation_in_time():
    clock = Clock()
    c = guard.Confirmations(10.0, clock)
    nonce = c.ask('start:stack')
    clock.t += 9.0
    assert c.take(nonce) == ('start:stack', None)
    # A nonce is good for one answer only.
    action, why = c.take(nonce)
    assert action is None and 'no longer valid' in why


def test_confirmation_expires():
    clock = Clock()
    c = guard.Confirmations(10.0, clock)
    nonce = c.ask('killall')
    clock.t += 10.5
    assert c.pending() is None
    action, why = c.take(nonce)
    assert action is None and 'ask again' in why


def test_new_question_replaces_the_old_one():
    c = guard.Confirmations(10.0, Clock())
    old = c.ask('start:stack')
    new = c.ask('killall')
    assert c.take(old)[0] is None
    assert c.pending() == (new, 'killall')


def test_cancel():
    c = guard.Confirmations(10.0, Clock())
    nonce = c.ask('start:planner')
    c.cancel(nonce)
    assert c.pending() is None and c.take(nonce)[0] is None


def test_what_needs_a_confirmation():
    assert set(guard.NEEDS_CONFIRM) == {'start:stack', 'start:planner', 'killall'}
    assert 'estop' not in guard.NEEDS_CONFIRM
