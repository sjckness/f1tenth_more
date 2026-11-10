"""Keyboard teleop safety: dead-man, stop, discrete speed >= 0.4 m/s, steering limits.

    python3 -m pytest tools/isaac_keyboard/test_isaac_keyboard.py -q
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import isaac_keyboard as kb  # noqa: E402


def core(**kw):
    return kb.KeyboardTeleopCore(kb.TeleopParams(**kw))


def test_no_key_is_zero():
    c = core()
    assert c.command(0.0) == (0.0, 0.0)          # never sampled
    c.sample(0.0, set())
    assert c.command(0.0) == (0.0, 0.0)


def test_held_key_drives_and_release_stops_at_once():
    c = core()
    c.sample(0.0, {kb.FWD})
    assert c.command(0.0) == (0.4, 0.0)
    c.sample(0.05, set())                          # released
    assert c.command(0.05) == (0.0, 0.0)


def test_deadman_zero_within_0_3_s_when_input_stops():
    c = core()
    c.sample(0.0, {kb.FWD, kb.LEFT})
    assert c.command(0.29) == (0.4, 0.319)        # still within the dead-man window
    assert c.command(0.301) == (0.0, 0.0)         # no fresh sample: zero
    c.sample(1.0, {kb.FWD})                       # input back, key still held
    assert c.command(1.0) == (0.4, 0.0)


def test_space_stops_and_latches_until_drive_keys_released():
    c = core()
    c.sample(0.0, {kb.FWD})
    c.sample(0.1, {kb.FWD, kb.STOP})
    assert c.command(0.1) == (0.0, 0.0)
    c.sample(0.2, {kb.FWD})                       # space released, W still held: stays stopped
    assert c.command(0.2) == (0.0, 0.0)
    c.sample(0.3, set())                          # everything released: latch cleared
    c.sample(0.4, {kb.FWD})
    assert c.command(0.4) == (0.4, 0.0)


def test_speed_is_zero_or_at_least_0_4_at_every_level():
    c = core(max_speed=1.0, speed_step=0.1)
    seen = set()
    for i in range(20):                           # past both ends of the level range
        c.sample(i, {kb.FWD}, {kb.FASTER} if i < 10 else {kb.SLOWER})
        v, _ = c.command(i)
        seen.add(v)
        assert v == 0.0 or 0.4 <= abs(v) <= 1.0
    assert min(seen) == 0.4 and max(seen) == 1.0
    c.sample(30, {kb.BACK})
    v, _ = c.command(30)
    assert v == -0.4


def test_misconfigured_min_speed_is_still_0_4():
    c = core(min_speed=0.1, speed_step=0.1)
    c.sample(0.0, {kb.FWD})
    assert c.command(0.0)[0] == 0.4
    assert kb.clamp_command(0.05, 0.0, kb.TeleopParams())[0] == 0.4
    assert kb.clamp_command(-0.2, 0.0, kb.TeleopParams())[0] == -0.4
    assert kb.clamp_command(5.0, 0.0, kb.TeleopParams(max_speed=1.0))[0] == 1.0


def test_steering_within_car_limits():
    c = core()
    c.sample(0.0, {kb.LEFT})
    assert c.command(0.0) == (0.0, 0.319)
    c.sample(0.1, {kb.RIGHT})
    assert c.command(0.1) == (0.0, -0.309)
    c.sample(0.2, {kb.LEFT, kb.RIGHT})
    assert c.command(0.2) == (0.0, 0.0)
    p = kb.TeleopParams()
    assert kb.clamp_command(0.0, 1.0, p)[1] == 0.319
    assert kb.clamp_command(0.0, -1.0, p)[1] == -0.309


def test_forward_and_back_together_is_zero_speed():
    c = core()
    c.sample(0.0, {kb.FWD, kb.BACK})
    assert c.command(0.0) == (0.0, 0.0)
