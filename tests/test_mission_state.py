"""Tests for the mission state machine, with no rclpy and no clock.

The state machine was extracted out of the node precisely so these could
exist: mission-level failures live in state machines, and the first run of
this file found one (see
``test_arrival_dominates_reachability_and_does_not_orbit``).
"""

from math import cos, hypot, radians, sin

import numpy as np
import pytest

from go_to_object.mission_state import (
    GoToObjectMission,
    MissionCommand,
    MissionParams,
    MissionState,
)
from go_to_object.pursuit_geometry import (
    CurvatureLimiter,
    PursuitParams,
    solve_pursuit,
    wrap_pi,
)


class StubTrack:
    """Only what the state machine reads: position, converged, confidence."""

    def __init__(self, position, converged=True, confidence=1.0):
        self.position = np.array(position, dtype=float)
        self.converged = converged
        self.confidence = confidence


def _mission(rate=0.5, **mission_kwargs):
    return GoToObjectMission(PursuitParams(), MissionParams(**mission_kwargs),
                             CurvatureLimiter(rate))


def _drive(machine, obj, *, speed=2.0, dt=0.01, t_max=30.0,
           start=(0.0, 0.0, 0.0), track_factory=StubTrack):
    """Run the machine closed-loop against a perfectly known object."""
    x, y, psi = start
    log = []
    for i in range(int(t_max / dt)):
        t = i * dt
        command = machine.update(t, (x, y), psi, track_factory(obj))
        log.append((t, command, hypot(obj[0] - x, obj[1] - y)))
        if command.state is MissionState.ARRIVED:
            break
        ds = speed * dt if command.drive_enable else 0.0
        x += ds * cos(psi)
        y += ds * sin(psi)
        psi = wrap_pi(psi + command.curvature * ds)
    return log


def _states(log):
    return [entry[1].state for entry in log]


# -- ACQUIRE gating --------------------------------------------------------

@pytest.mark.parametrize('converged,confidence,expected', [
    (False, 0.0, MissionState.ACQUIRE),
    (True, 0.0, MissionState.ACQUIRE),      # converged alone is not enough
    (False, 1.0, MissionState.ACQUIRE),     # confidence alone is not enough
    (True, 1.0, MissionState.APPROACH),     # both
])
def test_approach_requires_converged_and_confidence_together(
        converged, confidence, expected):
    machine = _mission()
    machine.update(0.00, (0.0, 0.0), 0.0, StubTrack((10.0, 0.0), False, 0.0))

    command = machine.update(0.01, (0.0, 0.0), 0.0,
                             StubTrack((10.0, 0.0), converged, confidence))

    assert command.state is expected


def test_confidence_exactly_at_the_threshold_is_accepted():
    machine = _mission()
    machine.update(0.00, (0.0, 0.0), 0.0, StubTrack((10.0, 0.0), False, 0.0))
    threshold = machine.mission.confidence_threshold

    command = machine.update(0.01, (0.0, 0.0), 0.0,
                             StubTrack((10.0, 0.0), True, threshold))

    assert command.state is MissionState.APPROACH


def test_acquire_commands_nothing_while_it_waits():
    machine = _mission()
    command = machine.update(0.0, (0.0, 0.0), 0.0,
                             StubTrack((10.0, 0.0), False, 0.0))

    assert command.state is MissionState.ACQUIRE
    assert command.curvature == 0.0
    assert command.drive_enable is False


# -- LOST ------------------------------------------------------------------

def test_a_vanished_estimate_enters_lost_and_ramps_the_steering_out():
    """Drive gate drops at once; the wheels unwind under the rate limit.

    The servo cannot step to centre any more than it can step anywhere else,
    so LOST ramps. The vehicle is already being stopped by the dropped drive
    gate while that happens.
    """
    rate, dt = 0.5, 0.01
    machine = _mission(rate=rate)
    for i in range(200):
        machine.update(i * dt, (0.0, 0.0), 0.0, StubTrack((4.0, 4.0)))
    assert machine.state is MissionState.APPROACH
    steering = machine.limiter.previous
    assert abs(steering) > 0.1, 'need a real curvature to unwind from'

    curvatures = []
    for i in range(200, 400):
        command = machine.update(i * dt, (0.0, 0.0), 0.0, None)
        assert command.state is MissionState.LOST
        assert command.drive_enable is False, 'gate drops immediately'
        curvatures.append(command.curvature)

    steps = [abs(b - a) for a, b in zip([steering] + curvatures, curvatures)]
    assert max(steps) <= rate * dt + 1e-12, 'a ramp, not a step'
    assert curvatures[-1] == 0.0, 'and it does reach centre'


def test_lost_resyncs_the_limiter_to_measured_steering_when_available():
    """Where the wheels are beats where they were last told to go."""
    machine = _mission()
    for i in range(100):
        machine.update(i * 0.01, (0.0, 0.0), 0.0, StubTrack((10.0, 6.0)))
    commanded = machine.limiter.previous

    machine.update(1.00, (0.0, 0.0), 0.0, None, measured_kappa=-0.31)

    assert commanded != pytest.approx(-0.31)
    # Resynced to -0.31 on entry, then one ramp step toward zero.
    assert machine.limiter.previous == pytest.approx(-0.31 + 0.5 * 0.01)


def test_lost_without_measured_steering_keeps_the_last_commanded_curvature():
    machine = _mission()
    for i in range(100):
        machine.update(i * 0.01, (0.0, 0.0), 0.0, StubTrack((10.0, 6.0)))
    commanded = machine.limiter.previous

    machine.update(1.00, (0.0, 0.0), 0.0, None)

    assert machine.limiter.previous == pytest.approx(commanded - 0.5 * 0.01)


def test_search_stays_in_search_when_no_track_has_ever_existed():
    machine = _mission()
    command = machine.update(0.0, (0.0, 0.0), 0.0, None)

    assert command.state is MissionState.SEARCH
    assert command.drive_enable is False


def test_re_acquisition_does_not_inherit_the_stale_curvature():
    """A command from an abandoned approach must not slew into the next one."""
    machine = _mission()
    obj = (10.0, 6.0)
    for i in range(60):                       # build up a curvature
        machine.update(i * 0.01, (0.0, 0.0), 0.0, StubTrack(obj))
    assert machine.state is MissionState.APPROACH
    stale = machine.limiter.previous
    assert stale is not None and abs(stale) > 0.0

    machine.update(0.60, (0.0, 0.0), 0.0, None)          # -> LOST
    machine.update(0.61, (0.0, 0.0), 0.0, StubTrack(obj))  # -> ACQUIRE
    command = machine.update(0.62, (0.0, 0.0), 0.0, StubTrack(obj))

    expected = solve_pursuit((0.0, 0.0), 0.0, obj, machine.pursuit).curvature
    assert command.state is MissionState.APPROACH
    assert command.curvature == pytest.approx(expected), 'seeded, not slewed'


# -- ARRIVED ---------------------------------------------------------------

def test_arrived_latches_and_does_not_chatter_around_d_stop():
    machine = _mission()
    d_stop = machine.pursuit.d_stop
    machine.update(0.00, (0.0, 0.0), 0.0, StubTrack((10.0, 0.0)))
    machine.update(0.01, (0.0, 0.0), 0.0, StubTrack((10.0, 0.0)))
    machine.update(0.02, (0.0, 0.0), 0.0, StubTrack((d_stop * 0.5, 0.0)))
    assert machine.state is MissionState.ARRIVED

    # Jitter the estimate back and forth across d_stop; latching subsumes
    # hysteresis, so the state must not follow it.
    for i, offset in enumerate([1.5, 0.5, 2.0, 0.4, 3.0]):
        command = machine.update(0.03 + i * 0.01, (0.0, 0.0), 0.0,
                                 StubTrack((d_stop * offset, 0.0)))
        assert command.state is MissionState.ARRIVED
        assert command.curvature == 0.0
        assert command.drive_enable is False


def test_reset_releases_the_latch():
    machine = _mission()
    machine.update(0.00, (0.0, 0.0), 0.0, StubTrack((0.1, 0.0)))
    machine.update(0.01, (0.0, 0.0), 0.0, StubTrack((0.1, 0.0)))
    assert machine.state is MissionState.ARRIVED

    machine.reset()

    assert machine.state is MissionState.SEARCH
    assert machine.limiter.previous == 0.0, 'synced, not forgotten'


# -- REPOSITION ------------------------------------------------------------

UNREACHABLE_POSES = [(0.0, 2.0), (0.0, -2.0), (1.0, 1.5), (0.5, 2.5),
                     (-0.5, 1.8), (0.2, 1.0), (0.0, 0.8)]


@pytest.mark.parametrize('obj', UNREACHABLE_POSES)
def test_unreachable_targets_reposition_then_complete_the_approach(obj):
    """Each pose from the measured recovery table resolves inside the timeout."""
    machine = _mission()
    log = _drive(machine, obj, speed=2.0)
    states = _states(log)

    assert MissionState.REPOSITION in states, 'must recognise it cannot turn that tight'

    entered = states.index(MissionState.REPOSITION)
    after = states[entered:]
    assert MissionState.APPROACH in after, 'must leave REPOSITION'

    left = entered + after.index(MissionState.APPROACH)
    duration = log[left][0] - log[entered][0]
    assert duration <= machine.mission.reposition_timeout
    # Measured worst case across these poses at the default limiter rate is
    # 0.93 s. The unlimited-slew figure was 0.61 s; routing the opposite-lock
    # demand through the rate limiter, as Part A requires, is what costs the
    # difference.
    assert duration < 1.2, f'recovery took {duration:.2f} s'

    assert states[-1] is MissionState.ARRIVED, 'and the approach completes'


@pytest.mark.parametrize('obj', UNREACHABLE_POSES)
def test_reposition_turns_away_from_the_object_not_toward_it(obj):
    machine = _mission()
    machine.update(0.00, (0.0, 0.0), 0.0, StubTrack(obj))
    command = machine.update(0.01, (0.0, 0.0), 0.0, StubTrack(obj))

    assert command.state is MissionState.REPOSITION
    solution = solve_pursuit((0.0, 0.0), 0.0, obj, machine.pursuit)
    # The demand is full opposite lock; what reaches the servo is the ramp
    # toward it, which is why curvature_raw is reported separately.
    assert command.curvature_raw * solution.alpha < 0.0, 'away, not toward'
    assert abs(command.curvature_raw) == pytest.approx(1.0 / machine.pursuit.r_min)
    assert abs(command.curvature) <= abs(command.curvature_raw)


@pytest.mark.parametrize('obj', UNREACHABLE_POSES)
def test_turning_toward_an_unreachable_target_is_the_livelock(obj):
    """The counterfactual that justifies turning away. Not a code path.

    Driving ``+1/r_min`` toward the target orbits it: the vehicle circles at
    the minimum radius and the reachability condition never resolves.
    """
    x, y, psi = 0.0, 0.0, 0.0
    pursuit = PursuitParams()
    speed, dt = 2.0, 0.01
    became_reachable = False

    for _ in range(int(5.0 / dt)):
        solution = solve_pursuit((x, y), psi, obj, pursuit)
        if solution.reachable:
            became_reachable = True
            break
        curvature = (1.0 / pursuit.r_min) * (1.0 if solution.alpha >= 0 else -1.0)
        ds = speed * dt
        x += ds * cos(psi)
        y += ds * sin(psi)
        psi = wrap_pi(psi + curvature * ds)

    assert not became_reachable, 'turning toward it must never resolve'


def test_reposition_aborts_to_lost_when_it_runs_out_of_time():
    """The timeout is there so an unmodelled geometry cannot hang the mission.

    Every measured pose recovers in well under 0.7 s, so the timeout is
    shortened here rather than inventing a geometry that cannot recover --
    the behaviour under test is the abort path, not the geometry.
    """
    machine = _mission(reposition_timeout=0.05)
    obj = (0.0, 2.0)

    machine.update(0.00, (0.0, 0.0), 0.0, StubTrack(obj))
    machine.update(0.01, (0.0, 0.0), 0.0, StubTrack(obj))
    assert machine.state is MissionState.REPOSITION

    before = machine.limiter.previous
    command = machine.update(0.20, (0.0, 0.0), 0.0, StubTrack(obj))

    assert command.state is MissionState.LOST
    assert command.drive_enable is False
    assert abs(command.curvature) < abs(before), 'unwinding toward centre'


def test_arrival_dominates_reachability_and_does_not_orbit():
    """Regression: found by the first run of this file.

    A target within ``d_stop`` but inside the minimum turning circle is
    *reached*, not unreachable. Checking arrival only in APPROACH let the
    vehicle pass within 0.2 m of the object and then reposition away from it,
    cycling APPROACH <-> REPOSITION until the 120 s sim budget ran out.
    """
    machine = _mission()
    d_stop = machine.pursuit.d_stop
    obj = (0.0, d_stop * 0.6)          # inside d_stop, and hard abeam

    solution = solve_pursuit((0.0, 0.0), 0.0, obj, machine.pursuit)
    assert solution.arrived and not solution.reachable, 'the conflicting case'

    machine.update(0.00, (0.0, 0.0), 0.0, StubTrack(obj))
    command = machine.update(0.01, (0.0, 0.0), 0.0, StubTrack(obj))

    assert command.state is MissionState.ARRIVED


# -- the limiter is applied to every commanded curvature -------------------

@pytest.mark.parametrize('obj', [(10.0, 6.0), (0.0, 2.0)])
def test_every_command_passes_through_the_curvature_limiter(obj):
    """Including the REPOSITION override and the behind-case maximum."""
    rate, dt = 0.5, 0.01
    machine = _mission(rate=rate)
    log = _drive(machine, obj, dt=dt)

    driving = [e[1].curvature for e in log if e[1].drive_enable]
    steps = [abs(b - a) for a, b in zip(driving, driving[1:])]

    assert steps
    assert max(steps) <= rate * dt + 1e-12


def test_a_commanded_curvature_is_reported_with_its_solution():
    machine = _mission()
    machine.update(0.00, (0.0, 0.0), 0.0, StubTrack((10.0, 4.0)))
    command = machine.update(0.01, (0.0, 0.0), 0.0, StubTrack((10.0, 4.0)))

    assert isinstance(command, MissionCommand)
    assert command.solution is not None
    assert command.solution.distance == pytest.approx(hypot(10.0, 4.0))


# -- Part A: the limiter is never bypassed at a mission change -------------

def test_lost_then_reacquire_across_a_sign_change_is_a_ramp_not_a_step():
    """The failure Part A exists to prevent.

    Approach hard left, lose the track, re-acquire an object hard right. The
    old ``reset()`` cleared the limiter to unseeded and passed the first new
    command straight through -- a full sign-reversal step handed to a servo
    that cannot take one.
    """
    rate, dt = 0.5, 0.01
    machine = _mission(rate=rate)

    for i in range(200):
        machine.update(i * dt, (0.0, 0.0), 0.0, StubTrack((6.0, 6.0)))
    assert machine.limiter.previous > 0.1

    commands = [machine.limiter.previous]
    for i in range(200, 260):                      # lost
        commands.append(machine.update(i * dt, (0.0, 0.0), 0.0, None).curvature)
    for i in range(260, 600):                      # re-acquired, other side
        commands.append(
            machine.update(i * dt, (0.0, 0.0), 0.0, StubTrack((6.0, -6.0))).curvature)

    steps = [abs(b - a) for a, b in zip(commands, commands[1:])]
    assert max(steps) <= rate * dt + 1e-12, f'step of {max(steps):.4f} reached the servo'
    assert min(commands) < -0.1, 'and it did eventually steer the other way'


def test_cold_start_ramps_from_centred_wheels():
    rate, dt = 0.5, 0.01
    machine = _mission(rate=rate)
    assert machine.limiter.previous == 0.0

    commands = [0.0]
    for i in range(50):
        commands.append(
            machine.update(i * dt, (0.0, 0.0), 0.0, StubTrack((4.0, 4.0))).curvature)

    steps = [abs(b - a) for a, b in zip(commands, commands[1:])]
    assert max(steps) <= rate * dt + 1e-12
    assert commands[1] == pytest.approx(0.0), 'first tick has dt = 0'


def test_measured_steering_overrides_last_commanded_on_reacquire():
    """Both present and far enough apart to tell which one was used."""
    rate, dt = 0.5, 0.01
    machine = _mission(rate=rate)
    for i in range(200):
        machine.update(i * dt, (0.0, 0.0), 0.0, StubTrack((6.0, 6.0)))
    commanded = machine.limiter.previous

    measured = -0.5
    assert abs(commanded - measured) > 0.5, 'the two must be distinguishable'

    machine.update(2.00, (0.0, 0.0), 0.0, None, measured_kappa=measured)
    machine.update(2.01, (0.0, 0.0), 0.0, StubTrack((6.0, 6.0)),
                   measured_kappa=measured)
    command = machine.update(2.02, (0.0, 0.0), 0.0, StubTrack((6.0, 6.0)),
                             measured_kappa=measured)

    assert command.curvature < measured + 3 * rate * dt, 'ramped up from the wheels'
    assert command.curvature < 0.0, 'not from the stale commanded value'


# -- C4: watchdog ----------------------------------------------------------

def test_watchdog_fires_when_odometry_stops_arriving():
    machine = _mission(odom_timeout=0.2)
    for i in range(100):
        machine.update(i * 0.01, (0.0, 0.0), 0.0, StubTrack((10.0, 6.0)),
                       last_odom_stamp=i * 0.01)
    assert machine.state is MissionState.APPROACH
    assert abs(machine.limiter.previous) > 0.0

    frozen = 0.99
    inside = machine.update(1.10, (0.0, 0.0), 0.0, StubTrack((10.0, 6.0)),
                            last_odom_stamp=frozen)
    assert inside.watchdog is False, 'still inside the timeout'

    tripped = machine.update(1.30, (0.0, 0.0), 0.0, StubTrack((10.0, 6.0)),
                             last_odom_stamp=frozen)

    assert tripped.watchdog is True
    assert tripped.curvature == 0.0, 'failsafe is immediate, not a ramp'
    assert tripped.drive_enable is False
    assert tripped.state is MissionState.LOST
    assert machine.limiter.previous == 0.0


def test_watchdog_fires_when_the_track_stays_missing():
    machine = _mission(state_timeout=1.0)
    for i in range(100):
        machine.update(i * 0.01, (0.0, 0.0), 0.0, StubTrack((10.0, 6.0)))

    ordinary = machine.update(1.20, (0.0, 0.0), 0.0, None)
    assert ordinary.state is MissionState.LOST
    assert ordinary.watchdog is False, 'an ordinary LOST ramps out'

    tripped = machine.update(2.50, (0.0, 0.0), 0.0, None)

    assert tripped.watchdog is True
    assert tripped.curvature == 0.0


def test_a_healthy_tick_never_trips_the_watchdog():
    machine = _mission()
    for i in range(500):
        command = machine.update(i * 0.01, (0.0, 0.0), 0.0,
                                 StubTrack((30.0, 6.0)), last_odom_stamp=i * 0.01)
        assert command.watchdog is False


def test_the_watchdog_recovers_when_data_returns():
    machine = _mission(odom_timeout=0.2)
    machine.update(0.0, (0.0, 0.0), 0.0, StubTrack((10.0, 6.0)), last_odom_stamp=0.0)
    assert machine.update(1.0, (0.0, 0.0), 0.0, StubTrack((10.0, 6.0)),
                          last_odom_stamp=0.0).watchdog is True

    recovered = machine.update(1.01, (0.0, 0.0), 0.0, StubTrack((10.0, 6.0)),
                               last_odom_stamp=1.01)

    assert recovered.watchdog is False
    assert recovered.state is MissionState.APPROACH, 'straight back to driving'
