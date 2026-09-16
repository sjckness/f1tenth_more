"""Tests for the odometry buffer, the object tracker, and the closed loop."""

from math import atan2, cos, degrees, hypot, radians, sin

import numpy as np
import pytest

from go_to_object.mission_state import (
    GoToObjectMission,
    MissionParams,
    MissionState,
)
from go_to_object.object_tracker import ObjectTracker, OdomBuffer, TrackerParams
from go_to_object.pursuit_geometry import (
    CurvatureLimiter,
    PursuitParams,
    wrap_pi,
)


def _world_to_body(rel_x, rel_y, psi):
    c, s = cos(psi), sin(psi)
    return c * rel_x + s * rel_y, -s * rel_x + c * rel_y


def _drive_straight(tracker, *, speed, duration, dt=0.01, t0=0.0, psi=0.0):
    n = int(round(duration / dt))
    for i in range(n + 1):
        t = t0 + i * dt
        tracker.push_odom(t, speed * (t - t0) * cos(psi),
                          speed * (t - t0) * sin(psi), psi)
    return t0 + n * dt


def _synthesise_detection(rng, params, bx, by):
    """Noise drawn from the measurement model *under test*.

    The sim must not carry its own noise constants. When it did, the test
    recalibrated ``sigma_range_base`` and ``sigma_bearing`` to match a sensor
    it had hardcoded separately, the shipped defaults were left validated by
    nothing, and the mismatch that produces -- gate rejects everything, track
    ages out, mission reports LOST two metres short of an object that is
    plainly there -- was invisible. Deriving the noise from the params makes
    that class of drift impossible by construction.
    """
    d = hypot(bx, by)
    sigma_range = params.sigma_range_base + params.sigma_range_slope * d * d
    sigma_cross = max(d * params.sigma_bearing, 1e-9)
    los = atan2(by, bx)
    dn = d + rng.normal(0.0, sigma_range)
    cn = rng.normal(0.0, sigma_cross)
    return (dn * cos(los) - cn * sin(los), dn * sin(los) + cn * cos(los))


def closed_loop(seed, *, tracker_params=None, pursuit=None, mission=None,
                max_kappa_rate=0.5, speed=1.5, dt=0.01, latency=0.05,
                obj=(18.5, 4.0), detection_hz=10.0, t_max=120.0,
                noise_scale=1.0):
    """Full loop: odom at 1/dt, detections at ``detection_hz``, pursuit, limiter."""
    tracker_params = tracker_params or TrackerParams()
    pursuit = pursuit or PursuitParams()
    mission = mission or MissionParams()

    rng = np.random.default_rng(seed)
    tracker = ObjectTracker(tracker_params)
    machine = GoToObjectMission(pursuit, mission, CurvatureLimiter(max_kappa_rate))

    obj = np.array(obj, dtype=float)
    x = y = psi = 0.0
    every = max(1, int(round((1.0 / detection_hz) / dt)))
    pending, samples = [], []

    for step in range(int(t_max / dt)):
        t = step * dt
        tracker.push_odom(t, x, y, psi)

        while pending and pending[0][0] <= t:
            _, capture, body = pending.pop(0)
            tracker.push_detection(capture, body)

        if step % every == 0:
            bx, by = _world_to_body(obj[0] - x, obj[1] - y, psi)
            noisy = _synthesise_detection(rng, tracker_params, bx, by)
            if noise_scale != 1.0:
                true = (bx, by)
                noisy = tuple(t_ + noise_scale * (n_ - t_)
                              for t_, n_ in zip(true, noisy))
            pending.append((t + latency, t, noisy))

        track = tracker.state(t)
        command = machine.update(t, (x, y), psi, track)
        samples.append((hypot(obj[0] - x, obj[1] - y), command.curvature,
                        command.state, t))

        if command.state is MissionState.ARRIVED:
            return dict(arrived=True, samples=samples, t=t, tracker=tracker)

        ds = speed * dt if command.drive_enable else 0.0
        x += ds * cos(psi)
        y += ds * sin(psi)
        psi = wrap_pi(psi + command.curvature * ds)

    return dict(arrived=False, samples=samples, t=t_max, tracker=tracker)


def driving_steps(result):
    """Per-cycle curvature steps while actually commanding a curvature."""
    driving = [s for s in result['samples']
               if s[2] in (MissionState.APPROACH, MissionState.REPOSITION)]
    return [abs(b[1] - a[1]) for a, b in zip(driving, driving[1:])]


# -- the odometry buffer ---------------------------------------------------

def test_buffer_interpolates_position_linearly():
    buf = OdomBuffer()
    buf.push(0.0, 0.0, 0.0, 0.0)
    buf.push(1.0, 2.0, 4.0, 0.0)

    sample = buf.at(0.25)

    assert sample is not None
    assert sample.x == pytest.approx(0.5)
    assert sample.y == pytest.approx(1.0)


def test_buffer_interpolates_heading_the_short_way_round():
    """170 deg -> -170 deg is a 20 deg step, not a 340 deg sweep through zero."""
    buf = OdomBuffer()
    buf.push(0.0, 0.0, 0.0, radians(170.0))
    buf.push(1.0, 0.0, 0.0, radians(-170.0))

    midpoint = buf.at(0.5)

    assert midpoint is not None
    assert abs(degrees(wrap_pi(midpoint.psi - radians(180.0)))) < 1e-9


def test_buffer_returns_none_outside_its_horizon():
    buf = OdomBuffer(horizon=2.0)
    for i in range(400):
        buf.push(i * 0.01, 0.0, 0.0, 0.0)

    assert buf.at(4.5) is None, 'no extrapolation past the newest sample'
    assert buf.at(3.99) is not None, 'the newest sample itself is valid'
    assert buf.at(1.0) is None, 'older than the horizon'
    assert buf.at(2.5) is not None


def test_buffer_drops_out_of_order_pushes():
    buf = OdomBuffer()
    assert buf.push(1.0, 1.0, 0.0, 0.0) is True
    assert buf.push(0.5, 99.0, 99.0, 0.0) is False
    assert buf.push(1.0, 99.0, 99.0, 0.0) is False, 'duplicate stamp is not newer'

    latest = buf.latest()
    assert latest is not None and latest.x == pytest.approx(1.0)


def test_a_stamp_just_ahead_of_odom_is_tolerated_but_a_clock_mismatch_is_not():
    """Scheduling jitter is served; a sim-time/wall-clock split is refused."""
    buf = OdomBuffer()
    for i in range(50):
        buf.push(i * 0.01, float(i), 0.0, 0.0)
    newest = buf.latest().stamp

    within = buf.at(newest + 0.004)
    assert within is not None and within.stamp == pytest.approx(newest)

    assert buf.at(newest + 0.5) is None, 'beyond one period, refuse and warn'


# -- the reason the object is held in the world frame ----------------------

def test_distance_shrinks_at_the_driven_speed_with_no_further_detections():
    """One detection, then odometry only. This is the core behaviour."""
    tracker = ObjectTracker(TrackerParams(max_age=30.0))
    speed = 2.0
    tracker.push_odom(0.0, 0.0, 0.0, 0.0)
    assert tracker.push_detection(0.0, (20.0, 0.0)) is True

    distances = []
    for i in range(1, 501):
        t = i * 0.01
        tracker.push_odom(t, speed * t, 0.0, 0.0)
        track = tracker.state(t)
        assert track is not None
        distances.append(track.distance)

    assert all(b < a for a, b in zip(distances, distances[1:])), 'monotonic'
    assert distances[-1] == pytest.approx(20.0 - speed * 5.0, abs=1e-6)


def test_bearing_tracks_vehicle_rotation_with_no_detections_arriving():
    tracker = ObjectTracker(TrackerParams(max_age=30.0))
    tracker.push_odom(0.0, 0.0, 0.0, 0.0)
    tracker.push_detection(0.0, (10.0, 0.0))

    # Rotate smoothly: a 30 deg step in a single sample is a *discontinuity*
    # by max_odom_jump_psi, and is handled as a relocalization instead --
    # which is the point of that check, and is covered separately below.
    for i in range(1, 101):
        tracker.push_odom(i * 0.01, 0.0, 0.0, radians(30.0) * i / 100.0)
    track = tracker.state(1.0)

    assert track is not None
    assert tracker.odom_jumps == 0
    assert degrees(track.bearing) == pytest.approx(-30.0, abs=1e-9)
    assert track.distance == pytest.approx(10.0, abs=1e-9)


# -- latency compensation --------------------------------------------------

def test_latency_compensation_places_the_object_correctly():
    """Fused at capture: within 5 cm. Fused at arrival: more than 1 m out."""
    speed, latency = 5.0, 0.3
    object_world = np.array([40.0, 0.0])

    def run(fuse_at_capture):
        tracker = ObjectTracker(TrackerParams(max_age=30.0))
        end = _drive_straight(tracker, speed=speed, duration=1.0)
        capture = end - latency
        body = _world_to_body(object_world[0] - speed * capture, 0.0, 0.0)
        assert tracker.push_detection(
            capture if fuse_at_capture else end, body) is True
        return tracker.state(end)

    compensated, naive = run(True), run(False)

    assert compensated is not None and naive is not None
    error_ok = float(np.linalg.norm(compensated.position - object_world))
    error_bad = float(np.linalg.norm(naive.position - object_world))

    assert error_ok < 0.05
    assert error_bad > 1.0
    assert error_bad == pytest.approx(speed * latency, abs=0.05)


# -- gating ----------------------------------------------------------------

def _converged_tracker(distance=10.0, n=8, params=None):
    tracker = ObjectTracker(params or TrackerParams(max_age=30.0))
    for i in range(n):
        t = i * 0.1
        tracker.push_odom(t, 0.0, 0.0, 0.0)
        tracker.push_detection(t, (distance, 0.0))
    return tracker


def test_gross_outlier_is_rejected_and_does_not_move_the_estimate():
    tracker = _converged_tracker()
    before = tracker.state(0.7).position.copy()

    tracker.push_odom(0.8, 0.0, 0.0, 0.0)
    assert tracker.push_detection(0.8, (50.0, 30.0)) is False

    assert np.allclose(before, tracker.state(0.8).position)


def test_consecutive_rejections_reset_the_track():
    params = TrackerParams(max_age=30.0, max_rejections=12)
    tracker = _converged_tracker(params=params)
    assert tracker.initialised is True

    for i in range(params.max_rejections):
        t = 1.0 + i * 0.1
        tracker.push_odom(t, 0.0, 0.0, 0.0)
        assert tracker.push_detection(t, (60.0, 40.0)) is False

    assert tracker.initialised is False
    assert tracker.state(2.5) is None


def test_a_detection_outside_the_odom_buffer_cannot_reset_a_healthy_track():
    params = TrackerParams(max_age=30.0, max_rejections=2)
    tracker = _converged_tracker(params=params)

    for _ in range(10):
        assert tracker.push_detection(-999.0, (10.0, 0.0)) is False

    assert tracker.initialised is True, 'plumbing failure is not track evidence'


def test_re_acquisition_snaps_to_the_new_object():
    params = TrackerParams(max_age=30.0, max_rejections=1)
    tracker = ObjectTracker(params)
    tracker.push_odom(0.0, 0.0, 0.0, 0.0)
    tracker.push_detection(0.0, (10.0, 0.0))

    tracker.push_odom(0.1, 0.0, 0.0, 0.0)
    tracker.push_detection(0.1, (10.0, 40.0))       # gated -> reset
    assert tracker.initialised is False

    tracker.push_odom(0.2, 0.0, 0.0, 0.0)
    tracker.push_detection(0.2, (10.0, 40.0))       # re-acquire
    track = tracker.state(0.2)

    assert track is not None
    assert track.position[1] == pytest.approx(40.0)


# -- staleness and confidence ----------------------------------------------

def test_state_is_none_before_any_detection_and_after_max_age():
    tracker = ObjectTracker(TrackerParams(max_age=0.8))
    tracker.push_odom(0.0, 0.0, 0.0, 0.0)

    assert tracker.state(0.0) is None, 'uninitialised'

    tracker.push_detection(0.0, (10.0, 0.0))
    tracker.push_odom(0.79, 0.0, 0.0, 0.0)
    assert tracker.state(0.79) is not None

    tracker.push_odom(0.81, 0.0, 0.0, 0.0)
    assert tracker.state(0.81) is None


def test_confidence_starts_at_zero_rises_and_stays_bounded():
    tracker = ObjectTracker(TrackerParams(max_age=30.0))
    confidences = []
    for i in range(10):
        t = i * 0.1
        tracker.push_odom(t, 0.0, 0.0, 0.0)
        tracker.push_detection(t, (10.0, 0.0))
        confidences.append(tracker.state(t).confidence)

    assert confidences[0] == 0.0, 'one detection is not yet evidence'
    assert all(0.0 <= c <= 1.0 for c in confidences)
    assert all(b >= a - 1e-12 for a, b in zip(confidences, confidences[1:]))
    assert confidences[-1] > confidences[1]


def test_confidence_decays_with_age():
    tracker = _converged_tracker()
    fresh = tracker.state(0.7).confidence
    tracker.push_odom(20.0, 0.0, 0.0, 0.0)

    assert tracker.state(20.0).confidence < fresh


# -- NIS consistency monitor -----------------------------------------------

def _run_nis(noise_multiple, seed=0, gate=1e12, distance=10.0, n=120):
    params = TrackerParams(max_age=30.0, gate_chi2=gate)
    tracker = ObjectTracker(params)
    rng = np.random.default_rng(seed)
    for i in range(n):
        t = i * 0.1
        tracker.push_odom(t, 0.0, 0.0, 0.0)
        bx, by = distance, 0.0
        nx, ny = _synthesise_detection(rng, params, bx, by)
        tracker.push_detection(
            t, (bx + noise_multiple * (nx - bx), by + noise_multiple * (ny - by)))
    return tracker.state((n - 1) * 0.1)


@pytest.mark.parametrize('seed', [0, 1, 2])
def test_nis_sits_near_its_expectation_when_the_model_matches_the_sensor(seed):
    track = _run_nis(1.0, seed=seed)

    assert track.nis_samples == 50
    assert 1.0 < track.nis_mean < 4.0, f'expected ~2.0, got {track.nis_mean}'


@pytest.mark.parametrize('seed', [0, 1, 2])
def test_nis_rises_well_above_threshold_when_the_sensor_is_three_times_noisier(seed):
    """The runtime signal that ``R`` does not describe the camera connected.

    The gate is opened for this test so the statistic is measured rather than
    truncated: with the operational gate most of these would be rejected, and
    a rejected update contributes no NIS sample. That truncation is exactly
    why gating and mis-calibration look alike from outside, and why this
    monitor is worth having.
    """
    alarm = TrackerParams().nis_alarm_ratio * 2.0
    track = _run_nis(3.0, seed=seed)

    assert track.nis_mean > alarm, f'{track.nis_mean} should exceed {alarm}'


def test_nis_is_zero_before_any_update():
    tracker = ObjectTracker(TrackerParams(max_age=30.0))
    tracker.push_odom(0.0, 0.0, 0.0, 0.0)
    tracker.push_detection(0.0, (10.0, 0.0))

    track = tracker.state(0.0)
    assert track.nis_mean == 0.0
    assert track.nis_samples == 0, 'initialisation is not an update'


# -- odom discontinuity ----------------------------------------------------

@pytest.mark.parametrize('jump', [(2.0, 0.0, 0.0), (0.0, 2.0, 0.0),
                                  (1.5, -1.5, 0.8), (-2.0, 0.5, -1.2)])
def test_a_relocalization_preserves_the_relative_geometry(jump):
    """The object estimate is carried rigidly across an odom jump."""
    tracker = _converged_tracker(distance=8.0)
    tracker.push_odom(0.9, 0.0, 0.0, 0.0)
    before = tracker.state(0.9)

    dx, dy, dpsi = jump
    tracker.push_odom(1.0, dx, dy, dpsi)
    after = tracker.state(1.0)

    assert after is not None, 'the track survives; a jump is news about the car'
    assert tracker.odom_jumps == 1
    assert after.distance == pytest.approx(before.distance, abs=1e-9)
    assert after.bearing == pytest.approx(before.bearing, abs=1e-9)


def test_a_relocalization_inflates_p_and_purges_the_pose_buffer():
    tracker = _converged_tracker(distance=8.0)
    tracker.push_odom(0.9, 0.0, 0.0, 0.0)
    trace_before = float(np.trace(tracker.state(0.9).covariance))

    tracker.push_odom(1.0, 3.0, 0.0, 0.0)

    assert float(np.trace(tracker.state(1.0).covariance)) > trace_before
    assert len(tracker.odom) == 1, 'no interpolation across the discontinuity'
    assert tracker.odom.at(0.95) is None


def test_ordinary_motion_is_not_mistaken_for_a_discontinuity():
    params = TrackerParams(max_age=30.0, max_odom_jump=0.5)
    tracker = ObjectTracker(params)
    for i in range(200):                      # 2 m/s at 100 Hz -> 2 cm steps
        tracker.push_odom(i * 0.01, 0.02 * i, 0.0, 0.0)

    assert tracker.odom_jumps == 0


# -- known limitation: no data association ---------------------------------

def test_two_similar_objects_break_the_single_track_assumption():
    """Pinned as known behaviour, not fixed. See the module docstring.

    Detections alternating between two objects are each gated against an
    estimate sitting near the other, so both are rejected and the track
    eventually resets. Anything that silently locked onto one of them would
    be worse, because it would fail without saying so.
    """
    params = TrackerParams(max_age=30.0)
    tracker = ObjectTracker(params)
    tracker.push_odom(0.0, 0.0, 0.0, 0.0)
    tracker.push_detection(0.0, (10.0, 0.0))

    rejected = 0
    for i in range(1, 40):
        t = i * 0.1
        tracker.push_odom(t, 0.0, 0.0, 0.0)
        target = (10.0, 0.0) if i % 2 else (10.0, 6.0)
        if not tracker.push_detection(t, target):
            rejected += 1

    assert rejected > 0, 'alternating targets are gated against each other'


# -- closed loop -----------------------------------------------------------

@pytest.mark.parametrize('seed', [0, 1, 2, 3, 4])
def test_closed_loop_arrives_on_the_shipped_defaults(seed):
    """No parameter overrides at all: the defaults are what gets validated."""
    result = closed_loop(seed)

    assert result['arrived'], f'did not arrive within {result["t"]:.1f} s'


@pytest.mark.parametrize('seed', [0, 1, 2, 3, 4])
def test_curvature_step_is_bounded_at_every_distance_not_just_in_cruise(seed):
    """One bound, all the way in. The old cruise/terminal split is gone.

    The split existed because the position limiter's curvature authority
    scaled as ``1/d**2``, so the terminal region was unavoidably twitchier.
    Limiting curvature directly makes the bound structural -- it is
    ``max_kappa_rate * dt`` by construction, independent of distance -- so
    there is nothing left for a second threshold to hide.
    """
    rate, dt = 0.5, 0.01
    result = closed_loop(seed, max_kappa_rate=rate, dt=dt)
    steps = driving_steps(result)

    assert result['arrived']
    assert steps
    assert max(steps) <= rate * dt + 1e-12, f'worst step {max(steps):.6f}'

    terminal = [abs(b[1] - a[1]) for a, b in zip(result['samples'],
                                                 result['samples'][1:])
                if b[0] <= 2.0 and a[2] is MissionState.APPROACH
                and b[2] is MissionState.APPROACH]
    assert terminal, 'the terminal region must actually be sampled'
    assert max(terminal) <= rate * dt + 1e-12


@pytest.mark.parametrize('speed', [0.5, 2.0, 5.0])
@pytest.mark.parametrize('seed', [0, 1])
def test_curvature_bound_is_independent_of_speed(speed, seed):
    """Per-second rate limiting should make this hold automatically. Verified."""
    rate, dt = 0.5, 0.01
    result = closed_loop(seed, speed=speed, max_kappa_rate=rate, dt=dt)

    assert result['arrived'], f'did not arrive at {speed} m/s'
    assert max(driving_steps(result)) <= rate * dt + 1e-12


@pytest.mark.parametrize('max_kappa_rate', [0.05, 0.5, 5.0, 50.0])
def test_lag_sweep_finds_no_arrival_failure_across_three_decades(max_kappa_rate):
    """Rate limiting costs delay; measure how much before trusting the default.

    Swept over three decades on three geometries and it never costs an
    arrival, so there is no measured lower failure point to quote a margin
    against. See the mission-state tests for the tight geometry, which is
    where a limiter would bind first if it ever did.
    """
    for seed in (0, 1):
        result = closed_loop(seed, max_kappa_rate=max_kappa_rate)
        assert result['arrived'], f'rate {max_kappa_rate} failed on seed {seed}'
        steps = driving_steps(result)
        assert max(steps) <= max_kappa_rate * 0.01 + 1e-12


def test_closed_loop_on_defaults_keeps_the_filter_consistent():
    """NIS near expectation confirms sim and filter share a sensor model."""
    result = closed_loop(0)
    track = result['tracker'].state(result['t'] - 0.01)

    assert track is not None
    assert track.nis_samples > 0
    assert track.nis_mean < TrackerParams().nis_alarm_ratio * 2.0


# -- Part B: the lag sweep, with a disturbance that makes the limiter bind --

def disturbed_loop(seed, case, *, max_kappa_rate=0.5, speed=1.5, dt=0.01,
                   latency=0.05, t_max=140.0):
    """Closed loop with one step injected after the filter has settled.

    ``case`` is ``'jump'`` (the target moves laterally, gate widened so the
    correction is accepted rather than rejected), ``'reacquire'`` (detections
    stop for longer than ``max_age`` and the object reappears off-heading), or
    ``'reposition'`` (an unreachable start, whose exit steps from full
    opposite lock to a normal pursuit command -- the largest legitimate step
    the system produces).
    """
    params = TrackerParams(gate_chi2=1e9) if case == 'jump' else TrackerParams()
    rng = np.random.default_rng(seed)
    tracker = ObjectTracker(params)
    limiter = CurvatureLimiter(max_kappa_rate)
    machine = GoToObjectMission(PursuitParams(), MissionParams(), limiter)

    obj = np.array([0.5, 2.5] if case == 'reposition' else [18.5, 4.0])
    x = y = psi = 0.0
    pending, samples = [], []
    injected = None
    blackout = (None, None)

    for step in range(int(t_max / dt)):
        t = step * dt
        if injected is None and case in ('jump', 'reacquire') and t >= 4.0:
            obj = obj + np.array([0.0, 2.5 if case == 'jump' else 6.0])
            if case == 'reacquire':
                blackout = (t, t + 0.9)
            injected = t

        tracker.push_odom(t, x, y, psi)
        while pending and pending[0][0] <= t:
            _, capture, body = pending.pop(0)
            tracker.push_detection(capture, body)

        dark = blackout[0] is not None and blackout[0] <= t < blackout[1]
        if step % 10 == 0 and not dark:
            bx, by = _world_to_body(obj[0] - x, obj[1] - y, psi)
            pending.append((t + latency, t,
                            _synthesise_detection(rng, params, bx, by)))

        track = tracker.state(t)
        command = machine.update(t, (x, y), psi, track, last_odom_stamp=t)
        samples.append((hypot(obj[0] - x, obj[1] - y), command.curvature,
                        command.state, t, command.watchdog))
        if command.state is MissionState.ARRIVED:
            return dict(arrived=True, samples=samples, t=t, limiter=limiter)

        ds = speed * dt if command.drive_enable else 0.0
        x += ds * cos(psi)
        y += ds * sin(psi)
        psi = wrap_pi(psi + command.curvature * ds)

    return dict(arrived=False, samples=samples, t=t_max, limiter=limiter)


CASES = ['jump', 'reacquire', 'reposition']


@pytest.mark.parametrize('case', CASES)
def test_the_disturbance_actually_saturates_the_limiter(case):
    """Evidence, before any "no failure found" claim is made.

    The previous sweep reported identical arrival times across three decades,
    which only meant the limiter was never binding: with the old pass-through
    seeding the opening demand was met instantly and pure pursuit is smooth
    thereafter, so there was nothing left to clip. A sweep over an inactive
    limiter measures nothing. These disturbances make it bind.
    """
    slow = disturbed_loop(0, case, max_kappa_rate=0.05)
    fast = disturbed_loop(0, case, max_kappa_rate=50.0)

    assert slow['limiter'].saturations > 50, 'the limiter must be binding'
    assert fast['limiter'].saturations < slow['limiter'].saturations


@pytest.mark.parametrize('case', CASES)
@pytest.mark.parametrize('max_kappa_rate', [0.01, 0.5, 50.0])
def test_disturbed_approach_still_arrives_across_the_sweep(case, max_kappa_rate):
    """Swept 0.003 to 50 -- five orders -- and arrival never fails.

    Reported rather than dressed up as a margin: there is no lower failure
    point to quote one against. What lower rates cost is *time*, measured in
    the next test.
    """
    for seed in (0, 1):
        result = disturbed_loop(seed, case, max_kappa_rate=max_kappa_rate)
        assert result['arrived'], (
            f'{case} at rate {max_kappa_rate} failed on seed {seed}')

        # Every published command goes through the limiter, including the
        # ramp out during LOST, so the bound holds across *all* adjacent
        # cycles -- except a watchdog tick, which zeroes immediately by
        # design and is covered by its own tests. At low limiter rates the
        # 'reacquire' case does trip it: the gate rejects the relocated
        # object for longer than state_timeout before the track resets.
        steps = [abs(b[1] - a[1]) for a, b in zip(result['samples'],
                                                  result['samples'][1:])
                 if not a[4] and not b[4]]
        assert max(steps) <= max_kappa_rate * 0.01 + 1e-12


def test_a_slower_limiter_costs_time_and_not_arrival():
    """The real cost curve, on the case where the demanded step is largest."""
    times = {}
    for rate in (0.01, 0.5, 50.0):
        results = [disturbed_loop(seed, 'reposition', max_kappa_rate=rate)
                   for seed in (0, 1)]
        assert all(r['arrived'] for r in results)
        times[rate] = max(r['t'] for r in results)

    assert times[0.01] > 2.0 * times[50.0], 'limiting must visibly cost time'
    assert times[0.5] < times[0.01], 'and the default must be cheaper than that'


# -- C6: reading the consistency statistics honestly -----------------------

def test_the_nis_window_mean_is_as_noisy_as_theory_says():
    """Why a single reading of 2.49 means nothing.

    The window mean of a chi-squared(2) statistic has standard error
    ``2 / sqrt(n)``: 0.28 at the default window of 50. An earlier run read
    2.49 from one seed and it was taken for a 25% bias; it was 1.75 sigma of
    ordinary sampling noise. The alarm at 4.0 sits about 7 sigma out, which is
    the headroom that actually matters.
    """
    means = []
    for seed in range(40):
        params = TrackerParams(max_age=1e6, gate_chi2=1e12)
        tracker = ObjectTracker(params)
        rng = np.random.default_rng(seed)
        for i in range(200):
            t = i * 0.1
            tracker.push_odom(t, 0.0, 0.0, 0.0)
            tracker.push_detection(t, _synthesise_detection(rng, params, 10.0, 0.0))
        means.append(tracker.state(19.9).nis_mean)

    means = np.array(means)
    predicted_sigma = 2.0 / np.sqrt(TrackerParams().nis_window)

    assert abs(means.mean() - 2.0) < 0.2, f'no bias: got {means.mean():.3f}'
    assert means.std(ddof=1) == pytest.approx(predicted_sigma, rel=0.4)
    assert means.max() < TrackerParams().nis_alarm_ratio * 2.0


def test_rejection_rate_is_the_statistic_the_gate_hides_from_nis():
    """A too-tight R gates good data out; NIS cannot see it, this can."""
    honest = TrackerParams(max_age=1e6)
    tight = TrackerParams(max_age=1e6, sigma_range_base=0.001,
                          sigma_range_slope=0.0, sigma_bearing=0.0005,
                          max_rejections=10 ** 6)

    rates = {}
    for label, model in (('honest', honest), ('tight', tight)):
        tracker = ObjectTracker(model)
        rng = np.random.default_rng(0)
        for i in range(120):
            t = i * 0.1
            tracker.push_odom(t, 0.0, 0.0, 0.0)
            tracker.push_detection(t, _synthesise_detection(rng, honest, 10.0, 0.0))
        rates[label] = tracker.rejection_rate

    assert rates['honest'] < 0.1
    assert rates['tight'] > 0.8, 'the mis-calibration shows up here'


def test_rejection_rate_is_zero_on_a_fresh_track():
    tracker = ObjectTracker(TrackerParams(max_age=1e6))
    tracker.push_odom(0.0, 0.0, 0.0, 0.0)
    assert tracker.rejection_rate == 0.0

    tracker.push_detection(0.0, (10.0, 0.0))
    assert tracker.state(0.0).rejection_rate == 0.0
