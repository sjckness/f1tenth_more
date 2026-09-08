"""steering_offset_calibration_node.py / steering_offset_fit.py tests.

Same convention as test_gyro_bias_calibration_node.py: pure functions are
tested directly with no rclpy at all, and the node-level tests construct a
real (never spun) Node via parameter_overrides and call its methods directly
-- no live topics, no hardware, no executor.

Covers, grouped by what each group protects:

  - The FIT itself, against synthetic data with a known answer. This is the
    only way to catch a sign error in the Gauss-Newton step, which is exactly
    the bug that was in the first version of the solver: the gates all still
    fired, so a gate-only test suite would have called it green while the
    solver walked uphill into its L_eff clamp.
  - Each of the three refuse-to-write gates, each against data built to fail
    that gate specifically and pass the others.
  - The provenance writer, whose two real bugs were both found by running it
    against the actual steering_calibration.yaml rather than a toy file:
    destroying a human-written comment, and stacking one block per run.
  - The servo-offset conversion, which is where a sign error would silently
    apply the calibration backwards -- doubling the error instead of removing
    it, with no other symptom.
  - The mux lane ordering, which is a genuine coupling to mux.yaml (see
    CLAUDE.md on config defaults encoded in tests): if someone renumbers the
    lanes, a calibration drive could end up outranking the joystick.

Run standalone: python3 -m pytest test/test_steering_offset_calibration_node.py -v
"""
import inspect
import math
import os
import random
import shutil
import time

import pytest
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.qos import DurabilityPolicy
import yaml as pyyaml

from f1tenth_diagnostics import steering_offset_fit as fitlib
from f1tenth_diagnostics.calibration_common import (
    strip_provenance_block,
    write_yaml_config_with_provenance,
)
from f1tenth_diagnostics.steering_offset_calibration_node import (
    EXIT_GATES_REFUSED,
    FULL_SEGMENT_PLAN,
    SteeringOffsetCalibrationNode,
    wrap_angle,
    yaw_from_quaternion,
)

TRUE_GAIN = 0.82
TRUE_DELTA0 = math.radians(1.4)
TRUE_BACKLASH = math.radians(2.0)
PINNED_L = fitlib.PINNED_WHEELBASE_M


class _Logger:
    """Stand-in for a node logger for the pure-function writer tests."""

    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(('info', message))

    def warning(self, message):
        self.messages.append(('warning', message))

    def error(self, message):
        self.messages.append(('error', message))


def _synthetic_samples(repetitions=3, noise=0.0015, seed=7, amplitude_deg=10.0,
                       one_sided=False, n_pose=5):
    """Segments generated from the exact model the fit inverts, so the fit has
    a known right answer. Sign alternates per repetition, mirroring
    _segment_spec's own rep_sign."""
    rng = random.Random(seed)
    amplitude = math.radians(amplitude_deg)
    out = []
    for rep in range(repetitions):
        rep_sign = 1.0 if rep % 2 == 0 else -1.0
        plan = ((0, 0.0, 0.6), (1, +1.0, 0.6), (2, -1.0, 1.2), (3, +1.0, 0.6),
                (4, 0.0, 0.6))
        for seg, sign, length in plan:
            delta = sign * rep_sign * amplitude
            if one_sided:
                delta = abs(delta)
            dpsi = (math.tan(TRUE_GAIN * delta + TRUE_DELTA0) / PINNED_L * length +
                    rng.gauss(0.0, noise))
            out.append(fitlib.Sample(rep, seg, delta, dpsi, length, n_pose,
                                     ds_chord=length * 0.995))
    return out


# --------------------------------------------------------------------- fit
def test_fit_recovers_known_gain_and_offset_from_synthetic_drive():
    samples = _synthetic_samples()
    fit = fitlib.fit_gain_offset(samples)
    assert fit.converged
    assert fit.gain == pytest.approx(TRUE_GAIN, abs=0.01)
    assert fit.delta0 == pytest.approx(TRUE_DELTA0, abs=math.radians(0.1))
    assert fit.wheelbase == PINNED_L


def test_gain_and_offset_stay_separable_where_gain_and_wheelbase_would_not():
    """The whole reason L is pinned. g and delta0 are separable because g
    multiplies delta_cmd and delta0 does not -- so the correlation stays low
    on the standard S-curve, whose straight segments pin the offset directly.
    A (g, L) pair on the same data would be collinear by construction."""
    fit = fitlib.fit_gain_offset(_synthetic_samples())
    assert abs(fit.correlation) < 0.5, fit.summary()


def test_fit_residual_stays_at_the_noise_floor_so_an_uphill_step_is_caught():
    """The Gauss-Newton step must be ADDED, not subtracted. With the sign
    wrong the solver drives L_eff into its lower clamp and leaves a residual
    of order 100 rad -- while still reporting a finite delta0 and a plausible
    correlation. Asserting on the residual is what distinguishes the two."""
    fit = fitlib.fit_gain_offset(_synthetic_samples(noise=0.0015))
    assert fit.residual_rms < 0.01


def test_fit_refuses_fewer_samples_than_free_parameters_plus_one():
    with pytest.raises(ValueError):
        fitlib.fit_gain_offset(_synthetic_samples()[:2])


# ------------------------------------------------------------------- gates
def test_conditioning_gate_passes_a_two_sided_drive():
    samples = _synthetic_samples()
    gate = fitlib.check_conditioning(samples, fitlib.fit_gain_offset(samples))
    assert gate.passed, gate.detail


def test_conditioning_gate_refuses_a_drive_that_never_steered_the_other_way():
    """The collinearity case the work order names. Note this must be caught by
    the SIGN check, not the correlation alone: on this data the correlation
    comes out around 0.91, under the 0.95 limit, so a correlation-only gate
    would let a one-sided drive through."""
    samples = _synthetic_samples(one_sided=True)
    gate = fitlib.check_conditioning(samples, fitlib.fit_gain_offset(samples))
    assert not gate.passed
    assert 'both signs' in gate.detail


def test_repetition_agreement_gate_passes_a_genuinely_constant_offset():
    samples = _synthetic_samples(noise=0.0003)
    fit = fitlib.fit_gain_offset(samples)
    assert fitlib.check_repetition_agreement(samples, fit.gain, fit.wheelbase).passed


def test_repetition_agreement_gate_refuses_an_offset_that_moved_between_runs():
    samples = _synthetic_samples(noise=0.0003)
    for sample in samples:
        if sample.repetition == 1:
            sample.dpsi += 0.02
    fit = fitlib.fit_gain_offset(samples)
    gate = fitlib.check_repetition_agreement(samples, fit.gain, fit.wheelbase)
    assert not gate.passed
    assert 'not a constant offset' in gate.detail
    # Backlash is named as the leading suspect -- that is the actionable half.
    assert 'Backlash' in gate.detail


def test_repetition_agreement_gate_refuses_when_too_few_repetitions_were_driven():
    samples = [s for s in _synthetic_samples() if s.repetition == 0]
    fit = fitlib.fit_gain_offset(samples)
    assert not fitlib.check_repetition_agreement(
        samples, fit.gain, fit.wheelbase).passed


def test_pose_support_gate_refuses_a_segment_built_from_too_few_slam_fixes():
    samples = _synthetic_samples()
    samples[3].n_pose = 2
    gate = fitlib.check_pose_support(samples, 4)
    assert not gate.passed
    assert 'rep0/seg3=2' in gate.detail


def test_pose_support_gate_passes_when_every_segment_meets_the_floor():
    assert fitlib.check_pose_support(_synthetic_samples(n_pose=5), 4).passed


# ------------------------------------------------------------- raw-sample IO
def test_raw_samples_survive_a_csv_round_trip_so_the_fit_can_be_redone_offline(tmp_path):
    samples = _synthetic_samples()
    path = str(tmp_path / 'samples.csv')
    fitlib.write_samples_csv(path, samples)
    restored = fitlib.read_samples_csv(path)
    assert len(restored) == len(samples)
    original = fitlib.fit_gain_offset(samples)
    replayed = fitlib.fit_gain_offset(restored)
    # 1e-6, not bit-exact: the CSV stores 9 decimal places, so a replayed fit
    # agrees to about that precision rather than identically. That is a
    # property of the dump format and is far tighter than any calibration
    # tolerance -- 1e-6 rad is 6e-5 deg.
    assert replayed.delta0 == pytest.approx(original.delta0, abs=1e-6)
    assert replayed.gain == pytest.approx(original.gain, abs=1e-6)


def test_offline_refit_cli_reports_failure_for_a_run_that_would_be_refused(tmp_path):
    path = str(tmp_path / 'one_sided.csv')
    fitlib.write_samples_csv(path, _synthetic_samples(one_sided=True))
    assert fitlib.main([path]) == 1


# ------------------------------------------------------------ provenance IO
def test_strip_provenance_removes_only_its_own_bracketed_block():
    text = ('# a human wrote this\n'
            '# >>> BEGIN t provenance -- auto-written, edits below are overwritten\n'
            '# fitted delta0 = 1.4\n'
            '# <<< END t provenance\n'
            '# and this')
    assert strip_provenance_block(text, 't') == '# a human wrote this\n# and this'


def test_strip_provenance_leaves_an_unterminated_block_alone_rather_than_eating_the_rest():
    text = ('# >>> BEGIN t provenance -- auto-written, edits below are overwritten\n'
            '# someone deleted the end sentinel by hand\n'
            '# an unrelated human comment that must survive')
    assert strip_provenance_block(text, 't') == text


def _workspace_src(*parts):
    """Anchor off this test file rather than an absolute path: test/ ->
    f1tenth_diagnostics/ -> src/. Same "walk up to a known layout" idea as
    calibration_common.resolve_source_config_path, without needing an
    installed workspace."""
    src_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    return os.path.join(src_dir, *parts)


def _steering_calibration_fixture(tmp_path):
    """A writable copy of the REAL steering_calibration.yaml when this is a
    normal workspace checkout -- the provenance bugs these tests cover were
    both specific to that file's actual comment layout and would not reproduce
    against a toy fixture. Falls back to a minimal inline equivalent (without
    the neighbouring human comment, so the test that needs it skips) if the
    sibling package is not present."""
    target = str(tmp_path / 'steering_calibration.yaml')
    real = _workspace_src('f1tenth_hardware', 'f1tenth_hardware', 'config',
                          'steering_calibration.yaml')
    if os.path.exists(real):
        shutil.copy2(real, target)
        return target
    with open(target, 'w') as handle:
        handle.write(
            '/**:\n'
            '  ros__parameters:\n'
            '    servo_min: 0.15\n'
            '    servo_max: 0.8318\n'
            '    steering_angle_to_servo_offset: 0.4494\n'
            '    steering_angle_to_servo_gain_left: -1.2135\n')
    return target


def test_repeated_writes_leave_exactly_one_provenance_block(tmp_path):
    """Three calibration runs must not leave three stacked blocks -- a reader
    could not tell which one describes the value currently in the file. This
    failed before the BEGIN/END sentinels went in, because ruamel round-trips
    a 'before key' comment back into the PRECEDING key's after-value slot."""
    path = _steering_calibration_fixture(tmp_path)
    for value in (0.479, 0.482, 0.490):
        write_yaml_config_with_provenance(
            path, {'/**': {'steering_angle_to_servo_offset': value}},
            [f'fitted offset {value}'], _Logger(), tool='steering_offset_calibration_node')
    text = open(path).read()
    assert text.count('BEGIN steering_offset_calibration_node provenance') == 1
    assert pyyaml.safe_load(text)['/**']['ros__parameters'][
        'steering_angle_to_servo_offset'] == 0.490


def test_writing_a_key_preserves_human_comments_about_neighbouring_keys(tmp_path):
    """Found live: clearing the whole ruamel comment entry for the patched key
    deleted steering_calibration.yaml's own 'Placeholder until retuned' note,
    which documents two OTHER keys but is parsed into the patched key's
    after-value slot."""
    path = _steering_calibration_fixture(tmp_path)
    before = open(path).read()
    if 'Placeholder until retuned' not in before:
        pytest.skip('fixture without the neighbouring human comment')
    write_yaml_config_with_provenance(
        path, {'/**': {'steering_angle_to_servo_offset': 0.479}},
        ['fitted offset 0.479'], _Logger(), tool='steering_offset_calibration_node')
    assert 'Placeholder until retuned' in open(path).read()


def test_write_leaves_a_backup_and_a_file_that_still_parses(tmp_path):
    path = _steering_calibration_fixture(tmp_path)
    backup, diff = write_yaml_config_with_provenance(
        path, {'/**': {'steering_angle_to_servo_offset': 0.479}},
        ['fitted offset 0.479'], _Logger(), tool='steering_offset_calibration_node')
    assert os.path.exists(backup)
    assert 'steering_angle_to_servo_offset' in diff
    loaded = pyyaml.safe_load(open(path))['/**']['ros__parameters']
    assert loaded['steering_angle_to_servo_offset'] == 0.479
    assert loaded['servo_min'] == 0.15


# -------------------------------------------------------------- node-level
@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _construct(param_dict=None):
    from rclpy.parameter import Parameter
    overrides = [Parameter(k, value=v) for k, v in (param_dict or {}).items()]
    return SteeringOffsetCalibrationNode(parameter_overrides=overrides)


def test_write_back_math_reduces_to_the_old_offset_only_formula_at_unit_gain():
    """gain_new = gain/g and offset_new = offset - delta0*gain_new must reduce
    to the pre-gain formula (offset - gain*delta0) at g = 1. If it does not,
    adding gain silently changed the behaviour of every offset-only
    calibration that came before it."""
    node = _construct()
    try:
        gain_new, offset_new = node._new_gain_and_offset(
            1.0, math.radians(1.4), -1.2135, 0.4494)
        assert gain_new == pytest.approx(-1.2135, abs=1e-12)
        assert offset_new == pytest.approx(0.4494 - (-1.2135) * math.radians(1.4),
                                           abs=1e-12)
    finally:
        node.destroy_node()


def test_write_back_math_makes_the_commanded_angle_the_achieved_angle():
    """The identity the whole derivation exists for: under the new constants,
    commanding theta must put the wheels at theta. Verified against the real
    servo relation servo = gain*theta + offset (ackermann_to_vesc.cpp:142) and
    the composed error delta_actual = g*theta + delta0."""
    node = _construct()
    try:
        g, delta0 = 0.82, math.radians(1.4)
        gain_c, off_c = -1.2135, 0.4494
        gain_n, off_n = node._new_gain_and_offset(g, delta0, gain_c, off_c)
        for theta in (math.radians(-12.0), 0.0, math.radians(9.0)):
            servo = gain_n * theta + off_n
            # What the hardware actually does with that servo value, expressed
            # through the OLD constants plus the measured error.
            achieved = g * ((servo - off_c) / gain_c) + delta0
            assert achieved == pytest.approx(theta, abs=1e-12)
    finally:
        node.destroy_node()


def test_write_back_refuses_a_degenerate_gain():
    node = _construct()
    try:
        with pytest.raises(ValueError):
            node._new_gain_and_offset(0.0, 0.0, -1.2135, 0.4494)
    finally:
        node.destroy_node()


def test_servo_extremes_are_checked_at_both_ends_of_an_asymmetric_range():
    """A changed gain rescales the whole range, so checking the centre or one
    end is not enough -- a wrong gain can drive the servo past its mechanical
    stop at the far end while the centre still looks fine. This car's limits
    are asymmetric (-0.264 / +0.314 rad), so the two ends are not mirrors."""
    node = _construct()
    try:
        at_min, at_max = node._servo_extremes(-1.2135, 0.4494)
        assert at_min == pytest.approx(-1.2135 * -0.264 + 0.4494)
        assert at_max == pytest.approx(-1.2135 * 0.314 + 0.4494)
        assert at_min != at_max
    finally:
        node.destroy_node()


def test_preflight_refuses_while_a_mission_is_running():
    node = _construct()
    try:
        node._mission_seen = True
        node._mission_state = 'RUNNING'
        assert not node._check_mission_idle()
    finally:
        node.destroy_node()


def test_preflight_refuses_when_a_mission_emergency_stop_is_latched():
    node = _construct()
    try:
        node._mission_seen = True
        node._mission_state = 'IDLE'
        node._mission_estop = True
        assert not node._check_mission_idle()
    finally:
        node.destroy_node()


def test_preflight_allows_an_idle_mission():
    node = _construct()
    try:
        node._mission_seen = True
        node._mission_state = 'IDLE'
        assert node._check_mission_idle()
    finally:
        node.destroy_node()


def test_preflight_refuses_an_amplitude_beyond_this_cars_asymmetric_steering_limits():
    """min_steering_angle (-0.264) and max_steering_angle (+0.314) are not
    symmetric on this car, so an amplitude legal on the max side can still be
    illegal on the min side. The check must apply both bounds."""
    node = _construct({'amplitude_rad': 0.30})
    try:
        assert not node._check_steering_limits()
    finally:
        node.destroy_node()


def test_preflight_allows_the_default_ten_degree_amplitude():
    node = _construct()
    try:
        assert node._check_steering_limits()
    finally:
        node.destroy_node()


def test_preflight_does_not_gate_on_a_pose_a_parked_car_cannot_produce():
    """The regression this whole two-stage preflight exists for.

    slam_toolbox's shouldProcessScan is distance-gated
    (minimum_travel_distance 0.03), so a stationary car publishes no
    /slam/pose no matter how long preflight listens. The old
    _check_pose_source() therefore refused every first run and blamed SLAM
    for it. Nothing evaluated before the car moves may depend on a pose."""
    node = _construct()
    try:
        assert not hasattr(node, '_check_pose_source')
        node._map_seen = True
        node._map_info = (200, 200, 0.05)
        assert node._pose is None
        # Every standstill check still passes with no pose whatsoever.
        assert node._check_slam_map_ready()
    finally:
        node.destroy_node()


def test_readiness_check_refuses_when_slam_toolbox_published_no_map():
    node = _construct()
    try:
        assert not node._map_seen
        assert not node._check_slam_map_ready()
    finally:
        node.destroy_node()


def test_readiness_check_passes_on_a_latched_map_with_the_car_stationary():
    node = _construct()
    try:
        grid = OccupancyGrid()
        grid.info.width, grid.info.height, grid.info.resolution = 384, 384, 0.05
        node._on_map(grid)
        assert node._check_slam_map_ready()
        assert node._pose is None, 'readiness must not require any motion'
    finally:
        node.destroy_node()


def test_map_subscription_is_transient_local_so_a_latched_map_is_not_missed():
    """slam_toolbox latches the map. A VOLATILE subscription would only see a
    map published after this node started, which would make the readiness
    check as unsatisfiable as the pose check it replaced."""
    node = _construct()
    try:
        matching = [s for s in node.get_subscriptions_info_by_topic(node.map_topic)
                    if s.node_name == node.get_name()]
        assert matching, 'no subscription on the map topic'
        assert (matching[0].qos_profile.durability ==
                DurabilityPolicy.TRANSIENT_LOCAL)
    finally:
        node.destroy_node()


def test_nudge_refuses_when_the_creep_produces_no_pose():
    """No pose after moving means something between the mux lane and the scan
    match is broken. Refusing here costs 0.18 m; not refusing costs 3.6 m of
    open-loop drive fitted against nothing."""
    node = _construct({'nudge_distance_m': 0.05, 'nudge_speed_mps': 0.5,
                       'nudge_pose_timeout_sec': 0.2})
    try:
        assert not node._nudge_and_confirm_pose()
        assert node._phase == 'preflight'
    finally:
        node.destroy_node()


def test_nudge_passes_once_a_pose_arrives_and_leaves_the_car_stopped():
    node = _construct({'nudge_distance_m': 0.05, 'nudge_speed_mps': 0.5,
                       'nudge_pose_timeout_sec': 0.2})
    sent = []
    node._publish_drive = lambda steering, speed: (
        sent.append((steering, speed)),
        # One pose lands as soon as the car has actually been commanded to
        # move -- exactly what slam_toolbox does once past its travel gate.
        node.__setattr__('_pose_count', node._pose_count + 1)
        if speed > 0.0 and node._pose_count == 0 else None)
    try:
        assert node._nudge_and_confirm_pose()
        assert any(speed > 0.0 for _, speed in sent), 'the nudge never commanded motion'
        assert all(steering == 0.0 for steering, _ in sent), 'the nudge must steer straight'
        assert sent[-1] == (0.0, 0.0), 'the nudge must leave the car commanded to zero'
        assert node._phase == 'preflight'
    finally:
        node.destroy_node()


def test_nudge_is_distance_bounded_by_its_commanded_speed():
    """The creep must stop on its own bound rather than running until a pose
    happens to show up: it is the one part of preflight that moves the car."""
    node = _construct({'nudge_distance_m': 0.1, 'nudge_speed_mps': 0.1,
                       'nudge_pose_timeout_sec': 0.1, 'publish_rate_hz': 50.0})
    try:
        started = time.monotonic()
        assert not node._nudge_and_confirm_pose()
        elapsed = time.monotonic() - started
        nominal = node.nudge_distance_m / node.nudge_speed_mps
        # 1.5x the nominal duration for the creep, plus the post-creep wait.
        assert elapsed < 1.5 * nominal + node.nudge_pose_timeout_sec + 1.0
    finally:
        node.destroy_node()


def test_nudge_stops_immediately_when_the_signal_handler_sets_done():
    """main() installs the SIGINT/SIGTERM handler BEFORE preflight is entered,
    and it sets node.done. The nudge loop has to honour that, or Ctrl-C during
    the creep would be ignored until the distance bound expired."""
    node = _construct({'nudge_distance_m': 10.0, 'nudge_speed_mps': 0.1,
                       'nudge_pose_timeout_sec': 0.1})
    node.done = True
    try:
        started = time.monotonic()
        assert not node._nudge_and_confirm_pose()
        assert time.monotonic() - started < 5.0, 'nudge ignored done and ran its bound'
        assert node._phase == 'preflight'
    finally:
        node.destroy_node()


def test_nudge_refuses_when_front_clearance_is_below_the_drive_threshold():
    """The nudge moves, so the drive phase's clearance abort applies to it."""
    node = _construct({'min_front_clearance_m': 0.9})
    node._clearance = 0.2
    node._clearance_stamp = time.monotonic()
    try:
        assert not node._nudge_and_confirm_pose()
    finally:
        node.destroy_node()


def test_static_preflight_touches_neither_slam_nor_clearance():
    """Mode A reads wheel angles off a protractor with the car parked and
    supported. It uses no SLAM and it does not move, so neither the readiness
    check nor the nudge nor the clearance check belongs in its preflight."""
    node = _construct({'calibration_mode': 'static'})
    try:
        node._mission_seen = True
        node._mission_state = 'IDLE'
        assert not node._map_seen and node._pose is None and node._clearance is None
        assert node._check_sweep_limits()
        assert node._check_mission_idle()
        source = inspect.getsource(node.run_static_preflight)
        for forbidden in ('_check_slam_map_ready', '_nudge_and_confirm_pose',
                          '_check_pose_source', 'clearance'):
            assert forbidden not in source, f'mode A preflight still references {forbidden}'
    finally:
        node.destroy_node()


def test_tick_stays_inert_while_the_nudge_owns_the_drive_lane():
    """The nudge publishes from its own loop. If the segment-plan timer also
    ran during 'nudge' the two would fight over the drive topic and the first
    segment would start before preflight had finished."""
    node = _construct()
    sent = []
    node._publish_drive = lambda steering, speed: sent.append((steering, speed))
    try:
        node._phase = 'nudge'
        node._tick()
        assert sent == []
    finally:
        node.destroy_node()


def _fit_result(gain, delta0):
    return fitlib.FitResult(gain=gain, delta0=delta0, ci_gain=0.001, ci_delta0=0.001,
                            wheelbase=fitlib.PINNED_WHEELBASE_M, residual_rms=0.001,
                            correlation=0.1, n_samples=15, converged=True, iterations=4)


def test_plausibility_gate_refuses_a_mechanically_impossible_offset():
    node = _construct()
    try:
        assert not node._plausibility_gate(
            _fit_result(1.0, math.radians(25.0))).passed
    finally:
        node.destroy_node()


def test_plausibility_gate_refuses_an_implausible_gain():
    node = _construct()
    try:
        assert not node._plausibility_gate(
            _fit_result(4.0, math.radians(1.0))).passed
        assert not node._plausibility_gate(
            _fit_result(0.1, math.radians(1.0))).passed
    finally:
        node.destroy_node()


def test_plausibility_gate_passes_a_realistic_result():
    node = _construct()
    try:
        assert node._plausibility_gate(
            _fit_result(0.82, math.radians(1.4))).passed
    finally:
        node.destroy_node()


def test_a_refused_run_writes_no_config_but_still_dumps_its_raw_samples(tmp_path):
    """The two halves of the refusal contract: change nothing, and still leave
    enough on disk to redo the fit offline without re-driving the car."""
    node = _construct({'raw_dump_dir': str(tmp_path), 'write_enabled': True})
    try:
        node.samples = _synthetic_samples(one_sided=True)
        node.finish()
        assert node.exit_code == EXIT_GATES_REFUSED
        dumps = list(tmp_path.glob('*.csv'))
        assert len(dumps) == 1
        assert len(fitlib.read_samples_csv(str(dumps[0]))) == len(node.samples)
    finally:
        node.destroy_node()


def test_a_dry_run_passes_its_gates_without_touching_any_config(tmp_path):
    node = _construct({'raw_dump_dir': str(tmp_path), 'write_enabled': False})
    try:
        node.samples = _synthetic_samples()
        node.finish()
        assert node.exit_code == 0
        assert node.done
    finally:
        node.destroy_node()


def test_segment_plan_alternates_the_starting_sign_between_repetitions():
    """This is what makes the conditioning gate satisfiable at all -- without
    the alternation every repetition steers the same way first and the fit
    cannot separate delta0 from L_eff."""
    node = _construct()
    try:
        _, first_rep = node._segment_spec(0, 1)
        _, second_rep = node._segment_spec(1, 1)
        assert first_rep > 0.0 > second_rep
    finally:
        node.destroy_node()


def test_short_plan_is_the_default_and_is_three_segments():
    node = _construct()
    try:
        assert node.segment_plan_name == 'short'
        assert len(node.segment_plan) == 3
        assert node._check_segment_plan()
    finally:
        node.destroy_node()


def test_short_plan_still_gives_the_conditioning_gate_what_it_needs():
    """Three segments per repetition is only acceptable if the fit is still
    identifiable: at least one near-zero segment to pin delta0 directly, and
    >= 2 segments of EACH steering sign across the run. Cutting the profile
    down is worthless if it cuts the gate's inputs below its thresholds."""
    node = _construct({'segment_plan': 'short', 'repetitions': 3})
    try:
        deltas = [node._segment_spec(rep, seg)[1]
                  for rep in range(node.repetitions)
                  for seg in range(len(node.segment_plan))]
        assert sum(1 for d in deltas if d > fitlib.DEFAULT_MIN_ABS_DELTA_RAD) >= \
            fitlib.DEFAULT_MIN_SIGN_SAMPLES
        assert sum(1 for d in deltas if d < -fitlib.DEFAULT_MIN_ABS_DELTA_RAD) >= \
            fitlib.DEFAULT_MIN_SIGN_SAMPLES
        assert sum(1 for d in deltas if abs(d) < fitlib.DEFAULT_MIN_ABS_DELTA_RAD) >= 1
        # >= 3 samples, or fit_gain_offset has no residual variance at all.
        assert len(deltas) >= 3
    finally:
        node.destroy_node()


def test_short_plan_needs_less_space_than_the_full_one():
    short = _construct({'segment_plan': 'short'})
    full = _construct({'segment_plan': 'full'})
    try:
        per_rep = lambda n: sum(getattr(n, key) for key, _ in n.segment_plan)  # noqa: E731
        assert per_rep(short) < per_rep(full)
        assert per_rep(short) == pytest.approx(2.4)
        assert per_rep(full) == pytest.approx(3.6)
    finally:
        short.destroy_node()
        full.destroy_node()


def test_full_plan_is_still_selectable_and_unchanged():
    node = _construct({'segment_plan': 'full'})
    try:
        assert node.segment_plan == FULL_SEGMENT_PLAN
        assert len(node.segment_plan) == 5
        assert node._check_segment_plan()
    finally:
        node.destroy_node()


def test_preflight_refuses_an_unknown_segment_plan_instead_of_driving_another():
    """Falling back to a default would drive a footprint the operator never
    cleared space for, and the only symptom would be geometry in the log that
    does not match what they asked for."""
    node = _construct({'segment_plan': 'figure-eight'})
    try:
        assert not node._check_segment_plan()
    finally:
        node.destroy_node()


def test_segment_advance_follows_the_selected_plan_length():
    """_tick advances on len(self.segment_plan), not the module constant. If
    it still read the 5-entry constant, a short run would index past its own
    plan on segment 3."""
    node = _construct({'segment_plan': 'short'})
    try:
        for seg in range(len(node.segment_plan)):
            node._segment_spec(0, seg)
        with pytest.raises(IndexError):
            node._segment_spec(0, len(node.segment_plan))
    finally:
        node.destroy_node()


# ---------------------------------------------------------------- couplings
def test_calibration_mux_lane_stays_below_joystick_and_above_navigation():
    """A coupling to f1tenth_bringup/config/mux.yaml, deliberately (see
    CLAUDE.md). If the lanes are renumbered so calibration outranks joystick,
    an open-loop calibration drive would ignore the operator's own override --
    which is the one thing this tool's safety story depends on."""
    path = _workspace_src('f1tenth_bringup', 'config', 'mux.yaml')
    if not os.path.exists(path):
        pytest.skip('mux.yaml not present in this checkout layout')
    topics = pyyaml.safe_load(open(path))['ackermann_mux']['ros__parameters']['topics']
    assert topics['calibration']['topic'] == 'calibration_drive'
    assert (topics['navigation']['priority'] < topics['calibration']['priority']
            < topics['joystick']['priority'] < topics['safety_stop']['priority'])


# ------------------------------------------------------------------ helpers
def test_yaw_from_quaternion_matches_a_known_rotation():
    class _Q:
        x = 0.0
        y = 0.0
        z = math.sin(math.radians(30.0) / 2.0)
        w = math.cos(math.radians(30.0) / 2.0)
    assert yaw_from_quaternion(_Q()) == pytest.approx(math.radians(30.0), abs=1e-9)


def test_wrap_angle_keeps_a_heading_difference_across_pi_small():
    assert wrap_angle(math.radians(359.0) - math.radians(1.0)) == pytest.approx(
        math.radians(-2.0), abs=1e-9)


def test_fit_results_are_plain_python_floats_not_numpy_scalars():
    """ruamel.yaml refuses to serialise a numpy.float64 ("cannot represent an
    object"), and the write-back happens AFTER the drive and AFTER the backup
    is made -- so a numpy scalar leaking out of the fit crashes the run at the
    most expensive possible moment. Found by an end-to-end smoke test, not by
    the writer tests above, which pass plain literals."""
    fit = fitlib.fit_gain_offset(_synthetic_samples())
    for name in ('gain', 'delta0', 'ci_gain', 'ci_delta0', 'wheelbase',
                 'residual_rms', 'correlation'):
        assert type(getattr(fit, name)) is float, name


def test_writer_accepts_the_values_a_real_fit_produces(tmp_path):
    """The regression test for the above, at the actual boundary: take a real
    fit's output and put it through the real writer."""
    fit = fitlib.fit_gain_offset(_synthetic_samples())
    path = _steering_calibration_fixture(tmp_path)
    write_yaml_config_with_provenance(
        path, {'/**': {'steering_angle_to_servo_offset': float(round(fit.delta0, 6))}},
        ['from a real fit'], _Logger(), tool='steering_offset_calibration_node')
    assert pyyaml.safe_load(open(path))['/**']['ros__parameters'][
        'steering_angle_to_servo_offset'] == pytest.approx(round(fit.delta0, 6))


def test_writer_reverts_round_trip_damage_to_values_it_was_not_asked_to_change(tmp_path):
    """ruamel's round-trip re-emitted an untouched neighbour of the key being
    written -- accel_variance_y lost its last digit when wheelbase was
    written to the real vesc.yaml. Numerically trivial, but it is an
    unrequested edit to a separately-calibrated value, and it makes the
    printed diff misrepresent what the calibration did."""
    source = _workspace_src('f1tenth_bringup', 'config', 'vesc.yaml')
    if not os.path.exists(source):
        pytest.skip('vesc.yaml not present in this checkout layout')
    target = str(tmp_path / 'vesc.yaml')
    shutil.copy2(source, target)
    before = open(target).read()

    write_yaml_config_with_provenance(
        target, {'vesc_to_odom_node': {'wheelbase': 0.330936}},
        ['fitted L_eff 0.330936'], _Logger(), tool='steering_offset_calibration_node')

    after = open(target).read()
    changed = {
        line.split(':')[0].strip()
        for line in after.split('\n')
        if ':' in line and not line.strip().startswith('#') and line not in before
    }
    assert 'accel_variance_y' not in changed
    assert changed <= {'wheelbase'}, f'unexpected keys changed: {changed}'
    loaded = pyyaml.safe_load(after)
    assert loaded['vesc_to_odom_node']['ros__parameters']['wheelbase'] == 0.330936
    assert loaded['/**']['ros__parameters']['accel_variance_y'] == pyyaml.safe_load(
        before)['/**']['ros__parameters']['accel_variance_y']


# ------------------------------------------------- mode A: the static sweep
def _synthetic_sweep(gain=TRUE_GAIN, offset=TRUE_DELTA0, backlash=TRUE_BACKLASH,
                     noise=0.0015, seed=3, curvature=0.0, amplitude_deg=15.0,
                     steps=7, one_direction=False):
    """A rising sweep then a falling one, generated from the exact model
    fit_static_sweep inverts, so the fit has a known right answer for all
    three of gain, offset and backlash."""
    rng = random.Random(seed)
    commands = [math.radians(-amplitude_deg + 2 * amplitude_deg * i / (steps - 1))
                for i in range(steps)]
    branches = [(+1, commands)]
    if not one_direction:
        branches.append((-1, list(reversed(commands))))
    out = []
    index = 0
    for direction, sequence in branches:
        for command in sequence:
            measured = (curvature * command ** 2 + gain * command + offset +
                        direction * (backlash / 2.0) + rng.gauss(0.0, noise))
            out.append(fitlib.SweepSample(index, command, measured, direction))
            index += 1
    return out


def test_static_sweep_recovers_known_gain_offset_and_backlash():
    """The test the work order asks for by name: recover all three from
    synthetic data, not merely check that gates fire. Gate-only testing
    already passed a solver walking uphill once."""
    fit = fitlib.fit_static_sweep(_synthetic_sweep())
    assert fit.gain == pytest.approx(TRUE_GAIN, abs=0.01)
    assert fit.offset == pytest.approx(TRUE_DELTA0, abs=math.radians(0.2))
    assert fit.backlash == pytest.approx(TRUE_BACKLASH, abs=math.radians(0.2))
    assert fit.is_linear


def test_static_sweep_residual_stays_at_the_measurement_noise_floor():
    fit = fitlib.fit_static_sweep(_synthetic_sweep(noise=0.0015))
    assert fit.residual_rms < math.radians(0.5)


def test_static_sweep_reports_a_nonlinear_linkage_instead_of_fitting_a_line():
    fit = fitlib.fit_static_sweep(_synthetic_sweep(curvature=0.9))
    assert not fit.is_linear
    assert abs(fit.curvature) > fit.ci_curvature


def test_static_sweep_with_zero_backlash_does_not_invent_any():
    fit = fitlib.fit_static_sweep(_synthetic_sweep(backlash=0.0))
    assert fit.backlash < math.radians(0.3)


def test_sweep_span_gate_refuses_a_range_too_narrow_to_determine_a_gain():
    fit = fitlib.fit_static_sweep(_synthetic_sweep(amplitude_deg=2.0))
    gate = fitlib.check_sweep_span(fit)
    assert not gate.passed
    assert 'too narrow' in gate.detail


def test_sweep_span_gate_passes_the_default_fifteen_degree_sweep():
    assert fitlib.check_sweep_span(fitlib.fit_static_sweep(_synthetic_sweep())).passed


def test_branch_gate_refuses_a_one_directional_sweep_that_cannot_see_backlash():
    fit = fitlib.fit_static_sweep(_synthetic_sweep(one_direction=True))
    gate = fitlib.check_sweep_branches(fit)
    assert not gate.passed
    assert 'BOTH directions' in gate.detail


def test_branch_gate_reports_real_backlash_as_a_finding_not_a_failure():
    """Backlash is a first-class output, not an error -- the gate must pass
    while naming it, so a car with genuine linkage slop can still be
    calibrated."""
    gate = fitlib.check_sweep_branches(fitlib.fit_static_sweep(_synthetic_sweep()))
    assert gate.passed
    assert 'RESOLVED as real backlash' in gate.detail


def test_branch_gate_refuses_an_implausibly_large_backlash():
    fit = fitlib.fit_static_sweep(_synthetic_sweep(backlash=math.radians(25.0)))
    assert not fitlib.check_sweep_branches(fit).passed


def test_sweep_csv_round_trips(tmp_path):
    samples = _synthetic_sweep()
    path = str(tmp_path / 'sweep.csv')
    fitlib.write_sweep_csv(path, samples)
    restored = fitlib.read_sweep_csv(path)
    assert len(restored) == len(samples)
    assert [s.direction for s in restored] == [s.direction for s in samples]
    assert fitlib.fit_static_sweep(restored).gain == pytest.approx(
        fitlib.fit_static_sweep(samples).gain, abs=1e-6)


def test_offline_cli_detects_a_sweep_csv_and_fits_it_as_mode_a(tmp_path):
    path = str(tmp_path / 'sweep.csv')
    fitlib.write_sweep_csv(path, _synthetic_sweep())
    assert fitlib.main([path]) == 0


def test_linearity_gate_refuses_to_write_a_single_gain_through_a_curve():
    node = _construct()
    try:
        fit = fitlib.fit_static_sweep(_synthetic_sweep(curvature=0.9))
        assert not node._linearity_gate(fit).passed
    finally:
        node.destroy_node()


# ------------------------------------------- turning points, with error bars
def test_turning_point_offset_is_free_of_wheelbase_and_path_length():
    """delta0 = -g*theta(t*) uses only the command at the crossing, so a
    turning-point estimate must not move when the wheelbase assumption does."""
    times = [0.0, 1.0, 2.0, 3.0]
    yaws = [0.0, 0.10, 0.15, 0.10]

    def command_at(t):
        return math.radians(-4.0)

    points = fitlib.turning_point_estimates(times, yaws, command_at, sigma_t=0.26)
    assert len(points) == 1
    assert points[0].delta0 == pytest.approx(math.radians(4.0), abs=1e-9)


def test_turning_point_uncertainty_grows_with_the_command_slew_rate():
    """A turning point found while the command was ramping fast is nearly
    uninformative; one found during a slow ramp is sharp. Reporting the bare
    number without this is what makes a 'not a constant offset' conclusion
    look established when it is only suggestive."""
    times = [0.0, 1.0, 2.0, 3.0]
    yaws = [0.0, 0.10, 0.15, 0.10]
    slow = fitlib.turning_point_estimates(
        times, yaws, lambda t: math.radians(-4.0 - 0.1 * t), sigma_t=0.26)[0]
    fast = fitlib.turning_point_estimates(
        times, yaws, lambda t: math.radians(-4.0 - 5.0 * t), sigma_t=0.26)[0]
    assert fast.ci_delta0 > slow.ci_delta0
    assert slow.ci_delta0 > 0.0


def test_turning_points_that_overlap_within_their_intervals_are_not_a_contradiction():
    points = [
        fitlib.TurningPoint(5.0, math.radians(-4.0), math.radians(4.0), 0.26),
        fitlib.TurningPoint(9.0, math.radians(-3.0), math.radians(4.0), 0.26),
    ]
    assert fitlib.turning_points_agree(points).passed


def test_turning_points_far_apart_relative_to_their_intervals_do_not_agree():
    points = [
        fitlib.TurningPoint(5.0, math.radians(-4.4), math.radians(0.2), 0.26),
        fitlib.TurningPoint(9.0, math.radians(+0.3), math.radians(0.2), 0.26),
    ]
    verdict = fitlib.turning_points_agree(points)
    assert not verdict.passed
    assert 'do NOT overlap' in verdict.detail


# --------------------------------------------------------- pinned wheelbase
def test_wheelbase_is_pinned_to_the_documented_spec_value():
    assert fitlib.PINNED_WHEELBASE_M == 0.3302


def test_node_reports_the_vesc_yaml_wheelbase_discrepancy_without_changing_it():
    node = _construct()
    try:
        note = node._report_wheelbase_discrepancy()
        assert '0.3302' in note and '0.3050' in note
        assert 'NOT changed' in note
    finally:
        node.destroy_node()


def test_static_sweep_prompt_flow_collects_both_branches():
    """Drive the mode A loop with a scripted prompt instead of a human, so the
    sequencing (rising then falling, both labelled) is covered without
    hardware or stdin."""
    node = _construct({'sweep_steps': 3, 'settle_sec': 0.0,
                       'sweep_amplitude_rad': math.radians(10.0)})
    try:
        answers = iter(['5', '0', '-5', '-5', '0', '5'])

        def scripted(_text):
            return next(answers)

        assert node.run_static_sweep(prompt=scripted)
        assert len(node.sweep_samples) == 6
        assert {s.direction for s in node.sweep_samples} == {1, -1}
    finally:
        node.destroy_node()


def test_static_sweep_aborts_cleanly_when_the_operator_quits():
    node = _construct({'sweep_steps': 3, 'settle_sec': 0.0})
    try:
        assert not node.run_static_sweep(prompt=lambda _t: 'q')
    finally:
        node.destroy_node()
