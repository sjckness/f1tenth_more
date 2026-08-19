"""gyro_bias_calibration_node.py / calibration_common.py tests --
calibration-safety-gates pass (Phase A). Same convention
test_slam_pose_covariance_calibration_node.py already uses for the node-level
tests: construct a real (never spun) rclpy Node via parameter_overrides and
call its callbacks/_finish() directly -- no live topics/hardware, no
executor, pure in-process method calls. The pure calibration_common
functions (check_sanity_bound/exceeds_motion_threshold/
exceeds_vibration_threshold) need no rclpy Node at all.

Covers, per gate (see gyro_bias_calibration_node.py's own module docstring
and calibration_common.py's own docstrings for the full reasoning -- this
pass's own report has the investigation these gates close):
  - A1 (check_sanity_bound): pure-function boundary tests, plus _finish()
    refusing to write on a violation -- including a test reproducing the
    actual 2026-08-19 11:06:58 bad write's own numbers.
  - A2 (min_samples hard floor): _finish() with count < min_samples must not
    compute a mean or write.
  - A3 (continuous stationarity + accelerometer check): _state_callback/
    _imu_callback aborting mid-sampling; _state_sub staying alive through
    _start_sampling() is the actual structural fix (previously destroyed
    there) and is asserted directly, not just its downstream effect.
  - A full happy-path test end to end, so a regression in the untouched code
    around these three gates is also caught, not just the gates in isolation.

Run standalone: python3 -m pytest test/test_gyro_bias_calibration_node.py -v
"""
import statistics

import pytest
import rclpy
from sensor_msgs.msg import Imu
from vesc_msgs.msg import VescState, VescStateStamped

from f1tenth_diagnostics.calibration_common import (
    EXIT_INSUFFICIENT_SAMPLES,
    EXIT_MOTION_DURING_SAMPLING,
    EXIT_SANITY_VIOLATION,
    EXIT_SUCCESS,
    check_sanity_bound,
    exceeds_motion_threshold,
    exceeds_vibration_threshold,
)
from f1tenth_diagnostics.gyro_bias_calibration_node import GyroBiasCalibrationNode


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _construct_with_params(param_dict):
    """See test_slam_pose_covariance_calibration_node.py's own matching
    helper -- same reasoning (rclpy.node.Node's parameter_overrides is the
    directly-constructed-Node equivalent of launch's parameters=[{...}])."""
    from rclpy.parameter import Parameter
    overrides = [Parameter(k, value=v) for k, v in param_dict.items()]
    return GyroBiasCalibrationNode(parameter_overrides=overrides)


def _fake_state(speed):
    msg = VescStateStamped()
    msg.state = VescState()
    msg.state.speed = speed
    return msg


def _fake_imu(gyro_z=0.0, gyro_x=0.0, gyro_y=0.0, accel_x=0.0, accel_y=0.0, accel_z=1.0):
    """accel defaults to (0, 0, 1.0) -- this hardware's actual at-rest g-unit
    reading (see exceeds_vibration_threshold's own docstring), not (0, 0, 0)
    or (0, 0, 9.81)."""
    msg = Imu()
    msg.angular_velocity.x = gyro_x
    msg.angular_velocity.y = gyro_y
    msg.angular_velocity.z = gyro_z
    msg.linear_acceleration.x = accel_x
    msg.linear_acceleration.y = accel_y
    msg.linear_acceleration.z = accel_z
    return msg


_VESC_YAML_TEMPLATE = """\
/**:
  ros__parameters:
    gyro_bias_z: {gyro_bias_z}
vesc_to_odom_node:
  ros__parameters:
    wheelbase: 0.33
"""

# The actual old (pre-incident) value from vesc.yaml.bak.20260819T110629 --
# used throughout as a realistic "old_gyro_bias_z" rather than an arbitrary
# number, matching this pass's own investigation.
_REAL_OLD_GYRO_BIAS_Z = -0.006683456952634612
# The actual bad value this pass's investigation traced to
# gyro_bias_calibration_node's 2026-08-19 11:06:58 write.
_REAL_BAD_GYRO_BIAS_Z = -0.05902505703321537


class _FakeLogger:
    """Minimal stand-in for rclpy's logger -- records .error()/.info() calls,
    for the pure calibration_common function tests below that don't need a
    real node/logger at all. .info() added for the delta-bound-recovery-trap
    fix's own "log the branch taken at INFO level" requirement."""

    def __init__(self):
        self.errors = []
        self.infos = []

    def error(self, msg):
        self.errors.append(msg)

    def info(self, msg):
        self.infos.append(msg)


# ==============================================================================
# check_sanity_bound -- pure function (A1)
# ==============================================================================

class TestCheckSanityBound:

    def test_within_both_bounds_passes(self):
        logger = _FakeLogger()
        assert check_sanity_bound('gyro_bias_z', 0.01, 0.005, 0.05, 0.02, logger) is True
        assert logger.errors == []

    def test_absolute_bound_violation_fails(self):
        logger = _FakeLogger()
        # |new|=0.06 >= 0.05; delta=0.005 stays well under 0.02, isolating
        # the absolute bound specifically.
        assert check_sanity_bound('gyro_bias_z', 0.06, 0.055, 0.05, 0.02, logger) is False
        assert len(logger.errors) == 1
        assert 'absolute_bound' in logger.errors[0]
        assert 'delta_bound' not in logger.errors[0]

    def test_delta_bound_violation_fails(self):
        logger = _FakeLogger()
        # |new|=0.03 stays well under 0.05, but delta=0.025 >= 0.02.
        assert check_sanity_bound('gyro_bias_z', 0.03, 0.005, 0.05, 0.02, logger) is False
        assert 'delta_bound' in logger.errors[0]
        assert 'absolute_bound' not in logger.errors[0]

    def test_todays_actual_bad_write_trips_both_bounds(self):
        """The actual event this pass closes -- see this function's own
        docstring and this pass's own report."""
        logger = _FakeLogger()
        result = check_sanity_bound(
            'gyro_bias_z', _REAL_BAD_GYRO_BIAS_Z, _REAL_OLD_GYRO_BIAS_Z, 0.05, 0.02, logger)
        assert result is False
        assert 'absolute_bound' in logger.errors[0]
        assert 'delta_bound' in logger.errors[0]

    @pytest.mark.parametrize('new_value,expected', [
        (0.049999, True),   # just under the absolute bound -> passes
        (0.05, False),      # exactly at the absolute bound -> violation ("< bound" to pass)
        (0.050001, False),  # just over -> violation
    ])
    def test_absolute_bound_boundary(self, new_value, expected):
        logger = _FakeLogger()
        # old_value == new_value here -> delta is always 0, isolating the
        # absolute bound specifically.
        assert check_sanity_bound(
            'gyro_bias_z', new_value, new_value, 0.05, 0.02, logger) is expected

    @pytest.mark.parametrize('delta,expected', [
        (0.019999, True),
        (0.02, False),
        (0.020001, False),
    ])
    def test_delta_bound_boundary(self, delta, expected):
        logger = _FakeLogger()
        old = 0.001
        assert check_sanity_bound(
            'gyro_bias_z', old + delta, old, 0.05, 0.02, logger) is expected

    # -- delta-bound recovery trap fix ---------------------------------------
    # See check_sanity_bound's own "delta-bound recovery trap" docstring
    # paragraph: an unconditional delta bound would permanently trap
    # vesc.yaml in a bad state, since correcting a bad old value back to a
    # genuinely good one is itself a large delta.

    def test_recovery_from_bad_old_value_skips_delta_bound(self):
        """Today's actual incident-recovery scenario -- old is the real bad
        write this pass's investigation found; new is a plausible corrected
        value. The delta (~0.051) would fail delta_bound=0.02 on its own, but
        old_value itself already fails absolute_bound, so delta_bound is
        skipped entirely and only the (passing) absolute check applies."""
        logger = _FakeLogger()
        old = _REAL_BAD_GYRO_BIAS_Z  # -0.05902505703321537, itself >= 0.05
        new = -0.008
        assert check_sanity_bound('gyro_bias_z', new, old, 0.05, 0.02, logger) is True
        assert logger.errors == []
        assert any(
            'recovery from out-of-bound stored value' in msg for msg in logger.infos)

    def test_healthy_start_still_rejects_large_delta(self):
        """From a healthy old value, a large swing is still rejected on
        delta_bound -- the fix only relaxes the "old value already bad"
        case, never loosens protection when old_value was fine."""
        logger = _FakeLogger()
        old = -0.008  # within absolute_bound -- delta_bound stays enforced
        new = -0.058
        assert check_sanity_bound('gyro_bias_z', new, old, 0.05, 0.02, logger) is False
        assert 'delta_bound' in logger.errors[0]
        assert not any(
            'recovery from out-of-bound stored value' in msg for msg in logger.infos)

    def test_healthy_start_still_rejects_absolute(self):
        """Unchanged behavior: a healthy old value doesn't relax the
        absolute bound on an implausible new value either."""
        logger = _FakeLogger()
        old = -0.008
        new = 0.06
        assert check_sanity_bound('gyro_bias_z', new, old, 0.05, 0.02, logger) is False
        assert 'absolute_bound' in logger.errors[0]


# ==============================================================================
# exceeds_motion_threshold / exceeds_vibration_threshold -- pure functions (A3)
# ==============================================================================

class TestExceedsMotionThreshold:

    @pytest.mark.parametrize('speed,threshold,expected', [
        (0.0, 500.0, False),
        (499.9, 500.0, False),
        (500.0, 500.0, False),   # exactly at threshold -> NOT exceeding (strict >)
        (500.1, 500.0, True),
        (-500.1, 500.0, True),   # sign-independent
    ])
    def test_boundary(self, speed, threshold, expected):
        assert exceeds_motion_threshold(speed, threshold) is expected


class TestExceedsVibrationThreshold:

    def test_at_rest_reading_does_not_trip(self):
        # This hardware's actual measured at-rest reading (this pass's own
        # 20s/965-sample live probe) -- gravity=1.0 g, not 9.81.
        assert exceeds_vibration_threshold(0.036, 0.043, 0.996, 0.3) is False

    def test_9_81_convention_would_have_falsely_tripped(self):
        """Regression guard for the actual unit mismatch caught live during
        this pass's own verification: this stack's vendored VESC IMU driver
        publishes linear_acceleration in g, not m/s^2 (vesc_packet.cpp's own
        acc_x()/acc_y()/acc_z() "g/s" comment). Explicitly exercises the
        WRONG gravity constant to confirm it would indeed have falsely
        tripped on every single sample at rest -- production code always
        calls this with the default gravity=1.0, never 9.81."""
        assert exceeds_vibration_threshold(0.036, 0.043, 0.996, 0.3, gravity=9.81) is True

    @pytest.mark.parametrize('deviation,expected', [
        (0.25, False),
        (0.5, False),
        # exactly at threshold -> NOT exceeding (strict >). 0.25/0.5 are
        # exact binary fractions -- deliberately NOT e.g. 0.3/0.29999/0.30001:
        # 1.0 + 0.3 - 1.0 != 0.3 in IEEE 754 double, which made this
        # exact-boundary case flaky the first time this test was written --
        # not a bug in exceeds_vibration_threshold itself, just an
        # unrepresentable-decimal trap in the test's own arithmetic.
        (0.75, True),
    ])
    def test_boundary_via_z_axis(self, deviation, expected):
        # ax=ay=0 isolates az as the only contributor to the summed deviation.
        assert exceeds_vibration_threshold(0.0, 0.0, 1.0 + deviation, 0.5) is expected


# ==============================================================================
# GyroBiasCalibrationNode -- A2: min_samples hard floor
# ==============================================================================

class TestMinSamplesHardFloor:

    def test_below_min_samples_does_not_write(self, tmp_path):
        yaml_path = tmp_path / 'vesc.yaml'
        yaml_path.write_text(_VESC_YAML_TEMPLATE.format(gyro_bias_z=_REAL_OLD_GYRO_BIAS_Z))
        before_mtime = yaml_path.stat().st_mtime_ns
        before_content = yaml_path.read_text()

        node = _construct_with_params({
            'vesc_yaml_path': str(yaml_path),
            'min_samples': 300,
        })
        try:
            node._start_sampling()
            for _ in range(299):  # one short of min_samples
                node._imu_callback(_fake_imu(gyro_z=0.001))
            node._finish()

            assert node.exit_code == EXIT_INSUFFICIENT_SAMPLES
            assert node.done is True
            assert yaml_path.stat().st_mtime_ns == before_mtime
            assert yaml_path.read_text() == before_content
            assert list(tmp_path.glob('*.bak.*')) == []
        finally:
            node.destroy_node()

    def test_zero_samples_still_insufficient_not_a_crash(self, tmp_path):
        """The pre-Phase-A `count < 2` floor still needs to work as a special
        case of the same hard gate -- statistics.mean([]) must never be
        reached."""
        yaml_path = tmp_path / 'vesc.yaml'
        yaml_path.write_text(_VESC_YAML_TEMPLATE.format(gyro_bias_z=_REAL_OLD_GYRO_BIAS_Z))

        node = _construct_with_params({'vesc_yaml_path': str(yaml_path), 'min_samples': 300})
        try:
            node._start_sampling()
            node._finish()  # timer fires with zero IMU messages ever received

            assert node.exit_code == EXIT_INSUFFICIENT_SAMPLES
            assert node.done is True
        finally:
            node.destroy_node()

    def test_at_exactly_min_samples_proceeds(self, tmp_path):
        """count == min_samples must NOT be treated as insufficient (only
        count < min_samples is) -- proceeds to a real, sane write."""
        yaml_path = tmp_path / 'vesc.yaml'
        yaml_path.write_text(_VESC_YAML_TEMPLATE.format(gyro_bias_z=_REAL_OLD_GYRO_BIAS_Z))

        node = _construct_with_params({'vesc_yaml_path': str(yaml_path), 'min_samples': 5})
        try:
            node._start_sampling()
            for _ in range(5):
                node._imu_callback(_fake_imu(gyro_z=0.001))
            node._finish()

            assert node.exit_code == EXIT_SUCCESS
            assert node.done is True
        finally:
            node.destroy_node()


# ==============================================================================
# GyroBiasCalibrationNode -- A1: sanity bound refuses the write
# ==============================================================================

class TestSanityBoundRefusesWrite:

    def test_sanity_violation_refuses_write(self, tmp_path):
        yaml_path = tmp_path / 'vesc.yaml'
        yaml_path.write_text(_VESC_YAML_TEMPLATE.format(gyro_bias_z=_REAL_OLD_GYRO_BIAS_Z))
        before_mtime = yaml_path.stat().st_mtime_ns

        node = _construct_with_params({
            'vesc_yaml_path': str(yaml_path),
            'min_samples': 5,
        })
        try:
            node._start_sampling()
            # residual chosen so old_value + residual reproduces this pass's
            # own actual bad write (_REAL_BAD_GYRO_BIAS_Z) -- see this pass's
            # own report.
            residual = _REAL_BAD_GYRO_BIAS_Z - _REAL_OLD_GYRO_BIAS_Z
            for _ in range(10):
                node._imu_callback(_fake_imu(gyro_z=residual))
            node._finish()

            assert node.exit_code == EXIT_SANITY_VIOLATION
            assert node.done is True
            assert yaml_path.stat().st_mtime_ns == before_mtime
            assert list(tmp_path.glob('*.bak.*')) == []
        finally:
            node.destroy_node()

    def test_sane_value_still_writes(self, tmp_path):
        """Confirms the sanity gate isn't over-tight -- a small, plausible
        correction must still succeed."""
        yaml_path = tmp_path / 'vesc.yaml'
        yaml_path.write_text(_VESC_YAML_TEMPLATE.format(gyro_bias_z=_REAL_OLD_GYRO_BIAS_Z))

        node = _construct_with_params({'vesc_yaml_path': str(yaml_path), 'min_samples': 5})
        try:
            node._start_sampling()
            for _ in range(10):
                node._imu_callback(_fake_imu(gyro_z=0.001))
            node._finish()

            assert node.exit_code == EXIT_SUCCESS
            assert list(tmp_path.glob('vesc.yaml.bak.*')) != []
        finally:
            node.destroy_node()


# ==============================================================================
# GyroBiasCalibrationNode -- A3: continuous stationarity/vibration check
# ==============================================================================

class TestMotionDuringSampling:

    def test_state_sub_stays_alive_through_sampling(self, tmp_path):
        """The actual structural fix this gate depends on -- previously
        _start_sampling() destroyed self._state_sub immediately, so nothing
        could ever watch the car again for the rest of the window. Asserted
        directly, not just inferred from the abort tests below working."""
        yaml_path = tmp_path / 'vesc.yaml'
        yaml_path.write_text(_VESC_YAML_TEMPLATE.format(gyro_bias_z=_REAL_OLD_GYRO_BIAS_Z))
        node = _construct_with_params({'vesc_yaml_path': str(yaml_path)})
        try:
            node._start_sampling()
            assert node._state_sub is not None
        finally:
            node.destroy_node()

    def test_erpm_motion_during_sampling_aborts(self, tmp_path):
        yaml_path = tmp_path / 'vesc.yaml'
        yaml_path.write_text(_VESC_YAML_TEMPLATE.format(gyro_bias_z=_REAL_OLD_GYRO_BIAS_Z))
        before_mtime = yaml_path.stat().st_mtime_ns

        node = _construct_with_params({'vesc_yaml_path': str(yaml_path)})
        try:
            node._start_sampling()
            node._imu_callback(_fake_imu(gyro_z=0.001))  # a good sample first
            node._state_callback(_fake_state(speed=600.0))  # exceeds default 500 ERPM

            assert node.exit_code == EXIT_MOTION_DURING_SAMPLING
            assert node.done is True
            assert node._sub is None
            assert node._state_sub is None
            assert yaml_path.stat().st_mtime_ns == before_mtime
            assert list(tmp_path.glob('*.bak.*')) == []
        finally:
            node.destroy_node()

    def test_erpm_at_exactly_threshold_does_not_abort(self, tmp_path):
        yaml_path = tmp_path / 'vesc.yaml'
        yaml_path.write_text(_VESC_YAML_TEMPLATE.format(gyro_bias_z=_REAL_OLD_GYRO_BIAS_Z))
        node = _construct_with_params({'vesc_yaml_path': str(yaml_path)})
        try:
            node._start_sampling()
            node._state_callback(_fake_state(speed=500.0))  # == default threshold, not over
            assert node.done is False
        finally:
            node.destroy_node()

    def test_vibration_during_sampling_aborts(self, tmp_path):
        yaml_path = tmp_path / 'vesc.yaml'
        yaml_path.write_text(_VESC_YAML_TEMPLATE.format(gyro_bias_z=_REAL_OLD_GYRO_BIAS_Z))
        before_mtime = yaml_path.stat().st_mtime_ns

        node = _construct_with_params({'vesc_yaml_path': str(yaml_path)})
        try:
            node._start_sampling()
            node._imu_callback(_fake_imu(gyro_z=0.001))  # a good sample first
            # az deviates by 1.0g from the 1.0g rest reference -- far over
            # the default 0.3 threshold.
            node._imu_callback(_fake_imu(gyro_z=0.001, accel_z=2.0))

            assert node.exit_code == EXIT_MOTION_DURING_SAMPLING
            assert node.done is True
            assert yaml_path.stat().st_mtime_ns == before_mtime
        finally:
            node.destroy_node()

    def test_disturbed_sample_itself_is_not_recorded(self, tmp_path):
        """The IMU message that trips the vibration check must not be
        appended to the sample buffers first -- doesn't matter for whether
        the write happens (aborted either way), but matters for anyone
        reading self._samples_z after the fact (e.g. future tooling)."""
        yaml_path = tmp_path / 'vesc.yaml'
        yaml_path.write_text(_VESC_YAML_TEMPLATE.format(gyro_bias_z=_REAL_OLD_GYRO_BIAS_Z))
        node = _construct_with_params({'vesc_yaml_path': str(yaml_path)})
        try:
            node._start_sampling()
            node._imu_callback(_fake_imu(gyro_z=0.001))
            count_before = len(node._samples_z)
            node._imu_callback(_fake_imu(gyro_z=999.0, accel_z=2.0))
            assert len(node._samples_z) == count_before
        finally:
            node.destroy_node()


# ==============================================================================
# Full happy path, end to end
# ==============================================================================

class TestFullHappyPath:

    def test_confirms_samples_and_writes(self, tmp_path):
        yaml_path = tmp_path / 'vesc.yaml'
        yaml_path.write_text(_VESC_YAML_TEMPLATE.format(gyro_bias_z=_REAL_OLD_GYRO_BIAS_Z))

        node = _construct_with_params({
            'vesc_yaml_path': str(yaml_path),
            'min_samples': 5,
            'stationary_confirm_sec': 0.0,
        })
        try:
            # Pre-sampling gate: StationaryGate needs two samples to confirm
            # even with confirm_sec=0.0 (the first just seeds _stationary_
            # since -- see StationaryGate.on_speed_sample's own if/elif).
            node._state_callback(_fake_state(speed=0.0))
            node._state_callback(_fake_state(speed=0.0))
            assert node._sub is not None  # gate confirmed, sampling started

            residuals = [0.0010, 0.0015, 0.0009, 0.0011, 0.0012, 0.0008]
            for r in residuals:
                node._imu_callback(_fake_imu(gyro_z=r))
            node._finish()

            assert node.exit_code == EXIT_SUCCESS
            assert node.done is True

            import ruamel.yaml
            loaded = ruamel.yaml.YAML().load(yaml_path.read_text())
            written = float(loaded['/**']['ros__parameters']['gyro_bias_z'])
            assert written == pytest.approx(_REAL_OLD_GYRO_BIAS_Z + statistics.mean(residuals))
            # vesc_to_odom_node's own untouched section must survive the
            # round-trip patch unchanged.
            assert loaded['vesc_to_odom_node']['ros__parameters']['wheelbase'] == 0.33
            assert len(list(tmp_path.glob('vesc.yaml.bak.*'))) == 1
        finally:
            node.destroy_node()
