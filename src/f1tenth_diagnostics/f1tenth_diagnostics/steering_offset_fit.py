"""Pure-math half of steering_offset_calibration_node: the bicycle-model fit of
(delta0, L_eff) from per-segment (delta_cmd, dpsi, ds) samples, plus the three
"refuse to write" gates that guard it.

Deliberately rclpy-free and importable on its own. Two reasons, both practical:

  - The node dumps its raw samples to CSV on EVERY run (including refused ones,
    see the node's own _dump_raw_samples). This module's main() reads that CSV
    back and redoes the identical fit offline -- so a refused or suspicious run
    can be re-analysed, re-gated with different thresholds, or compared against
    a later run WITHOUT re-driving the car. Re-driving is the expensive part;
    the fit is not.
  - The gates are the part most worth unit-testing, and testing them through a
    live rclpy node would mean spinning an executor to check arithmetic.

THE MODEL -- (gain, offset) AT A PINNED WHEELBASE
    d(psi)/d(s) = tan(g * delta_cmd + delta0) / L

integrated over a segment of (approximately) constant commanded steering:

    dpsi = tan(g * delta_cmd + delta0) / L * ds

g is the steering GAIN error (commanded-to-actual scale) and delta0 the
steering OFFSET. L is PINNED, never fitted.

WHY L IS PINNED, AND WHY FITTING L_eff WAS WRONG
The obvious three-parameter fit (g, delta0, L) is not identifiable at the
amplitudes this routine drives. For small angles tan(x) ~= x, so

    dpsi/ds ~= (g * delta_cmd + delta0) / L

in which g and L appear ONLY as the ratio g/L. At the 10 deg amplitude used
here the tan nonlinearity is about 1% -- far too little to separate them. A
three-parameter fit converges, reports a small residual, and splits g against
L arbitrarily.

This is not a hypothetical: the previous version of this module fitted
(delta0, L_eff) with g implicitly fixed at 1, which meant L_eff silently
absorbed exactly the gain error we now want to measure. Any gain error showed
up as a plausible-looking effective wheelbase instead of as a gain.

Fixed by construction: pin L, fit (g, delta0). Those two ARE separable,
because g multiplies delta_cmd while delta0 does not -- provided the drive
visits a real spread of commanded angles including both signs, which is what
check_conditioning() enforces.

    L = 0.3302 m (PINNED_WHEELBASE_M)

ASSUMED FROM THE F1TENTH SPEC (lf 0.15875 + lr 0.17145, i.e. 13 inches), NOT
MEASURED ON THIS CAR. Published F1TENTH figures disagree -- another parameter
set gives lf 0.128 / lr 0.137 = 0.265 m, citing Traxxas Slash 4x4 dimensions.
0.3302 is a deliberate choice, not a lookup. L enters as a divisor, so an
error in it becomes a proportional error in the fitted gain -- the same order
as the effect being measured. The provenance block records this verbatim so a
strange fitted gain later has a flagged assumption to point at.

MODE A (static sweep) IS THE GROUND TRUTH
fit_static_sweep() fits gain, offset and BACKLASH from directly measured wheel
angles, with no driving, no SLAM and no estimation chain in the path. The
drive fit above is validation of it plus whatever slip it cannot see.
"""

import csv
import math
import sys

import numpy as np

# Fit defaults. Exposed as module constants (not buried in the signature) so the
# node's ROS parameters and the offline CLI can share one source of truth.
DEFAULT_MAX_PARAM_CORRELATION = 0.95
DEFAULT_MIN_SIGN_SAMPLES = 2
DEFAULT_MIN_ABS_DELTA_RAD = 0.02  # ~1.15 deg -- below this a sample carries no sign information
# PINNED, never fitted -- see the module docstring. F1TENTH spec
# lf 0.15875 + lr 0.17145 = 13 inches. ASSUMED FROM SPEC, NOT MEASURED ON THIS
# CAR; published F1TENTH figures disagree (another parameter set gives
# lf 0.128 / lr 0.137 = 0.265 m citing Traxxas Slash 4x4 dimensions).
PINNED_WHEELBASE_M = 0.3302
# The value this workspace's vesc.yaml currently uses for /odom dead reckoning.
# Kept here only so the node can REPORT the discrepancy against the pinned
# value; nothing fits or writes it.
VESC_YAML_WHEELBASE_M = 0.305
# Half-width beyond which rising/falling branches are called backlash rather
# than noise, in radians (~0.5 deg).
DEFAULT_BACKLASH_TOLERANCE_RAD = 0.0087
# A static sweep must span at least this much commanded steering, or the gain
# has no lever arm.
DEFAULT_MIN_SWEEP_SPAN_RAD = 0.20

# Columns of the raw-sample CSV, in order. Written by the node, read by main().
CSV_FIELDS = (
    'repetition', 'segment', 'delta_cmd_rad', 'dpsi_rad', 'ds_m', 'ds_chord_m',
    'n_pose_samples', 't_start', 't_end',
)


class Sample:
    """One usable segment: a stretch of (approximately) constant commanded
    steering, with the heading change and arc length accumulated across it.

    dpsi/ds are INTEGRATED over the segment, never differentiated from
    individual poses -- with a 2 Hz pose source (see the node's own module
    docstring on why /slam/pose is the only gyro-independent option here),
    differentiating pose to get a yaw rate would divide two small, noisy
    differences by each other. Integrating instead means the pose noise enters
    only through the segment's two endpoints.
    """

    __slots__ = ('repetition', 'segment', 'delta_cmd', 'dpsi', 'ds', 'ds_chord',
                 'n_pose', 't_start', 't_end')

    def __init__(self, repetition, segment, delta_cmd, dpsi, ds, n_pose,
                 t_start=0.0, t_end=0.0, ds_chord=0.0):
        self.repetition = int(repetition)
        self.segment = int(segment)
        self.delta_cmd = float(delta_cmd)
        self.dpsi = float(dpsi)
        self.ds = float(ds)
        # Straight-line distance between the segment's first and last fix.
        # Not used by the fit -- recorded so an offline re-fit can check how
        # sensitive the result is to path-length inflation from pose noise
        # (ds sums consecutive fixes, so noise adds length; the chord
        # under-reads an arc instead, and the truth is between them).
        self.ds_chord = float(ds_chord)
        self.n_pose = int(n_pose)
        self.t_start = float(t_start)
        self.t_end = float(t_end)

    def as_row(self):
        return {
            'repetition': self.repetition,
            'segment': self.segment,
            'delta_cmd_rad': f'{self.delta_cmd:.9f}',
            'dpsi_rad': f'{self.dpsi:.9f}',
            'ds_m': f'{self.ds:.9f}',
            'ds_chord_m': f'{self.ds_chord:.9f}',
            'n_pose_samples': self.n_pose,
            't_start': f'{self.t_start:.6f}',
            't_end': f'{self.t_end:.6f}',
        }

    @classmethod
    def from_row(cls, row):
        return cls(
            repetition=row['repetition'], segment=row['segment'],
            delta_cmd=row['delta_cmd_rad'], dpsi=row['dpsi_rad'], ds=row['ds_m'],
            ds_chord=row.get('ds_chord_m', 0.0) or 0.0,
            n_pose=row['n_pose_samples'],
            t_start=row.get('t_start', 0.0) or 0.0,
            t_end=row.get('t_end', 0.0) or 0.0)


class FitResult:
    """Fitted (gain, delta0) at a pinned wheelbase, plus everything the gates
    and the provenance block need. ci_* are 95% confidence half-widths
    (1.96 sigma), NOT raw sigmas -- named ci_ to keep that explicit at every
    call site, since the agreement gate compares values against them.
    """

    def __init__(self, gain, delta0, ci_gain, ci_delta0, wheelbase, residual_rms,
                 correlation, n_samples, converged, iterations):
        # float(), not the values as handed in: they arrive as numpy scalars
        # and ruamel.yaml refuses to serialise a numpy.float64
        # ("RepresenterError: cannot represent an object"), which would crash
        # the write-back AFTER the car had driven the whole profile and AFTER
        # the backup was made. Coerced here, at the single point every
        # consumer reads these from.
        self.gain = float(gain)
        self.delta0 = float(delta0)
        self.ci_gain = float(ci_gain)
        self.ci_delta0 = float(ci_delta0)
        self.wheelbase = float(wheelbase)
        self.residual_rms = float(residual_rms)
        self.correlation = float(correlation)
        self.n_samples = int(n_samples)
        self.converged = converged
        self.iterations = iterations

    def summary(self):
        return (
            f'gain = {self.gain:.5f} +/- {self.ci_gain:.5f} (95%); '
            f'delta0 = {self.delta0:+.5f} rad ({math.degrees(self.delta0):+.3f} deg) '
            f'+/- {math.degrees(self.ci_delta0):.3f} deg (95%); '
            f'L pinned at {self.wheelbase:.4f} m; '
            f'residual RMS = {self.residual_rms:.6f} rad; '
            f'corr(gain, delta0) = {self.correlation:+.4f}; n = {self.n_samples}')


class GateResult:
    """A single refuse-to-write gate's verdict. `passed` is the only thing the
    caller branches on; `detail` is what gets logged and put in the report
    whether it passed or not (the numbers are useful either way -- a run that
    passed marginally is worth seeing).
    """

    def __init__(self, name, passed, detail):
        self.name = name
        self.passed = passed
        self.detail = detail

    def __repr__(self):
        return f'<GateResult {self.name} {"PASS" if self.passed else "REFUSE"}: {self.detail}>'


def _residuals_and_jacobian(samples, gain, delta0, wheelbase):
    """r_i = dpsi_i - tan(g*theta_i + delta0)/L * ds_i, and its Jacobian wrt
    [g, delta0]. L is a fixed constant here, never a free parameter.

        d(r)/d(g)      = -sec^2(g*theta_i + delta0) * theta_i / L * ds_i
        d(r)/d(delta0) = -sec^2(g*theta_i + delta0) / L * ds_i

    Note the structure that makes these two separable where (delta0, L) were
    not: the gain column carries a factor theta_i and the offset column does
    not, so a segment driven at theta = 0 constrains delta0 ALONE. The
    S-curve's straight segments are therefore doing real work -- they pin the
    offset directly, leaving the steered segments to determine the gain.
    """
    theta = np.array([s.delta_cmd for s in samples], dtype=float)
    dpsi = np.array([s.dpsi for s in samples], dtype=float)
    ds = np.array([s.ds for s in samples], dtype=float)

    total = gain * theta + delta0
    tan_t = np.tan(total)
    sec2_t = 1.0 / np.cos(total) ** 2

    residual = dpsi - tan_t / wheelbase * ds
    jac = np.column_stack((
        -sec2_t * theta / wheelbase * ds,
        -sec2_t / wheelbase * ds,
    ))
    return residual, jac


def fit_gain_offset(samples, wheelbase=PINNED_WHEELBASE_M, gain_init=1.0,
                    delta0_init=0.0, max_iterations=50, tol=1e-12):
    """Gauss-Newton fit of (gain, delta0) at a PINNED wheelbase.

    Replaces the old fit_bicycle(), which fitted (delta0, L_eff) and so let
    L_eff absorb the gain error -- see the module docstring.

    Gauss-Newton by hand rather than scipy.optimize: scipy is installed here
    but is not a declared dependency of this package, and this is a
    2-parameter problem with an analytic Jacobian.

    Raises ValueError below 3 samples: with 2 parameters, n = 2 leaves zero
    degrees of freedom, so the residual variance and every confidence interval
    would be 0.0 or NaN -- which the agreement gate would then read as
    "perfect agreement".
    """
    n = len(samples)
    if n < 3:
        raise ValueError(
            f'need at least 3 segments to fit 2 parameters with a residual variance, got {n}')

    gain = float(gain_init)
    delta0 = float(delta0_init)
    converged = False
    iterations = 0

    def cost(g, d0):
        res, _ = _residuals_and_jacobian(samples, g, d0, wheelbase)
        return float(res @ res)

    current_cost = cost(gain, delta0)
    for iterations in range(1, max_iterations + 1):
        residual, jac = _residuals_and_jacobian(samples, gain, delta0, wheelbase)
        # lstsq rather than explicit normal equations: on a badly conditioned
        # set (exactly what check_conditioning exists to catch) forming J^T J
        # squares the condition number and can fail outright, whereas lstsq's
        # SVD path still returns a usable step so we reach the gate and report
        # a real correlation instead of crashing.
        #
        # step solves J @ step ~= -residual, i.e. it IS the increment to ADD.
        # Subtracting it walks uphill: that was a real bug in the first
        # version of this solver, and the gates did NOT catch it -- they
        # refused, but for the wrong reason. Only the residual distinguishes
        # the two, which is why there is a test asserting on it directly.
        step, *_ = np.linalg.lstsq(jac, -residual, rcond=None)

        # Backtracking line search -- keeps the iteration on a descent path
        # without full Levenberg-Marquardt.
        scale = 1.0
        accepted = False
        for _ in range(30):
            trial_g = gain + scale * step[0]
            trial_d0 = delta0 + scale * step[1]
            trial_cost = cost(trial_g, trial_d0)
            if trial_cost <= current_cost:
                gain, delta0, current_cost = trial_g, trial_d0, trial_cost
                accepted = True
                break
            scale *= 0.5
        if not accepted:
            converged = True
            break
        if abs(scale * step[0]) < tol and abs(scale * step[1]) < tol:
            converged = True
            break

    residual, jac = _residuals_and_jacobian(samples, gain, delta0, wheelbase)
    dof = max(n - 2, 1)
    sigma2 = float(residual @ residual) / dof
    try:
        cov = sigma2 * np.linalg.inv(jac.T @ jac)
    except np.linalg.LinAlgError:
        cov = np.full((2, 2), np.inf)

    var_g, var_d0 = float(cov[0, 0]), float(cov[1, 1])
    if np.isfinite(var_g) and np.isfinite(var_d0) and var_g > 0.0 and var_d0 > 0.0:
        correlation = float(cov[0, 1] / math.sqrt(var_g * var_d0))
    else:
        correlation = 1.0

    return FitResult(
        gain=gain,
        delta0=delta0,
        ci_gain=1.96 * math.sqrt(var_g) if np.isfinite(var_g) else float('inf'),
        ci_delta0=1.96 * math.sqrt(var_d0) if np.isfinite(var_d0) else float('inf'),
        wheelbase=wheelbase,
        residual_rms=math.sqrt(float(residual @ residual) / n),
        correlation=float(np.clip(correlation, -1.0, 1.0)),
        n_samples=n,
        converged=converged,
        iterations=iterations)


def fit_delta0_only(samples, gain, wheelbase=PINNED_WHEELBASE_M):
    """Fit delta0 alone with the gain held at a pooled value. Used
    per-repetition by check_repetition_agreement().

    Per-repetition fits deliberately do NOT re-fit the gain: one repetition is
    5 segments, of which only 3 carry steering, and freeing both parameters on
    that little data widens every interval until the agreement check passes
    vacuously. Holding the gain fixed asks the sharper question the gate
    actually wants answered: given ONE gain, does each repetition still want
    the same offset?
    """
    n = len(samples)
    if n < 2:
        raise ValueError(f'need at least 2 segments to fit delta0 with a residual, got {n}')

    delta0 = 0.0
    for _ in range(50):
        residual, jac = _residuals_and_jacobian(samples, gain, delta0, wheelbase)
        col = jac[:, 1:2]
        step, *_ = np.linalg.lstsq(col, -residual, rcond=None)
        delta0 += step[0]
        if abs(step[0]) < 1e-12:
            break

    residual, jac = _residuals_and_jacobian(samples, gain, delta0, wheelbase)
    col = jac[:, 1:2]
    dof = max(n - 1, 1)
    sigma2 = float(residual @ residual) / dof
    # .item(), not float(): col.T @ col is a 1x1 2-D array and float() on an
    # ndim > 0 array is deprecated in numpy >= 1.25 and becomes an error.
    jtj = float((col.T @ col).item())
    var = sigma2 / jtj if jtj > 0.0 else float('inf')
    return float(delta0), (1.96 * math.sqrt(var) if np.isfinite(var) else float('inf'))


def check_conditioning(samples, fit, max_correlation=DEFAULT_MAX_PARAM_CORRELATION,
                       min_sign_samples=DEFAULT_MIN_SIGN_SAMPLES,
                       min_abs_delta=DEFAULT_MIN_ABS_DELTA_RAD):
    """Refuse if gain and delta0 are not separably determined by this data.

    Three independent checks, any one failing refuses the write:
      - parameter correlation from the covariance matrix;
      - steering visited both signs (a one-sided drive cannot tell a gain
        error from an offset, since over a one-sided range the two produce
        nearly the same curvature change);
      - at least one near-zero-steering segment OR two distinct magnitudes.
        A gain is a SLOPE: with every sample at one magnitude there is no
        lever arm to measure it, and the fit reports whatever split of
        (g, delta0) reproduces that single operating point.
    The sign and magnitude checks are the ones that state the problem in terms
    an operator can act on, rather than as an abstract correlation.
    """
    signed = [s.delta_cmd for s in samples if abs(s.delta_cmd) >= min_abs_delta]
    n_pos = sum(1 for d in signed if d > 0.0)
    n_neg = sum(1 for d in signed if d < 0.0)
    magnitudes = sorted({round(abs(d), 4) for d in signed})
    n_zero = sum(1 for s in samples if abs(s.delta_cmd) < min_abs_delta)

    failures = []
    if abs(fit.correlation) > max_correlation:
        failures.append(
            f'|corr(gain, delta0)| = {abs(fit.correlation):.4f} > {max_correlation:.4f} '
            '-- the two are not separably determined by this data; the reported split '
            'between them is arbitrary even though the residual is small')
    if n_pos < min_sign_samples or n_neg < min_sign_samples:
        failures.append(
            f'steering did not visit both signs enough: {n_pos} segments above '
            f'+{min_abs_delta:.3f} rad and {n_neg} below -{min_abs_delta:.3f} rad '
            f'(need >= {min_sign_samples} of each). A one-sided drive cannot separate '
            'a gain error from a steering offset')
    if n_zero < 1 and len(magnitudes) < 2:
        failures.append(
            f'no near-zero-steering segment and only one distinct |delta| magnitude '
            f'({magnitudes}) -- a gain is a slope and there is no lever arm to measure '
            'it here')

    detail = (
        f'corr = {fit.correlation:+.4f} (limit +/-{max_correlation:.2f}); '
        f'signed segments: {n_pos} positive / {n_neg} negative '
        f'(need >= {min_sign_samples} each); {n_zero} near-zero segment(s) pinning the '
        f'offset directly; distinct |delta| magnitudes: {magnitudes}')
    if failures:
        detail = detail + ' || ' + ' ; '.join(failures)
    return GateResult('conditioning', not failures, detail)


def check_repetition_agreement(samples, gain, wheelbase=PINNED_WHEELBASE_M,
                               min_repetitions=3):
    """Refuse if per-repetition delta0 values disagree beyond their CIs.

    A constant steering offset is by definition the same on every repetition.
    If two repetitions want different offsets and their intervals do not
    overlap, whatever is being measured is not a constant offset -- BACKLASH
    is the leading candidate on this car (see fit_static_sweep), and averaging
    two branches of a hysteresis loop into one number is exactly the wrong
    response.

    Pairwise interval overlap rather than a pooled chi-square: it is the
    literal reading of "disagree beyond their confidence intervals", and it
    names the two specific repetitions that clash, which is what an operator
    needs in order to go and look at what differed between them.
    """
    by_rep = {}
    for s in samples:
        by_rep.setdefault(s.repetition, []).append(s)

    per_rep = {}
    skipped = []
    for rep, rep_samples in sorted(by_rep.items()):
        if len(rep_samples) < 2:
            skipped.append(rep)
            continue
        per_rep[rep] = fit_delta0_only(rep_samples, gain, wheelbase)

    if len(per_rep) < min_repetitions:
        return GateResult(
            'repetition_agreement', False,
            f'only {len(per_rep)} repetition(s) produced a usable per-repetition delta0 '
            f'(need >= {min_repetitions}; skipped for too few segments: {skipped}) -- '
            'cannot check whether the offset is actually constant across runs')

    reps = sorted(per_rep)
    lines = [f'rep {r}: delta0 = {math.degrees(per_rep[r][0]):+.3f} '
             f'+/- {math.degrees(per_rep[r][1]):.3f} deg' for r in reps]

    worst = None
    for i, ri in enumerate(reps):
        for rj in reps[i + 1:]:
            d_i, ci_i = per_rep[ri]
            d_j, ci_j = per_rep[rj]
            gap = abs(d_i - d_j)
            allowed = ci_i + ci_j
            margin = gap - allowed
            if worst is None or margin > worst[0]:
                worst = (margin, ri, rj, gap, allowed)

    margin, ri, rj, gap, allowed = worst
    detail = ('; '.join(lines) +
              f' || widest disagreement: rep {ri} vs rep {rj}, '
              f'gap {math.degrees(gap):.3f} deg vs combined CI {math.degrees(allowed):.3f} deg')
    if margin > 0.0:
        detail += (' -- intervals do NOT overlap, so this is not a constant offset. '
                   'Backlash is the leading suspect; run the static sweep (mode A), '
                   'which measures it directly')
    return GateResult('repetition_agreement', margin <= 0.0, detail)


def check_pose_support(samples, min_pose_samples_per_segment):
    """Refuse if segments were fit from too few pose fixes.

    This is the gate most likely to fire on this stack and it is not a
    formality: /slam/pose publishes at ~2 Hz (measured, see the node's own
    module docstring), so a 0.6 m segment driven at 0.25 m/s spans about
    2.4 s and yields roughly 5 fixes -- and only ~4 after the settling
    discard. There is very little headroom, and a segment that came back with
    2 fixes has a dpsi that is essentially one noisy pose difference.
    """
    thin = [s for s in samples if s.n_pose < min_pose_samples_per_segment]
    counts = sorted(s.n_pose for s in samples)
    detail = (f'pose fixes per segment: min = {counts[0] if counts else 0}, '
              f'median = {counts[len(counts) // 2] if counts else 0}, '
              f'max = {counts[-1] if counts else 0} '
              f'(need >= {min_pose_samples_per_segment} each)')
    if thin:
        detail += (f' || {len(thin)} of {len(samples)} segments below the floor: ' +
                   ', '.join(f'rep{s.repetition}/seg{s.segment}={s.n_pose}' for s in thin))
    return GateResult('pose_support', not thin, detail)


# ===========================================================================
# MODE A -- static sweep. The ground truth: measured wheel angles, no driving,
# no SLAM, no estimation chain anywhere in the path.
# ===========================================================================

SWEEP_CSV_FIELDS = ('index', 'commanded_rad', 'measured_rad', 'direction')


class SweepSample:
    """One static-sweep point: a commanded steering angle, the wheel angle a
    human actually measured at it, and which way the sweep was moving when it
    arrived there (+1 rising, -1 falling).

    `direction` is not bookkeeping -- it is the whole reason the sweep is run
    in both directions. The gap between the two branches at the same commanded
    angle IS the backlash, and without the direction recorded it is
    indistinguishable from measurement noise.
    """

    __slots__ = ('index', 'commanded', 'measured', 'direction')

    def __init__(self, index, commanded, measured, direction):
        self.index = int(index)
        self.commanded = float(commanded)
        self.measured = float(measured)
        self.direction = 1 if float(direction) >= 0 else -1

    def as_row(self):
        return {'index': self.index,
                'commanded_rad': f'{self.commanded:.9f}',
                'measured_rad': f'{self.measured:.9f}',
                'direction': self.direction}

    @classmethod
    def from_row(cls, row):
        return cls(row['index'], row['commanded_rad'], row['measured_rad'],
                   row['direction'])


class StaticFitResult:
    """gain / offset / backlash from a static sweep, with a curvature term
    carried alongside so a nonlinear command->angle relation is reported as
    such rather than silently fitted with a straight line."""

    def __init__(self, gain, offset, backlash, ci_gain, ci_offset, ci_backlash,
                 curvature, ci_curvature, residual_rms, n_samples,
                 n_rising, n_falling, span):
        self.gain = float(gain)
        self.offset = float(offset)
        self.backlash = float(backlash)
        self.ci_gain = float(ci_gain)
        self.ci_offset = float(ci_offset)
        self.ci_backlash = float(ci_backlash)
        self.curvature = float(curvature)
        self.ci_curvature = float(ci_curvature)
        self.residual_rms = float(residual_rms)
        self.n_samples = int(n_samples)
        self.n_rising = int(n_rising)
        self.n_falling = int(n_falling)
        self.span = float(span)

    @property
    def is_linear(self):
        """True when the fitted quadratic term is not distinguishable from
        zero at 95%. If this is False, the gain below is a straight line
        through a curve and should be reported as such, not applied."""
        return abs(self.curvature) <= self.ci_curvature

    def summary(self):
        linear = 'linear' if self.is_linear else 'NOT LINEAR'
        return (
            f'gain = {self.gain:.5f} +/- {self.ci_gain:.5f} (95%); '
            f'offset = {math.degrees(self.offset):+.3f} '
            f'+/- {math.degrees(self.ci_offset):.3f} deg (95%); '
            f'backlash = {math.degrees(self.backlash):.3f} '
            f'+/- {math.degrees(self.ci_backlash):.3f} deg (95%); '
            f'residual RMS = {math.degrees(self.residual_rms):.3f} deg; '
            f'{linear} (quadratic term {self.curvature:+.4f} '
            f'+/- {self.ci_curvature:.4f} rad^-1); '
            f'n = {self.n_samples} ({self.n_rising} rising / {self.n_falling} falling), '
            f'span = {math.degrees(self.span):.1f} deg')


def fit_static_sweep(samples):
    """Least-squares fit of

        measured = curvature*commanded^2 + gain*commanded + offset
                   + direction * (backlash / 2)

    Linear in all four parameters, so this is one lstsq -- no iteration, no
    initial guess, nothing to diverge.

    The quadratic term is fitted but NOT applied to the reported gain: it
    exists so is_linear can say whether a straight line was a legitimate
    description of the data. Fitting it and reporting it is what makes "if it
    is not linear, say so rather than fitting a line through a curve"
    checkable rather than a matter of opinion.

    backlash comes out as twice the fitted direction coefficient and is
    reported as a positive width. Its sign carries no physical meaning (it
    depends only on which branch is labelled +1), so it is taken as an
    absolute value; the SIGN of the raw coefficient stays available through
    the residuals if anyone needs it.
    """
    n = len(samples)
    if n < 5:
        raise ValueError(
            f'need at least 5 sweep points to fit 4 parameters with a residual, got {n}')

    commanded = np.array([s.commanded for s in samples], dtype=float)
    measured = np.array([s.measured for s in samples], dtype=float)
    direction = np.array([s.direction for s in samples], dtype=float)

    design = np.column_stack((commanded ** 2, commanded, np.ones(n), direction))
    params, *_ = np.linalg.lstsq(design, measured, rcond=None)
    residual = measured - design @ params

    dof = max(n - 4, 1)
    sigma2 = float(residual @ residual) / dof
    try:
        cov = sigma2 * np.linalg.inv(design.T @ design)
        errs = [1.96 * math.sqrt(abs(float(cov[i, i]))) for i in range(4)]
    except np.linalg.LinAlgError:
        errs = [float('inf')] * 4

    curvature, gain, offset, half_backlash = (float(v) for v in params)
    return StaticFitResult(
        gain=gain, offset=offset, backlash=2.0 * abs(half_backlash),
        ci_gain=errs[1], ci_offset=errs[2], ci_backlash=2.0 * errs[3],
        curvature=curvature, ci_curvature=errs[0],
        residual_rms=math.sqrt(float(residual @ residual) / n),
        n_samples=n,
        n_rising=sum(1 for s in samples if s.direction > 0),
        n_falling=sum(1 for s in samples if s.direction < 0),
        span=float(commanded.max() - commanded.min()))


def check_sweep_span(fit, min_span=DEFAULT_MIN_SWEEP_SPAN_RAD):
    """Refuse a sweep too narrow to determine a gain. A gain is a slope; over a
    short baseline any slope error is absorbed by the offset."""
    passed = fit.span >= min_span
    detail = (f'commanded span = {math.degrees(fit.span):.2f} deg '
              f'(need >= {math.degrees(min_span):.2f} deg)')
    if not passed:
        detail += (' -- too narrow to determine a gain; the fitted slope would be '
                   'almost entirely absorbed by the offset')
    return GateResult('sweep_span', passed, detail)


def check_sweep_branches(fit, backlash_tolerance=DEFAULT_BACKLASH_TOLERANCE_RAD):
    """Rising and falling branches must either agree, or the disagreement must
    be reported as backlash rather than averaged away.

    This gate does NOT refuse merely because backlash exists -- backlash is a
    first-class measured output here. It refuses when a sweep cannot say
    anything about it: no falling branch at all (so the two-directional
    approach was not actually run), or a backlash so large it is no longer
    credible as linkage slop.
    """
    failures = []
    if fit.n_rising < 2 or fit.n_falling < 2:
        failures.append(
            f'only {fit.n_rising} rising / {fit.n_falling} falling points -- every '
            'angle must be approached from BOTH directions or backlash cannot be '
            'separated from measurement noise')
    if fit.backlash > 20 * backlash_tolerance:
        failures.append(
            f'backlash {math.degrees(fit.backlash):.2f} deg is implausibly large for '
            'linkage slop -- suspect a mis-entered measurement or a loose servo horn')
    significant = fit.backlash > fit.ci_backlash and fit.backlash > backlash_tolerance
    detail = (f'backlash = {math.degrees(fit.backlash):.3f} '
              f'+/- {math.degrees(fit.ci_backlash):.3f} deg; '
              f'{fit.n_rising} rising / {fit.n_falling} falling points; '
              + ('RESOLVED as real backlash (exceeds both its own CI and the '
                 f'{math.degrees(backlash_tolerance):.2f} deg tolerance)'
                 if significant else
                 'not distinguishable from noise at this tolerance'))
    if failures:
        detail += ' || ' + ' ; '.join(failures)
    return GateResult('sweep_branches', not failures, detail)


def write_sweep_csv(path, samples):
    with open(path, 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(SWEEP_CSV_FIELDS))
        writer.writeheader()
        for sample in samples:
            writer.writerow(sample.as_row())


def read_sweep_csv(path):
    with open(path, newline='') as handle:
        return [SweepSample.from_row(row) for row in csv.DictReader(handle)]


# ===========================================================================
# Turning-point estimator (mode B diagnostics)
# ===========================================================================

class TurningPoint:
    """A heading-rate sign change and the steering offset it implies.

    At a turning point the ACTUAL wheel angle is zero, so
    g*theta(t*) + delta0 = 0, i.e. delta0 = -g*theta(t*). This is free of
    L_eff and of path length entirely -- it depends only on locating t* and
    reading the command there, which is what makes it a useful independent
    cross-check on the fitted offset.

    The catch, and the reason this class carries an uncertainty: t* is located
    between pose fixes, and /slam/pose runs at ~1.9 Hz here. A +/-0.25-0.5 s
    timing uncertainty is inherited by the command, amplified by however fast
    the command was slewing through that window:

        sigma_delta0 = |g| * |d(theta)/dt| * sigma_t

    A turning point found while the command was ramping quickly is therefore
    almost uninformative, and one found during a slow ramp is sharp. Reporting
    the bare number without this is what makes a "not a constant offset"
    conclusion look established when it is only suggestive.
    """

    def __init__(self, time, theta_at, slew_rate, sigma_t, gain=1.0):
        self.time = float(time)
        self.theta_at = float(theta_at)
        self.slew_rate = float(slew_rate)
        self.sigma_t = float(sigma_t)
        self.gain = float(gain)
        self.delta0 = -self.gain * self.theta_at
        self.sigma_delta0 = abs(self.gain) * abs(self.slew_rate) * self.sigma_t

    @property
    def ci_delta0(self):
        """95% half-width from the propagated timing uncertainty."""
        return 1.96 * self.sigma_delta0

    def summary(self):
        return (f't* = {self.time:.2f} s: command {math.degrees(self.theta_at):+.2f} deg, '
                f'slewing {math.degrees(self.slew_rate):+.2f} deg/s -> '
                f'delta0 = {math.degrees(self.delta0):+.2f} '
                f'+/- {math.degrees(self.ci_delta0):.2f} deg (95%, from '
                f'sigma_t = {self.sigma_t:.2f} s)')


def turning_point_estimates(times, yaws, command_at, sigma_t=0.26, gain=1.0):
    """Find heading-rate sign changes in a pose series and return a
    TurningPoint per crossing.

    sigma_t defaults to 0.26 s -- half a /slam/pose interval at the ~1.9 Hz
    this stack actually publishes, which is the resolution with which a
    zero-crossing of a rate estimated between consecutive fixes can be placed.

    `command_at` is a callable t -> commanded steering (rad); the slew rate is
    differenced across +/- sigma_t around t*, i.e. over exactly the window the
    timing uncertainty spans, rather than an instantaneous derivative that
    would understate it.
    """
    points = []
    for i in range(1, len(times) - 1):
        dt_prev = times[i] - times[i - 1]
        dt_next = times[i + 1] - times[i]
        if dt_prev <= 0.0 or dt_next <= 0.0:
            continue
        rate_prev = (yaws[i] - yaws[i - 1]) / dt_prev
        rate_next = (yaws[i + 1] - yaws[i]) / dt_next
        if rate_prev * rate_next >= 0.0:
            continue
        mid_prev = 0.5 * (times[i - 1] + times[i])
        mid_next = 0.5 * (times[i] + times[i + 1])
        # Linear interpolation of the rate's zero crossing between the two
        # interval midpoints (each rate estimate belongs at its midpoint).
        t_star = mid_prev + (mid_next - mid_prev) * (rate_prev / (rate_prev - rate_next))
        slew = (command_at(t_star + sigma_t) - command_at(t_star - sigma_t)) / (2.0 * sigma_t)
        points.append(TurningPoint(t_star, command_at(t_star), slew, sigma_t, gain))
    return points


def turning_points_agree(points):
    """Do all turning-point offsets share a common value within their
    intervals? Returns a GateResult-shaped verdict so it reads like the other
    checks, though it is reported as a DIAGNOSTIC rather than used to refuse a
    write -- it is an independent cross-check on the fit, not a gate on it."""
    if len(points) < 2:
        return GateResult('turning_points', True,
                          f'{len(points)} turning point(s) -- too few to cross-check')
    lines = [p.summary() for p in points]
    worst = None
    for i, pi in enumerate(points):
        for pj in points[i + 1:]:
            gap = abs(pi.delta0 - pj.delta0)
            allowed = pi.ci_delta0 + pj.ci_delta0
            margin = gap - allowed
            if worst is None or margin > worst[0]:
                worst = (margin, pi, pj, gap, allowed)
    margin, pi, pj, gap, allowed = worst
    detail = ('; '.join(lines) +
              f' || widest: t*={pi.time:.2f} vs t*={pj.time:.2f}, gap '
              f'{math.degrees(gap):.2f} deg vs combined CI {math.degrees(allowed):.2f} deg')
    if margin > 0.0:
        detail += (' -- these do NOT overlap even after propagating the timing '
                   'uncertainty, so a single constant offset does not describe them')
    else:
        detail += (' -- these DO overlap once timing uncertainty is propagated; a '
                   'constant offset remains consistent with them')
    return GateResult('turning_points', margin <= 0.0, detail)


def write_samples_csv(path, samples):
    """Always called, on every run, including refused ones -- so the fit can be
    redone offline (main() below) without re-driving the car."""
    with open(path, 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(CSV_FIELDS))
        writer.writeheader()
        for sample in samples:
            writer.writerow(sample.as_row())


def read_samples_csv(path):
    with open(path, newline='') as handle:
        return [Sample.from_row(row) for row in csv.DictReader(handle)]


def main(argv=None):
    """Offline re-fit: `python3 -m f1tenth_diagnostics.steering_offset_fit <csv>`.

    Auto-detects which kind of CSV it was given -- a drive dump (mode B) or a
    static sweep (mode A) -- by its header, and runs the matching fit and
    gates. Same code the live node runs, so a refused run can be re-examined,
    or a threshold tried out, without touching the car.
    """
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv:
        print('usage: python3 -m f1tenth_diagnostics.steering_offset_fit <csv> '
              '[min_pose_samples_per_segment]', file=sys.stderr)
        return 2

    with open(argv[0], newline='') as handle:
        header = handle.readline()

    if 'measured_rad' in header:
        samples = read_sweep_csv(argv[0])
        fit = fit_static_sweep(samples)
        gates = [check_sweep_span(fit), check_sweep_branches(fit)]
        print('MODE A (static sweep): ' + fit.summary())
    else:
        samples = read_samples_csv(argv[0])
        min_pose = int(argv[1]) if len(argv) > 1 else 4
        fit = fit_gain_offset(samples)
        gates = [
            check_conditioning(samples, fit),
            check_repetition_agreement(samples, fit.gain, fit.wheelbase),
            check_pose_support(samples, min_pose),
        ]
        print('MODE B (drive): ' + fit.summary())

    for gate in gates:
        print(f'  [{"PASS  " if gate.passed else "REFUSE"}] {gate.name}: {gate.detail}')
    ok = all(gate.passed for gate in gates)
    print('all gates passed' if ok else 'REFUSED -- see the failing gate(s) above')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
