"""The tracker's own motion-uncertainty model, and the measurement behind it.

NOT /odom's covariance. vesc.yaml publishes fixed literals (x/y 0.2, yaw 0.03)
and vx_variance is 0.0, so consuming them would hand the tracker a closing rate
with no uncertainty at all and a position uncertainty that never grows.

The model has to answer one question: given that the vehicle travelled ds and
turned dtheta since a track was last corrected, how wrong can the predicted
position of that track's surface be? That is a covariance per step, which the
tracker compounds (Step 1) rather than sums.

ON THE TWO PARAMETERS THE BRIEF NAMES. sigma_perp per metre and sigma_yaw per
radian are not sufficient on their own: a straight traverse has dtheta ~ 0, so
a model with only those two assigns a straight run ZERO heading uncertainty,
and heading error is exactly what rotates subsequent translation into
cross-track error while coasting. sigma_yaw_per_m is therefore a third
parameter here, and the calibration has to fit it. Stated rather than folded in
quietly, because it is a departure from the brief.

ON WHAT THIS MODEL CANNOT REPRESENT. White process noise compounds as
sigma ~ k*sqrt(distance). The dominant error in every recording in this
archive is not white: it is a SCALE BIAS of about 1.22 (see measure_odom_scale
below), which grows linearly with distance and is perfectly correlated step to
step. No Gaussian process-noise model fits it, and inflating the white terms
until the NEES looks right would only be smuggling a systematic error in as
noise, where it would be under-reported at short range and over-reported at
long. Calibrate this model only on recordings made AFTER the gain is fixed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .geometry import Pose2D


@dataclass(frozen=True)
class ProcessNoiseParams:
    """Per-step motion noise, in the frame of the step.

    The defaults are DELIBERATELY not a calibration. They are placeholders that
    keep the units honest, and are what an uncalibrated tracker would run with;
    every recording available when this was written predates the drivetrain
    gain fix, so no honest numbers exist yet. calibrated=False says so, and a
    consumer that needs real uncertainty should refuse to run on these.
    """

    sigma_along_per_m: float = 0.02      # m of along-track error per metre driven
    sigma_perp_per_m: float = 0.02       # m of cross-track error per metre driven
    sigma_yaw_per_rad: float = 0.02      # rad of heading error per radian turned
    sigma_yaw_per_m: float = 0.01        # rad of heading error per metre driven
    calibrated: bool = False

    def step_cov(self, ds: float, dtheta: float) -> np.ndarray:
        """3x3 covariance of one odometry step (x, y, theta), step frame.

        Variance is linear in the distance travelled and the angle turned, i.e.
        sigma grows as sqrt of them: that is the random-walk form, the only one
        that compounds correctly over a sequence of steps. A term that grew
        sigma linearly per step would be claiming the errors are perfectly
        correlated, which is a bias, not noise -- see the module docstring.
        """
        d, a = abs(float(ds)), abs(float(dtheta))
        return np.diag([
            self.sigma_along_per_m ** 2 * d,
            self.sigma_perp_per_m ** 2 * d,
            self.sigma_yaw_per_rad ** 2 * a + self.sigma_yaw_per_m ** 2 * d,
        ])


def compound(delta: Pose2D, delta_cov: np.ndarray, step: Pose2D,
             step_cov: np.ndarray) -> tuple[Pose2D, np.ndarray]:
    """Accumulate one more step onto a track's motion since its last update.

    delta' = delta (+) step, with

        J1 = d(delta (+) step)/d(delta),  J2 = d(delta (+) step)/d(step)

    Summing per-step covariances instead drops the cross terms and
    under-reports: a heading error early in the interval rotates every later
    translation, so the error compounds rather than adds, and that coupling
    lives in J1's off-diagonal. The previous run measured the difference at
    1.5 cm summed against 4.2 cm true over 2 s.
    """
    c, s = math.cos(delta.theta), math.sin(delta.theta)
    dx, dy = step.x, step.y
    j1 = np.array([[1.0, 0.0, -s * dx - c * dy],
                   [0.0, 1.0,  c * dx - s * dy],
                   [0.0, 0.0, 1.0]])
    j2 = np.array([[c, -s, 0.0],
                   [s,  c, 0.0],
                   [0.0, 0.0, 1.0]])
    cov = j1 @ np.asarray(delta_cov, dtype=float) @ j1.T \
        + j2 @ np.asarray(step_cov, dtype=float) @ j2.T
    return delta.compose(step), cov


@dataclass(frozen=True)
class ScaleMeasurement:
    """What a bag says about the odometry distance scale."""

    scale: float                 # true distance / odometry distance
    lidar_closing_m: float
    odom_distance_m: float
    n_frames: int
    bag: str

    @property
    def implied_gain(self) -> float:
        """speed_to_erpm_gain that would make this bag's odometry true."""
        return 5499.271647286143 / self.scale


def measure_odom_scale(replay, *, beam_halfwidth: int = 3,
                       min_travel_m: float = 2.0) -> ScaleMeasurement:
    """Measure the odometry distance scale from a straight drive at a wall.

    The vehicle drives at a flat end wall; the lidar's forward beam reports how
    far the wall actually receded, and /odom reports how far the vehicle
    thought it went. Their ratio is the scale error, measured with no SLAM, no
    filter and no second sensor in the loop.

    Only valid on a recording that is a straight run at a surface square to the
    path -- it takes the raw forward beam, so a yaw during the run projects
    into it. Use it on drive_stop_2m_from_wall-style recordings, and read the
    n_frames/travel it reports before believing the number.

    This is deliberately NOT an extraction-based measurement: at this point in
    the build the fitted-line path is what we are trying to validate, and
    calibrating it against itself would prove nothing.
    """
    frames = list(replay)
    if len(frames) < 2:
        raise ValueError('need at least two frames')

    def forward(frame) -> float:
        i = int(round((0.0 - frame.angles[0]) / (frame.angles[1] - frame.angles[0])))
        lo, hi = max(0, i - beam_halfwidth), i + beam_halfwidth + 1
        window = frame.ranges[lo:hi]
        window = window[np.isfinite(window)]
        return float(np.median(window)) if window.size else float('nan')

    first, last = forward(frames[0]), forward(frames[-1])
    odom_distance = sum(f.ds for f in frames)
    if not (math.isfinite(first) and math.isfinite(last)):
        raise ValueError('no forward return at one end of the recording')
    if odom_distance < min_travel_m:
        raise ValueError(f'only {odom_distance:.2f} m travelled; need {min_travel_m}')
    closing = first - last
    return ScaleMeasurement(scale=closing / odom_distance, lidar_closing_m=closing,
                            odom_distance_m=odom_distance, n_frames=len(frames),
                            bag=str(getattr(replay, 'db', '?')))
