"""T1 -- yaw invariance, the original bug. BLOCKED: the recording does not exist.

The previous version of this file asserted T1 against a simulated corridor and
simulated yaw. That is exactly the evidence the brief rejects: it exercised the
fitting algebra, not the sensor, the mount or the real returns, and it passed
while the mount in the URDF was 180 degrees away from the mount every recording
was actually made with (see replay.SENSOR_IN_BASE_NOTE). It is deleted rather
than ported.

WHAT THIS TEST NEEDS, and nothing in ~/f1tenth_archive contains it:

    TWO STATIONARY YAW SWEEPS, at roughly 4 m and roughly 8 m from a flat end
    wall. Two, because the rho covariance grows with distance and the
    association gate behaves differently at each: one well-conditioned range
    and one poorly-conditioned one. The exact distances need not be known.

    Park facing the end wall. Do NOT translate. Rotate in place through at
    least +-60 degrees, slowly enough that a few hundred scans land across the
    sweep. Record /scan, /odom, /odometry/filtered and /tf_static throughout.

    The truth is obtained by construction: the vehicle does not translate, so
    the distance to the end wall is constant whatever the heading. That is what
    makes this stronger than the simulated version -- it uses the real sensor,
    the real mount and the real returns, and needs no ground-truth measurement.

    Surveyed: of 141 archived runs, 107 carry both /scan and /odom, and NONE is
    a stationary yaw sweep. The largest yaw range among runs that stayed within
    0.5 m of their start is 13.2 degrees, against the >= 120 degrees this needs.

Then the assertions are:
  * the published FRONTAL distance varies by less than 5 cm across the sweep;
  * the goal predicate never fires;
  * naive_frontal_range() below DOES cross the threshold during the sweep, so
    the test documents the bug it protects against rather than merely passing.
"""

import numpy as np
import pytest


def naive_frontal_range(ranges, angles, half_angle: float = np.radians(15.0)) -> float:
    """min(range) over a frontal cone -- THE BUG, kept only as a test baseline.

    P1 forbids this quantity in any navigation predicate. It exists here so T1
    can assert that the naive measure fails on the same data where the fitted
    one holds; nothing outside a test may call it.
    """
    r = np.asarray(ranges, dtype=float)
    a = np.asarray(angles, dtype=float)
    cone = np.abs(a) <= half_angle
    valid = cone & np.isfinite(r)
    return float(np.min(r[valid])) if valid.any() else float('inf')


@pytest.mark.skip(reason='needs a stationary yaw-sweep recording; none exists '
                         '(see this module docstring for what to record)')
def test_fitted_frontal_distance_survives_yaw():
    raise AssertionError('unreachable until the recording exists')
