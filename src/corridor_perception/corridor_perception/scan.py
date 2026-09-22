"""The single entry point for scan data, and the type that proves it was used.

The UST driver does not publish inf for "no return". It publishes 65.533 m, the
uint16 millimetre ceiling, for 20-24% of the returns in an office corridor.
Nothing in a real scan is ever inf, so `isfinite` is not a no-return test, and
code that believes it is will fit a wall 65 m away. That is not hypothetical:
it reached the glass recording.

One filter, one place. `clean_ranges()` is it. What it returns is a CleanRanges,
a marker subclass of ndarray, and the pipeline entry points refuse anything
else -- an assertion rather than a convention, because a convention is exactly
what failed. The cost is that a caller synthesising a range array for a unit
test has to say which limits it is synthesising against, which is a feature: a
range array means nothing without them.

numpy only; nothing here imports ROS.
"""

from __future__ import annotations

import numpy as np

# The driver's no-return value, and the bound that catches it. The sentinel is
# 65.533 (uint16 mm); the bound sits well below it and well above the ~20 m of
# real returns these recordings contain. It is a guard against a sentinel, NOT
# a sensor horizon: nothing in this package may treat it as one. Acquirability
# is decided by the presence or absence of returns, never by a distance.
NO_RETURN_SENTINEL_M = 65.533
NO_RETURN_MIN_M = 40.0


class CleanRanges(np.ndarray):
    """Ranges that have been through clean_ranges(). Carries no data of its own.

    A view-cast marker: slicing or viewing one keeps the type, which is what
    lets a consumer take a window of beams and still satisfy the assertion.
    Arithmetic that produces a new array does not, and should not -- a derived
    quantity is no longer a set of measured ranges.
    """


def clean_ranges(ranges, range_min: float, range_max: float) -> CleanRanges:
    """Invalid returns to inf. The ONE place a return is judged usable.

    Invalid is: non-finite, below the sensor's own minimum, or at/above the
    no-return bound. After this, and only after this, `isfinite` IS the
    no-return test, which is why everything downstream may rely on it.
    """
    r = np.asarray(ranges, dtype=float)
    ceiling = min(float(range_max), NO_RETURN_MIN_M)
    bad = ~np.isfinite(r) | (r < float(range_min)) | (r >= ceiling)
    return np.where(bad, np.inf, r).view(CleanRanges)


def require_clean(ranges, caller: str) -> np.ndarray:
    """Refuse a range array that did not come through clean_ranges().

    P1 and T12: no module accepts a raw range array. Raised rather than
    asserted so that running under `python -O` cannot silently disable it.
    """
    if not isinstance(ranges, CleanRanges):
        raise TypeError(
            f'{caller} was given a raw range array. Every scan enters through '
            'corridor_perception.scan.clean_ranges(); a raw array still '
            'carries the 65.533 m no-return sentinel as a finite value and '
            'will be fitted as a surface.')
    return np.asarray(ranges, dtype=float)
