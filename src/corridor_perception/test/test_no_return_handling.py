"""T12 -- no-return handling. A real bug that reached the glass recording.

The driver's no-return value is 65.533 m and is FINITE, so `isfinite` does not
detect it and a raw array fits walls 65 m away. These tests assert the three
things that stop that: the sentinel is recognised, no module accepts a raw
array, and no surface is ever produced at that distance.
"""

import numpy as np
import pytest

from corridor_perception.extraction import extract_segments
from corridor_perception.scan import (NO_RETURN_SENTINEL_M, CleanRanges,
                                      clean_ranges, require_clean)


def test_the_sentinel_is_treated_as_no_return():
    r = clean_ranges([1.0, NO_RETURN_SENTINEL_M, 2.0], 0.02, 30.0)
    assert np.isinf(r[1])
    assert np.isfinite(r[0]) and np.isfinite(r[2])


def test_a_sentinel_below_a_generous_ceiling_is_still_rejected():
    """A caller passing range_max=100 must not thereby admit the sentinel."""
    assert np.isinf(clean_ranges([NO_RETURN_SENTINEL_M], 0.02, 100.0)[0])


def test_short_and_nonfinite_returns_are_rejected_too():
    r = clean_ranges([0.001, np.nan, np.inf, 5.0], 0.02, 30.0)
    assert list(np.isfinite(r)) == [False, False, False, True]


def test_extraction_refuses_a_raw_range_array():
    angles = np.linspace(-1.0, 1.0, 50)
    raw = np.full(50, 2.0)
    with pytest.raises(TypeError, match='raw range array'):
        extract_segments(raw, angles)


def test_the_guard_survives_slicing_a_window_of_beams():
    """Consumers take windows of beams; a view must stay clean."""
    r = clean_ranges(np.full(100, 3.0), 0.02, 30.0)
    assert isinstance(r[10:20], CleanRanges)
    require_clean(r[10:20], 'test')


def test_replayed_frames_are_clean_and_fit_no_distant_surface(straight_traverse):
    for f in straight_traverse[::200]:
        require_clean(f.ranges, 'bag frame')
        for s in extract_segments(f.ranges, f.angles):
            assert s.rho < 30.0, f'surface fitted at rho={s.rho:.1f} m'


def test_the_glass_recording_is_clean_too(glass_scans):
    """The npz carries raw driver output and is where this bug actually landed."""
    for ranges, angles in glass_scans:
        require_clean(ranges, 'glass npz')
        assert max((s.rho for s in extract_segments(ranges, angles)), default=0.0) < 30.0
