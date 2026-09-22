"""Fixtures over RECORDED data. Nothing here generates a scan or a pose.

The recordings live outside the repo, under ~/f1tenth_archive, so every fixture
that needs one skips when it is absent rather than failing: a checkout on
another machine still runs the tests that need no vehicle data. Set
CORRIDOR_ARCHIVE to point somewhere else.
"""

import math
import os
import pathlib

import pytest

ARCHIVE = pathlib.Path(
    os.environ.get('CORRIDOR_ARCHIVE', pathlib.Path.home() / 'f1tenth_archive' / 'complete'))

# A straight run at a flat end wall: 15.0 m of odometry, 19 deg of yaw range,
# 1240 scans, and the end wall visible from the first frame. The reference
# recording for anything about driving down a corridor.
STRAIGHT_TRAVERSE = '2026-09-11T16-11-08_mission-drive_stop_2m_from_wall'

# Five real scans of a glazed corridor, in-repo, no poses and no motion.
GLASS_NPZ = (pathlib.Path(__file__).resolve().parents[2]
             / 'f1tenth_perception' / 'test' / 'data' / 'glass_corridor_ust10lx.npz')


def chi2_mean_bounds(dof: int, n: int, p: float = 0.99) -> tuple[float, float]:
    """Two-sided bounds on the MEAN of n chi-square(dof) samples.

    n * mean ~ chi2(n * dof); Wilson-Hilferty keeps the tests numpy-only.
    """
    z = {0.95: 1.959964, 0.99: 2.575829}[p]
    k = n * dof

    def quantile(zz: float) -> float:
        return k * (1.0 - 2.0 / (9.0 * k) + zz * math.sqrt(2.0 / (9.0 * k))) ** 3

    return quantile(-z) / n, quantile(z) / n


@pytest.fixture
def chi2_bounds():
    return chi2_mean_bounds


def _require(path: pathlib.Path, what: str) -> pathlib.Path:
    if not path.exists():
        pytest.skip(f'recording not available: {what} ({path})')
    return path


@pytest.fixture(scope='session')
def straight_traverse_path():
    """The straight-traverse recording, skipping if the archive is absent."""
    return _require(ARCHIVE / STRAIGHT_TRAVERSE, 'straight traverse')


@pytest.fixture(scope='session')
def straight_traverse(straight_traverse_path):
    """Frames of the straight traverse, read once for the whole session."""
    from corridor_perception.replay import BagReplay
    return list(BagReplay(straight_traverse_path))


@pytest.fixture(scope='session')
def glass_scans():
    """The five recorded glass-corridor scans as (ranges, angles) pairs."""
    import numpy as np
    from corridor_perception.replay import clean_ranges
    path = _require(GLASS_NPZ, 'glass corridor npz')
    d = np.load(path)
    ranges = d['ranges']
    angles = float(d['angle_min']) + float(d['angle_increment']) * np.arange(ranges.shape[1])
    # The npz carries raw driver output, sentinels and all, and has no
    # LaserScan header to carry its own limits: apply the same rule replay
    # applies to a bag, or extraction fits walls at 65 m.
    return [(clean_ranges(r, float(d['range_min']), float(d['range_max'])), angles)
            for r in ranges]


@pytest.fixture(scope='session')
def straight_traverse_filtered(straight_traverse_path):
    """The same traverse, driven off /odometry/filtered instead of /odom.

    Raw /odom accumulates ~19 deg of yaw over this 15 m straight run that
    neither SLAM (+1.6 deg), the global EKF (+1.2 deg) nor the lidar itself
    sees. The local EKF agrees with them, is continuous and is odom-framed, so
    it is the motion source anything tracking surfaces has to use on this
    hardware. See test_axis.py's pair of axis-stability tests.
    """
    from corridor_perception.replay import BagReplay
    return list(BagReplay(straight_traverse_path, odom_topic='/odometry/filtered'))
