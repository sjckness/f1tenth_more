"""
The car's base_link -> laser static TF sits where the tape put the scan centre.

2026-10-13, measured on the car to the bumper face: scan centre 0.10 m behind
it. The bumper is base_front (base_front.xacro), so the laser's x must stay
base_front_x - 0.10; moving one without the other moves every LiDAR front
distance and the e-stop cone. The ZED's twin of this test is in
f1tenth_perception (test_zed_extrinsic.py).
"""

import os
import re

from ament_index_python.packages import get_package_share_directory
from f1tenth_description.vehicle_frames import base_front_x
import pytest

#: Scan centre -> bumper face, taped 2026-10-13.
LASER_BEHIND_BUMPER_M = 0.10


def _static_tf_args(path, name):
    src = open(path).read()
    m = re.search(rf"name='{name}',\s*arguments=\[([^\]]*)\]", src)
    assert m, f'{name} not found in {path}'
    return [a.strip().strip("'\"") for a in m.group(1).split(',') if a.strip()]


def test_the_laser_sits_0_10_behind_the_bumper():
    path = os.path.join(get_package_share_directory('f1tenth_description'),
                        'launch', 'description.launch.py')
    x, y, z, yaw, pitch, roll, parent, child = _static_tf_args(path, 'static_baselink_to_laser')
    assert (parent, child) == ('base_link', 'laser')
    assert float(x) == pytest.approx(base_front_x() - LASER_BEHIND_BUMPER_M, abs=1e-3)
    assert (float(y), float(z), float(yaw), float(pitch), float(roll)) == (0.0, 0.20, 0.0, 0.0, 0.0)
