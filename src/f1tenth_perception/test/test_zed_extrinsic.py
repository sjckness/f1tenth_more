"""
The car's base_link -> zed2_camera_link static TF sits where the tape put the camera.

2026-10-13, measured on the car to the bumper face: the camera's front face
0.15 m behind it. zed2_camera_link is the mounting screw; on the ZED 2i the
front face is 0.0151 m ahead of it (zed2i.stl reaches +0.0251 m from a mesh
origin at screw_offset_x -0.01, zed_macro.urdf.xacro). The bumper is
base_front (f1tenth_description/urdf/base_front.xacro), so the camera's x must
stay base_front_x - 0.15 - 0.0151; moving one without the other moves every
camera front distance.
"""

import math
import os
import re

from ament_index_python.packages import get_package_share_directory
from f1tenth_description.vehicle_frames import base_front_x
import pytest

#: Camera front face -> bumper face, taped 2026-10-13.
ZED_FACE_BEHIND_BUMPER_M = 0.15
#: zed2_camera_link (mounting screw) -> front face of a ZED 2i.
ZED2I_SCREW_TO_FACE_M = 0.0151


def test_the_zed_front_face_sits_0_15_behind_the_bumper():
    path = os.path.join(get_package_share_directory('f1tenth_perception'),
                        'launch', 'camera.launch.py')
    src = open(path).read()
    m = re.search(r"name='static_baselink_to_zed2',\s*arguments=\[([^\]]*)\]", src)
    assert m, 'static_baselink_to_zed2 not found in camera.launch.py'
    x, y, z, yaw, pitch, roll, parent, child = [
        a.strip().strip("'\"") for a in m.group(1).split(',') if a.strip()]
    assert (parent, child) == ('base_link', 'zed2_camera_link')
    expected = base_front_x() - ZED_FACE_BEHIND_BUMPER_M - ZED2I_SCREW_TO_FACE_M
    assert float(x) == pytest.approx(expected, abs=1e-3)
    assert (float(y), float(z), float(yaw), float(roll)) == (0.0, 0.205, 0.0, 0.0)


def test_the_zed_mount_pitch_is_the_imu_measured_nose_up():
    """2026-10-15: the ZED IMU in the static 1 m calib clip says 2.083 deg nose up.

    REP-103: positive pitch about y is nose down, so nose up is negative. The
    positional static_transform_publisher order is x y z YAW PITCH ROLL.
    """
    path = os.path.join(get_package_share_directory('f1tenth_perception'),
                        'launch', 'camera.launch.py')
    m = re.search(r"name='static_baselink_to_zed2',\s*arguments=\[([^\]]*)\]",
                  open(path).read())
    assert m, 'static_baselink_to_zed2 not found in camera.launch.py'
    pitch = float(m.group(1).split(',')[4].strip().strip("'\""))
    assert math.degrees(pitch) == pytest.approx(-2.083, abs=0.005)


def test_the_wrapper_is_launched_as_the_zed2i_it_is():
    """The 0.0151 m screw-to-face offset is the ZED 2i's; the model must match."""
    path = os.path.join(get_package_share_directory('f1tenth_perception'),
                        'launch', 'camera.launch.py')
    assert re.search(r"'camera_model':\s*'zed2i'", open(path).read())
