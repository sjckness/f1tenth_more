"""
base_front: the front bumper frame, the reference of every front distance.

The expanded URDF -- what robot_state_publisher publishes on /tf_static --
and vehicle_frames.base_front_xyz() -- what Python reads without TF -- must be
the same number, on the car's configuration and on the sim's.
"""

import os
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from f1tenth_description.vehicle_frames import (
    BASE_FRONT_FRAME, base_front_x, base_front_xacro_path, base_front_xyz)
import pytest

SHARE = get_package_share_directory('f1tenth_description')


def _expanded_joints(*args):
    urdf = subprocess.check_output(
        ['xacro', os.path.join(SHARE, 'urdf', 'roboracer.urdf.xacro'),
         f'pkg_share:={SHARE}', *args], text=True)
    return {j.get('name'): j for j in ET.fromstring(urdf).iter('joint')}


@pytest.mark.parametrize('args', [
    ('use_sim:=false', 'enable_sensors:=false', 'control_config:='),   # the car
    ('use_sim:=true', 'enable_sensors:=true', 'enable_camera_mock:=true',
     'control_config:='),                                             # the sim
])
def test_the_published_frame_is_the_number_python_reads(args):
    joint = _expanded_joints(*args)['base_front_joint']
    assert joint.get('type') == 'fixed'
    assert joint.find('parent').get('link') == 'base_link'
    assert joint.find('child').get('link') == BASE_FRONT_FRAME
    xyz = [float(v) for v in joint.find('origin').get('xyz').split()]
    assert xyz == pytest.approx(list(base_front_xyz()))


def test_the_front_bumper_is_0_443_ahead_of_base_link():
    """The URDF chassis-mesh value; change it here and in base_front.xacro together."""
    assert base_front_xyz() == pytest.approx((0.443, 0.0, 0.0))
    assert base_front_x() == pytest.approx(0.443)


def test_the_installed_file_is_the_one_parsed():
    assert os.path.basename(base_front_xacro_path()) == 'base_front.xacro'
    assert os.path.isfile(base_front_xacro_path())


def test_a_definition_missing_an_axis_is_refused(tmp_path):
    broken = tmp_path / 'base_front.xacro'
    broken.write_text(
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">'
        '<xacro:property name="base_front_x" value="0.443"/></robot>')
    with pytest.raises(ValueError):
        base_front_xyz(str(broken))
