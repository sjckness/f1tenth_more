"""
The car's front bumper offset, read from the URDF without TF.

urdf/base_front.xacro defines the base_front frame (fixed to base_link) and is
the one place the base_link -> front offset is written down. robot_state_
publisher puts it on /tf_static; this module parses the same file's
``base_front_x/y/z`` properties for code that needs the number without a TF
buffer -- launch files, pure geometry modules, parameter defaults, tools.

Pure Python (no rclpy); the file is found through the package's install share
directory, so it is the URDF robot_state_publisher actually loads.
"""

import functools
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

__all__ = ['BASE_FRONT_FRAME', 'MIN_FRONT_DISTANCE_M', 'base_front_x', 'base_front_xyz',
           'base_front_xacro_path']

#: The frame name, for TF lookups (robot_state_publisher publishes it).
BASE_FRONT_FRAME = 'base_front'

#: The smallest distance from base_front a REAL reading is published as [m].
#: Every front-distance consumer reads <= 0 as "no reading" (condition_eval's
#: front_clearance guard, wall_turn, the LiDAR wall seed), so a reading the
#: re-referencing puts at or behind the bumper -- touching -- must stay
#: positive, or the stop it should trigger is switched off instead.
MIN_FRONT_DISTANCE_M = 0.01

_XACRO_NS = 'http://www.ros.org/wiki/xacro'


def base_front_xacro_path():
    """Return the installed urdf/base_front.xacro path."""
    return os.path.join(get_package_share_directory('f1tenth_description'),
                        'urdf', 'base_front.xacro')


@functools.lru_cache(maxsize=None)
def base_front_xyz(path=None):
    """
    Return (x, y, z) [m] of base_front in base_link, from base_front.xacro.

    Raises ValueError when the file does not define all three properties as
    plain numbers -- a broken definition must not turn into a silent default.
    """
    path = path or base_front_xacro_path()
    root = ET.parse(path).getroot()
    props = {el.get('name'): el.get('value')
             for el in root.iter(f'{{{_XACRO_NS}}}property')}
    try:
        return tuple(float(props[f'base_front_{axis}']) for axis in ('x', 'y', 'z'))
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(
            f'{path} must define base_front_x/y/z as numbers: {exc!r}') from exc


def base_front_x(path=None):
    """Return the front bumper's x [m] ahead of base_link."""
    return base_front_xyz(path)[0]
