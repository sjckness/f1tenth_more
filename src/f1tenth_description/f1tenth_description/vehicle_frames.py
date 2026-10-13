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
import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

__all__ = ['BASE_FRONT_FRAME', 'MIN_FRONT_DISTANCE_M', 'base_front_x', 'base_front_xyz',
           'base_front_xacro_path', 'beside_body_behind_bumper', 'front_side_limit_m']

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


def front_side_limit_m():
    """
    Return |y| [m] beyond which a point behind the bumper face is BESIDE the car.

    The body half-width plus a margin, both from stack_params.yaml
    (swept_clearance_body_half_width_m + front_beside_body_margin_m). Behind
    the bumper face and inside this band a point is touching the car (the
    front readings clamp it to MIN_FRONT_DISTANCE_M); outside it, the point is
    beside the car and is not a front reading at all.
    """
    from f1tenth_params.param_defaults import get_value
    return (float(get_value('swept_clearance_body_half_width_m'))
            + float(get_value('front_beside_body_margin_m')))


def beside_body_behind_bumper(x, y, r=0.0, *, front_x, side_limit_m,
                              corridor_half_width_m=math.inf):
    """
    Tell whether a footprint in base_link is beside the car, not in front of it.

    The footprint is a disc (x, y, radius r; r = 0 for a point). It is beside
    the car, and must be EXCLUDED from every front distance (2026-10-13), when
    its near face is at or behind the bumper face (x - r <= front_x) and it
    reaches neither
      - the contact band behind the bumper face inside the car's width
        (X <= front_x, |Y| <= side_limit_m): touching, so it still counts
        and reads MIN_FRONT_DISTANCE_M; nor
      - the corridor ahead of the bumper face (X > front_x,
        |Y| < corridor_half_width_m): a wide object beside the nose that also
        sticks out in front of it is still in the way.
    For a point this is simply: x <= front_x and |y| > side_limit_m.
    """
    if x - r > front_x:
        return False

    def _reaches(cx, cy):
        return math.hypot(x - cx, y - cy) <= r

    if _reaches(min(x, front_x), max(-side_limit_m, min(y, side_limit_m))):
        return False
    if (x + r > front_x and abs(y) - r < corridor_half_width_m
            and _reaches(max(x, front_x),
                         max(-corridor_half_width_m, min(y, corridor_half_width_m)))):
        return False
    return True
