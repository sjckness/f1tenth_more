"""The ZED 2i contract (config/camera_zed2i.yaml) and the camera extrinsics Hydra needs.

Pure Python + numpy: no ROS, so the launch file and the tests can both use it.
"""

import math
import os

import numpy as np
import yaml


def config_dir():
    """The package's config directory: the install share, or the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('f1tenth_hydra'), 'config')
    except Exception:  # noqa: BLE001 - not installed: running from the source tree
        return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')


def load_yaml(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def load_camera_contract(path=None):
    return load_yaml(path or os.path.join(config_dir(), 'camera_zed2i.yaml'))


def _quat_xyzw_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _rpy_to_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def _tf(rot=None, xyz=(0.0, 0.0, 0.0)):
    m = np.eye(4)
    if rot is not None:
        m[:3, :3] = rot
    m[:3, 3] = xyz
    return m


def _matrix_to_quat_wxyz(r):
    """Rotation matrix -> unit quaternion (w, x, y, z), w >= 0."""
    t = np.trace(r)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s]
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        q = [(r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s]
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        q = [(r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        q = [(r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    q /= np.linalg.norm(q)
    return q if q[0] >= 0 else -q


def base_T_left_optical(contract):
    """4x4 pose of the left optical frame in base_link (mount + ZED URDF chain)."""
    m, c = contract['mount'], contract['chain']
    return (_tf(_quat_xyzw_to_matrix(m['quat_xyzw']), m['xyz'])
            @ _tf(xyz=c['link_to_center_xyz'])
            @ _tf(xyz=c['center_to_left_xyz'])
            @ _tf(_rpy_to_matrix(*c['left_to_optical_rpy'])))


def lens_midpoint(contract):
    """Lens-plane midpoint in base_link: the point Isaac's lens_center_xyz places."""
    m, c = contract['mount'], contract['chain']
    link = _tf(_quat_xyzw_to_matrix(m['quat_xyzw']), m['xyz'])
    mid = np.array(c['link_to_center_xyz']) + np.array([c['center_to_left_xyz'][0], 0.0,
                                                        c['center_to_left_xyz'][2]])
    return (link @ np.append(mid, 1.0))[:3]


def hydra_param_extrinsics(contract):
    """Hydra `extrinsics: {type: param, ...}` block for base_link -> left optical."""
    t = base_T_left_optical(contract)
    w, x, y, z = (float(v) for v in _matrix_to_quat_wxyz(t[:3, :3]))
    return {
        'type': 'param',
        'body_R_sensor': {'w': round(w, 9), 'x': round(x, 9), 'y': round(y, 9), 'z': round(z, 9)},
        'body_p_sensor': [round(float(v), 6) for v in t[:3, 3]],
    }
