"""Pure frame math for the camera pan (no ROS) -- unit tested in test_camera_pan_tf.py.

Chain: base_link --(static pivot)--> camera_pan_base --(yaw)--> zed2_camera_link.
At yaw=0 the composed base_link -> zed2_camera_link is the old static transform
(pivot xyz, identity rotation), so pan=0 is bit-identical to today.
"""
import math

PAN_BASE_FRAME = 'camera_pan_base'
CAMERA_FRAME = 'zed2_camera_link'


def yaw_to_quat(yaw: float):
    """Return the (x, y, z, w) quaternion for a pure yaw about +z."""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def base_to_pan_base(pivot_xyz, parent='base_link'):
    """Return the static base_link -> camera_pan_base TF content.

    The pivot translation; identity rotation.
    """
    return {
        'parent': parent, 'child': PAN_BASE_FRAME,
        'translation': tuple(float(v) for v in pivot_xyz),
        'rotation': (0.0, 0.0, 0.0, 1.0),
    }


def pan_base_to_camera(yaw: float):
    """Return the dynamic camera_pan_base -> zed2_camera_link TF content.

    Pure yaw, no translation (the camera rotates about the pivot); yaw=0 is
    identity.
    """
    return {
        'parent': PAN_BASE_FRAME, 'child': CAMERA_FRAME,
        'translation': (0.0, 0.0, 0.0),
        'rotation': yaw_to_quat(yaw),
    }


def compose_base_to_camera(pivot_xyz, yaw):
    """Return the composed base_link -> zed2_camera_link (the old static TF).

    Because camera_pan_base sits at the pivot and the camera has zero
    translation from it, the composition is just translation = pivot,
    rotation = yaw. Returns (translation xyz, quaternion xyzw).
    """
    return (tuple(float(v) for v in pivot_xyz), yaw_to_quat(yaw))


def joint_state_is_fresh(now_ns, stamp_ns, max_stale_ns):
    """Return True if a measurement stamped stamp_ns is still fresh at now_ns.

    Older than max_stale_ns -> stale; camera_pan_tf_node then STOPS publishing
    the pan TF rather than re-stamping an old angle (A2), so a stuck/laggy servo
    makes the TF disappear (fail-safe) instead of lying about where the camera
    points. A future stamp (clock skew) counts as fresh.
    """
    return (now_ns - stamp_ns) <= max_stale_ns


def rotate_point_to_base(pivot_xyz, yaw, point_in_camera):
    """Map a point expressed in zed2_camera_link into base_link.

    For tests that check the lens swings the right way: pure yaw about z, then
    + pivot.
    """
    px, py, pz = point_in_camera
    c, s = math.cos(yaw), math.sin(yaw)
    return (pivot_xyz[0] + c * px - s * py,
            pivot_xyz[1] + s * px + c * py,
            pivot_xyz[2] + pz)
