"""
IsObstacleDetected: stop_distance is measured from the front bumper.

The behaviour is constructed directly (its __init__ needs no rclpy) and fed
Detection3DArray messages; the camera frame's pose comes from its own
/tf_static buffer, as on the car. The lane is off by default
(enable_camera_obstacle_stop), which is no reason to leave it untested.
"""

from f1tenth_behavior.behaviours.is_obstacle_detected import IsObstacleDetected
from f1tenth_description.vehicle_frames import base_front_x
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from vision_msgs.msg import Detection3D, Detection3DArray

FRONT = base_front_x()
CAMERA_FRAME = 'zed2_left_camera_frame'
#: The car's committed TF: the camera 0.12 m ahead of base_link.
LENS_BEHIND_M = 0.323


def _behaviour(camera_x=FRONT - LENS_BEHIND_M, **kwargs):
    behaviour = IsObstacleDetected(**kwargs)
    if camera_x is not None:
        t = TransformStamped()
        t.header.frame_id = 'base_link'
        t.child_frame_id = CAMERA_FRAME
        t.transform.translation.x = float(camera_x)
        t.transform.rotation.w = 1.0
        behaviour._frames.on_tf_static(TFMessage(transforms=[t]))
    return behaviour


def _detections(*xs, y=0.0, z=0.0):
    msg = Detection3DArray()
    msg.header.frame_id = CAMERA_FRAME
    for x in xs:
        det = Detection3D()
        det.bbox.center.position.x = float(x)
        det.bbox.center.position.y = float(y)
        det.bbox.center.position.z = float(z)
        msg.detections.append(det)
    return msg


def test_stop_distance_is_counted_from_the_bumper_not_the_lens():
    behaviour = _behaviour(stop_distance=1.0)
    # 1.2 m from the lens = 0.877 m from the bumper: within 1.0 now (the old
    # lens-referenced check did not stop for it).
    behaviour._detections_callback(_detections(1.2))
    assert behaviour.obstacle_present is True
    # 1.4 m from the lens = 1.077 m from the bumper: beyond it.
    behaviour._detections_callback(_detections(1.4))
    assert behaviour.obstacle_present is False


def test_anything_behind_the_lens_never_counts():
    behaviour = _behaviour(stop_distance=1.0)
    behaviour._detections_callback(_detections(-0.1))
    assert behaviour.obstacle_present is False


def test_the_lateral_corridor_is_unchanged():
    behaviour = _behaviour(stop_distance=1.0, corridor_half_width=0.32)
    behaviour._detections_callback(_detections(0.8, y=0.5))
    assert behaviour.obstacle_present is False


def test_without_the_camera_transform_it_can_only_trip_early():
    """The lens is assumed at base_link: 1.4 m reads 1.4 - 0.443 from the bumper."""
    behaviour = _behaviour(camera_x=None, stop_distance=1.0)
    behaviour._detections_callback(_detections(1.4))
    assert behaviour.obstacle_present is True
