"""
Semantic-layer objects as obstacle circles in the odom frame. Pure Python, no ROS.

semantic_layer_node (f1tenth_costmap) publishes its object memory on
/costmap/semantic_objects as f1tenth_messages/SemanticObjectArray, in the MAP
frame: a remembered object is a fact about the world, and the world does not
move when localisation corrects itself. The MPC plans in ODOM. The two are
joined by the map -> odom edge the global EKF broadcasts, and that edge JUMPS
whenever a SLAM correction is folded in. So a consumer must re-express the
stored map positions through the CURRENT edge every cycle, never convert once
and cache odom coordinates: a cached circle would ride along with the jump
and end up where the object is not.

Lives here, in the dependency-free leaf package, because the producer
(f1tenth_costmap) and the consumer (mpc_controller) must agree on it and
neither may depend on the other; it needs nothing but math, so it does not
import the message either -- objects are duck-typed (the generated message
classes fit, so does any stand-in with the same attribute names).

THE map -> odom CONVENTION is MPC_corr's: map_to_odom = (x, y, yaw) is the
edge exactly as tf2 returns it for lookup_transform('map', 'odom', ...),
header.frame_id 'map', child_frame_id 'odom' -- the pose of the odom frame's
origin expressed in map. A map point p_map is then
p_odom = R(-yaw) (p_map - (x, y)), the same arithmetic as MPC_corr's
_pose_map_to_odom.
"""

import math

from f1tenth_params.object_geometry import class_margin_for

__all__ = [
    'MEMORY',
    'TRACKED',
    'map_point_to_odom',
    'map_to_odom_from_poses',
    'objects_to_odom_circles',
    'odom_point_to_map',
]

# f1tenth_messages/SemanticObject.msg's state constants, repeated because this
# package may not import the message (f1tenth_costmap's tests pin the match).
TRACKED = 0
MEMORY = 1


def map_point_to_odom(x_map, y_map, map_to_odom):
    """Express a map-frame point in the odom frame through map_to_odom (x, y, yaw)."""
    tf_x, tf_y, tf_yaw = map_to_odom
    c, s = math.cos(tf_yaw), math.sin(tf_yaw)
    dx, dy = float(x_map) - tf_x, float(y_map) - tf_y
    return c * dx + s * dy, -s * dx + c * dy


def odom_point_to_map(x_odom, y_odom, map_to_odom):
    """Express an odom-frame point in the map frame; the inverse of map_point_to_odom."""
    tf_x, tf_y, tf_yaw = map_to_odom
    c, s = math.cos(tf_yaw), math.sin(tf_yaw)
    x_odom, y_odom = float(x_odom), float(y_odom)
    return tf_x + c * x_odom - s * y_odom, tf_y + s * x_odom + c * y_odom


def map_to_odom_from_poses(base_in_map, base_in_odom):
    """
    Return map -> odom (x, y, yaw) from the same base_link pose in both frames.

    base_in_map: (x, y, yaw) of base_link in map (/ekf_global/odometry/filtered);
    base_in_odom: (x, y, yaw) in odom (/odometry/filtered), at the same time.
    T_map_odom = T_map_base * inverse(T_odom_base). For a consumer that has
    both poses but no TF buffer; a TF lookup of 'map' <- 'odom' is the same
    edge when one is available.
    """
    xm, ym, yawm = base_in_map
    xo, yo, yawo = base_in_odom
    yaw = math.atan2(math.sin(yawm - yawo), math.cos(yawm - yawo))
    c, s = math.cos(yaw), math.sin(yaw)
    return xm - (c * xo - s * yo), ym - (s * xo + c * yo), yaw


def objects_to_odom_circles(objects, map_to_odom, include_memory=True,
                            min_confidence=0.0, class_margins=None):
    """
    Turn SemanticObject-like objects into odom-frame obstacle circles [(x, y, r)].

    objects: e.g. SemanticObjectArray.objects -- each needs x, y (map),
        effective_radius_m, state, confidence and class_id.
    map_to_odom: (x, y, yaw), the CURRENT edge (module docstring).
    include_memory: False keeps TRACKED objects only.
    min_confidence: drop objects below this confidence (MEMORY decays to 0).
    class_margins: optional {class_id: metres} (or obstacle_class_margin_m's
        JSON text) added to the radius, the same per-class clearance
        obstacle_projector_node adds to /perception/obstacles_2d radii.

    r is effective_radius_m (+ margin): the base footprint, grown for a
    person in MEMORY by how far they may have walked since last seen.
    """
    circles = []
    for obj in objects:
        state = int(obj.state)
        if state == MEMORY and not include_memory:
            continue
        if float(obj.confidence) < float(min_confidence):
            continue
        x, y = map_point_to_odom(obj.x, obj.y, map_to_odom)
        r = float(obj.effective_radius_m) + class_margin_for(class_margins, str(obj.class_id))
        circles.append((x, y, r))
    return circles
