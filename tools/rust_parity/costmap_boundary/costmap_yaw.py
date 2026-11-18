"""The yaw costmap_boundary_node._pose_cb takes from an Odometry pose."""

from f1tenth_costmap.costmap_boundary import yaw_from_quaternion


def yaw(q):
    return yaw_from_quaternion(q)
