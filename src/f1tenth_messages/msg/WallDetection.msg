# One detected wall/planar-surface segment.

float32 distance          # perpendicular distance from robot origin to plane [m]
float32 bearing            # angle to the wall's centroid, robot frame, +x forward, + = left [rad]
geometry_msgs/Vector3 normal    # unit normal, oriented back toward the robot
geometry_msgs/Point centroid    # centroid of the inlier points, robot frame
float32 width               # approximate lateral extent of the detected segment [m]
bool is_corner              # true if this wall was found alongside a second, roughly-perpendicular wall
