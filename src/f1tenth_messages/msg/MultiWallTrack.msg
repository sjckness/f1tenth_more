# One wall track of f1tenth_perception's multi_wall_tracker (lidar_front_wall_node,
# use_wall_tracker). Named MultiWallTrack because WallTrack.msg is already
# MPC_corr's wall_turn feed (/mpc/wall_track).
#
# A wall is the line rho = x cos(theta) + y sin(theta), rho >= 0, so theta is
# the normal pointing from the frame origin toward the line.

uint32 id                   # never reused
string label                # front | left | right | back: the car-to-wall normal vs the heading, +-45 deg sectors
uint8 state
uint8 STATE_TENTATIVE = 0   # seen, not yet confirmed (M of N hits)
uint8 STATE_CONFIRMED = 1   # matched on this scan
uint8 STATE_COASTING = 2    # confirmed earlier, not matched on this scan; covariance growing

float32 rho                 # [m] odom
float32 theta               # [rad] odom
float32 rho_bl              # [m] base_link, at the scan stamp
float32 theta_bl            # [rad] base_link, at the scan stamp
geometry_msgs/Point p0      # fitted extent endpoints, odom (z = 0)
geometry_msgs/Point p1
geometry_msgs/Point p0_bl   # the same endpoints in base_link at the scan stamp
geometry_msgs/Point p1_bl
float64[4] cov              # row-major 2x2 covariance of (rho, theta), odom

uint32 hits                 # scans the track was matched on
float32 age                 # [s] since the track was born
float32 d_perp_front        # [m] perpendicular distance from base_front to the line
float32 d_ray_front         # [m] along the heading ray from base_front to the segment; NaN if it misses
