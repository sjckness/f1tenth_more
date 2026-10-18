# One wall track of f1tenth_perception's multi_wall_tracker (lidar_front_wall_node,
# use_wall_tracker). Named MultiWallTrack because WallTrack.msg is already
# MPC_corr's wall_turn feed (/mpc/wall_track).
#
# A wall is the line rho = x cos(theta) + y sin(theta), rho >= 0, so theta is
# the normal pointing from the frame origin toward the line.

uint32 id                   # never reused
string label                # front | back | left | right | other -- the GEOMETRIC role
                            # (multi_wall_tracker.geometric_role), not a bearing sector
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

# --- since 2026-10-15 (step 4: the tracker consumes wall observations) -------
# Additive only; every field above keeps its meaning.
string role                 # front | back | left | right | other, WITH HYSTERESIS
                            # at the thresholds so a wall on one does not flip
                            # every scan. 'other' is a first-class answer: the
                            # track is real, tracked and drawn, and is a
                            # reference wall for nothing -- because the
                            # geometry puts it nowhere near the car's path,
                            # because the rectangle REJECTED it, or because it
                            # is behind another confirmed wall. Same value as
                            # `label`, kept for compatibility; prefer this one.
uint8 source                # the BEST evidence this track has ever had
uint8 SOURCE_LIDAR = 0
uint8 SOURCE_STEREO = 1     # nothing has ever cross-checked this wall
uint8 SOURCE_BOTH = 2
float32 confidence          # [0, 1] of the observation that last matched it
float32 z_min               # vertical extent [m] above the floor; the default
float32 z_max               # 0 .. wall_default_height_m unless height_measured
bool height_measured        # false: z_min/z_max are assumed, not seen
float32 d_side              # [m] from the car's SIDE EDGE (not its centreline)
                            # to the wall, for a left/right role; NaN otherwise
bool is_front               # this track IS the wall in the way: confirmed, its
                            # segment crosses the car-width strip ahead of the
                            # bumper, and (if stereo-only) square to the
                            # heading. NOT the same as role == 'front', which
                            # is only about the normal's bearing.

