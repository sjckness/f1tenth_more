# One robust line fit to the forward LiDAR sector, from f1tenth_perception's
# lidar_front_wall_node on /perception/lidar_front_wall. Published for EVERY
# /scan, valid or not: a consumer that stops receiving it is looking at a dead
# node or a dead LiDAR, never at "no wall". See that node's module docstring
# for the thresholds behind each reason and the fit's known blind spots.
#
# header.stamp is the scan's own stamp; header.frame_id is base_link.

std_msgs/Header header

# true only when reason == REASON_OK. A rejected fit reports the FIRST check
# it failed, in the order listed.
bool valid
uint8 reason
uint8 REASON_OK = 0
uint8 REASON_NO_TRANSFORM = 1         # base_link <- scan frame not known yet
uint8 REASON_TOO_FEW_RETURNS = 2      # fewer valid returns than the inlier floor
uint8 REASON_LOW_INLIER_COUNT = 3     # line holds fewer inliers than the floor
uint8 REASON_LOW_INLIER_FRACTION = 4  # line is not the dominant surface
uint8 REASON_OBLIQUE = 5              # normal too far off the heading
uint8 REASON_WRONG_SURFACE = 6        # disagrees with the dead-reckoned wall

# The fitted line, filled whenever one could be fitted at all (also when valid
# is false, so a rejection shows what the fit landed on). NaN when none.
float32 distance          # base_link origin to the line, perpendicular [m]
float32 normal_angle      # car-to-wall normal bearing, base_link [rad], 0 ahead, + left
uint16 inlier_count
uint16 valid_returns      # returns inside the sector (NaN/Inf/out of range excluded)
uint16 sector_beams       # beams inside the sector, returns or not
float32 inlier_fraction   # inlier_count / valid_returns
float32 rms_residual      # of the inliers about the line [m]

# The wrong-surface test. NaN when it was not run: no prediction to test
# against (never measured, prediction aged out, odometry stale), or the fit had
# already failed a single-scan check.
float32 predicted_distance  # [m]
float32 innovation          # distance - predicted_distance [m]
