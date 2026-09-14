# d_wall: the wall a wall_turn was triggered by, tracked as a line in the odom
# frame by mpc_controller's wall_tracker.py, reported as the FRONT BUMPER's
# perpendicular distance to it. Published by MPC_corr on /mpc/wall_track once
# per control tick while a wall_turn drive command is active, valid or not, so
# silence means "no wall_turn active" (or a dead node), never "no wall".
#
# It feeds d_avail in the wall_turn increment rule (mpc_controller/wall_turn.py)
# and nothing else. It is NOT a safety signal and does not replace
# /perception/front_distance, which keeps driving the safety layer and the
# commit decision itself. See wall_tracker.py's module docstring for the
# selection gates and the tracking rule.
#
# header.stamp is the control tick; header.frame_id is the odom frame.

std_msgs/Header header

bool valid                          # a wall is selected and d_wall is a distance
uint8 provenance
uint8 PROVENANCE_NONE = 0           # no wall selected: before commit, or nothing passed the gates
uint8 PROVENANCE_MEASURED = 1       # the line was refitted from a fresh scan on this control tick
uint8 PROVENANCE_DEAD_RECKONED = 2  # this tick could not refit (no fresh scan, or the refit degraded): stored line + odometry

float32 d_wall              # [m] bumper to the tracked line, perpendicular; NaN when not valid
float32 normal_yaw          # [rad] the line's normal in odom, car -> wall; NaN when not valid
float32 psi_commit          # [rad] heading frozen at commit, the gating reference; NaN before commit
uint16 inlier_count         # returns on the line at the last (re)fit
float32 span_m              # [m] extent of those returns along the line; NaN when none
float32 since_last_fit_sec  # [s] since the last successful (re)fit; NaN when none
