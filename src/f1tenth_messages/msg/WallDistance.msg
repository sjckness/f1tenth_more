# Tracked-wall geometry, from f1tenth_perception's wall_distance_node on
# /perception/d_wall. Published at the node's own rate, independent of whether
# any turn is active. valid=false means "no usable track", which is NOT the
# same as d_wall==0.
#
# SIGN CONVENTION: d_wall is the SIGNED perpendicular offset of the tracked
# wall from the car, LEFT POSITIVE -- a wall on the car's left gives
# d_wall > 0, a wall on its right gives d_wall < 0, and |d_wall| is the
# perpendicular distance. That is the stack's existing convention for every
# lateral quantity that carries a sign (the corridor normal
# n = (-sin psi, cos psi) in MPC_corr.build_straight_corridor, d_lat in
# mpc_solver.corridor_lateral_coordinates, swept_corridor's y-left frame, and
# every "+ = left" bearing field). It is NOT a new convention.
#
# IT DIFFERS FROM WallTrack.d_wall ON PURPOSE, and both topics can be live at
# once. WallTrack.d_wall is an unsigned MAGNITUDE (abs(), direction carried
# separately in normal_yaw) because the wall_turn increment rule only ever
# needed "how far". A proportional correction needs a signed error, so this
# message carries the sign in the value. Do not assume the two are
# interchangeable: they are not, and the difference is one sign flip away
# from steering into the wall.
#
# header.stamp is the observation the geometry was derived from (the scan
# stamp when provenance is GEOMETRIC or GLASS_CONFIRMED, the publish instant
# when PREDICTED); header.frame_id is base_link, the frame d_wall and
# heading_rel are measured in.
std_msgs/Header header

# STABLE WHILE THE TRACK IS TRUSTED -- NOT NECESSARILY ACROSS A WHOLE TURN, and
# the difference was measured rather than assumed. A change means EITHER a
# re-association (the fit landed on a different surface) OR a coast cap having
# fired: past wall_distance_max_coast_{distance,yaw} the stored line is no
# longer trusted, valid goes false for a tick, and a fresher confirmed track
# takes over with a new id. On 2026-09-10T15-33-46_mission-wall_turn, replayed
# offline, one 90 degree turn produced 6 ids at the shipping caps and 1 with the
# yaw cap lifted. That is the designed trade: the caps come from the measured
# odometry bias (distance 17-20% short, gyro yaw gain 0.93-1.09), and a 90
# degree turn genuinely cannot be coasted end to end at that quality.
# Either way the change is PUBLISHED, not hidden, so the consumer can reject
# the resulting step in d_wall instead of following it.
uint32  track_id
float32 d_wall            # signed perpendicular distance, metres, node frame convention
float32 heading_rel       # rad, vehicle heading minus wall direction
float32 fit_rms           # metres
uint16  inlier_count
float32 age               # seconds since last direct observation (0.0 = this scan)
uint8   provenance        # 0 NONE, 1 GEOMETRIC, 2 GLASS_CONFIRMED, 3 PREDICTED
uint8   phase             # 0 UNKNOWN, 1 WALL_TURN, 2 COMMITTED, 3 CORRIDOR
bool    valid

# Named constants for the two enumerations above. wall_distance.py mirrors
# these and test_wall_distance.py asserts they stay equal.
uint8 PROVENANCE_NONE = 0            # no usable track: valid is false
uint8 PROVENANCE_GEOMETRIC = 1       # fitted from this scan's returns, no glass evidence
uint8 PROVENANCE_GLASS_CONFIRMED = 2 # fitted AND confirmed by glass_detect's evidence
uint8 PROVENANCE_PREDICTED = 3       # no returns this scan: stored line carried on odometry

uint8 PHASE_UNKNOWN = 0   # no positive signal. Startup, and every ambiguous silence.
uint8 PHASE_WALL_TURN = 1 # /mpc/wall_track arriving, psi_commit NaN: issued, not yet bending
uint8 PHASE_COMMITTED = 2 # /mpc/wall_track arriving, psi_commit finite: bending
uint8 PHASE_CORRIDOR = 3  # clean COMMITTED -> silence with a valid track held throughout
