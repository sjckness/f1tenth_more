# mpc_corr's view of an in-progress object approach, on /mpc/object_status.
#
# A new message rather than a reuse: MpcSolverStatus is about the QP (cost,
# iterations, saturation) and none of these fields belong to it. Everything
# here is the approach's own geometry, and it exists because that geometry is
# otherwise only visible in log prose -- psi_c in particular is a HELD state
# that survives across rebuilds, so a run cannot be read back from the pose
# and the target alone.
std_msgs/Header header    # stamp: when mpc_corr computed this (its own clock).
                          # frame_id: the frame r/bearing/psi_c are expressed
                          # in -- the odom frame, since that is where the
                          # corridor and the solver live.
string move_id            # echoes the ObjectGoal that is driving this
string target_class       # echoes the ObjectGoal, for log correlation
#
# LIVE vs REBUILD fields. r, bearing, e and the four flag fields are computed
# EVERY control tick from the live pose. psi_c is the held heading in force.
# k and dpsi_max describe the last corridor REBUILD and are republished
# unchanged between rebuilds.
float32 r                 # LIVE range to the STANDOFF point: distance to
                          # target minus standoff. Negative means the vehicle
                          # is already inside the standoff. The single range
                          # source for the mission's object_reached condition.
float32 bearing           # LIVE absolute heading from the vehicle to the
                          # target [rad], odom frame
float32 psi_c             # the HELD corridor heading [rad], odom frame. Not
                          # derivable from anything else published.
float32 alpha             # LIVE wrap(bearing - vehicle yaw): where the target
                          # is relative to the car's heading [rad]. |alpha| at
                          # the end of a move is its arrival bearing error.
float32 e                 # LIVE wrap(bearing - psi_c): how far the held
                          # heading still has to rotate. Measured against
                          # psi_c, NOT against live yaw -- see object_approach.
float32 k                 # REBUILD: distance schedule in [0, 1], the fraction
                          # of e the last rebuild asked for
float32 dpsi_max          # REBUILD: the kinematic cap that was in force [rad]
bool inside_turn_radius   # ADVISORY, never an outcome: the goal is inside a
                          # full-lock circle by more than the heading margin.
                          # Only evaluated while r > object_r_freeze_m.
bool target_behind        # raw, this tick: the target is in the rear
                          # half-plane
float32 target_behind_for_s  # how long target_behind has held continuously
bool target_behind_terminal  # TERMINAL: target_behind has held for at least
                          # object_behind_persist_sec. The mission ends the
                          # move target_unreachable on this, not on the raw
                          # flag.
float32 target_age_s      # now minus the ObjectGoal header stamp. The whole
                          # point of that stamp being the capture time: this
                          # is the real age of the estimate being driven at,
                          # not the age of the last message received.
float32 speed_ref         # the speed command derived from r [m/s]
bool target_stale         # the map->odom reprojection is holding its last
                          # good value rather than tracking a live transform
bool goal_watchdog        # ObjectGoal refreshes stopped for longer than
                          # object_goal_timeout_sec: mpc_corr is holding the
                          # car at zero (not ramped) until they resume
