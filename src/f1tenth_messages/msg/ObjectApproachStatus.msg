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
float32 r                 # range to the STANDOFF point: distance to target
                          # minus standoff. Negative means the vehicle is
                          # already inside the standoff.
float32 bearing           # absolute heading from the vehicle to the target
                          # [rad], odom frame
float32 psi_c             # the HELD corridor heading [rad], odom frame. Not
                          # derivable from anything else published.
float32 e                 # wrap(bearing - psi_c): how far the held heading
                          # still has to rotate. Measured against psi_c, NOT
                          # against live yaw -- see object_approach.
float32 k                 # distance schedule in [0, 1]: the fraction of e
                          # this rebuild asked for
float32 dpsi_max          # the kinematic cap that was in force [rad]
bool feasible             # false when the approach cannot be flown as asked
string reason             # "" when feasible; otherwise "target_behind" or
                          # "inside_turn_radius"
float32 target_age_s      # now minus the ObjectGoal header stamp. The whole
                          # point of that stamp being the capture time: this
                          # is the real age of the estimate being driven at,
                          # not the age of the last message received.
float32 speed_ref         # the speed command derived from r [m/s]
bool target_stale         # the map->odom reprojection is holding its last
                          # good value rather than tracking a live transform
