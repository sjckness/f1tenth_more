# Drive-to-a-tracked-object goal for mpc_corr's /mpc/goal_object.
#
# The fourth goal shape alongside goal_distance (Float32), goal_pose
# (PoseStamped) and goal_turn (TurnGoal), and the first one that is REPUBLISHED
# throughout a move rather than sent once. That difference is the whole reason
# it carries move_id.
#
# WHY move_id EXISTS. A tracked target moves, and the sender refreshes this
# message at the track rate. mpc_corr's other goal callbacks treat every
# message as a new move: they clear the opposing modes, drop the corridor
# cache, drop the RTI warm start, and re-anchor. Doing that per refresh would
# re-anchor the corridor and cold-start the solver on every message -- at a
# 12.5-20 Hz refresh, on every control tick. move_id lets the callback tell
# "a new move began" from "the same move's target moved": a new id
# re-anchors, a repeat updates the target point and nothing else.
#
# So move_id must be STABLE for the whole move and DISTINCT between moves,
# including between two runs of the same single-move mission -- a mission's
# move ids repeat across runs (see MissionRuntimeState.run_generation), so a
# sender using the bare move id would have run 2 silently continue run 1's
# anchor. Compose it with the run generation.
std_msgs/Header header    # frame_id: "map". stamp: the DETECTION CAPTURE time
                          # this target was estimated at -- NOT the send time.
                          # mpc_corr reports it as target_age_s and a consumer
                          # of that cannot reconstruct it from arrival time
                          # (see semantic_layer_node._publish_tracks).
string move_id            # stable for the move, distinct between moves
string target_class       # what is being approached, for logs and status only
                          # -- mpc_corr never matches on it. The sender owns
                          # which track this point came from.
geometry_msgs/Point point # the TARGET itself in the header frame, not the
                          # standoff point. mpc_corr subtracts the standoff
                          # along its own held corridor heading, so the goal
                          # stays consistent with the heading it is driving.
                          # z is ignored; this vehicle is planar.
float32 standoff          # metres short of `point` to stop. Must be > 0: a
                          # zero standoff drives the vehicle into the target.
float32 speed             # desired linear speed on approach [m/s]. mpc_corr
                          # ramps DOWN from this as the range closes (see
                          # object_approach.object_speed_ref); it is a
                          # ceiling, not a setpoint.
