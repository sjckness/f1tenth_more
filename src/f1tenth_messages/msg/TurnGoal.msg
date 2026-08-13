# Turn-step goal for mpc_corr's /mpc/goal_turn -- published once per turn-step
# entry by f1tenth_behavior's PublishMoveGoal, mirroring how goal_distance/
# goal_pose already work (see f1tenth_behavior's mission schema docs for the
# full "turn" step JSON shape this comes from).

float32 heading_delta_deg   # signed, + = left/CCW, - = right/CW (right-hand rule, standard ROS yaw convention)
float32 speed                # desired linear speed while turning [m/s]
string steering               # "full_lock" or "partial:<deg>" -- see mpc_corr's
                               # goal_turn_callback for exactly how this is used
                               # (shapes the turn's target-approach geometry via the
                               # standard Ackermann turn-radius relationship; NOT a
                               # literal open-loop steering-angle command -- the
                               # existing MPC solver still computes the actual
                               # steering output)
