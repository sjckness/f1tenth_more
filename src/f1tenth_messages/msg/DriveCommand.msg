# Open-ended drive command for mpc_corr's /mpc/goal_drive -- published once per
# mission "drive" step entry (schema_version 3.0) by f1tenth_behavior's
# PublishMoveGoal, mirroring how goal_distance/goal_pose/goal_turn already work.
#
# The FOURTH goal shape, and the only OPEN-ENDED one: mpc_corr never sets
# goal_reached for it, never publishes /mpc/goal_reached, and never self-
# terminates. The behaviour tree's own stop_condition is the sole authority on
# when the move ends. That is the whole point of this message -- it carries a
# driving MODE, not a target, so "keep going until the guard fires" is
# expressible without inventing a fake goal for the controller to chase.
#
# Ported from f110_autonomy's phase model (mode + guard, no goal), which this
# stack's LLM planner has always emitted and which used to be squeezed through
# a lossy translator into goal_distance: 50.0.

# "straight"  -- hold the move's own start heading.
# "wall_turn" -- turn by turn_sign * turn_mag_deg off the move's own start
#                heading, then hold the result.
# Anything else is rejected at mission-load time (mission_config.py's
# DRIVE_MODES) and ignored-with-a-warning on the wire.
string mode

# Turn direction, matching mission_config.py's TurnSpec sign convention and
# the LLM's own turn_sign: +1.0 = left/CCW, -1.0 = right/CW. Ignored when
# mode is "straight".
float32 turn_sign

# Turn magnitude in DEGREES (unsigned; turn_sign carries the direction).
# SENTINEL: 0.0 means "use the node's own default" (mpc_corr's
# drive_default_turn_mag_deg, 90.0) -- f110_autonomy hardcoded pi/2 here and
# never read a magnitude off the phase at all, which is one of the two bugs
# this port deliberately does not reproduce.
float32 turn_mag_deg

# Desired linear speed [m/s] for the duration of the move.
# SENTINEL: 0.0 means "use the node default" (mpc_corr's own self.vdes).
float32 speed

# Total obstacle standoff [m] to use for the duration of this move -- the
# quantity mpc_corr spends as car_radius + avoidance_margin (compute_local_
# target's R_safe, and mpc_solver's own obstacle trigger and boundary rows).
# Reproduces f110_autonomy's terminal-stop d_safe relaxation, whose purpose is
# to stop the obstacle penalty pushing the car AROUND the very object it was
# commanded to stop in front of.
# SENTINEL: any negative value means "use the node default" (leave
# car_radius/avoidance_margin alone). 0.0 is a real, meaningful value
# (no standoff at all), which is why the sentinel here is < 0 and not == 0.
float32 d_safe
