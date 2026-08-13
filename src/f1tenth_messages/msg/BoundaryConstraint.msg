# One hard linear boundary constraint for mpc_controller's OSQP/RTI solver:
# normal . (x, y) <= offset, both already in the frame the PUBLISHER
# computed them in (base_link for both current producers -- wall_detector_
# node's /perception/front_wall_boundary and the lidar boundary node's
# /perception/lidar_boundaries -- see each node's own docstring). Consumers
# that operate in a different frame (mpc_controller's own MPC state is
# world/odom-frame, not base_link) are responsible for their own rigid-
# transform of (normal, offset) into whatever frame they need -- this
# message does not carry a header of its own precisely so a single
# constraint stays a plain, frame-agnostic halfspace; BoundaryConstraintArray
# carries the header (which frame these were computed in, and when).
#
# SIGN CONVENTION: `normal` points AWAY from the safe/free side, i.e.
# TOWARD the obstacle/wall the constraint is keeping the robot out of --
# the free-space side is where normal . (x, y) is SMALL. This is the
# OPPOSITE convention from WallDetection.msg's own `normal` field (which
# points back TOWARD the robot) -- deliberately: WallDetection's normal
# answers "which way is the robot from this wall", BoundaryConstraint's
# answers "which way must the robot NOT cross past" the more natural
# direction for a `<=` halfspace test. See wall_detector_node.py's own
# module docstring for the derivation/verification against WallDetection's
# existing sign convention.
#
# `offset` is NOT pre-shrunk by any safety margin -- it is the raw
# geometric boundary (offset = normal . point_on_boundary, e.g. the
# perpendicular distance from the producer's own frame origin to the wall/
# line). Consumers subtract their own margin (car_radius +
# obstacle_safety_margin_m) at the point of use, so this message stays a
# pure geometric fact, not a policy decision baked in by the producer.

float64[2] normal
float64 offset
