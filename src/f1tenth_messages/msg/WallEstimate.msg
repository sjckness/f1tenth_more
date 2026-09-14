# The usable-for-control front wall distance, from f1tenth_perception's
# lidar_front_wall_node on /perception/lidar_front_wall_virtual.
#
# CONTRACT: published once per /scan, always -- but the value may be invalid.
# It is NOT "always has a usable number". Consumers MUST check `valid`.
# `valid` is false (provenance NONE) whenever there is neither a live or
# dead-reckonable measurement nor a fresh seed: at startup before either
# arrives, and after a dead-reckoned value ages out with no fresh seed to fall
# back on. See the node's module docstring for the provenance rules.
#
# header.stamp is the scan's own stamp; header.frame_id is base_link.

std_msgs/Header header

bool valid
uint8 provenance
uint8 PROVENANCE_NONE = 0           # no measurement lineage and no fresh seed
uint8 PROVENANCE_MEASURED = 1       # this scan's fit
uint8 PROVENANCE_DEAD_RECKONED = 2  # last accepted fit, projected along odometry
uint8 PROVENANCE_SEEDED = 3         # /costmap/front_clearance, a DIFFERENT quantity

float32 distance      # [m]; NaN when provenance is NONE
float32 normal_angle  # car-to-wall normal bearing, base_link [rad]; NaN unless
                      # MEASURED or DEAD_RECKONED (a seed has no normal)
# Age of what `distance` came from [s]: 0 when MEASURED; since the last accepted
# fit when DEAD_RECKONED; since the seed was received when SEEDED; since the last
# accepted fit when NONE (NaN if there never was one).
float32 source_age
