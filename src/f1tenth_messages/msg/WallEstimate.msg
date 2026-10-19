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

# --- since 2026-10-15 (step 5: the front wall output contract) ---------------
# Additive; every field above keeps its meaning. `state` is the one a consumer
# should branch on. `valid`/`provenance` describe the LiDAR ESTIMATOR's own
# lineage, which is a different question from "is there a wall ahead".
uint8 state
uint8 STATE_NO_DATA = 0     # the sensors are not telling us. The Float32 wall
                            # topics carry -1.0 and the hold logic applies.
                            # NOT "clear": treating it as clear drives blind.
uint8 STATE_WALL = 1        # a FRONT track crosses the car's strip; the wall
                            # topics carry its distance from the bumper.
uint8 STATE_CLEAR = 2       # the sensors work and NOTHING is in the way. The
                            # wall topics carry wall_clear_distance_m, a large
                            # finite number -- not a wall at that distance.
                            # This is the open corridor, the state the old
                            # pipeline could not express, which is why the side
                            # wall became a phantom front wall at 1.1 m.
