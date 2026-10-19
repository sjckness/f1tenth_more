# One open doorway from f1tenth_perception's door_detection.py
# (lidar_front_wall_node, door_detection_enabled), on /perception/doors.
# PERCEPTION ONLY: nothing in the control path, the costmap, the BT or the
# missions reads it.
#
# A door is a gap of door_width_min_m .. door_width_max_m between two pieces
# of one wall line the multi-wall tracker holds (confirmed or coasting), that
# every LiDAR beam crossing it sees THROUGH. Closed doors are wall to the
# LiDAR and never appear here. door_detection.py's module docstring has the
# full rule.
#
# Positions are in the tracker's fixed frame (DoorArray.header.frame_id, the
# odom frame every wall track lives in), on the floor (z = 0); the *_bl
# copies are base_link at the scan stamp (DoorArray.header.stamp).

uint32 id                       # never reused, and NOT tied to the parent wall
                                # ids: doors are associated by centre, normal
                                # and width, because wall ids re-spawn often
uint8 state
uint8 STATE_TENTATIVE = 0       # seen, not yet confirmed (M of N cycles)
uint8 STATE_CONFIRMED = 1       # seen this cycle, or held by a person in it
uint8 STATE_COASTING = 2        # confirmed earlier, not seen this cycle

geometry_msgs/Point center      # midpoint of the two jambs
geometry_msgs/Vector3 normal    # unit, the crossing direction: from the side
                                # the car first saw the door from, through it
float32 width                   # [m] jamb to jamb
geometry_msgs/Point jamb0
geometry_msgs/Point jamb1
geometry_msgs/Point center_bl
geometry_msgs/Point jamb0_bl
geometry_msgs/Point jamb1_bl

uint32 parent_wall_id0          # MultiWallTrack ids of the walls at jamb0 and
uint32 parent_wall_id1          # jamb1 on the last cycle that saw the door;
                                # equal when the doorway lies inside one track

bool occupied                   # a detected person is standing in it; that
                                # keeps a CONFIRMED door alive while its beams
                                # are blocked
float32 confidence              # [0, 1]: LiDAR evidence 0.5-0.8, + stereo
bool stereo_support             # the class mask saw no wall in the gap and
                                # floor beyond it (optional; LiDAR-only doors
                                # are doors)
uint32 hits                     # cycles the LiDAR saw it as a candidate
float32 age                     # [s] since the door was born
float32 since_seen              # [s] since the LiDAR last saw it: the coasting age
uint16 n_beams                  # open beams through the gap on the last sighting
