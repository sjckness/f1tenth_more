# One open doorway from f1tenth_perception's door_detection.py
# (lidar_front_wall_node, door_detection_enabled), on /perception/doors.
# PERCEPTION ONLY: nothing in the control path, the costmap, the BT or the
# missions reads it. Doors are an OVERLAY: they never modify walls or LiDAR
# data, and nothing may treat a doorway as free space because a door is
# published there.
#
# A door is a gap of door_width_min_m .. door_width_max_m between two stretches
# of one wall line, on HOST walls only: wall tracks the multi-wall tracker holds
# CONFIRMED with role front, left or right (an unknown segment never hosts a
# door). The LiDAR sees THROUGH its core, every depth measured perpendicular to
# the line; an open leaf hung on a jamb is allowed. Closed doors are wall to the
# LiDAR and never appear here. door_detection.py's module docstring has the
# full rule.
#
# LIFECYCLE: a positive sighting makes a CANDIDATE; it is CONFIRMED once seen
# positive from door_min_viewpoints car positions door_viewpoint_sep_m apart
# with a steady width. A STRONG rejection on a visible gap strikes it back to
# CANDIDATE, a second deletes it; unseen for door_candidate_max_age_sec
# (candidate) or door_confirmed_max_age_sec (confirmed) it is dropped. Doors no
# longer coast. /perception/doors carries CONFIRMED doors only, unless
# door_publish_candidates.
#
# Positions are in the tracker's fixed frame (DoorArray.header.frame_id, the
# odom frame every wall track lives in), on the floor (z = 0); the *_bl
# copies are base_link at the scan stamp (DoorArray.header.stamp).

uint32 id                       # never reused, and NOT tied to the parent wall
                                # ids: doors are associated by centre, normal
                                # and width, because wall ids re-spawn often
uint8 state
uint8 STATE_CANDIDATE = 0       # seen positive, not (or no longer) confirmed
uint8 STATE_CONFIRMED = 1       # seen positive from enough viewpoints, steady width
uint8 STATE_TENTATIVE = 0       # LEGACY name of STATE_CANDIDATE (until 2026-10-20)
uint8 STATE_COASTING = 2        # LEGACY: no longer published (doors do not coast)

geometry_msgs/Point center      # midpoint of the two jambs
geometry_msgs/Vector3 normal    # unit, the crossing direction: from the side
                                # the car first saw the door from, through it
float32 width                   # [m] jamb to jamb
geometry_msgs/Point jamb0
geometry_msgs/Point jamb1
geometry_msgs/Point center_bl
geometry_msgs/Point jamb0_bl
geometry_msgs/Point jamb1_bl

uint32 parent_wall_id0          # MultiWallTrack ids of the host walls at jamb0
uint32 parent_wall_id1          # and jamb1 on the last positive sighting; equal
                                # when the doorway lies inside one track. Always
                                # wall track ids: there are no frame pieces any
                                # more (until 2026-10-20 a 0 meant one)

bool occupied                   # a detected person is standing in it; that
                                # keeps a CONFIRMED door alive while its beams
                                # are blocked
float32 confidence              # [0, 1]: LiDAR evidence 0.5-0.8, + stereo
bool stereo_support             # the class mask saw no wall in the gap and
                                # floor beyond it (optional; LiDAR-only doors
                                # are doors)
uint32 hits                     # positive sightings, ever
float32 age                     # [s] since the door was born
float32 since_seen              # [s] since its last POSITIVE sighting
uint16 n_beams                  # decided beams across the gap on the last
                                # positive sighting (through, shallow, leaf and
                                # reveal returns)

# Appended (door-detection step 2); every field above keeps its meaning.
uint8 leaf_side                 # LEAF_*: the jamb an OPEN LEAF hangs on, looking
                                # along `normal` (door_detection.py, THE LEAF)
uint8 LEAF_NONE = 0
uint8 LEAF_LEFT = 1
uint8 LEAF_RIGHT = 2
bool glass_suspect              # door_stereo_glass_veto on and the stereo depth
                                # over the gap mostly invalid: not vetoed, but
                                # possibly a pane neither sensor ranged

# Appended (door-detection addendum, 2026-10-20); every field above keeps its
# meaning.
bool jamb_height_unknown        # true while no positive sighting has had both
                                # jambs' wall height judged by the stereo
                                # (door_detection.stereo_jamb_heights): no stereo
                                # frame, no valid depth or WALL pixels beside a
                                # jamb, or a view clipped below
                                # door_jamb_min_height_m. The furniture check
                                # (desks, cabinets) has not vouched for it.

# Appended (freespace scan-jump detector, 2026-10-28); every field above keeps
# its meaning.
bool strong                     # HOW MUCH THIS DOOR HAS EARNED. A confirmed
                                # door is publishable; a STRONG one is one a
                                # mission may drive at. The detector that set
                                # it decides what it takes (semantic_doors'
                                # DoorParams: strong_min_hits sightings,
                                # strong_min_spread_m between the two robot
                                # positions furthest apart that saw it, and no
                                # flags). Both WALL detectors leave those
                                # thresholds at zero and so publish every
                                # confirmed door strong, which is the
                                # behaviour they had before this field
                                # existed. The scan-jump detector does not:
                                # from a single scan a doorway and a gap
                                # between two pieces of furniture are nearly
                                # the same thing, and only being seen from
                                # somewhere else tells them apart.
                                # f1tenth_behavior's enter_door selects over
                                # STRONG doors only (-> no_strong_door).
