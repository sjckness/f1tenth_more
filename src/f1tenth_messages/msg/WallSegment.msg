# The tracked wall as a line SEGMENT, from f1tenth_perception's
# wall_distance_node on /perception/d_wall/segment. The companion to
# WallDistance: that message answers "how far, which side"; this one answers
# "where does the surface actually run".
#
# WHY IT IS A SEPARATE MESSAGE AND NOT TWO MORE FIELDS ON WallDistance.
# WallDistance is a control input, published every tick and read for one
# scalar. This is geometry, and its only consumer is swept_clearance_node,
# which needs the endpoints and nothing else. Keeping them apart means the
# control path does not re-serialise two Points it never reads.
#
# FRAME: base_link, x forward, y left -- NOT odom, even though the track
# itself is maintained in odom. The consumer (swept_clearance_node) works in
# base_link and subscribes to no odometry at all; converting here, where the
# car pose the track was evaluated against is already in hand, keeps a second
# node from needing its own odom subscription and its own transform.
#
# THE SEGMENT DELIBERATELY EXTENDS PAST WHERE BEAMS CURRENTLY LAND. That
# extension is the whole reason the wall is tracked rather than re-fitted:
# through a turn the pane sweeps into incidence angles where glass returns
# nothing, and the surface is still there. A consumer must therefore NOT
# treat these endpoints as observed points -- read `provenance` on the
# matching WallDistance for how much of this is measurement.
#
# header.stamp and header.frame_id match the WallDistance published in the
# same breath, so the two can never disagree about which observation they
# describe.
std_msgs/Header header

uint32 track_id                # matches WallDistance.track_id
geometry_msgs/Point p0         # segment endpoint, base_link; z is always 0.0
geometry_msgs/Point p1         # segment endpoint, base_link; z is always 0.0
bool valid                     # false: no usable track, and p0/p1 are NaN
