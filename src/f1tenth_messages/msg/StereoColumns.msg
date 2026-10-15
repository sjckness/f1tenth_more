# The stereo half of the wall cross-check, summarised PER IMAGE COLUMN, from
# f1tenth_perception's front_clearance_node on /perception/stereo_columns.
#
# WHY A COLUMN PROFILE AND NOT THE POINT CLOUD. The fusion that consumes this
# runs next to the LiDAR (lidar_front_wall_node): it needs the scan, the line
# fits and the tracker's own clock, none of which live on the camera side. The
# WALL points themselves are ~50k per frame, about 3 MB/s at 5 Hz, and the
# fusion only ever asks one question per column ("how far does the stereo say
# this bearing is?"). Summarising on the camera side, where the pixels already
# are, turns that into 640 columns of four numbers: ~10 KB a frame.
# /perception/wall_points still carries the full cloud for inspection and
# recording; nothing in the control path reads it.
#
# EMPTY COLUMNS CARRY NaN, not 0.0 and not a large number: "the stereo has
# nothing to say about this bearing" is a third state, and a consumer that
# treats it as a distance will brake for it.
#
# header.stamp IS THE CAMERA IMAGE STAMP -- the depth frame these columns were
# summarised from, not the time this message was built. The consumer pairs it
# with its own scan on that basis and REJECTS the pair when the two stamps are
# too far apart, so an exposure time that drifts from the publish time must not
# be hidden here. header.frame_id is the base frame the heights are measured
# in, so the fusion can place them without a second lookup.
std_msgs/Header header

string camera_frame         # the DEPTH image's own optical frame, so the
                            # consumer can project into these columns without
                            # a second message to tell it where the lens is
uint32 width                # image width; every array below is this long
uint32 height               # image height, for the record
float32 fx                  # the intrinsics the columns were summarised with,
float32 fy                  # at THIS width/height, already rescaled from
float32 cx                  # camera_info if the depth image differs from it
float32 cy

# Per column, all `width` long:
float32[] distance_m        # low percentile of the column's WALL depth
                            # (optical-axis Z); NaN where fewer than min_px
float32[] z_min             # lowest/highest base-frame height of that column's
float32[] z_max             # WALL points; NaN where there are none
uint32[] n_px               # WALL pixels in the column, before the min_px gate

float32 percentile          # which percentile distance_m is
uint32 min_px               # fewest WALL pixels for a non-NaN distance_m
