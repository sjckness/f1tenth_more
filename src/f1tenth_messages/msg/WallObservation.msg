# ONE wall seen in ONE cycle, from f1tenth_perception's lidar_front_wall_node
# on /perception/wall_observations. Stateless: this message is what the two
# sensors say right now, with no track, no history and no prediction in it.
# The tracker (multi_wall_tracker.py) is the only thing that remembers.
#
# WHY THE SPLIT. Before this, the tracker fitted lines to the scan itself and
# the stereo entered only as a scalar fallback distance, so "the LiDAR sees
# nothing in the strip" could silently become "the stereo plane is the wall" --
# which on an open corridor made the SIDE wall the front wall at 1.1 m
# (corridor_phantom, 2026-10-15). Separating the per-cycle evidence from the
# tracking means the evidence can be rejected on its own merits before
# anything persistent is built on it.
#
# A WALL IS A VERTICAL PLANE: the line n . (x, y) = d in the ground plane,
# extruded from z_min to z_max. n is the unit normal pointing from the base
# frame's origin toward the wall, so d >= 0 -- the same convention as
# scene_labels.WallLine and lidar_front_wall.py.
#
# FRAME: base_link at the observation's stamp. The tracker places it in odom.
std_msgs/Header header

float32 nx                  # unit normal, base_link
float32 ny
float32 d                   # n . (x, y) = d, d >= 0 [m]
geometry_msgs/Point p0      # the observed extent's endpoints, base_link; z = 0
geometry_msgs/Point p1
float32 z_min               # the vertical extent [m] above the floor. From the
float32 z_max               # stereo WALL points when there are any, otherwise
                            # 0 .. wall_default_height (see `height_measured`)
bool height_measured        # false: z_min/z_max are the default, not seen

uint8 source
uint8 SOURCE_LIDAR = 0      # LiDAR only: outside the camera's view, or the
                            # stereo had nothing to say about those columns
uint8 SOURCE_STEREO = 1     # stereo only, over a contiguous span: glass, or a
                            # surface too dark for the LiDAR
uint8 SOURCE_BOTH = 2       # both sensors, agreeing

float32 confidence          # [0, 1]; see wall_fusion.confidence() for the model
uint32 inliers              # LiDAR returns on the line (0 for a stereo-only wall)
float32 rms                 # [m] of those inliers (NaN for a stereo-only wall)
uint32 col_begin            # the image columns this wall was judged over,
uint32 col_end              # half-open; equal when it was never in the frame
uint32 n_agree              # column statuses inside that span
uint32 n_lidar_only
uint32 n_stereo_only
uint32 n_disagree
