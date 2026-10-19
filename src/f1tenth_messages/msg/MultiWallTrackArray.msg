# /perception/walls: the published wall tracks (at most the 4 highest-scoring
# confirmed tracks, plus coasting ones within that cap). header.frame_id is the
# odom frame and header.stamp the scan stamp.
std_msgs/Header header
MultiWallTrack[] tracks
