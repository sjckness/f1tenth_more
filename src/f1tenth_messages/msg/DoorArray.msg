# /perception/doors: the CONFIRMED doors door_detection holds -- and, with
# door_publish_candidates, its CANDIDATE doors too (Door.state says which) --
# published on every wall tracker cycle, empty included. header.frame_id is the
# odom frame, header.stamp the scan stamp. PERCEPTION ONLY. /viz/doors draws
# every live door, both states, whatever door_publish_candidates says.
std_msgs/Header header
Door[] doors
