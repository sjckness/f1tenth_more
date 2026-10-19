# /perception/doors: every door track door_detection holds, tentative and
# coasting included (Door.state says which), published on every wall tracker
# cycle, empty included. header.frame_id is the odom frame, header.stamp the
# scan stamp. PERCEPTION ONLY.
std_msgs/Header header
Door[] doors
