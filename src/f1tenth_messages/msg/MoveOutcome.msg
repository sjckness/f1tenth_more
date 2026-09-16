# One finished mission move, on /mission/move_outcome.
#
# Published by f1tenth_behavior's MissionLoader, which watches the mission's
# move_outcomes list and emits each new entry once -- the same watcher that
# republishes /mission/status on state changes, so no behaviour needs a
# publisher of its own. Mirrors move_scoring.MoveOutcome. Unset floats are NaN,
# unset strings empty.
std_msgs/Header header
string mission_id
string move_id
string move_type            # goal_distance | goal_pose | turn | drive | go_to_object
string stop_reason          # e.g. stop_condition:object_reached, timeout:abort,
                            # go_to_object:target_lost
string outcome              # go_to_object only: reached | target_not_found |
                            # target_lost | target_unreachable | timeout
string wire_move_id         # go_to_object only: the ObjectGoal move_id
float32 duration_s
float32 commanded
float32 actual
float32 score_percent
bool mismatch_flagged
float32 arrival_bearing_error_deg
float32 track_range_m
string note
