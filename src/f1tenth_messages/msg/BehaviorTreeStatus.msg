# Per-tick behaviour-tree lane/condition outcome, published by
# f1tenth_behavior's behavior_executor_node on /behavior/tree_status.
#
# WHY THIS EXISTS: which root lane won a tick, and which emergency condition
# tripped, previously existed ONLY as py_trees text in the behaviour node's
# per-component supervisor log -- which is overwritten on every supervisor
# restart. During the 2026-09-01 analysis that log covered only the window
# AFTER the last mission, so every stop cause had to be re-derived offline by
# replaying each condition's predicate against recorded sensor data. Both Stop
# publishers also emit byte-identical messages on the same /safety_stop topic,
# so a bag could not distinguish an emergency stop from an obstacle stop at
# all. This topic makes both directly readable from the bag.

std_msgs/Header header

# Name of the root Selector child that returned SUCCESS this tick ('emergency',
# 'handle_obstacle', 'mission', 'navigation'), or '' when the whole root
# failed (nothing active -- e.g. no mission loaded).
string active_lane

# Status of each root lane this tick, index-aligned with lane_statuses.
# Values: 'SUCCESS' / 'FAILURE' / 'RUNNING' / 'INVALID'.
string[] lane_names
string[] lane_statuses

# Name of the emergency_condition child that tripped, or '' if none did.
# This is the field that answers "why did the car stop?" directly:
# 'IsBatteryLow' / 'IsEmergencyStopTriggered' / 'IsProximityTooClose' /
# 'IsSystemOverheated'.
string emergency_trip

# True while any Stop behaviour is publishing on the safety_stop mux lane this
# tick (i.e. the car is being actively held stopped by the BT rather than by
# the MPC's own output).
bool safety_stop_active

# Which Stop instance is publishing: 'emergency', 'obstacle', or '' when none
# is. Mirrors the header.frame_id the corresponding Stop stamps, so bag
# analysis can cross-check this topic against /safety_stop itself.
string stop_source
