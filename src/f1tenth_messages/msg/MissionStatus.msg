# Published by f1tenth_behavior's MissionLoader (mission/loader.py) on
# /mission/status with transient-local ("latched") QoS -- a subscriber that
# starts after this node still gets the current value immediately on
# subscribe, not just future changes. Republished whenever mission state
# changes: on load, start, abort, or emergency_stop (NOT on every BT tick).
#
# Consumed by f1tenth_behavior's IsEmergencyStopTriggered BT condition (the
# emergency lane) for emergency_stop_active. state/json_path are for external
# introspection (dashboards, `ros2 topic echo`, etc.) -- the BT's own mission
# subtree gates on MissionActive/MISSION_KEY directly against the same
# MissionRuntimeState object (same process, no need to round-trip through
# this topic just for that).

std_msgs/Header header

# MissionState value: IDLE / LOADED / RUNNING / HOLDING / COMPLETE / ABORTED
string state

# Path last passed to /mission/load_mission (or resolved from
# mission_file_name / /mission/load_path); '' if nothing has been loaded yet.
string json_path

# Latched by /mission/emergency_stop -- once true, stays true for the rest of
# this node's/process's lifetime. No reset service exists on purpose; the only
# way out is restarting the node (or the whole BT process).
bool emergency_stop_active
