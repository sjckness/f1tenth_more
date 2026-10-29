#!/usr/bin/env bash
# Why are there no doors in Foxglove? Nine checks, in the order they fail.
#
# Run on the machine that runs the stack, with the workspace sourced:
#
#     source install/setup.bash && tools/doors_doctor.sh
#
# Every check prints OK or WHY, and nothing here touches the running stack:
# it only lists, gets and echoes. Safe to run during a mission.
#
# The checks are ordered so the FIRST failure is the cause. Reading past it
# is usually a waste of time -- a node that is not running has no parameters,
# and a node with door_source:=profile is behaving correctly by publishing
# nothing.
set -uo pipefail

NODE=/freespace_door_node
T=3
pass=0; fail=0
ok()   { printf '  \033[32mOK\033[0m   %s\n' "$*"; pass=$((pass+1)); }
why()  { printf '  \033[31mWHY\033[0m  %s\n' "$*"; fail=$((fail+1)); }
note() { printf '       %s\n' "$*"; }
head_() { printf '\n\033[1m%s\033[0m\n' "$*"; }

head_ '1. the executable exists in the install space'
if ros2 pkg executables f1tenth_costmap 2>/dev/null | grep -q freespace_door_node; then
  ok 'f1tenth_costmap exposes freespace_door_node'
else
  why 'f1tenth_costmap has no freespace_door_node executable'
  note 'the package was never rebuilt on this machine. On the Thor the install'
  note 'space is NOT a symlink farm for everything, so a git pull alone changes'
  note 'nothing:'
  note '  colcon build --symlink-install --packages-select \'
  note '      f1tenth_messages f1tenth_params f1tenth_costmap f1tenth_behavior'
fi

head_ '2. Door.msg carries the strong field'
if ros2 interface show f1tenth_messages/msg/Door 2>/dev/null | grep -q 'bool strong'; then
  ok 'f1tenth_messages/msg/Door has bool strong'
else
  why 'Door.msg has no strong field -- f1tenth_messages is the old one'
  note 'rebuild f1tenth_messages FIRST, then f1tenth_costmap and f1tenth_behavior'
  note 'against it. A stale message definition makes every door publish fail.'
fi

head_ '3. the installed stack_params.yaml is this branch'
YAML=$(ros2 pkg prefix f1tenth_params 2>/dev/null)/share/f1tenth_params/config/stack_params.yaml
if [ -f "$YAML" ]; then
  if grep -q 'freespace_doors_merge_tracks_overlap' "$YAML"; then
    ok "$YAML is current"
  else
    why "$YAML predates the freespace detector"
    note 'rebuild f1tenth_params. With --symlink-install this file is a symlink'
    note 'into src/ and a pull is enough; without it, it is a stale copy.'
  fi
  note "door_source default in that file: $(grep -A1 '^door_source:' "$YAML" | sed -n 2p | tr -s ' ')"
else
  why 'f1tenth_params is not installed on this machine'
fi

head_ '4. the node is running'
if ros2 node list 2>/dev/null | grep -qx "$NODE"; then
  ok "$NODE is up"
else
  why "$NODE is not in the node list"
  note 'it lives in f1tenth_costmap/costmap.launch.py, which is the SECOND'
  note "entry of the 'slam' component and is gated on enable_slam. Check:"
  note '  ros2 param get /supervisor enable_slam     # or the launch argument'
  note '  grep -n freespace $(ros2 pkg prefix f1tenth_bringup)/share/*/config/components.yaml'
  note 'and look in the supervisor log for a non-zero exit of the component.'
  printf '\n%d ok, %d to look at -- stop here, nothing below can pass.\n' "$pass" "$fail"
  exit 1
fi

head_ '5. the node is enabled, not gated off'
src=$(timeout $T ros2 param get "$NODE" door_source 2>/dev/null | awk '{print $NF}')
en=$(timeout $T ros2 param get "$NODE" freespace_doors_enabled 2>/dev/null | awk '{print $NF}')
if [ "$src" = "freespace" ] && [ "$en" = "True" ]; then
  ok "door_source=$src freespace_doors_enabled=$en"
else
  why "door_source=${src:-?} freespace_doors_enabled=${en:-?}"
  note 'THE NODE IS DELIBERATELY SILENT. Both gates must hold: it publishes'
  note 'nothing at all with another source selected, so the three detectors'
  note 'can never fight over /viz/doors_semantic. Relaunch with'
  note '  door_source:=freespace'
  note 'or fix the default in stack_params.yaml and rebuild f1tenth_params.'
fi

head_ '6. /scan reaches it'
if timeout 5 ros2 topic hz /scan 2>/dev/null | grep -q 'average rate'; then
  ok '/scan is publishing'
else
  why '/scan is silent -- the detector is driven by the scan and never cycles'
fi

head_ '7. base_link -> the scan frame is in /tf_static'
frame=$(timeout 5 ros2 topic echo --once /scan 2>/dev/null | grep -m1 frame_id | awk '{print $2}')
if [ -n "${frame:-}" ]; then
  note "scan frame_id: $frame"
  if timeout 5 ros2 run tf2_ros tf2_echo base_link "$frame" 2>&1 | grep -q 'Translation'; then
    ok "base_link -> $frame resolves"
  else
    why "base_link -> $frame does not resolve"
    note 'the node logs "no ... transform yet -- the scan cannot be placed" once'
    note 'per cycle and publishes nothing. Check the URDF / static publishers.'
  fi
else
  why 'could not read a scan to find its frame'
fi

head_ '8. the detector is cycling'
dbg=$(timeout 5 ros2 topic echo --once /costmap/freespace_doors/debug 2>/dev/null | sed -n 's/^data: //p')
if [ -n "${dbg:-}" ]; then
  ok 'it publishes its debug line'
  note "$dbg"
  note 'door_errors > 0 means cycles are raising -- look in the node log.'
  note 'scan_jumps 0 means the scan has no discontinuities at all.'
  note 'door_candidates 0 with scan_jumps > 0 means every opening was'
  note 'rejected: door_codes says which gate did it.'
else
  why 'nothing on /costmap/freespace_doors/debug -- the node is not cycling'
fi

head_ '9. the markers reach Foxglove'
if timeout 5 ros2 topic hz /viz/doors_semantic 2>/dev/null | grep -q 'average rate'; then
  ok '/viz/doors_semantic is publishing'
  note 'EVERY live track goes out here, candidates included and drawn faded,'
  note 'so an empty 3D panel with this topic ticking means the detector found'
  note 'nothing -- not that the markers are lost. ^/viz/.* is in'
  note 'foxglove_topic_whitelist, so the bridge is not the problem.'
  note 'Add a panel for /viz/doors_freespace_debug too: that one carries the'
  note 'raw scan jumps (yellow) and rejected candidates (grey).'
else
  why '/viz/doors_semantic is silent'
fi

head_ '10. is anything confirmed?'
n=$(timeout 5 ros2 topic echo --once /perception/doors_semantic 2>/dev/null | grep -c 'frame_id: door_')
note "confirmed doors on /perception/doors_semantic: ${n:-0}"
if [ "${n:-0}" -eq 0 ]; then
  note 'A STATIONARY CAR NEVER CONFIRMS A DOOR. The shipped defaults are'
  note 'freespace_doors_confirm_pose_spread_m 1.0 with'
  note 'freespace_doors_confirm_same_pose_hits 0, i.e. a door needs two'
  note 'sightings from robot positions a metre apart and there is no'
  note 'standing-still fallback. Drive a couple of metres, or set'
  note '  freespace_doors_confirm_same_pose_hits:=3'
  note 'for a bench test. Unconfirmed candidates are still on'
  note '/viz/doors_semantic, faded.'
fi

printf '\n%d ok, %d to look at. The FIRST WHY above is the cause.\n' "$pass" "$fail"
