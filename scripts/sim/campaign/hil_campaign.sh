#!/usr/bin/env bash
# hil_campaign.sh -- the office HIL campaign: one natural-language command through the full
# LLM->motion pipeline, repeated, recorded and judged, with Isaac on the workstation and the
# unchanged car stack on the Jetson Orin.
#
# Runs ON THE ORIN (host side; it drives the container through docker), like scripts/sim/hil_orin.
#
# Usage:
#   scripts/sim/campaign/hil_campaign.sh preflight            both sides agree, container ready,
#                                                             campaign folder and prompts present
#   scripts/sim/campaign/hil_campaign.sh translate            --dry-run all three commands: the
#                                                             translation check, no motion
#   scripts/sim/campaign/hil_campaign.sh mission M02 [--reps 3]
#   scripts/sim/campaign/hil_campaign.sh all [--reps 3]       the three missions, stopping at each
#                                                             boundary for the manual scene change
#   scripts/sim/campaign/hil_campaign.sh judge [M02]          export + print what is unjudged
#   scripts/sim/campaign/hil_campaign.sh analyze              export, plots, report, aggregate
#   scripts/sim/campaign/hil_campaign.sh bags                 the bag index as it stands
# Options: --reps N, --dry-run (print, run nothing), --no-gate (do not refuse on unjudged runs), -h.
#
# WHAT ONE RUN IS, and why there is no "start the mission" step: ONE invocation of
# llm_planner_node IS the run. It publishes /test/plan_result before it loads anything, which is
# what opens the test folder in the recorder, then loads and starts the mission. The recorder
# closes the folder by itself post_roll_s after the mission ends. So this script's job is the
# things around that single call: put the car back, prove the scene is the right one, start a
# side recording for the ground truth, send the command, wait for the mission to end, and tie the
# bag to the folder.
#
# THE RECORDER IS NOT STARTED HERE. test_campaign_logger runs once per SESSION, in its own
# terminal, and survives the stack restarts between missions:
#
#   scripts/sim/hil_orin shell
#   ros2 launch f1tenth_logger test_campaign_logger.launch.py \
#       root:=<the Orin checkout> \
#       config:=$(ros2 pkg prefix f1tenth_logger)/share/f1tenth_logger/config/office_hil_campaign_logger.yaml
#
# Starting it per run would lose the campaign's own repetition counter and would race the
# planner: the folder must already be open when /test/plan_result arrives.
#
# THREE ISAAC SESSIONS, NOT ONE, and the script stops between them on purpose. The scene is fixed
# at Isaac launch, and the three missions need three different scenes: M01 starts somewhere else,
# and M03's person stands ON M02's second leg (0.45 m clear to its left, 0.80 m to its right,
# against the 1.18 m the car needs to pass a person), so M02 cannot be driven in M03's scene.
# Changing the scene means restarting Isaac on the workstation, which this script cannot do: it
# prints what to type there and waits.
#
# WHY NOT scripts/sim/llm.sh OR planner_compare/run.sh: both assume the stack is on the local
# Docker host. llm.sh re-execs itself through sim_isaac.sh run and curls llama-server on
# 127.0.0.1:8083; run.sh brings the stack up with sim_isaac.sh stack. Here the stack, the planner
# and llama-server are all in the Orin container and Isaac is on another machine. What IS reused,
# unchanged: hil_orin (env, preflight, check), restart.sh (through its F1T_RESTART_INNER hook,
# which skips its own sim_isaac.sh re-exec), planner_compare/pc_record.py (ready, record).
if [ -z "${BASH_VERSION:-}" ]; then exec bash "$0" "$@"; fi
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
log() { echo "[campaign $(date +%H:%M:%S)] $*"; }
die() { echo "[campaign] ERROR: $*" >&2; exit 1; }
usage() { awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; }

CAMPAIGN=office_hil_campaign
GUARD_IN_CONTAINER=/tmp/f1tenth_campaign_guard.py
ARCHIVE="$HOME/f1tenth_sim_archive/complete"
CONTROLLER_NODE=mpc_corr          # the campaign is corridor_mpc only, by decision
COUNTDOWN_S=15.0                  # see "THE COUNTDOWN" below
RUN_TIMEOUT=240                   # s, the longest a mission may take before the recorder gives up

# THE COUNTDOWN. mission_countdown_sec has been 0.0 since 2026-10-21, and mission_logger_node's
# rosbag2 recorder needs 2.4-14 s of discovery before its first message lands. At 0 the bag of a
# short mission starts mid-drive or empty. The parameter is read live at every mission start and
# is lost on a stack restart, so it is set (and read back) after every bringup.

# ---- the missions ----------------------------------------------------------------------------
# id|prompt_num|mission dir|Isaac preset|start x,y,yaw_deg|needs a person|the command text
# Every pose comes from sim/isaac/tools/office_missions.py (output/office_missions/report.txt);
# the texts are byte-identical to office_hil_campaign/prompts.yaml, which is what makes the
# recorder's prompt hash match. Order: M02 first, it needs no perception at all.
MISSION_IDS=(M02 M01 M03)
m_def() {
  case "$1" in
    M01) echo "1|M01_door_left_chair|campaign_ma|9.33,-15.87,90|no|go straight, enter the door on the left and then approach the chair" ;;
    M02) echo "2|M02_wall_left_wall|campaign_mb|16.83,-9.87,180|no|go straight at the wall, turn left and then approach the wall" ;;
    M03) echo "3|M03_person_door_right|campaign_md|16.83,-9.87,180|yes|at the wall turn left, then go to the person, then enter the door on the right" ;;
    *) return 1 ;;
  esac
}

# ---- CLI -------------------------------------------------------------------------------------
CMD="${1:-}"
case "$CMD" in
  preflight|translate|mission|all|judge|analyze|bags) shift ;;
  -h|--help|help|"") usage; exit 0 ;;
  *) die "unknown subcommand '$CMD' (preflight|translate|mission|all|judge|analyze|bags; --help)" ;;
esac
DRY=false; REPS=3; GATE=true; TARGET=""
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY=true; shift ;;
    --reps) REPS="${2:?--reps needs a number}"; shift 2 ;;
    --reps=*) REPS="${1#*=}"; shift ;;
    --no-gate) GATE=false; shift ;;
    -h|--help) usage; exit 0 ;;
    M0[123]) TARGET="$1"; shift ;;
    *) die "unknown argument '$1' (--help)" ;;
  esac
done
[[ "$REPS" =~ ^[0-9]+$ ]] && [ "$REPS" -ge 1 ] || die "--reps must be a positive integer, got '$REPS'"

command -v python3 >/dev/null || die "python3 not found"
command -v docker >/dev/null || die "docker not found (this script runs on the Orin HOST)"
eval "$(python3 "$HERE/../hil_config.py" --sh)" || die "cannot read the HIL config"
TMPD="$(mktemp -d /tmp/hil_campaign.XXXXXX)"
trap 'rm -rf "$TMPD"' EXIT

CAMPAIGN_DIR="$REPO/$CAMPAIGN"
OUT="$CAMPAIGN_DIR/aux"
BAG_INDEX="$CAMPAIGN_DIR/bag_index.csv"

# The container must see this checkout at the same path: the campaign folder, prompts.yaml,
# restart.sh and pc_record.py are all read from it in-container. That is what the repo bind mount
# of docker/orin is for (output/hil/run_windows.md).
container_env() {
  CENV=(-e "ROS_DOMAIN_ID=$HIL_ROS_DOMAIN_ID" -e "RMW_IMPLEMENTATION=$HIL_ROS_RMW"
        -e "F1TENTH_DISCOVERY_SERVER=$HIL_DISCOVERY_SERVER" -e "ROS_DISCOVERY_SERVER=$HIL_DISCOVERY_SERVER"
        -e "FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/f1tenth_hil_profile.xml"
        -e ROS_AUTOMATIC_DISCOVERY_RANGE= -e ROS_STATIC_PEERS=)
}
container_env

# THE ENVIRONMENT EVERYTHING RUNS IN, and the one subtlety of this setup: the container's ROS
# workspace is the IMAGE's (/opt/f1tenth_ws), built from whichever branch produced the image --
# NOT the checkout that is bind-mounted in. So `ros2 run f1tenth_logger ...` gets the image's
# f1tenth_logger, even though this script and the campaign folder come from the mount.
#
# That is fine by default: the campaign needs nothing from this branch's f1tenth_logger that the
# image does not already have. The recorder is given its config by ABSOLUTE PATH out of the
# mounted source tree, so it needs no build at all, and campaign_aggregate.py imports
# analyze_tests from the image install, which works.
#
# What the image does NOT have is the two robustness patches taken from origin/doors-enter (one
# corrupted run no longer aborts the whole export; analyze_tests isolates a bad run). If you want
# them, build an OVERLAY of that one package from the mounted tree, in the container:
#
#   cd <repo> && colcon build --symlink-install --packages-select f1tenth_logger
#
# and this function then sources it on top, so the overlay is used consistently by every call
# instead of some of them. Without the overlay nothing here changes behaviour.
SOURCE_ENV=". /opt/f1tenth_ws/scripts/env/jazzy.sh; \
  [ -f $REPO/install/setup.bash ] && . $REPO/install/setup.bash;"

# cexec CMD...     run inside the container, in that environment.
# </dev/null: docker exec keeps stdin open and would otherwise swallow the rest of a piped script.
cexec() {
  $DRY && { echo "  (dry) docker exec $HIL_ORIN_CONTAINER -- $*"; return 0; }
  docker exec "${CENV[@]}" "$HIL_ORIN_CONTAINER" bash -lc \
    "$SOURCE_ENV exec $(printf '%q ' "$@")" < /dev/null
}
cexec_bg() {   # same, backgrounded in its own process group so it can be signalled as a tree
  $DRY && { echo "  (dry, bg) docker exec $HIL_ORIN_CONTAINER -- $*"; return 0; }
  setsid docker exec "${CENV[@]}" "$HIL_ORIN_CONTAINER" bash -lc \
    "$SOURCE_ENV exec $(printf '%q ' "$@")" < /dev/null &
  BG_PID=$!
}
running() { docker ps -q -f "name=^${HIL_ORIN_CONTAINER}$" | grep -q .; }

# ---- the guard / verdict helper, copied into the container like hil_orin does with hil_check ---
# In the container rather than on the host because the host has no ROS. Copied rather than taken
# from the image for the same reason hil_check is: the image was built from whichever branch
# produced it, and these checks must match THIS checkout.
write_guard() {
  cat > "$TMPD/guard.py" <<'PY'
#!/usr/bin/env python3
"""Scene guards and the verdict sheet for hil_campaign.sh. Runs inside the Orin container.

    guard.py scene  X Y YAW_DEG [--person] [--timeout 20]
    guard.py sheet  --gt FILE --target X,Y [--yaw DEG]

`scene` is what stops the right command reaching the wrong scene, which is a real risk here
because two of the three missions share a start pose and differ only by a person. It reads the
LATCHED /sim/ground_truth/start_pose (the pose /sim/reset returns the car to, published by the
sim from the preset) and, with --person, requires a `person` class in the latched
/sim/ground_truth/objects.
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
POS_TOL, YAW_TOL_DEG = 0.02, 1.0        # restart.sh's own tolerances for "back at the start"


def yaw_of(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def cmd_scene(args):
    from vision_msgs.msg import Detection3DArray
    rclpy.init()
    node = Node('campaign_scene_guard')
    got = {}
    node.create_subscription(PoseStamped, '/sim/ground_truth/start_pose',
                             lambda m: got.setdefault('pose', m), LATCHED)
    node.create_subscription(Detection3DArray, '/sim/ground_truth/objects',
                             lambda m: got.setdefault('objects', m), LATCHED)
    deadline = node.get_clock().now().nanoseconds / 1e9 + args.timeout
    need = {'pose'} | ({'objects'} if args.person else set())
    while not need <= set(got) and node.get_clock().now().nanoseconds / 1e9 < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    bad = []
    if 'pose' not in got:
        bad.append('no latched /sim/ground_truth/start_pose in %.0f s -- is Isaac running, and '
                   'on this domain and Discovery Server?' % args.timeout)
    else:
        p = got['pose'].pose
        dx, dy = p.position.x - args.x, p.position.y - args.y
        dist = math.hypot(dx, dy)
        dyaw = (yaw_of(p.orientation) - args.yaw + 180.0) % 360.0 - 180.0
        print('start_pose  (%.3f, %.3f) yaw %.1f deg   expected (%.2f, %.2f) yaw %.1f   '
              'off by %.3f m / %.2f deg' % (p.position.x, p.position.y, yaw_of(p.orientation),
                                            args.x, args.y, args.yaw, dist, dyaw))
        if dist > POS_TOL or abs(dyaw) > YAW_TOL_DEG:
            bad.append('the scene loaded in Isaac is NOT this mission: wrong layout/preset')
    if args.person:
        if 'objects' not in got:
            bad.append('no latched /sim/ground_truth/objects in %.0f s' % args.timeout)
        else:
            classes = [h.hypothesis.class_id for d in got['objects'].detections for h in d.results]
            people = [d for d in got['objects'].detections
                      if any(h.hypothesis.class_id == 'person' for h in d.results)]
            print('gt objects: %s' % (', '.join(sorted(set(classes))) or '(none)'))
            for d in people:
                c = d.bbox.center.position
                print('person at (%.2f, %.2f)' % (c.x, c.y))
            if not people:
                bad.append('this mission needs a person and the scene has none: wrong preset '
                           '(campaign_md), or people.enabled is false')
    node.destroy_node()
    rclpy.shutdown()
    for b in bad:
        print('FAIL: %s' % b, file=sys.stderr)
    return 1 if bad else 0


def cmd_sheet(args):
    """Last ground-truth pose against the mission's target. No ROS: pc_record.py wrote the CSV."""
    import csv
    rows = list(csv.DictReader(open(args.gt)))
    if not rows:
        print('FAIL: %s is empty -- the recorder saw no /sim/ground_truth/odom' % args.gt,
              file=sys.stderr)
        return 1
    last = rows[-1]
    x, y, yaw = float(last['x']), float(last['y']), math.degrees(float(last['yaw']))
    tx, ty = (float(v) for v in args.target.split(','))
    print('final GT pose  (%.3f, %.3f) yaw %.1f deg' % (x, y, yaw))
    print('target         (%.2f, %.2f)%s' % (tx, ty, '' if args.yaw is None else
                                             ' yaw %.1f deg' % args.yaw))
    print('off by         %.3f m%s' % (math.hypot(x - tx, y - ty), '' if args.yaw is None else
          ' / %.2f deg' % ((yaw - args.yaw + 180.0) % 360.0 - 180.0)))
    print('path samples   %d' % len(rows))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('scene')
    s.add_argument('x', type=float)
    s.add_argument('y', type=float)
    s.add_argument('yaw', type=float)
    s.add_argument('--person', action='store_true')
    s.add_argument('--timeout', type=float, default=20.0)
    s.set_defaults(fn=cmd_scene)
    h = sub.add_parser('sheet')
    h.add_argument('--gt', required=True)
    h.add_argument('--target', required=True, help='X,Y')
    h.add_argument('--yaw', type=float, default=None)
    h.set_defaults(fn=cmd_sheet)
    args = ap.parse_args(argv)
    return args.fn(args)


if __name__ == '__main__':
    sys.exit(main())
PY
  $DRY || docker cp "$TMPD/guard.py" "$HIL_ORIN_CONTAINER:$GUARD_IN_CONTAINER" >/dev/null
}

# ---- preflight -------------------------------------------------------------------------------
do_preflight() {
  local rc=0
  running || die "container $HIL_ORIN_CONTAINER is not running (scripts/sim/hil_orin up)"
  log "container $HIL_ORIN_CONTAINER up, domain $HIL_ROS_DOMAIN_ID, DS $HIL_DISCOVERY_SERVER"

  [ -f "$CAMPAIGN_DIR/prompts.yaml" ] || { echo "FAIL: $CAMPAIGN_DIR/prompts.yaml missing"; rc=1; }
  [ -f "$CAMPAIGN_DIR/analysis/run_selection.yaml" ] \
    || { echo "FAIL: $CAMPAIGN_DIR/analysis/run_selection.yaml missing"; rc=1; }

  # the container must see THIS checkout at THIS path, or nothing below resolves
  if ! cexec test -f "$REPO/scripts/sim/restart.sh"; then
    echo "FAIL: the container cannot see $REPO (the repo bind mount is missing:"
    echo "      docker run ... -v $REPO:$REPO ... , see output/hil/run_windows.md)"
    rc=1
  fi
  if ! cexec test -f "$REPO/scripts/sim/planner_compare/pc_record.py"; then
    echo "FAIL: pc_record.py not visible in the container"; rc=1
  fi

  # the campaign's provenance is the root's git commit: it must be the tree the sim side runs
  local head dirty
  head="$(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || echo '?')"
  dirty="$(git -C "$REPO" status --porcelain 2>/dev/null | head -5)"
  log "Orin checkout at $head$([ -n "$dirty" ] && echo ' (DIRTY)')"
  if [ -n "$dirty" ]; then
    echo "WARNING: the Orin tree is dirty; campaign.json will record $head, which is not what"
    echo "         is actually running. Commit or stash before a real session."
    echo "$dirty" | sed 's/^/         /'
  fi
  echo "         compare with the workstation: git rev-parse --short HEAD"

  # the recorder must already be running: it owns the repetition counter
  if cexec ros2 node list 2>/dev/null | grep -q test_campaign_logger; then
    log "test_campaign_logger is up"
  else
    echo "FAIL: test_campaign_logger is NOT running. Start it in its own terminal first"
    echo "      (see this script's header) -- it must be up before any command is sent, or the"
    echo "      run is simply not recorded."
    rc=1
  fi
  [ "$rc" = 0 ] && log "preflight OK" || log "preflight FAILED"
  return $rc
}

# ---- the translation check (no motion) -------------------------------------------------------
# Safe with the recorder running: --dry-run returns before _announce_plan(), so it never
# publishes /test/plan_result and never opens a test folder.
do_translate() {
  local rc=0 id def text
  for id in "${MISSION_IDS[@]}"; do
    def="$(m_def "$id")"
    IFS='|' read -r _ _ _ _ _ text <<< "$def"
    echo
    log "$id --dry-run: \"$text\""
    if cexec ros2 run llm llm_planner_node --dry-run "$text" 2>&1 | tee "$TMPD/$id.log" \
         | grep -E 'RESULT|unsupported|plan' ; then :; fi
    grep -q '"status": "ok"' "$TMPD/$id.log" || { echo "FAIL: $id did not translate"; rc=1; }
    if grep -q '"unsupported": \[[^]]' "$TMPD/$id.log"; then
      echo "FAIL: $id produced a non-empty \`unsupported\`. The planner will ask [s/N] and the"
      echo "      session will stall. Rephrase in prompts.yaml (an ordinal does this)."
      rc=1
    fi
  done
  echo
  [ "$rc" = 0 ] && log "all three commands translate, nothing unsupported" || log "translation FAILED"
  return $rc
}

# ---- one repetition --------------------------------------------------------------------------
reset_car() {
  log "reset: /reset_all and the five checks"
  $DRY && { echo "  (dry) restart.sh inner"; return 0; }
  docker exec "${CENV[@]}" -e F1T_RESTART_INNER=1 -e F1T_RESTART_SIM_ONLY=false \
    -e F1T_RESTART_SETTLE=5 -e F1T_RESTART_START= "$HIL_ORIN_CONTAINER" bash -lc \
    "$SOURCE_ENV exec bash $REPO/scripts/sim/restart.sh" < /dev/null
}

newest_test_dir() {   # the folder the recorder just closed, for this mission
  ls -1dt "$CAMPAIGN_DIR/$1"/P*-R*-* 2>/dev/null | head -1
}

index_bag() {   # $1 test dir, $2 the epoch second the repetition started
  local test_dir="$1" since="$2" d manifest
  [ -d "$ARCHIVE" ] || { log "no $ARCHIVE yet -- is enable_mission_logger true?"; return 0; }
  # the newest completed run whose directory appeared after this repetition began. The manifest's
  # mission_id is a sha1 of the INTENT, so it is identical for all repetitions of one command:
  # time is what separates them, not the id.
  d="$(find "$ARCHIVE" -maxdepth 1 -mindepth 1 -type d -newermt "@$since" 2>/dev/null \
        | sort | tail -1)"
  [ -n "$d" ] || { log "no new bag in $ARCHIVE for this run"; return 0; }
  manifest="$(ls -1 "$d"/*.manifest.json 2>/dev/null | head -1)"
  [ -s "$BAG_INDEX" ] || echo "test_id,bag_run_id,bag_dir,mission_id,outcome,bag_bytes" > "$BAG_INDEX"
  python3 - "$test_dir" "$d" "$manifest" "$BAG_INDEX" <<'PY'
import json, os, sys
test_dir, bag_dir, manifest, index = sys.argv[1:5]
m = json.load(open(manifest)) if manifest and os.path.exists(manifest) else {}
row = dict(test_id=os.path.basename(test_dir), bag_run_id=os.path.basename(bag_dir),
           bag_dir=bag_dir, mission_id=m.get('mission_id', ''),
           outcome=m.get('outcome', ''), bag_bytes=m.get('bag_bytes', ''))
with open(index, 'a') as f:
    f.write('{test_id},{bag_run_id},{bag_dir},{mission_id},{outcome},{bag_bytes}\n'.format(**row))
# the sidecar goes INSIDE the closed test folder: additive, and the export ignores files it does
# not know, so the run carries its own pointer to its bag wherever it is copied.
if os.path.isdir(test_dir):
    json.dump(row, open(os.path.join(test_dir, 'bag_ref.json'), 'w'), indent=1)
print('bag: %s  outcome=%s  %s B' % (row['bag_run_id'], row['outcome'], row['bag_bytes']))
PY
}

run_once() {   # $1 id, $2 prompt_num, $3 mission dir, $4 x,y,yaw, $5 needs person, $6 text, $7 rep
  local id="$1" pn="$2" mdir="$3" start="$4" person="$5" text="$6" rep="$7"
  local x="${start%%,*}" rest="${start#*,}"; local y="${rest%%,*}" yaw="${rest#*,}"
  local pflag=(); [ "$person" = yes ] && pflag=(--person)
  echo
  log "=== $id repetition $rep of $REPS ==="

  cexec python3 "$GUARD_IN_CONTAINER" scene "$x" "$y" "$yaw" "${pflag[@]}" \
    || die "scene guard failed -- not sending the command (see above)"

  reset_car || { log "restart.sh FAILED a check: repetition NOT started (that is the finding)"; return 1; }

  local started aux
  started=$(date +%s)
  aux="$OUT/$id/$(date +%Y%m%dT%H%M%S)"
  mkdir -p "$aux"
  # the side recording: ground truth in the MAP frame, the mission trail and /drive, which is what
  # the verdict is read off. It also owns the "when is the run over" decision: it exits when
  # /mission/status leaves RUNNING.
  cexec_bg python3 "$REPO/scripts/sim/planner_compare/pc_record.py" record \
      --node "$CONTROLLER_NODE" --out "$aux" --timeout "$RUN_TIMEOUT"
  local waited=0
  while [ ! -f "$aux/RECORDING" ] && [ "$waited" -lt 30 ]; do sleep 1; waited=$((waited + 1)); done
  [ -f "$aux/RECORDING" ] || { $DRY || log "WARNING: the recorder did not start in 30 s"; }
  sleep 2   # subscriptions matched before the car moves

  log "sending: \"$text\" (--prompt-num $pn)"
  cexec ros2 run llm llm_planner_node "$text" --prompt-num "$pn" 2>&1 | tee "$aux/llm.log" || true
  grep -q '"status": "ok"' "$aux/llm.log" 2>/dev/null \
    || log "the planner did not deliver a mission -- this still counts as a run (and is a finding)"

  log "waiting for the mission to end (recorder timeout ${RUN_TIMEOUT}s)"
  $DRY || wait "$BG_PID" 2>/dev/null || true

  local test_dir; test_dir="$(newest_test_dir "$mdir")"
  if [ -z "$test_dir" ]; then
    log "WARNING: no test folder under $CAMPAIGN_DIR/$mdir -- was test_campaign_logger running?"
  else
    log "test folder: $(basename "$test_dir")"
    index_bag "$test_dir" "$started" || true
    verdict_sheet "$id" "$test_dir" "$aux" | tee "$aux/verdict_sheet.txt"
  fi
}

verdict_sheet() {   # what the judge needs, in one place, in MAP coordinates
  local id="$1" test_dir="$2" aux="$3" target yaw
  case "$id" in
    M01) target="5.02,-13.75"; yaw="" ;;        # 0.5 m from ConfOfficeChairBlack5
    M02) target="11.63,-21.62"; yaw="90" ;;     # 2.0 m from the far wall, heading south
    M03) target="8.08,-13.07"; yaw="180" ;;     # standing in the doorway, facing through it
  esac
  echo "---- verdict sheet: $(basename "$test_dir") ----"
  python3 - "$CAMPAIGN_DIR/prompts.yaml" "$id" <<'PY' 2>/dev/null || true
import sys
# no PyYAML on the Orin host necessarily: print the criterion line verbatim
want = {'M01': 'M01_', 'M02': 'M02_', 'M03': 'M03_'}[sys.argv[2]]
show = False
for line in open(sys.argv[1], encoding='utf-8'):
    if line.startswith('- prompt_num'):
        show = False
    if want in line and 'mission:' in line:
        show = True
    if show and ('text:' in line or 'success_criterion:' in line):
        print('  ' + line.strip())
PY
  if [ -f "$aux/gt.csv" ]; then
    cexec python3 "$GUARD_IN_CONTAINER" sheet --gt "$aux/gt.csv" --target "$target" \
      ${yaw:+--yaw "$yaw"} 2>/dev/null | sed 's/^/  /' || true
  fi
  [ -f "$aux/mission.csv" ] && { echo "  mission trail:"; sed 's/^/    /' "$aux/mission.csv"; }
  [ -f "$test_dir/events.jsonl" ] && { echo "  events:"; sed 's/^/    /' "$test_dir/events.jsonl"; }
  [ -f "$test_dir/bag_ref.json" ] && { echo "  bag:"; sed 's/^/    /' "$test_dir/bag_ref.json"; }
  echo "  judge: fill success / transl_ok / notes for this test_id in campaign_results.csv"
}

# ---- per mission -----------------------------------------------------------------------------
mission_setup() {   # $1 id -- once per mission, after its stack bringup
  local id="$1"
  log "$id: stack readiness ($CONTROLLER_NODE), then the mission countdown"
  cexec python3 "$REPO/scripts/sim/planner_compare/pc_record.py" ready \
      --node "$CONTROLLER_NODE" --timeout 300 \
    || die "the stack is not ready (or a second controller is in the graph -- a leftover bringup)"
  cexec ros2 param set /behavior_executor_node mission_countdown_sec "$COUNTDOWN_S" || true
  local got
  got="$(cexec ros2 param get /behavior_executor_node mission_countdown_sec 2>/dev/null | tail -1)"
  log "mission_countdown_sec: $got (wanted $COUNTDOWN_S, so the bag has time to discover)"
}

boundary() {   # $1 id, $2 preset, $3 index, $4 total
  cat <<TXT

=================== mission $3 of $4: $1 (Isaac preset $2) ===================
The scene is fixed at Isaac launch, so this is a manual step. In order:

 1. Orin, the stack terminal: Ctrl+C, then WAIT 8 s.
    (5 s Fast DDS participant lease: the next bringup must not see this one's nodes.)
 2. Workstation: Ctrl+C Isaac, then
      scripts\\sim\\win\\hil_sim_link.bat --layout $2
    Wait for RTF 1.000 in scripts\\sim\\win\\run.log.
 3. Orin, the stack terminal:
      scripts/sim/hil_orin stack enable_intelligence:=true enable_mission_logger:=true \\
            planner_controller_mode:=corridor_mpc
    llama-server reloads the model (20-30 s): curl 127.0.0.1:8083/health must answer 200.
 4. Leave the test_campaign_logger terminal ALONE -- it survives this.

Then press Enter here: the scene, the graph and the controller are verified before any run.
TXT
  read -r _
}

do_mission() {   # $1 id
  local id="$1" def pn mdir preset start person text rep
  def="$(m_def "$id")" || die "unknown mission '$id'"
  IFS='|' read -r pn mdir preset start person text <<< "$def"
  mkdir -p "$OUT/$id"
  mission_setup "$id"
  local failed=0
  for rep in $(seq 1 "$REPS"); do
    run_once "$id" "$pn" "$mdir" "$start" "$person" "$text" "$rep" || failed=$((failed + 1))
  done
  [ "$failed" = 0 ] || log "$id: $failed repetition(s) were not started (reset checks failed)"
  echo
  log "$id: $REPS repetition(s) attempted. Judging now, before the next mission."
  do_judge "$mdir"
}

# ---- judging and analysis --------------------------------------------------------------------
# JUDGE PER MISSION, NOT AT THE END. first_test_campaign's analysis/report.txt reads
# "evaluated by hand: 0", so every k/N in it is 0/0: 75 runs of data and no verdicts, because the
# judging was left until after the sessions. Three five-minute stops are the fix.
do_judge() {
  local mdir="${1:-}"
  cexec ros2 run f1tenth_logger test_campaign_export --plain "$CAMPAIGN_DIR" || true
  local csv="$CAMPAIGN_DIR/campaign_results.csv"
  [ -f "$csv" ] || { log "no campaign_results.csv yet"; return 0; }
  local unjudged
  unjudged="$(python3 - "$csv" "$mdir" <<'PY'
import csv, sys
path, mdir = sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else ''
rows = list(csv.DictReader(open(path, newline='', encoding='utf-8', errors='replace')))
open_rows = [r for r in rows if (not mdir or r.get('mission') == mdir)
             and not (r.get('success') or '').strip()]
for r in open_rows:
    print(r.get('test_id', '?'))
PY
)"
  if [ -z "$unjudged" ]; then
    log "every run of ${mdir:-the campaign} has a verdict"
    return 0
  fi
  echo
  echo "UNJUDGED runs in ${mdir:-the campaign}:"
  echo "$unjudged" | sed 's/^/  /'
  echo
  echo 'Fill success (1/0), transl_ok and notes for these rows in'
  echo "  $CAMPAIGN_DIR/campaign_results.csv"
  echo "The evidence is already written: each run's aux/<ts>/verdict_sheet.txt (final GT pose vs"
  echo "the target, the mission trail), its corridor_plot.png, and its bag (bag_ref.json)."
  if $GATE; then
    die "refusing to continue with unjudged runs (--no-gate overrides, and then nobody judges them)"
  fi
}

do_analyze() {
  cexec ros2 run f1tenth_logger test_campaign_export --plain "$CAMPAIGN_DIR" || true
  cexec ros2 run f1tenth_logger test_campaign_corridor_plot --campaign "$CAMPAIGN_DIR" || true
  do_judge ""
  cexec ros2 run f1tenth_logger test_campaign_analyze "$CAMPAIGN_DIR" \
        --footprints 2.0 --corridor-plots
  local id def mdir
  for id in "${MISSION_IDS[@]}"; do
    def="$(m_def "$id")"; IFS='|' read -r _ mdir _ _ _ _ <<< "$def"
    cexec ros2 run f1tenth_logger test_campaign_analyze "$CAMPAIGN_DIR" --mission "$mdir" \
      || true
  done
  cexec python3 "$REPO/tools/campaign_aggregate.py" "$CAMPAIGN_DIR"
  log "analysis written under $CAMPAIGN_DIR/analysis"
  log "the campaign is done when analysis/report.txt says 'not yet evaluated: 0'"
}

do_bags() {
  [ -f "$BAG_INDEX" ] || { log "no $BAG_INDEX yet"; return 0; }
  column -s, -t < "$BAG_INDEX" 2>/dev/null || cat "$BAG_INDEX"
  echo
  echo "pull them to the workstation with, e.g.:"
  echo "  rsync -a --info=progress2 $HIL_ORIN_SSH:f1tenth_sim_archive/complete/ <dest>/"
}

# ---- dispatch --------------------------------------------------------------------------------
case "$CMD" in
  preflight) do_preflight ;;
  translate)
    running || die "container $HIL_ORIN_CONTAINER is not running (scripts/sim/hil_orin up)"
    do_translate ;;
  mission)
    [ -n "$TARGET" ] || die "mission needs an id (M01|M02|M03)"
    write_guard; do_preflight || die "preflight failed"; do_mission "$TARGET" ;;
  all)
    write_guard; do_preflight || die "preflight failed"
    i=0; n=${#MISSION_IDS[@]}
    for id in "${MISSION_IDS[@]}"; do
      i=$((i + 1))
      def="$(m_def "$id")"; IFS='|' read -r _ _ preset _ _ _ <<< "$def"
      [ "$i" = 1 ] || boundary "$id" "$preset" "$i" "$n"
      do_mission "$id"
    done
    do_analyze ;;
  judge)
    if [ -n "$TARGET" ]; then def="$(m_def "$TARGET")"; IFS='|' read -r _ mdir _ _ _ _ <<< "$def"
    else mdir=""; fi
    GATE=false; do_judge "$mdir" ;;
  analyze) do_analyze ;;
  bags) do_bags ;;
esac
