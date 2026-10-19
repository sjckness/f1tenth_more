#!/usr/bin/env bash
# CPU test for the F1TENTH stack on the Orin. ONE COMMAND, NO SUDO.
#
#   tools/cpu_test.sh                        # all four phases, 60 s each
#   tools/cpu_test.sh --phase a              # one phase
#   tools/cpu_test.sh --no-motion            # c and d without driving
#   tools/cpu_test.sh --dry-run              # NO SENSORS: a bag plays instead
#
# Everything lands in output/cpu_test/<timestamp>/ with a summary.md at the end.
#
# THE FOUR PHASES, 60 s each:
#   a  stack up, car stationary, mission logger OFF
#   b  stack up, car stationary, mission logger ON and recording
#   c  the corridor10m mission, logger OFF
#   d  the corridor10m mission, logger ON
# a->b is what recording costs. c->d is what it costs while the stack is busy.
#
# WHAT IT DOES NOT DO: it never launches a sensor driver, a VESC, the ackermann
# mux or anything else that can move the car, and it never asks for sudo. In
# phases c and d it WAITS for you to start and finish the mission unless
# --no-motion is given; the car on a stand is what --no-motion is for.
#
# WHAT MEASURES WHAT:
#   tegrastats 1 s         per-core load, GPU, temperatures, clocks, rails
#   pidstat -u -t -d 1     per-thread CPU and per-process I/O -- IF INSTALLED.
#                          It is not, on this box (no sysstat), so
#                          cpu_test_probe.py reads the same numbers out of
#                          /proc without root. The summary says which ran.
#   cpu_test_probe.py rates   the publish rate of each topic OUTSIDE any bag,
#                          because a bag under load loses messages (the
#                          nightly audit: 24 % of /scan missing from a
#                          recording the LiDAR published at 40 Hz)
#   the nodes' own JSON    tracker_ms, the mask tick's ms, the MPC solve time
#
# nvpmodel -q and the CPU clocks are captured per phase. `jetson_clocks --show`
# needs root and is NOT run with sudo: the script records its refusal and reads
# /sys/devices/system/cpu/*/cpufreq instead, which needs nothing.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/.." && pwd)"
DURATION=60
PHASES="a b c d"
NO_MOTION=0
DRY_RUN=0
OUT=""
BAG="${HOME}/raw_data/corridor10m_20261015T171753"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --phase)     PHASES="$2"; shift 2 ;;
        --duration)  DURATION="$2"; shift 2 ;;
        --no-motion) NO_MOTION=1; shift ;;
        --dry-run)   DRY_RUN=1; NO_MOTION=1; shift ;;
        --out)       OUT="$2"; shift 2 ;;
        --bag)       BAG="$2"; shift 2 ;;
        -h|--help)   sed -n '2,40p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done
[[ -n "$OUT" ]] || OUT="${WS}/output/cpu_test/$(date +%Y%m%dT%H%M%S)"
mkdir -p "$OUT"

# Every background pid this script starts, and nothing else. The trap kills
# only these, so a Ctrl-C can never take the stack down with it.
declare -a OWNED=()
# EVERY `ros2 launch` CHILD GETS ITS OWN PROCESS GROUP and the cleanup kills
# the GROUP. Killing the launch pid alone is not enough: a dry run left two
# orphaned depth_throttle_5hz nodes behind because launch was killed before it
# had forwarded the signal to what it had started. A pkill by node name would
# be the wrong fix -- on the car it would reach the live stack's own nodes --
# so this script owns process groups and signals nothing outside them.
# Stop everything this script owns, groups and all, and forget them. Used by
# the trap AND by the per-phase dry-run teardown -- the per-phase one used to
# kill bare pids, which is where the orphans actually came from.
stop_owned() {
    [[ ${#OWNED[@]} -eq 0 ]] && return 0
    echo "    stopping ${#OWNED[@]} process(es)/group(s) this script started..."
    for pid in "${OWNED[@]:-}"; do
        kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
    done
    sleep 3
    for pid in "${OWNED[@]:-}"; do
        kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    OWNED=()
}

cleanup() {
    local rc=$?
    trap - EXIT INT TERM
    echo
    stop_owned
    echo "done: $OUT"
    exit $rc
}
trap cleanup EXIT INT TERM

own() { OWNED+=("$1"); }

# A launch tree, in its own process group, so cleanup() can take all of it.
spawn_group() { setsid "$@" & own $!; }

have() { command -v "$1" >/dev/null 2>&1; }

snapshot_static() {   # $1 = phase dir
    local d="$1"
    {
        echo "# nvpmodel -q"
        nvpmodel -q 2>&1 || echo "(nvpmodel failed)"
        echo
        echo "# jetson_clocks --show (NOT run with sudo on purpose)"
        jetson_clocks --show 2>&1 || true
        echo
        echo "# /sys cpufreq, which needs no root"
        for c in /sys/devices/system/cpu/cpu[0-9]*; do
            local n; n="$(basename "$c")"
            printf '%s cur=%s min=%s max=%s gov=%s\n' "$n" \
                "$(cat "$c/cpufreq/scaling_cur_freq" 2>/dev/null || echo -)" \
                "$(cat "$c/cpufreq/scaling_min_freq" 2>/dev/null || echo -)" \
                "$(cat "$c/cpufreq/scaling_max_freq" 2>/dev/null || echo -)" \
                "$(cat "$c/cpufreq/scaling_governor" 2>/dev/null || echo -)"
        done
        echo
        echo "# thermal zones"
        for z in /sys/devices/virtual/thermal/thermal_zone*; do
            printf '%s %s %s\n' "$(basename "$z")" \
                "$(cat "$z/type" 2>/dev/null || echo -)" \
                "$(cat "$z/temp" 2>/dev/null || echo -)"
        done
        echo
        echo "# the stack's processes and their affinity"
        python3 - <<'PYEOF'
import os, sys
sys.path.insert(0, os.environ['WS'] + '/tools')
from cpu_test_probe import stack_pids
for pid, (name, cmd) in sorted(stack_pids().items()):
    try:
        aff = sorted(os.sched_getaffinity(pid))
    except OSError:
        aff = []
    nthreads = len(os.listdir(f'/proc/{pid}/task')) if os.path.isdir(f'/proc/{pid}/task') else 0
    print(f'{pid:7d} {name:34s} threads={nthreads:3d} affinity={aff}')
PYEOF
    } > "$d/static.txt" 2>&1
}

dry_run_inputs() {    # $1 = phase dir, $2 = phase letter
    local d="$1" phase="$2"
    echo "  dry run: lidar_front_wall.launch.py + ros2 bag play (NO SENSORS)"
    # THE DISCOVERY SERVER IS THE STACK'"'"'S, AND IN A DRY RUN IT IS NOT RUNNING.
    # ROS_DISCOVERY_SERVER is set in this shell'"'"'s environment (127.0.0.1:11811)
    # and supervisor_bringup is what starts the server; with the stack down,
    # every participant points at a server that is not there and NOTHING
    # discovers anything -- the first dry run of this script measured 0 Hz on
    # every topic for exactly that reason. Unsetting it for the dry run'"'"'s own
    # children puts them on plain multicast discovery, which also means they
    # cannot see, or be seen by, a live stack. On the car this is NOT done:
    # there the real stack and this script must share the server.
    unset ROS_DISCOVERY_SERVER
    spawn_group ros2 launch f1tenth_perception lidar_front_wall.launch.py \
        > "$d/lidar_front_wall.log" 2>&1
    if [[ "$phase" == "b" || "$phase" == "d" ]]; then
        spawn_group ros2 launch f1tenth_logger mission_logger.launch.py \
            enable_mission_logger:=true > "$d/mission_logger.log" 2>&1
    fi
    sleep 8
    if [[ "$phase" == "b" || "$phase" == "d" ]]; then
        # The recorder starts on a /mission/status transition and on nothing
        # else (its own docstring). Nothing else is running in a dry run, so
        # publishing one is safe and is the only way to measure it recording.
        # AT A RATE, UNDER A timeout: `ros2 topic pub -1` exits only once it
        # believes the message is away and BLOCKS FOREVER when no subscriber
        # has matched yet, which hung the first dry run of this script; and
        # the recorder needs 5-14 s of discovery on this box before it is
        # subscribed at all, so one message would be lost anyway.
        timeout 12 ros2 topic pub -r 2 /mission/status \
            f1tenth_messages/msg/MissionStatus \
            '{state: RUNNING, json_path: "", emergency_stop_active: false}' \
            > "$d/mission_status.log" 2>&1 || true
        sleep 4
    fi
    ros2 bag play "$BAG" --clock --loop --rate 1.0 \
        --topics /scan /odometry/filtered /tf /tf_static /camera/detections_3d \
                 /perception/stereo_columns /perception/wall_points \
                 /costmap/front_clearance /zed2/zed_node/depth/camera_info \
                 /perception/scene_labels /perception/scene_labels/stats \
        > "$d/bag_play.log" 2>&1 &
    own $!
    sleep 4
}

stop_dry_run() {      # $1 = phase letter
    local phase="$1"
    if [[ "$phase" == "b" || "$phase" == "d" ]]; then
        timeout 10 ros2 topic pub -r 2 /mission/status \
            f1tenth_messages/msg/MissionStatus \
            '{state: COMPLETE, json_path: "", emergency_stop_active: false}' \
            >/dev/null 2>&1 || true
        sleep 4
    fi
}

run_phase() {
    local phase="$1"
    local d="$OUT/phase_$phase"
    mkdir -p "$d"
    echo
    echo "=== phase $phase (${DURATION}s) -> $d"
    case "$phase" in
        a) echo "    stationary, mission logger OFF" ;;
        b) echo "    stationary, mission logger ON (recording)" ;;
        c) echo "    the corridor10m mission, logger OFF" ;;
        d) echo "    the corridor10m mission, logger ON" ;;
    esac

    if [[ $DRY_RUN -eq 1 ]]; then
        dry_run_inputs "$d" "$phase"
    else
        if [[ "$phase" == "b" || "$phase" == "d" ]]; then
            cat <<'MSG'
    THIS PHASE NEEDS THE LOGGER ON. The stack must already be up with
      ros2 launch f1tenth_bringup supervisor_bringup.launch.py enable_mission_logger:=true
    and a mission must be RUNNING for the recorder to be recording.
MSG
        fi
        if [[ "$phase" == "c" || "$phase" == "d" ]] && [[ $NO_MOTION -eq 0 ]]; then
            read -r -p "    start the corridor10m mission now, then press Enter: " _
        fi
    fi

    snapshot_static "$d"

    tegrastats --interval 1000 > "$d/tegrastats.log" 2>&1 &
    own $!
    local TEGRA=$!

    if have pidstat; then
        echo "pidstat" > "$d/cpu_source.txt"
        pidstat -u -t -d 1 "$DURATION" > "$d/pidstat.txt" 2>&1 &
        own $!
    else
        echo "/proc via cpu_test_probe.py (pidstat not installed)" > "$d/cpu_source.txt"
        python3 "$HERE/cpu_test_probe.py" procstat --out "$d" \
            --duration "$DURATION" > "$d/procstat.log" 2>&1 &
        own $!
    fi

    python3 "$HERE/cpu_test_probe.py" rates --out "$d" --duration "$DURATION" \
        > "$d/rates.log" 2>&1 &
    own $!

    sleep "$DURATION"
    sleep 3                       # let the samplers write their files

    kill -INT "$TEGRA" 2>/dev/null || true
    if [[ $DRY_RUN -eq 1 ]]; then
        stop_dry_run "$phase"
        stop_owned
    fi
    echo "    phase $phase done"
}

export WS
echo "cpu_test: out=$OUT phases='$PHASES' duration=${DURATION}s dry_run=$DRY_RUN"
{
    echo "argv: $0 $*"
    echo "date: $(date -Iseconds)"
    echo "host: $(uname -a)"
    echo "git:  $(git -C "$WS" rev-parse --short HEAD 2>/dev/null || echo unknown) \
$(git -C "$WS" status --porcelain 2>/dev/null | grep -c . || echo 0) modified file(s)"
    echo "dry_run: $DRY_RUN  no_motion: $NO_MOTION  duration: $DURATION"
    echo "bag: $BAG"
} > "$OUT/run.txt"

for phase in $PHASES; do
    run_phase "$phase"
done

python3 "$HERE/cpu_test_summary.py" --out "$OUT" || echo "summary failed" >&2
echo
echo "summary: $OUT/summary.md"
