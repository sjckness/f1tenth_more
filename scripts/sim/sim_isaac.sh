#!/usr/bin/env bash
# sim_isaac.sh -- the car's stack driving the Isaac Sim car (~/isaac_ws, branch f1tenth).
# The stack is the SAME bringup as on the car (supervisor_bringup.launch.py), started with
# sim:=isaac: device drivers off, every node on /clock, config/sim/isaac.yaml loaded after
# the normal parameters (output/jazzy_port/phase3.md).
#
# Usage:
#   scripts/sim/sim_isaac.sh isaac [--scene base_room|office|hospital] [--layout L] [--gui]  # sim host (linus)
#   scripts/sim/sim_isaac.sh stack [ros2-launch args ...]                        # stack host (Thor or linus)
#   scripts/sim/sim_isaac.sh all   [--scene ...] [--gui] [ros2-launch args ...]  # both on linus; Ctrl+C stops both
#   scripts/sim/sim_isaac.sh reset [--net eth]                                   # call /sim/reset
#   scripts/sim/sim_isaac.sh shell [--net eth]                                   # interactive shell, stack's ROS env
#   scripts/sim/sim_isaac.sh run [--net eth] [--] CMD ...                        # one command in that env
#     e.g. scripts/sim/sim_isaac.sh run --net eth ros2 run llm llm_planner_node "go to the person"
#
# Options (any subcommand):
#   --net local|eth   ROS network (default local):
#                       local  everything on this machine: Fast DDS on loopback + shared memory
#                              (config/sim/dds/local.xml, Isaac: isaac_local.xml); nothing on the
#                              LAN can join, whatever its ROS_DOMAIN_ID.
#                       eth    two machines on the direct Ethernet link: UDP only on the link
#                              interface (config/sim/dds/eth.xml.in, rendered with this host's
#                              address on the route to the Thor), Discovery Server on the Thor.
#   --domain N        ROS_DOMAIN_ID (default: $ROS_DOMAIN_ID if exported, else 42 as in sim_net.sh)
#   --scene NAME      Isaac scene (isaac/all; default base_room)
#   --layout NAME     scene layout preset (isaac/all; base_room: campaign (default), obstacle,
#                     gazebo_6x6, e2e_legacy; see ~/isaac_ws/config/scenes/base_room.yaml)
#   --gui             Isaac Sim GUI (isaac/all; default headless)
#   --clean           scripts/kill_ros2.py -y first (stack/all)
#   --dry-run         print the environment and the commands, launch nothing
#   -h, --help        this text
# Any other argument of 'stack'/'all' is passed to supervisor_bringup.launch.py (k:=v), e.g.
# enable_intelligence:=true (llama-server = the 'intelligence' component, as on the car).
#
# shell/run: the stack's ROS environment (domain, RMW, the rendered Fast DDS profile, the
# Discovery Server, the workspace) as a Discovery Server SUPER CLIENT, so `ros2 topic list`
# and `echo` see the whole graph. The shell stops the ros2 daemon of this domain first (a
# daemon started from a plain client environment would answer as a plain client).
#
# Two machines (direct Ethernet link; same network variables as the Gazebo scripts' sim_net.sh):
#   1. if the Thor's link address is not 10.42.0.2, on BOTH machines:
#        export F1TENTH_SIM_DISCOVERY_IP=<THOR_LINK_IP>
#   2. linus (Isaac Sim, a Discovery Server client of the Thor):
#        scripts/sim/sim_isaac.sh isaac --net eth --scene base_room
#   3. Thor  (Fast DDS Discovery Server bound to F1TENTH_SIM_DISCOVERY_IP + the stack):
#        scripts/sim/sim_isaac.sh stack --net eth
#   Either side may start first: Isaac retries the Discovery Server, and 'stack' starts it,
#   then waits up to 30 s for /clock before launching (start 'isaac' first: it needs 20-70 s).
#   Each side whitelists its own address on the route to F1TENTH_SIM_DISCOVERY_IP.
#
# Environment (all optional):
#   ISAAC_WS                    Isaac workspace (default ~/isaac_ws)
#   F1TENTH_SIM_DISCOVERY_IP    the Thor's link address (net=eth; default 10.42.0.2, sim_net.sh)
#   F1TENTH_SIM_DISCOVERY_PORT  Discovery Server port (default 11811, stack_params discovery_server_port)
#   F1TENTH_EXTRA_SETUP  file sourced before the workspace on hosts that lack apt packages
#                        (default ~/dev_ws/jc_env.sh, skipped if absent; linus today)
#   F1T_SIM_HOST_OVERLAY sim_overlay_extra for supervisor_bringup (default: config/sim/hosts/
#                        linus.yaml when this host has fewer than 12 CPUs; 'none' to disable)
#   F1T_SIM_CLOCK_WAIT   seconds 'stack' waits for /clock (default 30)
#   F1T_SIM_LOG_DIR      where 'all' writes isaac.log (default: a temporary dir, removed on exit)
if [ -z "${BASH_VERSION:-}" ]; then exec bash "$0" "$@"; fi
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"

# ---- network (defaults as in the Gazebo scripts' sim_net.sh on the jazzy branch) -------
: "${F1TENTH_SIM_DISCOVERY_IP:=10.42.0.2}"     # link IP of the Discovery Server host (the Thor)
: "${F1TENTH_SIM_DISCOVERY_PORT:=11811}"

ISAAC_WS="${ISAAC_WS:-$HOME/isaac_ws}"
ISAAC_IMAGE="isaac_ws_devcontainer-isaac-sim:latest"
ISAAC_CONTAINER="${ISAAC_CONTAINER_NAME:-f1t_isaac}"
MIN_FREE_DISK_GB=25
CLOCK_WAIT="${F1T_SIM_CLOCK_WAIT:-30}"

usage() { awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; }
log() { echo "[sim_isaac $(date +%H:%M:%S)] $*"; }
die() { echo "[sim_isaac] ERROR: $*" >&2; exit 1; }

# ---- arguments ---------------------------------------------------------------------------
CMD="${1:-}"
case "$CMD" in
  isaac|stack|all|reset|shell|run) shift ;;
  -h|--help|help|"") usage; exit 0 ;;
  *) die "unknown subcommand '$CMD' (isaac|stack|all|reset|shell|run; --help)" ;;
esac
NET=local; DOMAIN="${ROS_DOMAIN_ID:-42}"; SCENE=base_room; LAYOUT=""; GUI=false; CLEAN=false; DRY=false
LAUNCH_ARGS=(); RUN_CMD=()
while [ $# -gt 0 ]; do
  if [ "$CMD" = run ] && [ ${#RUN_CMD[@]} -eq 0 ]; then
    case "$1" in
      --net|--net=*|--domain|--dry-run|-h|--help) ;;   # options before the command
      --) shift; RUN_CMD=("$@"); break ;;
      *) RUN_CMD=("$@"); break ;;
    esac
  fi
  case "$1" in
    --net) NET="${2:?--net needs local|eth}"; shift 2 ;;
    --net=*) NET="${1#*=}"; shift ;;
    --domain) DOMAIN="${2:?--domain needs a number}"; shift 2 ;;
    --scene) SCENE="${2:?--scene needs a name}"; shift 2 ;;
    --scene=*) SCENE="${1#*=}"; shift ;;
    --layout) LAYOUT="${2:?--layout needs a preset name}"; shift 2 ;;
    --layout=*) LAYOUT="${1#*=}"; shift ;;
    --gui) GUI=true; shift ;;
    --clean) CLEAN=true; shift ;;
    --dry-run) DRY=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *:=*) LAUNCH_ARGS+=("$1"); shift ;;
    *) die "unknown option '$1' (--help)" ;;
  esac
done
case "$NET" in local|eth) ;; *) die "--net must be local or eth, got '$NET'" ;; esac
[ "$CMD" = run ] && [ ${#RUN_CMD[@]} -eq 0 ] && die "run: no command given (e.g. run ros2 topic list)"
case "$DOMAIN" in ''|*[!0-9]*) die "--domain must be a number, got '$DOMAIN'" ;; esac
if [ "$NET" = eth ]; then
  DS_ADDR="$F1TENTH_SIM_DISCOVERY_IP"
  # This host's address on the route to the Thor (the Thor itself: its own link address).
  ROUTE="$(ip -o -4 route get "$DS_ADDR" 2>&1)" \
    || die "no route to F1TENTH_SIM_DISCOVERY_IP=$DS_ADDR: $ROUTE (is the link up? see --help)"
  # A route via a gateway means the direct link is down and traffic would leave on the LAN.
  case "$ROUTE" in
    *" via "*) die "the route to $DS_ADDR goes via a gateway, not the direct link ($ROUTE): is the cable up?" ;;
  esac
  LINK_IP="$(sed -n 's/.* src \([0-9.]*\).*/\1/p' <<< "$ROUTE")"
  [ -n "$LINK_IP" ] || die "no source address on the route to $DS_ADDR ($ROUTE)"
else
  DS_ADDR=127.0.0.1
  LINK_IP=127.0.0.1
fi
DS_PORT="$F1TENTH_SIM_DISCOVERY_PORT"
DS="$DS_ADDR:$DS_PORT"
DDS_SRC="$REPO/src/f1tenth_bringup/config/sim/dds"
TMPD="$(mktemp -d /tmp/sim_isaac.XXXXXX)"

render_eth_profile() {  # <with_shm:true|false> -> path (this host's link address)
  local shm="$1" out="$TMPD/eth_$1.xml" t=''
  [ "$shm" = true ] && t='<transport_id>f1t_shm</transport_id>'
  sed -e "s/@ETH_IP@/$LINK_IP/" -e "s#@SHM_TRANSPORT@#$t#" "$DDS_SRC/eth.xml.in" > "$out"
  echo "$out"
}

# ---- common ROS environment (stack side, reset, the /clock check) -------------------------
ros_env() {
  # ROS setup scripts are not set -u/-e clean; relax only around the sources.
  set +e +u
  local extra="${F1TENTH_EXTRA_SETUP:-$HOME/dev_ws/jc_env.sh}"
  if [ -f "$extra" ]; then
    # shellcheck disable=SC1090
    source "$extra"
    log "sourced host setup $extra"
  fi
  [ -f "$REPO/install/setup.bash" ] || die "no $REPO/install: build the workspace first (colcon build)"
  export ROS_DOMAIN_ID="$DOMAIN"
  export F1TENTH_DISCOVERY_SERVER="$DS"
  # shellcheck disable=SC1091
  source "$REPO/scripts/env/jazzy.sh"
  set -e
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DISCOVERY_SERVER="$DS"
  unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_LOCALHOST_ONLY ROS_STATIC_PEERS
  if [ "$NET" = eth ]; then
    FASTRTPS_DEFAULT_PROFILES_FILE="$(render_eth_profile true)"
  else
    FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_SRC/local.xml"
  fi
  export FASTRTPS_DEFAULT_PROFILES_FILE
}

DS_PID=""; ISAAC_STARTED=false; STACK_PID=""
descendants() {  # <pid> -> all descendant pids, deepest first
  local c
  for c in $(pgrep -P "$1"); do descendants "$c"; echo "$c"; done
}
cleanup() {
  local rc=$?
  trap - EXIT
  trap '' INT TERM   # a second Ctrl+C or the caller's SIGTERM must not cut the shutdown short
  if [ -n "$STACK_PID" ] && kill -0 "$STACK_PID" 2>/dev/null; then
    # SIGINT = Ctrl+C for ros2 launch: it stops the supervisor, which stops its components.
    log "stopping the stack (SIGINT to the launch, up to 30 s)"
    kill -INT "$STACK_PID" 2>/dev/null || true
    for _ in $(seq 1 60); do kill -0 "$STACK_PID" 2>/dev/null || break; sleep 0.5; done
    if kill -0 "$STACK_PID" 2>/dev/null; then
      log "the launch is still running: SIGTERM (then use scripts/kill_ros2.py -y for leftovers)"
      kill -TERM "$STACK_PID" 2>/dev/null || true
    fi
  fi
  if $ISAAC_STARTED && docker ps -q -f "name=^${ISAAC_CONTAINER}$" | grep -q .; then
    log "stopping Isaac Sim (docker stop $ISAAC_CONTAINER)"
    docker stop -t 40 "$ISAAC_CONTAINER" >/dev/null 2>&1 || true
  fi
  if [ -n "$DS_PID" ] && kill -0 "$DS_PID" 2>/dev/null; then
    log "stopping the Discovery Server this script started (pid $DS_PID)"
    # ensure_discovery_server.py execs `fastdds` (sh -> fastdds.py -> fast-discovery-server):
    # signal the whole tree, leaves first.
    local tree
    tree="$(descendants "$DS_PID") $DS_PID"
    # shellcheck disable=SC2086  # word splitting of the pid list is intended
    kill -INT $tree 2>/dev/null || true
    sleep 1
    # shellcheck disable=SC2086
    kill -TERM $tree 2>/dev/null || true   # survivors only; the dead ones just fail
  fi
  rm -rf "$TMPD"
  exit "$rc"
}

ensure_discovery_server() {
  # The supervisor starts the same idempotent server later (ensure_discovery_server.py reuses a
  # live one), but the /clock check needs it before the stack exists: start it here.
  [ -n "$DS_PID" ] && return 0   # 'all' started it already
  local script
  script="$(ros2 pkg prefix f1tenth_bringup)/share/f1tenth_bringup/scripts/ensure_discovery_server.py"
  if $DRY; then echo "  python3 $script $DS_ADDR $DS_PORT &"; return; fi
  python3 "$script" "$DS_ADDR" "$DS_PORT" > "$TMPD/discovery_server.log" 2>&1 &
  DS_PID=$!
  sleep 1
  kill -0 "$DS_PID" 2>/dev/null || die "Discovery Server did not start: $(cat "$TMPD/discovery_server.log")"
  log "Discovery Server $DS (pid $DS_PID)"
}

wait_for_clock() {  # <timeout_s>
  local t="$1"
  if $DRY; then echo "  timeout --foreground $t ros2 topic echo --once --no-daemon /clock rosgraph_msgs/msg/Clock"; return; fi
  log "waiting up to ${t}s for /clock (domain $ROS_DOMAIN_ID, Discovery Server $DS) ..."
  # --foreground: Ctrl+C reaches the echo (timeout otherwise makes its own process group and
  # bash defers the INT trap until it returns); --no-daemon: no ros2 daemon outlives the script.
  if timeout --foreground "$t" ros2 topic echo --once --no-daemon /clock rosgraph_msgs/msg/Clock \
      > "$TMPD/clock.txt" 2>&1; then
    log "/clock received: $(grep -E 'sec:' "$TMPD/clock.txt" | head -1 | tr -s ' ')"
  else
    die "no /clock within ${t}s: the Isaac sim is not reachable. Start it ('$0 isaac --net $NET'), \
check that both sides use --net $NET and ROS_DOMAIN_ID $DOMAIN (and, net=eth, the same \
F1TENTH_SIM_DISCOVERY_IP=$DS_ADDR)."
  fi
}

# ---- isaac -------------------------------------------------------------------------------
isaac_precheck() {
  command -v docker >/dev/null || die "docker not found"
  docker info >/dev/null 2>&1 || die "cannot talk to the docker daemon (is it running, are you in group docker?)"
  docker info --format '{{json .Runtimes}}' | grep -q nvidia || die "docker has no 'nvidia' runtime (NVIDIA Container Toolkit)"
  docker image inspect "$ISAAC_IMAGE" >/dev/null 2>&1 \
    || die "image $ISAAC_IMAGE not present. It is never pulled by this script (~20 GB); build it in $ISAAC_WS/.devcontainer"
  [ -x "$ISAAC_WS/launch.sh" ] || die "$ISAAC_WS/launch.sh not found (ISAAC_WS=$ISAAC_WS)"
  local free_gb
  free_gb="$(df -BG --output=avail / | tail -1 | tr -dc 0-9)"
  [ "$free_gb" -ge "$MIN_FREE_DISK_GB" ] || die "only ${free_gb} GB free on /, need >= ${MIN_FREE_DISK_GB} GB (shader/asset caches)"
  log "disk: ${free_gb} GB free on /"
  if command -v nvidia-smi >/dev/null; then
    log "GPU: $(nvidia-smi --query-gpu=name,memory.used,memory.total --format=csv,noheader | head -1) (used, total)"
  fi
  if docker ps -q -f "name=^${ISAAC_CONTAINER}$" | grep -q .; then
    die "container $ISAAC_CONTAINER is already running (docker stop $ISAAC_CONTAINER)"
  fi
}

isaac_cmd() {  # fills ISAAC_ENV and ISAAC_ARGS
  local profile
  if [ "$NET" = eth ]; then profile="$(render_eth_profile false)"; else profile="$DDS_SRC/isaac_local.xml"; fi
  ISAAC_ENV=(ROS_DOMAIN_ID="$DOMAIN" ROS_DISCOVERY_SERVER="$DS" ROS_AUTOMATIC_DISCOVERY_RANGE= ISAAC_CONTAINER_NAME="$ISAAC_CONTAINER")
  ISAAC_ARGS=("$ISAAC_WS/launch.sh" --ros-distro jazzy --dds-profile "$profile" --scene "$SCENE" --rtf-log
    # The stack publishes every static sensor frame itself (as on the car): Isaac must not.
    --set ros_interface.publish_static_tf=false
    # Render the ZED where the stack's TF says it is (the deployed car TF, bag 2026-10-15:
    # lens midpoint (0.2294, 0, 0.2196), pitch -2.083 deg). Isaac's own default puts the lens
    # at the tape's 0.255 m, the camera's FRONT FACE per camera.launch.py: 2.6 cm ahead.
    --set "car_spec.sensors.zed.lens_center_xyz=[0.2293615, 0.0, 0.2196267]"
    --set car_spec.sensors.zed.pitch=-0.03635
    # /odom covariances of the car's vesc_to_odom (vesc.yaml): the EKF weighs odom vs IMU as on the car.
    --set sensors.odometry.covariance.pose_xy=0.2
    --set sensors.odometry.covariance.pose_yaw=0.03
    --set sensors.odometry.covariance.twist_vx=0.0
    # Gyro noise of the car's VESC IMU (vesc.yaml gyro_variance_z 1.746756e-06 rad^2/s^2): the
    # EKF fuses only vyaw, weighted by this variance, which Isaac publishes as stddev^2.
    --set sensors.imu.noise.gyro_stddev=0.0013217)
  [ -n "$LAYOUT" ] && ISAAC_ARGS+=(--set "scene.layout=$LAYOUT")
  $GUI && ISAAC_ARGS+=(--gui)
  return 0
}

run_isaac() {  # until Isaac exits or Ctrl+C (cleanup() stops the container)
  isaac_precheck
  isaac_cmd
  echo "== Isaac Sim (scene $SCENE, net $NET, domain $DOMAIN, Discovery Server $DS) =="
  echo "env: ${ISAAC_ENV[*]}"
  echo "cmd: ${ISAAC_ARGS[*]}"
  $DRY && { echo "(dry run)"; return 0; }
  ISAAC_STARTED=true
  # In the background + wait, so a signal reaches cleanup() at once (bash runs traps only
  # after a foreground child returns).
  env "${ISAAC_ENV[@]}" "${ISAAC_ARGS[@]}" < /dev/null &
  wait $! || true
}

# ---- stack -------------------------------------------------------------------------------
stack_cmd() {  # fills STACK_ARGS
  local share overlay
  share="$(ros2 pkg prefix f1tenth_bringup)/share/f1tenth_bringup"
  STACK_ARGS=(ros2 launch f1tenth_bringup supervisor_bringup.launch.py sim:=isaac use_sim_time:=true
    discovery_server_address:="$DS_ADDR" discovery_server_port:="$DS_PORT")
  overlay="${F1T_SIM_HOST_OVERLAY:-}"
  if [ -z "$overlay" ] && [ "$(nproc)" -lt 12 ]; then
    overlay="$share/config/sim/hosts/linus.yaml"
    log "$(nproc) CPUs (< 12, the Orin's pinning): host overlay $overlay"
  fi
  [ -n "$overlay" ] && [ "$overlay" != none ] && STACK_ARGS+=(sim_overlay_extra:="$overlay")
  local llama asked="" a
  for a in "${LAUNCH_ARGS[@]}"; do
    case "$a" in enable_intelligence:=*) asked="${a#*:=}" ;; esac
  done
  llama="$(python3 -c "from f1tenth_params.param_defaults import get_value; print(get_value('llama_server_path'))" 2>/dev/null || true)"
  if [ -n "$llama" ] && [ ! -x "${llama/#\~/$HOME}" ]; then
    case "$asked" in
      true|True|1) die "enable_intelligence:=true but llama-server is not on this host ($llama, stack_params llama_server_path)" ;;
      "") log "llama-server not found ($llama): enable_intelligence:=false (the LLM planner is not a mission dependency)"
          STACK_ARGS+=(enable_intelligence:=false) ;;
    esac
  elif [ -n "$llama" ]; then
    log "llama-server: $llama (intelligence component: ${asked:-stack_params default})"
  fi
  STACK_ARGS+=("${LAUNCH_ARGS[@]}")
}

run_stack() {  # <clock wait seconds>
  ros_env
  echo "== stack (sim:=isaac, net $NET, domain $ROS_DOMAIN_ID, Discovery Server $DS) =="
  echo "FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE"
  if $CLEAN && ! $DRY; then python3 "$REPO/scripts/kill_ros2.py" -y; fi
  ensure_discovery_server
  wait_for_clock "$1"
  stack_cmd
  echo "cmd: ${STACK_ARGS[*]}"
  $DRY && { echo "(dry run)"; return 0; }
  # A background job of a script starts with SIGINT ignored (bash, no job control), and
  # ros2 launch then ignores the SIGINT cleanup() sends: restore the default first.
  env --default-signal=INT "${STACK_ARGS[@]}" &
  STACK_PID=$!
  wait "$STACK_PID" || true
}

# ---- main --------------------------------------------------------------------------------
trap cleanup EXIT
trap 'exit 130' INT TERM
case "$CMD" in
  isaac)
    run_isaac ;;
  stack)
    run_stack "$CLOCK_WAIT" ;;
  all)
    ros_env
    isaac_precheck
    $CLEAN && ! $DRY && python3 "$REPO/scripts/kill_ros2.py" -y
    ensure_discovery_server
    isaac_cmd
    ISAAC_LOG="${F1T_SIM_LOG_DIR:-$TMPD}/isaac.log"
    mkdir -p "$(dirname "$ISAAC_LOG")"
    echo "== Isaac Sim (background; log $ISAAC_LOG) =="
    echo "cmd: ${ISAAC_ARGS[*]}"
    if ! $DRY; then
      ISAAC_STARTED=true
      env "${ISAAC_ENV[@]}" "${ISAAC_ARGS[@]}" < /dev/null > "$ISAAC_LOG" 2>&1 &
    fi
    # Isaac needs 20-70 s to the first /clock (shader cache warm); then the stack.
    CLEAN=false
    run_stack 300 ;;
  reset)
    ros_env
    if $DRY; then echo "ros2 service call /sim/reset std_srvs/srv/Trigger"; exit 0; fi
    timeout --foreground 20 ros2 service call /sim/reset std_srvs/srv/Trigger ;;
  shell|run)
    ros_env
    export ROS_SUPER_CLIENT=TRUE
    echo "== ROS env (net $NET, domain $ROS_DOMAIN_ID, Discovery Server $DS, super client) =="
    echo "FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE"
    if $DRY; then echo "(dry run) ${RUN_CMD[*]:-bash -i}"; exit 0; fi
    if [ "$CMD" = run ]; then
      echo "cmd: ${RUN_CMD[*]}"
      rc=0; "${RUN_CMD[@]}" || rc=$?
      exit "$rc"
    fi
    ros2 daemon stop >/dev/null 2>&1 || true
    cat > "$TMPD/shellrc" <<RC
PS1='[sim $NET d$ROS_DOMAIN_ID] \\w\\$ '
echo "sim shell: ROS_DOMAIN_ID=\$ROS_DOMAIN_ID ROS_DISCOVERY_SERVER=\$ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT=\$ROS_SUPER_CLIENT (exit to leave)"
RC
    # Not exec: the rendered DDS profile lives in $TMPD, removed by cleanup() when the shell exits.
    bash --rcfile "$TMPD/shellrc" -i || true ;;
esac
