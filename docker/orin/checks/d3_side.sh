#!/usr/bin/env bash
# Run one D3 harness step on ONE side of the Orin, the same way on both.
#
# Runs on: the Orin host (always). The native side runs the command on the
# host with the native Humble environment; the container side runs it with
# `docker exec` inside f1tenth_car started in the harness role
# (`docker_orin.sh up --harness`, docker/orin/compose.harness.yaml). Before
# each run it records the host's context (the running image's ID, nvpmodel,
# clocks: P9 "every result records image ID, nvpmodel and jetson_clocks
# state") as OUT_DIR/context_host.json, and the command line as
# OUT_DIR/d3_side.txt. Native and container never run together: the native
# side refuses while f1tenth_car runs, the container side refuses unless it
# runs in the harness role and no native ROS stack process runs on the host.
#
#   d3_side.sh native    OUT_DIR -- COMMAND [ARG ...]
#   d3_side.sh container OUT_DIR -- COMMAND [ARG ...]
#
# COMMAND is a path relative to this checkout (scripts/jazzy_parity/...) or
# any command; ARGs may name OUT_DIR. Examples (both sides, same arguments):
#   R=$PWD/output/orin_container/d3/$(date +%Y%m%dT%H%M)
#   export FEED=live LIVE_VESC_OK=1     # or: export FEED=bag BAG=$R/inputs/bringup_input
#   d3_side.sh native    "$R/native" -- scripts/jazzy_parity/phase5_bringup.sh hold "$R/native/hold"
#   d3_side.sh container "$R/cont"   -- scripts/jazzy_parity/phase5_bringup.sh hold "$R/cont/hold"
# phase5_bringup.sh writes into its own OUT_DIR, which must be new: give
# d3_side.sh the PARENT for those.
#
# What the harness container sees of this checkout (compose.harness.yaml):
# scripts/, tools/ and docker/orin/checks/ read-only and output/ read-write,
# each at its host path; nothing else of the host (the container's /tmp is its
# own). So on the container side OUT_DIR, and the path knobs BAG and
# MPC_PARAMS, must lie under <checkout>/output (D3_RW_ROOT): d3_side.sh
# refuses anything else (the D3 results root and the input bags therefore live
# under output/orin_container/d3/<ts>/, the same path on both sides). Other
# absolute host paths among the arguments that the container cannot see get
# a warning.
#
# Environment:
#   NATIVE_ENV   the native ctl env (default ~/dev_ws/f1tenth_more/src/f1tenth_ctl/
#                config/ctl.env): sourced with set -a on the native side, and
#                exported as CTL_ENV so phase5_bringup.sh starts the stack as
#                the native f1tenth_ctl does (ROS_SUPER_CLIENT=TRUE etc.)
#   F1T_CONTAINER_ENV  the same inside the image (default: the container's own
#                F1T_CTL_ENV, set by compose.yaml, else the installed
#                share/f1tenth_ctl/config/ctl_container.env)
#   D3_PATH_MAP  HOST_PREFIX:CONTAINER_PREFIX when the harness mounts the
#                checkout elsewhere than its host path; the mount-set check
#                above is then skipped (the caller vouches for the paths)
#   D3_RW_ROOT   the container-writable root (default <checkout>/output; the
#                tests point it elsewhere)
#   The harness knobs below are passed into the container when set (the
#   native side inherits them anyway), so both sides run with the same ones:
#     FEED BAG STACK_ARGS COMPONENTS_ARGS HOLD_SEC HOLD_LOOPS WARMUP_S RATE_S
#     SETTLE_S STABLE_S RUN_GAP KEY_TOPICS LIVE_MISSION LIVE_VESC_OK
#     HARNESS_DOMAIN_ID DS_PORT ALLOW_RUNNING_DS STACK_SUPER_CLIENT CLOCK_HZ
#     LAYER_ARGS MPC_CPU_AFFINITY MPC_PARAMS PLAY_DELAY PLAY_RATE PLAY_TOPICS
#   D3_PASS      more variable names to pass (space-separated). The linus-only
#                CPU_REMAP / PARITY_IGNORE are never passed.
set -uo pipefail
case "${1:-}" in -h|--help|'') sed -n '2,/^set -uo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
                   [ -n "${1:-}" ]; exit $? ;; esac
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=lib_checks.sh
. "$HERE/lib_checks.sh"
SIDE=$1
[ $# -ge 4 ] && [ "$3" = -- ] || { echo "usage: d3_side.sh native|container OUT_DIR -- COMMAND [ARG ...]" >&2; exit 2; }
OUT_ARG=$2
shift 3
NATIVE_ENV=${NATIVE_ENV:-$HOME/dev_ws/f1tenth_more/src/f1tenth_ctl/config/ctl.env}
D3_RW_ROOT=$(readlink -m "${D3_RW_ROOT:-$REPO/output}")
PASS_VARS="FEED BAG STACK_ARGS COMPONENTS_ARGS HOLD_SEC HOLD_LOOPS WARMUP_S RATE_S
SETTLE_S STABLE_S RUN_GAP KEY_TOPICS LIVE_MISSION LIVE_VESC_OK HARNESS_DOMAIN_ID DS_PORT
ALLOW_RUNNING_DS STACK_SUPER_CLIENT CLOCK_HZ LAYER_ARGS MPC_CPU_AFFINITY MPC_PARAMS
PLAY_DELAY PLAY_RATE PLAY_TOPICS ${D3_PASS:-}"
cmd=("$@")
[ -f "$REPO/${cmd[0]}" ] && cmd[0]="$REPO/${cmd[0]}"
case "${cmd[0]}" in *.sh) cmd=(bash "${cmd[@]}") ;; *.py) cmd=(python3 "${cmd[@]}") ;; esac

native_ros_running() {
  # ROS processes of the native install on the host (not in a container: the
  # container's processes run from /opt/f1tenth_ws).
  python3 "$REPO/scripts/jazzy_parity/proc_audit.py" snapshot | grep -v '/opt/f1tenth_ws/' | grep -q .
}

# under_rw PATH: true if PATH (resolved) lies under D3_RW_ROOT.
under_rw() {
  local p
  p=$(readlink -m "$1")
  [ "$p" = "$D3_RW_ROOT" ] || [ "${p#"$D3_RW_ROOT"/}" != "$p" ]
}

# visible_in_container PATH: under one of the harness mounts (or not a host
# path at all: /opt/f1tenth_ws, /dev, ... exist in the image).
visible_in_container() {
  local p m
  p=$(readlink -m "$1")
  for m in "$D3_RW_ROOT" "$REPO/scripts" "$REPO/tools" "$REPO/docker/orin/checks"; do
    [ "$p" = "$m" ] || [ "${p#"$m"/}" != "$p" ] && return 0
  done
  case "$p" in /dev|/dev/*|/proc/*|/opt/f1tenth_ws|/opt/f1tenth_ws/*) return 0 ;; esac
  return 1
}

# The side first, before anything is written.
case "$SIDE" in
  native)
    if [ "$(container_state)" = running ]; then
      echo "refusing: $CONTAINER is running; the native side runs alone (stack_mode.sh native)" >&2
      exit 2
    fi
    [ -r "$NATIVE_ENV" ] || { echo "no native env file $NATIVE_ENV" >&2; exit 2; }
    ;;
  container)
    if [ "$(container_state)" != running ]; then
      echo "refusing: $CONTAINER is not running (docker_orin.sh up --harness)" >&2; exit 2
    fi
    if [ "$(container_role)" != harness ]; then
      echo "refusing: $CONTAINER runs as '$(container_role)', not 'harness'" >&2; exit 2
    fi
    if native_ros_running; then
      echo "refusing: native ROS processes run on the host" >&2; exit 2
    fi
    ;;
  *) echo "side must be native or container" >&2; exit 2 ;;
esac

# Then what the harness container can see (see the header).
if [ "$SIDE" = container ] && [ -z "${D3_PATH_MAP:-}" ]; then
  bad=0
  if ! under_rw "$OUT_ARG"; then
    echo "refusing: OUT_DIR $OUT_ARG is outside $D3_RW_ROOT, the only host directory the" \
         "harness container can write (compose.harness.yaml); use output/orin_container/d3/<ts>/" >&2
    bad=1
  fi
  for v in BAG MPC_PARAMS; do
    if [ -n "${!v:-}" ] && ! under_rw "${!v}"; then
      echo "refusing: $v=${!v} is outside $D3_RW_ROOT, which the harness container cannot see;" \
           "copy the input under output/orin_container/d3/<ts>/inputs/" >&2
      bad=1
    fi
  done
  [ $bad -eq 0 ] || exit 2
  for a in "${cmd[@]:1}"; do
    case "$a" in
      /*) if [ -e "$a" ] && ! visible_in_container "$a"; then
            echo "warning: argument $a exists on the host but not in the harness container" >&2
          fi ;;
    esac
  done
fi

mkdir -p "$OUT_ARG" || exit 2
OUT=$(parity_abs "$OUT_ARG")
host_context "$OUT" host "d3_side=$SIDE" "command=${cmd[*]}"
{
  echo "side=$SIDE time=$(date -Is) checkout=$REPO"
  printf 'command:'; printf ' %q' "${cmd[@]}"; echo
  echo "container: state=$(container_state) role=$(container_role) image=$(container_image_id)"
  for v in $PASS_VARS; do [ -n "${!v+x}" ] && printf '%s=%q\n' "$v" "${!v}"; done
} > "$OUT/d3_side.txt"

case "$SIDE" in
  native)
    echo "env file: $NATIVE_ENV" >> "$OUT/d3_side.txt"
    cd "$REPO" || exit 2
    # shellcheck disable=SC2016  # expanded by the inner bash
    CTL_ENV=${CTL_ENV:-$NATIVE_ENV} bash -c 'set -a; source "$1" || exit 97; set +a; shift
      exec "$@"' _ "$NATIVE_ENV" "${cmd[@]}" 2>&1 | tee "$OUT/d3_side.log"
    exit "${PIPESTATUS[0]}"
    ;;
  container)
    if [ -n "${D3_PATH_MAP:-}" ]; then
      from=${D3_PATH_MAP%%:*}
      to=${D3_PATH_MAP#*:}
      for i in "${!cmd[@]}"; do cmd[i]=${cmd[i]//$from/$to}; done
      REPO_IN=${REPO//$from/$to}
    else
      REPO_IN=$REPO
    fi
    envs=()
    for v in $PASS_VARS; do
      case "$v" in CPU_REMAP|PARITY_IGNORE|CTL_ENV) continue ;; esac
      if [ -n "${!v+x}" ]; then
        val=${!v}
        [ -n "${D3_PATH_MAP:-}" ] && val=${val//$from/$to}
        envs+=(-e "$v=$val")
      fi
    done
    # The inner bash prints the env file it chose (the first output line).
    # shellcheck disable=SC2016  # expanded inside the container
    "$DOCKER" exec "${envs[@]}" -w "$REPO_IN" "$CONTAINER" bash -c "set +u; $CONTAINER_ENV_PICK"'
      echo "[d3_side] env file: $envf"
      set -a; source "$envf" || exit 97; set +a
      export CTL_ENV=${CTL_ENV:-$envf}
      exec "$@"' _ "$F1T_CONTAINER_ENV" "$F1T_CTL_ENV_INSTALLED" "${cmd[@]}" < /dev/null 2>&1 \
      | tee "$OUT/d3_side.log"
    exit "${PIPESTATUS[0]}"
    ;;
  *) echo "side must be native or container" >&2; exit 2 ;;
esac
