# shellcheck shell=bash
# Shared by docker/orin/checks/*.sh (the Orin host side of the D2 / D3
# checks). Sourced, never run. Also sources scripts/jazzy_parity/lib_parity.sh
# (process groups, need_empty_dir, port checks, run context).
#
# The names below are the interface contracts of design.md; each can be
# overridden from the environment (the checks are written before W2's
# scripts exist, and the tests run them against fakes):
#   F1T_CONTAINER       the container (f1tenth_car)
#   DOCKER_ORIN         scripts/docker_orin.sh
#   STACK_MODE          docker/orin/host/stack_mode.sh
#   ORIN_STACK_GUARD    scripts/orin_stack_guard.sh
#   F1T_STATE_ROOT      /scratch/fabiocar/f1tenth_container (host state root)
#   F1T_CONTAINER_ENV   the ctl env inside the image (default, chosen inside the
#                       container: its F1T_CTL_ENV, which compose.yaml sets,
#                       else the installed share/f1tenth_ctl/config/ctl_container.env)
#   F1T_HEALTHCHECK     healthcheck.sh inside the image (the Dockerfile copies
#                       docker/orin/container/ to the same path under
#                       /opt/f1tenth_ws; its HEALTHCHECK runs it from there)
#   DOCKER              the docker CLI (tests put a fake one here)
CHECKS_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$CHECKS_DIR/../../.." && pwd)
# shellcheck source=../../../scripts/jazzy_parity/lib_parity.sh
. "$REPO/scripts/jazzy_parity/lib_parity.sh"
CONTAINER=${F1T_CONTAINER:-f1tenth_car}
DOCKER=${DOCKER:-docker}
DOCKER_ORIN=${DOCKER_ORIN:-$REPO/scripts/docker_orin.sh}
STACK_MODE=${STACK_MODE:-$REPO/docker/orin/host/stack_mode.sh}
ORIN_STACK_GUARD=${ORIN_STACK_GUARD:-$REPO/scripts/orin_stack_guard.sh}
F1T_STATE_ROOT=${F1T_STATE_ROOT:-/scratch/fabiocar/f1tenth_container}
# Empty = resolved inside the container (see CONTAINER_ENV_PICK below).
F1T_CONTAINER_ENV=${F1T_CONTAINER_ENV:-}
F1T_CTL_ENV_INSTALLED=/opt/f1tenth_ws/install/share/f1tenth_ctl/config/ctl_container.env
# The image copy of docker/orin/container/healthcheck.sh (design.md: the repo
# is /opt/f1tenth_ws in the image).
F1T_HEALTHCHECK=${F1T_HEALTHCHECK:-/opt/f1tenth_ws/docker/orin/container/healthcheck.sh}
# The stack's ports (design.md): Discovery Server, gateway, llama-server,
# foxglove_bridge.
STACK_UDP_PORTS=${STACK_UDP_PORTS:-11811}
STACK_TCP_PORTS=${STACK_TCP_PORTS:-8770 8083 8765}
# Where Fast DDS keeps its shared-memory files (tests point it elsewhere).
SHM_DIR=${SHM_DIR:-/dev/shm}

# Every inspect says --type container: an image, volume or network named
# f1tenth_car must never answer for the container.
# container_state: running | exited | ... | "" (no such container)
container_state() {
  "$DOCKER" inspect --type container -f '{{.State.Status}}' "$CONTAINER" 2>/dev/null
}
# container_health: healthy | unhealthy | starting | none | ""
container_health() {
  "$DOCKER" inspect --type container \
    -f '{{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}}' "$CONTAINER" 2>/dev/null
}
# container_role: F1T_CONTAINER_ROLE of the running container (car | harness)
container_role() {
  "$DOCKER" inspect --type container -f '{{range .Config.Env}}{{println .}}{{end}}' \
    "$CONTAINER" 2>/dev/null | sed -n 's/^F1T_CONTAINER_ROLE=//p'
}
container_image_id() {
  "$DOCKER" inspect --type container -f '{{.Image}}' "$CONTAINER" 2>/dev/null
}

# The ctl env file, picked INSIDE the container by an inner bash called with
# "$F1T_CONTAINER_ENV" "$F1T_CTL_ENV_INSTALLED" as $1 $2: the explicit file
# when given, else the container's own F1T_CTL_ENV (the contract variable
# compose.yaml sets), else the installed copy. Sets envf and shifts both.
# shellcheck disable=SC2016  # expanded inside the container
CONTAINER_ENV_PICK='envf=${1:-${F1T_CTL_ENV:-$2}}; shift 2'

# cexec CMD [ARG...]: run CMD inside the container as the image's user, with
# the stack's environment (the ctl env above sourced with set -a; the image's
# jazzy.sh if that file is missing). stdin is closed.
cexec() {
  # shellcheck disable=SC2016  # expanded inside the container
  "$DOCKER" exec "$CONTAINER" bash -c "set +u; $CONTAINER_ENV_PICK"'
    if [ -r "$envf" ]; then set -a; source "$envf"; set +a
    else source /opt/f1tenth_ws/scripts/env/jazzy.sh; fi
    exec "$@"' _ "$F1T_CONTAINER_ENV" "$F1T_CTL_ENV_INSTALLED" "$@" < /dev/null
}

# record OUT_DIR NAME PASS|FAIL|WARN|INFO DETAIL...: one verdict line, printed
# and appended to OUT_DIR/results.txt.
record() {
  local out=$1 name=$2 verdict=$3
  shift 3
  printf '%s %s: %s\n' "$verdict" "$name" "$*" | tee -a "$out/results.txt"
}

# overall OUT_DIR: exit status 0 if results.txt has no FAIL line.
overall() {
  if grep -q '^FAIL ' "$1/results.txt" 2>/dev/null; then
    echo "RESULT FAIL ($(grep -c '^FAIL ' "$1/results.txt") failing, see $1/results.txt)"
    return 1
  fi
  echo "RESULT PASS ($1/results.txt)"
  return 0
}

# host_context OUT_DIR LABEL [KEY=VALUE ...]: run_context.py on the host (image
# ID of the running container, nvpmodel, clocks) as OUT_DIR/context_LABEL.json.
host_context() {
  local out=$1 label=$2
  shift 2
  python3 "$REPO/scripts/jazzy_parity/run_context.py" --out "$out/context_$label.json" \
    --extra "context=$label" "$@" > /dev/null 2>&1 || echo "run_context.py failed" >&2
  return 0
}
