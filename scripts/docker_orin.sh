#!/bin/bash
# The car's Jazzy container on the Orin (plan P6.3): compose project
# f1tenth_car, container f1tenth_car, image pinned in ~/.config/f1tenth/car_image.
#
#   docker_orin.sh pin IMAGE          record the image (tag + ID) that `up` runs
#   docker_orin.sh up [--harness] [--archive separate|shared] [--allow-drift]
#                     [--autostart none|stack] [--dry-run]
#   docker_orin.sh down [--dry-run]   graceful stop, then a leftovers report
#   docker_orin.sh restart [up options]
#   docker_orin.sh status             pin, container, health, which side runs
#   docker_orin.sh shell [CMD ...]    a shell (or CMD) in the container, ROS sourced
#   docker_orin.sh exec CMD ...       CMD in the container, no tty; its exit code
#   docker_orin.sh logs [-f] [-n N]   the container's log (the main process)
#   docker_orin.sh engine [--force] [--dry-run]
#                                     build the YOLO TensorRT engine into the
#                                     f1tenth_car_trt volume (stack stopped)
#   docker_orin.sh drift              drift check: native tree vs the pinned image's CAR_SYNC
#   docker_orin.sh report             D2 report: labels, health, docker diff, state ownership
#   docker_orin.sh history [-n N]     the up/down/pin/engine history
#   docker_orin.sh wait-host [SECONDS]
#                                     wait (F1T_WAIT_HOST_S, 120 s) until dockerd
#                                     answers and /run/jtop.sock is a socket; the
#                                     user unit's ExecStartPre (at boot the user
#                                     manager cannot order after system units)
#
# `up` refuses (exit 1, says why) unless: the pinned tag still has the pinned
# ID; the host's group IDs (video render dialout input zed jtop) equal the
# image's f1tenth.gid.* labels; /proc/devices has a hidraw major and the fixed
# majors compose grants (ttyACM 166, ttyUSB 188, usb_device 189,
# video4linux 81, input 13) are not different; / has >= 5 GB free and the
# state disk >= 50 GB; the state directories exist and belong to this user;
# the read-only inputs exist (GGUF models, gateway token, PWA www, /run/jtop.sock);
# scripts/orin_stack_guard.sh --for jazzy is clear; and
# scripts/orin/drift_check.sh finds the native tree equal to the image's
# CAR_SYNC.json (--allow-drift overrides, logged). Then it waits for the
# container's healthcheck (F1T_UP_TIMEOUT_S, 150 s); a container that exits
# or stays unhealthy is stopped again (nothing half-up) and `up` fails.
#
# --archive separate (default): runs go to $F1T_STATE/archive; shared: the
# native ~/f1tenth_archive (synced by f1tenth-archive.service). --harness: the
# D3 harness override (compose.harness.yaml: sleeps, mounts this worktree's
# scripts/ tools/ docker/orin/checks ro and output/ rw, DDS domain 88).
#
# Env: F1T_STATE (/scratch/fabiocar/f1tenth_container), F1T_ARCHIVE_MODE and
# F1T_ALLOW_DRIFT (defaults of --archive and --allow-drift; the user unit's
# EnvironmentFile sets them), F1T_NATIVE_WS (~/dev_ws/f1tenth_more),
# F1T_CAR_IMAGE_FILE (~/.config/f1tenth/car_image), F1T_UP_TIMEOUT_S; for tests
# off the car: F1T_GUARD, F1T_DRIFT_CHECK (the two tools), F1T_PROC_DEVICES,
# F1T_JTOP_SOCK (also compose's source), F1T_MIN_FREE_GB_ROOT (5),
# F1T_MIN_FREE_GB_STATE (50).
# Exit: 0 ok, 1 refused or failed, 2 usage, 3 `down` left something behind
# (listed) or `status`: not running, 4 `status`: running but not healthy.
set -euo pipefail
SELF=$(readlink -f "${BASH_SOURCE[0]}")
REPO=$(cd "$(dirname "$SELF")/.." && pwd)
DOCKER_DIR=$REPO/docker/orin
PROJECT=f1tenth_car
CTR=f1tenth_car
STATE=${F1T_STATE:-/scratch/fabiocar/f1tenth_container}
PIN_FILE=${F1T_CAR_IMAGE_FILE:-$HOME/.config/f1tenth/car_image}
NATIVE_WS=${F1T_NATIVE_WS:-$HOME/dev_ws/f1tenth_more}
HISTORY=$STATE/history.log
LOGDIR=$REPO/output/orin_container/logs
GUARD=${F1T_GUARD:-$REPO/scripts/orin_stack_guard.sh}
DRIFT_CHECK=${F1T_DRIFT_CHECK:-$REPO/scripts/orin/drift_check.sh}
PROC_DEVICES=${F1T_PROC_DEVICES:-/proc/devices}
JTOP_SOCK=${F1T_JTOP_SOCK:-/run/jtop.sock}
PROG=$(basename "$0")
# State directories (compose.yaml mounts each one; install.sh creates them).
STATE_DIRS=(ros archive ctl gateway archive_trigger zed_settings hf_cache
            missions/mission_reports missions/mission_videos missions/llm_generated missions/corridors_jsons
            run logs)

usage() { sed -n '2,/^set -euo/p' "$SELF" | sed '$d; s/^# \{0,1\}//'; }
die() { echo "$PROG: $*" >&2; exit 1; }
usage_err() { echo "$PROG: $*" >&2; exit 2; }
note() { echo "$PROG: $*"; }
history_line() {
  mkdir -p "$STATE" 2>/dev/null || true
  printf '%s %s\n' "$(date -Iseconds)" "$*" >> "$HISTORY" 2>/dev/null || true
}
# The container's state (created running paused restarting removing exited
# dead) or "absent", from `docker ps -a`'s exit code and an exact name match,
# never from Docker's message texts: Docker 29 changed them, and its failed
# `inspect -f` prints an empty line before the error. Docker itself not
# answering (daemon down, no permission on its socket) is an error, not
# "absent": it says why on stderr and returns 1, so callers write
# `st=$(ctr_state) || exit 1` (or decide otherwise).
ctr_state() {
  local out
  if ! out=$(docker ps -a --filter "name=$CTR" --format '{{.Names}} {{.State}}' 2>&1); then
    echo "$PROG: cannot ask Docker for $CTR's state: $(printf '%s\n' "$out" | grep -v '^[[:space:]]*$' | tail -1)" >&2
    return 1
  fi
  printf '%s\n' "$out" | awk -v n="$CTR" '$1 == n {s = $2} END {print (s == "" ? "absent" : s)}'
}
ctr_health() {
  local h
  h=$(docker inspect --type container -f '{{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}}' "$CTR" 2>/dev/null) || h=none
  echo "${h:-none}"
}
ctr_role() {
  { docker inspect --type container -f '{{range .Config.Env}}{{println .}}{{end}}' "$CTR" 2>/dev/null || true; } \
    | sed -n 's/^F1T_CONTAINER_ROLE=//p' | head -1
}
free_gb() { { df -P -BG "$1" 2>/dev/null || true; } | awk 'NR==2 {sub("G", "", $4); print $4}'; }

# ---- the pinned image ------------------------------------------------------------
pin_get() { [ -f "$PIN_FILE" ] || return 0; awk -v k="$1" '$1 == k {print $2; exit}' "$PIN_FILE"; }

# Sets IMAGE and IMAGE_ID from the pin file; refuses when the tag moved.
resolve_image() {
  [ -f "$PIN_FILE" ] || die "no pinned image ($PIN_FILE): $PROG pin <image>"
  IMAGE=$(pin_get tag)
  IMAGE_ID=$(pin_get id)
  if [ -z "$IMAGE" ] || [ -z "$IMAGE_ID" ]; then die "$PIN_FILE has no 'tag' and 'id' lines: $PROG pin <image>"; fi
  local now
  now=$(docker image inspect -f '{{.Id}}' "$IMAGE" 2>/dev/null) || die "pinned image $IMAGE is not present (docker image ls)"
  [ "$now" = "$IMAGE_ID" ] || die "the tag $IMAGE now has ID $now, the pin says $IMAGE_ID (rebuilt?): check it, then $PROG pin $IMAGE"
}

label() { docker image inspect -f "{{index .Config.Labels \"$1\"}}" "$IMAGE" 2>/dev/null || true; }

cmd_pin() {
  [ $# -eq 1 ] || { echo "usage: $PROG pin IMAGE" >&2; exit 2; }
  local ref=$1 id car
  id=$(docker image inspect -f '{{.Id}}' "$ref" 2>/dev/null) || die "no image $ref"
  IMAGE=$ref
  car=$(label f1tenth.car_sync.sha256)
  if [ -z "$car" ] || [ "$car" = unknown ]; then die "$ref has no f1tenth.car_sync.sha256 label: not built by docker/orin/build.sh"; fi
  mkdir -p "$(dirname "$PIN_FILE")"
  printf 'tag %s\nid %s\npinned %s\nrevision %s\n' "$ref" "$id" "$(date -Iseconds)" \
    "$(label org.opencontainers.image.revision)" > "$PIN_FILE.tmp"
  mv -f "$PIN_FILE.tmp" "$PIN_FILE"
  history_line "pin image=$ref id=$id"
  note "pinned $ref ($id) in $PIN_FILE"
}

# ---- preflight ----------------------------------------------------------------------
check_gids() {
  local g host want var
  for g in video render dialout input zed jtop; do
    host=$(getent group "$g" | cut -d: -f3 || true)
    want=$(label "f1tenth.gid.$g")
    [ -n "$host" ] || die "host group $g does not exist"
    [ -n "$want" ] || die "image $IMAGE has no f1tenth.gid.$g label (built without build.sh?)"
    [ "$host" = "$want" ] || die "group $g: host GID $host, image GID $want (rebuild the image on this host)"
    var=GID_$(echo "$g" | tr '[:lower:]' '[:upper:]')
    export "$var=$host"
  done
}

check_majors() {
  local pair name major have
  HIDRAW_MAJOR=$(awk '$2 == "hidraw" {print $1; exit}' "$PROC_DEVICES")
  [ -n "$HIDRAW_MAJOR" ] || die "no hidraw major in $PROC_DEVICES (ZED IMU access needs it)"
  export HIDRAW_MAJOR
  for pair in ttyACM:166 ttyUSB:188 usb_device:189 video4linux:81 input:13; do
    name=${pair%%:*}; major=${pair#*:}
    have=$(awk -v n="$name" '$2 == n {print $1; exit}' "$PROC_DEVICES")
    [ -z "$have" ] || [ "$have" = "$major" ] || die "$PROC_DEVICES: $name is major $have, compose.yaml grants $major"
  done
}

check_disk() {
  local root state
  root=$(free_gb /); state=$(free_gb "$STATE")
  [ "${root:-0}" -ge "${F1T_MIN_FREE_GB_ROOT:-5}" ] || die "/ has ${root:-?} GB free (< ${F1T_MIN_FREE_GB_ROOT:-5} GB)"
  [ "${state:-0}" -ge "${F1T_MIN_FREE_GB_STATE:-50}" ] || die "$STATE's disk has ${state:-?} GB free (< ${F1T_MIN_FREE_GB_STATE:-50} GB)"
}

check_state() {
  local d
  [ -d "$STATE" ] || die "$STATE missing: run docker/orin/host/install.sh"
  for d in "${STATE_DIRS[@]}"; do
    [ -d "$STATE/$d" ] || die "$STATE/$d missing: run docker/orin/host/install.sh"
    [ -O "$STATE/$d" ] || die "$STATE/$d is not owned by $(id -un)"
  done
  ls "$STATE"/zed_settings/SN*.conf > /dev/null 2>&1 \
    || note "WARN: no ZED calibration in $STATE/zed_settings (install.sh seeds it from /usr/local/zed/settings)"
}

check_inputs() {
  [ -d "$HOME/projects/llm/models" ] || die "no $HOME/projects/llm/models (GGUF models, mounted read-only)"
  [ -f "$HOME/.config/f1tenth_gateway/env" ] || die "no $HOME/.config/f1tenth_gateway/env (the gateway token)"
  [ -d "$HOME/.local/share/f1tenth_gateway/www" ] || die "no $HOME/.local/share/f1tenth_gateway/www (the PWA; install.sh creates an empty one)"
  [ -d "$HOME/.config/f1tenth_car" ] || die "no $HOME/.config/f1tenth_car (install.sh creates it)"
  [ -S "$JTOP_SOCK" ] || die "$JTOP_SOCK is not a socket: the host's jtop service is down (sudo systemctl start jtop)"
  export F1T_JTOP_SOCK=$JTOP_SOCK
}

# The image's CAR_SYNC.json, copied out of a created (never started) container.
car_sync_of_image() {
  local short=${IMAGE_ID#sha256:} out cid
  out=$STATE/run/car_sync_${short:0:12}.json
  if [ ! -s "$out" ]; then
    cid=$(docker create --network none "$IMAGE" true) || die "docker create $IMAGE failed"
    docker cp "$cid:/opt/f1tenth_ws/CAR_SYNC.json" "$out.tmp" > /dev/null || { docker rm -f "$cid" > /dev/null; die "no CAR_SYNC.json in $IMAGE"; }
    docker rm -f "$cid" > /dev/null
    mv -f "$out.tmp" "$out"
  fi
  echo "$out"
}

run_drift() {
  local car_sync rc=0
  car_sync=$(car_sync_of_image) || return 2
  [ -x "$DRIFT_CHECK" ] || { echo "$PROG: $DRIFT_CHECK missing" >&2; return 2; }
  "$DRIFT_CHECK" --car-sync "$car_sync" --ws "$NATIVE_WS" || rc=$?
  return $rc
}

# ---- up --------------------------------------------------------------------------------
compose_files() {
  COMPOSE=(docker compose -p "$PROJECT" -f "$DOCKER_DIR/compose.yaml")
  [ "${HARNESS:-0}" = 1 ] && COMPOSE+=(-f "$DOCKER_DIR/compose.harness.yaml")
  return 0
}

cmd_up() {
  local archive=${F1T_ARCHIVE_MODE:-separate} allow=${F1T_ALLOW_DRIFT:-0} autostart=${F1T_AUTOSTART:-none}
  local dry=0 rc t0 timeout=${F1T_UP_TIMEOUT_S:-150} st hl
  HARNESS=0
  while [ $# -gt 0 ]; do
    case "$1" in
      --harness) HARNESS=1 ;;
      --archive) [ $# -ge 2 ] || usage_err "up: --archive needs a value (separate|shared)"
                 archive=$2; shift ;;
      --archive=*) archive=${1#*=} ;;
      --allow-drift) allow=1 ;;
      --autostart) [ $# -ge 2 ] || usage_err "up: --autostart needs a value (none|stack)"
                   autostart=$2; shift ;;
      --autostart=*) autostart=${1#*=} ;;
      --dry-run) dry=1 ;;
      *) echo "$PROG up: unknown option '$1'" >&2; exit 2 ;;
    esac
    shift
  done
  case "$archive" in
    separate) F1T_ARCHIVE_DIR=$STATE/archive ;;
    shared) F1T_ARCHIVE_DIR=$HOME/f1tenth_archive
            [ -d "$F1T_ARCHIVE_DIR" ] || die "--archive shared: $F1T_ARCHIVE_DIR missing" ;;
    *) echo "$PROG up: --archive separate|shared, got '$archive'" >&2; exit 2 ;;
  esac
  case "$autostart" in none|stack) ;; *) echo "$PROG up: --autostart none|stack, got '$autostart'" >&2; exit 2 ;; esac
  case "$allow" in 1|true|yes) allow=1 ;; *) allow=0 ;; esac
  export F1T_ARCHIVE_DIR F1T_STATE=$STATE F1T_AUTOSTART=$autostart
  [ -d "$STATE/run" ] || die "$STATE/run missing: run docker/orin/host/install.sh"
  exec 9> "$STATE/run/docker_orin.lock"
  flock -n 9 || die "another $PROG up/down is running"

  st=$(ctr_state) || exit 1
  case "$st" in
    running) die "$CTR already runs ($PROG status; $PROG down first)" ;;
    absent) ;;
    *) note "removing the old $st container $CTR"; [ "$dry" = 1 ] || docker rm -f "$CTR" > /dev/null ;;
  esac
  resolve_image
  export F1TENTH_IMAGE=$IMAGE F1TENTH_IMAGE_ID=$IMAGE_ID
  check_gids
  check_majors
  check_state
  check_disk
  check_inputs
  if [ "$HARNESS" = 1 ]; then
    export F1T_REPO=$REPO
    local d
    for d in scripts tools docker/orin/checks output; do
      [ -d "$REPO/$d" ] || die "--harness: $REPO/$d missing"
    done
  fi
  "$GUARD" --for jazzy || die "the guard refuses (above): stop the native side first (stack_mode.sh container does it)"
  rc=0; run_drift || rc=$?
  case "$rc" in
    0) ;;
    3) if [ "$allow" = 1 ]; then
         note "WARN: the native tree drifted from the image's CAR_SYNC (above); --allow-drift given, starting anyway"
         history_line "allow-drift image=$IMAGE"
       else
         die "the native tree has changes the image does not have (above): car-sync and rebuild (plan P1), or --allow-drift"
       fi ;;
    *) if [ "$allow" = 1 ]; then
         note "WARN: the drift check did not run (exit $rc); --allow-drift given, starting anyway"
         history_line "allow-drift (check failed, exit $rc) image=$IMAGE"
       else
         die "the drift check failed (exit $rc); fix it, or --allow-drift"
       fi ;;
  esac
  compose_files
  if [ "$dry" = 1 ]; then
    printf '%s: dry-run: would run:' "$PROG"; printf ' %q' "${COMPOSE[@]}" up -d --no-build --pull never; echo
    note "dry-run: env F1TENTH_IMAGE=$F1TENTH_IMAGE F1T_ARCHIVE_DIR=$F1T_ARCHIVE_DIR F1T_AUTOSTART=$F1T_AUTOSTART HIDRAW_MAJOR=$HIDRAW_MAJOR GID_RENDER=$GID_RENDER GID_INPUT=$GID_INPUT"
    return 0
  fi
  find /dev/shm -mindepth 1 -maxdepth 1 -printf '%f\n' > "$STATE/run/shm_before_up.txt" 2>/dev/null || true
  "${COMPOSE[@]}" up -d --no-build --pull never || { history_line "up FAILED (compose) image=$IMAGE"; die "docker compose up failed"; }
  t0=$(date +%s)
  while :; do
    st=$(ctr_state) || st=unknown
    hl=$(ctr_health)
    [ "$st" = running ] && [ "$hl" = healthy ] && break
    if [ "$st" != running ] || [ $(( $(date +%s) - t0 )) -ge "$timeout" ]; then
      echo "$PROG: $CTR is $st / $hl after $(( $(date +%s) - t0 )) s; last log lines:" >&2
      docker logs --tail 40 "$CTR" >&2 2>&1 || true
      docker inspect --type container -f '{{if .State.Health}}{{range .State.Health.Log}}{{.Output}}{{end}}{{end}}' "$CTR" 2>/dev/null | tail -3 >&2 || true
      history_line "up FAILED ($st/$hl) image=$IMAGE"
      note "stopping it again (nothing stays half-up)"
      cmd_down_inner > /dev/null 2>&1 || true
      exit 1
    fi
    sleep 2
  done
  local cid
  cid=$(docker inspect --type container -f '{{.Id}}' "$CTR")
  history_line "up image=$IMAGE id=$IMAGE_ID container=${cid:0:12} role=$( [ "$HARNESS" = 1 ] && echo harness || echo car) archive=$archive autostart=$autostart allow_drift=$allow"
  note "up: $CTR ${cid:0:12} healthy in $(( $(date +%s) - t0 )) s"
  note "image $IMAGE $IMAGE_ID ($(label org.opencontainers.image.revision | cut -c1-12), car_sync $(label f1tenth.car_sync.native_head | cut -c1-12))"
}

# ---- down --------------------------------------------------------------------------------
# Role car: `docker stop` sends SIGTERM to tini, which forwards it to
# f1tenth_container_main.py; its ordered stop ends the managed processes (whose
# runners are tini's children, not the main's), then any ROS process left, the
# exec-started ones included, and cleans the Fast DDS SHM.
# Role harness: the main process is `sleep infinity`, so the stacks the harness
# started with `docker exec` are stopped first by this script, run inside the
# container (python3 -): SIGINT to every process that is not PID 1 (tini), not
# the main process (and its tree) and not this script itself; after 15 s
# SIGTERM to what is left; anything still running is killed by the container
# stop.
read -r -d '' STOP_EXECED <<'PY' || true
import os, signal, sys, time
me = os.getpid()


def stat(p):
    try:
        with open('/proc/%d/stat' % p) as f:
            s = f.read()
        return int(s[s.rindex(')') + 2:].split()[1])
    except (OSError, ValueError, IndexError):
        return None


def cmdline(p):
    try:
        with open('/proc/%d/cmdline' % p, 'rb') as f:
            return f.read().split(b'\0')[:-1]
    except OSError:
        return []


def unmanaged():
    pids = [int(d) for d in os.listdir('/proc') if d.isdigit()]
    parent = {p: stat(p) for p in pids}
    init = cmdline(1)
    want = init[init.index(b'--') + 2:] if b'--' in init else []
    kids = sorted(p for p, pp in parent.items() if pp == 1)
    main = next((p for p in kids if cmdline(p) == want), kids[0] if kids else None)
    mine = set()
    p = me
    while p not in (None, 0, 1):
        mine.add(p)
        p = parent.get(p, stat(p))
    out = []
    for p in pids:
        if p == 1 or p in mine:
            continue
        q, managed = p, False
        while q not in (None, 0, 1):
            if q == main:
                managed = True
                break
            q = parent.get(q)
        if not managed:
            out.append(p)
    return out


def signal_all(sig, wait):
    victims = unmanaged()
    for p in victims:
        try:
            os.kill(p, sig)
        except OSError:
            pass
    deadline = time.time() + wait
    while time.time() < deadline and unmanaged():
        time.sleep(0.5)
    return victims


first = signal_all(signal.SIGINT, 15.0)
print('exec-started processes signalled (SIGINT): %d %s' % (len(first), [b' '.join(cmdline(p)[:3]).decode(errors='replace') for p in first][:8]))
if unmanaged():
    second = signal_all(signal.SIGTERM, 5.0)
    print('still running after SIGINT, SIGTERM: %d' % len(second))
left = unmanaged()
print('left for the container stop: %d' % len(left))
PY

cmd_down_inner() {
  local st
  st=$(ctr_state) || return 1
  if [ "$st" = running ]; then
    if [ "$(ctr_role)" != car ]; then
      docker exec -i "$CTR" python3 - <<< "$STOP_EXECED" || note "WARN: could not signal the exec-started processes"
    fi
    # SIGTERM to tini, forwarded to the main process (the car's ordered stop);
    # SIGKILL after stop_grace_period's 120 s.
    docker stop -t 120 "$CTR" > /dev/null
  fi
  [ "$st" = absent ] || docker rm "$CTR" > /dev/null
}

leftovers() {
  local found=0 new
  if ! "$GUARD" --for humble --quiet 2> "$STATE/run/down_guard.txt"; then
    found=1
    echo "leftovers (the guard's view):"
    sed 's/^/  /' "$STATE/run/down_guard.txt"
  fi
  if [ -f "$STATE/run/shm_before_up.txt" ]; then
    new=$(comm -13 <(sort "$STATE/run/shm_before_up.txt") <(find /dev/shm -mindepth 1 -maxdepth 1 -printf '%f\n' 2>/dev/null | sort) | grep -E '^(fastrtps_|fast_datasharing|sem\.fastrtps_)' || true)
    if [ -n "$new" ]; then
      echo "leftovers: $(echo "$new" | wc -l) Fast DDS SHM files created since up (stale, not mapped): $(echo "$new" | tr '\n' ' ' | cut -c1-200)"
      found=1
    fi
  fi
  [ "$found" = 0 ] && echo "leftovers: none (no ROS process, UDP 11811 / TCP 8770 8083 8765 free, no live or new Fast DDS SHM)"
  return $found
}

cmd_down() {
  local dry=0 st t0
  case "${1:-}" in
    '') ;;
    --dry-run) dry=1 ;;
    *) usage_err "down: unknown option '$1'" ;;
  esac
  st=$(ctr_state) || exit 1
  if [ "$dry" = 1 ]; then
    note "dry-run: $CTR is $st; would signal exec-started processes, docker stop -t 120 $CTR, docker rm $CTR, then report leftovers"
    return 0
  fi
  mkdir -p "$STATE/run"
  exec 9> "$STATE/run/docker_orin.lock"
  flock -w 150 9 || die "another $PROG up/down holds the lock"
  t0=$(date +%s)
  if [ "$st" = absent ]; then
    note "$CTR does not exist"
  else
    note "stopping $CTR ($st): exec-started processes first, then the main process's ordered stop"
    cmd_down_inner
    note "down in $(( $(date +%s) - t0 )) s"
  fi
  if leftovers; then
    history_line "down container_state=$st seconds=$(( $(date +%s) - t0 )) leftovers=none"
  else
    history_line "down container_state=$st seconds=$(( $(date +%s) - t0 )) leftovers=FOUND"
    return 3
  fi
}

# ---- status / shell / exec / logs -----------------------------------------------------------------
cmd_status() {
  local st hl tag id now
  tag=$(pin_get tag); id=$(pin_get id)
  if [ -n "$tag" ]; then
    # From the exit code: a failed `inspect -f` prints an empty line first.
    now=$(docker image inspect -f '{{.Id}}' "$tag" 2>/dev/null) || now=missing
    if [ "$now" = missing ]; then now="  WARNING: the tag is not present (docker image ls)"
    elif [ "$now" = "$id" ]; then now=""
    else now="  WARNING: the tag is now ${now:7:12}"
    fi
    echo "pinned:    $tag ${id:7:12} (pinned $(pin_get pinned))$now"
  else
    echo "pinned:    none ($PIN_FILE)"
  fi
  st=$(ctr_state) || return 1
  if [ "$st" = absent ]; then
    echo "container: $CTR absent; the native side may run (stack_mode.sh status)"
    return 3
  fi
  hl=$(ctr_health)
  echo "container: $CTR $st, health $hl, role $(ctr_role), image $(docker inspect --type container -f '{{.Config.Image}} {{.Image}}' "$CTR" | cut -c1-80)"
  echo "started:   $(docker inspect --type container -f '{{.State.StartedAt}}' "$CTR")"
  docker inspect --type container -f '{{if .State.Health}}{{range .State.Health.Log}}{{.Output}}{{end}}{{end}}' "$CTR" 2>/dev/null \
    | grep -v '^$' | tail -1 | sed 's/^/health:    /'
  [ "$st" = running ] || return 3
  [ "$hl" = healthy ] || [ "$(ctr_role)" != car ] || return 4
  return 0
}

need_running() {
  local st
  st=$(ctr_state) || exit 1
  [ "$st" = running ] || die "$CTR is not running ($PROG up)"
}

cmd_shell() {
  need_running
  local tty=(-i)
  [ -t 0 ] && [ -t 1 ] && tty=(-it)
  if [ $# -eq 0 ]; then
    exec docker exec "${tty[@]}" "$CTR" bash
  fi
  exec docker exec "${tty[@]}" "$CTR" /opt/f1tenth_ws/entrypoint.sh "$@"
}

cmd_exec() {
  [ $# -gt 0 ] || { echo "usage: $PROG exec CMD [ARGS...]" >&2; exit 2; }
  need_running
  exec docker exec -i "$CTR" /opt/f1tenth_ws/entrypoint.sh "$@"
}

cmd_logs() {
  local args=(--tail 200)
  while [ $# -gt 0 ]; do
    case "$1" in
      -f|--follow) args+=(-f) ;;
      -n) [ $# -ge 2 ] || usage_err "logs: -n needs a number"
          args=(--tail "$2" "${args[@]:2}"); shift ;;
      *) echo "$PROG logs: unknown option '$1'" >&2; exit 2 ;;
    esac
    shift
  done
  docker logs "${args[@]}" "$CTR"
  echo "(f1tenth_ctl's process logs: $STATE/ctl; the gateway's: $STATE/gateway/gateway.log)" >&2
}

# ---- engine -----------------------------------------------------------------------------------------
cmd_engine() {
  local pass=() paths model engine log rc=0
  while [ $# -gt 0 ]; do
    case "$1" in --force|--dry-run) pass+=("$1") ;; *) echo "$PROG engine: unknown option '$1'" >&2; exit 2 ;; esac
    shift
  done
  need_running
  # Where the stack looks for it (the installed stack_params.yaml,
  # set_installed_params.py) and the model it is built from.
  paths=$(docker exec "$CTR" python3 -c '
import yaml
d = yaml.safe_load(open("/opt/f1tenth_ws/install/share/f1tenth_params/config/stack_params.yaml"))
print(d["yolo_model"]["default"], d["yolo_engine_path"]["default"])') || die "cannot read the installed stack_params.yaml"
  read -r model engine <<< "$paths"
  case "$engine" in
    /var/cache/f1tenth/trt/?*) ;;
    *) die "the image's yolo_engine_path ('$engine') is not in the TensorRT volume /var/cache/f1tenth/trt" ;;
  esac
  mkdir -p "$LOGDIR"
  log=$LOGDIR/engine_$(date +%Y%m%d_%H%M%S).log
  note "engine: $model -> $engine (log $log)"
  docker exec "$CTR" mkdir -p "$(dirname "$engine")"
  docker exec -i "$CTR" /opt/f1tenth_ws/entrypoint.sh /opt/f1tenth_ws/scripts/export_yolo_engine.sh \
    --model "/opt/f1tenth_ws/install/share/f1tenth_perception/models/$model" \
    --models-dir "$(dirname "$engine")" "${pass[@]}" 2>&1 | tee "$log" || rc=$?
  [ "$rc" = 0 ] || { history_line "engine FAILED (exit $rc)"; die "export_yolo_engine.sh failed (exit $rc), see $log"; }
  case " ${pass[*]} " in *" --dry-run "*) return 0 ;; esac
  docker exec "$CTR" test -s "$engine" \
    || die "the export wrote no $engine: the engine name and set_installed_params.py's yolo_engine_path disagree (see $log)"
  history_line "engine $engine image=$(docker inspect --type container -f '{{.Config.Image}}' "$CTR")"
  note "engine ready: $engine (used with yolo_backend:=engine)"
}

# ---- drift / report / history -------------------------------------------------------------------
cmd_drift() {
  resolve_image
  mkdir -p "$STATE/run"
  local rc=0
  run_drift || rc=$?
  case "$rc" in
    0) note "no drift: the native tree $NATIVE_WS equals $IMAGE's CAR_SYNC" ;;
    3) note "DRIFT: the native tree has changes the image does not have (above)" ;;
    *) note "the drift check failed (exit $rc)" ;;
  esac
  return $rc
}

cmd_report() {
  local out st
  mkdir -p "$LOGDIR"
  out=$LOGDIR/report_$(date +%Y%m%d_%H%M%S).txt
  {
    echo "== docker_orin.sh report $(date -Iseconds) on $(hostname)"
    cmd_status || true
    st=$(ctr_state) || st=unknown
    if [ "$st" != absent ] && [ "$st" != unknown ]; then
      IMAGE=$(docker inspect --type container -f '{{.Config.Image}}' "$CTR")
      echo "== labels"
      docker image inspect -f '{{range $k, $v := .Config.Labels}}{{$k}}={{$v}}
{{end}}' "$IMAGE" | grep -E '^(f1tenth\.|org\.opencontainers\.image\.(revision|version))' | sed 's/^/  /'
      echo "== docker diff $CTR (expected: /tmp, /run, /var/tmp, the user's ~/.config ~/.cache ~/.local ~/.nv outside the mounts, Ultralytics settings, the ZED SDK's .log; 'runtime' = put there by the nvidia runtime's CSV mounts)"
      # The nvidia runtime (CSV mode) mounts the host's L4T files listed in
      # host-files-for-container.d into the container layer: each such path,
      # its parent directories and the ldconfig it runs belong to the runtime.
      local rt_list
      rt_list=$(mktemp)
      cat /etc/nvidia-container-runtime/host-files-for-container.d/*.csv 2>/dev/null \
        | awk -F', *' 'NF >= 2 && $2 ~ "^/" {print $1, $2}' >"$rt_list"
      docker diff "$CTR" 2>/dev/null | awk -v rl="$rt_list" '
        BEGIN {
          while ((getline l < rl) > 0) {
            split(l, a, " ")
            if (a[2] ~ "^/lib/") a[2] = "/usr" a[2]   # merged /usr on 24.04
            rt[a[2]] = 1
            if (a[1] == "dir") rdir[a[2]] = 1
            n = split(a[2], parts, "/"); pre = ""
            for (i = 2; i < n; i++) { pre = pre "/" parts[i]; parent[pre] = 1 }
          }
        }
        {
          p = $2
          ok = (p ~ "^/(tmp|run|var/tmp)(/|$)" || p ~ "^/home/fabiocar(/\\.config|/\\.cache|/\\.local|/\\.nv|/\\.python_history|/\\.bash_history|/\\.wget-hsts)?(/|$)" \
                || p == "/home" || p ~ "^/dev(/|$)" || p == "/var" || p == "/etc/localtime" || p == "/usr/local" || p ~ "^/usr/local/zed(/\\.log(/|$)|$)")
          run = (p in rt) || (p in parent)
          for (d in rdir) if (index(p, d "/") == 1) run = 1
          if (p ~ "^/(etc/ld\\.so\\.cache|etc/ld\\.so\\.conf\\.d(/00-nvcr-[0-9]+\\.conf)?|var/cache(/ldconfig(/aux-cache)?)?|usr/sbin/docker-init|etc/nv_tegra_release|etc)$") run = 1
          # Not in the CSV, mounted by the runtime as well (Xorg driver, GLX, the libv4l links).
          if (p ~ "^/usr/lib/xorg(/|$)" || p ~ "^/usr/lib/aarch64-linux-gnu/(nvidia/libGLX_indirect\\.so\\.0|libv4l(2|convert)\\.so\\.0)$") run = 1
          printf "  %s %s %s\n", (ok ? "ok        " : (run ? "runtime   " : "UNEXPECTED")), $1, p
        }' | sort | uniq | head -400
      rm -f -- "$rt_list"
      echo "== processes in $CTR"
      docker top "$CTR" -o pid,ppid,ni,cls,rtprio,etime,args 2>/dev/null | cut -c1-200 | head -80
    fi
    echo "== state $STATE: files not owned by $(id -un)"
    find "$STATE" ! -user "$(id -un)" 2>/dev/null | head -20 | sed 's/^/  /'
    echo "  ($(find "$STATE" ! -user "$(id -un)" 2>/dev/null | wc -l) in total)"
    echo "== state usage"
    du -sh "$STATE"/* 2>/dev/null | sed 's/^/  /'
    echo "== volumes"
    docker volume ls --filter name=f1tenth_car --format '  {{.Name}}' 2>/dev/null
    echo "== host view (guard --for humble)"
    "$GUARD" --for humble 2>&1 | sed 's/^/  /' || true
  } 2>&1 | tee "$out"
  note "report: $out"
}

cmd_history() {
  local n=30
  [ "${1:-}" = -n ] && n=${2:-30}
  tail -n "$n" "$HISTORY" 2>/dev/null || note "no history yet ($HISTORY)"
}

# ---- wait-host: the user unit's ExecStartPre --------------------------------------------------
# A user unit cannot be ordered after the system's docker.service or jtop.service,
# so after the cutover (enabled at boot, plan P12) `up` could run before either
# is ready and refuse. This waits for both, then returns at once on a running
# system.
host_ready() { docker version --format '{{.Server.Version}}' > /dev/null 2>&1 && [ -S "$JTOP_SOCK" ]; }
cmd_wait_host() {
  local limit=${1:-${F1T_WAIT_HOST_S:-120}} t0 waited slept=0
  case "$limit" in ''|*[!0-9]*) usage_err "wait-host: SECONDS must be a whole number, got '$limit'" ;; esac
  t0=$(date +%s)
  until host_ready; do
    waited=$(( $(date +%s) - t0 ))
    if [ "$waited" -ge "$limit" ]; then
      docker version --format '{{.Server.Version}}' > /dev/null 2>&1 \
        || echo "$PROG: wait-host: dockerd does not answer after $waited s" >&2
      [ -S "$JTOP_SOCK" ] || echo "$PROG: wait-host: $JTOP_SOCK is not a socket after $waited s (the jtop service)" >&2
      history_line "wait-host FAILED after ${waited}s"
      exit 1
    fi
    sleep 2
    slept=1
  done
  [ "$slept" = 0 ] || note "wait-host: dockerd and $JTOP_SOCK ready after $(( $(date +%s) - t0 )) s"
  return 0
}

# ---- main -----------------------------------------------------------------------------------------------
[ $# -gt 0 ] || { usage >&2; exit 2; }
sub=$1; shift
case "$sub" in
  -h|--help|help) usage ;;
  pin) cmd_pin "$@" ;;
  up) cmd_up "$@" ;;
  down) cmd_down "$@" ;;
  # A fresh process for each half, so `set -e` applies inside both; leftovers
  # (exit 3) do not stop the restart, `up`'s guard judges what is still live.
  # --dry-run reaches both halves (a dry restart must not stop anything).
  restart)
    down_args=()
    for a in "$@"; do [ "$a" != --dry-run ] || down_args=(--dry-run); done
    rc=0; "$SELF" down "${down_args[@]}" || rc=$?
    [ "$rc" = 0 ] || [ "$rc" = 3 ] || exit "$rc"
    exec "$SELF" up "$@" ;;
  status) cmd_status ;;
  shell) cmd_shell "$@" ;;
  exec) cmd_exec "$@" ;;
  logs) cmd_logs "$@" ;;
  engine) cmd_engine "$@" ;;
  drift) cmd_drift ;;
  report) cmd_report ;;
  history) cmd_history "$@" ;;
  wait-host) cmd_wait_host "$@" ;;
  *) echo "$PROG: unknown command '$sub'" >&2; usage >&2; exit 2 ;;
esac
