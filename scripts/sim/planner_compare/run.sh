#!/usr/bin/env bash
# run.sh -- planner-controller comparison in Isaac Sim: ONE natural-language command sent through
# the full stack (LLM planner -> mission -> behaviour tree -> controller -> mux -> Isaac car), once
# per planner_controller_mode, then one report comparing paths, trajectories and CPU cost.
#
# Usage (on the sim host, everything on this machine, Isaac included):
#   scripts/sim/planner_compare/run.sh                                   # all three modes, "go straight 3m"
#   scripts/sim/planner_compare/run.sh --modes corridor_rrt,p2p_geometric --command "go ahead 2 meters" --target 2
#   scripts/sim/planner_compare/run.sh --isaac external                  # Isaac already running (same --net/--domain)
#   scripts/sim/planner_compare/run.sh --analyze-only output/planner_compare/20261113_1200
#
# Per mode, in this order:
#   1. stack:   sim_isaac.sh stack planner_controller_mode:=<mode> (a fresh bringup: the mode is fixed at launch)
#   2. ready:   pc_record.py ready -- the mode's controller node is up, no other controller is,
#               /mission/status, /odometry/filtered and /sim/ground_truth/odom flow
#   3. reset:   restart.sh (/reset_all: car back at the scene's start pose, EKFs and SLAM zeroed, checked)
#   4. record:  pc_record.py record (background) -- ends when the mission leaves RUNNING, + 4 s
#   5. command: llm.sh "<command>" -- llama-server translates, the planner loads + starts the mission
#   6. stop the stack (SIGINT to its process group, then TERM/KILL), wait out the 5 s DDS lease
# then pc_analyze.py writes OUT/report.md, summary.{json,csv}, trajectories/paths/timeseries/cpu.png.
#
# Options:
#   --modes LIST        comma-separated (default corridor_mpc,corridor_rrt,p2p_geometric)
#   --command TEXT      what the LLM gets (default "go straight 3m")
#   --target M          commanded distance for the report (default 3.0)
#   --out DIR           output dir (default output/planner_compare/<YYYYmmdd_HHMMSS>)
#   --isaac start|external   start Isaac here (default) or use a running one
#   --scene NAME / --layout NAME / --gui    passed to sim_isaac.sh isaac (default base_room / campaign)
#   --net local|eth     ROS network (default local; eth = the runner runs on the Discovery Server host)
#   --domain N          ROS_DOMAIN_ID (default $ROS_DOMAIN_ID, else 42)
#   --ready-timeout S   per-mode bringup budget (default 300)
#   --run-timeout S     per-mode recording budget, wall (default 150)
#   --analyze-only DIR  only (re)write the report of an existing session
#   -h, --help
# Any k:=v argument is passed to every stack bringup (supervisor_bringup.launch.py).
#
# The LLM: llama-server is started ONCE for the session (llm.launch.py, i.e. the 'intelligence'
# component's own launch, outside the supervisor) unless one already answers on
# 127.0.0.1:8083; every stack runs with enable_intelligence:=false so the three bringups do not
# reload the model. Binary: $LLAMA_SERVER_PATH, else stack_params llama_server_path, else
# `llama-server` on PATH, else ~/projects/llm/llama-*/llama-server. Model: llm/config/models.yaml.
#
# Never run a second ROS stack on the same domain meanwhile; the runner refuses to start if a
# controller or a component_supervisor is already running on this host.

if [ -z "${BASH_VERSION:-}" ]; then exec bash "$0" "$@"; fi
# A shell that starts with SIGINT ignored (a background job, some tool runners) cannot trap it,
# so Ctrl+C / kill -INT would skip cleanup and leave the stack running: restore it once.
if [ -z "${F1T_COMPARE_SIGINT:-}" ]; then
  F1T_COMPARE_SIGINT=1 exec env --default-signal=INT bash "$0" "$@"
fi
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIMDIR="$(cd "$HERE/.." && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
SIM="$SIMDIR/sim_isaac.sh"

MODES="corridor_mpc,corridor_rrt,p2p_geometric"
COMMAND="go straight 3m"
TARGET="3.0"
OUT=""
ISAAC=start
SCENE=base_room
LAYOUT=campaign
GUI=false
NET=local
DOMAIN="${ROS_DOMAIN_ID:-42}"
READY_TIMEOUT=300
RUN_TIMEOUT=150
ANALYZE_ONLY=""
STACK_ARGS=()

usage() { awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; }
log() { echo "[compare $(date +%H:%M:%S)] $*"; }
die() { echo "[compare] ERROR: $*" >&2; exit 1; }

while [ $# -gt 0 ]; do
  case "$1" in
    --modes) MODES="$2"; shift 2 ;;
    --command) COMMAND="$2"; shift 2 ;;
    --target) TARGET="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    --isaac) ISAAC="$2"; shift 2 ;;
    --scene) SCENE="$2"; shift 2 ;;
    --layout) LAYOUT="$2"; shift 2 ;;
    --gui) GUI=true; shift ;;
    --net) NET="$2"; shift 2 ;;
    --domain) DOMAIN="$2"; shift 2 ;;
    --ready-timeout) READY_TIMEOUT="$2"; shift 2 ;;
    --run-timeout) RUN_TIMEOUT="$2"; shift 2 ;;
    --analyze-only) ANALYZE_ONLY="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *:=*) STACK_ARGS+=("$1"); shift ;;
    *) die "unknown argument '$1' (--help)" ;;
  esac
done

if [ -n "$ANALYZE_ONLY" ]; then
  exec python3 "$HERE/pc_analyze.py" "$ANALYZE_ONLY"
fi

case "$ISAAC" in start|external) ;; *) die "--isaac must be start or external" ;; esac
case "$NET" in local|eth) ;; *) die "--net must be local or eth" ;; esac
IFS=',' read -r -a MODE_LIST <<< "$MODES"
declare -A NODE=([corridor_mpc]=mpc_corr [corridor_rrt]=corridor_rrt [p2p_geometric]=p2p_geometric)
for m in "${MODE_LIST[@]}"; do
  [ -n "${NODE[$m]:-}" ] || die "unknown mode '$m' (corridor_mpc, corridor_rrt, p2p_geometric)"
done
for a in "${STACK_ARGS[@]}"; do
  case "$a" in planner_controller_mode:=*|enable_intelligence:=*)
    die "$a is set by the runner itself" ;; esac
done
[ -f "$REPO/install/setup.bash" ] || die "no $REPO/install: build the workspace first"

OUT="${OUT:-$REPO/output/planner_compare/$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT/logs"
OUT="$(cd "$OUT" && pwd)"
COMMON=(--net "$NET" --domain "$DOMAIN")
if [ "$NET" = local ]; then DS_ADDR=127.0.0.1; else DS_ADDR="${F1TENTH_SIM_DISCOVERY_IP:-10.42.0.2}"; fi
DS_PORT="${F1TENTH_SIM_DISCOVERY_PORT:-11811}"

# ---- prechecks ------------------------------------------------------------------------------
# /proc scan in python: pgrep -f would match this script's own command line.
busy="$(python3 - <<'PY'
import os
me = {os.getpid(), os.getppid()}
hits = []
for d in os.listdir('/proc'):
    if not d.isdigit() or int(d) in me:
        continue
    try:
        args = open(f'/proc/{d}/cmdline', 'rb').read().split(b'\0')
    except OSError:
        continue
    flat = b' '.join(args)
    if any(f'__node:={n}'.encode() in args for n in ('mpc_corr', 'corridor_rrt', 'p2p_geometric')) \
            or b'component_supervisor_node' in flat or b'supervisor_bringup.launch.py' in flat:
        hits.append(f'{d}: {flat[:120].decode(errors="replace")}')
print('\n'.join(hits))
PY
)"
[ -z "$busy" ] || die "a stack is already running on this host:
$busy"
if [ "$ISAAC" = start ] && docker ps -q -f name=^f1t_isaac$ 2>/dev/null | grep -q .; then
  die "an Isaac container (f1t_isaac) is already running: stop it or use --isaac external"
fi
avail_mb="$(awk '/MemAvailable/ {print int($2/1024)}' /proc/meminfo)"
[ "$avail_mb" -ge 6000 ] || log "WARNING: only ${avail_mb} MB available (Isaac + stack + LLM want ~10 GB)"

# ---- process bookkeeping --------------------------------------------------------------------
PGIDS=()          # every background group we started, stopped in reverse on exit
STACK_PG=""
REC_PG=""
start_bg() {  # <log> <cmd...>: own session (setsid) so the whole tree can be signalled
  local logf="$1"; shift
  # A background job of a script starts with SIGINT ignored, and its children (python, ros2
  # launch) inherit that: restore the default so stop_group's SIGINT is a clean shutdown.
  setsid env --default-signal=INT "$@" < /dev/null > "$logf" 2>&1 &
  echo $!
}
alive() { kill -0 -- "-$1" 2>/dev/null; }
FG_PG=""
run_fg() {  # <cmd...>: a step we wait for, in its own session so a signal to us is not
  # deferred until it returns (bash runs traps only between foreground commands); stdout and
  # stderr are the caller's redirections.
  local rc=0
  setsid env --default-signal=INT "$@" < /dev/null &
  FG_PG=$!
  wait "$FG_PG" || rc=$?
  FG_PG=""
  return "$rc"
}
stop_group() {  # <pgid> <name> [grace_s]
  local pg="$1" name="$2" grace="${3:-40}" i
  [ -n "$pg" ] || return 0
  alive "$pg" || return 0
  log "stopping $name (process group $pg)"
  kill -INT -- "-$pg" 2>/dev/null || true
  for ((i = 0; i < grace * 2; i++)); do alive "$pg" || return 0; sleep 0.5; done
  kill -TERM -- "-$pg" 2>/dev/null || true
  for ((i = 0; i < 20; i++)); do alive "$pg" || return 0; sleep 0.5; done
  log "$name still alive after TERM: SIGKILL"
  kill -KILL -- "-$pg" 2>/dev/null || true
}
cleanup() {
  set +e
  stop_group "$FG_PG" "current step" 10
  stop_group "$REC_PG" recorder 5
  stop_group "$STACK_PG" stack 40
  local i
  for ((i = ${#PGIDS[@]} - 1; i >= 0; i--)); do
    stop_group "${PGIDS[$i]%%:*}" "${PGIDS[$i]#*:}" 60
  done
}
trap cleanup EXIT
trap 'exit 130' INT TERM

simrun() { run_fg "$SIM" run "${COMMON[@]}" -- "$@"; }

# ---- session record -------------------------------------------------------------------------
python3 - "$OUT/session.json" <<PY
import json, socket, subprocess, sys, time
def git(*a):
    try:
        return subprocess.check_output(['git', '-C', '$REPO', *a], text=True).strip()
    except Exception:
        return None
json.dump({'command': """$COMMAND""", 'target_m': float('$TARGET'), 'modes': '$MODES'.split(','),
           'scene': '$SCENE', 'layout': '$LAYOUT', 'net': '$NET', 'domain': int('$DOMAIN'),
           'isaac': '$ISAAC', 'stack_args': """${STACK_ARGS[*]:-}""".split(),
           'host': socket.gethostname(), 'started': time.strftime('%Y-%m-%d %H:%M:%S'),
           'git_branch': git('rev-parse', '--abbrev-ref', 'HEAD'),
           'git_commit': git('rev-parse', '--short', 'HEAD'),
           'git_dirty': bool(git('status', '--porcelain', '--untracked-files=no'))},
          open(sys.argv[1], 'w'), indent=2)
PY
log "session $OUT: \"$COMMAND\", modes ${MODES}, net $NET, domain $DOMAIN"

# ---- Discovery Server, held for the whole session -------------------------------------------
# Each 'sim_isaac.sh stack' would otherwise start its own and kill it on exit, under Isaac's feet.
# ensure_discovery_server.py reuses a live server (and then just idles).
pg="$(start_bg "$OUT/logs/discovery_server.log" "$SIM" run "${COMMON[@]}" -- bash -c \
  'exec python3 "$(ros2 pkg prefix f1tenth_bringup)/share/f1tenth_bringup/scripts/ensure_discovery_server.py" "$0" "$1"' \
  "$DS_ADDR" "$DS_PORT")"
PGIDS+=("$pg:discovery-server")
sleep 6
alive "$pg" || die "Discovery Server did not start: $(tail -5 "$OUT/logs/discovery_server.log")"

# ---- LLM (llama-server), once for the session -----------------------------------------------
health() { curl -s -o /dev/null -w '%{http_code}' --max-time 3 http://127.0.0.1:8083/health || true; }
if [ "$(health)" = 200 ]; then
  log "llama-server already answers on 127.0.0.1:8083: using it"
else
  bin="${LLAMA_SERVER_PATH:-}"
  if [ -z "$bin" ]; then
    bin="$(simrun python3 -c "from f1tenth_params.param_defaults import get_value; print(get_value('llama_server_path'))" 2>/dev/null | tail -1 || true)"
    bin="${bin/#\~/$HOME}"
  fi
  [ -x "$bin" ] || bin="$(command -v llama-server || true)"
  [ -x "$bin" ] || bin="$(ls -1d "$HOME"/projects/llm/llama-*/llama-server 2>/dev/null | tail -1 || true)"
  [ -x "$bin" ] || die "no llama-server binary (set LLAMA_SERVER_PATH)"
  log "starting llama-server $bin (llm.launch.py)"
  pg="$(start_bg "$OUT/logs/llm.log" "$SIM" run "${COMMON[@]}" -- ros2 launch llm llm.launch.py \
    llama_server_path:="$bin" llama_server_cwd:="$(dirname "$bin")")"
  PGIDS+=("$pg:llama-server")
  for ((i = 0; i < 120; i++)); do [ "$(health)" = 200 ] && break; sleep 2; done
  [ "$(health)" = 200 ] || die "llama-server not healthy after 240 s: $(tail -5 "$OUT/logs/llm.log")"
  log "llama-server ready"
fi

# ---- Isaac ----------------------------------------------------------------------------------
if [ "$ISAAC" = start ]; then
  iargs=(isaac "${COMMON[@]}" --scene "$SCENE" --layout "$LAYOUT")
  $GUI && iargs+=(--gui)
  log "starting Isaac Sim (${iargs[*]}), log logs/isaac.log"
  pg="$(start_bg "$OUT/logs/isaac.log" "$SIM" "${iargs[@]}")"
  PGIDS+=("$pg:isaac")
fi
log "waiting up to 300 s for /clock"
simrun timeout 300 ros2 topic echo --once --no-daemon /clock rosgraph_msgs/msg/Clock \
  > "$OUT/logs/clock_wait.log" 2>&1 || die "no /clock from Isaac (logs/isaac.log)"
log "/clock is up"

# ---- one run per mode -----------------------------------------------------------------------
for mode in "${MODE_LIST[@]}"; do
  node="${NODE[$mode]}"
  d="$OUT/$mode"
  mkdir -p "$d"
  log "=== $mode ($node) ==="
  t_bring=$SECONDS
  STACK_PG="$(start_bg "$OUT/logs/stack_$mode.log" "$SIM" stack "${COMMON[@]}" \
    planner_controller_mode:="$mode" enable_intelligence:=false "${STACK_ARGS[@]}")"
  ready_rc=0
  simrun python3 "$HERE/pc_record.py" ready --node "$node" --timeout "$READY_TIMEOUT" \
    > "$OUT/logs/ready_$mode.log" 2>&1 || ready_rc=$?
  ready_s=$((SECONDS - t_bring))
  restart_rc=-1; llm_rc=-1
  if [ "$ready_rc" -ne 0 ]; then
    log "$mode: stack not ready after ${ready_s}s (logs/ready_$mode.log, logs/stack_$mode.log)"
  else
    log "$mode: ready after ${ready_s}s; /reset_all"
    restart_rc=0
    run_fg "$SIMDIR/restart.sh" "${COMMON[@]}" --settle 5 > "$OUT/logs/restart_$mode.log" 2>&1 || restart_rc=$?
    if [ "$restart_rc" -ne 0 ]; then
      log "$mode: restart.sh rc=$restart_rc, once more in 10 s"
      sleep 10
      restart_rc=0
      run_fg "$SIMDIR/restart.sh" "${COMMON[@]}" --settle 5 >> "$OUT/logs/restart_$mode.log" 2>&1 || restart_rc=$?
      [ "$restart_rc" -eq 0 ] || log "$mode: restart.sh still failing (rc=$restart_rc): running anyway, see logs/restart_$mode.log"
    fi
    REC_PG="$(start_bg "$OUT/logs/record_$mode.log" "$SIM" run "${COMMON[@]}" -- \
      python3 "$HERE/pc_record.py" record --node "$node" --out "$d" --timeout "$RUN_TIMEOUT")"
    for ((i = 0; i < 60; i++)); do [ -f "$d/RECORDING" ] && break; alive "$REC_PG" || break; sleep 0.5; done
    [ -f "$d/RECORDING" ] || log "$mode: recorder did not start (logs/record_$mode.log)"
    sleep 2   # subscriptions matched before the command
    log "$mode: sending \"$COMMAND\" through the LLM"
    llm_rc=0
    run_fg "$SIMDIR/llm.sh" "${COMMON[@]}" "$COMMAND" > "$d/llm.log" 2>&1 || llm_rc=$?
    grep -E '^\[llm' "$d/llm.log" | tail -3 || true
    if ! grep -q '^RESULT {"status": "ok"' "$d/llm.log"; then
      log "$mode: the LLM planner did not deliver a mission (rc=$llm_rc, $d/llm.log): no run"
      stop_group "$REC_PG" recorder 5
    fi
    while alive "$REC_PG"; do sleep 1; done
    REC_PG=""
    log "$mode: $(grep -E '\[record\] done' "$OUT/logs/record_$mode.log" | tail -1 || echo 'recorder ended without a summary')"
  fi
  python3 - "$d" "$mode" "$node" "$ready_rc" "$ready_s" "$restart_rc" "$llm_rc" <<'PY'
import hashlib, json, os, shutil, sys
d, mode, node, ready_rc, ready_s, restart_rc, llm_rc = sys.argv[1:]
res = None
try:
    for ln in open(os.path.join(d, 'llm.log'), errors='replace'):
        if ln.startswith('RESULT '):
            try:
                res = json.loads(ln[len('RESULT '):])
            except ValueError:
                pass
except OSError:
    pass
run = {'mode': mode, 'node': node, 'ready_rc': int(ready_rc), 'ready_s': int(ready_s),
       'restart_rc': int(restart_rc), 'llm_rc': int(llm_rc)}
if res:
    run.update({'llm_status': res.get('status'), 'llm_plan_seconds': res.get('plan_seconds'),
                'llm_raw': res.get('llm_raw'), 'mission_path': res.get('mission_path'),
                'llm_error': res.get('error_message')})
    mp = res.get('mission_path')
    if mp and os.path.exists(mp):
        shutil.copy(mp, os.path.join(d, 'mission.json'))
        moves = json.load(open(mp)).get('moves')
        run['mission_sha'] = hashlib.sha256(json.dumps(moves, sort_keys=True).encode()).hexdigest()
json.dump(run, open(os.path.join(d, 'run.json'), 'w'), indent=2)
print(f"[compare] {mode}: llm {run.get('llm_status')} in {run.get('llm_plan_seconds')} s, "
      f"mission {run.get('mission_sha', '-')[:12]}")
PY
  stop_group "$STACK_PG" "stack ($mode)" 40
  STACK_PG=""
  sleep 8   # past the 5 s participant lease: the next bringup must not see this one's nodes
done

# ---- report ---------------------------------------------------------------------------------
log "analysis"
python3 "$HERE/pc_analyze.py" "$OUT" || log "pc_analyze.py failed: rerun with --analyze-only $OUT"
log "done: $OUT/report.md"
