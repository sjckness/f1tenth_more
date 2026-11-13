#!/usr/bin/env bash
# hil_night.sh -- the office HIL campaign UNATTENDED: start it in the evening, read MORNING.md.
#
# Runs ON THE ORIN (host side), detached from the terminal. The workstation side is
# scripts\sim\win\hil_follow.bat, left open: it restarts Isaac with whatever layout this script
# asks for, through a mailbox on this machine (~/.local/state/f1tenth_hil/isaac/), polled by the
# workstation over ssh -- so nothing here connects to the PC and the PC opens no port.
#
# Usage:
#   scripts/sim/campaign/hil_night.sh start [--reps N]   detach and run the whole night
#   scripts/sim/campaign/hil_night.sh status             where it is (any shell, any time)
#   scripts/sim/campaign/hil_night.sh stop               abort cleanly: stack, recorder, Isaac
#   scripts/sim/campaign/hil_night.sh run [--reps N]     the same night, in the foreground
#
# WHAT A NIGHT IS -- the RUNBOOK's Phases 2-4 with every manual step replaced:
#   Isaac campaign_mb, stack, the gate (hil_orin check), recorder, preflight, translate,
#   M02 x1 and the CHAIN CHECK (the export has a row for it, or nothing below is worth running),
#   M02 x(N-1); boundary to campaign_ma and the chair GO/NO-GO, switching to campaign_ma_bright
#   if no chair is tracked; M01 xN; boundary to campaign_md; M03 xN; analyze; everything off.
#   A boundary is: stack down (the launch tree, from inside the container), 8 s of Fast DDS
#   lease, Isaac restarted on the new layout and running, stack up, llama-server healthy, one
#   controller in the graph (pc_record.py ready).
# A mission that ends short of its N runs gets one recovery: a full boundary on the same layout,
# then the missing runs. The recorder numbers repetitions max+1, so nothing is ever overwritten.
#
# WHAT IT DOES NOT DO: judge. success / transl_ok stay empty on purpose -- the runs are run with
# --no-gate, and MORNING.md lists each one with its evidence (final ground-truth pose against the
# target and the criterion's tolerance, move outcomes, the plan) so the verdicts take minutes.
# first_test_campaign had 75 runs and "evaluated by hand: 0"; the morning file is the fix.
#
# Output: office_hil_campaign/aux/night/<start>/ -- night.log (everything), steps.tsv, one log per
# stack bringup, the recorder's log, every follower state snapshot, and MORNING.md (also copied
# to office_hil_campaign/MORNING.md).
if [ -z "${BASH_VERSION:-}" ]; then exec bash "$0" "$@"; fi
set -uo pipefail          # NOT -e: every step is checked by hand, and a failure is reported
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
DRIVER="$HERE/hil_campaign.sh"
HIL_ORIN="$REPO/scripts/sim/hil_orin"
CAMPAIGN_DIR="$REPO/office_hil_campaign"
MB="${HIL_NIGHT_MAILBOX:-$HOME/.local/state/f1tenth_hil/isaac}"   # hil_follow.py MAILBOX (env: tests only)
PIDFILE="$MB/../night.pid"
CURFILE="$MB/../night.current"                  # the running night's directory
STACK_ARGS=(enable_intelligence:=true enable_mission_logger:=true planner_controller_mode:=corridor_mpc)
LLM_HEALTH="http://127.0.0.1:8083/health"       # LLM1, f1tenth_intelligence/llm/config/models.yaml
LEASE_S=8
ISAAC_TIMEOUT=600        # s from the request to the first RTF line (headless start is 20-70 s, a
                         # cold shader cache much more)
FOLLOWER_WAIT=7200       # s the night waits at start for hil_follow.bat to show up

log() { echo "[night $(date +%H:%M:%S)] $*"; }
die() { echo "[night] ERROR: $*" >&2; exit 1; }
usage() { awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; }

CMD="${1:-}"; [ $# -gt 0 ] && shift
REPS=3
while [ $# -gt 0 ]; do
  case "$1" in
    --reps) REPS="${2:?--reps needs a number}"; shift 2 ;;
    --reps=*) REPS="${1#*=}"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown argument '$1' (--help)" ;;
  esac
done
[[ "$REPS" =~ ^[0-9]+$ ]] && [ "$REPS" -ge 1 ] || die "--reps must be a positive integer"
mkdir -p "$MB"

night_alive() { [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; }

case "$CMD" in
  start)
    night_alive && die "a night is already running (pid $(cat "$PIDFILE"); '$0 status')"
    ND="$CAMPAIGN_DIR/aux/night/$(date +%Y%m%dT%H%M%S)"
    mkdir -p "$ND"
    NIGHT_DIR="$ND" setsid nohup bash "$0" run --reps "$REPS" > "$ND/night.log" 2>&1 < /dev/null &
    sleep 2
    echo "night started (pid $(cat "$PIDFILE" 2>/dev/null || echo '?')), $REPS repetitions per mission"
    echo "  log:     $ND/night.log"
    echo "  status:  $0 status"
    echo "  morning: $CAMPAIGN_DIR/MORNING.md"
    echo "Now, on the workstation: scripts\\sim\\win\\hil_follow.bat (if it is not open already)."
    exit 0 ;;
  status)
    if night_alive; then echo "night RUNNING (pid $(cat "$PIDFILE"))"; else echo "no night running"; fi
    nd="$(cat "$CURFILE" 2>/dev/null)"
    [ -n "$nd" ] && [ -d "$nd" ] || { echo "(no night directory yet)"; exit 0; }
    echo "dir: $nd"
    [ -f "$nd/steps.tsv" ] && { echo "--- steps"; column -t -s $'\t' "$nd/steps.tsv"; }
    if [ -f "$MB/state" ]; then
      echo "--- workstation follower (state written $(( $(date +%s) - $(stat -c %Y "$MB/state") )) s ago)"
      sed -n '1,/^---- log tail/p' "$MB/state" | grep -v '^---- log tail'
    else
      echo "--- workstation follower: never seen (hil_follow.bat)"
    fi
    echo "--- night.log, last lines"; tail -n 15 "$nd/night.log" 2>/dev/null
    exit 0 ;;
  stop)
    night_alive || { echo "no night running"; exit 0; }
    pid="$(cat "$PIDFILE")"
    echo "stopping the night (pid $pid): it shuts the stack, the recorder and Isaac down itself"
    # the whole group (start runs it under setsid): bash defers its trap until the foreground
    # child exits, and a run can take minutes
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid"
    for _ in $(seq 1 120); do kill -0 "$pid" 2>/dev/null || { echo "stopped"; exit 0; }; sleep 1; done
    echo "still running after 120 s: kill -KILL $pid, then '$HIL_ORIN shell' to check the container"
    exit 1 ;;
  run) ;;
  -h|--help|help|"") usage; exit 0 ;;
  *) die "unknown subcommand '$CMD' (start|status|stop|run; --help)" ;;
esac

# =========================================== the night ===========================================
night_alive && [ "$(cat "$PIDFILE")" != "$$" ] && die "a night is already running (pid $(cat "$PIDFILE"))"
ND="${NIGHT_DIR:-$CAMPAIGN_DIR/aux/night/$(date +%Y%m%dT%H%M%S)}"
mkdir -p "$ND"
echo $$ > "$PIDFILE"
echo "$ND" > "$CURFILE"
touch "$ND/started"
find "$CAMPAIGN_DIR" -mindepth 2 -maxdepth 2 -type d -name 'P*-R*-*' 2>/dev/null | sort > "$ND/preexisting_runs.txt"
printf 'time\tstep\tstatus\tdetail\n' > "$ND/steps.tsv"
step() {   # $1 step, $2 OK|FAIL|WARN|INFO, $3 detail
  printf '%s\t%s\t%s\t%s\n' "$(date +%H:%M:%S)" "$1" "$2" "${3:-}" >> "$ND/steps.tsv"
  log "[$2] $1${3:+ -- $3}"
}
eval "$(python3 "$REPO/scripts/sim/hil_config.py" --sh)" || die "cannot read the HIL config"
C="$HIL_ORIN_CONTAINER"
log "night $ND, $REPS repetitions per mission, checkout $(git -C "$REPO" rev-parse --short HEAD)"
dirty="$(git -C "$REPO" status --porcelain -- . ':!office_hil_campaign' 2>/dev/null | head -5)"
[ -n "$dirty" ] && step "provenance" WARN "tree dirty outside office_hil_campaign: $(echo $dirty)"

# ---- the workstation follower -------------------------------------------------------------
SEQ="$(awk '{print $1; exit}' "$MB/request" 2>/dev/null)"
[[ "${SEQ:-}" =~ ^[0-9]+$ ]] || SEQ=0
st() { sed -n "s/^$1=//p" "$MB/state" 2>/dev/null | head -1; }
follower_age() { [ -f "$MB/state" ] && echo $(( $(date +%s) - $(stat -c %Y "$MB/state") )) || echo 99999; }
isaac_request() {   # $1 run|stop, $2 layout
  SEQ=$((SEQ + 1))
  printf '%s %s %s\n' "$SEQ" "$1" "${2:-}" > "$MB/request.tmp" && mv "$MB/request.tmp" "$MB/request"
}
wait_follower() {
  local t0; t0=$(date +%s)
  while [ "$(follower_age)" -gt 20 ]; do
    [ $(( $(date +%s) - t0 )) -ge "$FOLLOWER_WAIT" ] && return 1
    [ $(( ($(date +%s) - t0) % 60 )) -lt 3 ] && log "waiting for scripts\\sim\\win\\hil_follow.bat on the workstation..."
    sleep 3
  done
  log "workstation follower alive (host $(st host), state $(follower_age) s old)"
}
isaac_run() {   # $1 layout -> 0 once Isaac logs its first RTF on that layout
  local layout="$1" t0; t0=$(date +%s)
  isaac_request run "$layout"
  log "asked the workstation for Isaac on $layout (seq $SEQ)"
  while :; do
    sleep 3
    if [ "$(follower_age)" -gt 60 ]; then
      step "isaac $layout" FAIL "the workstation follower stopped answering ($(follower_age) s)"; return 1
    fi
    if [ "$(st seq)" = "$SEQ" ]; then
      case "$(st phase)" in
        running) break ;;
        exited) [ "$(st restarts)" -ge 2 ] && {
                  cp "$MB/state" "$ND/isaac_${SEQ}_${layout}.state"
                  step "isaac $layout" FAIL "Isaac exited ($(st exit_code)) after $(st restarts) restarts; tail in isaac_${SEQ}_${layout}.state"
                  return 1; } ;;
      esac
    fi
    if [ $(( $(date +%s) - t0 )) -ge "$ISAAC_TIMEOUT" ]; then
      cp "$MB/state" "$ND/isaac_${SEQ}_${layout}.state" 2>/dev/null
      step "isaac $layout" FAIL "no RTF line in ${ISAAC_TIMEOUT} s (phase $(st phase))"; return 1
    fi
  done
  sleep 12    # one more RTF period: the first one includes the warm-up
  cp "$MB/state" "$ND/isaac_${SEQ}_${layout}.state"
  local rtf; rtf="$(st rtf)"
  step "isaac $layout" OK "RTF $rtf, $(( $(date +%s) - t0 )) s after the request"
  awk -v r="$rtf" 'BEGIN { exit !(r + 0 < 0.95) }' \
    && step "isaac $layout" WARN "RTF $rtf < 0.95: the run is slower than real time"
  return 0
}

# ---- the stack, the recorder ----------------------------------------------------------------
NSTACK=0
cpkill_tree() {   # $1 pgrep -f pattern: SIGINT the oldest match, then SIGKILL what is left of its tree
  docker exec "$C" bash -c '
    pat="$1"; p=$(pgrep -o -f "$pat") || exit 0
    tree() { echo "$1"; for c in $(pgrep -P "$1"); do tree "$c"; done; }
    all=$(tree "$p")
    kill -INT "$p" 2>/dev/null
    for i in $(seq 1 40); do kill -0 "$p" 2>/dev/null || break; sleep 1; done
    sleep 2
    left=""; for q in $all; do kill -0 "$q" 2>/dev/null && left="$left $q"; done
    [ -n "$left" ] && { echo "SIGKILL:$left"; kill -KILL $left 2>/dev/null; }
    true' _ "$1" < /dev/null
}
port_busy() { ss -Hlntu 2>/dev/null | awk '{print $5}' | grep -qE ":$1\$"; }
stack_down() {
  log "stack down"
  local out; out="$(cpkill_tree '[r]os2 launch f1tenth_bringup [s]upervisor_bringup')"
  [ -n "$out" ] && log "  $out"
  local i
  for i in $(seq 1 20); do
    port_busy "$HIL_ROS_DISCOVERY_SERVER_PORT" || port_busy 8083 || break; sleep 1
  done
  port_busy "$HIL_ROS_DISCOVERY_SERVER_PORT" && step "stack down" WARN "port $HIL_ROS_DISCOVERY_SERVER_PORT still bound"
  port_busy 8083 && step "stack down" WARN "port 8083 (llama-server) still bound"
  sleep "$LEASE_S"     # the Fast DDS participant lease: the next bringup must not see these nodes
}
stack_up() {   # -> 0 once llama-server is healthy and exactly one controller is up
  NSTACK=$((NSTACK + 1))
  local slog="$ND/stack_${NSTACK}.log" t0; t0=$(date +%s)
  log "stack up ($slog)"
  setsid nohup "$HIL_ORIN" stack "${STACK_ARGS[@]}" > "$slog" 2>&1 < /dev/null &
  local code=""
  while [ $(( $(date +%s) - t0 )) -lt 300 ]; do
    code="$(curl -s -o /dev/null -w '%{http_code}' --max-time 3 "$LLM_HEALTH" 2>/dev/null)"
    [ "$code" = 200 ] && break
    sleep 3
  done
  [ "$code" = 200 ] || { step "stack $NSTACK" FAIL "llama-server not healthy in 300 s (last HTTP $code; $slog)"; return 1; }
  if ! "$DRIVER" ready >> "$ND/ready_${NSTACK}.log" 2>&1; then
    step "stack $NSTACK" FAIL "not ready: see ready_${NSTACK}.log (a leftover controller, or a node that died)"
    return 1
  fi
  step "stack $NSTACK" OK "llama healthy, $CONTROLLER ready, $(( $(date +%s) - t0 )) s"
}
CONTROLLER=mpc_corr
boundary() {   # $1 layout: stack down, Isaac on it, stack up
  stack_down
  isaac_run "$1" || return 1
  stack_up
}
REC_RUNS=0
recorder_up() {
  cpkill_tree '[t]est_campaign_logger.launch.py' >/dev/null
  REC_RUNS=$((REC_RUNS + 1))
  setsid nohup "$DRIVER" recorder > "$ND/recorder_${REC_RUNS}.log" 2>&1 < /dev/null &
  local i
  for i in $(seq 1 12); do
    sleep 5
    docker exec "$C" pgrep -f '[t]est_campaign_logger' >/dev/null 2>&1 \
      && "$DRIVER" preflight > "$ND/preflight_rec_${REC_RUNS}.log" 2>&1 \
      && { step "recorder" OK "test_campaign_logger up (recorder_${REC_RUNS}.log)"; return 0; }
  done
  step "recorder" FAIL "test_campaign_logger not in the graph after 60 s (recorder_${REC_RUNS}.log, preflight_rec_${REC_RUNS}.log)"
  return 1
}
recorder_alive() { docker exec "$C" pgrep -f '[t]est_campaign_logger' >/dev/null 2>&1; }

# ---- missions ---------------------------------------------------------------------------------
mdir_of() { case "$1" in M01) echo M01_door_left_chair ;; M02) echo M02_wall_left_wall ;;
                         M03) echo M03_person_door_right ;; esac; }
all_runs() { find "$CAMPAIGN_DIR" -mindepth 2 -maxdepth 2 -type d -name 'P*-R*-*' 2>/dev/null | sort; }
count_runs() {   # test folders of mission $1 that did not exist when the night began
  all_runs | grep "/$(mdir_of "$1")/" | comm -23 - "$ND/preexisting_runs.txt" | wc -l
}
csv_rows() {   # rows of $1's mission in campaign_results.csv (after an export)
  python3 - "$CAMPAIGN_DIR/campaign_results.csv" "$(mdir_of "$1")" <<'PY' 2>/dev/null || echo 0
import csv, sys
try:
    rows = list(csv.DictReader(open(sys.argv[1], newline='', encoding='utf-8', errors='replace')))
except OSError:
    rows = []
print(sum(1 for r in rows if r.get('mission') == sys.argv[2]))
PY
}
mission_runs() {   # $1 id, $2 how many, $3 layout (for the recovery boundary)
  local id="$1" want="$2" layout="$3" got=0 before attempt
  for attempt in 1 2; do
    recorder_alive || { step "$id" WARN "the recorder was gone: restarting it"; recorder_up || return 1; }
    before="$(count_runs "$id")"
    log "$id: $((want - got)) run(s), attempt $attempt"
    "$DRIVER" mission "$id" --reps "$((want - got))" --no-gate >> "$ND/mission_$id.log" 2>&1 \
      || log "$id: the driver exited non-zero (mission_$id.log)"
    got=$((got + $(count_runs "$id") - before))
    [ "$got" -ge "$want" ] && { step "$id" OK "$got run(s) recorded"; return 0; }
    [ "$attempt" = 2 ] && break
    step "$id" WARN "only $got of $want run(s) recorded: recovering with a full boundary on $layout"
    boundary "$layout" || break
  done
  step "$id" FAIL "$got of $want run(s) recorded (mission_$id.log)"
  return 1
}
gonogo() {   # $1 class -> 0 when a track of it reaches /costmap/semantic_tracks, car at rest
  "$DRIVER" see "$1" >> "$ND/gonogo.log" 2>&1
}

# ---- the end, whatever happened -----------------------------------------------------------------
FINISHED=false
finish() {
  $FINISHED && return; FINISHED=true
  trap - INT TERM
  log "=== shutting down ==="
  # a run interrupted by `stop`: its planner and side recorder were started by docker exec, and a
  # killed tty-less docker exec client leaves its process running in the container
  cpkill_tree '[l]lm_planner_node' >/dev/null
  cpkill_tree '[p]c_record.py record' >/dev/null
  stack_down
  if [ "$(all_runs | comm -23 - "$ND/preexisting_runs.txt" | wc -l)" -gt 0 ]; then
    log "analyze (--no-gate: the verdicts are the morning's job)"
    if "$DRIVER" analyze --no-gate > "$ND/analyze.log" 2>&1; then step "analyze" OK "analysis/report.txt"
    else step "analyze" WARN "non-zero exit (analyze.log)"; fi
  fi
  cpkill_tree '[t]est_campaign_logger.launch.py' >/dev/null
  isaac_request stop
  log "asked the workstation to stop Isaac (seq $SEQ)"
  morning
  rm -f "$PIDFILE"
  log "done: $CAMPAIGN_DIR/MORNING.md"
}
abort() { step "night" FAIL "$1"; finish; exit 1; }
trap 'step night FAIL "stopped by a signal"; finish; exit 130' INT TERM
trap 'finish' EXIT

# ---- MORNING.md ---------------------------------------------------------------------------------
morning() {
  python3 - "$ND" "$CAMPAIGN_DIR" "$REPS" "$(git -C "$REPO" rev-parse --short HEAD)" <<'PY' \
    > "$ND/MORNING.md" 2> "$ND/morning.err"
import csv, glob, json, os, re, sys, time
nd, cdir, reps, head = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4]
started = os.path.getmtime(os.path.join(nd, 'started'))
P = print
steps = list(csv.DictReader(open(os.path.join(nd, 'steps.tsv')), delimiter='\t'))
fails = [s for s in steps if s['status'] == 'FAIL']
P('# Office HIL campaign -- night of %s' % time.strftime('%Y-%m-%d', time.localtime(started)))
P()
P('Checkout `%s`, %d repetitions per mission, folder `%s`.' % (head, reps, os.path.relpath(nd, cdir)))
P()
P('**%s**' % ('Completed without a failed step.' if not fails else
              'FAILED STEPS: ' + '; '.join('%s (%s)' % (s['step'], s['detail']) for s in fails)))
P()
P('## Steps')
P()
P('| time | step | status | detail |')
P('|---|---|---|---|')
for s in steps:
    P('| %s | %s | %s | %s |' % (s['time'], s['step'], s['status'], (s['detail'] or '').replace('|', '/')))
P()
# ---- the runs, with their evidence ----
TOL = {'M01': 0.5, 'M02': 0.4, 'M03': 0.3}   # the criterion's tolerance on the FINAL pose
judged = {}
try:
    for r in csv.DictReader(open(os.path.join(cdir, 'campaign_results.csv'), newline='',
                                 encoding='utf-8', errors='replace')):
        judged[r.get('test_id', '')] = r
except OSError:
    pass
P('## Runs to judge')
P()
P('Fill `success` (1/0), `transl_ok` (1/0) and `notes` for each test_id in '
  '`office_hil_campaign/campaign_results.csv`, then `scripts/sim/campaign/hil_campaign.sh analyze`.')
P('"final within tol" is ONLY the last target of the criterion (prompts.yaml has the intermediate '
  'ones); the sheet has the trail.')
P()
P('| mission | test_id | final off by | final within tol | mission states (last 3) | bag outcome | sheet |')
P('|---|---|---|---|---|---|---|')
n = 0
for mid in ('M02', 'M01', 'M03'):
    for sheet in sorted(glob.glob(os.path.join(cdir, 'aux', mid, '*', 'verdict_sheet.txt'))):
        if os.path.getmtime(sheet) < started:
            continue
        n += 1
        text = open(sheet, encoding='utf-8', errors='replace').read()
        tid = re.search(r'verdict sheet: (\S+)', text)
        tid = tid.group(1) if tid else '?'
        off = re.search(r'off by\s+([0-9.]+) m', text)
        offs = '%s m' % off.group(1) if off else 'no GT'
        within = ('yes' if float(off.group(1)) <= TOL[mid] else 'NO') if off else '?'
        bag = text.split('  bag:')[-1] if '  bag:' in text else ''
        outc = re.search(r'"outcome":\s*"([^"]*)"', bag)
        trail = text.split('  mission trail:')[-1].split('  events:')[0] if 'mission trail:' in text else ''
        states = [l.strip().split(',')[2] for l in trail.splitlines()
                  if l.strip() and l.strip().count(',') >= 2 and not l.strip().startswith('t_sim')]
        P('| %s | `%s` | %s | %s (<= %.1f) | %s | %s | `%s` |' % (
            mid, tid, offs, within, TOL[mid], ' > '.join(states[-3:]) or '-',
            outc.group(1) if outc else '?', os.path.relpath(sheet, cdir)))
if not n:
    P('| - | no run was recorded tonight | | | | | |')
P()
P('Translation (`transl_ok`): compare each test folder\'s `plan.json` with the '
  '`expected_sequence` comment above the mission in `prompts.yaml`.')
P()
# ---- the go/no-go and the scene each mission ran in ----
gl = os.path.join(nd, 'gonogo.log')
if os.path.exists(gl):
    P('## M01 go/no-go (chair tracked at rest)')
    P()
    P('```')
    P(open(gl, encoding='utf-8', errors='replace').read().strip())
    P('```')
    P()
# ---- Isaac ----
P('## Isaac sessions')
P()
for f in sorted(glob.glob(os.path.join(nd, 'isaac_*.state')), key=os.path.getmtime):
    kv = dict(l.split('=', 1) for l in open(f, encoding='utf-8', errors='replace').read().split('---- log tail')[0].splitlines() if '=' in l)
    P('- seq %s `%s`: phase %s, RTF %s, restarts %s%s' % (kv.get('seq'), kv.get('layout'), kv.get('phase'),
      kv.get('rtf'), kv.get('restarts'), ', exit %s' % kv['exit_code'] if kv.get('exit_code') else ''))
P()
rep = os.path.join(cdir, 'analysis', 'report.txt')
if os.path.exists(rep) and os.path.getmtime(rep) >= started:
    P('## analysis/report.txt')
    P()
    P('```')
    P(open(rep, encoding='utf-8', errors='replace').read().strip()[:6000])
    P('```')
    P()
P('## Afterwards')
P()
P('```bash')
P('git add office_hil_campaign && git commit -m "office_hil_campaign: night %s, judged"' %
  time.strftime('%Y-%m-%d', time.localtime(started)))
P('# workstation: git fetch orin campaign-office ; rsync the bags (hil_campaign.sh bags)')
P('```')
PY
  cp "$ND/MORNING.md" "$CAMPAIGN_DIR/MORNING.md" 2>/dev/null
}

# ---- the night itself -----------------------------------------------------------------------------
docker ps -q -f "name=^${C}$" | grep -q . || abort "container $C is not running (scripts/sim/hil_orin up)"
"$HIL_ORIN" preflight > "$ND/orin_preflight.log" 2>&1 \
  || abort "hil_orin preflight failed (orin_preflight.log)"
step "orin preflight" OK
wait_follower || abort "no workstation follower in $FOLLOWER_WAIT s: start scripts\\sim\\win\\hil_follow.bat"
step "follower" OK "$(st host)"

# Phase 2-3: Isaac on M02's scene, the stack, THE GATE
boundary campaign_mb || { log "retrying the first bringup once"; boundary campaign_mb; } \
  || abort "the first bringup failed twice"
if "$HIL_ORIN" check > "$ND/check_1.log" 2>&1; then
  step "gate (hil_orin check)" OK "check_1.log"
else
  step "gate (hil_orin check)" WARN "failed once: full boundary and again (check_1.log)"
  boundary campaign_mb || abort "bringup failed after the gate"
  "$HIL_ORIN" check > "$ND/check_2.log" 2>&1 \
    || abort "THE GATE FAILED twice (check_1.log, check_2.log): the link or the sim is not healthy, nothing downstream is worth running"
  step "gate (hil_orin check)" OK "second attempt (check_2.log)"
fi
recorder_up || abort "the recorder did not come up: no run would be recorded"
if "$DRIVER" translate > "$ND/translate.log" 2>&1; then step "translate" OK "translate.log"
else step "translate" WARN "a command did not translate cleanly (translate.log): its runs are findings"; fi

# M02: one run, the chain check, then the rest
mission_runs M02 1 campaign_mb || abort "the M02 smoke run was not recorded"
"$DRIVER" judge M02 > "$ND/chain_check.log" 2>&1
rows="$(csv_rows M02)"
[ "${rows:-0}" -ge 1 ] || abort "CHAIN CHECK: the export has no M02 row after the smoke run (chain_check.log): the analysis would have nothing"
step "chain check" OK "campaign_results.csv has $rows M02 row(s)"
[ "$REPS" -gt 1 ] && mission_runs M02 "$((REPS - 1))" campaign_mb

# M01: its scene, the chair go/no-go (brighter lights if it fails), the runs
M01_LAYOUT=campaign_ma
if boundary campaign_ma; then
  if gonogo chair; then
    step "M01 go/no-go" OK "chair tracked on campaign_ma"
  else
    step "M01 go/no-go" WARN "no chair on campaign_ma: switching to campaign_ma_bright (60000)"
    if boundary campaign_ma_bright; then
      M01_LAYOUT=campaign_ma_bright
      if gonogo chair; then step "M01 go/no-go" OK "chair tracked on campaign_ma_bright"
      else step "M01 go/no-go" WARN "NO chair even on campaign_ma_bright: M01 runs anyway (expect target_not_found -- a finding)"; fi
    else
      step "M01" FAIL "bringup on campaign_ma_bright failed"
    fi
  fi
  mission_runs M01 "$REPS" "$M01_LAYOUT"
else
  step "M01" FAIL "bringup on campaign_ma failed: M01 skipped"
fi

# M03: its scene (the person), the runs
if boundary campaign_md; then
  mission_runs M03 "$REPS" campaign_md
else
  step "M03" FAIL "bringup on campaign_md failed: M03 skipped"
fi

step "night" OK "all missions attempted"
finish
exit 0
