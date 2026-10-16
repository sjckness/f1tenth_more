#!/usr/bin/env bash
# llm.sh -- send one natural-language command to the LLM planner (llm_planner_node).
# Run it on the host where llama-server runs (the Thor: the 'intelligence' component, started
# by sim_stack.sh); llama-server listens on localhost only. It runs
#   sim_isaac.sh run --net <net> ros2 run llm llm_planner_node [--dry-run] "<command>"
# i.e. the stack's own ROS environment, after checking llama-server's /health (the URL the
# planner itself uses, llm_planner_node.LLAMA_URL): down -> exit with a message; 503 (model
# still loading) -> wait up to 60 s.
#
# Usage:
#   scripts/sim/llm.sh                        # sends COMMAND below
#   scripts/sim/llm.sh "go to the person"     # sends this instead
#   scripts/sim/llm.sh --dry-run "turn left at the wall"   # translate + write the mission, no load/start
# Options: --dry-run, --net local|eth (default eth), --domain N, -h|--help.
# Prints the command sent, the generated mission file and the planner's exit code. The
# planner exits 0 even when it refuses or fails to plan, so its RESULT status is printed too.

COMMAND="go ahead 3 meters"

if [ -z "${BASH_VERSION:-}" ]; then exec bash "$0" "$@"; fi
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HEALTH_WAIT=60

usage() { awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; }
log() { echo "[llm $(date +%H:%M:%S)] $*"; }
die() { echo "[llm] ERROR: $*" >&2; exit 1; }

# ---- inside the stack's ROS environment (re-invoked through sim_isaac.sh run) ------------
if [ -n "${F1T_LLM_INNER:-}" ]; then
  COMMAND="$F1T_LLM_COMMAND"
  url="$(python3 -c 'from llm.llm_planner_node import LLAMA_URL; print(LLAMA_URL)' 2>/dev/null)" \
    || die "cannot import llm.llm_planner_node: is the workspace built (llm package)?"
  base="${url%/completion}"
  log "llama-server: $base (llm_planner_node.LLAMA_URL)"
  t0=$SECONDS
  while true; do
    code="$(curl -s -o /dev/null -w '%{http_code}' --max-time 5 "$base/health" || true)"
    case "$code" in
      200) log "llama-server ready (/health 200)"; break ;;
      503) [ $((SECONDS - t0)) -ge "$HEALTH_WAIT" ] \
             && die "llama-server still loading its model after ${HEALTH_WAIT}s (/health 503)"
           log "llama-server loading the model (/health 503), waiting ..."; sleep 2 ;;
      *) [ "$code" = 000 ] && code="no answer"
         die "llama-server not reachable at $base (/health: ${code:-no answer}). It runs on the stack host \
only (localhost): start the stack there with scripts/sim/sim_stack.sh (enable_intelligence:=true) and \
run llm.sh on that host." ;;
    esac
  done
  PLANNER=(ros2 run llm llm_planner_node)
  [ "$F1T_LLM_DRY" = true ] && PLANNER+=(--dry-run)
  PLANNER+=("$COMMAND")
  dry_note=""
  [ "$F1T_LLM_DRY" = true ] && dry_note=" (--dry-run: translate + write the mission, no load/start)"
  log "command sent: \"$COMMAND\"$dry_note"
  out="$(mktemp /tmp/llm_sh.XXXXXX)"
  rc=0
  "${PLANNER[@]}" 2>&1 | tee "$out" || rc=${PIPESTATUS[0]}
  summary="$(python3 - "$out" <<'PY'
import json, sys
res = None
for line in open(sys.argv[1], errors='replace'):
    if line.startswith('RESULT '):
        try:
            res = json.loads(line[len('RESULT '):])
        except ValueError:
            pass
if res is None:
    print('-')
    print('<no RESULT line>')
else:
    print(res.get('mission_path') or '-')
    print(f"status={res.get('status')}"
          + (f" error={res.get('error_type')}: {res.get('error_message')}" if res.get('error_type') else ''))
PY
)"
  rm -f "$out"
  { read -r mission; read -r result; } <<< "$summary"
  echo "== llm.sh =="
  echo "command:        \"$COMMAND\""
  echo "mission file:   $mission"
  echo "planner result: $result"
  echo "planner exit:   $rc"
  exit "$rc"
fi

# ---- outside: arguments, then re-run this file inside sim_isaac.sh run ------------------
NET=eth; DRY=false; RUN_OPTS=()
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --dry-run) DRY=true; shift ;;
    --net) NET="${2:?--net needs local|eth}"; shift 2 ;;
    --net=*) NET="${1#*=}"; shift ;;
    --domain) RUN_OPTS+=(--domain "${2:?--domain needs a number}"); shift 2 ;;
    --) shift; [ $# -gt 0 ] && COMMAND="$*"; break ;;
    -*) die "unknown option '$1' (--help)" ;;
    *) COMMAND="$*"; break ;;
  esac
done
[ -n "$COMMAND" ] || die "empty command"
export F1T_LLM_INNER=1 F1T_LLM_COMMAND="$COMMAND" F1T_LLM_DRY="$DRY"
exec "$HERE/sim_isaac.sh" run --net "$NET" "${RUN_OPTS[@]}" -- "$HERE/llm.sh"
