# shellcheck shell=bash
# tools/core/lib/log.sh - output comune di tutti gli script f1t (core ed ext).
#
#   source "$F1T_CORE/lib/log.sh"
#   log_info|log_ok|log_warn|log_fail "testo"     riga con orario, a colori su terminale
#   log_steps 8; log_step "testo"                 [STEP 3/8] testo
#   log_die "testo" [exit]                        log_fail + exit (default 1)
#   check "descrizione" cmd [args...]             esegue cmd: exit 0 = OK, 10 = WARN, altro = FAIL;
#                                                 il valore e' l'ultima riga non vuota del suo output
#   check_warn "descrizione" cmd [args...]        come check, ma un fallimento e' solo WARN
#   check_skip "descrizione" "motivo"             registra un check non eseguito (SKIP)
#   check_record STATO "descrizione" "valore"     registra un risultato calcolato altrove
#   summary [titolo]                              tabella check | valore | stato; ritorna 1 se c'e' un FAIL
#   summary_json FILE [chiave=valore ...]         i check in JSON (per i file *_last.json)
#
# Ogni riga va su stderr e, senza colori, in logs/tools/<script>_<data>.log (F1T_LOG_DIR).
# NO_COLOR=1 toglie i colori. Nessuna dipendenza oltre bash 4 e python3 (solo summary_json).
[ -n "${_F1T_LOG_SH:-}" ] && return 0
_F1T_LOG_SH=1

F1T_CORE=${F1T_CORE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}
F1T_ROOT=${F1T_ROOT:-$(cd "$F1T_CORE/../.." && pwd)}
F1T_SCRIPT=${F1T_SCRIPT:-$(basename "${0%.*}")}
F1T_LOG_DIR=${F1T_LOG_DIR:-$F1T_ROOT/logs/tools}
F1T_LOG_FILE=${F1T_LOG_FILE:-$F1T_LOG_DIR/${F1T_SCRIPT}_$(date +%Y%m%d_%H%M%S).log}
F1T_CHECK_WARN_RC=10

if [ -t 2 ] && [ -z "${NO_COLOR:-}" ]; then
  _C_RST=$'\e[0m' _C_DIM=$'\e[2m' _C_BLU=$'\e[34m' _C_GRN=$'\e[32m' _C_YEL=$'\e[33m' _C_RED=$'\e[31m' _C_BLD=$'\e[1m'
else
  _C_RST='' _C_DIM='' _C_BLU='' _C_GRN='' _C_YEL='' _C_RED='' _C_BLD=''
fi

_log_file_ok=1
_log_ensure_dir() { mkdir -p "$(dirname "$F1T_LOG_FILE")" 2>/dev/null || true; }
_log_to_file() {
  [ "$_log_file_ok" = 1 ] || return 0
  if ! { mkdir -p "$(dirname "$F1T_LOG_FILE")" && printf '%s\n' "$*" >> "$F1T_LOG_FILE"; } 2>/dev/null; then
    _log_file_ok=0
  fi
}

# _log COLORE ETICHETTA testo...
_log() {
  local color=$1 tag=$2; shift 2
  local ts; ts=$(date +%H:%M:%S)
  printf '%s%s%s %s%-5s%s %s\n' "$_C_DIM" "$ts" "$_C_RST" "$color" "$tag" "$_C_RST" "$*" >&2
  _log_to_file "$ts $tag $*"
}
log_info() { _log "$_C_BLU" INFO "$*"; }
log_ok()   { _log "$_C_GRN" OK "$*"; }
log_warn() { _log "$_C_YEL" WARN "$*"; }
log_fail() { _log "$_C_RED" FAIL "$*"; }
log_die()  { log_fail "$1"; exit "${2:-1}"; }

_STEP_N=0 _STEP_TOT=0
log_steps() { _STEP_TOT=$1; _STEP_N=0; }
log_step() {
  _STEP_N=$((_STEP_N + 1))
  local ts; ts=$(date +%H:%M:%S)
  printf '%s%s%s %s[STEP %d/%d]%s %s\n' "$_C_DIM" "$ts" "$_C_RST" "$_C_BLD" "$_STEP_N" "$_STEP_TOT" "$_C_RST" "$*" >&2
  _log_to_file "$ts [STEP $_STEP_N/$_STEP_TOT] $*"
}

# ---------------------------------------------------------------- check / summary
_CHK_NAME=() _CHK_VAL=() _CHK_STATE=()

check_record() {
  local state=$1 desc=$2 val=${3:-}
  _CHK_NAME+=("$desc"); _CHK_VAL+=("$val"); _CHK_STATE+=("$state")
  local mark color
  case "$state" in
    OK) mark='✔' color=$_C_GRN ;;
    WARN) mark='!' color=$_C_YEL ;;
    SKIP) mark='-' color=$_C_DIM ;;
    *) mark='✘' color=$_C_RED ;;
  esac
  printf '  %s%s%s %-38s %s\n' "$color" "$mark" "$_C_RST" "$desc" "$val" >&2
  _log_to_file "  $mark $desc: $val ($state)"
}

_check_run() {
  local soft=$1 desc=$2; shift 2
  local out rc=0 val state
  out=$("$@" 2>&1) || rc=$?
  val=$(printf '%s\n' "$out" | sed '/^[[:space:]]*$/d' | tail -1)
  if [ "$rc" = 0 ]; then state=OK
  elif [ "$rc" = "$F1T_CHECK_WARN_RC" ] || [ "$soft" = 1 ]; then state=WARN
  else state=FAIL
  fi
  check_record "$state" "$desc" "$val"
  [ "$state" != FAIL ]
}
check()      { _check_run 0 "$@" || true; }
check_warn() { _check_run 1 "$@" || true; }
check_skip() { check_record SKIP "$1" "${2:-}"; }

# Stato dell'ultimo check registrato (OK/WARN/FAIL/SKIP).
check_last() { [ ${#_CHK_STATE[@]} -gt 0 ] && echo "${_CHK_STATE[-1]}"; }
# 0 se finora nessun check e' FAIL.
checks_ok() { local s; for s in "${_CHK_STATE[@]}"; do [ "$s" != FAIL ] || return 1; done; return 0; }
checks_count() { local n=0 s; for s in "${_CHK_STATE[@]}"; do [ "$s" != "$1" ] || n=$((n + 1)); done; echo "$n"; }

summary() {
  local title=${1:-$F1T_SCRIPT} i w=5 nf nw
  for i in "${!_CHK_NAME[@]}"; do [ ${#_CHK_NAME[$i]} -le "$w" ] || w=${#_CHK_NAME[$i]}; done
  nf=$(checks_count FAIL); nw=$(checks_count WARN)
  _log_ensure_dir
  {
    echo
    echo "== $title: riepilogo"
    printf '%-*s | %-40s | %s\n' "$w" check valore stato
    printf '%s\n' "$(printf '%*s' $((w + 52)) '' | tr ' ' '-')"
    for i in "${!_CHK_NAME[@]}"; do
      printf '%-*s | %-40s | %s\n' "$w" "${_CHK_NAME[$i]}" "${_CHK_VAL[$i]:0:40}" "${_CHK_STATE[$i]}"
    done
    echo "FAIL: $nf  WARN: $nw  totale: ${#_CHK_NAME[@]}   log: $F1T_LOG_FILE"
  } | tee -a "$F1T_LOG_FILE" 2>/dev/null || true
  [ "$nf" = 0 ]
}

summary_json() {
  local file=$1; shift
  mkdir -p "$(dirname "$file")"
  local i args=()
  for i in "${!_CHK_NAME[@]}"; do args+=("${_CHK_NAME[$i]}" "${_CHK_VAL[$i]}" "${_CHK_STATE[$i]}"); done
  python3 - "$file" "$#" "$@" "${args[@]}" <<'EOF'
import json, sys, time
f, n = sys.argv[1], int(sys.argv[2])
extra = dict(a.split('=', 1) for a in sys.argv[3:3 + n])
rest = sys.argv[3 + n:]
checks = [{'check': rest[i], 'value': rest[i + 1], 'state': rest[i + 2]} for i in range(0, len(rest), 3)]
states = [c['state'] for c in checks]
doc = {'time': int(time.time()), 'time_iso': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
       'status': 'FAIL' if 'FAIL' in states else 'OK', **extra, 'checks': checks}
with open(f, 'w') as fh:
    json.dump(doc, fh, indent=2)
    fh.write('\n')
EOF
}
