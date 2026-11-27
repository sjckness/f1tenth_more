#!/usr/bin/env bash
# One entry point for llm_analytics: the five documented modes, plus the server
# they all need, with the run-folder ritual done by the script instead of by you.
#
# Why this exists
# ---------------
# llm_analytics is correct and awkward to type. planner.md §2 asks for four
# things before every measurement -- cd, source the env, mkdir the run folder,
# invent a unique YYYY-MM-DD-<task-name> name -- and §2.4 then lists what happens
# when one is skipped: --events into a missing folder aborts the run at startup,
# --out/--events default to the current directory and truncate on purpose, a
# rerun into the same folder destroys the previous capture, and a server that is
# not up produces forty 502s instead of an error. All of that is memory work, and
# memory work is what goes wrong at 1 a.m.
#
# So this file owns exactly the bookkeeping and leaves the measurement to
# llm.llm_analytics:
#
#   * sources scripts/env/jazzy.sh itself, so it works from a cold shell;
#   * creates analytics_runs/YYYY-MM-DD-<label>/ before the tool starts;
#   * always passes --out/--events into that folder (nothing lands in the repo);
#   * --label is a NAME: the date is prepended, and a same-day rerun of the same
#     name gets -2, -3, ... (planner.md §2.3, .clinerules ANALYTICS RUN
#     PROTOCOL). A label that already starts with a date is used verbatim;
#   * checks /health first and exits 3 with the fix if the server is down;
#   * starts and stops llama-server (up/down/status). It passes the two launch
#     arguments this machine needs, because stack_params.yaml's
#     llama_server_path default is the Jetson's /home/llm/... path, which does
#     not exist on alex-desktop;
#   * prints the equivalent `ros2 run llm llm_analytics ...` line before it runs
#     it, so the long form stays learnable and nothing is hidden;
#   * can bring a named model up (`up --model NAME`, from config/models.yaml);
#   * can run two of them back to back and compare them (`compare A B`): the
#     folders live under analytics_runs/sequential/ (never mixed with the
#     single-model runs), and scripts/compare_reports.py turns the two captures
#     into one comparison.md.
#
# What it deliberately does NOT do: reimplement the tool. Every flag passed here
# is one of llm_analytics.py's own 19 (build_parser, llm_analytics.py:2082), and
# its 83 tests are untouched. `ros2 run llm llm_analytics --help` remains the
# source of truth; `scripts/analytics.sh help` is the shortcut.
#
# Usage: scripts/analytics.sh <mode> [argument] [options]
#   scripts/analytics.sh up
#   scripts/analytics.sh one "vai dritto e gira a sinistra alla parete"
#   scripts/analytics.sh sweep --label qwen3b-baseline
#   scripts/analytics.sh compare qwen25_3b_instruct qwen25_3b_instruct_q4
#   scripts/analytics.sh replay qwen3b-baseline
#   scripts/analytics.sh help
#
# Exit codes: 0 ok, 1 an exchange failed (the report still exists), 2 usage,
# 3 preflight (no server, no binary, no run to replay).

set -euo pipefail

ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runs_dir="$ws/src/f1tenth_intelligence/llm/analytics_runs"
package_dir="$ws/src/f1tenth_intelligence/llm"
behavior_dir="$ws/src/f1tenth_behavior"
sample_file="$package_dir/test/commands.sample.txt"
params_file="$ws/src/f1tenth_params/config/stack_params.yaml"
models_file="$package_dir/config/models.yaml"
seq_dir="$runs_dir/sequential"
server_dir="$runs_dir/_server"
pid_file="$server_dir/llama-server.pid"
log_file="$server_dir/llama-server.log"
model_file="$server_dir/current_model"

port="${LLAMA_PORT:-8083}"

# Options, set by parse_options(). Globals rather than locals: the cmd_*
# functions read them, and bash's dynamic scope would make a caller's `local`
# work only by accident.
opt_label=""
opt_last=0
opt_airlock=1
opt_go_to=1
opt_timeout=""
opt_wait=120
opt_force=0
opt_verbose=0
opt_simple=0
opt_model=""
opt_sequential=0
# --port was given (or LLAMA_PORT is set): `up --model` then keeps it instead of
# adopting the port models.yaml would make llama-server bind.
opt_port_set=0
if [ -n "${LLAMA_PORT:-}" ]; then opt_port_set=1; fi

# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------
die() { printf 'ERRORE: %s\n' "$*" >&2; exit 2; }
say() { printf '%s\n' "$*"; }

# The liveness probe used by every mode. curl when present, stdlib python
# otherwise: this script also runs on a Jetson and inside a container, and
# neither is guaranteed to ship curl.
health_body() {
  if command -v curl >/dev/null 2>&1; then
    curl -fsS -m 3 "http://127.0.0.1:${port}/health" 2>/dev/null
  else
    python3 - "${port}" <<'PY' 2>/dev/null
import sys
import urllib.request

with urllib.request.urlopen(
        f'http://127.0.0.1:{sys.argv[1]}/health', timeout=3) as response:
    sys.stdout.write(response.read().decode('utf-8', 'replace'))
PY
  fi
}

server_up() { health_body >/dev/null 2>&1; }

# ---------------------------------------------------------------------------
# Environment
# ---------------------------------------------------------------------------
source_env() {
  if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    say 'note          /opt/ros/jazzy/setup.bash not found: no ROS here.'
    say '              replay and probe still work; one/sweep need the server only.'
    return 0
  fi
  if [ -f "$ws/scripts/env/jazzy.sh" ]; then
    # ROS's own setup.bash reads $AMENT_TRACE_SETUP_FILES (and friends) without
    # a default, so it cannot be sourced under `set -u`: lift it for the source
    # and put it straight back. set -e stays on the whole time.
    set +u
    # shellcheck disable=SC1091  # a repo path, not a fixed file
    source "$ws/scripts/env/jazzy.sh"
    set -u
  fi
}

# ---------------------------------------------------------------------------
# stack_params.yaml is the single source of truth for launch defaults; read the
# two keys llama-server needs instead of copying their values into this script.
# ---------------------------------------------------------------------------
stack_param() {  # $1 = top-level key in f1tenth_params/config/stack_params.yaml
  python3 - "$params_file" "$1" <<'PY' 2>/dev/null || true
import sys

try:
    import yaml
except Exception:  # noqa: BLE001 -- no yaml here: print nothing, caller copes
    raise SystemExit(0)

with open(sys.argv[1], encoding='utf-8') as handle:
    data = yaml.safe_load(handle) or {}
entry = data.get(sys.argv[2]) or {}
value = entry.get('default', '') if isinstance(entry, dict) else ''
print('' if value is None else value)
PY
}

expand_home() { printf '%s' "${1/#\~/$HOME}"; }

first_existing() {  # print the first argument that exists on disk
  local candidate
  for candidate in "$@"; do
    [ -n "$candidate" ] || continue
    candidate="$(expand_home "$candidate")"
    if [ -e "$candidate" ]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done
  return 1
}

# ---------------------------------------------------------------------------
# Models. config/models.yaml is the only place a model name is defined; these
# helpers validate a name and read the port llama-server will bind, and nothing
# else. model_path/extra_args stay the launch file's business -- duplicating them
# here would be a second source of truth for the same server.
# ---------------------------------------------------------------------------
model_names() {  # one models.yaml key per line; the grep fallback needs no yaml
  local names
  names="$(python3 - "$models_file" <<'PY' 2>/dev/null || true
import sys

try:
    import yaml
except Exception:  # noqa: BLE001 -- no yaml here: the caller's grep copes
    raise SystemExit(0)

with open(sys.argv[1], encoding='utf-8') as handle:
    data = yaml.safe_load(handle) or {}
for name in data:
    print(name)
PY
)"
  if [ -z "$names" ]; then
    # Comment lines start with '#', so this only ever matches real top-level keys.
    names="$(grep -oE '^[A-Za-z0-9_.-]+:' "$models_file" 2>/dev/null | tr -d ':' || true)"
  fi
  printf '%s\n' "$names" | sed '/^$/d'
}

model_entry() {  # $1 = model name, $2 = a key in its models.yaml block
  python3 - "$models_file" "$1" "$2" <<'PY' 2>/dev/null || true
import sys

try:
    import yaml
except Exception:  # noqa: BLE001 -- no yaml here: print nothing, caller copes
    raise SystemExit(0)

with open(sys.argv[1], encoding='utf-8') as handle:
    data = yaml.safe_load(handle) or {}
entry = data.get(sys.argv[2]) or {}
value = entry.get(sys.argv[3], '') if isinstance(entry, dict) else ''
print('' if value is None else value)
PY
}

require_model() {  # $1 = model name -> dies (exit 2) when models.yaml disagrees
  local name="$1" known
  [ -n "$name" ] || die 'no model name given'
  known="$(model_names)"
  if [ -z "$known" ]; then
    # No readable models.yaml: llm.launch.py validates model:= itself and its
    # error names every available model, so do not invent a second gate here.
    return 0
  fi
  if printf '%s\n' "$known" | grep -qxF -- "$name"; then return 0; fi
  die "unknown model '$name' -- $models_file has: $(printf '%s' "$known" | tr '\n' ' ')"
}

props_model_path() {  # /props model_path of whatever answers on :$port, or nothing
  python3 - "$port" <<'PY' 2>/dev/null || true
import json
import sys
import urllib.request

try:
    with urllib.request.urlopen(
            f'http://127.0.0.1:{sys.argv[1]}/props', timeout=3) as response:
        data = json.load(response)
except Exception:  # noqa: BLE001 -- no server, old build, no /props: all empty
    raise SystemExit(0)
path = data.get('model_path') if isinstance(data, dict) else None
print(path if isinstance(path, str) else '')
PY
}

detect_model() {  # registered name of the running server, else the gguf basename
  local path name
  path="$(props_model_path)"
  [ -n "$path" ] || return 1
  name="$(python3 - "$models_file" "$path" <<'PY' 2>/dev/null || true
import os
import sys

try:
    import yaml
except Exception:  # noqa: BLE001
    raise SystemExit(0)

with open(sys.argv[1], encoding='utf-8') as handle:
    data = yaml.safe_load(handle) or {}
wanted = os.path.expanduser(sys.argv[2])
for name, entry in data.items():
    if not isinstance(entry, dict):
        continue
    if os.path.expanduser(str(entry.get('model_path', ''))) == wanted:
        print(name)
        break
PY
)"
  if [ -z "$name" ]; then name="$(basename "$path")"; fi
  printf '%s\n' "$name"
}

record_model() {  # $1 = name -> analytics_runs/_server/current_model
  mkdir -p "$server_dir"
  printf '%s\n' "$1" > "$model_file"
}

# Binary and cwd are two distinct values on purpose (llm/memory.md): cwd is the
# llama.cpp project root -- three levels up from .../build/bin/llama-server -- not
# the binary's own directory. Order: explicit env -> the stack_params default if
# it exists on this machine -> this desktop's tree.
launch_server_path() {
  first_existing "${LLAMA_SERVER_BIN:-}" \
      "$(stack_param llama_server_path)" \
      "$HOME/projects/llm/llama.cpp/build/bin/llama-server"
}

launch_server_cwd() {
  local bin cwd
  if [ -n "${LLAMA_SERVER_CWD:-}" ]; then
    expand_home "$LLAMA_SERVER_CWD"
    return 0
  fi
  cwd="$(stack_param llama_server_cwd)"
  if first_existing "$cwd" >/dev/null; then
    expand_home "$cwd"
    return 0
  fi
  bin="$(launch_server_path)" || return 1
  dirname "$(dirname "$(dirname "$bin")")"
}

# ---------------------------------------------------------------------------
# Labels and run folders (planner.md §2.3, .clinerules ANALYTICS RUN PROTOCOL)
# ---------------------------------------------------------------------------
sanitise_label() {  # lowercase, one '-' per run of junk, 48 chars, no edges
  local text
  text="$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]' \
    | sed -e 's/[^a-z0-9._-]\+/-/g' -e 's/^[._-]\+//' -e 's/[._-]\+$//')"
  text="${text:0:48}"
  printf '%s\n' "${text%[._-]}"
}

dated_label() {  # $1 = name -> YYYY-MM-DD-name; an already-dated name passes through
  local name="$1"
  if [[ "$name" =~ ^[0-9]{4}-[0-9]{2}-[0-9]{2}(-|$) ]]; then
    printf '%s\n' "$name"
  else
    printf '%s-%s\n' "$(date +%Y-%m-%d)" "$name"
  fi
}

new_run_dir() {  # $1 = dated label, $2 = root -> first free <root>/<label>[-N]
  local base="$1" root="${2:-$runs_dir}" candidate n=2
  candidate="$root/$1"
  while [ -e "$candidate" ]; do
    candidate="$root/${base}-${n}"
    n=$((n + 1))
  done
  printf '%s\n' "$candidate"
}

# RUN_DIR is a global, not an echoed string: a function's `die` inside $( )
# would only kill the subshell and the caller would carry on with an empty path.
RUN_DIR=""

make_run_dir() {  # $1 = label (name or already dated), $2 = root; sets RUN_DIR
  local name root="${2:-$runs_dir}"
  name="$(sanitise_label "$1")"
  [ -n "$name" ] || die "--label '$1' sanitises to nothing usable"
  RUN_DIR="$(new_run_dir "$(dated_label "$name")" "$root")"
  mkdir -p "$RUN_DIR"
}

# ---------------------------------------------------------------------------
# Calling the tool, and the preflight that stops a run before it wastes yours
# ---------------------------------------------------------------------------
have_console_script() {
  [ -x "$ws/install/llm/lib/llm/llm_analytics" ] && command -v ros2 >/dev/null 2>&1
}

run_tool() {  # every argument goes to llm_analytics unchanged
  if have_console_script; then
    ros2 run llm llm_analytics "$@"
  else
    say 'note          console script missing: running the source tree directly'
    say '              (scripts/colcon_build.sh build --packages-select llm restores it)'
    # f1tenth_behavior on PYTHONPATH because --airlock imports the translator
    # (llm/memory.md drift 8): a bare source tree has neither on sys.path.
    PYTHONPATH="$package_dir:$behavior_dir${PYTHONPATH:+:$PYTHONPATH}" \
      python3 -m llm.llm_analytics "$@"
  fi
}

ensure_server() {
  server_up && return 0
  local bin cwd
  {
    printf 'ERRORE: nothing answering on http://127.0.0.1:%s/health\n\n' "$port"
    printf '  The proxy only records: with no upstream server a run can produce\n'
    printf '  refused connections and nothing else, so this stops here instead.\n\n'
    printf '  start one :  scripts/analytics.sh up\n'
    if bin="$(launch_server_path)"; then
      cwd="$(launch_server_cwd || true)"
      printf '  by hand   :  ros2 launch llm llm.launch.py \\\n'
      printf '                 llama_server_path:=%s \\\n' "$bin"
      printf '                 llama_server_cwd:=%s\n' "$cwd"
    else
      printf '  by hand   :  ros2 launch llm llm.launch.py\n'
      printf '               (no llama-server binary found -- set LLAMA_SERVER_BIN,\n'
      printf '                or fix llama_server_path in stack_params.yaml)\n'
    fi
    printf '\n  check it  :  scripts/analytics.sh probe\n'
  } >&2
  exit 3
}

quote_argv() {
  local arg out=''
  for arg in "$@"; do
    out+="$(printf '%q' "$arg") "
  done
  printf '%s' "${out% }"
}

banner_run() {  # $1 = run dir; remaining args = the tool arguments being used
  local run="$1"
  shift
  say "run           $(basename "$run")"
  say "dir           $run"
  if have_console_script; then
    say "equivalent    ros2 run llm llm_analytics $(quote_argv "$@")"
  else
    say "equivalent    python3 -m llm.llm_analytics $(quote_argv "$@")"
  fi
  say ''
}

finish_run() {  # $1 = run dir, $2 = the tool's exit code
  local run="$1" rc="$2"
  say ''
  say "report        $run/report.md"
  say "next          scripts/analytics.sh replay $(basename "$run")"
  return "$rc"
}

# The flags every measuring mode shares. --out/--events are never optional here:
# the tool's own defaults land in the current directory and truncate on purpose
# (planner.md §2.4), which is exactly the trap this script removes.
common_args() {
  local run="$1"
  printf '%s\n' --out "$run/report.json" --events "$run/events.jsonl"
  if [ "$port" != "8083" ]; then
    printf '%s\n' --upstream "http://127.0.0.1:${port}"
  fi
  if [ -n "$opt_timeout" ]; then
    printf '%s\n' --timeout "$opt_timeout"
  fi
  if [ "$opt_verbose" -eq 1 ]; then
    printf '%s\n' --verbose
  fi
}

# ---------------------------------------------------------------------------
# help
# ---------------------------------------------------------------------------
usage() {
  cat <<'EOF'
scripts/analytics.sh — llm_analytics without the ritual

SERVER  (one llama-server, shared by every run; started once, killed once)
  up [--wait S] [--model NAME]
                         start llama-server, wait for /health (default 120 s),
                         log in analytics_runs/_server/llama-server.log.
                         --model launches model:=NAME from config/models.yaml and
                         keeps the active name in _server/current_model
  status                 /health, active model, launcher pid, rss, log path
  down [--force]         stop the server this script started (SIGINT, then
                         SIGTERM, then --force for SIGKILL)
  probe                  what this llama-server build actually exposes
                         (/metrics 501 is expected: llm.launch.py passes no
                         --metrics)

MEASURE  (each run gets analytics_runs/YYYY-MM-DD-<label>/ holding report.json,
          report.md, events.jsonl and commands.txt -- nothing lands in the repo)
  one "command"          one prompt, no node, one generation      (--exec-file)
  sweep [FILE]           every line of FILE (default test/commands.sample.txt)
  compare A B [FILE]     two models, one command file, back to back: up A ->
                         sweep -> down, up B -> sweep -> down. Lives in
                         analytics_runs/sequential/<date>-A-vs-B/ with A/ and B/
                         (their own report.json, report.md, events, commands,
                         llama-server.log) plus one comparison.md
  node "command"         the real node, dry run, its RESULT line   (--exec)
  live "command"         the real node and the mission LOADS       (--exec --live)
  idle                   recorder for a node you start by hand: it prints the
                         node command, you Ctrl-C and the report is written

READ
  replay <label|path>    re-analyse a capture: no server, no ROS, no llama.cpp
                         (writes that folder's replayed.json). `--last` picks the
                         newest run; a bare label matches the newest -NAME / -NAME-N
  runs [--sequential]    list run folders, newest first (sequential: comparisons)
  last [--sequential]    newest run folder plus its verdict table

OPTIONS  (any mode, before or after the argument)
  --label NAME           run name. The date is prepended for you; rerunning the
                         same NAME the same day gives YYYY-MM-DD-NAME-2, -3 ...
  --model NAME           up/compare: launch model:=NAME from config/models.yaml
                         (the model's port is adopted unless --port is explicit)
  --sequential           runs/last: list the sequential comparisons instead
  --last                 replay: use the newest run instead of naming one
  --no-airlock           do not run the output through translate()
  --no-go-to             measure the go_to-disabled prompt (swaps the file)
  --port N               llama-server port (default 8083, or $LLAMA_PORT)
  --timeout S            per-command timeout (default 180)
  --wait S               up: how long to wait for /health (default 120)
  --force                down: escalate to SIGKILL
  --simple-discovery     unset ROS_DISCOVERY_SERVER for standalone node runs
  --verbose              pass --verbose through to llm_analytics
  -h, --help             this text

WHAT IT FIXES SO YOU DO NOT HAVE TO REMEMBER IT  (planner.md §2.4)
  --events into a missing folder aborts the run   -> the folder is made first
  --out/--events default to the current directory -> always pointed into it
  a rerun truncates the previous capture          -> -2, -3 suffix instead
  a server that is not up costs 40 x 502          -> /health first, exit 3
  stack_params.yaml's llama_server_path is the Jetson's -> autodetected here
  two models would share RAM and compete          -> compare is sequential,
  yesterday's comparison would be overwritten     -> one folder per comparison
  a foreign server on :8083 has no pid to signal  -> compare exits 3, kills nothing

EXIT CODES  0 ok · 1 an exchange failed (the report still exists) · 2 usage ·
            3 preflight (no server, no binary, nothing to replay, or a server on
            the port that compare did not start and will not kill)
The long form is unchanged and still the reference:
  ros2 run llm llm_analytics --help
EOF
}

# ---------------------------------------------------------------------------
# Server lifecycle. up/down only ever touch a server this script started; a
# server you launched by hand is reported and left alone (idle is the mode for
# that case: it records whatever is already running).
# ---------------------------------------------------------------------------
server_pid_by_port() {  # llama-server on $port, whoever started it
  local pid
  pid="$({ pgrep -f "llama-server.*(--port[ =]|--port=)${port}(\$| )" || true; } | head -n 1)"
  [ -n "$pid" ] && printf '%s\n' "$pid"
  return 0
}

# True only for a launcher pid this script wrote and can still signal. `compare`
# leans on it: it restarts the server once per model, so it may only take over a
# server whose owner it can stop (a foreign one exits 3 instead).
server_is_ours() {
  local pid
  [ -f "$pid_file" ] || return 1
  pid="$(tr -d '[:space:]' < "$pid_file")"
  if [ -z "$pid" ]; then return 1; fi
  kill -0 "$pid" 2>/dev/null
}

rss_kb() {
  awk '/^VmRSS:/ {print $2}' "/proc/$1/status" 2>/dev/null || true
}

cmd_up() {
  local bin cwd pid waited=0 dead=0 model_port detected=''
  local -a launch_args=()

  if [ -n "$opt_model" ]; then
    require_model "$opt_model"
    model_port="$(model_entry "$opt_model" port)"
    if [ -n "$model_port" ]; then
      if [ "$opt_port_set" -eq 0 ]; then
        # models.yaml wins: llm.launch.py makes the server bind that port, so a
        # client dialling anything else would get 502 from a healthy server.
        port="$model_port"
      elif [ "$port" != "$model_port" ]; then
        say "note          $opt_model binds :$model_port (config/models.yaml), but"
        say "              --port said $port: the client would dial the wrong port."
      fi
    fi
  fi

  if server_up; then
    say "already up    http://127.0.0.1:${port}/health  ($(health_body))"
    detected="$(detect_model || true)"
    if [ -n "$opt_model" ] && [ -n "$detected" ] && [ "$detected" != "$opt_model" ]; then
      say "note          that server is '$detected', not '$opt_model'."
      say "              scripts/analytics.sh down, then up --model $opt_model."
    elif [ -n "$detected" ]; then
      record_model "$detected"
      say "  model       $detected"
    fi
    return 0
  fi
  command -v ros2 >/dev/null 2>&1 || die \
    'ros2 not on PATH -- build first: scripts/colcon_build.sh build --packages-select llm'
  bin="$(launch_server_path)" || die \
    'no llama-server binary: export LLAMA_SERVER_BIN=<path>, or fix llama_server_path in stack_params.yaml'
  cwd="$(launch_server_cwd)" || die \
    'cannot derive llama_server_cwd: export LLAMA_SERVER_CWD=<llama.cpp root>'

  if [ "$port" != "8083" ]; then
    say "note          llm.launch.py reads the port from config/models.yaml (8083);"
    say "              --port only changes what the client dials, not the server."
  fi

  mkdir -p "$server_dir"
  # The log is evidence too: rotate instead of truncating a previous capture.
  if [ -f "$log_file" ]; then mv -f "$log_file" "$log_file.prev"; fi
  : > "$log_file"

  launch_args=("llama_server_path:=$bin" "llama_server_cwd:=$cwd")
  if [ -n "$opt_model" ]; then launch_args+=("model:=$opt_model"); fi

  say "llama-server  $bin"
  say "  cwd         $cwd"
  if [ -n "$opt_model" ]; then
    say "  model       $opt_model (model:=$opt_model, resolved by config/models.yaml)"
  fi
  say "  launcher    ros2 launch llm llm.launch.py (start_server default is true)"
  say "  log         $log_file"

  # setsid: survive this terminal. The launcher itself is what `down` signals;
  # it forwards SIGINT to the processes it started, which is how Ctrl-C works.
  if command -v setsid >/dev/null 2>&1; then
    setsid ros2 launch llm llm.launch.py "${launch_args[@]}" \
        >"$log_file" 2>&1 &
  else
    ros2 launch llm llm.launch.py "${launch_args[@]}" \
        >"$log_file" 2>&1 &
  fi
  pid=$!
  printf '%s\n' "$pid" > "$pid_file"

  printf '  waiting for /health'
  while [ "$waited" -lt "$opt_wait" ]; do
    if server_up; then
      printf '\n'
      say "up            http://127.0.0.1:${port}/health  ($(health_body))"
      say "  launcher pid $pid   (scripts/analytics.sh down stops it)"
      detected="$(detect_model || true)"
      record_model "${detected:-${opt_model:-auto}}"
      say "  model       ${detected:-${opt_model:-auto}} (analytics_runs/_server/current_model)"
      return 0
    fi
    if kill -0 "$pid" 2>/dev/null; then dead=0; else dead=$((dead + 1)); fi
    if [ "$dead" -ge 3 ]; then
      printf '\n'
      printf 'ERRORE: llama-server exited before answering. Last lines of %s:\n' \
        "$log_file" >&2
      tail -n 20 "$log_file" >&2 || true
      rm -f "$pid_file"
      return 1
    fi
    sleep 2
    waited=$((waited + 2))
    printf '.'
  done
  printf '\n'
  printf 'ERRORE: no /health after %ss. Last lines of %s:\n' "$opt_wait" "$log_file" >&2
  tail -n 20 "$log_file" >&2 || true
  printf '  the launcher is still running (pid %s): scripts/analytics.sh down stops it\n' \
    "$pid" >&2
  return 1
}

cmd_status() {
  local pid="" srv ows model=""
  if server_up; then
    say "health        $(health_body)"
  else
    say "health        no answer on http://127.0.0.1:${port}/health"
  fi

  if [ -f "$model_file" ]; then model="$(tr -d '[:space:]' < "$model_file")"; fi
  if [ -n "$model" ]; then
    if server_up; then
      say "model         $model   (analytics_runs/_server/current_model)"
    else
      say "model         $model   (last recorded; the server is down)"
    fi
  fi

  if [ -f "$pid_file" ]; then pid="$(tr -d '[:space:]' < "$pid_file")"; fi
  if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
    say "launcher      pid $pid (started by this script)"
  elif [ -n "$pid" ]; then
    say "launcher      pid $pid is gone (stale $pid_file)"
  else
    say "launcher      none recorded (this script did not start it)"
  fi

  srv="$(server_pid_by_port)"
  if [ -n "$srv" ]; then
    ows="$(tr '\0' ' ' < "/proc/$srv/cmdline" 2>/dev/null | cut -c1-96)"
    say "llama-server  pid $srv   rss $(rss_kb "$srv") kB   $ows"
  else
    say "llama-server  no process matching --port $port"
  fi
  say "log           $log_file"
  if server_up; then return 0; else return 1; fi
}

cmd_down() {
  local pid="" srv="" waited=0 stopped=0
  if [ -f "$pid_file" ]; then pid="$(tr -d '[:space:]' < "$pid_file")"; fi
  if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
    say "stopping      ros2 launch pid $pid (SIGINT)"
    kill -INT "$pid" 2>/dev/null || true
  elif ! server_up; then
    say "down          nothing was running on :$port"
    rm -f "$pid_file"
    return 0
  else
    say "note          no live launcher pid: this server was started elsewhere."
    say "              stopping the llama-server on :$port because you asked for it."
  fi

  srv="$(server_pid_by_port)"
  if [ -n "$srv" ] && kill -0 "$srv" 2>/dev/null; then
    say "stopping      llama-server pid $srv (SIGINT)"
    kill -INT "$srv" 2>/dev/null || true
  fi

  while [ "$waited" -lt 25 ]; do
    if ! server_up; then
      stopped=1
      break
    fi
    sleep 1
    waited=$((waited + 1))
  done

  if [ "$stopped" -eq 0 ]; then
    say 'still up after 25 s: SIGTERM'
    if [ -n "$pid" ]; then kill -TERM "$pid" 2>/dev/null || true; fi
    if [ -n "$srv" ]; then kill -TERM "$srv" 2>/dev/null || true; fi
    waited=0
    while [ "$waited" -lt 10 ]; do
      if ! server_up; then stopped=1; break; fi
      sleep 1
      waited=$((waited + 1))
    done
  fi

  if [ "$stopped" -eq 0 ] && [ "$opt_force" -eq 1 ]; then
    say 'SIGKILL'
    if [ -n "$srv" ]; then kill -KILL "$srv" 2>/dev/null || true; fi
    if [ -n "$pid" ]; then kill -KILL "$pid" 2>/dev/null || true; fi
    sleep 2
  fi

  if server_up; then
    printf 'ERRORE: llama-server still answering on :%s (try --force)\n' "$port" >&2
    return 1
  fi
  rm -f "$pid_file"
  say "down          :$port no longer answers"
  return 0
}

cmd_probe() {
  local rc=0 was_down=0
  if ! server_up; then
    was_down=1
    say "note          nothing on :$port yet: probe will report every endpoint"
    say "              as unreachable. scripts/analytics.sh up starts one."
  fi
  run_tool --probe --upstream "http://127.0.0.1:${port}" || rc=$?
  say ''
  say 'probe         /metrics 501 is the expected answer: llm.launch.py passes'
  say '              no --metrics, so the endpoint exists but is disabled'
  # Exit 3 is this script's "no server" code (see the header), so a health
  # preflight can be a one-liner: scripts/analytics.sh probe || exit 3.
  if [ "$was_down" -eq 1 ]; then return 3; fi
  return "$rc"
}

# ---------------------------------------------------------------------------
# Measuring modes. All of them: preflight, make the folder, banner, run, report.
# ---------------------------------------------------------------------------
cmd_one() {
  local command="${positionals[0]:-}"
  local run args=() common=() rc=0
  [ -n "$command" ] || die \
    'one needs a command, e.g. scripts/analytics.sh one "vai dritto e gira a sinistra alla parete"'
  ensure_server
  make_run_dir "${opt_label:-one-$(sanitise_label "$command")}"
  run="$RUN_DIR"
  # The input lives beside its capture (.clinerules rule 7): --exec-file reads
  # this very file, and the tool's own commands.txt snapshot rewrites it
  # byte-identically, so the run stays reproducible.
  printf '%s\n' "$command" > "$run/commands.txt"

  args=(--exec-file "$run/commands.txt")
  if [ "$opt_airlock" -eq 1 ]; then args+=(--airlock); fi
  if [ "$opt_go_to" -eq 0 ]; then args+=(--no-go-to); fi
  mapfile -t common < <(common_args "$run")
  args+=("${common[@]}")

  banner_run "$run" "${args[@]}"
  run_tool "${args[@]}" || rc=$?
  finish_run "$run" "$rc"
}

cmd_sweep() {
  local file="${positionals[0]:-$sample_file}"
  local run args=() common=() rc=0 total
  [ -f "$file" ] || die "no such command file: $file"
  file="$(cd "$(dirname "$file")" && pwd)/$(basename "$file")"
  total="$(grep -cvE '^[[:space:]]*(#|$)' "$file" || true)"
  [ "${total:-0}" -gt 0 ] || die "$file has no commands (comments and blank lines only)"
  ensure_server
  make_run_dir "${opt_label:-sweep-$(sanitise_label "$(basename "$file" .txt)")}"
  run="$RUN_DIR"

  args=(--exec-file "$file")
  if [ "$opt_airlock" -eq 1 ]; then args+=(--airlock); fi
  if [ "$opt_go_to" -eq 0 ]; then args+=(--no-go-to); fi
  mapfile -t common < <(common_args "$run")
  args+=("${common[@]}")

  banner_run "$run" "${args[@]}"
  say "commands      $total, one after another (Ctrl-C: the partial report survives)"
  say ''
  run_tool "${args[@]}" || rc=$?
  finish_run "$run" "$rc"
}

cmd_exec() {  # $1 = 0 dry run (node), 1 a real load (live)
  local live="$1"
  local verb='node'
  local command="${positionals[0]:-}"
  local run args=() common=() rc=0 answer=''
  if [ "$live" -eq 1 ]; then verb='live'; fi
  [ -n "$command" ] || die \
    "$verb needs a command, e.g. scripts/analytics.sh $verb \"gira a sinistra dopo la curva\""
  # --exec shells out to `ros2 run llm llm_planner_node`, so the graph itself has
  # to exist: this fails at preflight rather than halfway through a run.
  command -v ros2 >/dev/null 2>&1 || die \
    "--exec runs the real node: build and source the workspace first (scripts/colcon_build.sh build)"
  ensure_server

  if [ "$live" -eq 1 ]; then
    say 'live:     llm_planner_node will LOAD AND START the mission (the robot moves).'
    printf "type 'yes' to continue: "
    read -r answer || answer=''
    [ "$answer" = 'yes' ] || die 'aborted: nothing was run'
  fi

  make_run_dir "${opt_label:-$verb-$(sanitise_label "$command")}"
  run="$RUN_DIR"
  printf '%s\n' "$command" > "$run/commands.txt"

  args=(--exec "$command")
  if [ "$live" -eq 1 ]; then args+=(--live); fi
  if [ "$opt_airlock" -eq 1 ]; then args+=(--airlock); fi
  if [ "$opt_go_to" -eq 0 ]; then args+=(--no-go-to); fi
  mapfile -t common < <(common_args "$run")
  args+=("${common[@]}")

  banner_run "$run" "${args[@]}"
  run_tool "${args[@]}" || rc=$?
  finish_run "$run" "$rc"
}

cmd_idle() {
  local run args=() common=() rc=0
  ensure_server
  make_run_dir "${opt_label:-idle}"
  run="$RUN_DIR"
  mapfile -t common < <(common_args "$run")
  args=("${common[@]}")

  banner_run "$run" "${args[@]}"
  say 'recorder      it prints the node command to run in a second terminal.'
  say '              Drive the scenario there, then Ctrl-C here: the report and'
  say '              the events are written on the way out.'
  say ''
  run_tool "${args[@]}" || rc=$?
  finish_run "$run" "$rc"
}

# ---------------------------------------------------------------------------
# compare: two models, one command file, one side-by-side report. Strictly
# sequential -- model B starts only after model A's server is down and gone --
# because two llama-servers sharing RAM would make every latency and tok/s in
# the comparison a measurement of contention, not of the model.
# ---------------------------------------------------------------------------
COMPARE_DIR=""

compare_stop_server() {  # stop the server this script started; 1 if it survives
  local waited=0
  if ! cmd_down; then
    # A comparison cannot proceed with two servers on one port, so this is the
    # one place that escalates to SIGKILL by itself (down --force by default).
    say "note          SIGINT was not enough for :$port: escalating to SIGKILL"
    opt_force=1
    if ! cmd_down; then return 1; fi
  fi
  while [ -n "$(server_pid_by_port)" ]; do
    [ "$waited" -ge 10 ] && return 1
    sleep 1
    waited=$((waited + 1))
  done
  return 0
}

compare_one() {  # $1 = model name, $2 = its folder; returns the tool's exit code
  local model="$1" mdir="$2" rc=0 detected=''
  local args=() common=()
  opt_model="$model"

  say "--- $model ------------------------------------------------------------"
  if ! cmd_up; then
    say "up failed      $model never answered: stopping what did start"
    compare_stop_server || true
    return 3
  fi
  detected="$(detect_model || true)"
  if [ -n "$detected" ] && [ "$detected" != "$model" ]; then
    if printf '%s\n' "$(model_names)" | grep -qxF -- "$detected"; then
      printf 'ERRORE: :%s answers as %s, not %s -- refusing to measure the wrong model\n' \
        "$port" "'$detected'" "'$model'" >&2
      compare_stop_server || true
      return 3
    fi
    # /props reported a path no models.yaml entry names (a relative -m, a model
    # loaded by hand): it cannot be mapped to a registered name, so say so and
    # trust the launch -- cmd_up already passed model:=$model.
    say "note          :$port reports model_path '$detected', which is not a"
    say "              config/models.yaml name: trusting model:=$model"
  fi

  # The shared copy, never the original: the file cannot change between the two
  # runs, and each model's own commands.txt is then compared to this one.
  args=(--exec-file "$COMPARE_DIR/commands.txt")
  if [ "$opt_airlock" -eq 1 ]; then args+=(--airlock); fi
  if [ "$opt_go_to" -eq 0 ]; then args+=(--no-go-to); fi
  mapfile -t common < <(common_args "$mdir")
  args+=("${common[@]}")

  banner_run "$mdir" "${args[@]}"
  run_tool "${args[@]}" || rc=$?
  if [ "$rc" -eq 0 ]; then
    say "status        $model captured cleanly"
  else
    say "status        $model exited $rc (its report still exists)"
  fi
  say ''

  # up rotates the shared log at the next launch, so keep this model's copy now.
  if [ -f "$log_file" ]; then cp -f "$log_file" "$mdir/llama-server.log"; fi
  if ! compare_stop_server; then
    printf 'ERRORE: :%s still answers after down -- not starting the next model\n' \
      "$port" >&2
    return 3
  fi
  return "$rc"
}

cmd_compare() {
  local model_a="${positionals[0]:-}" model_b="${positionals[1]:-}"
  local file="${positionals[2]:-$sample_file}" file_given=0
  local name mdir_a mdir_b rc_a=0 rc_b=0 rc=0 total airlock_state='off'
  local go_to_state='disabled' model model_port

  if [ -n "${positionals[2]:-}" ]; then file_given=1; fi
  [ -n "$model_a" ] && [ -n "$model_b" ] || die \
    'compare needs two models: scripts/analytics.sh compare <modelA> <modelB> [FILE]'
  [ "$model_a" != "$model_b" ] || die \
    "compare wants two different models -- 'sweep' twice is the same-model noise test"
  require_model "$model_a"
  require_model "$model_b"
  if [ "$opt_port_set" -eq 1 ]; then
    # llm.launch.py makes the server bind the port in models.yaml, so a forced
    # --port that disagrees would measure refused connections, not a model.
    for model in "$model_a" "$model_b"; do
      model_port="$(model_entry "$model" port)"
      if [ -n "$model_port" ] && [ "$model_port" != "$port" ]; then
        die "$model binds :$model_port (config/models.yaml) but --port is $port -- drop --port"
      fi
    done
  fi
  [ -f "$file" ] || die "no such command file: $file"
  file="$(cd "$(dirname "$file")" && pwd)/$(basename "$file")"

  if server_up; then
    if server_is_ours; then
      say "preflight     stopping the server this script started (:${port} is reused)"
      compare_stop_server || die \
        "could not free :$port -- scripts/analytics.sh down --force, then compare again"
    else
      {
        printf 'ERRORE: a llama-server is already answering on :%s and this script did not start it\n\n' "$port"
        printf '  compare restarts the server once per model, so it will not take over a\n'
        printf '  process it cannot stop cleanly (no live pid in %s).\n\n' "$pid_file"
        printf '  stop it yourself, then rerun:\n'
        printf '    scripts/analytics.sh down --force     # if this script started it before\n'
        printf '    kill -INT <llama-server pid>          # otherwise (status shows the pid)\n'
      } >&2
      exit 3
    fi
  fi

  if [ -n "$opt_label" ]; then
    name="$opt_label"
  elif [ "$file_given" -eq 1 ]; then
    name="$model_a-vs-$model_b-$(sanitise_label "$(basename "$file" .txt)")"
  else
    name="$model_a-vs-$model_b"
  fi
  mkdir -p "$seq_dir"
  make_run_dir "$name" "$seq_dir"
  COMPARE_DIR="$RUN_DIR"
  mdir_a="$COMPARE_DIR/$(sanitise_label "$model_a")"
  mdir_b="$COMPARE_DIR/$(sanitise_label "$model_b")"
  [ "$mdir_a" != "$mdir_b" ] || die \
    "'$model_a' and '$model_b' sanitise to the same folder -- cannot keep them apart"
  mkdir -p "$mdir_a" "$mdir_b"
  cp -f "$file" "$COMPARE_DIR/commands.txt"
  total="$(grep -cvE '^[[:space:]]*(#|$)' "$COMPARE_DIR/commands.txt" || true)"
  [ "${total:-0}" -gt 0 ] || die "$file has no commands (comments and blank lines only)"
  if [ "$opt_airlock" -eq 1 ]; then airlock_state='on'; fi
  if [ "$opt_go_to" -eq 1 ]; then go_to_state='enabled'; fi

  say "compare       $(basename "$COMPARE_DIR")"
  say "dir           $COMPARE_DIR"
  say "models        $model_a -> $(basename "$mdir_a")"
  say "              $model_b -> $(basename "$mdir_b")"
  say "commands      $total from $file (one copy, both models)"
  say ''

  compare_one "$model_a" "$mdir_a" || rc_a=$?
  if [ "$rc_a" -ge 2 ]; then
    say ''
    say "aborted       $model_a produced no comparable capture (exit $rc_a)"
    say "partial       $COMPARE_DIR"
    return 3
  fi
  compare_one "$model_b" "$mdir_b" || rc_b=$?
  if [ "$rc_b" -ge 2 ]; then
    say ''
    say "aborted       $model_b produced no comparable capture (exit $rc_b)"
    say "partial       $COMPARE_DIR"
    return 3
  fi

  say '--- comparison --------------------------------------------------------'
  python3 "$ws/scripts/compare_reports.py" \
    --a "$mdir_a" --name-a "$model_a" \
    --b "$mdir_b" --name-b "$model_b" \
    --out "$COMPARE_DIR/comparison.md" \
    --commands "$COMPARE_DIR/commands.txt" \
    --airlock "$airlock_state" --go-to "$go_to_state" \
    --exit-a "$rc_a" --exit-b "$rc_b" || rc=2
  if [ "$rc" -eq 2 ]; then
    printf 'ERRORE: scripts/compare_reports.py could not write %s/comparison.md\n' \
      "$COMPARE_DIR" >&2
    return 2
  fi

  say ''
  say "comparison    $COMPARE_DIR/comparison.md"
  say "reports       $mdir_a/report.md"
  say "              $mdir_b/report.md"
  say "next          scripts/analytics.sh last --sequential"
  # A failed exchange is a capture with holes, not an aborted comparison: the
  # report exists and says so, so 1 keeps that distinction (see EXIT CODES).
  if [ "$rc_a" -ne 0 ] || [ "$rc_b" -ne 0 ]; then return 1; fi
  return 0
}

# ---------------------------------------------------------------------------
# Reading what was captured. replay is the only mode that needs no server, no
# ROS and no llama.cpp: it is a pure re-render of a saved events.jsonl.
# ---------------------------------------------------------------------------
run_dirs() {  # folder names under $1 (default runs_dir), newest first
  # _server is bookkeeping and sequential/ holds comparisons: neither is a run.
  local root="${1:-$runs_dir}"
  find "$root" -mindepth 1 -maxdepth 1 -type d ! -name '_*' ! -name 'sequential' \
    -printf '%T@ %f\n' 2>/dev/null | sort -rn | cut -d' ' -f2- || true
}

seq_run_dirs() {  # comparison folders, newest first
  [ -d "$seq_dir" ] || return 0
  find "$seq_dir" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %f\n' 2>/dev/null \
    | sort -rn | cut -d' ' -f2- || true
}

last_run_dir() {  # sets RUN_DIR; returns 1 when there is nothing to show
  local name
  # Dated folders first (the convention), legacy names only as a fallback: a
  # name like NEXT-02-... sorts after any 2026-... and would hide last night's
  # actual measurement.
  name="$(run_dirs | grep -E '^[0-9]{4}-[0-9]{2}-[0-9]{2}-' | head -n 1 || true)"
  if [ -z "$name" ]; then name="$(run_dirs | head -n 1)"; fi
  [ -n "$name" ] || return 1
  RUN_DIR="$runs_dir/$name"
}

resolve_run() {  # path | exact folder name | bare label (newest wins) | --last
  local arg="${1:-}" name found
  [ -n "$arg" ] || die \
    'replay needs a run: a label, a folder, or --last (scripts/analytics.sh runs lists them)'
  if [ "$arg" = "--last" ]; then
    last_run_dir || die "no runs yet in $runs_dir"
    return 0
  fi
  if [ -d "$arg" ]; then
    RUN_DIR="$(cd "$arg" && pwd)"
    return 0
  fi
  if [ -d "$runs_dir/$arg" ]; then
    RUN_DIR="$runs_dir/$arg"
    return 0
  fi
  name="$(sanitise_label "$arg")"
  [ -n "$name" ] || die "'$arg' is not a usable label"
  found="$({ find "$runs_dir" -mindepth 1 -maxdepth 1 -type d \
    \( -name "*-${name}" -o -name "*-${name}-[0-9]*" \) \
    -printf '%f\n' 2>/dev/null || true; } | sort -r | head -n 1)"
  [ -n "$found" ] || die "no run matching '$arg' in $runs_dir (scripts/analytics.sh runs)"
  say "match         $found (newest match for '$arg')" >&2
  RUN_DIR="$runs_dir/$found"
}

cmd_replay() {
  local run events out rc=0
  if [ "$opt_last" -eq 1 ]; then
    resolve_run --last
  else
    resolve_run "${positionals[0]:-}"
  fi
  run="$RUN_DIR"
  events="$run/events.jsonl"
  [ -f "$events" ] || die "no events.jsonl in $run -- that run captured nothing"
  out="$run/replayed.json"
  say "replaying     $(basename "$run")"
  say "  from        $events"
  say "  to          $out"
  if [ -f "$out" ]; then
    say '  note        replacing the previous replayed.json (the capture itself is untouched)'
  fi
  say ''
  run_tool --replay "$events" --out "$out" || rc=$?
  if [ "$rc" -eq 0 ]; then
    say ''
    say "markdown      $run/replayed.md"
  fi
  return "$rc"
}

cmd_runs() {
  local name count report models
  if [ "$opt_sequential" -eq 1 ]; then
    if [ ! -d "$seq_dir" ]; then
      say "no comparisons yet: $seq_dir does not exist"
      say '  make one: scripts/analytics.sh compare <modelA> <modelB> [FILE]'
      return 0
    fi
    while IFS= read -r name; do
      [ -n "$name" ] || continue
      count="$(grep -cvE '^[[:space:]]*(#|$)' "$seq_dir/$name/commands.txt" 2>/dev/null || true)"
      models="$({ find "$seq_dir/$name" -mindepth 1 -maxdepth 1 -type d \
        -printf '%f\n' 2>/dev/null || true; } | sort | paste -sd ' ' -)"
      report=''
      if [ -f "$seq_dir/$name/comparison.md" ]; then report='comparison.md'; fi
      printf '%-40s %4s cmd  %-40s %s\n' \
        "$name" "${count:-0}" "${models:--}" "$report"
    done < <(seq_run_dirs)
    return 0
  fi
  if [ ! -d "$runs_dir" ]; then
    say "no runs yet: $runs_dir does not exist (scripts/analytics.sh one \"...\" makes one)"
    return 0
  fi
  while IFS= read -r name; do
    [ -n "$name" ] || continue
    count="$(grep -cvE '^[[:space:]]*(#|$)' "$runs_dir/$name/commands.txt" 2>/dev/null || true)"
    report=''
    if [ -f "$runs_dir/$name/report.md" ]; then report='report.md'; fi
    printf '%-40s %4s cmd  %s\n' "$name" "${count:-0}" "$report"
  done < <(run_dirs)
  return 0
}

cmd_last() {
  local run
  if [ "$opt_sequential" -eq 1 ]; then
    run="$(seq_run_dirs | head -n 1 || true)"
    [ -n "$run" ] || die \
      "no comparisons yet in $seq_dir (scripts/analytics.sh compare <modelA> <modelB>)"
    run="$seq_dir/$run"
    say "comparison    $(basename "$run")"
    say "dir           $run"
    say "report        $run/comparison.md"
    say ''
    if [ -f "$run/comparison.md" ]; then
      grep -E '^\|' "$run/comparison.md" | head -n 24 || true
    else
      say '(no comparison.md: that comparison aborted before writing one)'
    fi
    return 0
  fi
  last_run_dir || die "no runs yet in $runs_dir"
  run="$RUN_DIR"
  say "run           $(basename "$run")"
  say "dir           $run"
  say "report        $run/report.md"
  say ''
  if [ -f "$run/report.md" ]; then
    grep -E '^\|' "$run/report.md" | head -n 24 || true
  else
    say '(no report.md: the run aborted before writing one)'
  fi
  return 0
}

# ---------------------------------------------------------------------------
# Options and dispatch
# ---------------------------------------------------------------------------
parse_options() {
  local value
  while [ "$#" -gt 0 ]; do
    case "$1" in
      --label)
        [ "$#" -ge 2 ] || die '--label needs a value'
        opt_label="$2"
        shift 2
        ;;
      --label=*) opt_label="${1#*=}"; shift ;;
      --model)
        [ "$#" -ge 2 ] || die '--model needs a value'
        opt_model="$2"
        shift 2
        ;;
      --model=*) opt_model="${1#*=}"; shift ;;
      --last) opt_last=1; shift ;;
      --sequential|--compare) opt_sequential=1; shift ;;
      --no-airlock) opt_airlock=0; shift ;;
      --no-go-to) opt_go_to=0; shift ;;
      --force|-f) opt_force=1; shift ;;
      --simple-discovery) opt_simple=1; shift ;;
      --verbose|-v) opt_verbose=1; shift ;;
      -h|--help) opt_help=1; shift ;;
      --timeout|--wait|--port)
        [ "$#" -ge 2 ] || die "$1 needs a value"
        value="$2"
        [[ "$value" =~ ^[0-9]+$ ]] || die "$1 wants a whole number, got '$value'"
        case "$1" in
          --timeout) opt_timeout="$value" ;;
          --wait) opt_wait="$value" ;;
          --port) port="$value"; opt_port_set=1 ;;
        esac
        shift 2
        ;;
      --timeout=*|--wait=*|--port=*)
        value="${1#*=}"
        [[ "$value" =~ ^[0-9]+$ ]] || die "${1%%=*} wants a whole number, got '$value'"
        case "${1%%=*}" in
          --timeout) opt_timeout="$value" ;;
          --wait) opt_wait="$value" ;;
          --port) port="$value"; opt_port_set=1 ;;
        esac
        shift
        ;;
      --)
        shift
        positionals+=("$@")
        break
        ;;
      -*)
        die "unknown option '$1' (scripts/analytics.sh help)"
        ;;
      *)
        positionals+=("$1")
        shift
        ;;
    esac
  done
}

main() {
  local mode
  opt_help=0
  positionals=()

  if [ "$#" -eq 0 ]; then
    usage
    exit 0
  fi
  mode="$1"
  shift
  case "$mode" in
    help|-h|--help)
      usage
      exit 0
      ;;
  esac

  parse_options "$@"
  if [ "$opt_help" -eq 1 ]; then
    usage
    exit 0
  fi

  # --model picks what a launch starts, so it means nothing to the other modes.
  case "$mode" in
    up|compare) ;;
    *)
      if [ -n "$opt_model" ]; then
        say "note          --model only changes what up/compare launch: ignored here"
      fi
      ;;
  esac

  # Everything below here may need ros2, PYTHONPATH and the workspace overlay.
  source_env
  if [ "$opt_simple" -eq 1 ]; then
    # planner.md §2.1: a standalone node/dry-run is fine without the Discovery
    # Server; leaving it set only produces warnings against a server that is
    # not up.
    unset ROS_DISCOVERY_SERVER
  fi

  case "$mode" in
    up) cmd_up || exit $? ;;
    status) cmd_status || exit $? ;;
    down) cmd_down || exit $? ;;
    probe) cmd_probe || exit $? ;;
    one) cmd_one || exit $? ;;
    sweep) cmd_sweep || exit $? ;;
    compare) cmd_compare || exit $? ;;
    node) cmd_exec 0 || exit $? ;;
    live) cmd_exec 1 || exit $? ;;
    idle) cmd_idle || exit $? ;;
    replay) cmd_replay || exit $? ;;
    runs) cmd_runs || exit $? ;;
    last) cmd_last || exit $? ;;
    *) die "unknown mode '$mode' (scripts/analytics.sh help)" ;;
  esac
}

main "$@"
