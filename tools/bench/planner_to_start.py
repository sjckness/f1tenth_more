#!/usr/bin/env python3
"""
Planner command -> mission started, on the LIVE stack, broken down per stage.

Measures, for each command, the time from "llm_planner_node gets the command"
to "the mission is RUNNING", with the real llama-server (8083), the real
MissionLoader and the real behaviour tree. Nothing here changes the stack:
it only spawns the planner (as f1tenth_ctl does), listens to /mission/status
and calls /mission/abort_mission.

SAFETY: a run STARTS A MISSION, which can make the car move. The tool refuses
to do anything without --car-is-safe (wheels lifted / car on blocks, or motor
output disabled). It aborts after every run, on any error and on
Ctrl-C/SIGTERM, and checks that /mission/status leaves RUNNING; if it cannot
confirm that it prints a loud warning: use the e-stop path.

Run it on the Orin, in the car container, from the host:

    tools/bench/planner_to_start_orin.sh --car-is-safe --suite --runs 10 --cold-first

(the wrapper copies this file into the container and runs it under the stack's
own f1tenth_ctl environment; see its header). Inside a shell that already has
that environment it is just `python3 tools/bench/planner_to_start.py ...`.

Modes:
  oneshot     one `ros2 run llm llm_planner_node "<cmd>"` per run: the process
              start, the server wait and node setup are part of every run.
  persistent  one `ros2 run llm llm_planner_node` for the whole session, a
              command per stdin line (the interactive `comando> ` loop that
              f1tenth_ctl drives through its FIFO); t=0 is the stdin write.
Both run the planner WITHOUT --dry-run (f1tenth_ctl's planner runs with it
and never starts anything: there a human starts the stored mission later
with `f1tenth_ctl mission-load` / `mission-start`).

Timestamps are time.perf_counter() in this process; every t_* in the output
is seconds since t_spawn (oneshot: just before Popen; persistent: just before
the stdin write). Sources:
  t_process_up      arrival of the planner's first output line (stdout+stderr)
  t_server_ready    arrival of its 'Server raggiunto ... scaldat* in' log line
  t_ready           arrival of its '... pronto --' log line (oneshot: the command
                    starts right after); persistent: 0 (the stdin write)
  t_llm_done        t_ready + the planner's own llm= (its 'latenza:' line /
                    RESULT latency, planner >= 18202006); without it, None
  t_planned         arrival of its 'missione generata in' line (LLM + translate)
  t_loaded          first /mission/status LOADED after t=0 (ROS, this process);
                    falls back to the arrival of 'load_mission OK in'
  t_start_returned  arrival of its 'start_mission OK' line: the planner has the
                    service response (only the planner sees that response)
  t_running         first /mission/status RUNNING after t=0 (ROS, this process).
                    It can come BEFORE t_start_returned: with no countdown the
                    loader publishes RUNNING inside the start callback.
  t_exit            planner process exit (oneshot only)
"""

import argparse
import json
import os
import re
import signal
import statistics
import subprocess
import sys
import threading
import time

# ---------------------------------------------------------------- constants

DEFAULT_SUITE = (
    # Real phrasings from src/f1tenth_intelligence/llm/test/test_intent_evals_live.py.
    'vai dritto e fermati a due metri dal muro',            # simple straight
    'vai dalla persona',                                    # go_to
    'vai dritto, al muro gira a destra e avanza 2 metri',   # wall + turn
)
PLANNER_EXE = ('ros2', 'run', 'llm', 'llm_planner_node')   # f1tenth_ctl F1T_PLANNER_CMD
CTL_ENV_DEFAULT = '/opt/f1tenth_ws/src/f1tenth_ctl/config/ctl_container.env'
# f1tenth_ctl config.BASE_KEYS / BASE_PATH: the base the env file is sourced on.
BASE_KEYS = ('HOME', 'USER', 'LOGNAME', 'LANG', 'LC_ALL', 'TERM', 'SHELL',
             'XDG_RUNTIME_DIR', 'DBUS_SESSION_BUS_ADDRESS', 'TZ')
BASE_PATH = '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
STATUS_TOPIC = '/mission/status'
ABORT_SERVICE = '/mission/abort_mission'
LOAD_SERVICE = '/mission/load_mission'
START_SERVICE = '/mission/start_mission'
STOPPED_STATES = ('IDLE', 'ABORTED', 'COMPLETE')
ACTIVE_STATES = ('RUNNING', 'HOLDING')
STAGES = ('process_start', 'llm', 'translate', 'services', 'start_to_running')
DELTAS = STAGES + ('total_to_start', 'total_to_running')
PROMPT_PREFIX = 'comando> '
RESULT_PREFIX = 'RESULT '

_RE_SERVER_READY = re.compile(r'Server raggiunto.*scaldat\w* in ([\d.]+)s')
_RE_PRONTO = re.compile(r'LLM Planner \(llama-server @ .*\) pronto')
_RE_INTERACTIVE = re.compile(r'Scrivi un comando e premi invio')
_RE_PLANNED = re.compile(r'missione generata in ([\d.]+)s')
_RE_LOADED = re.compile(r'load_mission OK in ([\d.]+)s')
_RE_STARTED = re.compile(r'start_mission OK')
_RE_DELIVERY_FAILED = re.compile(r'start_mission FALLITO|load_mission (NON CONFERMATO|FALLITO)')
_RE_LATENZA = re.compile(
    r'latenza: llm=([\d.]+)s traduzione=([\d.]+)s servizi=([\d.]+)s totale=([\d.]+)s')

SAFETY_REFUSAL = """\
REFUSED: this tool starts missions on the live stack, which can make the car move.
Run it only with the wheels lifted / the car on blocks, or with motor output
disabled, and say so with --car-is-safe. Nothing was called."""


# ---------------------------------------------------------------- pure parsing

def strip_prompt(line: str) -> str:
    """Remove the interactive 'comando> ' prompt(s) glued to the start of a line."""
    while line.startswith(PROMPT_PREFIX):
        line = line[len(PROMPT_PREFIX):]
    return line


def parse_latenza(line: str):
    """Return the planner's 'latenza:' numbers as {llm_s, translate_s, services_s, total_s}."""
    m = _RE_LATENZA.search(line)
    if not m:
        return None
    llm, tr, srv, tot = (float(x) for x in m.groups())
    return {'llm_s': llm, 'translate_s': tr, 'services_s': srv, 'total_s': tot}


def parse_result(line: str):
    """Return the RESULT line's JSON object, or None when the line is not one."""
    line = strip_prompt(line.strip())
    if not line.startswith(RESULT_PREFIX):
        return None
    try:
        obj = json.loads(line[len(RESULT_PREFIX):])
    except ValueError:
        return {'status': 'unparsed'}
    return obj if isinstance(obj, dict) else {'status': 'unparsed'}


def classify_line(line: str):
    """Return (event, value) for a planner output line, or None."""
    line = strip_prompt(line)
    result = parse_result(line)
    if result is not None:
        return 'result', result
    lat = parse_latenza(line)
    if lat is not None:
        return 'latenza', lat
    for event, rx in (('server_ready', _RE_SERVER_READY), ('planned', _RE_PLANNED),
                      ('loaded_log', _RE_LOADED)):
        m = rx.search(line)
        if m:
            return event, float(m.group(1))
    for event, rx in (('ready', _RE_PRONTO), ('interactive_ready', _RE_INTERACTIVE),
                      ('started_log', _RE_STARTED), ('delivery_failed', _RE_DELIVERY_FAILED)):
        if rx.search(line):
            return event, line.strip()
    return None


def first_state_after(statuses, state: str, t0: float):
    """First perf_counter time >= t0 at which /mission/status said `state`."""
    for t, s in statuses:
        if t >= t0 and s == state:
            return t
    return None


def compute_deltas(t: dict, latency, mode: str) -> dict:
    """
    Derive the per-stage durations (seconds) from one run's t_* and the planner's numbers.

    t: t_* relative to t_spawn (None when not seen). latency: the planner's own
    {llm_s, translate_s, services_s, total_s} or None (planner before 18202006).
    """
    def sub(a, b):
        return None if a is None or b is None else a - b

    lat = latency or {}
    d = {}
    if mode == 'oneshot':
        d['process_start'] = t.get('t_ready') if t.get('t_ready') is not None else (
            t.get('t_server_ready') if t.get('t_server_ready') is not None
            else t.get('t_process_up'))
    else:
        d['process_start'] = None      # paid once per session, not per command
    d['llm'] = lat.get('llm_s')
    d['translate'] = lat.get('translate_s')
    if d['llm'] is None and t.get('t_planned') is not None:
        # No 'latenza:' line: the planner's LLM + translate window, seen from here.
        d['llm'] = sub(t.get('t_planned'), t.get('t_ready', 0.0))
    d['services'] = sub(t.get('t_start_returned'), t.get('t_planned'))
    d['start_to_running'] = sub(t.get('t_running'), t.get('t_start_returned'))
    d['total_to_start'] = t.get('t_start_returned')
    d['total_to_running'] = t.get('t_running')
    return d


def representative(run: dict):
    """Return (True, '') for a run that belongs in the stats, else (False, why)."""
    res = run.get('result') or {}
    if run.get('error'):
        return False, run['error']
    if res.get('status') != 'ok':
        return False, f"status {res.get('status')}"
    if (res.get('attempts') or 1) > 1:
        return False, f"attempts {res.get('attempts')}"
    if res.get('unsupported'):
        return False, f"unsupported {res.get('unsupported')}"
    if res.get('ambiguous'):
        return False, 'ambiguous'
    if str(run.get('end', '')).startswith('ready_refused'):
        return True, ''          # --not-ready-start stop: stages up to the refusal only
    if run.get('t', {}).get('t_running') is None:
        return False, 'never RUNNING'
    return True, ''


def percentile(xs, q: float):
    """Nearest-rank percentile (q in 0..100) of a non-empty list; None for an empty one."""
    xs = sorted(x for x in xs if x is not None)
    if not xs:
        return None
    k = max(1, -(-len(xs) * q // 100))   # ceil(n * q / 100), at least 1
    return xs[int(k) - 1]


def stats(values):
    vals = [v for v in values if v is not None]
    if not vals:
        return None
    return {'n': len(vals), 'median': statistics.median(vals),
            'p95': percentile(vals, 95), 'max': max(vals)}


def summarize(runs, deltas=DELTAS):
    """Per command and overall: stats per delta over the representative runs."""
    kept = [r for r in runs if representative(r)[0]]
    excluded = [{'command': r['command'], 'run': r['run'], 'why': representative(r)[1]}
                for r in runs if not representative(r)[0]]
    groups = {}
    for r in kept:
        groups.setdefault(r['command'], []).append(r)
    out = {'per_command': {}, 'overall': {}, 'excluded': excluded, 'kept': len(kept)}
    for cmd, rs in groups.items():
        out['per_command'][cmd] = {d: stats([r['deltas'].get(d) for r in rs]) for d in deltas}
    out['overall'] = {d: stats([r['deltas'].get(d) for r in kept]) for d in deltas}
    return out


def _fmt(v):
    return '-' if v is None else f'{v:.3f}'


def markdown_table(summary, deltas=DELTAS, stages=STAGES) -> str:
    """One markdown table: rows = command (+ overall) x stage; the largest median stage bold."""
    lines = ['| command | ' + ' | '.join(deltas) + ' |',
             '|---|' + '---|' * len(deltas)]
    rows = list(summary['per_command'].items()) + [('**overall**', summary['overall'])]
    for name, st in rows:
        medians = {s: (st.get(s) or {}).get('median') for s in stages}
        known = {s: v for s, v in medians.items() if v is not None}
        largest = max(known, key=known.get) if known else None
        cells = []
        for d in deltas:
            s = st.get(d)
            cell = '-' if not s else f"{_fmt(s['median'])} / {_fmt(s['p95'])} / {_fmt(s['max'])}"
            cells.append(f'**{cell}**' if d == largest else cell)
        lines.append(f'| {name} | ' + ' | '.join(cells) + ' |')
    lines.append('')
    lines.append('cells: median / p95 / max, seconds; bold = the largest stage of that row '
                 f"(stages: {', '.join(stages)}). Representative runs: {summary['kept']}.")
    if summary['excluded']:
        lines.append('')
        lines.append('Excluded (not representative):')
        for e in summary['excluded']:
            lines.append(f"- run {e['run']} {e['command']!r}: {e['why']}")
    return '\n'.join(lines)


# ---------------------------------------------------------------- production flow (pure)
#
# --flow production drives the chain the operator drives, through f1tenth_ctl's
# own CLI, against the planner unit f1tenth_ctl already runs (persistent,
# --dry-run, stdin = its FIFO) and the stack's llama-server:
#   send planner "<cmd>"        -> the FIFO line            (t_cmd = 0, t_send_ret)
#   the RESULT line appended to the planner unit's log       (t_plan_done)
#   planner-log --since <cursor> -> ctl STORES the mission   (t_plan_fetched)
#   [pause: the operator reading the plan; 0 s or --variant-pause-s]
#   mission-load <stored id>    -> /mission/status LOADED    (t_load_call, t_loaded, t_load_ret)
#   [pause]
#   mission-start               -> /mission/status RUNNING   (t_start_call, t_running, t_start_ret)
#   first /ackermann_drive with a non-zero speed (or steering, see first_move_rule)
#                                                            (t_first_move)
PROD_STAGES = ('llm_and_ctl', 'fetch_cli', 'load_cli', 'start_cli', 'running_to_move')
PROD_DELTAS = PROD_STAGES + ('send_cli', 'ready_cli', 'pause', 'total_to_running',
                             'total_to_move', 'total_to_move_no_pause', 'cmd_to_plan',
                             'total_to_refusal')
CTL_BIN_DEFAULT = '/opt/f1tenth_ws/src/f1tenth_ctl/bin/f1tenth_ctl'
PLANNER_LOG_DEFAULT = '~/.local/state/f1tenth_ctl/logs/planner.log'
DRIVE_TOPIC = '/ackermann_drive'
COLD_IDLE_S = 300.0
MOVE_EPS = 1e-3


def compute_prod_deltas(t: dict) -> dict:
    """Per-stage durations (seconds) of one production run from its t_* (t_cmd = 0)."""
    def sub(a, b):
        return None if a is None or b is None else a - b

    pause = 0.0
    for a, b in (('t_load_call', 't_plan_fetched'), ('t_start_call', 't_load_ret')):
        p = sub(t.get(a), t.get(b))
        pause = None if p is None or pause is None else pause + p
    d = {
        'send_cli': t.get('t_send_ret'),
        'llm_and_ctl': t.get('t_plan_done'),
        'fetch_cli': sub(t.get('t_plan_fetched'), t.get('t_plan_done')),
        'pause': pause,
        'load_cli': sub(t.get('t_loaded'), t.get('t_load_call')),
        'start_cli': sub(t.get('t_running'), t.get('t_start_call')),
        'ready_cli': None,
        'running_to_move': sub(t.get('t_first_move'), t.get('t_running')),
        'total_to_running': t.get('t_running'),
        'total_to_move': t.get('t_first_move'),
    }
    if t.get('t_running') is None:
        # mission-start refused (READY false): its whole wall time is the READY part.
        d['ready_cli'] = sub(t.get('t_start_ret'), t.get('t_start_call'))
    d['total_to_move_no_pause'] = sub(d['total_to_move'], pause)
    d['cmd_to_plan'] = t.get('t_plan_fetched')       # command -> the plan in the caller's hand
    d['total_to_refusal'] = None
    if t.get('t_running') is None and t.get('t_start_ret') is not None:
        d['total_to_refusal'] = sub(t['t_start_ret'], pause)   # command -> READY refusal
    return d


def first_move_rule(mission) -> str:
    """'steering' when the mission's first move is a turn, else 'speed'."""
    try:
        mode = str(mission['moves'][0]['drive']['mode'])
    except (KeyError, IndexError, TypeError):
        return 'speed'
    return 'steering' if 'turn' in mode else 'speed'


def is_move(speed: float, steering: float, rule: str) -> bool:
    """True when a drive command is a move under `rule` (see first_move_rule)."""
    value = steering if rule == 'steering' else speed
    return abs(value) > MOVE_EPS


def parse_ctl_json(stdout: str):
    """f1tenth_ctl's answer: the last line of its stdout that is a JSON object."""
    for line in reversed((stdout or '').strip().splitlines()):
        try:
            obj = json.loads(line)
        except ValueError:
            continue
        if isinstance(obj, dict):
            return obj
    return None


def stored_mission(planner_log_answer):
    """(stored id, mission) of the last RESULT entry ctl stored, or (None, None)."""
    for entry in reversed((planner_log_answer or {}).get('results') or []):
        rec = entry.get('stored')
        if rec and rec.get('id'):
            return rec['id'], entry.get('mission')
    return None, None


def is_cold(idle_s, threshold: float = COLD_IDLE_S) -> bool:
    """A run after >= threshold s without planner activity is cold."""
    return idle_s is not None and idle_s >= threshold


def check_safety(car_is_safe: bool):
    """Return the refusal text when the car is not declared safe, else None."""
    return None if car_is_safe else SAFETY_REFUSAL


# ---------------------------------------------------------------- environment

def planner_environment(ctl_env: str, base=None) -> dict:
    """Return f1tenth_ctl's unit environment (procunits.unit_environment), before the env file."""
    src = os.environ if base is None else base
    env = {k: src[k] for k in BASE_KEYS if k in src}
    home = env.get('HOME', os.path.expanduser('~'))
    env['HOME'] = home
    env['PATH'] = f'{home}/.local/bin:{BASE_PATH}'
    env.update({'F1T_CTL_ENV': ctl_env, 'PYTHONUNBUFFERED': '1',
                'RCUTILS_LOGGING_BUFFERED_STREAM': '0'})
    return env


def planner_argv(mode: str, command: str = '', pythonpath: str = '') -> list:
    """bash -c that sources the env file like units.UNIT_SCRIPT, then execs the planner."""
    extra = f'PYTHONPATH="{pythonpath}:$PYTHONPATH"; export PYTHONPATH; ' if pythonpath else ''
    script = ('set -a; . "$F1T_CTL_ENV" || exit 97; set +a; ' + extra + 'exec "$@"')
    argv = ['/bin/bash', '-c', script, 'planner', *PLANNER_EXE]
    if mode == 'oneshot':
        argv.append(command)
    return argv


def session_metadata(args) -> dict:
    """What explains a difference between two sessions."""
    meta = {'git_head': os.environ.get('F1T_BENCH_GIT_HEAD'),
            'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
            'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION'),
            'ros_discovery_server': os.environ.get('ROS_DISCOVERY_SERVER'),
            'fastdds_profile': os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE'),
            'mode': args.mode, 'planner_pythonpath': args.planner_pythonpath or None,
            'time': time.strftime('%Y-%m-%dT%H:%M:%S')}
    if meta['git_head'] is None:
        try:
            meta['git_head'] = subprocess.run(
                ['git', '-C', os.path.dirname(os.path.abspath(__file__)), 'rev-parse',
                 '--short', 'HEAD'], capture_output=True, text=True, timeout=5).stdout.strip()
        except (OSError, subprocess.SubprocessError):
            pass
    try:
        with open('/opt/f1tenth_ws/SOURCES') as fh:
            meta['image_sources'] = fh.read().splitlines()[:4]
    except OSError:
        meta['image_sources'] = None
    procs = {}
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as fh:
                argv = fh.read().split(b'\0')
        except OSError:
            continue
        cmd = ' '.join(a.decode(errors='replace') for a in argv if a)
        if 'llama-server' in cmd and '--port 8083' in cmd:
            procs['llama_server_cmdline'] = cmd
        if '_ros2_daemon' in cmd:
            procs['ros2_daemon'] = cmd
    meta['llama_server_cmdline'] = procs.get('llama_server_cmdline')
    meta['ros2_daemon_running'] = 'ros2_daemon' in procs
    try:
        import urllib.request
        with urllib.request.urlopen('http://127.0.0.1:8083/props', timeout=3) as r:
            props = json.load(r)
        meta['llama_server_props'] = {k: props.get(k) for k in (
            'model_path', 'total_slots', 'build_info', 'n_ctx')}
        meta['llama_server_props']['n_ctx'] = (
            props.get('default_generation_settings', {}).get('n_ctx'))
    except Exception as exc:  # noqa: BLE001 - metadata only
        meta['llama_server_props'] = f'unavailable: {type(exc).__name__}'
    env = planner_environment(args.ctl_env)
    probe = ('set -a; . "$F1T_CTL_ENV" >/dev/null 2>&1; set +a; '
             + (f'PYTHONPATH="{args.planner_pythonpath}:$PYTHONPATH" '
                if args.planner_pythonpath else '')
             + 'python3 -c "import llm.llm_planner_node as m; print(m.__file__)"')
    try:
        meta['planner_module'] = subprocess.run(
            ['/bin/bash', '-c', probe], env=env, capture_output=True, text=True,
            timeout=30).stdout.strip() or None
    except (OSError, subprocess.SubprocessError):
        meta['planner_module'] = None
    return meta


# ---------------------------------------------------------------- live part

class Planner:
    """A planner process whose merged stdout/stderr lines are time-stamped on arrival."""

    def __init__(self, argv, env, stdin_pipe: bool):
        self.lines = []          # (perf_counter, line)
        self.cond = threading.Condition()
        self.t_spawn = time.perf_counter()
        self.proc = subprocess.Popen(
            argv, env=env, stdin=subprocess.PIPE if stdin_pipe else subprocess.DEVNULL,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True,
            bufsize=0)
        self.t_exit = None
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    def _read(self):
        buf = b''
        while True:
            chunk = os.read(self.proc.stdout.fileno(), 65536)
            now = time.perf_counter()
            if not chunk:
                break
            buf += chunk
            while b'\n' in buf:
                raw, buf = buf.split(b'\n', 1)
                with self.cond:
                    self.lines.append((now, raw.decode(errors='replace')))
                    self.cond.notify_all()
        if buf:
            with self.cond:
                self.lines.append((time.perf_counter(), buf.decode(errors='replace')))
        self.proc.wait()
        with self.cond:
            self.t_exit = time.perf_counter()
            self.cond.notify_all()

    def send(self, text: str) -> float:
        t = time.perf_counter()
        self.proc.stdin.write((text + '\n').encode())
        self.proc.stdin.flush()
        return t

    def wait_for(self, pred, start_index: int, timeout: float):
        """Wait until a line from start_index on satisfies pred(line); return its index."""
        deadline = time.perf_counter() + timeout
        i = start_index
        with self.cond:
            while True:
                while i < len(self.lines):
                    if pred(self.lines[i][1]):
                        return i
                    i += 1
                if self.t_exit is not None:
                    return None
                left = deadline - time.perf_counter()
                if left <= 0:
                    return None
                self.cond.wait(left)

    def stop(self, timeout: float = 10.0):
        """Stop the process tree the way Ctrl-C does (CLAUDE.md: killpg SIGINT)."""
        if self.proc.poll() is not None:
            return
        try:
            pgid = os.getpgid(self.proc.pid)
            if self.proc.stdin:
                try:
                    self.proc.stdin.close()
                except OSError:
                    pass
            os.killpg(pgid, signal.SIGINT)
            self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(pgid, signal.SIGKILL)
            self.proc.wait(timeout=5)
        except ProcessLookupError:
            pass


class Stack:
    """The tool's own ROS side: /mission/status (kept for the session) and the mission services."""

    def __init__(self, drive=False):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                               QoSReliabilityPolicy)
        from rclpy.signals import SignalHandlerOptions
        from f1tenth_messages.msg import MissionStatus
        from std_srvs.srv import Trigger
        self._rclpy = rclpy
        self._trigger = Trigger
        # No rclpy signal handlers: Ctrl-C/SIGTERM must reach run_session's
        # handler, whose finally stops the planner and aborts the mission.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('planner_to_start_bench')
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.statuses = []       # (perf_counter, state)
        self.cond = threading.Condition()
        self.node.create_subscription(MissionStatus, STATUS_TOPIC, self._on_status, qos)
        self.abort_client = self.node.create_client(Trigger, ABORT_SERVICE)
        self.drives = []         # (perf_counter, speed, steering_angle), --flow production
        if drive:
            from ackermann_msgs.msg import AckermannDriveStamped
            self.node.create_subscription(AckermannDriveStamped, DRIVE_TOPIC,
                                          self._on_drive, 50)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._spin = threading.Thread(target=self.executor.spin, daemon=True)
        self._spin.start()

    def _on_drive(self, msg):
        with self.cond:
            self.drives.append((time.perf_counter(), float(msg.drive.speed),
                                float(msg.drive.steering_angle)))
            self.cond.notify_all()

    def _on_status(self, msg):
        with self.cond:
            self.statuses.append((time.perf_counter(), str(msg.state)))
            self.cond.notify_all()

    def state(self):
        with self.cond:
            return self.statuses[-1][1] if self.statuses else None

    def wait_state(self, states, timeout: float):
        """Wait until the latest /mission/status state is in `states`; return it (None: timeout)."""
        deadline = time.perf_counter() + timeout
        with self.cond:
            while True:
                if self.statuses and self.statuses[-1][1] in states:
                    return self.statuses[-1][1]
                left = deadline - time.perf_counter()
                if left <= 0:
                    return None
                self.cond.wait(left)

    def services_available(self, timeout: float = 5.0) -> bool:
        return self.abort_client.wait_for_service(timeout_sec=timeout)

    def abort(self, timeout: float = 5.0):
        """Call /mission/abort_mission; return (ok, message)."""
        if not self.abort_client.wait_for_service(timeout_sec=timeout):
            return False, f'{ABORT_SERVICE} not available'
        fut = self.abort_client.call_async(self._trigger.Request())
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout) or fut.result() is None:
            return False, f'{ABORT_SERVICE}: no answer in {timeout:.0f} s'
        res = fut.result()
        return bool(res.success), str(res.message)

    def close(self):
        try:
            self.executor.shutdown(timeout_sec=2.0)
            self.node.destroy_node()
            self._rclpy.try_shutdown()
        except Exception:  # noqa: BLE001 - best effort at exit
            pass


LOUD = """
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!! ABORT NOT CONFIRMED: /mission/status is {state!r} after abort_mission.
!!! The mission may still be RUNNING. Use the e-stop path NOW:
!!!     f1tenth_ctl estop      (zero drive + /mission/emergency_stop + abort)
!!! {detail}
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
"""


def safe_abort(stack: Stack, log) -> bool:
    """Abort and confirm the state left RUNNING; print the loud warning when it did not."""
    ok, msg = stack.abort()
    state = stack.wait_state(STOPPED_STATES, timeout=5.0)
    if state is None:
        cur = stack.state()
        if cur in (None, 'LOADED') and ok:
            # LOADED/no status: nothing is moving; a start that never fired.
            log(f'abort: {msg} (state {cur})')
            return True
        print(LOUD.format(state=cur, detail=f'abort_mission said: ok={ok} {msg}'),
              file=sys.stderr, flush=True)
        return False
    log(f'abort: {msg} -> {state}')
    return True


def run_once(stack: Stack, planner_factory, mode: str, command: str, index: int,
             tag: str, run_timeout: float, log, persistent=None, current=None) -> dict:
    """One measured run. The caller aborts afterwards (always)."""
    run = {'command': command, 'run': index, 'tag': tag, 'mode': mode, 't': {},
           'result': None, 'latency': None, 'error': None}
    # 0. reset
    ok, msg = stack.abort()
    state = stack.wait_state(STOPPED_STATES, timeout=10.0)
    if state is None and stack.state() not in (None,):
        run['error'] = f'reset: state {stack.state()!r} not idle/aborted/complete in 10 s ({msg})'
        return run
    run['state_before'] = state or stack.state()
    n_status = len(stack.statuses)
    # 1-2. spawn / send
    if mode == 'oneshot':
        planner = planner_factory(command)
        if current is not None:
            current['planner'] = planner
        t0 = planner.t_spawn
        start_line = 0
    else:
        planner = persistent
        start_line = len(planner.lines)
        t0 = planner.send(command)
    # 3. wait for the RESULT line (or exit / timeout), aborting as soon as RUNNING shows
    deadline = t0 + run_timeout
    aborted_early = False
    while True:
        left = deadline - time.perf_counter()
        idx = planner.wait_for(lambda ln: parse_result(ln) is not None, start_line,
                               timeout=min(0.2, max(left, 0.0)))
        if not aborted_early and first_state_after(
                stack.statuses[n_status:], 'RUNNING', t0) is not None:
            # Safety first: stop the mission the moment it runs.
            safe_abort(stack, log)
            aborted_early = True
        if idx is not None or planner.t_exit is not None or left <= 0:
            break
    if mode == 'oneshot':
        if planner.proc.poll() is None:
            remaining = max(0.0, deadline - time.perf_counter())
            try:
                planner.proc.wait(timeout=min(remaining, 15.0))
            except subprocess.TimeoutExpired:
                planner.stop()
        planner._reader.join(timeout=5.0)
    elif idx is None:
        run['error'] = f'no RESULT within {run_timeout:.0f} s'
    # collect events from the lines of this run
    t = run['t']
    lines = planner.lines[start_line:]
    if lines:
        t['t_process_up'] = lines[0][0] - t0 if mode == 'oneshot' else None
    for ts, line in lines:
        ev = classify_line(line)
        if ev is None:
            continue
        name, val = ev
        key = {'server_ready': 't_server_ready', 'ready': 't_ready', 'planned': 't_planned',
               'loaded_log': 't_loaded_log', 'started_log': 't_start_returned'}.get(name)
        if key and key not in t:
            t[key] = ts - t0
        if name == 'latenza':
            run['latency'] = val
        elif name == 'result':
            run['result'] = val
        elif name == 'delivery_failed':
            run['error'] = run['error'] or val
    if mode == 'persistent':
        t['t_ready'] = 0.0
    res = run['result'] or {}
    if run['latency'] is None and isinstance(res.get('latency'), dict):
        run['latency'] = res['latency']
    sts = stack.statuses[n_status:]
    t_loaded = first_state_after(sts, 'LOADED', t0)
    t['t_loaded'] = (t_loaded - t0) if t_loaded is not None else t.get('t_loaded_log')
    t_running = first_state_after(sts, 'RUNNING', t0)
    t['t_running'] = (t_running - t0) if t_running is not None else None
    if mode == 'oneshot':
        t['t_exit'] = (planner.t_exit - t0) if planner.t_exit is not None else None
    if run['latency'] and t.get('t_ready') is not None:
        t['t_llm_done'] = t['t_ready'] + run['latency']['llm_s']
    run['deltas'] = compute_deltas(t, run['latency'], mode)
    run['attempts'] = res.get('attempts')
    run['status'] = res.get('status')
    if mode == 'oneshot':
        run['output_tail'] = [ln for _, ln in lines[-40:]]
    if t.get('t_running') is None and not run['error']:
        run['error'] = 'never RUNNING' + (f" (result {res.get('status')})" if res else '')
    return run


class LogTail:
    """Lines appended to a file from its current end on, time-stamped on arrival (5 ms poll)."""

    def __init__(self, path: str):
        self.path = path
        self.lines = []          # (perf_counter, line)
        self.cond = threading.Condition()
        self._stop = False
        try:
            self._offset = os.path.getsize(path)
        except OSError:
            self._offset = 0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        buf = b''
        while not self._stop:
            try:
                with open(self.path, 'rb') as fh:
                    fh.seek(self._offset)
                    chunk = fh.read()
            except OSError:
                chunk = b''
            now = time.perf_counter()
            if chunk:
                self._offset += len(chunk)
                buf += chunk
                while b'\n' in buf:
                    raw, buf = buf.split(b'\n', 1)
                    with self.cond:
                        self.lines.append((now, raw.decode(errors='replace')))
                        self.cond.notify_all()
            time.sleep(0.005)

    def wait_for(self, pred, timeout: float):
        """First (time, line) satisfying pred(line), waiting up to timeout; None otherwise."""
        deadline = time.perf_counter() + timeout
        i = 0
        with self.cond:
            while True:
                while i < len(self.lines):
                    if pred(self.lines[i][1]):
                        return self.lines[i]
                    i += 1
                left = deadline - time.perf_counter()
                if left <= 0:
                    return None
                self.cond.wait(left)

    def stop(self):
        self._stop = True
        self._thread.join(timeout=1.0)


def ctl_call(ctl: list, argv: list, timeout: float, current=None):
    """Run one f1tenth_ctl CLI command; return (t_call, t_return, answer, rc)."""
    t_call = time.perf_counter()
    proc = subprocess.Popen([*ctl, *argv], stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, text=True, start_new_session=True)
    if current is not None:
        current['ctl'] = proc
    try:
        out, _err = proc.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        out, _err = proc.communicate()
    t_ret = time.perf_counter()
    if current is not None:
        current['ctl'] = None
    return t_call, t_ret, parse_ctl_json(out), proc.returncode


def planner_unit(planner_log: str) -> dict:
    """The running production planner (ctl's unit): its command line and log idle time."""
    info = {'cmdline': None, 'dry_run': None, 'idle_s': None}
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as fh:
                argv = [a.decode(errors='replace') for a in fh.read().split(b'\0') if a]
        except OSError:
            continue
        if any(a.endswith('llm_planner_node') for a in argv) and 'run' not in argv[:3]:
            info['cmdline'] = ' '.join(argv)
            info['dry_run'] = '--dry-run' in argv
    try:
        info['idle_s'] = time.time() - os.path.getmtime(planner_log)
    except OSError:
        pass
    return info


def watch_start(stack, t_call: float, n_status: int, rule: str, th, holder, move_timeout: float,
                log):
    """Watch RUNNING and the first move while a start runs in `th`; abort at once on either."""
    n_drive = len(stack.drives)
    deadline = t_call + 40.0
    t_move = t_run = None
    while time.perf_counter() < deadline:
        t_run = first_state_after(stack.statuses[n_status:], 'RUNNING', t_call)
        if t_run is not None:
            for td, speed, steer in stack.drives[n_drive:]:
                if td >= t_run and is_move(speed, steer, rule):
                    t_move = td
                    break
            if t_move is not None or time.perf_counter() > t_run + move_timeout:
                break
        elif not th.is_alive() and time.perf_counter() > holder.get('r', (0, 0))[1] + 2.0:
            break           # the start returned and nothing is RUNNING 2 s later
        time.sleep(0.002)
    if t_move is not None or t_run is not None:
        safe_abort(stack, log)       # stop the mission at once
    th.join(timeout=45.0)
    return t_run, t_move


def run_prod_once(stack, args, command: str, index: int, pause: float, variant: str,
                  log, current) -> dict:
    """One production-flow run. The caller aborts afterwards (always)."""
    ctl = [args.ctl]
    unit = planner_unit(args.planner_log)
    run = {'command': command, 'run': index, 'variant': variant, 'flow': 'production',
           'tag': 'cold' if is_cold(unit['idle_s']) else 'warm', 'idle_s': unit['idle_s'],
           't': {}, 'result': None, 'latency': None, 'error': None, 'ctl': {}}
    ok, msg = stack.abort()
    if stack.wait_state(STOPPED_STATES, timeout=10.0) is None and stack.state() is not None:
        run['error'] = f'reset: state {stack.state()!r} not idle/aborted/complete in 10 s ({msg})'
        return run
    t = run['t']
    tail = LogTail(os.path.expanduser(args.planner_log))
    n_status = len(stack.statuses)
    try:
        # send: the line goes to the planner's FIFO (t_cmd = 0)
        t0, t_ret, ans, rc = ctl_call(ctl, ['send', 'planner', command], 30.0, current)
        t['t_cmd'] = 0.0
        t['t_send_ret'] = t_ret - t0
        run['ctl']['send'] = ans
        if not (ans or {}).get('ok'):
            run['error'] = f'send failed: {ans}'
            return run
        cursor = ans.get('cursor', 0)
        fetched = None
        if args.plan_wait > 0:
            # the bot's way: planner-log --wait right after the send, no polling
            _, t_ret, fetched, rc = ctl_call(
                ctl, ['planner-log', '--since', str(cursor), '--wait', f'{args.plan_wait:g}'],
                args.plan_wait + 30.0, current)
            t['t_plan_fetched'] = t_ret - t0
        elif args.plan_poll > 0:
            # the bot today: sleep, planner-log from the cursor, repeat until the RESULT
            cur, deadline = cursor, time.perf_counter() + args.run_timeout
            while time.perf_counter() < deadline:
                time.sleep(args.plan_poll)
                _, t_ret, answer, rc = ctl_call(
                    ctl, ['planner-log', '--since', str(cur)], 30.0, current)
                cur = (answer or {}).get('cursor', cur)
                if (answer or {}).get('results'):
                    fetched = answer
                    t['t_plan_fetched'] = t_ret - t0
                    break
        hit = tail.wait_for(lambda ln: parse_result(ln) is not None, args.run_timeout)
        if hit is None:
            run['error'] = f'no RESULT in the planner log within {args.run_timeout:.0f} s'
            return run
        t['t_plan_done'] = hit[0] - t0
        run['result'] = parse_result(hit[1])
        for ts, line in tail.lines:
            lat = parse_latenza(line)
            if lat:
                run['latency'] = lat
        res = run['result'] or {}
        run['attempts'] = res.get('attempts')
        run['status'] = res.get('status')
        if res.get('status') != 'ok':
            return run              # refused / failed plan: listed, nothing to load
        if fetched is None:
            # planner-log: what the bot polls; ctl stores the mission here
            _, t_ret, fetched, rc = ctl_call(ctl, ['planner-log', '--since', str(cursor)],
                                             30.0, current)
            t['t_plan_fetched'] = t_ret - t0
        ans = fetched
        sid, mission = stored_mission(ans)
        if sid is None:
            run['error'] = 'planner-log stored no mission'
            return run
        run['stored_id'] = sid
        rule = first_move_rule(mission or res.get('mission'))
        run['first_move_rule'] = rule
        time.sleep(pause)
        t_call, t_ret, ans, rc = ctl_call(ctl, ['mission-load', sid], 40.0, current)
        t['t_load_call'], t['t_load_ret'] = t_call - t0, t_ret - t0
        run['ctl']['load'] = {k: ans.get(k) for k in ('ok', 'error', 'refused', 'elapsed_s')} \
            if ans else None
        t_loaded = first_state_after(stack.statuses[n_status:], 'LOADED', t_call)
        t['t_loaded'] = t_loaded - t0 if t_loaded is not None else None
        if not (ans or {}).get('ok'):
            run['error'] = f'mission-load failed: {ans}'
            return run
        time.sleep(pause)
        # mission-start runs in the background: the first move is watched (and the
        # mission aborted) while the CLI may still be waiting for its status echo.
        t_call = time.perf_counter()
        t['t_start_call'] = t_call - t0
        holder = {}
        th = threading.Thread(target=lambda: holder.update(
            r=ctl_call(ctl, ['mission-start'], 40.0, current)), daemon=True)
        th.start()
        t_run, t_move = watch_start(stack, t_call, n_status, rule, th, holder,
                                    args.move_timeout, log)
        ans = holder['r'][2] if 'r' in holder else None
        if 'r' in holder:
            t['t_start_ret'] = holder['r'][1] - t0
            run['ctl']['start'] = {k: ans.get(k) for k in (
                'ok', 'error', 'refused', 'elapsed_s', 'countdown_s')} if ans else None
        refused = str((ans or {}).get('refused') or '')
        if t_run is None and refused.startswith('not READY') and args.not_ready_start == 'stop':
            # READY is false (on the bench: battery disconnected). ctl refuses before it
            # starts anything; the run ends here, timed up to the refusal (ready_cli).
            run['end'] = f'ready_refused: {refused}'
            return run
        if t_run is None and not (ans or {}).get('ok'):
            run['error'] = f'mission-start failed: {ans}'
        t['t_running'] = t_run - t0 if t_run is not None else None
        t['t_first_move'] = t_move - t0 if t_move is not None else None
        if t_run is not None and t_move is None:
            run['error'] = run['error'] or f'no move within {args.move_timeout:.0f} s of RUNNING'
        return run
    finally:
        tail.stop()
        run['deltas'] = compute_prod_deltas(t)
        if (t.get('t_running') is None and not run['error'] and run.get('status') == 'ok'
                and not run.get('end')):
            run['error'] = 'never RUNNING'


def run_prod_session(args, log=print) -> dict:
    """--flow production: the operator's chain through f1tenth_ctl, aborted after every run."""
    meta = session_metadata(args)
    meta['flow'] = 'production'
    meta['image_ref'] = os.environ.get('F1TENTH_IMAGE_REF')
    meta['planner_unit'] = planner_unit(os.path.expanduser(args.planner_log))
    meta['ctl'] = args.ctl
    meta['first_move'] = (f'first {DRIVE_TOPIC} after RUNNING with |drive.speed| > {MOVE_EPS} '
                          '(|drive.steering_angle| when the first move is a turn)')
    log('session: ' + json.dumps(meta, ensure_ascii=False))
    if not meta['planner_unit']['cmdline']:
        raise SystemExit('no planner unit running: start it the operator way, '
                         '`f1tenth_ctl start planner`')
    if not meta['planner_unit']['dry_run']:
        raise SystemExit('the running planner is not --dry-run: it would start missions itself')
    stack = Stack(drive=True)
    current = {'ctl': None}
    runs = []
    interrupted = {'sig': None}

    def on_signal(sig, _frame):
        interrupted['sig'] = sig
        raise KeyboardInterrupt

    old = {s: signal.signal(s, on_signal) for s in (signal.SIGINT, signal.SIGTERM)}
    confirmed = False
    try:
        if not stack.services_available(10.0):
            raise RuntimeError(f'{ABORT_SERVICE} not available: is the stack up?')
        commands = list(args.cmd or DEFAULT_SUITE)
        plan = [(i, c, 0.0, 'pause0') for i in range(args.runs) for c in commands]
        plan += [(args.runs + k, commands[k % len(commands)], args.variant_pause_s,
                  f'pause{args.variant_pause_s:g}') for k in range(args.variant_runs)]
        for index, command, pause, variant in plan:
            run = run_prod_once(stack, args, command, index, pause, variant, log, current)
            if not safe_abort(stack, log):
                run['error'] = run['error'] or 'abort not confirmed'
                runs.append(run)
                raise RuntimeError('abort not confirmed: stopping the session')
            runs.append(run)
            slim = dict(run, result={k: v for k, v in (run['result'] or {}).items()
                                     if k not in ('mission', 'llm_raw')})
            log('RUN ' + json.dumps(slim, ensure_ascii=False))
            if args.out:
                _write_prod(args.out, meta, runs)
        confirmed = True
    except KeyboardInterrupt:
        log(f'interrupted ({interrupted["sig"]}): aborting')
    finally:
        for s in old:
            signal.signal(s, signal.SIG_IGN)
        # A mission-start CLI in flight could still start the mission: kill it first.
        proc = current.get('ctl')
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        ok = safe_abort(stack, log)
        confirmed = confirmed and ok
        final_state = stack.state()
        for s, h in old.items():
            signal.signal(s, h)
        stack.close()
    session = prod_session_doc(meta, runs)
    session.update(complete=confirmed, final_state=final_state)
    if args.out:
        _write_prod(args.out, meta, runs, session)
    return session


def prod_session_doc(meta, runs) -> dict:
    """The production session document: runs + one summary per pause variant."""
    variants = {}
    for r in runs:
        variants.setdefault(r.get('variant', 'pause0'), []).append(r)
    return {'meta': meta, 'runs': runs, 'flow': 'production',
            'summary': summarize(variants.get('pause0', []), PROD_DELTAS),
            'summaries': {v: summarize(rs, PROD_DELTAS) for v, rs in variants.items()},
            'complete': False}


def _write_prod(path, meta, runs, session=None):
    doc = session or prod_session_doc(meta, runs)
    tmp = path + '.tmp'
    with open(tmp, 'w') as fh:
        json.dump(doc, fh, indent=1, ensure_ascii=False)
    os.replace(tmp, path)


def run_session(args, log=print) -> dict:
    """The live part: the whole session, always ending in a confirmed abort."""
    if getattr(args, 'flow', None) == 'production':
        return run_prod_session(args, log)
    meta = session_metadata(args)
    log('session: ' + json.dumps(meta, ensure_ascii=False))
    stack = Stack()
    persistent = None
    current = {'planner': None}     # the oneshot planner in flight, stopped before the abort
    runs = []
    interrupted = {'sig': None}

    def on_signal(sig, _frame):
        interrupted['sig'] = sig
        raise KeyboardInterrupt

    old = {s: signal.signal(s, on_signal) for s in (signal.SIGINT, signal.SIGTERM)}
    env = planner_environment(args.ctl_env)
    confirmed = False
    try:
        if not stack.services_available(10.0):
            raise RuntimeError(f'{ABORT_SERVICE} not available: is the stack up?')
        if stack.wait_state(STOPPED_STATES + ('LOADED',) + ACTIVE_STATES, 5.0) is None:
            log(f'warning: no {STATUS_TOPIC} message within 5 s')

        def factory(command):
            return Planner(planner_argv('oneshot', command, args.planner_pythonpath),
                           env, stdin_pipe=False)

        if args.mode == 'persistent':
            persistent = Planner(planner_argv('persistent', '', args.planner_pythonpath),
                                 env, stdin_pipe=True)
            if persistent.wait_for(lambda ln: classify_line(ln) is not None and
                                   classify_line(ln)[0] == 'interactive_ready',
                                   0, timeout=120.0) is None:
                raise RuntimeError('persistent planner never became ready')
            meta['persistent_ready_s'] = persistent.lines[-1][0] - persistent.t_spawn
        commands = list(args.cmd or DEFAULT_SUITE)
        for i in range(args.runs):
            for command in commands:
                tag = 'cold' if (args.cold_first and i == 0 and command == commands[0]) \
                    else 'warm'
                run = run_once(stack, factory, args.mode, command, i, tag,
                               args.run_timeout, log, persistent, current)
                if not safe_abort(stack, log):
                    run['error'] = run['error'] or 'abort not confirmed'
                    runs.append(run)
                    raise RuntimeError('abort not confirmed: stopping the session')
                runs.append(run)
                slim = {k: v for k, v in run.items() if k != 'output_tail'}
                log('RUN ' + json.dumps(slim, ensure_ascii=False))
                if args.out:
                    _write(args.out, meta, runs)
        confirmed = True
    except KeyboardInterrupt:
        log(f'interrupted ({interrupted["sig"]}): aborting')
    finally:
        # A second Ctrl-C must not skip the abort below.
        for s in old:
            signal.signal(s, signal.SIG_IGN)
        # Stop the planner FIRST, so it cannot call start_mission after the abort.
        for planner in (persistent, current['planner']):
            if planner is not None:
                planner.stop()
        ok = safe_abort(stack, log)
        confirmed = confirmed and ok
        final_state = stack.state()
        for s, h in old.items():
            signal.signal(s, h)
        stack.close()
    session = {'meta': meta, 'runs': runs, 'summary': summarize(runs), 'complete': confirmed,
               'final_state': final_state}
    if args.out:
        _write(args.out, meta, runs, session)
    return session


def _write(path, meta, runs, session=None):
    doc = session or {'meta': meta, 'runs': runs, 'summary': summarize(runs), 'complete': False}
    tmp = path + '.tmp'
    with open(tmp, 'w') as fh:
        json.dump(doc, fh, indent=1, ensure_ascii=False)
    os.replace(tmp, path)


def build_parser():
    p = argparse.ArgumentParser(
        description='LIVE stack: planner command -> mission RUNNING, per stage. '
                    'Starts real missions: needs --car-is-safe.')
    p.add_argument('--cmd', action='append', help='a command (repeatable)')
    p.add_argument('--suite', action='store_true',
                   help='the built-in 3 commands (default when no --cmd)')
    p.add_argument('--runs', type=int, default=10, help='rounds over the commands (default 10)')
    p.add_argument('--cold-first', action='store_true',
                   help='tag the first run cold (restart the stack yourself beforehand)')
    p.add_argument('--mode', choices=('oneshot', 'persistent'), default='oneshot')
    p.add_argument('--flow', choices=('oneshot', 'persistent', 'production'),
                   help='oneshot / persistent (as --mode), or production: the operator chain '
                        'through f1tenth_ctl (send, planner-log, mission-load, mission-start) '
                        'against the planner unit it runs')
    p.add_argument('--ctl', default=CTL_BIN_DEFAULT, help='production: the f1tenth_ctl CLI')
    p.add_argument('--planner-log', default=PLANNER_LOG_DEFAULT,
                   help="production: the planner unit's log (f1tenth_ctl logs/planner.log)")
    p.add_argument('--plan-wait', type=float, default=0.0,
                   help='production: fetch the plan with `planner-log --wait S` right after '
                        'the send (the bot with a long-poll), not after the log tail saw it')
    p.add_argument('--plan-poll', type=float, default=0.0,
                   help="production: emulate the bot's polling: sleep S, planner-log, repeat "
                        '(the bot uses 2)')
    p.add_argument('--move-timeout', type=float, default=10.0,
                   help='production: seconds after RUNNING to wait for the first move')
    p.add_argument('--variant-runs', type=int, default=0,
                   help='production: extra runs with --variant-pause-s before load and start')
    p.add_argument('--variant-pause-s', type=float, default=3.0)
    p.add_argument('--not-ready-start', choices=('fail', 'stop'), default='fail',
                   help="production: when mission-start refuses 'not READY', fail the run "
                        "(default) or end it there, timed up to the refusal (ready_cli); "
                        'READY is never bypassed')
    p.add_argument('--out', help='results JSON (rewritten after every run)')
    p.add_argument('--budget-s', type=float,
                   help='exit 1 when the median total_to_running exceeds this')
    p.add_argument('--run-timeout', type=float, default=90.0, help='per run, seconds')
    p.add_argument('--ctl-env', default=os.environ.get('F1T_CTL_ENV', CTL_ENV_DEFAULT),
                   help='f1tenth_ctl env file the planner is started with')
    p.add_argument('--planner-pythonpath', default='',
                   help='prepended to the planner PYTHONPATH (an llm source tree, to time '
                        'code the image does not have yet)')
    p.add_argument('--car-is-safe', action='store_true',
                   help='REQUIRED: wheels lifted / on blocks, or motor output disabled')
    return p


def main(argv=None, session_fn=None) -> int:
    args = build_parser().parse_args(argv)
    refusal = check_safety(args.car_is_safe)
    if refusal:
        print(refusal, file=sys.stderr)
        return 2
    if args.cmd and args.suite:
        print('use --cmd or --suite, not both', file=sys.stderr)
        return 2
    if args.flow in ('oneshot', 'persistent'):
        args.mode = args.flow
    args.planner_log = os.path.expanduser(args.planner_log)
    pid_file = os.environ.get('F1T_BENCH_PID_FILE')
    if pid_file:
        with open(pid_file, 'w') as fh:
            fh.write(str(os.getpid()))
    session = (session_fn or run_session)(args)
    print()
    if session.get('flow') == 'production':
        for variant, summary in session['summaries'].items():
            print(f'### {variant}')
            print(markdown_table(summary, PROD_DELTAS, PROD_STAGES))
            print()
    else:
        print(markdown_table(session['summary']))
    print()
    print('attempts per run: ' + ', '.join(
        f"{r['run']}:{r['command'][:24]}={r.get('attempts')}" for r in session['runs']))
    if not session['complete']:
        return 3
    if args.budget_s is not None:
        key = 'total_to_move' if session.get('flow') == 'production' else 'total_to_running'
        med = (session['summary']['overall'].get(key) or {}).get('median')
        if med is None or med > args.budget_s:
            print(f'BUDGET EXCEEDED: median {key} {med} > {args.budget_s} s')
            return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
