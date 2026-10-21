"""
The car's own load, read by the gateway: CPU per core, GPU, pinned processes.

From /proc and sysfs only, read-only: no tegrastats, no jtop, no psutil, so
it needs no sudo, no jtop group and nothing installed. Gateway.sysstats_loop
samples it at 1 Hz while a phone is connected, and the status message carries
the last sample (status_payload 'system').

- CPU: /proc/stat deltas, in total and per core; iowait counts as idle, as in
  top and psutil. The cores are /sys/devices/system/cpu/present, online or not
  by .../online. An offline core has no line in /proc/stat and no %.
- GPU: the Jetson's GPU 'load' file, per-mille: the first of GPU_LOAD_PATHS
  that reads, else the 'device/load' of a devfreq device named like a GPU.
  The frequency is that devfreq's cur_freq (max_freq is the power mode's cap).
  Found on the Orin (L4T R36.5, 2026-10-22):
  /sys/devices/platform/bus@0/17000000.gpu/load and
  /sys/class/devfreq/17000000.gpu. Without a Jetson GPU (linus), nvidia-smi
  if it is installed, run here (a pool thread, never the event loop) with a
  1 s timeout, so at most once per sample; else n/a. One stuck in the driver
  (it outlives its SIGKILL) is left behind and nvidia-smi is off from then on.
- Pinned processes, every PROCS_PERIOD_S: a process whose Cpus_allowed_list
  leaves out an online core (the launch files' `taskset -c` prefix,
  scripts/check_cpu_pinning.py). Kernel threads are left out: the per-CPU
  ones (ksoftirqd/N, migration/N, ...) are pinned by nature, 123 of them on
  the idle Orin, and would bury the stack's own. A process' CPU % is its
  utime + stime over the period, 100 = one core, as in top. Any process can go
  away between two reads: it is then skipped, never an error. So is one on its
  way out (a zombie, or no command line any more), which would otherwise show
  up for one scan under its 15-character comm.
"""

import glob
import os
import re
import shutil
import subprocess
import time
from typing import Dict, List, Optional, Tuple

PROCS_PERIOD_S = 5.0
STALE_S = 5.0                    # an older sample is not sent: nobody has been sampling
NVIDIA_SMI_TIMEOUT_S = 1.0
NVIDIA_SMI_RETRY_S = 30.0        # after a failure or a timeout
NVIDIA_SMI_KILL_WAIT_S = 0.5     # for it to die after the timeout's SIGKILL
CLK_TCK = os.sysconf('SC_CLK_TCK')

GPU_LOAD_PATHS = (
    '/sys/devices/platform/bus@0/17000000.gpu/load',    # Orin, L4T R36 (measured)
    '/sys/devices/platform/17000000.gpu/load',          # the same device, R36
    '/sys/devices/17000000.ga10b/load',                 # Orin, L4T R35
    '/sys/devices/gpu.0/load',                          # Xavier, Orin R35
    '/sys/devices/17000000.gv11b/load',                 # Xavier
    '/sys/devices/17000000.gp10b/load',                 # TX2
    '/sys/devices/57000000.gpu/load',                   # Nano, TX1
)
GPU_DEVFREQ_NAMES = ('gpu', 'ga10b', 'gv11b', 'gp10b', 'gm20b')
INTERPRETERS = re.compile(r'(python[0-9.]*|sh|bash|dash|zsh|perl|node)$')
COMMAND_OPT = re.compile(r'-[A-Za-z]*c$')    # -c, bash -lc: a command string follows
VALUE_OPTS = ('-W', '-X')                    # python options whose value is not the script


# ---- parsing (pure: the tests feed it fixture files) ---------------------------------------

def parse_cpu_list(text: str) -> List[int]:
    """Return the cores of a kernel cpu list: '0-3,6' -> [0, 1, 2, 3, 6]."""
    cores = set()
    for part in text.strip().split(','):
        part = part.strip()
        if not part:
            continue
        lo, dash, hi = part.partition('-')
        first = int(lo)
        last = int(hi) if dash else first
        if last < first:
            raise ValueError(f'bad cpu range {part!r}')
        cores.update(range(first, last + 1))
    return sorted(cores)


def format_cpu_list(cores: List[int]) -> str:
    """Return [0, 1, 2, 3, 6] as '0-3,6', the way the kernel writes it."""
    out, run = [], []
    for c in sorted(set(cores)):
        if run and c == run[-1] + 1:
            run.append(c)
            continue
        if run:
            out.append(f'{run[0]}-{run[-1]}' if len(run) > 1 else str(run[0]))
        run = [c]
    if run:
        out.append(f'{run[0]}-{run[-1]}' if len(run) > 1 else str(run[0]))
    return ','.join(out)


def parse_proc_stat(text: str) -> Dict[str, Tuple[int, int]]:
    """Return /proc/stat's cpu lines as {'cpu': (busy, total), 'cpu0': ...} in clock ticks."""
    out = {}
    for line in text.splitlines():
        if not line.startswith('cpu'):
            continue
        parts = line.split()
        ticks = [int(x) for x in parts[1:9]]        # user .. steal; guest is inside user
        total = sum(ticks)
        out[parts[0]] = (total - sum(ticks[3:5]), total)    # idle + iowait are not busy
    return out


def cpu_percent(prev: Optional[Tuple[int, int]], cur: Optional[Tuple[int, int]]
                ) -> Optional[float]:
    """Return the busy % between two (busy, total) samples, None without both."""
    if prev is None or cur is None:
        return None
    busy, total = cur[0] - prev[0], cur[1] - prev[1]
    if total <= 0:
        return None
    return round(min(100.0, max(0.0, 100.0 * busy / total)), 1)


def parse_gpu_load(text: str) -> float:
    """Return the Jetson GPU 'load' file (per-mille, '347') as % (34.7)."""
    return round(min(1000, max(0, int(text.strip()))) / 10.0, 1)


def parse_nvidia_smi(text: str) -> Optional[Dict]:
    """Return the first GPU of `--query-gpu=utilization.gpu,clocks.gr,clocks.max.gr`."""
    lines = [ln for ln in text.splitlines() if ln.strip()]
    if not lines:
        return None

    def num(v: str) -> Optional[float]:
        try:
            return float(v)
        except ValueError:                  # '[N/A]', '[Not Supported]'
            return None
    fields = [num(v.strip()) for v in lines[0].split(',')] + [None, None, None]
    mhz = [None if f is None else int(round(f)) for f in fields[1:3]]
    return {'percent': fields[0], 'freq_mhz': mhz[0], 'max_mhz': mhz[1]}


def parse_status(text: str) -> Dict[str, str]:
    """Return /proc/<pid>/status as {key: value}."""
    out = {}
    for line in text.splitlines():
        key, sep, value = line.partition(':')
        if sep:
            out[key] = value.strip()
    return out


def parse_pid_stat(text: str) -> Tuple[int, int]:
    """Return (utime + stime, starttime) from /proc/<pid>/stat, in clock ticks."""
    rest = text[text.rindex(')') + 2:].split()     # comm may hold spaces and ')'
    return int(rest[11]) + int(rest[12]), int(rest[19])


def friendly_name(cmdline: List[str], comm: str) -> str:
    """
    Name a process the way the stack does.

    The ROS node name (`__node:=` from launch's `-r`) first, then a launch
    file (`ros2 launch pkg x.launch.py`), then the program: the script an
    interpreter runs (sh -c: the command's first word; python -c: python),
    else the executable. The kernel's 15-character comm when there is no
    command line.
    """
    for arg in cmdline:
        if '__node:=' in arg:
            return arg.split('__node:=', 1)[1]
    if not cmdline:
        return comm
    args = list(cmdline)
    if len(args) == 1 and ' ' in args[0]:          # a process that rewrote its title
        args = args[0].split()
    if 'launch' in args and 'ros2' in [os.path.basename(a) for a in args]:
        after = args[args.index('launch') + 1:]
        files = [a for a in after if a.endswith(('.py', '.xml', '.yaml'))]
        if files:
            return os.path.basename(files[0])
    exe = os.path.basename(args[0]).rstrip(':')
    if INTERPRETERS.match(exe):
        rest = iter(args[1:])
        for arg in rest:
            if arg == '-m':
                return next(rest, exe)
            if COMMAND_OPT.match(arg):            # sh -c 'exe args': exe, never the whole string
                words = next(rest, '').split()
                if words[:1] == ['exec']:
                    words = words[1:]
                if exe.startswith('python') or not words:
                    return exe
                return os.path.basename(words[0])
            if arg in VALUE_OPTS:
                next(rest, None)
                continue
            if arg.startswith('-'):
                continue
            return os.path.basename(arg)
    return exe or comm


def _read(path: str) -> str:
    with open(path) as f:
        return f.read()


def scan_procs(proc: str, online: List[int]) -> Tuple[List[Dict], int, int]:
    """
    Return (pinned, unpinned, kernel_threads) from /proc/<pid>/status.

    pinned: [{pid, name, cores, ticks, start}] for every process that may not
    run on some online core. A process that goes away half way (status read,
    stat gone), or is on its way out, is skipped: it is in none of the three.
    """
    want = set(online)
    pinned, unpinned, kthreads = [], 0, 0
    try:
        pids = [int(d) for d in os.listdir(proc) if d.isdigit()]
    except OSError:
        return [], 0, 0
    for pid in pids:
        base = os.path.join(proc, str(pid))
        try:
            st = parse_status(_read(base + '/status'))
            allowed = st.get('Cpus_allowed_list')
            if allowed is None or st.get('State', '')[:1] in ('Z', 'X'):   # zombie, dead
                continue
            if pid == 2 or st.get('PPid') == '2':   # kthreadd and its children
                kthreads += 1
                continue
            cores = parse_cpu_list(allowed)
            if want <= set(cores):
                unpinned += 1
                continue
            ticks, start = parse_pid_stat(_read(base + '/stat'))
            with open(base + '/cmdline', 'rb') as f:
                raw = f.read()
        except (OSError, ValueError, IndexError):  # gone between two reads, or a zombie
            continue
        cmdline = [a.decode('utf-8', 'replace') for a in raw.split(b'\0') if a]
        if not cmdline:     # exiting: its memory, and with it the command line, already gone
            continue
        pinned.append({'pid': pid, 'name': friendly_name(cmdline, st.get('Name', '?')),
                       'cores': cores, 'ticks': ticks, 'start': start})
    return pinned, unpinned, kthreads


# ---- the sampler -------------------------------------------------------------------------

class SysStats:
    """Sample CPU, GPU and pinned processes; keep the last view for the status message."""

    def __init__(self, proc: str = '/proc', sys: str = '/sys', gpu_paths=GPU_LOAD_PATHS,
                 nvidia_smi: Optional[str] = 'auto', clock=time.monotonic, wall=time.time):
        self.proc, self.sys = proc, sys
        self.gpu_paths = gpu_paths
        self.smi = shutil.which('nvidia-smi') if nvidia_smi == 'auto' else nvidia_smi
        self.clock, self.wall = clock, wall
        self._gpu: Optional[Dict] = None         # {'load': path, 'devfreq': dir}, found once
        self._smi_next = 0.0
        self._last_t: Optional[float] = None
        self._prev_stat: Dict = {}
        self._procs_t: Optional[float] = None
        self._prev_ticks: Dict[Tuple[int, int], int] = {}
        self._procs: Dict = {'pinned': [], 'unpinned': None, 'kernel_threads': None}
        self.view: Optional[Dict] = None
        self.view_t: Optional[float] = None

    def snapshot(self) -> Optional[Dict]:
        """Return the last sample, or None when nobody has sampled for STALE_S."""
        view, t = self.view, self.view_t
        if view is None or t is None or self.clock() - t > STALE_S:
            return None
        return view

    def sample(self) -> Dict:
        now = self.clock()
        if self._last_t is not None and now - self._last_t > STALE_S:
            # Nobody watched for a while: a delta over that gap is an average, not a load.
            self._prev_stat, self._procs_t, self._prev_ticks = {}, None, {}
        self._last_t = now
        cpu = self._cpu()
        gpu = self._gpu_sample(now)
        if self._procs_t is None or now - self._procs_t >= PROCS_PERIOD_S - 0.05:
            self._scan(now, [c['id'] for c in cpu['cores'] if c['online']])
        view = {'t': self.wall(), 'cpu_percent': cpu['percent'], 'cores': cpu['cores'],
                'gpu': gpu, **self._procs}
        self.view, self.view_t = view, now
        return view

    # ---- CPU -------------------------------------------------------------------------------

    def _cpu_list(self, name: str) -> Optional[List[int]]:
        try:
            return parse_cpu_list(_read(f'{self.sys}/devices/system/cpu/{name}'))
        except (OSError, ValueError):
            return None

    def _cpu(self) -> Dict:
        try:
            cur = parse_proc_stat(_read(self.proc + '/stat'))
        except (OSError, ValueError):
            cur = {}
        prev, self._prev_stat = self._prev_stat, cur
        listed = sorted(int(k[3:]) for k in cur if k != 'cpu')
        present = self._cpu_list('present') or listed
        online = set(self._cpu_list('online') or listed)
        cores = []
        for c in present:
            on = c in online
            pct = cpu_percent(prev.get(f'cpu{c}'), cur.get(f'cpu{c}')) if on else None
            cores.append({'id': c, 'online': on, 'percent': pct})
        return {'percent': cpu_percent(prev.get('cpu'), cur.get('cpu')), 'cores': cores}

    # ---- GPU -------------------------------------------------------------------------------

    def _find_gpu(self) -> Dict:
        load = None
        for path in self.gpu_paths:
            try:
                parse_gpu_load(_read(path))
            except (OSError, ValueError):
                continue
            load = path
            break
        devfreq = None
        for d in sorted(glob.glob(f'{self.sys}/class/devfreq/*')):
            if any(k in os.path.basename(d).lower() for k in GPU_DEVFREQ_NAMES):
                devfreq = d
                if load is None and os.path.exists(d + '/device/load'):
                    load = d + '/device/load'
                break
        return {'load': load, 'devfreq': devfreq}

    def _mhz(self, path: str) -> Optional[int]:
        try:
            return int(round(int(_read(path).strip()) / 1e6))
        except (OSError, ValueError):
            return None

    def _gpu_sample(self, now: float) -> Dict:
        if self._gpu is None:
            self._gpu = self._find_gpu()
        g = self._gpu
        out = {'percent': None, 'freq_mhz': None, 'max_mhz': None, 'source': None}
        if g['load']:
            try:
                out['percent'] = parse_gpu_load(_read(g['load']))
                out['source'] = g['load']
            except (OSError, ValueError):
                self._gpu = None                  # look again at the next sample
            if g['devfreq']:
                out['freq_mhz'] = self._mhz(g['devfreq'] + '/cur_freq')
                out['max_mhz'] = self._mhz(g['devfreq'] + '/max_freq')
            return out
        if self.smi and now >= self._smi_next:
            res = self._nvidia_smi()
            if res is None:
                self._smi_next = now + NVIDIA_SMI_RETRY_S
            else:
                out.update(res, source='nvidia-smi')
        return out

    def _nvidia_smi(self) -> Optional[Dict]:
        try:
            proc = subprocess.Popen(
                [self.smi, '--query-gpu=utilization.gpu,clocks.gr,clocks.max.gr',
                 '--format=csv,noheader,nounits'],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        except (OSError, subprocess.SubprocessError):
            return None
        try:
            out, _ = proc.communicate(timeout=NVIDIA_SMI_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            # Not subprocess.run: after the kill it waits for the child with no limit, and an
            # nvidia-smi stuck in the driver (D state) outlives SIGKILL. The sample, and the
            # pool thread the e-stop also uses, would never come back.
            proc.kill()
            try:
                proc.wait(timeout=NVIDIA_SMI_KILL_WAIT_S)
            except subprocess.TimeoutExpired:
                self.smi = None                   # left behind; no more nvidia-smi this run
            proc.stdout.close()
            return None
        return parse_nvidia_smi(out) if proc.returncode == 0 else None

    # ---- pinned processes ------------------------------------------------------------------

    def _scan(self, now: float, online: List[int]) -> None:
        pinned, unpinned, kthreads = scan_procs(self.proc, online)
        dt = None if self._procs_t is None else now - self._procs_t
        out = []
        for p in pinned:
            before = self._prev_ticks.get((p['pid'], p['start']))   # start: a reused pid is new
            pct = None
            if before is not None and dt:
                pct = round(max(0.0, 100.0 * (p['ticks'] - before) / (CLK_TCK * dt)), 1)
            out.append({'pid': p['pid'], 'name': p['name'], 'cores': p['cores'],
                        'cores_text': format_cpu_list(p['cores']), 'cpu_percent': pct})
        out.sort(key=lambda p: (p['cores'][0] if p['cores'] else -1, p['name'], p['pid']))
        self._prev_ticks = {(p['pid'], p['start']): p['ticks'] for p in pinned}
        self._procs_t = now
        self._procs = {'pinned': out, 'unpinned': unpinned, 'kernel_threads': kthreads}
