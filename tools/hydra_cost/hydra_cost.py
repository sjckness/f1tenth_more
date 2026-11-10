#!/usr/bin/env python3
"""hydra_cost -- what Hydra costs, sampled at 1 Hz, to CSV + a short markdown summary.

Per sample (long CSV: t_wall, t_rel, kind, entity, metric, value):
  system     cpu_pct, ram_used_mb, ram_total_mb
  proc       cpu_pct (100 = one core), rss_mb, threads       per matched process
  thread     cpu_pct                                          top threads of each matched process
  gpu        util_pct, mem_used_mb, mem_total_mb              per GPU (NVML, else nvidia-smi)
  gpu_proc   mem_mb, sm_pct (NVML only)                       per process on the GPU
  topic      hz, kbps (kB/s)                                  per watched topic
  graph      nodes / edges per layer, total_nodes, msg_kb     from /hydra/backend/dsg
  hydra      status_ok (1/0), obs_gap_s                       from /hydra/status
At exit (SIGINT/SIGTERM/--duration) it writes summary.md, and also every 60 s, so a killed run
still has one. With --hydra-log-dir it adds Hydra's own timing_stats.csv (written by Hydra when
it shuts down) to the summary.

    python3 tools/hydra_cost/hydra_cost.py [--out-dir DIR] [--duration S] [--no-ros]
    ros2 launch f1tenth_hydra hydra.launch.py hydra_cost:=true     # started with Hydra

Output: ~/.ros/hydra_cost/<stamp>/{samples.csv,summary.md}; ~/.ros/hydra_cost/latest points at
the newest (the mission logger copies it into the run folder, log_scene_graph:=true).
Run it on the host that runs Hydra: subscribing to the image topics pulls them to this host.
"""

import argparse
import csv
import datetime
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time

import psutil

DEFAULT_MATCH = [
    'hydra_ros_node', 'closed_set_node', 'hydra_visualizer_node', 'odom_to_tf', 'rviz2',
    'hydra_dsg_viewer', 'sim/main.py', 'mission_logger', 'hydra_cost.py',
]
DEFAULT_TOPICS = [
    '/camera/image_raw', '/zed2/zed_node/depth/depth_registered',
    '/sim/ground_truth/left/semantic', '/hydra_seg/semantic/image_raw',
    '/hydra/backend/dsg', '/hydra/backend/dsg_mesh', '/hydra_visualizer/graph',
    '/sim/ground_truth/odom', '/tf',
]
IMAGE_TOPICS = DEFAULT_TOPICS[:4]
DSG_TOPIC = '/hydra/backend/dsg'
STATUS_TOPIC = '/hydra/status'
TOP_THREADS = 8
SHELLS = ('bash', 'sh', 'zsh', 'dash', 'timeout', 'env')


def default_out_dir():
    stamp = datetime.datetime.now().strftime('%Y%m%dT%H%M%S')
    return os.path.join(os.path.expanduser('~'), '.ros', 'hydra_cost', stamp)


# ------------------------------------------------------------------ processes
class ProcSampler:
    """CPU (per process and per thread), RSS and threads of processes matching patterns."""

    def __init__(self, patterns):
        self.patterns = patterns
        self.procs = {}          # pid -> (psutil.Process, label)
        self.thread_times = {}   # (pid, tid) -> cpu seconds at the last sample
        self.last = time.monotonic()

    def _label(self, proc):
        try:
            if proc.name() in SHELLS:      # `bash -c "... hydra_ros_node ..."` wrappers
                return None
            cmd = ' '.join(proc.cmdline())
        except (psutil.Error, OSError):
            return None
        for pat in self.patterns:
            if pat in cmd:
                return f'{pat.split("/")[-1]}[{proc.pid}]'
        return None

    def refresh(self):
        for proc in psutil.process_iter():
            if proc.pid in self.procs:
                continue
            label = self._label(proc)
            if label:
                try:
                    proc.cpu_percent(None)       # prime
                    self.procs[proc.pid] = (proc, label)
                except psutil.Error:
                    pass

    def sample(self):
        now = time.monotonic()
        dt = max(now - self.last, 1e-3)
        self.last = now
        rows = []
        for pid, (proc, label) in list(self.procs.items()):
            try:
                with proc.oneshot():
                    rows += [('proc', label, 'cpu_pct', proc.cpu_percent(None)),
                             ('proc', label, 'rss_mb', proc.memory_info().rss / 2 ** 20),
                             ('proc', label, 'threads', proc.num_threads())]
                    threads = []
                    for th in proc.threads():
                        total = th.user_time + th.system_time
                        prev = self.thread_times.get((pid, th.id))
                        self.thread_times[(pid, th.id)] = total
                        if prev is not None:
                            threads.append((100.0 * (total - prev) / dt, th.id))
                threads.sort(reverse=True)
                for pct, tid in threads[:TOP_THREADS]:
                    rows.append(('thread', f'{label}/{_thread_name(pid, tid)}:{tid}',
                                 'cpu_pct', pct))
            except psutil.Error:
                del self.procs[pid]
        return rows


def _thread_name(pid, tid):
    try:
        with open(f'/proc/{pid}/task/{tid}/comm', 'r', encoding='utf-8') as f:
            return f.read().strip()
    except OSError:
        return '?'


# ------------------------------------------------------------------ GPU
class GpuSampler:
    def __init__(self):
        self.nvml = None
        self.last_ts = 0
        try:
            import pynvml
            pynvml.nvmlInit()
            self.nvml = pynvml
            self.handles = [pynvml.nvmlDeviceGetHandleByIndex(i)
                            for i in range(pynvml.nvmlDeviceGetCount())]
        except Exception:  # noqa: BLE001 - no pynvml / no driver: nvidia-smi or nothing
            self.nvml = None
        self.smi = shutil.which('nvidia-smi')
        self.source = 'nvml' if self.nvml else ('nvidia-smi' if self.smi else 'none')

    def sample(self):
        if self.nvml:
            return self._nvml()
        if self.smi:
            return self._smi()
        return []

    def _nvml(self):
        n, rows = self.nvml, []
        for i, h in enumerate(self.handles):
            util = n.nvmlDeviceGetUtilizationRates(h)
            mem = n.nvmlDeviceGetMemoryInfo(h)
            rows += [('gpu', f'gpu{i}', 'util_pct', util.gpu),
                     ('gpu', f'gpu{i}', 'mem_used_mb', mem.used / 2 ** 20),
                     ('gpu', f'gpu{i}', 'mem_total_mb', mem.total / 2 ** 20)]
            procs = {}
            for getter in (n.nvmlDeviceGetComputeRunningProcesses,
                           n.nvmlDeviceGetGraphicsRunningProcesses):
                try:
                    for p in getter(h):
                        procs[p.pid] = max(procs.get(p.pid, 0), (p.usedGpuMemory or 0))
                except n.NVMLError:
                    pass
            sm = {}
            try:
                for s in n.nvmlDeviceGetProcessUtilization(h, self.last_ts):
                    sm[s.pid] = max(sm.get(s.pid, 0), s.smUtil)
                self.last_ts = int(time.time() * 1e6) - 1_000_000
            except n.NVMLError:
                pass
            for pid in set(procs) | set(sm):
                name = f'{_pname(pid)}[{pid}]'
                if pid in procs:
                    rows.append(('gpu_proc', name, 'mem_mb', procs[pid] / 2 ** 20))
                if pid in sm:
                    rows.append(('gpu_proc', name, 'sm_pct', sm[pid]))
        return rows

    def _smi(self):
        rows = []
        try:
            out = subprocess.run(
                [self.smi, '--query-gpu=index,utilization.gpu,memory.used,memory.total',
                 '--format=csv,noheader,nounits'], capture_output=True, text=True, timeout=5)
            for line in out.stdout.strip().splitlines():
                i, u, used, total = [x.strip() for x in line.split(',')]
                rows += [('gpu', f'gpu{i}', 'util_pct', float(u)),
                         ('gpu', f'gpu{i}', 'mem_used_mb', float(used)),
                         ('gpu', f'gpu{i}', 'mem_total_mb', float(total))]
            out = subprocess.run(
                [self.smi, '--query-compute-apps=pid,used_memory', '--format=csv,noheader,nounits'],
                capture_output=True, text=True, timeout=5)
            for line in out.stdout.strip().splitlines():
                pid, mem = [x.strip() for x in line.split(',')]
                rows.append(('gpu_proc', f'{_pname(int(pid))}[{pid}]', 'mem_mb', float(mem)))
        except (OSError, ValueError, subprocess.SubprocessError):
            pass
        return rows


def _pname(pid):
    try:
        return psutil.Process(pid).name()
    except psutil.Error:
        return '?'


# ------------------------------------------------------------------ ROS
class RosSampler:
    """Topic rates/bytes, the graph size per layer and Hydra's status, from one rclpy node."""

    def __init__(self, topics):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        self.rclpy = rclpy
        rclpy.init(args=None)
        self.node = rclpy.create_node('hydra_cost')
        self.qos = qos_profile_sensor_data
        self.topics = topics
        self.subs = {}
        self.lock = threading.Lock()
        self.count = {t: [0, 0] for t in topics}       # msgs, bytes since the last sample
        self.graph = {}
        self.status = None
        self.dsg = None
        try:
            import spark_dsg
            self.dsg = spark_dsg
        except ImportError:
            pass
        sys.path.insert(0, _f1tenth_hydra_src())
        from std_msgs.msg import String
        self.node.create_subscription(String, STATUS_TOPIC, self._on_status, 10)
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.thread.start()

    def _on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except ValueError:
            self.status = {'status': 'UNPARSEABLE'}

    def _counter(self, topic):
        def cb(raw):
            with self.lock:
                c = self.count[topic]
                c[0] += 1
                c[1] += len(raw)
        return cb

    def _on_dsg(self, msg):
        data = msg.layer_contents.tobytes()
        summary = {'msg_kb': len(data) / 1024.0}
        if self.dsg is not None:
            try:
                from f1tenth_hydra.dsg_tools import graph_summary
                summary.update(graph_summary(self.dsg.DynamicSceneGraph.from_binary(data)))
            except Exception as exc:  # noqa: BLE001
                summary['error'] = str(exc)
        with self.lock:
            self.graph = summary
            c = self.count.get(DSG_TOPIC)
            if c is not None:
                c[0] += 1
                c[1] += len(data)

    def discover(self):
        """Subscribe to watched topics once they exist (their type is needed)."""
        from rosidl_runtime_py.utilities import get_message
        types = dict(self.node.get_topic_names_and_types())
        for topic in self.topics:
            if topic in self.subs or topic not in types:
                continue
            try:
                msg_type = get_message(types[topic][0])
            except (AttributeError, ModuleNotFoundError, ValueError):
                continue
            if topic == DSG_TOPIC:
                self.subs[topic] = self.node.create_subscription(msg_type, topic, self._on_dsg, 1)
            else:
                self.subs[topic] = self.node.create_subscription(
                    msg_type, topic, self._counter(topic), self.qos, raw=True)

    def sample(self, dt):
        rows = []
        with self.lock:
            for topic, (n, b) in self.count.items():
                if topic in self.subs:
                    rows += [('topic', topic, 'hz', n / dt), ('topic', topic, 'kbps', b / dt / 1024)]
                self.count[topic] = [0, 0]
            graph, self.graph = self.graph, {}
        if graph:
            rows.append(('graph', 'dsg', 'msg_kb', graph['msg_kb']))
            if 'nodes' in graph:
                rows.append(('graph', 'dsg', 'total_nodes', graph['nodes']))
                rows.append(('graph', 'dsg', 'interlayer_edges', graph['interlayer_edges']))
                for layer, c in graph['layers'].items():
                    rows += [('graph', layer, 'nodes', c['nodes']), ('graph', layer, 'edges', c['edges'])]
        if self.status is not None:
            st = self.status
            rows.append(('hydra', 'status', 'status_ok', 1.0 if st.get('status') == 'NOMINAL' else 0.0))
            m = re.search(r'([0-9.eE+-]+) s', str(st.get('note', '')))
            if m:
                rows.append(('hydra', 'status', 'obs_gap_s', float(m.group(1))))
        return rows

    def close(self):
        try:
            self.node.destroy_node()
            if self.rclpy.ok():
                self.rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


def _f1tenth_hydra_src():
    here = os.path.dirname(os.path.realpath(__file__))
    return os.path.join(os.path.dirname(os.path.dirname(here)), 'src', 'f1tenth_hydra')


# ------------------------------------------------------------------ summary
def summarize(csv_path, out_md, meta, hydra_log_dir=None):
    """Short markdown summary of samples.csv."""
    series = {}
    with open(csv_path, 'r', encoding='utf-8') as f:
        for row in csv.DictReader(f):
            key = (row['kind'], row['entity'], row['metric'])
            series.setdefault(key, []).append(float(row['value']))
    duration = meta.get('duration_s', 0.0)

    def stat(kind, metric):
        out = []
        for (k, ent, met), vals in series.items():
            if k == kind and met == metric:
                out.append((ent, sum(vals) / len(vals), max(vals), vals[-1]))
        return sorted(out, key=lambda r: -r[1])

    lines = [f'# hydra_cost summary ({meta.get("start", "?")})', '',
             f'{meta.get("samples", 0)} samples over {duration:.0f} s on {meta.get("host", "?")}; '
             f'GPU source {meta.get("gpu_source", "?")}; CSV `{os.path.basename(csv_path)}`.', '']
    sysc, ram = stat('system', 'cpu_pct'), stat('system', 'ram_used_mb')
    if sysc:
        lines.append(f'System: CPU mean {sysc[0][1]:.0f} % / max {sysc[0][2]:.0f} % of all cores; '
                     f'RAM max {ram[0][2] / 1024:.1f} GB.')
        lines.append('')
    procs = stat('proc', 'cpu_pct')
    if procs:
        rss = {e: mx for e, _, mx, _ in stat('proc', 'rss_mb')}
        lines += ['| process | CPU mean % | CPU max % | RSS max MB |', '|---|---|---|---|']
        lines += [f'| {e} | {m:.0f} | {mx:.0f} | {rss.get(e, 0):.0f} |' for e, m, mx, _ in procs]
        lines.append('')
    threads = stat('thread', 'cpu_pct')[:10]
    if threads:
        lines += ['Top threads (mean CPU %): ' + ', '.join(f'{e} {m:.0f}' for e, m, _, _ in threads), '']
    gpus = stat('gpu', 'util_pct')
    if gpus:
        mem = {e: mx for e, _, mx, _ in stat('gpu', 'mem_used_mb')}
        lines += [f'GPU {e}: util mean {m:.0f} % / max {mx:.0f} %, VRAM max {mem.get(e, 0):.0f} MB'
                  for e, m, mx, _ in gpus]
        gp = stat('gpu_proc', 'mem_mb')
        if gp:
            lines.append('VRAM per process (max MB): ' + ', '.join(f'{e} {mx:.0f}' for e, _, mx, _ in gp))
        lines.append('')
    hz = stat('topic', 'hz')
    if hz:
        kb = {e: m for e, m, _, _ in stat('topic', 'kbps')}
        lines += ['| topic | Hz mean | kB/s mean |', '|---|---|---|']
        lines += [f'| `{e}` | {m:.1f} | {kb.get(e, 0):.0f} |' for e, m, _, _ in sorted(hz)]
        lines.append('')
    nodes = stat('graph', 'nodes')
    if nodes:
        edges = {e: last for e, _, _, last in stat('graph', 'edges')}
        lines += ['| layer | nodes (last) | nodes (max) | edges (last) |', '|---|---|---|---|']
        lines += [f'| {e} | {last:.0f} | {mx:.0f} | {edges.get(e, 0):.0f} |'
                  for e, _, mx, last in sorted(nodes)]
        kb = stat('graph', 'msg_kb')
        if kb:
            lines.append(f'\nGraph message: {kb[0][3]:.0f} kB last, {kb[0][2]:.0f} kB max.')
        lines.append('')
    ok = stat('hydra', 'status_ok')
    if ok:
        lines.append(f'Hydra status NOMINAL in {100 * ok[0][1]:.0f} % of samples.')
        lines.append('')
    timing = os.path.join(hydra_log_dir, 'timing_stats.csv') if hydra_log_dir else None
    if timing and os.path.isfile(timing):
        lines += ['Hydra timing (`timing_stats.csv`, first rows):', '', '```']
        with open(timing, 'r', encoding='utf-8') as f:
            lines += [ln.rstrip() for ln in f.readlines()[:25]]
        lines += ['```', '']
    elif hydra_log_dir:
        lines += [f'Hydra timing: no `timing_stats.csv` in `{hydra_log_dir}` yet (Hydra writes it '
                  'when it shuts down).', '']
    with open(out_md, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')


# ------------------------------------------------------------------ main
def main(argv=None):
    p = argparse.ArgumentParser(description='Hydra cost sampler (1 Hz CSV + markdown summary).')
    p.add_argument('--out-dir', default=None)
    p.add_argument('--period', type=float, default=1.0)
    p.add_argument('--duration', type=float, default=0.0, help='seconds, 0 = until SIGINT')
    p.add_argument('--match', action='append', default=[], help='extra process cmdline pattern')
    p.add_argument('--topic', action='append', default=[], help='extra topic to rate')
    p.add_argument('--no-image-topics', action='store_true',
                   help='do not subscribe to the image topics (Hydra on another host)')
    p.add_argument('--no-ros', action='store_true', help='processes, system and GPU only')
    p.add_argument('--hydra-log-dir', default=None, help="Hydra's log_path (timing_stats.csv)")
    a = p.parse_args(argv)

    out_dir = os.path.expanduser(a.out_dir) if a.out_dir else default_out_dir()
    os.makedirs(out_dir, exist_ok=True)
    latest = os.path.join(os.path.dirname(out_dir), 'latest')
    try:
        if os.path.islink(latest) or not os.path.exists(latest):
            if os.path.islink(latest):
                os.unlink(latest)
            os.symlink(out_dir, latest)
    except OSError:
        pass
    csv_path, md_path = os.path.join(out_dir, 'samples.csv'), os.path.join(out_dir, 'summary.md')

    procs = ProcSampler(DEFAULT_MATCH + a.match)
    gpu = GpuSampler()
    topics = [t for t in DEFAULT_TOPICS if not (a.no_image_topics and t in IMAGE_TOPICS)]
    topics += [t for t in a.topic if t not in topics]
    ros = None
    if not a.no_ros:
        try:
            ros = RosSampler(topics)
        except Exception as exc:  # noqa: BLE001 - no ROS: still sample the machine
            print(f'hydra_cost: ROS sampling off ({exc})', file=sys.stderr)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    meta = {'start': datetime.datetime.now().isoformat(timespec='seconds'),
            'host': os.uname().nodename, 'gpu_source': gpu.source, 'samples': 0}
    print(f'hydra_cost: {csv_path} (GPU via {gpu.source}, ROS {"on" if ros else "off"})', flush=True)
    psutil.cpu_percent(None)
    t0 = last = time.monotonic()
    wall0 = time.time()
    last_md = t0
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['t_wall', 't_rel', 'kind', 'entity', 'metric', 'value'])
        while not stop.is_set():
            stop.wait(max(0.0, a.period - (time.monotonic() - last)))
            now = time.monotonic()
            dt, last = now - last, now
            procs.refresh()
            vm = psutil.virtual_memory()
            rows = [('system', 'host', 'cpu_pct', psutil.cpu_percent(None)),
                    ('system', 'host', 'ram_used_mb', (vm.total - vm.available) / 2 ** 20),
                    ('system', 'host', 'ram_total_mb', vm.total / 2 ** 20)]
            rows += procs.sample() + gpu.sample()
            if ros:
                ros.discover()
                rows += ros.sample(dt)
            t_wall, t_rel = time.time(), now - t0
            for kind, ent, met, val in rows:
                w.writerow([f'{t_wall:.3f}', f'{t_rel:.3f}', kind, ent, met, f'{float(val):.4g}'])
            f.flush()
            meta['samples'] += 1
            meta['duration_s'] = t_rel
            if now - last_md > 60.0:
                summarize(csv_path, md_path, meta, a.hydra_log_dir)
                last_md = now
            if a.duration and t_rel >= a.duration:
                break
    if ros:
        ros.close()
    if a.hydra_log_dir:   # Hydra flushes timing at its own shutdown: give it a moment
        timing = os.path.join(a.hydra_log_dir, 'timing_stats.csv')
        for _ in range(20):
            if os.path.isfile(timing):
                shutil.copy2(timing, os.path.join(out_dir, 'hydra_timing_stats.csv'))
                break
            time.sleep(0.5)
    meta['wall_start'] = wall0
    summarize(csv_path, md_path, meta, a.hydra_log_dir)
    print(f'hydra_cost: {meta["samples"]} samples -> {out_dir}', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
