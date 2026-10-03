"""Time from os.killpg(pgid, SIGINT) to `ros2 launch` process exit, for the
same one-node launch file, as component_supervisor_node stops a component.
N trials; prints median / max seconds."""
import os, signal, statistics, subprocess, sys, time
label, n = sys.argv[1], int(sys.argv[2])
d = os.path.dirname(os.path.abspath(__file__))
ts = []
for i in range(n):
    env = dict(os.environ, PROBE_MARKER=f'/tmp/exit_time_{label}_{i}')
    p = subprocess.Popen(['ros2', 'launch', os.path.join(d, 'probe.launch.py')],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         env=env, start_new_session=True)
    time.sleep(3.0)
    t = time.monotonic()
    os.killpg(p.pid, signal.SIGINT)
    p.wait(timeout=30)
    ts.append(time.monotonic() - t)
print(f'{label}: ros2 launch exit after group SIGINT: median {statistics.median(ts):.2f} s, '
      f'max {max(ts):.2f} s, n={n}')
