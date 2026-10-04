"""N trials: start `ros2 launch probe.launch.py` in its own session, as
component_supervisor_node does, then os.killpg(pgid, SIGINT) as its
_stop_all_components() does (the node gets SIGINT from the group signal AND
from ros2 launch forwarding it). Count trials whose finally block completed."""
import os, signal, subprocess, sys, time
label, n = sys.argv[1], int(sys.argv[2])
d = os.path.dirname(os.path.abspath(__file__))
clean = tb = 0
for i in range(n):
    marker = f'/tmp/double_sigint_{label}_{i}'
    if os.path.exists(marker):
        os.remove(marker)
    env = dict(os.environ, PROBE_MARKER=marker)
    log = open(f'/tmp/double_sigint_{label}_{i}.log', 'w')
    p = subprocess.Popen(['ros2', 'launch', os.path.join(d, 'probe.launch.py')],
                         stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    time.sleep(3.0)
    os.killpg(p.pid, signal.SIGINT)
    try:
        p.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGKILL)
    log.close()
    ok = os.path.exists(marker)
    clean += ok
    tb += 'Traceback' in open(log.name).read()
print(f'{label}: {clean}/{n} clean finally, {tb}/{n} with a traceback')
