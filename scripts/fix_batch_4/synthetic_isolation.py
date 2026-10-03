#!/usr/bin/env python3
"""Fix batch 4 (H1): stack-free reproduction attempt of discovery isolation,
runnable unchanged on Jazzy and in the Humble container.

One round:
  1. start a fresh `fastdds discovery` server on 127.0.0.1:PORT;
  2. start a source process publishing /syn/in at 50 Hz;
  3. start W worker processes AT ONCE (the supervisor's startup burst): each
     subscribes /syn/in and republishes every message on /syn/out_<i>;
  4. after SETTLE s, a super-client checker counts messages per
     /syn/out_<i> for 5 s, and lists the node names it sees;
  5. a worker with no output is isolated (it may or may not be in the graph).
Every process is a plain Discovery Server client except the checker.

  synthetic_isolation.py OUT.jsonl ROUNDS [WORKERS] [PORT]

One JSON line per round. ROS_DOMAIN_ID is taken from the environment.
"""
import json
import os
import shutil
import signal
import subprocess
import sys
import time

WORKER = r'''
import sys, rclpy
from std_msgs.msg import String
i = sys.argv[1]
rclpy.init(); n = rclpy.create_node('syn_worker_' + i)
p = n.create_publisher(String, '/syn/out_' + i, 10)
n.create_subscription(String, '/syn/in', lambda m: p.publish(m), 10)
try:
    rclpy.spin(n)
except BaseException:
    pass
'''
SOURCE = r'''
import rclpy
from std_msgs.msg import String
rclpy.init(); n = rclpy.create_node('syn_source')
p = n.create_publisher(String, '/syn/in', 10)
n.create_timer(0.02, lambda: p.publish(String(data='x')))
try:
    rclpy.spin(n)
except BaseException:
    pass
'''
CHECK = r'''
import json, sys, time, rclpy
from std_msgs.msg import String
w = int(sys.argv[1])
rclpy.init(); n = rclpy.create_node('syn_checker')
c = {i: 0 for i in range(w)}
for i in range(w):
    n.create_subscription(String, '/syn/out_%d' % i,
                          lambda m, i=i: c.__setitem__(i, c[i] + 1), 10)
end = time.monotonic() + 5.0
while time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=0.05)
names = [x for x, _ in n.get_node_names_and_namespaces()]
print(json.dumps({'counts': c, 'names': names}))
'''


def main():
    out, rounds = sys.argv[1], int(sys.argv[2])
    workers = int(sys.argv[3]) if len(sys.argv) > 3 else 30
    port = int(sys.argv[4]) if len(sys.argv) > 4 else 11899
    settle = float(os.environ.get('SETTLE', '10'))
    env = dict(os.environ, ROS_DISCOVERY_SERVER=f'127.0.0.1:{port}')
    env.pop('ROS_SUPER_CLIENT', None)
    for r in range(rounds):
        srv = subprocess.Popen(['sh', shutil.which('fastdds'), 'discovery', '-i', '0', '-l', '127.0.0.1',
                                '-p', str(port)], stdout=subprocess.DEVNULL,
                               stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(1.5)
        procs = [subprocess.Popen([sys.executable, '-c', SOURCE], env=env,
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                  start_new_session=True)]
        for i in range(workers):
            procs.append(subprocess.Popen([sys.executable, '-c', WORKER, str(i)], env=env,
                                          stdout=subprocess.DEVNULL,
                                          stderr=subprocess.DEVNULL,
                                          start_new_session=True))
        time.sleep(settle)
        res = subprocess.run([sys.executable, '-c', CHECK, str(workers)],
                             env=dict(env, ROS_SUPER_CLIENT='TRUE'),
                             capture_output=True, text=True, timeout=60)
        try:
            data = json.loads(res.stdout.strip().splitlines()[-1])
        except (IndexError, ValueError):
            data = {'counts': {}, 'names': [], 'error': res.stderr[-500:]}
        alive = sum(p.poll() is None for p in procs)
        silent = [int(i) for i, v in data['counts'].items() if v == 0]
        rec = {'round': r, 'workers': workers, 'alive_procs': alive,
               'silent_workers': silent,
               'silent_in_graph': [i for i in silent if f'syn_worker_{i}' in data['names']],
               'nodes_seen': len(data['names'])}
        print(json.dumps(rec), flush=True)
        with open(out, 'a') as f:
            f.write(json.dumps(rec) + '\n')
        for p in procs + [srv]:
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        time.sleep(2.0)
        for p in procs + [srv]:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        time.sleep(1.0)


if __name__ == '__main__':
    main()
