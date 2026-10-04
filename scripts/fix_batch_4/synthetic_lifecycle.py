#!/usr/bin/env python3
"""Fix batch 4 (H1): stack-free reproduction of the slam_toolbox lifecycle
hang, runnable unchanged on Jazzy and in the Humble container.

One round: fresh `fastdds discovery` server on 127.0.0.1:PORT, a source and
W plain-client worker processes started at once (the startup burst, same
code as synthetic_isolation.py), and at the same moment
`ros2 launch synth/lc.launch.py` -- slam.launch.py's lifecycle pattern on a
minimal rclpy lifecycle node. After WAIT s: did the node log CONFIGURED and
ACTIVATED, and did launch_ros give up waiting for the change_state response?
Also counts silent workers (no republished output), as synthetic_isolation.

  synthetic_lifecycle.py OUT.jsonl ROUNDS [WORKERS] [PORT]

REUSE_DS=1: start the server once and keep it for every round (production
behaviour); default: a fresh server per round. GAP: seconds between rounds.
"""
import json
import os
import shutil
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from synthetic_isolation import CHECK, SOURCE, WORKER  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    out, rounds = sys.argv[1], int(sys.argv[2])
    workers = int(sys.argv[3]) if len(sys.argv) > 3 else 30
    port = int(sys.argv[4]) if len(sys.argv) > 4 else 11899
    wait = float(os.environ.get('WAIT', '15'))
    env = dict(os.environ, ROS_DISCOVERY_SERVER=f'127.0.0.1:{port}')
    env.pop('ROS_SUPER_CLIENT', None)
    logdir = os.path.splitext(out)[0] + '_logs'
    os.makedirs(logdir, exist_ok=True)
    reuse = os.environ.get('REUSE_DS') == '1'
    gap = float(os.environ.get('GAP', '1'))

    def start_server():
        s = subprocess.Popen(['sh', shutil.which('fastdds'), 'discovery', '-i', '0', '-l', '127.0.0.1',
                              '-p', str(port)], stdout=subprocess.DEVNULL,
                             stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(1.5)
        return s

    srv = start_server() if reuse else None
    for r in range(rounds):
        if not reuse:
            srv = start_server()
        procs = []
        lc_log = os.path.join(logdir, f'round_{r:02d}_launch.log')
        lf = open(lc_log, 'w')
        procs.append(subprocess.Popen(
            ['ros2', 'launch', os.path.join(HERE, 'synth', 'lc.launch.py')], env=env,
            stdout=lf, stderr=subprocess.STDOUT, start_new_session=True))
        procs.append(subprocess.Popen([sys.executable, '-c', SOURCE], env=env,
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                      start_new_session=True))
        for i in range(workers):
            procs.append(subprocess.Popen([sys.executable, '-c', WORKER, str(i)], env=env,
                                          stdout=subprocess.DEVNULL,
                                          stderr=subprocess.DEVNULL,
                                          start_new_session=True))
        time.sleep(wait)
        res = subprocess.run([sys.executable, '-c', CHECK, str(workers)],
                             env=dict(env, ROS_SUPER_CLIENT='TRUE'),
                             capture_output=True, text=True, timeout=60)
        try:
            data = json.loads(res.stdout.strip().splitlines()[-1])
        except (IndexError, ValueError):
            data = {'counts': {}, 'names': []}
        stop = procs if reuse else procs + [srv]
        for p in stop:
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        time.sleep(3.0)
        for p in stop:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        lf.close()
        text = open(lc_log, errors='replace').read()
        rec = {'round': r, 'configured': 'SYN CONFIGURED' in text,
               'activated': 'SYN ACTIVATED' in text,
               'abandoned_response_wait': 'service response, due to shutdown' in text,
               'abandoned_service_wait': "service, due to shutdown" in text,
               'silent_workers': [int(i) for i, v in data['counts'].items() if v == 0],
               'nodes_seen': len(data['names'])}
        rec['reuse_ds'] = reuse
        print(json.dumps(rec), flush=True)
        with open(out, 'a') as f:
            f.write(json.dumps(rec) + '\n')
        time.sleep(gap)
    if reuse:
        for sig in (signal.SIGINT, signal.SIGKILL):
            try:
                os.killpg(srv.pid, sig)
            except ProcessLookupError:
                pass
            time.sleep(1.0)


if __name__ == '__main__':
    main()
