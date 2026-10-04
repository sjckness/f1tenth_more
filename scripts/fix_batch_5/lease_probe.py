#!/usr/bin/env python3
"""Fix batch 5 (H1): does the profile actually change the participant lease?

Per trial: a talker (a Discovery Server client) starts, a super client sees
it in the graph, the talker is SIGKILLed (it cannot dispose), and the super
client measures how long the node stays in the graph. That time is the
dead participant's remaining lease as the Discovery Server enforces it.

Runs on its own Discovery Server (port 11911) and domain (94), so a stack on
the production server is not touched. The profile under test is applied to
the server, the talker and the observer alike, as the launch files do.

  lease_probe.py --profile FILE|none --trials N --out FILE.json
                 [--apply-to server,talker,observer]
"""
import argparse
import json
import os
import signal
import subprocess
import time

PORT = 11911
DOMAIN = '94'


APPLY_TO = {'server', 'talker', 'observer'}


def env_for(profile, super_client=False, role='talker'):
    env = dict(os.environ)
    env.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
    env.pop('ROS_SUPER_CLIENT', None)
    if profile != 'none' and role in APPLY_TO:
        env['FASTRTPS_DEFAULT_PROFILES_FILE'] = profile
    env['ROS_DOMAIN_ID'] = DOMAIN
    env['ROS_DISCOVERY_SERVER'] = f'127.0.0.1:{PORT}'
    if super_client:
        env['ROS_SUPER_CLIENT'] = 'TRUE'
    return env


OBSERVER = r'''
import sys, time, rclpy
rclpy.init()
n = rclpy.create_node('lease_observer')
target = sys.argv[1]
def present():
    return any(name == target for name, _ in n.get_node_names_and_namespaces())
end = time.monotonic() + 60
while time.monotonic() < end and not present():
    rclpy.spin_once(n, timeout_sec=0.05)
print('SEEN', flush=True)
line = sys.stdin.readline()           # parent writes the kill time (epoch)
t_kill = float(line)
while present() and time.time() - t_kill < 60:
    rclpy.spin_once(n, timeout_sec=0.05)
print('GONE %.3f' % (time.time() - t_kill), flush=True)
'''


def trial(profile, i):
    name = f'lease_talker_{i}'
    talker = subprocess.Popen(
        ['ros2', 'run', 'demo_nodes_cpp', 'talker', '--ros-args', '-r', f'__node:={name}'],
        env=env_for(profile), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True)
    obs = subprocess.Popen(['python3', '-c', OBSERVER, name], env=env_for(profile, True, 'observer'),
                           stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    try:
        if obs.stdout.readline().strip() != 'SEEN':
            return None
        time.sleep(3.0)  # past the client's initial sync phase
        os.killpg(talker.pid, signal.SIGKILL)
        obs.stdin.write(f'{time.time()}\n')
        obs.stdin.flush()
        line = obs.stdout.readline().split()
        return float(line[1]) if line and line[0] == 'GONE' else None
    finally:
        try:
            os.killpg(talker.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        obs.kill()
        obs.wait()
        talker.wait()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', required=True)
    ap.add_argument('--trials', type=int, default=5)
    ap.add_argument('--out', required=True)
    ap.add_argument('--apply-to', default='server,talker,observer',
                    help='which participants get the profile (default: all)')
    a = ap.parse_args()
    APPLY_TO.clear()
    APPLY_TO.update(a.apply_to.split(','))
    server = subprocess.Popen(['fastdds', 'discovery', '-i', '0', '-l', '127.0.0.1', '-p', str(PORT)],
                              env=env_for(a.profile, role='server'), stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL, start_new_session=True)
    time.sleep(2.0)
    try:
        res = [trial(a.profile, i) for i in range(a.trials)]
    finally:
        os.killpg(server.pid, signal.SIGINT)
        server.wait()
    ok = [r for r in res if r is not None]
    out = {'profile': a.profile, 'apply_to': sorted(APPLY_TO), 'removal_after_kill_s': res,
           'min': min(ok) if ok else None, 'max': max(ok) if ok else None}
    with open(a.out, 'w') as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out))


if __name__ == '__main__':
    main()
