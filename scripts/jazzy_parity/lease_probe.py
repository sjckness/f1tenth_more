#!/usr/bin/env python3
"""
Measure the participant lease the Fast DDS profile actually gives.

Runs on: the Orin host (native Humble), inside the harness container
(Jazzy), linus. From scripts/fix_batch_5/lease_probe.py on the jazzy line;
the talker is now docker/orin/checks/dds_probe.py (demo_nodes_cpp is not in
the image).

Plan P9.5: "DS lease 4.4-4.7 s" in the container (the shipped
config/fastdds_profile.xml: 5 s lease, CLAUDE.md "DDS / discovery"). The
native Humble stack has no profile (Fast DDS default, 20 s): measured there
for the record, not gated. parity_gates.py lease gives the verdict: every
trial in [3.8, 5.5] s (the lease is re-announced every 1 s, so the removal
time depends on where in that cycle the kill lands: 4.16-5.01 s measured
with the profile); trials outside the plan's 4.4-4.7 s are noted. Run the
default 5 trials.

Per trial: a talker (a Discovery Server client) starts, a super client sees
it in the graph, the talker is SIGKILLed (it cannot dispose), and the super
client measures how long the node stays in the graph: the dead
participant's remaining lease as the Discovery Server enforces it.

Runs on its own Discovery Server (--port, default 11911) and domain
(--domain, default 94), so a stack on the production server is untouched.
The profile is applied to the server, the talker and the observer alike, as
the launch files do (--apply-to narrows it).

  lease_probe.py --profile FILE|installed|none --trials N --out FILE.json
                 [--apply-to server,talker,observer] [--port P] [--domain D]
"""
import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PROBE = os.path.join(HERE, '..', '..', 'docker', 'orin', 'checks', 'dds_probe.py')

OBSERVER = r"""
import sys, time, rclpy
rclpy.init()
n = rclpy.create_node('lease_observer')
target = sys.argv[1]
def present():
    return any(name == target for name, _ in n.get_node_names_and_namespaces())
end = time.monotonic() + 60
while time.monotonic() < end and not present():
    rclpy.spin_once(n, timeout_sec=0.05)
print('SEEN' if present() else 'NEVER', flush=True)
line = sys.stdin.readline()           # parent writes the kill time (epoch)
t_kill = float(line)
while present() and time.time() - t_kill < 60:
    rclpy.spin_once(n, timeout_sec=0.05)
print('GONE %.3f' % (time.time() - t_kill), flush=True)
"""


def resolve_profile(profile):
    if profile == 'installed':
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('f1tenth_bringup'), 'config',
                         'fastdds_profile.xml')
        if not os.path.exists(p):
            raise SystemExit('no installed fastdds_profile.xml on this side (%s)' % p)
        return p
    if profile != 'none' and not os.path.exists(profile):
        raise SystemExit('profile %s not found' % profile)
    return profile


def env_for(a, role, super_client=False):
    env = dict(os.environ)
    env.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
    env.pop('ROS_SUPER_CLIENT', None)
    if a.profile != 'none' and role in a.apply_to:
        env['FASTRTPS_DEFAULT_PROFILES_FILE'] = a.profile
    env['ROS_DOMAIN_ID'] = str(a.domain)
    env['ROS_DISCOVERY_SERVER'] = '127.0.0.1:%d' % a.port
    if super_client:
        env['ROS_SUPER_CLIENT'] = 'TRUE'
    return env


def port_bound(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(('127.0.0.1', port))
        return False
    except OSError:
        return True
    finally:
        s.close()


def trial(a, i):
    name = 'lease_talker_%d' % i
    talker = subprocess.Popen(
        [sys.executable, PROBE, 'talker', '120', '--node', name],
        env=env_for(a, 'talker'), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True)
    obs = subprocess.Popen([sys.executable, '-c', OBSERVER, name],
                           env=env_for(a, 'observer', True), stdin=subprocess.PIPE,
                           stdout=subprocess.PIPE, text=True)
    try:
        if obs.stdout.readline().strip() != 'SEEN':
            return None
        time.sleep(3.0)  # past the client's initial sync phase
        os.killpg(talker.pid, signal.SIGKILL)
        obs.stdin.write('%f\n' % time.time())
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


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__[__doc__.index('  lease_probe.py'):])
    ap.add_argument('--profile', required=True,
                    help="profile XML, 'installed' (this side's f1tenth_bringup "
                         "config/fastdds_profile.xml) or 'none'")
    ap.add_argument('--trials', type=int, default=5)
    ap.add_argument('--out', required=True)
    ap.add_argument('--apply-to', default='server,talker,observer',
                    help='which participants get the profile (default: all)')
    ap.add_argument('--port', type=int, default=11911)
    ap.add_argument('--domain', type=int, default=94)
    a = ap.parse_args(argv)
    a.apply_to = set(a.apply_to.split(','))
    a.profile = resolve_profile(a.profile)
    if port_bound(a.port):
        raise SystemExit('UDP port %d is in use: pick another --port' % a.port)
    server = subprocess.Popen(['fastdds', 'discovery', '-i', '0', '-l', '127.0.0.1',
                               '-p', str(a.port)],
                              env=env_for(a, 'server'), stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL, start_new_session=True)
    time.sleep(2.0)
    try:
        res = [trial(a, i) for i in range(a.trials)]
    finally:
        os.killpg(server.pid, signal.SIGINT)
        try:
            server.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(server.pid, signal.SIGKILL)
            server.wait()
    ok = [r for r in res if r is not None]
    out = {'profile': a.profile, 'apply_to': sorted(a.apply_to),
           'ros_distro': os.environ.get('ROS_DISTRO'), 'removal_after_kill_s': res,
           'min': min(ok) if ok else None, 'max': max(ok) if ok else None}
    with open(a.out, 'w') as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out))
    return 0 if len(ok) == len(res) else 1


if __name__ == '__main__':
    sys.exit(main())
