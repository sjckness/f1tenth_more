#!/usr/bin/env bash
# restart.sh -- put the Isaac sim run back at its start, then check that it is.
# Default: the stack's own full reset, /reset_all (reset_manager, std_srvs/Trigger: stop + abort
# mission, /sim/reset = car and person back at their start poses (sim overlay
# wheel_odom.method service), EKF local /set_pose, SLAM restart, EKF global
# /ekf_global/set_pose, hooks), then --settle seconds, then the checks below.
# --sim-only calls just Isaac's /sim/reset (std_srvs/Trigger), for when the stack is not running.
# Runs in the stack's ROS environment (sim_isaac.sh run), like llm.sh.
#
# Usage:
#   scripts/sim/restart.sh                    # /reset_all, settle 5 s, verify
#   scripts/sim/restart.sh --sim-only         # /sim/reset only (car + person), verify the car
#   scripts/sim/restart.sh --settle 8 --net local
# Options: --sim-only, --settle N (s, default 5), --start X,Y,YAW_DEG (start pose override),
#          --net local|eth (default eth), --domain N, -h|--help.
#
# Checks (one OK/FAIL line each, with the values; exit 1 if any fails):
#   pose        /sim/ground_truth/odom within 2 cm / 1 deg of the start pose. The start pose is
#               Isaac's latched /sim/ground_truth/start_pose (scene car.start; isaac_ws f1tenth
#               with that topic), or --start.
#   speed       car at rest: |v| <= 0.05 m/s on /sim/ground_truth/odom (and /odometry/filtered
#               when the stack runs); 0.05 = reset_manager stop.standstill_speed_mps.
#   mission     /mission/status (latched) not LOADED/RUNNING/HOLDING, no emergency stop latched.
#               After an abort it reads ABORTED: that is the idle state of this stack.
#   mpc         mpc_corr commands zero: every /drive sample over 1 s has speed 0. The /mpc/hold it
#               received is printed: the abort inside /reset_all ENGAGES it by design, and the next
#               /mission/start_mission releases it (f1tenth_behavior mission/loader.py), so an
#               engaged hold does not block the next run and is not a failure. mpc_corr publishes
#               no hold state of its own; this is the last /mpc/hold seen while the script ran.
#   localization /odometry/filtered within 5 cm / 0.05 rad of 0 0 0, /ekf_global/odometry/filtered
#               and /slam/pose within 15 cm / 0.05 rad (reset_manager.yaml's own tolerances).
# --sim-only checks pose and speed only (the stack's state is not reset, so the rest is skipped).

if [ -z "${BASH_VERSION:-}" ]; then exec bash "$0" "$@"; fi
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() { awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; }
die() { echo "[restart] ERROR: $*" >&2; exit 1; }

# ---- inside the stack's ROS environment (re-invoked through sim_isaac.sh run) ------------
if [ -n "${F1T_RESTART_INNER:-}" ]; then
  exec python3 - <<'PY'
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

SIM_ONLY = os.environ['F1T_RESTART_SIM_ONLY'] == 'true'
SETTLE = float(os.environ['F1T_RESTART_SETTLE'])
START = os.environ.get('F1T_RESTART_START', '')
POS_TOL, YAW_TOL_DEG = 0.02, 1.0        # car back at its start pose
SPEED_TOL = 0.05                         # reset_manager.yaml stop.standstill_speed_mps
LOCAL_TOL, GLOBAL_TOL, LOC_YAW_TOL = 0.05, 0.15, 0.05   # reset_manager.yaml ekf_local/ekf_global
WINDOW = 1.0                             # s of samples after the settle
ACTIVE = ('LOADED', 'RUNNING', 'HOLDING')


def log(msg):
    print(f'[restart {time.strftime("%H:%M:%S")}] {msg}', flush=True)


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Restart(Node):

    def __init__(self):
        super().__init__('sim_restart')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.last = {}
        self.drive = []          # (monotonic, speed) of mpc_corr's /drive
        self.hold = None
        self.sampling = False

        def keep(key):
            return lambda m: self.last.__setitem__(key, (time.monotonic(), m))
        self.create_subscription(PoseStamped, '/sim/ground_truth/start_pose', keep('start'), latched)
        self.gt_speeds = []
        self.create_subscription(Odometry, '/sim/ground_truth/odom', self._gt, 50)
        if not SIM_ONLY:
            from f1tenth_messages.msg import MissionStatus
            status_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                    reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(MissionStatus, '/mission/status', keep('mission'), status_qos)
            self.create_subscription(Odometry, '/odometry/filtered', keep('ekf_local'), 10)
            self.create_subscription(Odometry, '/ekf_global/odometry/filtered', keep('ekf_global'), 10)
            self.create_subscription(PoseWithCovarianceStamped, '/slam/pose', keep('slam'), 10)
            self.create_subscription(Bool, '/mpc/hold', lambda m: setattr(self, 'hold', m.data), 10)
            self.create_subscription(AckermannDriveStamped, '/drive', self._drive, 10)

    def _gt(self, m):
        self.last['gt'] = (time.monotonic(), m)
        if self.sampling:
            t = m.twist.twist.linear
            self.gt_speeds.append(math.hypot(t.x, t.y))

    def _drive(self, m):
        if self.sampling:
            self.drive.append(m.drive.speed)

    def spin_for(self, sec, until=None):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if until is not None and until():
                return True
        return until() if until is not None else True

    def call(self, name, timeout):
        cli = self.create_client(Trigger, name)
        if not cli.wait_for_service(timeout_sec=10.0):
            return None, f'service {name} not available within 10 s'
        fut = cli.call_async(Trigger.Request())
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not fut.done():
            return None, f'no answer from {name} within {timeout:.0f} s'
        return fut.result(), ''


def main():
    rclpy.init()
    node = Restart()
    results = []

    def check(name, ok, detail):
        results.append(ok)
        print(f'  {"OK  " if ok else "FAIL"}  {name:<13} {detail}', flush=True)

    service, timeout = ('/sim/reset', 20.0) if SIM_ONLY else ('/reset_all', 150.0)
    node.spin_for(1.0)     # match subscriptions first: /reset_all's /mpc/hold must be seen
    log(f'calling {service} ...')
    t0 = time.monotonic()
    res, err = node.call(service, timeout)
    if res is None or not res.success:
        log(f'{service} FAILED after {time.monotonic() - t0:.1f} s: {err or res.message}')
        if res is None and SIM_ONLY:
            log('is Isaac running and reachable (same --net / domain)?')
        elif res is None:
            log('is the stack running? Without it, use --sim-only.')
        return 1
    log(f'{service} done in {time.monotonic() - t0:.1f} s: {res.message}')
    log(f'settling {SETTLE:g} s ...')
    node.spin_for(SETTLE)
    node.sampling = True
    node.spin_for(WINDOW)
    node.sampling = False
    now = time.monotonic()

    def fresh(key, max_age=2.0):
        v = node.last.get(key)
        return v[1] if v is not None and now - v[0] <= max_age else None

    print('== restart.sh checks ==')
    # pose
    if START:
        sx, sy, syaw_deg = (float(v) for v in START.split(','))
        src = '--start'
    elif 'start' in node.last:
        p = node.last['start'][1].pose
        sx, sy, syaw_deg = p.position.x, p.position.y, math.degrees(yaw(p.orientation))
        src = '/sim/ground_truth/start_pose'
    else:
        sx = None
    gt = fresh('gt')
    if gt is None:
        check('pose', False, 'no /sim/ground_truth/odom in the last 2 s (is Isaac running?)')
    elif sx is None:
        p = gt.pose.pose
        check('pose', False, f'start pose unknown: no latched /sim/ground_truth/start_pose (Isaac '
              f'without it?) and no --start; car at ({p.position.x:+.3f}, {p.position.y:+.3f}, '
              f'{math.degrees(yaw(p.orientation)):+.1f} deg)')
    else:
        p = gt.pose.pose
        d = math.hypot(p.position.x - sx, p.position.y - sy)
        dyaw = math.degrees(abs(wrap(yaw(p.orientation) - math.radians(syaw_deg))))
        check('pose', d <= POS_TOL and dyaw <= YAW_TOL_DEG,
              f'car ({p.position.x:+.3f}, {p.position.y:+.3f}, {math.degrees(yaw(p.orientation)):+.1f} deg) '
              f'vs start ({sx:+.3f}, {sy:+.3f}, {syaw_deg:+.1f} deg) [{src}]: {100 * d:.1f} cm, '
              f'{dyaw:.2f} deg (tol {100 * POS_TOL:.0f} cm, {YAW_TOL_DEG:g} deg)')
    # speed
    if not node.gt_speeds:
        check('speed', False, f'no /sim/ground_truth/odom during the {WINDOW:g} s window')
    else:
        v_gt = max(node.gt_speeds)
        detail = f'ground truth max |v| {v_gt:.3f} m/s'
        ok = v_gt <= SPEED_TOL
        ekf = None if SIM_ONLY else fresh('ekf_local')
        if ekf is not None:
            t = ekf.twist.twist.linear
            v_ekf = math.hypot(t.x, t.y)
            detail += f', /odometry/filtered {v_ekf:.3f} m/s'
            ok = ok and v_ekf <= SPEED_TOL
        check('speed', ok, f'{detail} (tol {SPEED_TOL} m/s)')
    if SIM_ONLY:
        print('  SKIP  mission, mpc, localization (--sim-only: the stack was not reset)')
    else:
        m = node.last.get('mission', (None, None))[1]
        if m is None:
            check('mission', False, 'no /mission/status (latched) received')
        else:
            check('mission', m.state not in ACTIVE and not m.emergency_stop_active,
                  f'state {m.state}, emergency_stop {str(m.emergency_stop_active).lower()} '
                  f'(fail if {"/".join(ACTIVE)} or emergency stop)')
        hold = {None: 'no /mpc/hold seen', True: 'hold engaged (released by the next start_mission)',
                False: 'hold released'}[node.hold]
        if not node.drive:
            check('mpc', False, f'no /drive from mpc_corr during the {WINDOW:g} s window; {hold}')
        else:
            vmax = max(abs(v) for v in node.drive)
            check('mpc', vmax == 0.0, f'/drive max |speed| {vmax:.3f} m/s over {len(node.drive)} '
                  f'samples; {hold}')
        parts, ok = [], True
        for key, topic, tol in (('ekf_local', '/odometry/filtered', LOCAL_TOL),
                                ('ekf_global', '/ekf_global/odometry/filtered', GLOBAL_TOL),
                                ('slam', '/slam/pose', GLOBAL_TOL)):
            msg = fresh(key, 5.0)
            if msg is None:
                parts.append(f'{topic}: nothing in 5 s')
                ok = False
                continue
            p = msg.pose.pose
            d, a = math.hypot(p.position.x, p.position.y), abs(wrap(yaw(p.orientation)))
            ok = ok and d <= tol and a <= LOC_YAW_TOL
            parts.append(f'{topic} ({p.position.x:+.3f}, {p.position.y:+.3f}, {a:.3f} rad) '
                         f'tol {100 * tol:.0f} cm')
        check('localization', ok, '; '.join(parts))
    failed = results.count(False)
    print(f'== {"OK" if not failed else f"FAIL ({failed} of {len(results)} checks)"} ==')
    node.destroy_node()
    rclpy.shutdown()
    return 1 if failed else 0


sys.exit(main())
PY
fi

# ---- outside: arguments, then re-run this file inside sim_isaac.sh run ------------------
NET=eth; SIM_ONLY=false; SETTLE=5; START=""; RUN_OPTS=()
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --sim-only) SIM_ONLY=true; shift ;;
    --settle) SETTLE="${2:?--settle needs seconds}"; shift 2 ;;
    --start) START="${2:?--start needs X,Y,YAW_DEG}"; shift 2 ;;
    --net) NET="${2:?--net needs local|eth}"; shift 2 ;;
    --net=*) NET="${1#*=}"; shift ;;
    --domain) RUN_OPTS+=(--domain "${2:?--domain needs a number}"); shift 2 ;;
    *) die "unknown argument '$1' (--help)" ;;
  esac
done
[[ "$SETTLE" =~ ^[0-9]+([.][0-9]+)?$ ]] || die "--settle must be a number of seconds, got '$SETTLE'"
[ -z "$START" ] || [[ "$START" =~ ^-?[0-9.]+,-?[0-9.]+,-?[0-9.]+$ ]] \
  || die "--start must be X,Y,YAW_DEG (e.g. 1.0,1.5,0), got '$START'"
export F1T_RESTART_INNER=1 F1T_RESTART_SIM_ONLY="$SIM_ONLY" F1T_RESTART_SETTLE="$SETTLE" \
  F1T_RESTART_START="$START"
exec "$HERE/sim_isaac.sh" run --net "$NET" "${RUN_OPTS[@]}" -- "$HERE/restart.sh"
