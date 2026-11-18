#!/usr/bin/env python3
"""One run of a tuning scenario against the live stack: place, reset, start, record. No bags.

Started by run_scenario.py inside the stack's ROS environment (env.sh). Steps, all with timeouts:

  1. ready     /clock, /sim/ground_truth/odom, /odometry/filtered and /mission/status flowing, and
               the mpc_corr node up;
  2. params    every expected parameter read back from the live nodes (mpc_corr, semantic_layer_node)
               and compared with the set under test: a mismatch ends the run as 'invalid' (exit 3);
  3. scenario  /sim/scenario (car start + obstacles) and its /sim/scenario_applied reply;
  4. reset     /reset_all (sim reset to that start, EKFs, slam restart) -- also undoes the parked
               car's creep -- then `settle_s` of sim time;
  5. mission   /mission/load_mission + /mission/start_mission with the generated JSON;
  6. record    CSV streams until the mission leaves RUNNING (+ `tail_s`) or `timeout_s` of sim time
               after the start (+ a wall-clock guard).

Writes into --out: gt.csv, odom.csv, drive.csv, mux.csv, mission.csv, solver.csv, status.csv,
hard_stop.csv, bt.csv, obstacles.csv, corridor.jsonl, events.jsonl, outcome.jsonl, cpu.csv, meta.json.
Exit 0 = recorded (whatever the outcome), 2 = the stack never got ready, 3 = parameters not
applied, 4 = scenario/reset/mission start failed.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from f1tenth_messages.msg import (BehaviorTreeStatus, MissionStatus, MoveOutcome, MpcSolverStatus,
                                  Obstacle2DArray)
from f1tenth_messages.srv import LoadMission
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.clock import Clock as RosClock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_srvs.srv import Trigger

BE = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
RELIABLE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
END_STATES = ('COMPLETE', 'ABORTED', 'IDLE')


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _stamp(msg) -> float:
    s = msg.header.stamp
    return s.sec + s.nanosec * 1e-9


class Csv:
    def __init__(self, path, header):
        self.fh = open(path, 'w', newline='', encoding='utf-8')
        self.w = csv.writer(self.fh)
        self.w.writerow(header)
        self.n = 0

    def row(self, *values):
        self.w.writerow(values)
        self.n += 1

    def close(self):
        self.fh.close()


def _mpc_pid() -> int | None:
    """PID of the mpc_corr process (its cmdline names the node), for the CPU-per-tick proxy."""
    for d in os.listdir('/proc'):
        if not d.isdigit():
            continue
        try:
            with open(f'/proc/{d}/cmdline', 'rb') as fh:
                cmd = fh.read().replace(b'\0', b' ')
        except OSError:
            continue
        if b'__node:=mpc_corr' in cmd and b'python' in cmd:
            return int(d)
    return None


def _cpu_seconds(pid: int) -> float | None:
    try:
        with open(f'/proc/{pid}/stat', encoding='ascii') as fh:
            fields = fh.read().rsplit(')', 1)[1].split()
        return (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK')
    except (OSError, IndexError, ValueError):
        return None


class Recorder(Node):

    def __init__(self, out: str):
        super().__init__('isaac_tuning_recorder',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.out = out
        self.recording = False
        self.t_sim = None
        self.seen = {k: 0 for k in ('clock', 'gt', 'odom', 'mission')}
        self.mission_state = None
        self.mission_states = []          # (t, state)
        self.applied = None
        f = lambda name, header: Csv(os.path.join(out, name), header)  # noqa: E731
        self.csv = {
            'gt': f('gt.csv', ['t', 'x', 'y', 'yaw', 'v', 'w']),
            'odom': f('odom.csv', ['t', 'x', 'y', 'yaw', 'v']),
            'drive': f('drive.csv', ['t', 'speed', 'steer']),
            'mux': f('mux.csv', ['t', 'speed', 'steer', 'frame']),
            'mission': f('mission.csv', ['t', 'state']),
            'solver': f('solver.csv', ['t', 'success', 'status', 'n_obstacles', 'cost', 'solve_dt']),
            'status': f('status.csv', ['t', 'status', 'solve_time_ms', 'success']),
            'hard_stop': f('hard_stop.csv', ['t', 'engaged', 'nearest_range_m']),
            'bt': f('bt.csv', ['t', 'active_lane', 'emergency_trip', 'safety_stop_active',
                               'stop_source']),
            'obstacles': f('obstacles.csv', ['t', 'frame', 'n', 'x', 'y', 'r', 'id', 'vx', 'vy',
                                             'v_valid', 'v_sigma']),
            'cpu': f('cpu.csv', ['t_wall', 't', 'mpc_cpu_s']),
            'movers': f('movers.csv', ['t', 'id', 'x', 'y']),
        }
        self.jsonl = {k: open(os.path.join(out, f'{k}.jsonl'), 'w', encoding='utf-8')
                      for k in ('corridor', 'events', 'outcome')}
        sub = self.create_subscription
        sub(Clock, '/clock', self._on_clock, BE)
        sub(Odometry, '/sim/ground_truth/odom', self._on_gt, BE)
        sub(Odometry, '/odometry/filtered', self._on_odom, BE)
        sub(AckermannDriveStamped, '/drive', self._on_drive, BE)
        sub(AckermannDriveStamped, '/ackermann_drive', self._on_mux, BE)
        sub(MissionStatus, '/mission/status', self._on_mission, LATCHED)
        sub(MoveOutcome, '/mission/move_outcome', self._on_outcome, RELIABLE)
        sub(MpcSolverStatus, '/mpc/solver_status', self._on_solver, BE)
        sub(String, '/mpc/status', self._on_status, BE)
        sub(String, '/safety/hard_stop', self._on_hard_stop, BE)
        sub(String, '/safety/event', self._on_event, BE)
        sub(BehaviorTreeStatus, '/behavior/tree_status', self._on_bt, BE)
        sub(Obstacle2DArray, '/costmap/semantic_obstacles', self._on_obstacles, BE)
        sub(String, '/corridor', self._on_corridor, BE)
        sub(String, '/sim/scenario_applied', self._on_applied, LATCHED)
        sub(String, '/sim/ground_truth/movers', self._on_movers, BE)
        self.pub_scenario = self.create_publisher(String, '/sim/scenario', RELIABLE)
        self.pub_go = self.create_publisher(String, '/sim/scenario_go', RELIABLE)
        self.cli_reset_all = self.create_client(Trigger, '/reset_all')
        self.cli_load = self.create_client(LoadMission, '/mission/load_mission')
        self.cli_start = self.create_client(Trigger, '/mission/start_mission')
        self.mpc_pid = None
        self.cpu_timer = self.create_timer(
            0.2, self._on_cpu, clock=RosClock(clock_type=ClockType.STEADY_TIME))

    # ------------------------------------------------------------ callbacks
    def _on_clock(self, msg):
        self.seen['clock'] += 1
        self.t_sim = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_gt(self, msg):
        self.seen['gt'] += 1
        if self.recording:
            p, t = msg.pose.pose, msg.twist.twist
            self.csv['gt'].row(_stamp(msg), p.position.x, p.position.y, _yaw(p.orientation),
                               t.linear.x, t.angular.z)

    def _on_odom(self, msg):
        self.seen['odom'] += 1
        if self.recording:
            p = msg.pose.pose
            self.csv['odom'].row(_stamp(msg), p.position.x, p.position.y, _yaw(p.orientation),
                                 msg.twist.twist.linear.x)

    def _t(self, msg=None):
        if msg is not None and (msg.header.stamp.sec or msg.header.stamp.nanosec):
            return _stamp(msg)
        return self.t_sim if self.t_sim is not None else float('nan')

    def _on_drive(self, msg):
        if self.recording:
            self.csv['drive'].row(self._t(msg), msg.drive.speed, msg.drive.steering_angle)

    def _on_mux(self, msg):
        if self.recording:
            self.csv['mux'].row(self._t(msg), msg.drive.speed, msg.drive.steering_angle,
                                msg.header.frame_id)

    def _on_mission(self, msg):
        self.seen['mission'] += 1
        t = self._t(msg)
        if msg.state != self.mission_state:
            self.mission_states.append((t, msg.state))
        self.mission_state = msg.state
        if self.recording:
            self.csv['mission'].row(t, msg.state)

    def _on_outcome(self, msg):
        if self.recording:
            self.jsonl['outcome'].write(json.dumps({
                't': self._t(msg), 'move_id': msg.move_id, 'move_type': msg.move_type,
                'stop_reason': msg.stop_reason, 'duration_s': msg.duration_s,
                'commanded': msg.commanded, 'actual': msg.actual,
                'score_percent': msg.score_percent}) + '\n')

    def _on_solver(self, msg):
        if self.recording:
            self.csv['solver'].row(self._t(msg), int(msg.success), msg.status, msg.n_obstacles,
                                   msg.cost, msg.solve_dt_sec)

    def _on_status(self, msg):
        if not self.recording:
            return
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        self.csv['status'].row(self._t(), d.get('status'), d.get('solve_time_ms'),
                               d.get('success'))

    def _on_hard_stop(self, msg):
        if not self.recording:
            return
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        self.csv['hard_stop'].row(self._t(), int(bool(d.get('engaged'))),
                                  d.get('nearest_range_m'))

    def _on_event(self, msg):
        if self.recording:
            self.jsonl['events'].write(json.dumps({'t': self._t(), 'data': msg.data}) + '\n')

    def _on_bt(self, msg):
        if self.recording:
            self.csv['bt'].row(self._t(msg), msg.active_lane, msg.emergency_trip,
                               int(msg.safety_stop_active), msg.stop_source)

    def _on_obstacles(self, msg):
        if not self.recording:
            return
        t = self._t(msg)
        if not msg.obstacles:
            self.csv['obstacles'].row(t, msg.header.frame_id, 0, '', '', '', '', '', '', '', '')
        for o in msg.obstacles:
            # id / velocity (odom) only on a stack with semantic_track_velocity_enabled
            self.csv['obstacles'].row(t, msg.header.frame_id, len(msg.obstacles), o.x, o.y, o.r,
                                      getattr(o, 'id', ''), getattr(o, 'vx', ''),
                                      getattr(o, 'vy', ''), int(getattr(o, 'v_valid', False)),
                                      getattr(o, 'v_sigma', ''))

    def _on_corridor(self, msg):
        if not self.recording:
            return
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        self.jsonl['corridor'].write(json.dumps({'t': self._t(), 'definition': d.get('definition'),
                                                 'psi_ref': d.get('psi_ref'),
                                                 'length_m': d.get('length_m')}) + '\n')

    def _on_movers(self, msg):
        if not self.recording:
            return
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        for oid, xy in (d.get('obstacles') or {}).items():
            self.csv['movers'].row(d.get('t'), oid, xy[0], xy[1])

    def _on_applied(self, msg):
        try:
            self.applied = json.loads(msg.data)
        except ValueError:
            self.applied = {'raw': msg.data}

    def _on_cpu(self):
        if not self.recording:
            return
        if self.mpc_pid is None:
            self.mpc_pid = _mpc_pid()
        if self.mpc_pid is not None:
            self.csv['cpu'].row(time.time(), self.t_sim, _cpu_seconds(self.mpc_pid))

    def close(self):
        for c in self.csv.values():
            c.close()
        for fh in self.jsonl.values():
            fh.close()

    # ------------------------------------------------------------ helpers
    def spin_until(self, cond, timeout_wall: float, step: float = 0.05) -> bool:
        t_end = time.time() + timeout_wall
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=step)
            if cond():
                return True
        return cond()

    def call(self, client, request, timeout_wall: float):
        if not client.wait_for_service(timeout_sec=min(timeout_wall, 10.0)):
            return None
        fut = client.call_async(request)
        self.spin_until(fut.done, timeout_wall)
        return fut.result() if fut.done() else None

    def get_params(self, node: str, names: list[str], timeout_wall: float = 10.0) -> dict:
        cli = self.create_client(GetParameters, f'/{node}/get_parameters')
        req = GetParameters.Request()
        req.names = list(names)
        res = self.call(cli, req, timeout_wall)
        self.destroy_client(cli)
        if res is None:
            return {}
        return {name: _pvalue(v) for name, v in zip(names, res.values)}

    def sim_sleep(self, seconds: float, wall_guard: float):
        t0 = self.t_sim
        if t0 is None:
            time.sleep(seconds)
            return
        self.spin_until(lambda: self.t_sim is not None and self.t_sim >= t0 + seconds,
                        wall_guard)


def _pvalue(v):
    """rcl_interfaces/ParameterValue -> python value."""
    return {1: v.bool_value, 2: v.integer_value, 3: v.double_value, 4: v.string_value}.get(
        v.type, None)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('--out', required=True)
    ap.add_argument('--instance', required=True, help='scenario instance JSON (run_scenario.py)')
    ap.add_argument('--mission', required=True, help='mission JSON path (absolute)')
    ap.add_argument('--expect', required=True, help='{node: {param: value}} JSON to verify')
    ap.add_argument('--seq', type=int, default=1)
    ap.add_argument('--ready-timeout', type=float, default=180.0)
    ap.add_argument('--settle-s', type=float, default=2.0)
    ap.add_argument('--tail-s', type=float, default=2.0)
    ap.add_argument('--skip-reset-all', action='store_true',
                    help='only /sim/scenario (its own sim reset), no /reset_all')
    args = ap.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)
    with open(args.instance, encoding='utf-8') as fh:
        inst = json.load(fh)
    with open(args.expect, encoding='utf-8') as fh:
        expect = json.load(fh)
    timeout_s = float(inst['mission']['timeout_s'])
    meta = {'instance': inst, 'mission_path': args.mission, 'wall_start': time.time(),
            'phases': {}, 'exit': None}

    rclpy.init(args=None)
    rec = Recorder(args.out)

    def phase(name, t0):
        meta['phases'][name] = round(time.time() - t0, 3)

    def finish(code, reason):
        meta['exit'] = {'code': code, 'reason': reason}
        meta['mission_states'] = rec.mission_states
        meta['wall_end'] = time.time()
        with open(os.path.join(args.out, 'meta.json'), 'w', encoding='utf-8') as fh:
            json.dump(meta, fh, indent=1, default=str)
        rec.close()
        rec.destroy_node()
        rclpy.try_shutdown()
        print(f'recorder: {reason} (exit {code})', flush=True)
        return code

    # 1. ready
    t0 = time.time()

    def ready():
        names = rec.get_node_names()
        return (all(rec.seen[k] > 0 for k in ('clock', 'gt', 'odom', 'mission'))
                and 'mpc_corr' in names and rec.mission_state in ('IDLE', 'COMPLETE', 'ABORTED'))
    if not rec.spin_until(ready, args.ready_timeout, step=0.2):
        meta['seen'] = rec.seen
        meta['mission_state'] = rec.mission_state
        return finish(2, f'stack not ready in {args.ready_timeout:.0f} s: seen={rec.seen} '
                         f'mission={rec.mission_state}')
    phase('ready', t0)

    # 2. params
    t0 = time.time()
    applied, bad = {}, []
    for node, want in expect.items():
        # A node can be in the graph before its parameter service answers (mpc_corr's
        # __init__ is long): ask again for up to 30 s while nothing comes back.
        t_end = time.time() + 30.0
        got = rec.get_params(node, list(want))
        while (not got or any(v is None for v in got.values())) and time.time() < t_end:
            rec.spin_until(lambda: False, 2.0)
            got = rec.get_params(node, list(want))
        applied[node] = got
        for k, v in want.items():
            g = got.get(k)
            ok = (g == v) if isinstance(v, str) else (
                isinstance(g, (int, float)) and abs(float(g) - float(v)) <= 1e-9 * max(1.0, abs(v)))
            if not ok:
                bad.append(f'{node}.{k}: want {v!r}, live {g!r}')
    meta['params_applied'] = applied
    phase('params', t0)
    if bad:
        return finish(3, 'parameters not applied: ' + '; '.join(bad))

    # 3. scenario
    t0 = time.time()
    req = {'seq': args.seq, 'car': inst['car'], 'obstacles': inst['obstacles'],
           'motion': inst.get('motion') or {}}
    rec.applied = None
    msg = String()
    msg.data = json.dumps(req)
    got = False
    for _ in range(3):   # the latched reply may be a previous run's: match the seq
        rec.pub_scenario.publish(msg)
        if rec.spin_until(lambda: rec.applied is not None and rec.applied.get('seq') == args.seq,
                          8.0):
            got = True
            break
    meta['scenario_applied'] = rec.applied
    phase('scenario', t0)
    if not got or not rec.applied.get('ok'):
        return finish(4, f'/sim/scenario not applied: {rec.applied}')

    # 4. reset
    t0 = time.time()
    if not args.skip_reset_all:
        res = rec.call(rec.cli_reset_all, Trigger.Request(), 60.0)
        meta['reset_all'] = {'success': getattr(res, 'success', None),
                             'message': getattr(res, 'message', None)}
        if res is None or not res.success:
            phase('reset', t0)
            return finish(4, f'/reset_all failed: {meta["reset_all"]}')
    rec.sim_sleep(args.settle_s, 30.0)
    phase('reset', t0)

    # 5. mission
    t0 = time.time()
    rec.recording = True
    lreq = LoadMission.Request()
    lreq.path = args.mission
    res = rec.call(rec.cli_load, lreq, 20.0)
    meta['load_mission'] = {'success': getattr(res, 'success', None),
                            'message': getattr(res, 'message', None)}
    if res is None or not res.success:
        return finish(4, f'load_mission failed: {meta["load_mission"]}')
    res = rec.call(rec.cli_start, Trigger.Request(), 20.0)
    meta['start_mission'] = {'success': getattr(res, 'success', None),
                             'message': getattr(res, 'message', None)}
    if res is None or not res.success:
        return finish(4, f'start_mission failed: {meta["start_mission"]}')
    phase('start', t0)

    # 6. record
    t0 = time.time()
    if not rec.spin_until(lambda: rec.mission_state == 'RUNNING', 15.0):
        return finish(4, f'mission never RUNNING (state {rec.mission_state})')
    t_run = rec.t_sim
    meta['t_running'] = t_run
    if inst.get('motion'):
        go = String()
        go.data = json.dumps({'seq': args.seq})
        rec.pub_go.publish(go)
        meta['t_go'] = rec.t_sim
    end_reason = None
    wall_guard = time.time() + 3.0 * timeout_s + 30.0
    while time.time() < wall_guard:
        rclpy.spin_once(rec, timeout_sec=0.05)
        if rec.mission_state in END_STATES:
            end_reason = f'mission {rec.mission_state}'
            break
        if rec.t_sim is not None and t_run is not None and rec.t_sim - t_run > timeout_s + 2.0:
            end_reason = 'sim timeout (mission still running)'
            break
    else:
        end_reason = 'wall-clock guard'
    meta['t_end'] = rec.t_sim
    meta['end_reason'] = end_reason
    rec.sim_sleep(args.tail_s, 10.0)
    rec.recording = False
    phase('record', t0)
    meta['rows'] = {k: c.n for k, c in rec.csv.items()}
    return finish(0, end_reason)


if __name__ == '__main__':
    sys.exit(main())
