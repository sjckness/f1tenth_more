#!/usr/bin/env python3
"""LLM planner end to end in the Isaac sim: one run per command, scored on ground truth.

Run it on the stack host (the planner talks to llama-server on 127.0.0.1), in the stack's ROS
environment as a Discovery Server super client:

    scripts/sim/sim_isaac.sh run --net eth python3 tools/sim/llm_runs.py --out output/sim_next/llm_runs

Per command:
  1. /reset_all (reset_manager: stop, /sim/reset = car back at its start pose, EKF local, SLAM
     restart, EKF global), then --settle seconds;
  2. `ros2 run llm llm_planner_node "<command>"` exactly as an operator would (translate, write
     the mission, abort/load/start); its stdout is kept, the RESULT line gives the mission;
  3. wait for /mission/status COMPLETE or ABORTED (or --mission-timeout), recording the sim's
     ground truth (/sim/ground_truth/odom, /sim/ground_truth/objects), /mission/move_outcome
     and /safety/event;
  4. score: translation (mission shape vs the command), success (campaign criteria of
     first_test_campaign/prompts.yaml), stop error vs the commanded value, min clearance of the
     car footprint to every object, durations.
Writes <out>/<id>.planner.log, <id>.mission.json, <id>.trajectory.csv, results.jsonl,
llm_runs.csv and llm_runs.md. Nothing here tunes or changes the stack.
"""
import argparse
import csv
import json
import math
import os
import re
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# Car footprint in base_link (rear axle centre): stack_params.yaml footprint, the taped bumper.
FOOT_X = (-0.082, 0.405)
FOOT_HALF_W = 0.136
CONTACT_M = 0.01

# Success criteria (first_test_campaign/prompts.yaml on campaign-final). The plant uses M03's
# "within 0.5 m, facing it" (M03 was the bottle; the prompt became "go to the potted plant").
COMMANDS = [
    {'id': 'forward_3m', 'text': 'go ahead 3 meters', 'kind': 'distance', 'target': 3.0,
     'tol': 0.15, 'lat_tol': 0.2,
     'criterion': 'stops at 3.0 m +- 0.15 m from start, lateral error < 0.2 m, no contact (M01)'},
    {'id': 'approach_wall', 'text': 'approach the wall', 'kind': 'wall', 'target': 0.5,
     'tol': 0.2, 'criterion': 'stops facing the wall at 0.5 m +- 0.2 m, no contact (M02)'},
    {'id': 'go_to_person', 'text': 'go to the person', 'kind': 'go_to', 'cls': 'person',
     'target': 1.0, 'tol': 0.3, 'criterion': 'stops at 1.0 m +- 0.3 m from the person, no contact (M04)'},
    {'id': 'go_to_plant', 'text': 'go to the potted plant', 'kind': 'go_to',
     'cls': 'potted plant', 'max_gap': 0.5, 'facing_deg': 30.0,
     'criterion': 'stops within 0.5 m of the plant, facing it (bearing < 30 deg), no contact (M03)'},
    {'id': 'turn_left_wall', 'text': 'turn left at the wall', 'kind': 'turn', 'sign': 1.0,
     'target': 1.0, 'tol': 0.2, 'turn_deg': 90.0, 'turn_tol': 15.0,
     'criterion': 'starts the left turn at 1.0 m +- 0.2 m from the wall, turns 90 +- 15 deg, no contact (M05)'},
    {'id': 'mixed', 'text': 'go ahead 2 meters then go to the person', 'kind': 'mixed',
     'cls': 'person', 'target': 2.0, 'tol': 0.15, 'gap': 1.0, 'gap_tol': 0.3,
     'criterion': 'first leg 2.0 m +- 0.15 m (M01 tolerance), then stops 1.0 m +- 0.3 m from the person (M04), no contact'},
]


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _rect(cx, cy, yaw, x0, x1, y0, y1):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx + c * x - s * y, cy + s * x + c * y) for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]


def _seg_point(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    t = 0.0 if dx == dy == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)


def _inside(p, poly):
    sign = None
    for i in range(4):
        a, b = poly[i], poly[(i + 1) % 4]
        cr = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        if sign is None:
            sign = cr >= 0
        elif (cr >= 0) != sign:
            return False
    return True


def rect_distance(r1, r2):
    """Min distance between two convex quads (0 when they overlap)."""
    if any(_inside(p, r2) for p in r1) or any(_inside(p, r1) for p in r2):
        return 0.0
    d = float('inf')
    for poly, other in ((r1, r2), (r2, r1)):
        for p in poly:
            for i in range(4):
                d = min(d, _seg_point(p, other[i], other[(i + 1) % 4]))
    return d


def car_rect(x, y, yaw):
    return _rect(x, y, yaw, FOOT_X[0], FOOT_X[1], -FOOT_HALF_W, FOOT_HALF_W)


def obj_rect(o):
    sx, sy = o['size'][0] / 2, o['size'][1] / 2
    return _rect(o['center'][0], o['center'][1], o['yaw'], -sx, sx, -sy, sy)


class Runner(Node):

    def __init__(self):
        super().__init__('llm_runs')
        from f1tenth_messages.msg import MissionStatus, MoveOutcome
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        from vision_msgs.msg import Detection3DArray
        self.gt = None
        self.front = float('nan')   # /sim/ground_truth/front_range: free distance ahead of the bumper
        self.traj = []          # (t_wall, x, y, yaw, v, front_range)
        self.objects = None
        self.states = []
        self.outcomes = []
        self.safety = []
        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        from sensor_msgs.msg import Range
        self.create_subscription(Odometry, '/sim/ground_truth/odom', self._on_gt, 50)
        self.create_subscription(Range, '/sim/ground_truth/front_range',
                                 lambda m: setattr(self, 'front', float(m.range)), 10)
        self.create_subscription(Detection3DArray, '/sim/ground_truth/objects', self._on_objects,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(MissionStatus, '/mission/status',
                                 lambda m: self.states.append((time.time(), m.state, m.emergency_stop_active)),
                                 latched)
        self.create_subscription(MoveOutcome, '/mission/move_outcome', self._on_outcome, 10)
        self.create_subscription(String, '/safety/event', lambda m: self.safety.append((time.time(), m.data)), 10)

    def _on_gt(self, m):
        p = m.pose.pose
        self.gt = (time.time(), p.position.x, p.position.y, _yaw(p.orientation), m.twist.twist.linear.x,
                   self.front)
        self.traj.append(self.gt)

    def _on_objects(self, m):
        objs = []
        for d in m.detections:
            q = d.bbox.center.orientation
            objs.append({'id': d.id, 'class': d.results[0].hypothesis.class_id if d.results else '',
                         'center': [d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.center.position.z],
                         'size': [d.bbox.size.x, d.bbox.size.y, d.bbox.size.z], 'yaw': _yaw(q)})
        self.objects = objs

    def _on_outcome(self, m):
        self.outcomes.append({'t': time.time(), 'move_id': m.move_id, 'move_type': m.move_type,
                              'stop_reason': m.stop_reason, 'outcome': m.outcome,
                              'commanded': round(m.commanded, 3), 'actual': round(m.actual, 3),
                              'duration_s': round(m.duration_s, 2), 'track_gap_m': round(m.track_gap_m, 3),
                              'gt': list(self.gt) if self.gt else None})

    def spin_for(self, sec, until=None):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if until is not None and until():
                return True
        return until() if until is not None else True

    def call(self, srv_type, name, req, timeout=30.0):
        cli = self.create_client(srv_type, name)
        if not cli.wait_for_service(timeout_sec=10.0):
            return None
        fut = cli.call_async(req)
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return fut.result() if fut.done() else None


def run_planner(node, text, log_path, timeout):
    """`ros2 run llm llm_planner_node "<text>"`, spinning our node meanwhile. -> (rc, stdout, wall s)."""
    import threading
    t0 = time.time()
    lines = []
    p = subprocess.Popen(['ros2', 'run', 'llm', 'llm_planner_node', text], stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True)
    reader = threading.Thread(target=lambda: lines.extend(iter(p.stdout.readline, '')), daemon=True)
    reader.start()
    while p.poll() is None:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() - t0 > timeout:
            p.kill()
            lines.append('[llm_runs] planner killed after timeout\n')
            break
    reader.join(timeout=5.0)
    out = ''.join(lines)
    with open(log_path, 'w') as fh:
        fh.write(out)
    return p.returncode, out, time.time() - t0


def parse_result(stdout):
    for line in reversed(stdout.splitlines()):
        if line.startswith('RESULT '):
            try:
                return json.loads(line[len('RESULT '):])
            except ValueError:
                return None
    return None


def check_translation(cmd, mission):
    """Does the mission do what the command says? -> (ok, note, commanded)."""
    if not mission or not mission.get('moves'):
        return False, 'no mission', None
    moves = mission['moves']
    stops = [m.get('stop_condition', {}) for m in moves]
    k = cmd['kind']
    if k == 'distance':
        ok = (len(moves) == 1 and stops[0].get('type') == 'distance_reached'
              and abs(stops[0].get('distance', 0) - cmd['target']) < 1e-6)
        return ok, f'{len(moves)} move(s), stop {stops[0]}', stops[0].get('distance')
    if k == 'wall':
        ok = len(moves) == 1 and stops[0].get('type') == 'front_clearance' and 'go_to_object' not in moves[0]
        return ok, f'stop {stops[0]}', stops[0].get('distance')
    if k == 'go_to':
        g = moves[0].get('go_to_object') or {}
        ok = len(moves) == 1 and g.get('target_class') == cmd['cls']
        return ok, f'go_to {g.get("target_class")} gap {g.get("gap_m")}', g.get('gap_m')
    if k == 'turn':
        d = moves[1].get('drive', {}) if len(moves) > 1 else {}
        ok = (len(moves) == 2 and stops[0].get('type') == 'front_clearance' and d.get('mode') == 'wall_turn'
              and float(d.get('turn_sign', 0)) * cmd['sign'] > 0 and abs(float(d.get('turn_mag_deg', 0)) - 90) < 1)
        return ok, f'wall at {stops[0].get("distance")} then turn sign {d.get("turn_sign")} {d.get("turn_mag_deg")} deg', \
            stops[0].get('distance')
    if k == 'mixed':
        g = moves[1].get('go_to_object') or {} if len(moves) > 1 else {}
        ok = (len(moves) == 2 and stops[0].get('type') == 'distance_reached'
              and abs(stops[0].get('distance', 0) - cmd['target']) < 1e-6 and g.get('target_class') == cmd['cls'])
        return ok, f'{stops[0]} then go_to {g.get("target_class")} gap {g.get("gap_m")}', \
            (stops[0].get('distance'), g.get('gap_m'))
    return False, 'unknown kind', None


def score(cmd, start, traj, objects, outcomes, commanded):
    """Ground-truth metrics and the success verdict."""
    out = {}
    if not traj or start is None or not objects:
        return False, {'error': 'no ground truth'}
    end = traj[-1]
    x0, y0, yaw0 = start[1], start[2], start[3]
    walls = [o for o in objects if o['class'] == 'wall']
    rects = {o['id']: obj_rect(o) for o in objects}
    clear = {o['id']: float('inf') for o in objects}
    for i, s in enumerate(traj):
        if i % 5 and i != len(traj) - 1:
            continue
        cr = car_rect(s[1], s[2], s[3])
        for oid, r in rects.items():
            clear[oid] = min(clear[oid], rect_distance(cr, r))
    out['min_clearance_m'] = {k: round(v, 3) for k, v in clear.items()}
    out['min_clearance_walls_m'] = round(min(clear[w['id']] for w in walls), 3) if walls else None
    contact = min(clear.values()) <= CONTACT_M
    out['contact'] = contact
    dx, dy = end[1] - x0, end[2] - y0
    out['travel_m'] = round(math.hypot(dx, dy), 3)
    out['along_m'] = round(dx * math.cos(yaw0) + dy * math.sin(yaw0), 3)
    out['lateral_m'] = round(-dx * math.sin(yaw0) + dy * math.cos(yaw0), 3)
    out['heading_change_deg'] = round(math.degrees(_wrap(end[3] - yaw0)), 1)
    end_rect = car_rect(end[1], end[2], end[3])

    def facing_wall_gap(x, y, yaw, sample=None):
        """Footprint gap to the wall the car faces: the first wall hit by a ray from the bumper. Scenes
        without wall objects (NVIDIA environments): the sim's ground-truth front range of that sample."""
        if not walls:
            return None if sample is None or not math.isfinite(sample[5]) else sample[5]
        bx, by = x + FOOT_X[1] * math.cos(yaw), y + FOOT_X[1] * math.sin(yaw)
        wr = [(w, obj_rect(w)) for w in walls]
        for i in range(1, 2001):
            px, py = bx + 0.005 * i * math.cos(yaw), by + 0.005 * i * math.sin(yaw)
            for w, r in wr:
                if _inside((px, py), r):
                    return rect_distance(car_rect(x, y, yaw), r)
        return None

    k = cmd['kind']
    ok = not contact
    if k == 'distance':
        out['stop_error_m'] = round(out['along_m'] - (commanded or cmd['target']), 3)
        ok = ok and abs(out['along_m'] - cmd['target']) <= cmd['tol'] and abs(out['lateral_m']) < cmd['lat_tol']
    elif k == 'wall':
        g = facing_wall_gap(end[1], end[2], end[3], end)
        if g is None:
            return False, dict(out, error='no wall ground truth')
        out['final_wall_gap_m'] = round(g, 3)
        if commanded is not None:
            out['stop_error_m'] = round(g - commanded, 3)
        ok = ok and abs(g - cmd['target']) <= cmd['tol']
    elif k in ('go_to', 'mixed'):
        tgt = [o for o in objects if o['class'] == cmd['cls']]
        if not tgt:
            return False, dict(out, error=f'no ground-truth object of class {cmd["cls"]}')
        o = min(tgt, key=lambda o: rect_distance(end_rect, obj_rect(o)))
        gap = rect_distance(end_rect, obj_rect(o))
        out['final_gap_m'] = round(gap, 3)
        bearing = math.degrees(_wrap(math.atan2(o['center'][1] - end[2], o['center'][0] - end[1]) - end[3]))
        out['bearing_to_target_deg'] = round(bearing, 1)
        cmd_gap = commanded[1] if k == 'mixed' and commanded else commanded
        if cmd_gap is not None:
            out['stop_error_m'] = round(gap - cmd_gap, 3)
        if k == 'go_to' and 'max_gap' in cmd:
            ok = ok and gap <= cmd['max_gap'] and abs(bearing) <= cmd['facing_deg']
        elif k == 'go_to':
            ok = ok and abs(gap - cmd['target']) <= cmd['tol']
        else:
            first = next((oc for oc in outcomes if oc['gt']), None)
            if first is None:
                ok = False
                out['first_leg_m'] = None
            else:
                fx, fy = first['gt'][1] - x0, first['gt'][2] - y0
                leg = fx * math.cos(yaw0) + fy * math.sin(yaw0)
                out['first_leg_m'] = round(leg, 3)
                out['first_leg_error_m'] = round(leg - cmd['target'], 3)
                ok = ok and abs(leg - cmd['target']) <= cmd['tol']
            ok = ok and abs(gap - cmd['gap']) <= cmd['gap_tol']
    elif k == 'turn':
        t_turn = next((s for s in traj if abs(_wrap(s[3] - yaw0)) > math.radians(5.0)), None)
        if t_turn is None:
            out['turn_start_wall_gap_m'] = None
            ok = False
        else:
            g = facing_wall_gap(t_turn[1], t_turn[2], yaw0, t_turn)
            out['turn_start_wall_gap_m'] = round(g, 3)
            if commanded is not None:
                out['stop_error_m'] = round(g - commanded, 3)
            ok = ok and abs(g - cmd['target']) <= cmd['tol']
        ok = ok and abs(abs(out['heading_change_deg']) - cmd['turn_deg']) <= cmd['turn_tol'] \
            and out['heading_change_deg'] * cmd['sign'] > 0
    return ok, out


def wait_mission(node, t_start, timeout):
    node.spin_for(timeout, until=lambda: any(t > t_start and s in ('COMPLETE', 'ABORTED') for t, s, _ in node.states))
    after = [(t, s, e) for t, s, e in node.states if t > t_start]
    return after


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', default='output/sim_next/llm_runs')
    ap.add_argument('--only', nargs='*', default=[], help='command ids to run (default all)')
    ap.add_argument('--settle', type=float, default=5.0)
    ap.add_argument('--mission-timeout', type=float, default=150.0)
    ap.add_argument('--planner-timeout', type=float, default=240.0)
    ap.add_argument('--label', default='', help='free text stored with every result (scene, layout)')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    node = Runner()
    from std_srvs.srv import Trigger
    node.spin_for(15.0, until=lambda: node.gt is not None and node.objects is not None)
    if node.gt is None or node.objects is None:
        print(json.dumps({'error': 'no ground truth from the sim (/sim/ground_truth/odom, /objects)'}))
        return 2
    results = []
    cmds = [c for c in COMMANDS if not a.only or c['id'] in a.only]
    for cmd in cmds:
        rid = cmd['id']
        rec = {'id': rid, 'command': cmd['text'], 'criterion': cmd['criterion'], 'label': a.label,
               'time': time.strftime('%Y-%m-%dT%H:%M:%S')}
        print(f'[llm_runs] === {rid}: "{cmd["text"]}"', flush=True)
        t0 = time.time()
        r = node.call(Trigger, '/reset_all', Trigger.Request(), timeout=120.0)
        rec['reset'] = {'success': None if r is None else r.success, 'duration_s': round(time.time() - t0, 1),
                        'message': None if r is None else r.message[:300]}
        node.spin_for(a.settle)
        start = node.gt
        rec['start_pose'] = [round(start[1], 3), round(start[2], 3), round(math.degrees(start[3]), 1)]
        node.traj.clear()
        node.outcomes.clear()
        node.safety.clear()
        t_cmd = time.time()
        rc, stdout, dt_plan = run_planner(node, cmd['text'], os.path.join(a.out, f'{rid}.planner.log'),
                                          a.planner_timeout)
        res = parse_result(stdout) or {}
        rec['planner'] = {'rc': rc, 'wall_s': round(dt_plan, 1), 'status': res.get('status'),
                          'error': res.get('error_message'), 'mission_path': res.get('mission_path')}
        m = re.search(r'Server raggiunto e scaldato in ([0-9.]+)s', stdout)
        rec['planner']['server_wait_s'] = float(m.group(1)) if m else None
        mission = res.get('mission')
        if mission is None and res.get('mission_path') and os.path.isfile(res['mission_path']):
            with open(res['mission_path']) as fh:
                mission = json.load(fh)
        if mission is not None:
            with open(os.path.join(a.out, f'{rid}.mission.json'), 'w') as fh:
                json.dump(mission, fh, indent=2)
        ok_tr, note, commanded = check_translation(cmd, mission)
        rec['translation'] = {'ok': ok_tr, 'note': note}
        if res.get('status') == 'ok' and mission is not None:
            after = wait_mission(node, t_cmd, a.mission_timeout)
            seq = []
            for _, s, _e in after:
                if not seq or seq[-1] != s:
                    seq.append(s)
            rec['mission_states'] = seq
            rec['estop'] = any(e for _, _, e in after)
            final = seq[-1] if seq else None
            rec['final_state'] = final
            t_run = next((t for t, s, _ in after if s == 'RUNNING'), None)
            t_end = next((t for t, s, _ in after if s in ('COMPLETE', 'ABORTED')), None)
            rec['duration_s'] = {'command_to_end': round((t_end or time.time()) - t_cmd, 1),
                                 'running': round((t_end or time.time()) - t_run, 1) if t_run else None}
        else:
            rec['final_state'] = None
        node.spin_for(1.0)
        traj = list(node.traj)
        ok, metrics = score(cmd, start, traj, node.objects, node.outcomes, commanded)
        rec['metrics'] = metrics
        rec['move_outcomes'] = [{k: v for k, v in oc.items() if k != 'gt'} for oc in node.outcomes]
        rec['safety_events'] = [s for _, s in node.safety][:20]
        rec['success'] = bool(ok and ok_tr and rec.get('final_state') == 'COMPLETE')
        with open(os.path.join(a.out, f'{rid}.trajectory.csv'), 'w') as fh:
            w = csv.writer(fh)
            w.writerow(['t_s', 'x', 'y', 'yaw_deg', 'v', 'front_range_m'])
            for s in traj[::5]:
                w.writerow([round(s[0] - t_cmd, 2), round(s[1], 3), round(s[2], 3), round(math.degrees(s[3]), 1),
                            round(s[4], 3), round(s[5], 3)])
        results.append(rec)
        with open(os.path.join(a.out, 'results.jsonl'), 'a') as fh:
            fh.write(json.dumps(rec) + '\n')
        print(json.dumps({k: rec.get(k) for k in ('id', 'success', 'final_state', 'translation', 'metrics')}),
              flush=True)
    write_tables(results, a.out)
    node.destroy_node()
    rclpy.shutdown()
    return 0


def write_tables(results, out):
    cols = ['id', 'command', 'translation_ok', 'final_state', 'success', 'stop_error_m', 'key_metric',
            'min_clear_person_m', 'min_clear_plant_m', 'min_clear_walls_m', 'command_to_end_s', 'running_s',
            'planner_s', 'note']
    rows = []
    for r in results:
        m = r.get('metrics', {})
        mc = m.get('min_clearance_m', {})
        key = {k: m[k] for k in ('along_m', 'lateral_m', 'final_wall_gap_m', 'final_gap_m', 'bearing_to_target_deg',
                                 'turn_start_wall_gap_m', 'heading_change_deg', 'first_leg_m') if k in m}
        rows.append({'id': r['id'], 'command': r['command'], 'translation_ok': r['translation']['ok'],
                     'final_state': r.get('final_state'), 'success': r['success'],
                     'stop_error_m': m.get('stop_error_m'), 'key_metric': json.dumps(key),
                     'min_clear_person_m': mc.get('person'), 'min_clear_plant_m': mc.get('plant'),
                     'min_clear_walls_m': m.get('min_clearance_walls_m'),
                     'command_to_end_s': (r.get('duration_s') or {}).get('command_to_end'),
                     'running_s': (r.get('duration_s') or {}).get('running'),
                     'planner_s': r['planner']['wall_s'], 'note': r['translation']['note']})
    with open(os.path.join(out, 'llm_runs.csv'), 'w') as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(out, 'llm_runs_table.md'), 'w') as fh:
        fh.write('| ' + ' | '.join(cols) + ' |\n|' + '---|' * len(cols) + '\n')
        for row in rows:
            fh.write('| ' + ' | '.join(str(row[c]).replace('|', '/') for c in cols) + ' |\n')


if __name__ == '__main__':
    raise SystemExit(main())
