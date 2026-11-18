"""Shared helpers of the Isaac MPC tuning harness: paths, scenarios, the parameter space, overlays.

Pure Python (PyYAML + stdlib), no ROS: imported by the runner, the loop, the checker and the tests.
"""
from __future__ import annotations

import copy
import math
import os
import random

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
SCENARIO_DIR = os.path.join(HERE, 'scenarios')
TUNING_DIR = os.path.join(HERE, 'tuning')
SPACE_PATH = os.path.join(TUNING_DIR, 'space.yaml')
OUT_ROOT = os.path.join(REPO, 'output', 'isaac_tuning')
BRINGUP_CONFIG = os.path.join(REPO, 'src', 'f1tenth_bringup', 'config')
SIM_OVERLAY_SRC = os.path.join(BRINGUP_CONFIG, 'sim', 'isaac.yaml')
HOST_OVERLAY_SRC = os.path.join(BRINGUP_CONFIG, 'sim', 'hosts', 'linus.yaml')

SCENARIO_IDS = ('S1', 'S2', 'S3', 'S4', 'S5')
MOVING_IDS = ('M1', 'M2', 'M3', 'M4')
PEOPLE_IDS = ('P1', 'P2', 'P3', 'P4')

# Car geometry the metrics need (car_spec / stack_params): base_link is the rear axle centre.
WHEELBASE_M = 0.305
CAR_RADIUS_M = 0.20                  # stack_params car_radius (disc around the axle midpoint)
FOOTPRINT_X_M = (-0.082, 0.405)      # rear bumper .. front bumper, base_link frame (car_spec)
FOOTPRINT_HALF_WIDTH_M = 0.136


def load_yaml(path: str):
    with open(path, encoding='utf-8') as fh:
        return yaml.safe_load(fh)


def dump_yaml(data, path: str) -> str:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as fh:
        yaml.safe_dump(data, fh, sort_keys=False)
    return path


# ---------------------------------------------------------------- scenarios
def is_dodge(sid: str) -> bool:
    """D<dd>_s<seed>: a generated scenario of the dodge family (dodge.py)."""
    return sid.startswith('D') and '_s' in sid


def load_scenario(sid: str) -> dict:
    if is_dodge(sid):
        import dodge
        d, seed = dodge.parse_id(sid)
        return dodge.ensure(d, seed)
    if sid.endswith('u') and sid[:-1] in PEOPLE_IDS:
        # P<k>u: the same person scenario with the harness's LiDAR-only objects ACTIVE (space.yaml
        # fixed_node_params), for the comparison with the car's setting (unknown objects off)
        scn = load_scenario(sid[:-1])
        scn.pop('fixed_node_params', None)
        scn['id'] = sid
        scn['description'] = scn.get('description', '') + ' (unknown objects active)'
        return scn
    path = os.path.join(SCENARIO_DIR, f'{sid}.yaml')
    scn = load_yaml(path)
    if scn.get('id') != sid:
        raise ValueError(f'{path}: id {scn.get("id")!r} != {sid!r}')
    return scn


def scenario_instance(scn: dict, rep: int) -> dict:
    """One repetition of a scenario: the jittered car start and the obstacle positions in map_sim.

    Obstacles are placed relative to the NOMINAL start (jitter moves the car only). The jitter is
    drawn from random.Random(seed * 1000 + rep), so repetition k of a scenario is the same pose on
    every run of every parameter set.
    """
    st = scn['car_start']
    x0, y0, yaw0 = float(st['x']), float(st['y']), math.radians(float(st['yaw_deg']))
    jit = scn.get('jitter') or {}
    rng = random.Random(int(jit.get('seed', 0)) * 1000 + int(rep))
    dxy, dyaw = float(jit.get('xy_m', 0.0)), float(jit.get('yaw_deg', 0.0))
    car = {'x': x0 + rng.uniform(-dxy, dxy), 'y': y0 + rng.uniform(-dxy, dxy),
           'yaw_deg': math.degrees(yaw0) + rng.uniform(-dyaw, dyaw)}
    obstacles = {}
    obstacle_info = []
    motion = {}
    def to_map(ahead, lat):
        return (x0 + ahead * math.cos(yaw0) - lat * math.sin(yaw0),
                y0 + ahead * math.sin(yaw0) + lat * math.cos(yaw0))

    for ob in scn.get('obstacles') or []:
        lat = float(ob.get('lateral_m', 0.0))
        if ob.get('lateral_sign') == 'alternate' and rep % 2 == 1:
            lat = -lat
        ahead = float(ob['ahead_m'])
        x, y = to_map(ahead, lat)
        obstacles[ob['id']] = [round(x, 4), round(y, 4)]
        info = {'id': ob['id'], 'x': x, 'y': y, 'ahead_m': ahead, 'lateral_m': lat,
                'radius_m': obstacle_radius(ob['id'])}
        if 'level' in ob:
            info['level'] = int(ob['level'])
        if 'class' in ob:
            info['class'] = ob['class']
        mo = ob.get('motion')
        if mo and mo.get('type') == 'waypoints':
            # along a polyline (nominal start frame in the scenario, map_sim here); phase 4 people
            pts = [to_map(float(a), float(b)) for a, b in mo['points']]
            motion[ob['id']] = {'type': 'waypoints', 'points': [[round(px, 4), round(py, 4)]
                                                                for px, py in pts],
                                'speed': float(mo['speed']), 'delay_s': float(mo.get('delay_s', 0.0))}
            info['motion'] = motion[ob['id']]
            info['class'] = ob.get('class', 'person')
        elif mo and mo.get('type') == 'pingpong':
            # back and forth a -> b (nominal start frame in the scenario, map_sim here)
            a, b = to_map(*mo['a']), to_map(*mo['b'])
            motion[ob['id']] = {'type': 'pingpong', 'a': [round(a[0], 4), round(a[1], 4)],
                                'b': [round(b[0], 4), round(b[1], 4)], 'speed': float(mo['speed']),
                                's0': float(mo['s0']), 'delay_s': float(mo.get('delay_s', 0.0))}
            info['motion'] = motion[ob['id']]
        elif mo:
            # velocity given in the nominal start frame (along the heading, left-positive)
            va, vl = float(mo.get('v_along', 0.0)), float(mo.get('v_lat', 0.0))
            v = [va * math.cos(yaw0) - vl * math.sin(yaw0), va * math.sin(yaw0) + vl * math.cos(yaw0)]
            motion[ob['id']] = {'v': [round(v[0], 4), round(v[1], 4)],
                                'delay_s': float(mo.get('delay_s', 0.0)),
                                'duration_s': float(mo.get('duration_s', 1e9))}
            info['motion'] = motion[ob['id']]
        obstacle_info.append(info)
    return {'scenario': scn['id'], 'rep': int(rep), 'car': car, 'obstacles': obstacles,
            'motion': motion,
            'obstacle_info': obstacle_info, 'mission': dict(scn['mission']),
            'nominal_start': {'x': x0, 'y': y0, 'yaw_deg': math.degrees(yaw0)}}


# The movable obstacles of the Isaac 'tuning' layout (~/isaac_ws config/scenes/base_room.yaml), and the
# 'dodge' layout's pool c030_<i> (D0.3) / c050_<i> (D0.5).
OBSTACLE_RADIUS_M = {'cyl050': 0.25, 'cyl100': 0.50}


# Movable persons of the Isaac 'people' layout: footprint 0.58 x 0.42 m (arms_down 4 deg) -> r 0.29.
PERSON_IDS = ('p0', 'p1')
PERSON_RADIUS_M = 0.29


def obstacle_radius(oid: str) -> float:
    if oid in OBSTACLE_RADIUS_M:
        return OBSTACLE_RADIUS_M[oid]
    if oid in PERSON_IDS:
        return PERSON_RADIUS_M
    if oid.startswith('c030_'):
        return 0.15
    if oid.startswith('c050_'):
        return 0.25
    raise KeyError(f'unknown obstacle id {oid!r}')


def pingpong_point(a, b, s: float):
    """Same formula as ~/isaac_ws sim/motion.py (test_dodge checks they agree)."""
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])
    length = math.hypot(bx - ax, by - ay)
    if length <= 1e-9:
        return [ax, ay]
    u = math.fmod(s, 2.0 * length)
    if u < 0.0:
        u += 2.0 * length
    f = (u if u <= length else 2.0 * length - u) / length
    return [ax + f * (bx - ax), ay + f * (by - ay)]


def polyline_point(points, s: float):
    """Point at arc length s along the polyline, clamped to its ends (Isaac sim/motion.py)."""
    pts = [(float(p[0]), float(p[1])) for p in points]
    s = max(0.0, s)
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        seg = math.hypot(bx - ax, by - ay)
        if s <= seg:
            f = s / seg if seg > 1e-9 else 0.0
            return [ax + f * (bx - ax), ay + f * (by - ay)]
        s -= seg
    return [pts[-1][0], pts[-1][1]]


def motion_position(m: dict, origin, tau: float):
    """Where an armed obstacle is tau s after /sim/scenario_go (Isaac sim/motion.py)."""
    tau = tau - float(m.get('delay_s', 0.0))
    if m.get('type') == 'waypoints':
        return polyline_point(m['points'], float(m['speed']) * max(tau, 0.0))
    if m.get('type') == 'pingpong':
        return pingpong_point(m['a'], m['b'], float(m.get('s0', 0.0)) + float(m['speed']) * max(tau, 0.0))
    tau = min(max(tau, 0.0), float(m.get('duration_s', 1e9)))
    return [float(origin[0]) + float(m['v'][0]) * tau, float(origin[1]) + float(m['v'][1]) * tau]


def mission_json(scn_inst: dict) -> dict:
    """The straight-drive mission the runner loads (schema 3.0, no LLM)."""
    m = scn_inst['mission']
    return {
        'mission_id': f'isaac_tuning_{scn_inst["scenario"]}_r{scn_inst["rep"]}',
        'schema_version': '3.0',
        'moves': [{
            'id': 'move_0_straight',
            'drive': {'mode': 'straight', 'speed': float(m['speed_mps'])},
            'stop_condition': {'type': 'distance_reached', 'distance': float(m['distance_m'])},
            'timeout_sec': float(m['timeout_s']),
            'on_timeout': 'abort',
            'terminal': True,
        }],
    }


# ---------------------------------------------------------------- parameter space
def load_space(path: str = SPACE_PATH) -> dict:
    return load_yaml(path)


def check_params(params: dict, space: dict) -> list[str]:
    """Every problem with a parameter set; empty means it is inside the declared space."""
    problems = []
    entries = space['parameters']
    for name, value in (params or {}).items():
        if name not in entries:
            problems.append(f'{name}: not in the space (allowed: {", ".join(sorted(entries))})')
            continue
        e = entries[name]
        if 'choices' in e:
            if value not in e['choices']:
                problems.append(f'{name}: {value!r} not one of {e["choices"]}')
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            problems.append(f'{name}: {value!r} is not a number')
            continue
        if not (float(e['min']) <= float(value) <= float(e['max'])):
            problems.append(f'{name}: {value} outside [{e["min"]}, {e["max"]}]')
    return problems


def full_params(params: dict, space: dict) -> dict:
    """The set with every space parameter: the given values over the declared defaults."""
    out = {name: e['default'] for name, e in space['parameters'].items()}
    out.update(params or {})
    return out


def write_overlays(params: dict, space: dict, out_dir: str) -> tuple[str, str]:
    """Write the two launch overlays for a parameter set; returns (sim_overlay, sim_overlay_extra).

    Never touches stack_params.yaml. Launch-argument keys (via: launch_arg) go to
    sim_overlay_extra's sim_component_args.navigation (a node-parameter block would lose against
    the values mpc_corr.launch.py passes); node-only keys (via: node_param) go to a copy of
    config/sim/isaac.yaml under the node's own block. The extra file also carries linus's own
    host overlay (cpu pinning), because the supervisor takes only one extra file.
    """
    entries = space['parameters']
    full = full_params(params, space)
    extra = copy.deepcopy(load_yaml(HOST_OVERLAY_SRC)) if os.path.exists(HOST_OVERLAY_SRC) else {}
    nav = extra.setdefault('component_supervisor_node', {}).setdefault(
        'ros__parameters', {}).setdefault('sim_component_args', {}).setdefault('navigation', {})
    overlay = copy.deepcopy(load_yaml(SIM_OVERLAY_SRC))
    margins = {}
    for name, value in full.items():
        e = entries[name]
        if e['via'] == 'launch_arg':
            nav[e.get('launch_arg', name)] = float(value)
        elif e['via'] == 'node_param':
            node = e['node']
            overlay.setdefault(node, {}).setdefault('ros__parameters', {})[name] = _typed(e, value)
        elif e['via'] == 'class_margin':
            margins[e['class']] = float(value)
        elif e['via'] == 'mission':
            pass                           # run_scenario.run_reps applies it to the mission
        else:
            raise ValueError(f'{name}: unknown via {e["via"]!r}')
    for node, block in (space.get('fixed_node_params') or {}).items():
        overlay.setdefault(node, {}).setdefault('ros__parameters', {}).update(block)
    if margins:
        overlay.setdefault('semantic_layer_node', {}).setdefault('ros__parameters', {})[
            'obstacle_class_margin_m'] = class_margin_json(margins)
    sim_overlay = dump_yaml(overlay, os.path.join(out_dir, 'sim_overlay.yaml'))
    sim_extra = dump_yaml(extra, os.path.join(out_dir, 'sim_overlay_extra.yaml'))
    return sim_overlay, sim_extra


def _typed(entry: dict, value):
    """A node_param's value as the node declares it: choices keep their type, numbers are floats."""
    return value if 'choices' in entry else float(value)


# stack_params' obstacle_class_margin_m default, which the class_margin entries add to.
BASE_CLASS_MARGINS = {'person': 0.5}


def class_margin_json(margins: dict) -> str:
    """semantic_layer_node's obstacle_class_margin_m string (JSON) with these classes set."""
    import json
    out = dict(BASE_CLASS_MARGINS)
    out.update({k: float(v) for k, v in margins.items()})
    return json.dumps(out, sort_keys=True)


def expected_node_params(params: dict, space: dict) -> dict:
    """{node: {ros_param: value}} the runner checks on the live nodes after bringup."""
    out: dict = {}
    margins = {}
    for name, value in full_params(params, space).items():
        e = space['parameters'][name]
        if e['via'] == 'mission':
            continue
        if e['via'] == 'class_margin':
            margins[e['class']] = float(value)
            continue
        out.setdefault(e['node'], {})[e.get('ros_param', name)] = _typed(e, value)
    if margins:
        out.setdefault('semantic_layer_node', {})['obstacle_class_margin_m'] = class_margin_json(
            margins)
    for node, block in (space.get('fixed_node_params') or {}).items():
        out.setdefault(node, {}).update(block)
    return out
