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
def load_scenario(sid: str) -> dict:
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
    for ob in scn.get('obstacles') or []:
        lat = float(ob.get('lateral_m', 0.0))
        if ob.get('lateral_sign') == 'alternate' and rep % 2 == 1:
            lat = -lat
        ahead = float(ob['ahead_m'])
        x = x0 + ahead * math.cos(yaw0) - lat * math.sin(yaw0)
        y = y0 + ahead * math.sin(yaw0) + lat * math.cos(yaw0)
        obstacles[ob['id']] = [round(x, 4), round(y, 4)]
        obstacle_info.append({'id': ob['id'], 'x': x, 'y': y, 'ahead_m': ahead, 'lateral_m': lat,
                              'radius_m': OBSTACLE_RADIUS_M[ob['id']]})
    return {'scenario': scn['id'], 'rep': int(rep), 'car': car, 'obstacles': obstacles,
            'obstacle_info': obstacle_info, 'mission': dict(scn['mission']),
            'nominal_start': {'x': x0, 'y': y0, 'yaw_deg': math.degrees(yaw0)}}


# The movable obstacles of the Isaac 'tuning' layout (~/isaac_ws config/scenes/base_room.yaml).
OBSTACLE_RADIUS_M = {'cyl050': 0.25, 'cyl100': 0.50}


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
    for name, value in full.items():
        e = entries[name]
        if e['via'] == 'launch_arg':
            nav[e.get('launch_arg', name)] = float(value)
        elif e['via'] == 'node_param':
            node = e['node']
            overlay.setdefault(node, {}).setdefault('ros__parameters', {})[name] = float(value)
        else:
            raise ValueError(f'{name}: unknown via {e["via"]!r}')
    for node, block in (space.get('fixed_node_params') or {}).items():
        overlay.setdefault(node, {}).setdefault('ros__parameters', {}).update(block)
    sim_overlay = dump_yaml(overlay, os.path.join(out_dir, 'sim_overlay.yaml'))
    sim_extra = dump_yaml(extra, os.path.join(out_dir, 'sim_overlay_extra.yaml'))
    return sim_overlay, sim_extra


def expected_node_params(params: dict, space: dict) -> dict:
    """{node: {ros_param: value}} the runner checks on the live nodes after bringup."""
    out: dict = {}
    for name, value in full_params(params, space).items():
        e = space['parameters'][name]
        out.setdefault(e['node'], {})[e.get('ros_param', name)] = float(value)
    for node, block in (space.get('fixed_node_params') or {}).items():
        out.setdefault(node, {}).update(block)
    return out
