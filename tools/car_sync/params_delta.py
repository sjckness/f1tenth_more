#!/usr/bin/env python3
"""
Every stack_params default that differs between the car and the container image.

The CAR: its native tree as tools/car_sync/capture_orin.sh captured it and
docker/orin/car_sync.json records it, i.e. stack_params.yaml at the car's
HEAD (native_head, in this repo as remotes/orin/...) plus the car's
uncommitted src/ patch (the capture's src.diff, sha256 checked against
car_sync.json).

The CONTAINER: stack_params.yaml at the image's commit (--image-ref; the
default `worktree` reads the working tree, for a report written before the
commit) as docker/orin/set_installed_params.py edits the installed copy, with
the car profile's launch arguments on top (F1T_CAR_PROFILE_ARGS in
src/f1tenth_ctl/config/ctl_container.env, which only supervisor_bringup's
declared arguments can carry).

Each differing key is one of:
  profile    the profile's launch argument gives the car's value: reproduced
  container  set by set_installed_params.py (the image's own paths): intended
  reviewed   a known difference, explained in REVIEWED below
  differs    not reproduced, not reviewed: needs a decision (--check fails)

    tools/car_sync/params_delta.py [--out output/orin_container/params_delta.md] [--check]
"""

import argparse
from datetime import datetime
import hashlib
import json
import os
import re
import subprocess
import sys
import tempfile

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PARAMS = 'src/f1tenth_params/config/stack_params.yaml'
CAR_SYNC = 'docker/orin/car_sync.json'
CTL_ENV = 'src/f1tenth_ctl/config/ctl_container.env'
SUPERVISOR_LAUNCH = 'src/f1tenth_bringup/launch/supervisor_bringup.launch.py'
OUT = 'output/orin_container/params_delta.md'

# Differences that stay, and why (reviewed 2026-10-24 for the car's container).
# A key in this table is reported, not failed; a new difference is not in it.
REVIEWED = {
    'enable_comms':
        'New (f1tenth_comms); absent on the car, i.e. no comms component, which is what '
        "false gives. The profile passes false explicitly: comms needs the host's "
        'NetworkManager, not supported in the container (v1, build_checks).',
    'foxglove_topic_whitelist':
        'The image adds the four /comms/* topics; with enable_comms false they never '
        'exist, so foxglove_bridge advertises the same topics as on the car.',
    'detection_3d_tf_static_only':
        'New on main-jazzy (2026-10-23). The car uses the full TransformListener for its '
        'one lookup, a fixed ZED joint; /tf_static gives the same transform with less CPU. '
        'The camera pan joint is off on the car (it would need false).',
    'obstacle_projector_tf_static_only':
        'As detection_3d_tf_static_only: the same fixed transform, from /tf_static '
        'only. Needs false only with the camera pan joint on (off on the car).',
    'enable_llm2':
        'New (two-llm-arch), off: the car runs no LLM2 either.',
    'llm2_model': 'Used only with enable_llm2 (off).',
    'llm2_port': 'Used only with enable_llm2 (off).',
    'llm2_param_min': 'Used only with llm2_params_enabled (off).',
    'llm2_param_max': 'Used only with llm2_params_enabled (off).',
    'llm2_param_whitelist': 'Used only with llm2_params_enabled (off).',
    'llm2_params_enabled':
        'New, off: mpc_corr does not subscribe to /llm2/mpc_params, as on the car.',
    'enable_swept_clearance':
        "NOT the car's behaviour: the car starts swept_clearance_node at boot (it "
        'predates the switch), the image does not. Nothing consumes '
        '/perception/swept_clearance on either, so only the CPU differs (one node less '
        'in the container: expect it in the P9 CPU tables). Not a supervisor launch '
        'argument; `scripts/stackctl.py start swept_clearance` starts it on demand.',
    'fastdds_profile':
        'The Jazzy line\'s 5 s participant lease (CLAUDE.md "DDS / discovery"); the '
        'Humble car runs Fast DDS defaults (20 s). Platform, intended.',
    'front_clearance_cpu_threads':
        'New (CPU caps, 2026-10-23); 0 = no cap, as on the car.',
    'semantic_layer_cpu_threads':
        'New (CPU caps, 2026-10-23); 0 = library defaults, as on the car.',
    'yolo_backend':
        'New: pt is what the car runs (it has no TensorRT engine path at all). '
        'yolo_backend:=engine is a stack_args trial once `docker_orin.sh engine` built it.',
    'yolo_cpu_threads':
        "NOT the car's behaviour: the image caps yolo_detector_node at 2 threads (its two "
        'taskset cores, 2026-10-22), the car uses one per core (0). Same detections, less '
        'CPU contention. Not a launch argument of the supervisor.',
    'yolo_cuda_blocking_sync':
        "NOT the car's behaviour: the image lets a GPU wait sleep (blocking sync), the car "
        'spins a core. Same results, less CPU. Not a launch argument of the supervisor.',
    'yolo_gpu_pipeline':
        "NOT the car's behaviour: pre/post-processing on the GPU (identical detections and "
        'masks on 48 frames, 2026-10-22), the car uses the Ultralytics path. Falls back per '
        'frame. Compared in P9 (YOLO IoU, |dconf|).',
    'yolo_max_rate_hz':
        "NOT the car's behaviour: the image runs YOLO on at most 15 frames/s (stamp-driven "
        'drop of the 30 Hz ZED stream), the car on every frame it gets. Detection topics run '
        'at <= 15 Hz in the container: check the object missions in P10/P11. Not a launch '
        'argument of the supervisor.',
}


def git(*args, binary=False):
    proc = subprocess.run(['git', '-C', REPO, *args], capture_output=True, check=True)
    return proc.stdout if binary else proc.stdout.decode()


def image_file(ref: str, path: str) -> str:
    """Return `path` at commit `ref`, or from the working tree for ref `worktree`."""
    if ref == 'worktree':
        with open(os.path.join(REPO, path)) as f:
            return f.read()
    return git('show', f'{ref}:{path}')


def apply_patch(text: str, patch: bytes, path: str) -> str:
    """Apply the hunks of `patch` for `path` to `text` (git apply, outside any repo)."""
    with tempfile.TemporaryDirectory() as tmp:
        target = os.path.join(tmp, path)
        os.makedirs(os.path.dirname(target))
        with open(target, 'w') as f:
            f.write(text)
        env = {k: v for k, v in os.environ.items() if not k.startswith('GIT_')}
        env['GIT_CEILING_DIRECTORIES'] = os.path.dirname(tmp)
        subprocess.run(['git', 'apply', f'--include={path}', '-'], input=patch, cwd=tmp,
                       env=env, check=True, capture_output=True)
        with open(target) as f:
            return f.read()


def profile_args(env_text: str) -> dict:
    """Return the name:=value pairs of F1T_CAR_PROFILE_ARGS (pure)."""
    m = re.search(r'^F1T_CAR_PROFILE_ARGS="([^"]*)"', env_text, re.M)
    if not m:
        raise SystemExit(f'{CTL_ENV}: no F1T_CAR_PROFILE_ARGS line')
    out = {}
    for word in m.group(1).split():
        name, sep, value = word.partition(':=')
        if not sep:
            raise SystemExit(f'{CTL_ENV}: {word!r} is not name:=value')
        out[name] = value
    return out


def launch_value(value):
    """Turn a launch argument string into what the yaml default would be."""
    low = value.lower()
    if low in ('true', 'false'):
        return low == 'true'
    for cast in (int, float):
        try:
            return cast(value)
        except ValueError:
            pass
    return value


def defaults(text: str) -> dict:
    data = yaml.safe_load(text)
    return {k: v.get('default') for k, v in data.items() if isinstance(v, dict) and 'default' in v}


def delta(car: dict, image: dict, installed: dict, profile: dict) -> list:
    """Return one row per key whose container value differs from the car's (pure)."""
    rows = []
    for key in sorted(set(car) | set(installed)):
        effective = (launch_value(profile[key]) if key in profile
                     else installed.get(key, '<absent>'))
        car_value = car.get(key, '<absent>')
        image_value = image.get(key, '<absent>')
        if car_value == image_value == effective:
            continue
        if key in profile and key in car:
            klass = 'profile' if effective == car_value else 'differs'
        elif key in profile:
            # New in the image, passed explicitly: reviewed like a new default.
            klass = 'reviewed' if key in REVIEWED else 'differs'
        elif installed.get(key) != image.get(key):
            klass = 'container'
        elif car_value == effective:
            continue
        elif key in REVIEWED:
            klass = 'reviewed'
        else:
            klass = 'differs'
        rows.append({'key': key, 'car': car_value, 'image': image_value,
                     'container': effective, 'class': klass,
                     'note': REVIEWED.get(key, '')})
    return rows


def fmt(value) -> str:
    text = json.dumps(value) if not isinstance(value, str) else value
    if len(text) > 60:
        text = text[:57] + '...'
    return '`' + text.replace('|', '\\|') + '`' if text else '`""`'


def render(meta: dict, rows: list) -> str:
    counts = {}
    for r in rows:
        counts[r['class']] = counts.get(r['class'], 0) + 1
    lines = [
        '# stack_params: the car vs the container',
        '',
        f"Generated by `tools/car_sync/params_delta.py` on {meta['generated']}. "
        'Read it before P7 (plan [U-dec]); the script fails `--check` on any '
        '`differs` row.',
        '',
        f"- **Car:** `{meta['car_ref'][:12]}` (`{meta['car_branch']}`, captured "
        f"{meta['captured_at']}) + its uncommitted patch `{meta['patch']}` "
        f"(sha256 `{meta['patch_sha'][:16]}...`, = car_sync.json).",
        f"- **Image:** `{meta['image_ref']}` (HEAD `{meta['image_sha'][:12]}`), installed "
        'copy as `docker/orin/set_installed_params.py` edits it.',
        f"- **Car profile** (`{CTL_ENV}`, `F1T_CAR_PROFILE_ARGS`): `{meta['profile_text']}`. "
        'Every one is a declared launch argument of supervisor_bringup.launch.py: '
        f"{meta['profile_declared']}.",
        f"- {meta['n_car']} keys on the car, {meta['n_image']} in the image; "
        f'{len(rows)} differ: ' + ', '.join(f'{v} {k}' for k, v in sorted(counts.items())) + '.',
        '',
        '| Key | Car | Image default | Container | Class |',
        '|---|---|---|---|---|',
    ]
    order = {'differs': 0, 'profile': 1, 'container': 2, 'reviewed': 3}
    for r in sorted(rows, key=lambda r: (order[r['class']], r['key'])):
        klass = f"**{r['class']}**" if r['class'] == 'differs' else r['class']
        lines.append(f"| `{r['key']}` | {fmt(r['car'])} | {fmt(r['image'])} | "
                     f"{fmt(r['container'])} | {klass} |")
    lines += ['', '## Why each kept difference is there', '']
    for r in sorted(rows, key=lambda r: r['key']):
        if r['class'] == 'reviewed':
            lines.append(f"- `{r['key']}`: {r['note']}")
        elif r['class'] == 'container':
            lines.append(f"- `{r['key']}`: the image's own path (set_installed_params.py).")
        elif r['class'] == 'profile':
            lines.append(f"- `{r['key']}`: the car's value, passed by the car profile "
                         f"(`{r['key']}:={meta['profile'][r['key']]}`).")
        else:
            lines.append(f"- `{r['key']}`: **not reproduced and not reviewed: decide.**")
    lines += ['', '## Not reproducible by the profile (for the review)', '',
              'The profile can only set what supervisor_bringup.launch.py declares '
              '(enable_intelligence, enable_mission_logger, enable_comms, calibration, ...). '
              'The rows marked "NOT the car\'s behaviour" above are main-jazzy defaults the '
              'car never had; reproducing them would mean another installed-params edit in '
              'the image (set_installed_params.py) or a code change. They are kept as the '
              'image ships them, and P9/P10 compare against native with them in place.', '']
    unchanged = [k for k in meta['profile'] if meta['image_defaults'].get(k)
                 == launch_value(meta['profile'][k])]
    if unchanged:
        lines += ['Profile arguments equal to the image default (explicit, so a default flip '
                  f"on main does not change the car): {', '.join(f'`{k}`' for k in unchanged)}.",
                  '']
    return '\n'.join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--car-sync', default=CAR_SYNC)
    ap.add_argument('--image-ref', default='worktree',
                    help='commit the image is built from (default: the working tree)')
    ap.add_argument('--out', default=OUT)
    ap.add_argument('--check', action='store_true', help='exit 1 on a `differs` row')
    args = ap.parse_args(argv)
    with open(os.path.join(REPO, args.car_sync)) as f:
        car_sync = json.load(f)
    car_ref = car_sync['native_head']
    patch_path = os.path.join(car_sync['capture'], 'src.diff')
    with open(os.path.join(REPO, patch_path), 'rb') as f:
        patch = f.read()
    patch_sha = hashlib.sha256(patch).hexdigest()
    if patch_sha != car_sync['uncommitted_src_patch_sha256']:
        raise SystemExit(f"{patch_path}: sha256 {patch_sha} is not car_sync.json's")
    car_text = git('show', f'{car_ref}:{PARAMS}')
    if PARAMS.encode() in patch:
        car_text = apply_patch(car_text, patch, PARAMS)
    image_text = image_file(args.image_ref, PARAMS)
    sys.dont_write_bytecode = True   # no __pycache__ in docker/orin (the build context)
    sys.path.insert(0, os.path.join(REPO, 'docker', 'orin'))
    import set_installed_params as sip
    installed_text = sip.edit(image_text, {'llama_server_path': sip.LLAMA_SERVER,
                                           'llama_server_cwd': sip.LLAMA_CWD,
                                           'yolo_engine_path': sip.YOLO_ENGINE})
    env_text = image_file(args.image_ref, CTL_ENV)
    profile = profile_args(env_text)
    launch_text = image_file(args.image_ref, SUPERVISOR_LAUNCH)
    declared = {k: bool(re.search(r"DeclareLaunchArgument\(\s*'%s'" % re.escape(k),
                                  launch_text)) for k in profile}
    car, image, installed = defaults(car_text), defaults(image_text), defaults(installed_text)
    rows = delta(car, image, installed, profile)
    meta = {
        'generated': datetime.now().astimezone().isoformat(timespec='seconds'),
        'car_ref': git('rev-parse', car_ref).strip(),
        'car_branch': car_sync.get('native_branch'), 'captured_at': car_sync.get('captured_at'),
        'patch': patch_path, 'patch_sha': patch_sha, 'image_ref': args.image_ref,
        'image_sha': git('rev-parse', 'HEAD' if args.image_ref == 'worktree'
                         else args.image_ref).strip(),
        'profile': profile, 'profile_text': ' '.join(f'{k}:={v}' for k, v in profile.items()),
        'profile_declared': ', '.join(f'{k} {"yes" if v else "NO"}' for k, v in declared.items()),
        'n_car': len(car), 'n_image': len(image), 'image_defaults': image,
    }
    text = render(meta, rows)
    out = os.path.join(REPO, args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w') as f:
        f.write(text + '\n')
    bad = [r['key'] for r in rows if r['class'] == 'differs'] + [
        f'{k} (not a supervisor launch argument)' for k, v in declared.items() if not v]
    print(f'{len(rows)} differing keys -> {args.out}'
          + (f'; NOT reproduced or reviewed: {", ".join(bad)}' if bad else ''))
    return 1 if args.check and bad else 0


if __name__ == '__main__':
    sys.exit(main())
