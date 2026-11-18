#!/usr/bin/env python3
"""
Record what a D3 harness run ran on, as one JSON file.

Runs on: the Orin host (native Humble, or before a `docker exec` into the
container), inside the harness container, and on linus. Stdlib only, so it
runs before any ROS environment is sourced.

Plan P9: "every result records image ID, nvpmodel and jetson_clocks state".
What it records, each field null when this side cannot see it:
  - host, kernel, time, whether this process is inside a container
    (/.dockerenv or /run/.containerenv), python and ROS_DISTRO;
  - the DDS environment (domain, discovery server, profile file, RMW);
  - the image: /opt/f1tenth_ws/SOURCES (commit, submodules, tree hash) and
    CAR_SYNC.json when present (inside the container), and, on the host,
    the running f1tenth_car container's image ID (docker inspect) plus the
    pin file ~/.config/f1tenth/car_image;
  - the power mode (`nvpmodel -q`, the host only) and the clocks read from
    sysfs: every CPU's governor and min/max/current frequency, the GPU's
    devfreq min/max/current. jetson_clocks pins min to max, so
    `jetson_clocks_likely` is true when every CPU and the GPU have
    min == max (sysfs is readable without root, `jetson_clocks --show` is
    not);
  - the harness checkout's `git describe` (the scripts themselves).

  run_context.py --out FILE.json [--extra KEY=VALUE ...]
"""
import argparse
import glob
import json
import os
import platform
import shutil
import subprocess
import sys
import time

IMAGE_SOURCES = '/opt/f1tenth_ws/SOURCES'
IMAGE_CAR_SYNC = '/opt/f1tenth_ws/CAR_SYNC.json'
CONTAINER = 'f1tenth_car'
ENV_KEYS = ('ROS_DISTRO', 'ROS_DOMAIN_ID', 'ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT',
            'ROS_AUTOMATIC_DISCOVERY_RANGE', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION',
            'FASTRTPS_DEFAULT_PROFILES_FILE', 'F1T_CONTAINER_ROLE', 'F1T_PROCESS_BACKEND',
            'F1TENTH_IMAGE', 'CUDA_CACHE_PATH')


def read_text(path, limit=65536):
    """Return the file's text, or None when it cannot be read."""
    try:
        with open(path, errors='replace') as f:
            return f.read(limit)
    except OSError:
        return None


def read_int(path):
    """Return the integer in a sysfs file, or None."""
    text = read_text(path, 64)
    try:
        return int(text.strip()) if text is not None else None
    except ValueError:
        return None


def run(cmd, timeout=5.0):
    """Return a command's stripped stdout, or None if it is missing or fails."""
    if shutil.which(cmd[0]) is None:
        return None
    try:
        out = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                             timeout=timeout, check=False, text=True)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return out.stdout.strip() if out.returncode == 0 else None


def parse_sources(text):
    """Turn the image's SOURCES file into a dict (commit, subject, tree hash, submodules)."""
    if not text:
        return None
    res = {'submodules': {}}
    for line in text.splitlines():
        key, _, rest = line.partition(' ')
        if key == 'submodule':
            parts = rest.split()
            if len(parts) >= 2:
                res['submodules'][parts[0]] = parts[1]
        elif key in ('commit', 'subject', 'tree_sha256', 'branch', 'describe'):
            res[key] = rest
    return res


def cpu_clocks(root='/sys/devices/system/cpu'):
    """Per CPU: governor and min/max/current kHz from cpufreq."""
    out = {}
    for d in sorted(glob.glob(os.path.join(root, 'cpu[0-9]*')),
                    key=lambda p: int(os.path.basename(p)[3:])):
        cf = os.path.join(d, 'cpufreq')
        if not os.path.isdir(cf):
            continue
        out[os.path.basename(d)] = {
            'online': read_int(os.path.join(d, 'online')),
            'governor': (read_text(os.path.join(cf, 'scaling_governor')) or '').strip() or None,
            'min_khz': read_int(os.path.join(cf, 'scaling_min_freq')),
            'max_khz': read_int(os.path.join(cf, 'scaling_max_freq')),
            'cur_khz': read_int(os.path.join(cf, 'scaling_cur_freq')),
        }
    return out


def gpu_clocks(pattern='/sys/class/devfreq/*'):
    """Devfreq entries that look like the GPU (Orin: 17000000.gpu / gpu-gpc-0)."""
    out = {}
    for d in sorted(glob.glob(pattern)):
        name = os.path.basename(d)
        if 'gpu' not in name and 'gv11b' not in name and 'ga10b' not in name:
            continue
        out[name] = {'min_hz': read_int(os.path.join(d, 'min_freq')),
                     'max_hz': read_int(os.path.join(d, 'max_freq')),
                     'cur_hz': read_int(os.path.join(d, 'cur_freq')),
                     'governor': (read_text(os.path.join(d, 'governor')) or '').strip() or None}
    return out


def clocks_pinned(cpus, gpus):
    """Tell whether every online CPU and GPU devfreq has min == max (None: unreadable)."""
    cpu_vals = [c for c in cpus.values() if c.get('online') != 0]
    if not cpu_vals and not gpus:
        return None
    for c in cpu_vals:
        if c['min_khz'] is None or c['max_khz'] is None or c['min_khz'] != c['max_khz']:
            return False
    for g in gpus.values():
        if g['min_hz'] is None or g['max_hz'] is None or g['min_hz'] != g['max_hz']:
            return False
    return True


def in_container():
    """Whether this process runs inside a container."""
    return os.path.exists('/.dockerenv') or os.path.exists('/run/.containerenv')


def repo_describe():
    """`git describe` of the checkout this script lives in (the harness itself)."""
    here = os.path.dirname(os.path.abspath(__file__))
    return run(['git', '-C', here, 'describe', '--always', '--dirty', '--tags'])


def host_image():
    """On the host: the running f1tenth_car's image ID and the pinned image file."""
    pin = read_text(os.path.expanduser('~/.config/f1tenth/car_image'))
    image_id = run(['docker', 'inspect', '--type', 'container', '-f', '{{.Image}}', CONTAINER])
    state = run(['docker', 'inspect', '--type', 'container', '-f', '{{.State.Status}}',
                 CONTAINER])
    return {'container': CONTAINER, 'state': state, 'image_id': image_id,
            'pin_file': pin.strip() if pin else None}


def collect(extra):
    """Everything this side can see."""
    cpus = cpu_clocks()
    gpus = gpu_clocks()
    container = in_container()
    ctx = {
        'time': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'host': platform.node(),
        'kernel': platform.release(),
        'machine': platform.machine(),
        'in_container': container,
        'python': platform.python_version(),
        'env': {k: os.environ.get(k) for k in ENV_KEYS},
        'image_sources': parse_sources(read_text(IMAGE_SOURCES)),
        'image_car_sync_present': os.path.exists(IMAGE_CAR_SYNC),
        'host_image': None if container else host_image(),
        'nvpmodel': run(['nvpmodel', '-q']),
        'l4t_release': (read_text('/etc/nv_tegra_release') or '').strip()[:200] or None,
        'cpu_clocks': cpus,
        'gpu_clocks': gpus,
        'jetson_clocks_likely': clocks_pinned(cpus, gpus),
        'harness_git_describe': repo_describe(),
        'argv0': sys.argv[0],
    }
    ctx.update(extra)
    return ctx


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('--out', required=True, help='JSON file to write')
    ap.add_argument('--extra', nargs='*', default=[], metavar='KEY=VALUE',
                    help='extra fields to record')
    a = ap.parse_args(argv)
    extra = {}
    for kv in a.extra:
        k, _, v = kv.partition('=')
        extra[k] = v
    ctx = collect(extra)
    with open(a.out, 'w') as f:
        json.dump(ctx, f, indent=2, sort_keys=True)
    print(json.dumps({k: ctx[k] for k in ('host', 'in_container', 'nvpmodel',
                                          'jetson_clocks_likely')}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
