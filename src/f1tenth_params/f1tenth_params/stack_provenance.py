"""Where a running stack came from: image provenance and the launch command (pure, stdlib only).

Shared by f1tenth_logger's mission_logger_node (every run's manifest) and
f1tenth_diagnostics' stack_info_node (/viz/stack_info). Moved here from
mission_logger_node.py unchanged (2026-11-24); the logger imports them from
here. f1tenth_params is the dependency-free leaf both packages already depend
on, so neither has to depend on the other.
"""

import json
import os


# The CAR_SYNC.json keys a manifest carries (the whole file is in the image).
_CAR_SYNC_KEYS = ('captured_at', 'native_branch', 'native_head', 'main_commit')


def read_image_provenance(sources_path, car_sync_path):
    """Return the manifest's `git` block for an image without .git (pure).

    Files only. sources_path: docker/orin/stage_sources.sh's SOURCES
    ("commit <sha>", "subject ...", "branch ...", "describe ...",
    "submodule <path> <sha> <url>", "tree_sha256 <hash>"); car_sync_path:
    docker/orin/car_sync.json, what of the car's native tree the image
    carries. Either may be missing. The image's tree IS that commit (a
    `git archive`), so dirty is False when a commit is known. source is
    'image' once either file was read.
    """
    info = {'commit': None, 'dirty': None, 'branch': None}
    read_any = False
    lines = None
    if sources_path:
        try:
            with open(sources_path) as fh:
                lines = fh.read().splitlines()
        except OSError:
            lines = None
    if lines is not None:
        read_any = True
        submodules = {}
        for line in lines:
            key, _, rest = line.partition(' ')
            rest = rest.strip()
            if key in ('commit', 'branch'):
                info[key] = rest or None
            elif key in ('subject', 'describe', 'tree_sha256'):
                info[key] = rest
            elif key == 'submodule':
                parts = rest.split()
                if len(parts) >= 2 and parts[1] != 'skipped':
                    submodules[parts[0]] = parts[1]
        if submodules:
            info['submodules'] = submodules
        info['dirty'] = False if info['commit'] else None
        info['sources_file'] = sources_path
    if car_sync_path:
        try:
            with open(car_sync_path) as fh:
                car_sync = json.load(fh)
        except (OSError, ValueError):
            car_sync = None
        if isinstance(car_sync, dict):
            read_any = True
            info['car_sync'] = {k: car_sync.get(k) for k in _CAR_SYNC_KEYS}
    if read_any:
        info['source'] = 'image'
    return info


def stack_launch_cmd(proc_root='/proc'):
    """Best effort: the command line of the stack's top-level launch, else None.

    Read from /proc, so it sees the launch arguments the stack really got
    (profile + ~/.config/f1tenth_car/stack_args), which resolved_params cannot.
    """
    try:
        pids = sorted((e for e in os.listdir(proc_root) if e.isdigit()), key=int)
    except OSError:
        return None
    for pid in pids:
        try:
            with open(os.path.join(proc_root, pid, 'cmdline'), 'rb') as fh:
                argv = [a for a in fh.read().decode(errors='replace').split('\0') if a]
        except OSError:
            continue
        if any(a.endswith(('supervisor_bringup.launch.py', 'stack_bringup.launch.py'))
               for a in argv):
            return ' '.join(argv)
    return None
