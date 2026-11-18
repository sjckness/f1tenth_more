#!/usr/bin/env python3
"""
Point the image's INSTALLED stack_params.yaml at the image's own paths.

Replaces set_llama_paths.py (Phase 5d decision 2026-10-07, extended for the
car's container, plan P4.5). Only the installed copy under
install/share/f1tenth_params is edited; the repo defaults stay those of the
native car. Three keys:

- llama_server_path, llama_server_cwd: the image's llama.cpp build
  (/opt/llama.cpp, sm_87) instead of the Humble host's /scratch/fabiocar/llama.cpp;
- yolo_engine_path: the TensorRT engine `docker_orin.sh engine` builds into
  the TRT cache volume, /var/cache/f1tenth/trt/<TensorRT>/<model stem>.trt<x.y.z>.fp16.engine,
  instead of '' (next to the model, inside the image). Used only with
  yolo_backend:=engine; a missing engine falls back to the .pt.

Refuses (exit 1, nothing written) unless every key exists exactly once with
the expected old value, yolo_model is the model the engine name is for, and
nothing else changes, so a reworked yaml fails the image build instead of
being half-edited. Refuses a symlink too: with --symlink-install the
installed file is the repo's own.

usage: set_installed_params.py [--llama-server P] [--llama-cwd P]
                               [--yolo-engine P] [--dry-run] <installed stack_params.yaml>
"""

import argparse
import os
import re
import sys

import yaml

LLAMA_SERVER = '/opt/llama.cpp/bin/llama-server'
LLAMA_CWD = '/opt/llama.cpp'
YOLO_ENGINE = '/var/cache/f1tenth/trt/10.3.0.26/yolo26s-seg.trt10.3.0.fp16.engine'

# key -> the value the repo must still have.
EXPECTED_OLD = {
    'llama_server_path': '/scratch/fabiocar/llama.cpp/build/bin/llama-server',
    'llama_server_cwd': '/scratch/fabiocar/llama.cpp',
    'yolo_engine_path': '',
}
# Keys that must hold these values for the new ones to make sense.
EXPECTED_CONTEXT = {
    'yolo_model': 'yolo26s-seg.pt',      # the engine above is built from it
    'yolo_backend': 'pt',                # the image keeps .pt the default
}


class Refused(Exception):
    """The yaml is not the one these edits were written for."""


def edit(text: str, new_values: dict) -> str:
    """Return `text` with each key's `default:` replaced (pure; raises Refused)."""
    before = yaml.safe_load(text)
    for key, old in {**EXPECTED_OLD, **EXPECTED_CONTEXT}.items():
        if not isinstance(before.get(key), dict) or 'default' not in before[key]:
            raise Refused(f'{key}: no `default` in the yaml')
        if before[key]['default'] != old:
            raise Refused(f'{key} default is {before[key]["default"]!r}, expected {old!r}')
    for key, value in new_values.items():
        if not value or any(c.isspace() or c in '\'"#:{}[],&*!|>%@`' for c in value):
            raise Refused(f'{key}: {value!r} is not a plain path')
        pattern = re.compile(r'^(%s:\n  default: )(\S+)$' % re.escape(key), re.M)
        text, n = pattern.subn(lambda m, v=value: m.group(1) + v, text)
        if n != 1:
            raise Refused(f'{key} block matched {n} times, expected 1')
    after = yaml.safe_load(text)
    changed = sorted(k for k in set(before) | set(after) if before.get(k) != after.get(k))
    if changed != sorted(new_values):
        raise Refused(f'unexpected keys changed: {changed}')
    for key, value in new_values.items():
        if after[key]['default'] != value:
            raise Refused(f'{key} reads back as {after[key]["default"]!r}, not {value!r}')
    return text


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('path', help='the INSTALLED stack_params.yaml')
    ap.add_argument('--llama-server', default=LLAMA_SERVER)
    ap.add_argument('--llama-cwd', default=LLAMA_CWD)
    ap.add_argument('--yolo-engine', default=YOLO_ENGINE)
    ap.add_argument('--dry-run', action='store_true', help='check and print, write nothing')
    args = ap.parse_args(argv)
    if os.path.islink(args.path):
        print(f'{args.path}: a symlink (--symlink-install?): refusing to edit the repo file',
              file=sys.stderr)
        return 1
    new_values = {'llama_server_path': args.llama_server, 'llama_server_cwd': args.llama_cwd,
                  'yolo_engine_path': args.yolo_engine}
    try:
        with open(args.path) as f:
            text = f.read()
        out = edit(text, new_values)
    except (OSError, Refused, yaml.YAMLError) as exc:
        print(f'{args.path}: {exc}', file=sys.stderr)
        return 1
    if not args.dry_run:
        tmp = f'{args.path}.tmp'
        with open(tmp, 'w') as f:
            f.write(out)
        os.replace(tmp, args.path)
    for key, value in new_values.items():
        print(f'{key}: {EXPECTED_OLD[key]!r} -> {value!r}'
              + (' (dry run)' if args.dry_run else ''))
    return 0


if __name__ == '__main__':
    sys.exit(main())
