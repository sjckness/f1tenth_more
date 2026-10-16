#!/usr/bin/env python3
"""Point the image's INSTALLED stack_params.yaml at the image's llama-server.

Phase 5d decision (2026-10-07): the container uses llama.cpp built in the image
(/opt/llama.cpp, sm_87), selected through the existing llama_server_path /
llama_server_cwd parameters, with no symlink. Only the installed copy under
install/share/f1tenth_params is changed; the repo default stays the Humble
host's /scratch/fabiocar/llama.cpp. Refuses (exit 1) unless both keys exist
exactly once with the expected old values, so a reworked yaml fails the build
instead of being half-edited.

usage: set_llama_paths.py <installed stack_params.yaml> <server binary> <server cwd>
"""
import re
import sys

import yaml

EXPECTED_OLD = {
    'llama_server_path': '/scratch/fabiocar/llama.cpp/build/bin/llama-server',
    'llama_server_cwd': '/scratch/fabiocar/llama.cpp',
}

path, server_bin, server_cwd = sys.argv[1:4]
new_values = {'llama_server_path': server_bin, 'llama_server_cwd': server_cwd}
text = open(path).read()
before = yaml.safe_load(text)
for key, old in EXPECTED_OLD.items():
    if before[key]['default'] != old:
        sys.exit(f'{path}: {key} default is {before[key]["default"]!r}, expected {old!r}')
for key, value in new_values.items():
    pattern = re.compile(r'^(%s:\n  default: )(\S+)$' % re.escape(key), re.M)
    text, n = pattern.subn(lambda m: m.group(1) + value, text)
    if n != 1:
        sys.exit(f'{path}: {key} block matched {n} times, expected 1')
after = yaml.safe_load(text)
for key, value in new_values.items():
    assert after[key]['default'] == value
changed = [k for k in before if before[k] != after.get(k)]
if sorted(changed) != sorted(new_values):
    sys.exit(f'{path}: unexpected keys changed: {changed}')
open(path, 'w').write(text)
for key, value in new_values.items():
    print(f'{key}: {EXPECTED_OLD[key]} -> {value}')
