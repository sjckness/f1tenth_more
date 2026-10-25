"""
set_installed_params.py: the checked edit of the image's installed stack_params.yaml.

Run on linus against the repo's own stack_params.yaml:
    python3 -m pytest docker/orin/tests -q
"""

import os
import shutil
import subprocess
import sys

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(os.path.dirname(HERE), 'set_installed_params.py')
REPO_PARAMS = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(HERE))),
                           'src', 'f1tenth_params', 'config', 'stack_params.yaml')
sys.dont_write_bytecode = True       # no __pycache__ in docker/orin (the build context)
sys.path.insert(0, os.path.dirname(HERE))
import set_installed_params as sip  # noqa: E402, I100


def _run(*args):
    return subprocess.run([sys.executable, SCRIPT, *args], capture_output=True, text=True)


@pytest.fixture
def installed(tmp_path):
    path = tmp_path / 'stack_params.yaml'
    shutil.copyfile(REPO_PARAMS, path)
    return path


def test_the_three_keys_change_and_nothing_else(installed):
    before = yaml.safe_load(installed.read_text())
    proc = _run(str(installed))
    assert proc.returncode == 0, proc.stderr
    after = yaml.safe_load(installed.read_text())
    assert after['llama_server_path']['default'] == '/opt/llama.cpp/bin/llama-server'
    assert after['llama_server_cwd']['default'] == '/opt/llama.cpp'
    assert after['yolo_engine_path']['default'] == (
        '/var/cache/f1tenth/trt/10.3.0.26/yolo26s-seg.trt10.3.0.fp16.engine')
    changed = sorted(k for k in before if before[k] != after[k])
    assert changed == ['llama_server_cwd', 'llama_server_path', 'yolo_engine_path']
    # Descriptions and comments are untouched: only three lines differ.
    old_lines = open(REPO_PARAMS).read().splitlines()
    new_lines = installed.read_text().splitlines()
    assert len(old_lines) == len(new_lines)
    assert sum(a != b for a, b in zip(old_lines, new_lines)) == 3


def test_a_second_run_is_refused_and_writes_nothing(installed):
    assert _run(str(installed)).returncode == 0
    text = installed.read_text()
    proc = _run(str(installed))
    assert proc.returncode == 1 and 'expected' in proc.stderr
    assert installed.read_text() == text


@pytest.mark.parametrize('key,line', [
    ('llama_server_path', '  default: /somewhere/else/llama-server'),
    ('yolo_engine_path', '  default: /models/x.engine'),
    ('yolo_model', '  default: yolo26n.pt'),
    ('yolo_backend', '  default: engine'),
])
def test_an_unexpected_old_value_is_refused(installed, key, line):
    text = installed.read_text()
    block = f'{key}:\n'
    start = text.index(block) + len(block)
    end = text.index('\n', start)
    installed.write_text(text[:start] + line + text[end:])
    mutated = installed.read_text()
    proc = _run(str(installed))
    assert proc.returncode == 1 and key in proc.stderr
    assert installed.read_text() == mutated


def test_a_symlink_is_refused(installed, tmp_path):
    link = tmp_path / 'link.yaml'
    link.symlink_to(installed)
    proc = _run(str(link))
    assert proc.returncode == 1 and 'symlink' in proc.stderr


def test_dry_run_writes_nothing(installed):
    text = installed.read_text()
    proc = _run('--dry-run', str(installed))
    assert proc.returncode == 0 and 'dry run' in proc.stdout
    assert installed.read_text() == text


def test_values_that_are_not_plain_paths_are_refused(installed):
    with pytest.raises(sip.Refused, match='plain path'):
        sip.edit(installed.read_text(), {'llama_server_path': '/a b',
                                         'llama_server_cwd': '/x', 'yolo_engine_path': '/y'})


def test_a_duplicated_block_is_refused():
    text = ('llama_server_path:\n  default: /scratch/fabiocar/llama.cpp/build/bin/llama-server\n'
            'llama_server_cwd:\n  default: /scratch/fabiocar/llama.cpp\n'
            "yolo_engine_path:\n  default: ''\n"
            'yolo_model:\n  default: yolo26s-seg.pt\n'
            'yolo_backend:\n  default: pt\n'
            '# a stray copy:\n#llama_server_cwd:\n#  default: x\n')
    out = sip.edit(text, {'llama_server_path': '/a', 'llama_server_cwd': '/b',
                          'yolo_engine_path': '/c'})
    assert '#  default: x' in out
    with pytest.raises(sip.Refused, match='matched 2 times'):
        sip.edit(text + 'llama_server_cwd:\n  default: /scratch/fabiocar/llama.cpp\n',
                 {'llama_server_path': '/a', 'llama_server_cwd': '/b',
                  'yolo_engine_path': '/c'})
