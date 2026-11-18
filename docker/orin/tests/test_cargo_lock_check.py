"""cargo_lock_check.py: the image's Rust build must resolve the committed crates."""

import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ORIN = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(ORIN))
CHECK = os.path.join(ORIN, 'cargo_lock_check.py')
sys.path.insert(0, ORIN)

import cargo_lock_check  # noqa: E402, I100

LOCK = """version = 4

[[package]]
name = "rclrs"
version = "0.8.0"
source = "registry+https://github.com/rust-lang/crates.io-index"
checksum = "aaaa"
dependencies = [
 "rosidl_runtime_rs",
]

[[package]]
name = "rosidl_runtime_rs"
version = "0.7.0"
source = "registry+https://github.com/rust-lang/crates.io-index"
checksum = "bbbb"

[[patch.unused]]
name = "sensor_msgs"
version = "5.3.7"
"""


def run(tmp_path, before, after):
    a, b = tmp_path / 'before.lock', tmp_path / 'after.lock'
    a.write_text(before)
    b.write_text(after)
    return subprocess.run([sys.executable, CHECK, str(a), str(b)], capture_output=True, text=True)


def test_equal_packages_pass_even_when_the_unused_patch_list_differs(tmp_path):
    after = LOCK.replace('version = "5.3.7"', 'version = "5.3.8"')
    after += '\n[[patch.unused]]\nname = "nav_msgs"\nversion = "5.3.8"\n'
    r = run(tmp_path, LOCK, after)
    assert r.returncode == 0, r.stderr
    assert 'patch.unused' in r.stdout and '2 packages' in r.stdout


def test_a_changed_version_fails(tmp_path):
    r = run(tmp_path, LOCK, LOCK.replace('0.7.0', '0.7.1'))
    assert r.returncode == 1
    assert 'removed: rosidl_runtime_rs 0.7.0' in r.stderr
    assert 'added: rosidl_runtime_rs 0.7.1' in r.stderr


def test_a_changed_checksum_or_dependency_list_fails(tmp_path):
    r = run(tmp_path, LOCK, LOCK.replace('"aaaa"', '"cccc"'))
    assert r.returncode == 1 and 'changed: rclrs 0.8.0' in r.stderr and 'checksum' in r.stderr
    r = run(tmp_path, LOCK, LOCK.replace(' "rosidl_runtime_rs",\n', ''))
    assert r.returncode == 1 and 'dependencies' in r.stderr


def test_an_added_package_fails(tmp_path):
    extra = '\n[[package]]\nname = "libc"\nversion = "0.2.0"\nsource = "registry+x"\n'
    r = run(tmp_path, LOCK, LOCK + extra + 'checksum = "dddd"\n')
    assert r.returncode == 1 and 'added: libc 0.2.0' in r.stderr


def test_usage_and_unreadable_input_exit_2(tmp_path):
    r = subprocess.run([sys.executable, CHECK], capture_output=True, text=True)
    assert r.returncode == 2 and 'usage' in r.stderr
    r = run(tmp_path, LOCK, 'not = [toml')
    assert r.returncode == 2


def test_the_committed_lock_equals_itself_without_its_unused_patches():
    path = os.path.join(REPO, 'src', 'rust', 'Cargo.lock')
    if not os.path.exists(path):
        pytest.skip('no src/rust/Cargo.lock')
    import tomllib
    with open(path, 'rb') as f:
        lock = tomllib.load(f)
    stripped = dict(lock)
    stripped.pop('patch', None)
    errors, notes = cargo_lock_check.compare(lock, stripped)
    assert errors == []
    assert notes == [] or 'patch.unused' in notes[0]
