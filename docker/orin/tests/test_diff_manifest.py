"""diff_manifest.py: the image's versions against jazzy_versions.manifest."""

import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ORIN = os.path.dirname(HERE)
DIFF = os.path.join(ORIN, 'diff_manifest.py')
sys.path.insert(0, ORIN)

import diff_manifest  # noqa: E402

MANIFEST = """[ros]
rclpy                  7.1.12               # comment
[python]
onnx                   1.22.0
torch                  NONE_IN_REFERENCE
[submodules]
vesc                   0e74a60090eb084822c7a90e61450fdd5a840a21   # x
[snapshots]
ubuntu                 20261024T000000Z
ros                    2026-09-11
[rust]
rustc                  1.99.1
rosidl_generator_rs    0.5.0
cargo_lock             src/rust/Cargo.lock
[gateway_venv]
requirements           reqs.txt
"""

IMAGE = """[ros_dpkg]
ros-jazzy-rclpy 7.1.12-1noble.20260902.143245
ros-jazzy-rosidl-generator-rs 0.5.0-1noble.20260902
[apt]
[pip]
onnx==1.22.0
torch==2.8.0
[gateway_venv]
fastapi==0.115.6
av==17.0.0
extra-pkg==1.0
[snapshots]
ubuntu 20261024T000000Z
ros 2026-09-11
[rust]
rustc 1.99.1
cargo_lock_sha256 0123456789abcdef0123
[workspace]
submodule src/f1tenth_hardware/vesc 0e74a60090eb084822c7a90e61450fdd5a840a21 https://x
"""


def write(tmp_path, image=IMAGE):
    (tmp_path / 'manifest').write_text(MANIFEST)
    (tmp_path / 'reqs.txt').write_text('# pins\nfastapi==0.115.6\nav==16.1.0  # PyAV\n')
    (tmp_path / 'thor.txt').write_text('ros-jazzy-rclpy 7.1.12-1noble.20260902.143245\n')
    (tmp_path / 'image.txt').write_text(image)
    files = [str(tmp_path / n) for n in ('manifest', 'thor.txt', 'image.txt')]
    return subprocess.run([sys.executable, DIFF] + files, capture_output=True, text=True)


def rows_of(text):
    """Map (area, component) to its table row."""
    rows = {}
    for line in text.splitlines():
        if line.startswith('| ') and not line.startswith('| Area'):
            cells = [c.strip() for c in line.strip('|').split('|')]
            rows[(cells[0], cells[1])] = line
    return rows


def ok(row):
    return row.rstrip().endswith('| = |')


def test_every_section_is_compared(tmp_path):
    r = write(tmp_path)
    assert r.returncode == 0, r.stderr
    rows = rows_of(r.stdout)
    for key in (('ros', 'rclpy'), ('python', 'onnx'), ('submodule', 'vesc'),
                ('snapshot', 'ubuntu'), ('rust', 'rustc'), ('rust', 'rosidl_generator_rs'),
                ('gateway_venv', 'fastapi')):
        assert ok(rows[key]), rows[key]
    assert 'n/a' in rows[('python', 'torch')]
    assert 'info' in rows[('rust', 'cargo_lock')]
    # The venv: each pin of the requirements file, then the extras.
    assert '| 16.1.0 | 17.0.0 | differs |' in rows[('gateway_venv', 'av')]
    assert 'differs (extra)' in rows[('gateway_venv', 'extra-pkg')]
    assert '**Pinned components:** 12, differing 2.' in r.stdout


def test_an_old_image_without_the_new_sections_reports_them_as_differing(tmp_path):
    rows = rows_of(write(tmp_path, '[ros_dpkg]\n[pip]\n[workspace]\n').stdout)
    assert 'unknown | differs' in rows[('snapshot', 'ubuntu')]
    assert 'missing | differs' in rows[('rust', 'rustc')]
    assert 'not installed | differs' in rows[('gateway_venv', 'fastapi')]


def test_the_shipped_manifest_parses_and_names_the_shipped_requirements():
    m = diff_manifest.sections(os.path.join(ORIN, 'jazzy_versions.manifest'))
    for section in ('ros', 'python', 'zed', 'submodules', 'snapshots', 'rust', 'gateway_venv'):
        assert m.get(section), section
    req = dict(diff_manifest.mval(line) for line in m['gateway_venv'])['requirements']
    with open(os.path.join(ORIN, req)) as f:
        lines = f.readlines()
    pins = diff_manifest.pins(lines)
    assert pins['faster-whisper'] == '1.2.1' and pins['av'] == '16.1.0'
    # Every line of the requirements is an exact pin (the venv is --no-deps).
    body = [line.split('#')[0].strip() for line in lines if line.split('#')[0].strip()]
    assert all('==' in line for line in body), body
    assert len(body) == len(pins)
    snaps = dict(diff_manifest.mval(line) for line in m['snapshots'])
    with open(os.path.join(ORIN, 'Dockerfile')) as f:
        dockerfile = f.read()
    assert f'ARG UBUNTU_SNAPSHOT={snaps["ubuntu"]}\n' in dockerfile
    assert f'ARG ROS_SNAPSHOT={snaps["ros"]}\n' in dockerfile
