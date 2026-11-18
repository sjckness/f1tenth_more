"""
The Dockerfile's inline shell, run on linus with mocks (the image builds only on the Orin).

Each test takes one RUN instruction out of docker/orin/Dockerfile as Docker
would run it (continuations joined, comment lines dropped, --mount flags
removed), points its absolute paths into tmp_path, and runs it under
`bash -o pipefail -c` with shell functions standing in for the commands that
would change the machine (apt-get, groupmod, find over /, colcon, rm, ...).
Plus consistency checks between the Dockerfile, .dockerignore and
stage_sources.sh.
"""

import os
import re
import subprocess

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ORIN = os.path.dirname(HERE)
DOCKERFILE = os.path.join(ORIN, 'Dockerfile')


def instructions():
    """Return the Dockerfile's instructions, comment lines inside them dropped."""
    out, cur = [], None
    with open(DOCKERFILE) as f:
        for line in f.read().splitlines():
            stripped = line.strip()
            if stripped.startswith('#') or (cur is None and not stripped):
                continue
            cur = line if cur is None else cur + '\n' + line
            if not cur.rstrip().endswith('\\'):
                out.append(cur)
                cur = None
    return out


def run_body(marker):
    """Return the shell command of the one RUN instruction that contains `marker`."""
    hits = [i for i in instructions() if i.startswith('RUN') and marker in i]
    assert len(hits) == 1, (marker, len(hits))
    cmd = re.sub(r'\\\n', ' ', hits[0])
    return re.sub(r'^RUN\s+((--mount=\S+)\s+)*', '', cmd)


def bash(script, env=None, cwd=None):
    e = dict(os.environ)
    e.update(env or {})
    return subprocess.run(['bash', '-o', 'pipefail', '-c', script], capture_output=True,
                          text=True, env=e, cwd=cwd)


def read(path):
    return path.read_text() if path.exists() else ''


# ---- base stage: group IDs ------------------------------------------------------------

GROUP_MOCKS = r"""
db=$TMP/group.db; log=$TMP/log
getent() {
  local k=$2 line
  while IFS=: read -r n g; do
    if [ "$n" = "$k" ] || [ "$g" = "$k" ]; then echo "$n:x:$g:"; return 0; fi
  done < "$db"
  return 2
}
groupmod() { echo "groupmod $*" >> "$log"; sed -i "s/^$3:.*/$3:$2/" "$db"; }
groupadd() {
  echo "groupadd $*" >> "$log"
  if [ "$1" = -g ]; then echo "$3:$2" >> "$db"; else echo "$2:999" >> "$db"; fi
}
find() { echo "find $*" >> "$log"; }
useradd() { echo "useradd $*" >> "$log"; }
"""


def group_run(tmp_path, groups, **args):
    (tmp_path / 'group.db').write_text(''.join(f'{n}:{g}\n' for n, g in groups.items()))
    env = {'TMP': str(tmp_path), 'USER_NAME': 'fabiocar', 'USER_UID': '1004', 'USER_GID': '1004',
           'ZED_GID': '1005', 'JTOP_GID': '1003', 'VIDEO_GID': '44', 'DIALOUT_GID': '20',
           'RENDER_GID': '104', 'INPUT_GID': ''}
    env.update(args)
    r = bash(GROUP_MOCKS + run_body('set_gid()'), env)
    db = dict(line.split(':') for line in (tmp_path / 'group.db').read_text().split())
    return r, db, read(tmp_path / 'log')


def test_render_moves_to_the_host_gid_and_its_files_follow(tmp_path):
    r, db, log = group_run(tmp_path, {'video': '44', 'dialout': '20', 'render': '994',
                                      'input': '995'}, INPUT_GID='106')
    assert r.returncode == 0, r.stderr
    assert db['render'] == '104' and db['input'] == '106'
    assert 'find / -xdev -gid 994 -exec chgrp -h 104 {} +' in log
    assert 'find / -xdev -gid 995 -exec chgrp -h 106 {} +' in log
    assert db['zed'] == '1005' and db['jtop'] == '1003' and db['fabiocar'] == '1004'
    assert ('useradd -m -d /home/fabiocar -s /bin/bash -u 1004 -g 1004 '
            '-G zed,jtop,video,render,dialout,input fabiocar') in log


def test_no_host_gid_keeps_the_image_gid_and_a_missing_group_is_created(tmp_path):
    r, db, log = group_run(tmp_path, {'video': '44', 'dialout': '20', 'render': '104'})
    assert r.returncode == 0, r.stderr
    assert 'group input: created' in r.stdout and db['input'] == '999'
    assert 'groupmod' not in log


def test_a_gid_another_group_holds_fails_the_build(tmp_path):
    r, db, log = group_run(tmp_path, {'video': '44', 'dialout': '20', 'render': '994',
                                      'input': '995', 'kvm': '104'})
    assert r.returncode != 0
    assert 'GID 104 is taken by group kvm' in r.stderr
    assert 'useradd' not in log


# ---- base stage: the Ubuntu snapshot ------------------------------------------------------

SNAP_MOCKS = r"""
apt-get() { echo "apt-get $*" >> "$TMP/log"; }
rm() { echo "rm $*" >> "$TMP/log"; }
"""

EXPECTED_SOURCES = """Types: deb
URIs: https://snapshot.ubuntu.com/ubuntu/20261024T000000Z/
Suites: noble noble-updates noble-backports
Components: main universe restricted multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg

Types: deb
URIs: https://snapshot.ubuntu.com/ubuntu/20261024T000000Z/
Suites: noble-security
Components: main universe restricted multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
"""


def snap_run(tmp_path, snapshot):
    body = run_body('snapshot.ubuntu.com')
    body = body.replace('/etc/apt/sources.list.d/ubuntu.sources', str(tmp_path / 'ubuntu.sources'))
    body = body.replace('/etc/f1tenth_ubuntu_snapshot', str(tmp_path / 'marker'))
    return bash(SNAP_MOCKS + body, {'TMP': str(tmp_path), 'UBUNTU_SNAPSHOT': snapshot})


def test_the_snapshot_sources_are_the_ones_apt_accepted(tmp_path):
    # EXPECTED_SOURCES is the file an `apt-get update` (arm64, non-root, own
    # state dir) fetched and verified against ubuntu-archive-keyring on linus
    # on 2026-10-24 (output/orin_container/w2.md).
    r = snap_run(tmp_path, '20261024T000000Z')
    assert r.returncode == 0, r.stderr
    assert (tmp_path / 'ubuntu.sources').read_text() == EXPECTED_SOURCES
    assert (tmp_path / 'marker').read_text() == '20261024T000000Z\n'
    log = (tmp_path / 'log').read_text().splitlines()
    assert log[0] == ('apt-get -o Acquire::https::Verify-Peer=false '
                      '-o Acquire::https::Verify-Host=false update')
    assert 'install -y --no-install-recommends ca-certificates' in log[1]


def test_snapshot_none_keeps_the_live_archive_and_says_so(tmp_path):
    r = snap_run(tmp_path, 'none')
    assert r.returncode == 0, r.stderr
    assert not (tmp_path / 'ubuntu.sources').exists() and not (tmp_path / 'log').exists()
    assert (tmp_path / 'marker').read_text() == 'none\n'


def test_a_malformed_snapshot_id_fails(tmp_path):
    r = snap_run(tmp_path, '2026-10-24')
    assert r.returncode != 0 and 'YYYYMMDDTHHMMSSZ' in r.stderr


# ---- cuda stage: L17 -----------------------------------------------------------------------

def test_l17_drops_every_static_library_but_cudart_static(tmp_path):
    cmd = run_body('L17_dropped_static_libs.txt')
    tail = cmd[cmd.index("find /usr/local/cuda-12.9 -name 'lib*_static*.a'"):]
    tail = tail.replace('/usr/local/cuda-12.9', str(tmp_path / 'cuda'))
    lib = tmp_path / 'cuda' / 'targets' / 'aarch64-linux' / 'lib'
    lib.mkdir(parents=True)
    cupti = tmp_path / 'cuda' / 'extras' / 'CUPTI' / 'lib64'
    cupti.mkdir(parents=True)
    for name, size in (('libcublas_static.a', 300), ('libcublasLt_static.a', 500),
                       ('libcufft_static_nocallback.a', 200), ('libnvrtc-builtins_static.a', 10),
                       ('libcudart_static.a', 7), ('libcudadevrt.a', 5), ('libculibos.a', 3),
                       ('libcublas.so.12', 1)):
        (lib / name).write_bytes(b'x' * size)
    (cupti / 'libcupti_static.a').write_bytes(b'x' * 50)
    r = bash(tail)
    assert r.returncode == 0, r.stderr
    assert sorted(os.listdir(lib)) == ['libcublas.so.12', 'libcudadevrt.a', 'libcudart_static.a',
                                       'libculibos.a']
    assert os.listdir(cupti) == []
    dropped = (tmp_path / 'cuda' / 'L17_dropped_static_libs.txt').read_text().split('\n')
    assert len([d for d in dropped if d]) == 5 and dropped[0].startswith('10 ')
    assert 'L17: dropping 5 static libraries' in r.stdout


# ---- ws stage: phase B (Rust) -------------------------------------------------------------

PHASE_B_MOCKS = r"""
rustc() { echo "host: aarch64-unknown-linux-gnu"; }
rm() { echo "rm $*" >> "$TMP/log"; }
colcon() {
  echo "colcon $* GIT_ALLOW_PROTOCOL=${GIT_ALLOW_PROTOCOL:-}" >> "$TMP/log"
  case "${FAKE_COLCON:-ok}" in
    subbuild) d=build/f1tenth_mpc_core/debug/build/osqp-sys-x/out/build/_deps
              mkdir -p "$d/qdldl-subbuild" ;;
    patch_unused) printf '\n[[patch.unused]]\nname = "x_msgs"\nversion = "1.0"\n' \
                    >> src/rust/Cargo.lock ;;
    newcrate) sed -i 's/0.8.0/0.8.1/' src/rust/Cargo.lock ;;
  esac
  echo "Summary: all built"
}
"""

LOCK = """version = 4

[[package]]
name = "rclrs"
version = "0.8.0"
source = "registry+https://github.com/rust-lang/crates.io-index"
checksum = "aaaa"
"""


def phase_b(tmp_path, rust='build', build_env=True, toolchain_name='osqp_offline.cmake',
            colcon='ok', qdldl=True):
    ws = tmp_path / 'ws'
    (ws / 'src' / 'rust').mkdir(parents=True)
    (ws / 'src' / 'rust' / 'Cargo.lock').write_text(LOCK)
    (ws / 'rust_packages.txt').write_text('f1tenth_mpc_core f1tenth_hard_stop_rs \n')
    (ws / 'colcon_build.log').write_text('')
    if qdldl:
        (ws / 'src' / 'rust' / 'third_party' / 'qdldl').mkdir(parents=True)
    tc = ws / 'src' / 'rust' / 'third_party' / toolchain_name
    tc.parent.mkdir(parents=True, exist_ok=True)
    tc.write_text('# toolchain\n')
    if build_env:
        (ws / 'src' / 'rust' / 'build_env.sh').write_text(
            f'export CMAKE_TOOLCHAIN_FILE_aarch64_unknown_linux_gnu="{tc}"\n')
    with open(os.path.join(ORIN, 'cargo_lock_check.py')) as f:
        (ws / 'cargo_lock_check.py').write_text(f.read())
    body = run_body('== phase B')
    body = body.replace('source /opt/ros/jazzy/setup.bash', 'true')
    body = body.replace('source /opt/f1tenth_ws/install/setup.bash', 'true')
    body = body.replace('/tmp/Cargo.lock.staged', str(tmp_path / 'Cargo.lock.staged'))
    body = body.replace('/opt/f1tenth_ws/', str(ws) + '/')
    env = {'TMP': str(tmp_path), 'RUST': rust, 'FAKE_COLCON': colcon}
    r = bash(PHASE_B_MOCKS + body, env, cwd=str(ws))
    return r, read(tmp_path / 'log'), ws


def test_phase_b_builds_only_the_rust_packages_offline_and_restores_the_lock(tmp_path):
    r, log, ws = phase_b(tmp_path, colcon='patch_unused')
    assert r.returncode == 0, r.stderr
    assert ('colcon build --merge-install --packages-select f1tenth_mpc_core '
            'f1tenth_hard_stop_rs') in log
    assert 'GIT_ALLOW_PROTOCOL=file' in log
    assert 'cargo_lock_check: note: [[patch.unused]] differs' in r.stdout
    assert (ws / 'src' / 'rust' / 'Cargo.lock').read_text() == LOCK
    assert ('offline QDLDL: CMAKE_TOOLCHAIN_FILE_aarch64_unknown_linux_gnu='
            in (ws / 'colcon_build.log').read_text())


def test_phase_b_is_skipped_for_rust_skip(tmp_path):
    r, log, _ = phase_b(tmp_path, rust='skip')
    assert r.returncode == 0 and 'no phase B' in r.stdout and 'colcon' not in log


@pytest.mark.parametrize('kw,message', [
    ({'build_env': False}, 'src/rust/build_env.sh missing'),
    ({'qdldl': False}, 'src/rust/third_party/qdldl missing'),
    ({'toolchain_name': 'other.cmake'},
     'did not set CMAKE_TOOLCHAIN_FILE_aarch64_unknown_linux_gnu'),
    ({'colcon': 'subbuild'}, 'QDLDL was fetched'),
    ({'colcon': 'newcrate'}, 'cargo_lock_check: FAILED'),
])
def test_phase_b_refuses(tmp_path, kw, message):
    r, log, _ = phase_b(tmp_path, **kw)
    assert r.returncode != 0
    assert message in r.stderr, r.stderr


# ---- consistency ----------------------------------------------------------------------------

def test_every_copy_from_the_context_is_in_the_dockerignore_allowlist():
    with open(os.path.join(ORIN, '.dockerignore')) as f:
        allowed = {line.strip()[1:] for line in f if line.startswith('!')}
    sources = []
    for ins in instructions():
        if ins.startswith('COPY') and '--from=' not in ins:
            sources += ins.split()[1:-1]
    assert sources
    for src in sources:
        assert src.rstrip('/') in allowed, src


def test_every_copy_from_the_src_context_is_staged():
    with open(os.path.join(ORIN, 'stage_sources.sh')) as f:
        stage = f.read()
    for ins in instructions():
        if ins.startswith('COPY --from=src'):
            for src in ins.split()[2:-1]:
                if src in ('src', 'SOURCES', 'CAR_SYNC.json'):
                    assert f'"$OUT/{src}"' in stage or f'{src} ' in stage, src
                else:
                    assert src in stage, src


def test_the_healthcheck_and_the_main_are_the_contract_paths():
    with open(DOCKERFILE) as f:
        text = f.read()
    assert 'CMD ["/opt/f1tenth_ws/docker/orin/container/healthcheck.sh"]' in text
    assert 'ARG RUST=build\n' in text
    with open(os.path.join(ORIN, 'compose.yaml')) as f:
        compose = f.read()
    assert '/opt/f1tenth_ws/docker/orin/container/f1tenth_container_main.py' in compose


def test_the_gateway_venv_installs_exactly_the_lock():
    # --ignore-installed: with --system-site-packages pip would otherwise skip
    # a pin the system site has at the same version, and build_checks.sh's
    # venv = lock equality would fail late in the build.
    body = run_body('/opt/f1tenth_gateway/venv/bin/pip install')
    install = re.search(r'/opt/f1tenth_gateway/venv/bin/pip install ([^&]*)', body).group(1)
    for flag in ('--no-deps', '--ignore-installed', '--only-binary=:all:',
                 '-r /opt/f1tenth_gateway/requirements.txt'):
        assert flag in install, flag
    assert '/opt/f1tenth_gateway/venv/bin/pip check' in body
