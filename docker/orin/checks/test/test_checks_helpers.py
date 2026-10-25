"""
Unit tests for docker/orin/checks.

The pure helpers, and the host scripts run against fake `docker`,
`docker_orin.sh`, `stack_mode.sh`, guard and ctl.

Run: python3 -m pytest docker/orin/checks/test scripts/jazzy_parity/test
"""
import base64
import json
import os
import socket
import stat
import struct
import subprocess
import sys
import threading

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
CHECKS = os.path.dirname(HERE)
sys.path.insert(0, CHECKS)

import ctl_contract_compare  # noqa: E402, I100
import docker_diff_check  # noqa: E402
import env_compare  # noqa: E402
import gateway_check  # noqa: E402
import yolo_compare  # noqa: E402


# ---- docker_diff_check ------------------------------------------------------

def test_docker_diff_allow(tmp_path):
    entries = docker_diff_check.parse_diff(
        'C /tmp\nA /tmp/launch_params_x\nC /etc\nA /etc/localtime\n'
        'C /usr\nC /usr/lib\nA /usr/lib/__pycache__\nA /usr/lib/__pycache__/x.pyc\n'
        # a bind mount's content in the diff: the mount is missing
        'C /opt\nC /opt/f1tenth_ws\nC /opt/f1tenth_ws/src\n'
        'C /opt/f1tenth_ws/src/f1tenth_control\n'
        'C /opt/f1tenth_ws/src/f1tenth_control/corridors_jsons\n'
        'A /opt/f1tenth_ws/src/f1tenth_control/corridors_jsons/c.csv\n'
        'C /home\nC /home/fabiocar\nC /home/fabiocar/.config\n'
        'A /home/fabiocar/.config/Ultralytics\n'
        'A /home/fabiocar/.config/Ultralytics/settings.json\nbogus line\n')
    globs = docker_diff_check.read_allow(os.path.join(CHECKS, 'docker_diff_allow.txt'))
    ok, bad = docker_diff_check.check(entries, globs)
    assert ok == ['C /tmp', 'A /tmp/launch_params_x', 'C /etc', 'A /etc/localtime',
                  'C /usr', 'C /usr/lib', 'A /usr/lib/__pycache__',
                  'A /usr/lib/__pycache__/x.pyc']
    assert 'A /opt/f1tenth_ws/src/f1tenth_control/corridors_jsons/c.csv' in bad
    assert 'A /home/fabiocar/.config/Ultralytics/settings.json' in bad
    # a changed parent passes only with an allowed entry beneath it
    assert 'C /opt' in bad and 'C /home/fabiocar/.config' in bad
    assert len(ok) + len(bad) == len(entries)


# ---- env_compare ------------------------------------------------------------

def test_env_compare_expectations():
    native = {'ROS_DISCOVERY_SERVER': '127.0.0.1:11811', 'ROS_SUPER_CLIENT': 'TRUE',
              'PATH': '/a', 'HOME': '/home/f'}
    container = {'ROS_DISCOVERY_SERVER': '127.0.0.1:11811',
                 'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
                 'FASTRTPS_DEFAULT_PROFILES_FILE': '/opt/x/fastdds_profile.xml',
                 'F1T_PROCESS_BACKEND': 'proc', 'PATH': '/b', 'HOME': '/home/f'}
    rows, dev = env_compare.compare(native, container)
    assert dev == []
    names = [r[0] for r in rows]
    assert 'PATH' in names and 'HOME' not in names
    container['ROS_SUPER_CLIENT'] = 'TRUE'
    assert env_compare.compare(native, container)[1] == ['ROS_SUPER_CLIENT']


# ---- ctl_contract_compare ---------------------------------------------------

def write_step(d, n, name, rc, data):
    d.mkdir(exist_ok=True)
    (d / ('%02d_%s.json' % (n, name))).write_text(json.dumps(data) if data is not None else 'oops')
    (d / ('%02d_%s.rc' % (n, name))).write_text('%d\n' % rc)


def status(state, unit, pid=None):
    st = {'state': state, 'unit': unit, 'managed': True}
    if pid:
        st['pid'] = pid
    return {'ok': True, 'ready': False, 'estop_latched': False,
            'processes': {'stack': st, 'discovery': {'state': 'running'}},
            'probe': {'ran': state == 'running', 'age_s': 1.0}}


def test_ctl_contract_compare(tmp_path):
    a, b = tmp_path / 'a', tmp_path / 'b'
    write_step(a, 1, 'status', 0, status('stopped', 'f1tenth-stack.service'))
    write_step(b, 1, 'status', 0, status('stopped', 'proc:stack'))
    write_step(a, 2, 'status_wait', 0, status('running', 'x', 5))
    write_step(a, 3, 'status_up', 0, status('running', 'x', 5))
    write_step(b, 2, 'status_up', 0, status('running', 'y', 9))
    write_step(a, 4, 'send_planner_down', 1, {'ok': False, 'error': 'planner not running'})
    write_step(b, 3, 'send_planner_down', 1, {'ok': False, 'error': 'nicht'})
    res = ctl_contract_compare.compare(str(a), str(b))
    assert res['ok'], res
    write_step(b, 4, 'status_killed', 0, status('crashed', 'y'))
    write_step(a, 5, 'status_killed', 0, status('running', 'x', 5))
    res = ctl_contract_compare.compare(str(a), str(b))
    assert not res['ok'] and res['steps']['status_killed']
    write_step(b, 5, 'logs', 2, None)
    assert ctl_contract_compare.compare(str(a), str(b))['only_container'] == ['logs']


# ---- yolo_compare -----------------------------------------------------------

def meta(n_det, conf=0.3):
    return {'images': 1, 'conf': conf, 'per_image': [{'image': 'f.jpg', 'n': n_det}],
            'python': '3', 'torch': 't', 'cuda': 'c', 'ultralytics': 'u', 'model': 'm',
            'detections': n_det, 'latency_ms_mean': 1, 'latency_ms_p95': 2}


def arrays(boxes, confs, classes):
    np = pytest.importorskip('numpy')
    return {'0_xyxy': np.array(boxes, dtype=float).reshape(-1, 4),
            '0_conf': np.array(confs, dtype=float), '0_cls': np.array(classes, dtype=int)}


def test_yolo_compare_pass_and_fail():
    box = [10, 10, 110, 60]
    a = arrays([box], [0.8], [0])
    b = arrays([[10, 10, 110.2, 60]], [0.8015], [0])
    res = yolo_compare.compare(meta(1), a, meta(1), b)
    assert res['result'] == 'PASS' and res['matched'] == 1 and res['min_box_iou'] > 0.99
    b = arrays([[10, 10, 110.2, 60]], [0.805], [0])
    assert yolo_compare.compare(meta(1), a, meta(1), b)['result'] == 'FAIL'


def test_yolo_compare_borderline_unmatched():
    a = arrays([[10, 10, 110, 60], [200, 200, 220, 230]], [0.8, 0.3015], [0, 1])
    b = arrays([[10, 10, 110, 60]], [0.8], [0])
    res = yolo_compare.compare(meta(2), a, meta(1), b)
    assert res['result'] == 'PASS' and len(res['borderline_unmatched']) == 1
    a = arrays([[10, 10, 110, 60], [200, 200, 220, 230]], [0.8, 0.45], [0, 1])
    assert yolo_compare.compare(meta(2), a, meta(1), b)['result'] == 'FAIL'


def test_iou():
    assert yolo_compare.iou([0, 0, 10, 10], [0, 0, 10, 10]) == 1.0
    assert yolo_compare.iou([0, 0, 10, 10], [10, 10, 20, 20]) == 0.0
    assert abs(yolo_compare.iou([0, 0, 10, 10], [5, 0, 15, 10]) - 1 / 3) < 1e-9


# ---- gateway_check: the WebSocket client against a local server -------------

def test_frame_round_trip():
    for n in (5, 300, 70000):
        payload = os.urandom(n)
        frame = gateway_check.build_frame(payload, opcode=2, mask_key=b'abcd')
        buf = [frame]

        def read(k):
            out, buf[0] = buf[0][:k], buf[0][k:]
            return out
        assert gateway_check.parse_frame(read) == (2, payload)


def ws_server(token, ready):
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('127.0.0.1', 0))
    srv.listen(2)
    ready.append(srv.getsockname()[1])

    def serve():
        for _ in range(2):
            conn, _ = srv.accept()
            head = b''
            while b'\r\n\r\n' not in head:
                head += conn.recv(1)
            conn.sendall(b'HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n'
                         b'Connection: Upgrade\r\nSec-WebSocket-Accept: x\r\n\r\n')

            def read(k, conn=conn):
                out = b''
                while len(out) < k:
                    out += conn.recv(k - len(out))
                return out
            _, payload = gateway_check.parse_frame(read)
            msg = json.loads(payload)
            if msg.get('token') == token:
                body = json.dumps({'type': 'hello', 'client_id': 1}).encode()
                conn.sendall(bytes([0x81, len(body)]) + body)
            else:
                conn.sendall(bytes([0x88, 2]) + struct.pack('>H', 4401))
            conn.close()
        srv.close()
    t = threading.Thread(target=serve, daemon=True)
    t.start()
    return t


def test_ws_client_hello_and_refusal():
    ready = []
    t = ws_server('secret', ready)
    url = 'http://127.0.0.1:%d' % ready[0]
    ws = gateway_check.WsClient(url, timeout=5)
    ws.send_json({'type': 'auth', 'token': 'secret'})
    kind, val = ws.recv()
    assert kind == 'text' and json.loads(val)['type'] == 'hello'
    ws.sock.close()
    ws = gateway_check.WsClient(url, timeout=5)
    ws.send_json({'type': 'auth', 'token': 'wrong'})
    assert ws.recv() == ('close', 4401)
    ws.sock.close()
    t.join(5)


def test_read_token(tmp_path):
    f = tmp_path / 'env'
    f.write_text('# x\nexport F1T_GATEWAY_TOKEN="abc123"\n')
    assert gateway_check.read_token(str(f)) == 'abc123'
    assert base64  # imported for the server handshake shape only


# ---- host scripts against fakes ---------------------------------------------

FAKE_DOCKER = r"""#!/bin/bash
# fake docker: container state in $FAKE_STATE (running|exited|absent). Like the
# real one, `exec` runs in the container's own environment (not the caller's):
# a clean env plus -e VAR=VAL and the container's F1T_CTL_ENV ($FAKE_CTL_ENV).
# The image's files: only the healthcheck at its contract path exists.
st=$(cat "$FAKE_STATE" 2>/dev/null || echo absent)
case "$1" in
  inspect)
    if [ "$2" != --type ] || [ "$3" != container ]; then
      echo "fake docker: inspect without --type container" >&2; exit 64
    fi
    [ "$st" = absent ] && exit 1
    case "$5" in
      *State.Status*) echo "$st" ;;
      *State.Health*) [ "$st" = running ] && echo healthy || echo none ;;
      *Config.Env*) echo "F1T_CONTAINER_ROLE=${FAKE_ROLE:-car}" ;;
      *.Image*) echo sha256:feedbeef ;;
    esac ;;
  ps) [ "$st" = running ] && echo abc123 ;;
  exec) shift; envs=(PATH="$PATH" HOME="$HOME")
        [ -n "${FAKE_CTL_ENV:-}" ] && envs+=(F1T_CTL_ENV="$FAKE_CTL_ENV")
        while [ "${1#-}" != "$1" ]; do
          case "$1" in -e) envs+=("$2"); shift 2 ;; -w|-u) shift 2 ;; *) shift ;; esac
        done
        shift
        case "$1" in
          /opt/f1tenth_ws/docker/orin/container/healthcheck.sh)
            echo "fake healthcheck: ok"; exit 0 ;;
          /opt/*) echo "fake docker exec: $1: no such file in the image"; exit 127 ;;
        esac
        exec env -i "${envs[@]}" "$@" ;;
  *) echo "fake docker: $*" ;;
esac
"""
FAKE_DOCKER_ORIN = r"""#!/bin/bash
case "$1" in up) echo running > "$FAKE_STATE" ;; down) echo exited > "$FAKE_STATE" ;; esac
echo "fake docker_orin $*"
"""
FAKE_STACK_MODE = r"""#!/bin/bash
case "$1" in container) echo running > "$FAKE_STATE" ;; native) echo exited > "$FAKE_STATE" ;; esac
echo "fake stack_mode $*"
"""
FAKE_GUARD = r"""#!/bin/bash
# refuses --for humble while the fake container runs
[ "$2" = humble ] && [ "$(cat "$FAKE_STATE")" = running ] && exit 3
exit 0
"""
FAKE_CTL = r"""#!/bin/bash
st=$(cat "$FAKE_CTL_STATE" 2>/dev/null || echo stopped)
case "$1" in
  start) echo running > "$FAKE_CTL_STATE"; echo '{"ok": true}' ;;
  stop) echo stopped > "$FAKE_CTL_STATE"; echo '{"ok": true}' ;;
  status) printf '{"ok": true, "ready": false, "estop_latched": false, '
          printf '"processes": {"stack": {"state": "%s", "pid": 4242}}}\n' "$st" ;;
  send) echo '{"ok": false, "error": "planner not running"}'; exit 1 ;;
  *) echo '{"ok": true}' ;;
esac
"""


def install(tmp_path, name, text):
    f = tmp_path / name
    f.write_text(text)
    f.chmod(f.stat().st_mode | stat.S_IEXEC)
    return str(f)


@pytest.fixture
def fakes(tmp_path):
    shm = tmp_path / 'shm'
    shm.mkdir()
    env = dict(os.environ)
    for k in ('F1T_HEALTHCHECK', 'F1T_CONTAINER_ENV', 'F1T_CTL_ENV', 'D3_RW_ROOT',
              'D3_PATH_MAP', 'D3_PASS', 'FEED', 'BAG', 'MPC_PARAMS', 'CPU_REMAP'):
        env.pop(k, None)
    env.update({
        'FAKE_STATE': str(tmp_path / 'state'),
        'DOCKER': install(tmp_path, 'docker', FAKE_DOCKER),
        'DOCKER_ORIN': install(tmp_path, 'docker_orin.sh', FAKE_DOCKER_ORIN),
        'STACK_MODE': install(tmp_path, 'stack_mode.sh', FAKE_STACK_MODE),
        'ORIN_STACK_GUARD': install(tmp_path, 'guard.sh', FAKE_GUARD),
        'SHM_DIR': str(shm),
        'STACK_UDP_PORTS': '9', 'STACK_TCP_PORTS': '9',  # discard: unbound here
        'PARITY_IGNORE': '.',  # other runs on this machine are not the fake's
    })
    (tmp_path / 'state').write_text('exited\n')
    return env, tmp_path


def run(script, args, env):
    return subprocess.run(['bash', os.path.join(CHECKS, script)] + args, env=env,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                          timeout=120)


def test_updown_cycles_with_fakes(fakes):
    env, tmp = fakes
    env['HOLD_UP_S'] = '0'
    r = run('updown_cycles.sh', [str(tmp / 'ud'), '2'], env)
    assert r.returncode == 0, r.stdout
    res = (tmp / 'ud' / 'results.txt').read_text()
    assert res.count('PASS cycle') >= 6 and 'FAIL' not in res


def test_updown_refuses_when_up(fakes):
    env, tmp = fakes
    (tmp / 'state').write_text('running\n')
    assert run('updown_cycles.sh', [str(tmp / 'ud2'), '1'], env).returncode == 2


def test_mode_switch_with_fakes(fakes):
    env, tmp = fakes
    r = run('mode_switch_cycles.sh', [str(tmp / 'ms'), '1'], env)
    assert r.returncode == 0, r.stdout
    assert 'PASS cycle1 guard --for humble' in (tmp / 'ms' / 'results.txt').read_text()


def test_host_leftovers_detects_running_container(fakes):
    env, tmp = fakes
    (tmp / 'state').write_text('running\n')
    r = run('host_leftovers.sh', [str(tmp / 'hl')], env)
    assert r.returncode == 1 and 'FAIL leftovers container' in r.stdout


def test_ctl_contract_with_fake_ctl(fakes):
    env, tmp = fakes
    env['FAKE_CTL_STATE'] = str(tmp / 'ctl_state')
    ctl = install(tmp, 'ctl', FAKE_CTL)
    r = run('ctl_contract.sh', [str(tmp / 'cc'), '--kill', 'echo kill {pid}', '--', ctl], env)
    assert r.returncode == 0, r.stdout
    kill = (tmp / 'cc' / 'kill.txt').read_text()
    assert 'kill 4242' in kill
    steps = (tmp / 'cc' / 'steps.tsv').read_text()
    assert 'send_planner_down\t1' in steps and 'stop_stack\t0' in steps
    r2 = run('ctl_contract.sh', [str(tmp / 'cc2'), '--', ctl], env)
    assert r2.returncode == 0
    res = ctl_contract_compare.compare(str(tmp / 'cc'), str(tmp / 'cc2'))
    # the second run had no kill test: status_killed exists on one side only
    assert res['only_native'] == ['status_killed']


def test_d3_side_native_refuses_while_container_runs(fakes):
    env, tmp = fakes
    (tmp / 'state').write_text('running\n')
    r = run('d3_side.sh', ['native', str(tmp / 'd3'), '--', 'true'], env)
    assert r.returncode == 2 and 'refusing' in r.stdout


def test_d3_side_container_needs_harness_role(fakes):
    env, tmp = fakes
    (tmp / 'state').write_text('running\n')
    r = run('d3_side.sh', ['container', str(tmp / 'd3c'), '--', 'true'], env)
    assert r.returncode == 2 and "not 'harness'" in r.stdout


def test_healthcheck_default_is_the_contract_path(fakes):
    env, tmp = fakes
    r = subprocess.run(['bash', '-c', '. "$1"; echo "$F1T_HEALTHCHECK"', '_',
                        os.path.join(CHECKS, 'lib_checks.sh')], env=env,
                       stdout=subprocess.PIPE, text=True, timeout=30)
    assert r.stdout.strip() == '/opt/f1tenth_ws/docker/orin/container/healthcheck.sh'
    # the fake image has nothing at the old (wrong) path: the check must fail
    env['HOLD_UP_S'] = '0'
    env['F1T_HEALTHCHECK'] = '/opt/f1tenth_ws/container/healthcheck.sh'
    r = run('updown_cycles.sh', [str(tmp / 'udw'), '1'], env)
    assert r.returncode == 1 and 'FAIL cycle 1 healthcheck' in r.stdout


def harness_env(fakes):
    env, tmp = fakes
    (tmp / 'state').write_text('running\n')
    env['FAKE_ROLE'] = 'harness'
    env['D3_RW_ROOT'] = str(tmp / 'out')
    (tmp / 'out' / 'inputs' / 'bag').mkdir(parents=True)
    return env, tmp


SHOW = ['bash', '-c',
        'echo "FEED=$FEED BAG=$BAG CTL_ENV=$CTL_ENV X=$X_FROM_ENV REMAP=$CPU_REMAP"']


def test_d3_side_container_refuses_paths_the_harness_cannot_see(fakes):
    env, tmp = harness_env(fakes)
    r = run('d3_side.sh', ['container', str(tmp / 'elsewhere'), '--'] + SHOW, env)
    assert r.returncode == 2 and 'refusing: OUT_DIR' in r.stdout
    assert not (tmp / 'elsewhere').exists()
    env.update({'FEED': 'bag', 'BAG': str(tmp / 'bag_outside')})
    r = run('d3_side.sh', ['container', str(tmp / 'out' / 'd3'), '--'] + SHOW, env)
    assert r.returncode == 2 and 'refusing: BAG=' in r.stdout


def test_d3_side_container_passes_knobs_and_picks_the_env(fakes):
    env, tmp = harness_env(fakes)
    ctl = tmp / 'ctl.env'
    ctl.write_text('X_FROM_ENV=ctl\n')
    bag = str(tmp / 'out' / 'inputs' / 'bag')
    env.update({'FAKE_CTL_ENV': str(ctl), 'FEED': 'bag', 'BAG': bag, 'CPU_REMAP': '1'})
    r = run('d3_side.sh', ['container', str(tmp / 'out' / 'a'), '--'] + SHOW, env)
    assert r.returncode == 0, r.stdout
    assert '[d3_side] env file: %s' % ctl in r.stdout
    # the knobs reach the container; the linus-only CPU_REMAP does not
    assert 'FEED=bag BAG=%s CTL_ENV=%s X=ctl REMAP=\n' % (bag, ctl) in r.stdout
    explicit = tmp / 'explicit.env'
    explicit.write_text('X_FROM_ENV=explicit\n')
    env['F1T_CONTAINER_ENV'] = str(explicit)
    r = run('d3_side.sh', ['container', str(tmp / 'out' / 'b'), '--'] + SHOW, env)
    assert r.returncode == 0 and 'X=explicit' in r.stdout, r.stdout
    assert 'FEED=bag' in (tmp / 'out' / 'b' / 'd3_side.txt').read_text()
