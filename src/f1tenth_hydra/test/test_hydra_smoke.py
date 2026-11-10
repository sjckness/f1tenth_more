"""Hydra smoke test: Hydra builds (its packages are installed), runs with seg_source:=gt and
pose_source:=isaac_gt, and produces a non-empty scene graph.

The feed is f1tenth_hydra's synthetic ZED 2i + Isaac GT publisher (no Isaac needed), on an
isolated, localhost-only ROS domain. Skipped when ~/hydra_ws is not sourced (hydra_ros,
hydra_msgs or the spark_dsg bindings missing): source scripts/hydra/hydra_env.sh first.

    source scripts/hydra/hydra_env.sh
    python3 -m pytest src/f1tenth_hydra/test/test_hydra_smoke.py -v -s
"""

import os
import random
import signal
import subprocess
import time

import pytest

from f1tenth_hydra import hydra_args


def _hydra_available():
    try:
        from ament_index_python.packages import get_package_share_directory
        for pkg in ('hydra', 'hydra_ros', 'f1tenth_hydra'):
            get_package_share_directory(pkg)
        import hydra_msgs.msg  # noqa: F401
        import spark_dsg  # noqa: F401
        return True, ''
    except Exception as exc:  # noqa: BLE001
        return False, str(exc)


def test_hydra_command_line_for_every_profile():
    """The node's command line builds for each profile x seg x pose (no Hydra needed)."""
    profiles = [f[:-5] for f in os.listdir(os.path.dirname(hydra_args.profile_path('x')))
                if f.endswith('.yaml')]
    assert 'workstation_full' in profiles
    for name in profiles:
        prof = hydra_args.load_profile(name)
        assert os.path.isfile(os.path.join(os.path.dirname(os.path.dirname(prof['_path'])),
                                           prof['hydra_config']))
        for seg in hydra_args.SEG_SOURCES:
            for pose in hydra_args.POSE_SOURCES:
                out = hydra_args.build(prof, seg, pose, '/tmp/x', '/h', '/hr',
                                       label_remap_path='/tmp/r.yaml')
                remaps = dict(out['remappings'])
                semantic = remaps['/hydra/input/zed/semantic/image_raw']
                assert semantic == ('/sim/ground_truth/left/semantic' if seg == 'gt'
                                    else '/hydra_seg/semantic/image_raw')
                joined = ' '.join(out['arguments'])
                assert ('"type":"param"' in joined) == (pose == 'isaac_gt')
                assert ('"publish_backend_tf":false' in joined) == (pose == 'stack_tf')
                assert out['frames']['map_frame'] == 'hydra_map'


@pytest.mark.skipif(not _hydra_available()[0], reason='Hydra not sourced: ' + _hydra_available()[1])
def test_hydra_gt_builds_a_non_empty_graph(tmp_path):
    import rclpy
    from hydra_msgs.msg import DsgUpdate
    import spark_dsg

    from f1tenth_hydra.dsg_tools import graph_summary

    env = dict(os.environ, ROS_DOMAIN_ID=str(random.randint(150, 199)),
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    env.pop('ROS_DISCOVERY_SERVER', None)
    env.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    os.environ.pop('ROS_DISCOVERY_SERVER', None)
    log_dir = tmp_path / 'hydra'
    out = open(tmp_path / 'launch.log', 'w')
    procs = [
        subprocess.Popen(['ros2', 'launch', 'f1tenth_hydra', 'hydra.launch.py',
                          'seg_source:=gt', 'pose_source:=isaac_gt', 'use_sim_time:=false',
                          'visualizer:=false', f'log_dir:={log_dir}'],
                         env=env, stdout=out, stderr=subprocess.STDOUT, start_new_session=True),
        subprocess.Popen(['ros2', 'run', 'f1tenth_hydra', 'hydra_synthetic_rgbd'],
                         env=env, stdout=out, stderr=subprocess.STDOUT, start_new_session=True),
    ]
    rclpy.init()
    node = rclpy.create_node('hydra_smoke_check')
    latest = {}

    def on_dsg(msg):
        latest['summary'] = graph_summary(
            spark_dsg.DynamicSceneGraph.from_binary(msg.layer_contents.tobytes()))

    node.create_subscription(DsgUpdate, '/hydra/backend/dsg', on_dsg, 1)
    deadline, ok = time.time() + 150.0, False
    try:
        while time.time() < deadline and all(p.poll() is None for p in procs):
            rclpy.spin_once(node, timeout_sec=0.5)
            s = latest.get('summary')
            if s:
                built = sum(s['layers'].get(k, {}).get('nodes', 0)
                            for k in ('OBJECTS', 'PLACES', 'MESH_PLACES'))
                if built > 0 and s['layers'].get('PLACES', {}).get('nodes', 0) > 0:
                    ok = True
                    break
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for p in procs:
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        for p in procs:
            try:
                p.wait(timeout=40)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
        out.close()
    print('graph summary:', latest.get('summary'))
    assert ok, (f'no non-empty Hydra graph within 150 s; last summary {latest.get("summary")}; '
                f'log {tmp_path / "launch.log"}')
