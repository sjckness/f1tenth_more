"""wifi_coverage_node on a private graph: TF, a fake proc file and link_status in, grids out."""

import json
import os
import time

import pytest
import yaml

PKG = os.path.join(os.path.dirname(__file__), '..')
HEADER = ('Inter-| sta-|   Quality        |   Discarded packets               | Missed | WE\n'
          ' face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22\n')


def _yaml_params(node_name):
    with open(os.path.join(PKG, 'config', 'comms.yaml')) as f:
        return yaml.safe_load(f)[node_name]['ros__parameters']


def test_every_yaml_key_is_a_declared_parameter_of_the_right_type(ros_context):
    from rclpy.parameter import Parameter
    from f1tenth_comms.coverage_node import WifiCoverageNode
    params = _yaml_params('wifi_coverage_node')
    # A float written as an int in the yaml would raise here, as Humble does on the car.
    node = WifiCoverageNode(context=ros_context, parameter_overrides=[
        Parameter(k, value=v) for k, v in params.items()])
    try:
        for k, v in params.items():
            assert node.has_parameter(k), k
            assert node.get_parameter(k).value == v, k
    finally:
        node.destroy_node()


def _spin_until(executor, pred, timeout=8.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        executor.spin_once(timeout_sec=0.05)
        if pred():
            return True
    return False


def test_grids_from_tf_rssi_and_link_status_then_reset(ros_context, tmp_path):
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import StaticTransformBroadcaster

    from f1tenth_comms.coverage import NO_LINK, UNKNOWN
    from f1tenth_comms.coverage_node import LATCHED, WifiCoverageNode
    from f1tenth_messages.msg import LinkStatus

    proc = tmp_path / 'wireless'
    proc.write_text(HEADER + 'fakewlan0: 0000   50.  -55.  -256   0  0  0  0  0  0\n')
    prefix = '/test_comms'
    node = WifiCoverageNode(context=ros_context, parameter_overrides=[
        Parameter('wifi_interface', value='fakewlan0'),
        Parameter('proc_net_wireless', value=str(proc)),
        Parameter('use_iw_fallback', value=False),
        Parameter('sample_period_s', value=0.2),
        Parameter('publish_period_s', value=1.0),
        Parameter('link_status_topic', value=prefix + '/link_status'),
        Parameter('rssi_topic', value=prefix + '/coverage_rssi'),
        Parameter('rtt_topic', value=prefix + '/coverage_rtt'),
        Parameter('cells_topic', value=prefix + '/coverage_cells'),
        Parameter('reset_service', value=prefix + '/coverage_reset'),
    ])
    peer = Node('coverage_test_peer', context=ros_context)
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(node)
    executor.add_node(peer)
    got = {}
    peer.create_subscription(OccupancyGrid, prefix + '/coverage_rssi',
                             lambda m: got.__setitem__('rssi', m), LATCHED)
    peer.create_subscription(OccupancyGrid, prefix + '/coverage_rtt',
                             lambda m: got.__setitem__('rtt', m), LATCHED)
    peer.create_subscription(String, prefix + '/coverage_cells',
                             lambda m: got.__setitem__('cells', json.loads(m.data)), LATCHED)
    link_pub = peer.create_publisher(LinkStatus, prefix + '/link_status', 10)
    tf = TransformStamped()
    tf.header.frame_id = 'map'
    tf.child_frame_id = 'base_link'
    tf.transform.translation.x = 2.5
    tf.transform.translation.y = -0.5
    tf.transform.rotation.w = 1.0
    broadcaster = StaticTransformBroadcaster(peer)
    broadcaster.sendTransform(tf)

    def link(state, rtt, answered, lost):
        m = LinkStatus(state=state, rtt_ms=rtt, rounds_answered=answered, rounds_lost=lost)
        link_pub.publish(m)

    try:
        n = [0]

        def feed():
            n[0] += 1
            link(LinkStatus.STATE_OK, 20.0, 10 + n[0], 0)
            return 'rtt' in got and got['rtt'].data[0] != UNKNOWN

        assert _spin_until(executor, feed), 'no RTT grid with data'
        rssi, rtt = got['rssi'], got['rtt']
        assert rssi.header.frame_id == 'map'
        assert (rssi.info.width, rssi.info.height) == (1, 1)
        assert (rssi.info.origin.position.x, rssi.info.origin.position.y) == (2.0, -1.0)
        assert rssi.data[0] == 30          # -55 dBm: 100 * (-40 - -55) / 50
        assert rtt.data[0] == 10           # 20 ms: 100 * 20 / 200

        assert _spin_until(executor, lambda: 'cells' in got)
        cell = got['cells']['cells'][0]
        assert (cell['ix'], cell['iy']) == (2, -1)
        assert cell['rssi_mean_dbm'] == -55.0 and cell['no_link'] == 0

        # Disassociated: the proc line goes. The cell turns NO_LINK once most samples are.
        proc.write_text(HEADER)
        assert _spin_until(executor, lambda: feed() is not None and
                           got['rssi'].data[0] == NO_LINK, timeout=15.0)

        cli = peer.create_client(Trigger, prefix + '/coverage_reset')
        assert cli.wait_for_service(timeout_sec=5.0)
        fut = cli.call_async(Trigger.Request())
        assert _spin_until(executor, fut.done)
        assert fut.result().success
        got.pop('rssi')
        assert _spin_until(executor, lambda: 'rssi' in got)
        # The reset grid can be overtaken by a new sample: either is a fresh grid.
        assert got['rssi'].info.width == 1 and got['rssi'].data[0] in (UNKNOWN, NO_LINK)
    finally:
        executor.shutdown()
        peer.destroy_node()
        node.destroy_node()


def test_launch_file_describes_the_node():
    pytest.importorskip('launch_ros')
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'comms_launch', os.path.join(PKG, 'launch', 'comms.launch.py'))
    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)
        ld = mod.generate_launch_description()
    except Exception as exc:    # noqa: BLE001 - not installed: get_package_share_directory
        pytest.skip(f'f1tenth_comms not installed: {exc}')
    from launch_ros.actions import Node
    nodes = [e for e in ld.entities if isinstance(e, Node)]
    assert 'wifi_coverage_node' in [n.node_executable for n in nodes]
