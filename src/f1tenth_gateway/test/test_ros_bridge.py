"""
RosBridge with real rclpy: the topics, services and QoS the gateway uses on the car.

Runs where rclpy, ackermann_msgs and f1tenth_messages are importable (linus
with the workspace sourced, or the Humble image f1tenth_more:cpu), on an
isolated domain, with in-process stand-ins for the mux, the mission services
and a few stack topics. Skipped elsewhere.
"""

import os
import threading
import time

import pytest


@pytest.fixture
def ros_env():
    rclpy = pytest.importorskip('rclpy')
    pytest.importorskip('ackermann_msgs.msg')
    pytest.importorskip('f1tenth_messages.msg')
    pytest.importorskip('f1tenth_ctl.rosprobe')
    os.environ.setdefault('ROS_DOMAIN_ID', '153')
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    os.environ.pop('ROS_DISCOVERY_SERVER', None)
    from f1tenth_gateway import config
    from f1tenth_gateway.ros_bridge import RosBridge
    from f1tenth_gateway.teleop import Teleop
    cfg = config.load(None)
    teleop = Teleop(cfg['teleop_cfg'])
    seen = {'status': [], 'outcome': []}
    spec = {'topic': '/safety_stop', 'service': '/mission/emergency_stop',
            'service_timeout_s': 0.5, 'abort_service': '/mission/abort_mission',
            'abort_timeout_s': 1.0, 'publish_s': 0.4, 'publish_hz': 50.0,
            'match_grace_s': 1.0}
    bridge = RosBridge(cfg, teleop, spec,
                       lambda *a: seen['status'].append(a),
                       lambda *a: seen['outcome'].append(a))
    bridge.start()
    from rclpy.executors import SingleThreadedExecutor
    fake = rclpy.create_node('fake_stack')
    ex = SingleThreadedExecutor()
    ex.add_node(fake)
    stop = threading.Event()

    def spin():
        while not stop.is_set():
            ex.spin_once(timeout_sec=0.01)
    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    yield bridge, teleop, fake, seen
    stop.set()
    thread.join(timeout=1.0)
    fake.destroy_node()
    bridge.stop()


def wait(cond, timeout=6.0):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.02)
    return False


def test_teleop_goes_out_on_the_teleop_lane_from_the_bridge_s_own_loop(ros_env):
    from ackermann_msgs.msg import AckermannDriveStamped
    bridge, teleop, fake, _ = ros_env
    got = []
    fake.create_subscription(AckermannDriveStamped, '/teleop', lambda m: got.append(m), 10)
    assert wait(lambda: bridge.teleop_graph()['subscribers'] >= 1)
    assert bridge.teleop_graph()['other_publishers'] == 0
    teleop.acquire('a', time.monotonic())
    for seq in range(1, 16):
        teleop.on_input('a', {'seq': seq, 'thr': 1.0, 'steer': 0.3, 'deadman': True},
                        time.monotonic())
        time.sleep(0.02)
    assert wait(lambda: len(got) >= 5)
    assert got[-1].header.frame_id == 'base_link/pwa_teleop'
    assert 0.4 - 1e-6 <= got[-1].drive.speed <= 0.6 + 1e-6   # float32 on the wire
    # Silence: the watchdog in the bridge's timer zeroes it, nothing from the web side.
    assert wait(lambda: got and got[-1].drive.speed == 0.0, timeout=1.0)
    joystick = fake.create_publisher(AckermannDriveStamped, '/teleop', 10)
    assert wait(lambda: bridge.teleop_graph()['other_publishers'] == 1)
    fake.destroy_publisher(joystick)


def test_the_estop_uses_the_publisher_and_clients_made_at_start(ros_env):
    from ackermann_msgs.msg import AckermannDriveStamped
    from std_srvs.srv import Trigger
    bridge, _teleop, fake, _ = ros_env
    zeros, calls = [], []
    fake.create_subscription(AckermannDriveStamped, '/safety_stop',
                             lambda m: zeros.append(m.drive.speed), 10)

    def answer(name, message):
        def cb(_req, resp):
            calls.append(name)
            resp.success, resp.message = True, message
            return resp
        return cb
    fake.create_service(Trigger, '/mission/emergency_stop', answer('estop', 'latched'))
    fake.create_service(Trigger, '/mission/abort_mission', answer('abort', 'aborted'))
    assert wait(lambda: bridge._safety_pub.get_subscription_count() >= 1
                and bridge._estop_cli.service_is_ready() and bridge._abort_cli.service_is_ready())
    res = bridge.estop()
    assert res['car_held'] and zeros and set(zeros) == {0.0}
    assert sorted(calls) == ['abort', 'estop']
    assert res['steps']['zero_drive']['matched_after_ms'] <= 50, 'matched before the press'
    calls.clear()
    assert bridge.abort()['ok'] and calls == ['abort']


def test_mission_status_battery_and_odometry_reach_the_gateway(ros_env):
    from f1tenth_messages.msg import BatteryStatus, MissionStatus, MoveOutcome
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    bridge, _teleop, fake, seen = ros_env
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    status = fake.create_publisher(MissionStatus, '/mission/status', latched)
    msg = MissionStatus()
    msg.state, msg.json_path, msg.emergency_stop_active = 'RUNNING', '/x.json', False
    status.publish(msg)
    assert wait(lambda: any(s[0] == 'RUNNING' for s in seen['status']))
    outcome = fake.create_publisher(MoveOutcome, '/mission/move_outcome', 10)
    battery = fake.create_publisher(BatteryStatus, '/diagnostics/battery_status', 10)
    odom = fake.create_publisher(Odometry, '/odom', 10)
    bridge.set_viewers(1)
    o = MoveOutcome()
    o.mission_id, o.move_id = 'llm_1', 'straight_1'
    b = BatteryStatus()
    b.has_data, b.voltage, b.min_voltage, b.ok = True, 11.6, 10.8, True
    od = Odometry()
    od.twist.twist.linear.x = 0.42

    def publish_all():
        outcome.publish(o)
        battery.publish(b)
        odom.publish(od)
        return ('llm_1', 'straight_1') in seen['outcome'] and \
            bridge.latest().get('battery', {}).get('v') == 11.6 and \
            bridge.latest().get('v_meas', {}).get('v') == 0.42
    assert wait(publish_all, timeout=8.0)


def test_link_status_reaches_the_status_payload(ros_env):
    """f1tenth_comms' /comms/link_status, report only: NaN becomes None."""
    lm = pytest.importorskip('f1tenth_messages.msg')
    if not hasattr(lm, 'LinkStatus'):
        pytest.skip('f1tenth_messages without LinkStatus')
    bridge, _teleop, fake, _seen = ros_env
    pub = fake.create_publisher(lm.LinkStatus, '/comms/link_status', 10)
    bridge.set_viewers(1)
    m = lm.LinkStatus()
    m.state, m.state_name, m.path, m.address = 2, 'LOST', '', ''
    m.rtt_ms, m.loss_pct, m.rssi_dbm = float('nan'), 100.0, -71.4
    m.since_last_reply_s, m.reconnect_attempts = 3.25, 2
    m.last_reconnect_result = "skipped: on eduroam, 'f1tenth-base' not in scan"

    def publish():
        pub.publish(m)
        return bridge.latest().get('link') is not None
    assert wait(publish, timeout=8.0)
    assert bridge.latest()['link'] == {
        'state': 'LOST', 'path': None, 'address': None, 'rtt_ms': None, 'loss_pct': 100.0,
        'rssi_dbm': -71.0, 'since_reply_s': 3.2, 'reconnect_attempts': 2,
        'reconnect_result': "skipped: on eduroam, 'f1tenth-base' not in scan"}
