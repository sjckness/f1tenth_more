"""
rosprobe.estop_with: the e-stop's three steps on a node that already exists.

The PWA gateway keeps one rclpy node up and runs f1tenth_ctl's e-stop on it,
with the /safety_stop publisher and the two service clients made at start-up
(already matched, so the zeros reach the mux at once). These tests run it
against in-process stand-ins for the mux and the mission services, on an
isolated domain.
"""

import os
import threading
import time

import pytest


@pytest.fixture
def ros():
    rclpy_ = pytest.importorskip('rclpy')
    pytest.importorskip('ackermann_msgs.msg')
    pytest.importorskip('std_srvs.srv')
    os.environ.setdefault('ROS_DOMAIN_ID', '151')
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    rclpy_.init()
    from rclpy.executors import MultiThreadedExecutor
    node = rclpy_.create_node('estop_with_test')
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    thread = threading.Thread(target=ex.spin, daemon=True)
    thread.start()
    yield node
    ex.shutdown(timeout_sec=1.0)
    node.destroy_node()
    rclpy_.shutdown()


def _spec(**kw):
    spec = {'topic': '/estop_with_test/safety_stop', 'publish_hz': 50.0, 'publish_s': 0.3,
            'match_grace_s': 1.0, 'service': '/estop_with_test/emergency_stop',
            'service_timeout_s': 0.5, 'abort_service': '/estop_with_test/abort_mission',
            'abort_timeout_s': 0.5}
    spec.update(kw)
    return spec


def test_premade_publisher_and_clients_are_used_and_hold_the_car(ros):
    from ackermann_msgs.msg import AckermannDriveStamped
    from std_srvs.srv import Trigger
    from f1tenth_ctl import rosprobe

    spec = _spec()
    got = []
    ros.create_subscription(AckermannDriveStamped, spec['topic'],
                            lambda m: got.append(m.drive.speed), 10)
    calls = []

    def answer(name, ok, message):
        def cb(_req, resp):
            calls.append(name)
            resp.success, resp.message = ok, message
            return resp
        return cb
    ros.create_service(Trigger, spec['service'], answer('estop', True, 'latched'))
    ros.create_service(Trigger, spec['abort_service'],
                       answer('abort', False, 'no mission to abort (state=IDLE)'))
    pub = ros.create_publisher(AckermannDriveStamped, spec['topic'], 10)
    clients = {'emergency_stop': ros.create_client(Trigger, spec['service']),
               'abort_mission': ros.create_client(Trigger, spec['abort_service'])}
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and not (
            pub.get_subscription_count() and all(c.service_is_ready()
                                                 for c in clients.values())):
        time.sleep(0.01)

    out = rosprobe.estop_with(ros, spec, spin=lambda: time.sleep(0.002), zero_pub=pub,
                              clients=clients)

    assert out['car_held'] is True
    # Matched before the call: the first zero already counts.
    assert out['steps']['zero_drive']['ok']
    assert out['steps']['zero_drive']['matched_after_ms'] <= 50
    assert out['steps']['emergency_stop']['ok']
    assert out['steps']['abort_mission']['ok'], '"no mission to abort" is a success'
    assert sorted(calls) == ['abort', 'estop']
    assert got and set(got) == {0.0}
    # No second publisher was made on the safety topic: the premade one did it.
    assert ros.count_publishers(spec['topic']) == 1


def test_without_services_the_zeros_still_go_out_and_the_car_is_not_held(ros):
    from ackermann_msgs.msg import AckermannDriveStamped
    from f1tenth_ctl import rosprobe

    spec = _spec(topic='/estop_with_test/lonely', service='/estop_with_test/none',
                 abort_service='/estop_with_test/none2', service_timeout_s=0.1,
                 abort_timeout_s=0.1)
    got = []
    ros.create_subscription(AckermannDriveStamped, spec['topic'],
                            lambda m: got.append(m.drive.speed), 10)
    out = rosprobe.estop_with(ros, spec, spin=lambda: time.sleep(0.002))
    assert out['steps']['zero_drive']['ok'] and got
    assert not out['steps']['emergency_stop']['ok']
    assert out['car_held'] is False
