"""stack_info_node: which stack is running, on /viz/stack_info, for Foxglove.

Publishes one diagnostic_msgs/DiagnosticStatus (stack_info.py builds it): the
profile (car / sim-<host> / native), the image or the git tree, the launch
command and the stack_args in force, the ROS setup. Reliable + transient_local,
depth 1: a Foxglove that connects later still gets it. Published at startup and
every period_s (10 s); /viz/ already passes the bridge's whitelist.

Light on purpose: no subscription, no CPU pinning. What does not change while
the process lives (environment, image, git tree, the launch command once found)
is read once; stack_args is re-read on every publish. Launched by
diagnostics_server.launch.py when enable_stack_info (stack_params.yaml) is true.
"""

import datetime
import os
import socket
import subprocess

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticStatus
from f1tenth_params.stack_provenance import read_image_provenance, stack_launch_cmd
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from f1tenth_diagnostics import stack_info

TOPIC = '/viz/stack_info'
STACK_ARGS_PATH = os.path.expanduser('~/.config/f1tenth_car/stack_args')


def in_container():
    return os.path.exists('/.dockerenv') or bool(os.environ.get('F1TENTH_IMAGE_SOURCES'))


def read_pid1_env(path='/proc/1/environ'):
    """The container's PID 1 environment (compose.yaml's variables); {} if unreadable."""
    try:
        with open(path, 'rb') as fh:
            raw = fh.read().decode(errors='replace')
    except OSError:
        return {}
    return dict(item.partition('=')[::2] for item in raw.split('\0') if '=' in item)


def read_stack_args(path=STACK_ARGS_PATH):
    """The content of stack_args, None when the file is absent or unreadable."""
    try:
        with open(path) as fh:
            return fh.read()
    except OSError:
        return None


def git_root():
    """The checkout that holds this stack's stack_params.yaml, None without .git."""
    try:
        d = os.path.dirname(os.path.realpath(os.path.join(
            get_package_share_directory('f1tenth_params'), 'config', 'stack_params.yaml')))
    except Exception:  # noqa: BLE001 - not installed: no tree to read
        return None
    while True:
        if os.path.exists(os.path.join(d, '.git')):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            return None
        d = parent


def native_git_info(root):
    """commit, branch, describe, subject, dirty of a checkout (git), missing keys on failure."""
    def _git(*args):
        return subprocess.run(['git', '-C', root, *args], capture_output=True, text=True,
                              timeout=10).stdout.strip()
    info = {}
    try:
        info['commit'] = _git('rev-parse', 'HEAD') or None
        info['branch'] = _git('rev-parse', '--abbrev-ref', 'HEAD') or None
        info['describe'] = _git('describe', '--always', '--tags') or None
        info['subject'] = _git('log', '-1', '--format=%s') or None
        info['dirty'] = bool(_git('status', '--porcelain'))
    except (OSError, subprocess.SubprocessError):
        pass
    return info


class StackInfoNode(Node):

    def __init__(self):
        super().__init__('stack_info_node')
        period = float(self.declare_parameter('period_s', 10.0).value)
        self.hostname = socket.gethostname()
        self.container = in_container()
        self.pid1_env = read_pid1_env() if self.container else {}
        if self.container:
            self.git = read_image_provenance(os.environ.get('F1TENTH_IMAGE_SOURCES'),
                                             os.environ.get('F1TENTH_CAR_SYNC'))
        else:
            root = git_root()
            self.git = native_git_info(root) if root else {}
        self.launch_cmd = None
        self.start_utc = datetime.datetime.now(datetime.timezone.utc).isoformat(
            timespec='seconds')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(DiagnosticStatus, TOPIC, qos)
        self.publish()
        self.create_timer(period, self.publish)

    def publish(self):
        if self.launch_cmd is None:
            self.launch_cmd = stack_launch_cmd()
        info = stack_info.collect(
            dict(os.environ), self.pid1_env, self.hostname, self.container, self.launch_cmd,
            read_stack_args(), self.git, self.get_parameter('use_sim_time').value,
            self.start_utc)
        status = stack_info.to_status(info, self.hostname)
        self.pub.publish(status)
        self.get_logger().info(f'{TOPIC}: {status.message}', once=True)


def main(args=None):
    rclpy.init(args=args)
    node = StackInfoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
