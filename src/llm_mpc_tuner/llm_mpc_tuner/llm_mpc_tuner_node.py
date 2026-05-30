"""LLM-driven MPC tuner with manual mode keys and an odometry feedback loop.

Modes (selected via single-key terminal input):
  'e' -> SLOW  (target velocity = 0.4 m/s)
  's' -> FAST  (target velocity = 2.0 m/s)
  'p' -> STOP  (publish zero AckermannDriveStamped on stop_topic, suspend loop)

In SLOW or FAST mode, a timer at `update_frequency` Hz reads the current
speed from the odometry topic, formats the prompt with the current speed
and target velocity, sends it to the LLM, parses the returned parameters
and applies them to the MPC node via `ros2 param set`.

NOTE on STOP: the spec says "publish velocity = 0 to the MPC input topic".
frenet_mpc_node has no command/input topic (only `/odom` in, `/drive`
out). The closest equivalent is publishing a zero AckermannDriveStamped on
the highest-priority mux input topic (`/teleop` by default, priority 100
vs `/drive`'s 10), which preempts the MPC's drive command at the mux.
"""

import ast
import re
import select
import subprocess
import sys
import termios
import threading
import tty

import requests

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry


MODE_IDLE = 'IDLE'
MODE_SLOW = 'SLOW'
MODE_FAST = 'FAST'


# Base prompt template — preserved verbatim from the original node, with
# only two minimal additions required by the new feedback loop:
#   * a {current_speed} placeholder added to "Current behavior"
#   * the literal target speed in the "Task" section replaced by
#     {target_velocity}, so SLOW/FAST variants differ only in this value
PROMPT_TEMPLATE = """
You are an AI assistant helping to tune the parameters of an MPC controller for an autonomous racing car.

Context:
The car is running a kinematic MPC in Frenet-like coordinates.
The trajectory is:
- straight
- smooth right 90-degree curve
- straight again

Current behavior:
- initial target speed is 0.45 m/s
- objective is smooth driving at 0.5 m/s
- current measured speed is {current_speed:.2f} m/s
- avoid aggressive steering oscillations
- avoid aggressive acceleration

Tuneable MPC parameters:
qv 0, 20, 10
qn 0, 100, 20
qalpha 0, 100, 7
qac 0, 1, 0.01
qddelta 0, 100, 0.1
alat_max 0, 20, 10
a_min -5, 0, -2
a_max 0, 5, 2
v_min 0, 0.3, 0
v_max 0.2, 1.0, 0.6
v_ref 0.2, 1.0, 0.5

Task:
Adapt the tunable parameters of the MPC so that the car achieves:
Track v ref at  {target_velocity} m/s accurately,
Drive smooth.

Constraints:
- do not invent new parameters
- output ONLY valid Python assignment
- avoid unsafe/aggressive parameters

Output format:

new_mpc_params = {{
'qv': value,
...
}}
"""

SLOW_TARGET_VELOCITY = 0.4
FAST_TARGET_VELOCITY = 2.0
STOP_PROMPT = None  # STOP does not use the LLM


class LLMMpcTuner(Node):

    def __init__(self):
        super().__init__('llm_mpc_tuner')

        self.declare_parameter('mpc_url', 'http://127.0.0.1:8082/completion')
        self.declare_parameter('target_node', '/frenet_mpc_controller')
        self.declare_parameter('update_frequency', 1.0)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('stop_topic', '/teleop')
        self.declare_parameter('llm_timeout_sec', 30.0)
        self.declare_parameter('slow_target_velocity', SLOW_TARGET_VELOCITY)
        self.declare_parameter('fast_target_velocity', FAST_TARGET_VELOCITY)

        self.mpc_url = str(self.get_parameter('mpc_url').value)
        self.target_node = str(self.get_parameter('target_node').value)
        self.update_frequency = float(self.get_parameter('update_frequency').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.stop_topic = str(self.get_parameter('stop_topic').value)
        self.llm_timeout = float(self.get_parameter('llm_timeout_sec').value)
        self.slow_target_velocity = float(
            self.get_parameter('slow_target_velocity').value
        )
        self.fast_target_velocity = float(
            self.get_parameter('fast_target_velocity').value
        )

        self.mode = MODE_IDLE
        self.current_speed = 0.0
        self._llm_busy = False
        self._mode_lock = threading.Lock()

        cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
            callback_group=cb_group,
        )

        self.stop_pub = self.create_publisher(
            AckermannDriveStamped,
            self.stop_topic,
            10,
        )

        period = 1.0 / max(self.update_frequency, 1e-3)
        self.timer = self.create_timer(
            period,
            self.feedback_tick,
            callback_group=cb_group,
        )

        self._stdin_fd = None
        self._stdin_old_settings = None
        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

        print(
            "[llm_mpc_tuner] Ready. Press 'e' for SLOW, 's' for FAST, 'p' to STOP.",
            flush=True,
        )

    # ------------------------------------------------------------------
    # Prompt building

    def build_prompt(self, target_velocity):
        return PROMPT_TEMPLATE.format(
            current_speed=self.current_speed,
            target_velocity=target_velocity,
        )

    def slow_prompt(self):
        return self.build_prompt(self.slow_target_velocity)

    def fast_prompt(self):
        return self.build_prompt(self.fast_target_velocity)

    # ------------------------------------------------------------------
    # ROS callbacks

    def odom_callback(self, msg):
        self.current_speed = float(msg.twist.twist.linear.x)

    def feedback_tick(self):
        with self._mode_lock:
            mode = self.mode

        if mode == MODE_IDLE:
            return

        if self._llm_busy:
            self.get_logger().debug(
                'Skipping tick: previous LLM call still running'
            )
            return

        if mode == MODE_SLOW:
            prompt = self.slow_prompt()
            label = 'SLOW'
        elif mode == MODE_FAST:
            prompt = self.fast_prompt()
            label = 'FAST'
        else:
            return

        self._llm_busy = True
        try:
            self.get_logger().info(
                f'[{label}] Calling LLM (current_speed={self.current_speed:.2f})'
            )
            raw = self.ask_llm(prompt)
            self.get_logger().debug(f'LLM raw output: {raw}')

            params = self.parse_params(raw)
            self.get_logger().info(f'Parsed params: {params}')

            self.set_params(params)
            self.get_logger().info('Update complete')
        except Exception as exc:
            self.get_logger().error(f'LLM update failed: {exc}')
        finally:
            self._llm_busy = False

    # ------------------------------------------------------------------
    # STOP action

    def do_stop(self):
        with self._mode_lock:
            self.mode = MODE_IDLE

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.stop_pub.publish(msg)

        self.get_logger().info(
            f'[STOP] Published zero command on {self.stop_topic}; '
            f'feedback loop suspended (MPC params unchanged)'
        )

    # ------------------------------------------------------------------
    # Keyboard listener

    def _keyboard_loop(self):
        try:
            self._stdin_fd = sys.stdin.fileno()
            self._stdin_old_settings = termios.tcgetattr(self._stdin_fd)
            tty.setcbreak(self._stdin_fd)
        except Exception as exc:
            # If stdin is not a TTY (no controlling terminal, redirected,
            # etc.) the keyboard listener cannot run. Log and return; the
            # node will still spin but mode stays IDLE.
            self.get_logger().warn(
                f'Keyboard listener disabled (stdin not a tty): {exc}. '
                f'Use `ros2 param set` to change mode externally.'
            )
            return

        try:
            while rclpy.ok():
                rlist, _, _ = select.select([sys.stdin], [], [], 0.2)
                if not rlist:
                    continue

                ch = sys.stdin.read(1)
                if not ch:
                    continue

                if ch == 'e':
                    with self._mode_lock:
                        self.mode = MODE_SLOW
                    self.get_logger().info(
                        f'[KEY e] Mode -> SLOW (target_velocity='
                        f'{self.slow_target_velocity} m/s)'
                    )
                elif ch == 's':
                    with self._mode_lock:
                        self.mode = MODE_FAST
                    self.get_logger().info(
                        f'[KEY s] Mode -> FAST (target_velocity='
                        f'{self.fast_target_velocity} m/s)'
                    )
                elif ch == 'p':
                    self.get_logger().info('[KEY p] STOP')
                    self.do_stop()
                elif ch in ('\x03', '\x04'):
                    # Ctrl-C / Ctrl-D — let main loop handle shutdown.
                    break
        finally:
            self._restore_stdin()

    def _restore_stdin(self):
        if self._stdin_fd is not None and self._stdin_old_settings is not None:
            try:
                termios.tcsetattr(
                    self._stdin_fd,
                    termios.TCSADRAIN,
                    self._stdin_old_settings,
                )
            except Exception:
                pass

    # ------------------------------------------------------------------
    # LLM + parameter machinery (unchanged from original)

    def clamp(self, name, value):
        ranges = {
            'qv': (0.0, 20.0),
            'qn': (0.0, 100.0),
            'qalpha': (0.0, 100.0),
            'qac': (0.0, 1.0),
            'qddelta': (0.0, 100.0),
            'alat_max': (0.0, 20.0),
            'a_min': (-5.0, 0.0),
            'a_max': (0.0, 5.0),
            'v_min': (0.0, 0.3),
            'v_max': (0.2, 1.0),
            'v_ref': (0.2, 1.0),
        }
        lo, hi = ranges[name]
        return max(lo, min(float(value), hi))

    def parse_params(self, text):
        match = re.search(
            r'new_mpc_params\s*=\s*(\{.*?\})',
            text,
            re.DOTALL,
        )
        if not match:
            raise RuntimeError('No new_mpc_params found in LLM output')

        params = ast.literal_eval(match.group(1))

        allowed = {
            'qv', 'qn', 'qalpha', 'qac', 'qddelta', 'alat_max',
            'a_min', 'a_max', 'v_min', 'v_max', 'v_ref',
        }
        clean = {}
        for k, v in params.items():
            if k in allowed:
                clean[k] = self.clamp(k, v)
        return clean

    def ask_llm(self, prompt):
        response = requests.post(
            self.mpc_url,
            json={
                'prompt': prompt,
                'n_predict': 256,
                'temperature': 0.1,
            },
            timeout=self.llm_timeout,
        )
        response.raise_for_status()
        data = response.json()
        return data.get('content', '')

    def set_params(self, params):
        for name, value in params.items():
            cmd = [
                'ros2', 'param', 'set',
                self.target_node, name, str(float(value)),
            ]
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode != 0:
                raise RuntimeError(
                    f'Failed setting {name}: {result.stderr}'
                )
            self.get_logger().info(f'Set {name} = {value}')

    def destroy_node(self):
        self._restore_stdin()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LLMMpcTuner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
