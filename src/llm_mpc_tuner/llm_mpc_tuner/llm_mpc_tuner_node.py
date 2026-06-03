"""LLM-driven MPC tuner with manual mode keys and an odometry feedback loop.

Modes (selected via single-key terminal input):
  'e' -> SLOW  (target velocity = 0.4 m/s)
  's' -> FAST  (target velocity = 2.0 m/s)
  'p' -> STOP  (publish zero AckermannDriveStamped on stop_topic, suspend loop)
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

# MPC params exposed to the LLM for tuning, with their (min, max) bounds
PARAM_RANGES = {
    'qn':          (0.0,   200.0),
    'qalpha':      (0.0,   100.0),
    'qddelta':     (0.0,    10.0),
    'alat_max':    (1.0,    20.0),
    'a_min':      (-5.0,     0.0),
    'a_max':       (0.0,     5.0),
    'v_min':       (0.0,     0.5),
    'v_max':      (-3.0,    -0.1),   # negative: reverse driving
    'v_ref':      (-3.0,    -0.1),   # negative: reverse driving
    'sine_amp':    (0.0,     0.5),
    'sine_period': (1.0,    10.0),
}

PROMPT_TEMPLATE = """
You are an AI assistant helping to tune the parameters of an MPC controller for an autonomous racing car.

Context:
The car drives a fixed circle using a geometric feedforward steering + MPC correction.
- Feedforward handles the basic circle turn
- MPC optimises a small correction delta on top to minimise radial and heading error
- Speed is open-loop, following a sine profile between v_min and v_max

Current behavior:
- current measured speed : {current_speed:.2f} m/s
- target speed           : {target_velocity:.2f} m/s
- speed error            : {speed_error:.2f} m/s  (positive = car too slow, negative = car too fast)

Step-by-step rules you MUST follow:
1. Set v_ref  = {target_velocity:.2f}  (MUST be negative — car drives in reverse)
2. Set v_max  = {v_max_target:.2f}     (MUST be negative, 0.5 more negative than v_ref)
3. Set a_max  = 3.0                    (always allow strong acceleration)
4. Set a_min  = -3.0                   (always allow strong braking)
5. For qn, qalpha, qddelta: pick values that give smooth circle tracking
   - qn     between 20 and 80   (higher = tighter circle tracking)
   - qalpha between 10 and 50   (higher = better heading alignment)
   - qddelta between 1 and 5    (higher = smoother steering)
6. Do NOT output values at or near the minimum of their range

Tuneable MPC parameters (name: min, max, current):
qn            0,   200,  {qn}
qalpha        0,   100,  {qalpha}
qddelta       0,    10,  {qddelta}
alat_max      1,    20,  {alat_max}
a_min        -5,     0,  {a_min}
a_max         0,     5,  {a_max}
v_min         0,   0.5,  {v_min}
v_max      -3.0,  -0.1,  {v_max}
v_ref      -3.0,  -0.1,  {v_ref}
sine_amp      0,   0.5,  {sine_amp}
sine_period   1,    10,  {sine_period}

Task:
Tune the parameters so that the car:
- stays on the circle (minimise radial error)
- stays tangent to the circle (minimise heading error)
- drives smoothly without oscillation
- tracks the sine speed profile accurately at {target_velocity:.2f} m/s

Constraints:
- do not invent new parameters
- do not include qv, qac or any other unlisted parameter
- qn, qalpha, qddelta MUST be > 0 — setting them to zero disables the controller entirely
- a_max MUST be > 0.5 — zero means the car can never accelerate, it will stay still
- v_ref MUST be negative (e.g. -1.0) — positive means forward, which is wrong
- do NOT output all minimum values — that is a degenerate broken solution
- output ONLY valid Python assignment, no explanation, no markdown

Output format:
new_mpc_params = {{
    'qn': value,
    'qalpha': value,
    'qddelta': value,
    'alat_max': value,
    'a_min': value,
    'a_max': value,
    'v_min': value,
    'v_max': value,
    'v_ref': value,
    'sine_amp': value,
    'sine_period': value,
}}
"""

SLOW_TARGET_VELOCITY = -0.5
FAST_TARGET_VELOCITY = -1.0


class LLMMpcTuner(Node):

    def __init__(self):
        super().__init__('llm_mpc_tuner')

        self.declare_parameter('mpc_url',              'http://127.0.0.1:8082/completion')
        self.declare_parameter('target_node',          '/andre_mpc_controller')
        self.declare_parameter('update_frequency',     1.0)
        self.declare_parameter('odom_topic',           '/odom')
        self.declare_parameter('stop_topic',           '/teleop')
        self.declare_parameter('llm_timeout_sec',      30.0)
        self.declare_parameter('slow_target_velocity', SLOW_TARGET_VELOCITY)
        self.declare_parameter('fast_target_velocity', FAST_TARGET_VELOCITY)

        self.mpc_url              = str(self.get_parameter('mpc_url').value)
        self.target_node          = str(self.get_parameter('target_node').value)
        self.update_frequency     = float(self.get_parameter('update_frequency').value)
        self.odom_topic           = str(self.get_parameter('odom_topic').value)
        self.stop_topic           = str(self.get_parameter('stop_topic').value)
        self.llm_timeout          = float(self.get_parameter('llm_timeout_sec').value)
        self.slow_target_velocity = float(self.get_parameter('slow_target_velocity').value)
        self.fast_target_velocity = float(self.get_parameter('fast_target_velocity').value)

        self.mode          = MODE_IDLE
        self.current_speed = 0.0
        self._llm_busy     = False
        self._mode_lock    = threading.Lock()

        # Local cache of MPC param values — populated lazily via get_param()
        self._param_cache: dict[str, float] = {}

        cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10,
            callback_group=cb_group,
        )
        self.stop_pub = self.create_publisher(AckermannDriveStamped, self.stop_topic, 10)

        period = 1.0 / max(self.update_frequency, 1e-3)
        self.create_timer(period, self.feedback_tick, callback_group=cb_group)

        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()
        self._stdin_fd            = None
        self._stdin_old_settings  = None

        # Validate target node exists before anything else
        try:
            check = subprocess.run(
                ['ros2', 'node', 'info', self.target_node],
                capture_output=True, text=True, timeout=5.0,
            )
            if check.returncode != 0:
                self.get_logger().error(
                    f'TARGET NODE NOT FOUND: {self.target_node}\n'
                    f'Run: ros2 node list | grep mpc\n'
                    f'Then set: --ros-args -p target_node:=/your_node_name'
                )
            else:
                self.get_logger().info(f'Target node confirmed: {self.target_node}')
        except Exception as exc:
            self.get_logger().warn(f'Node check failed: {exc}')

        # Pre-populate param cache so build_prompt never shells out
        self._fetch_all_params()

        print("[llm_mpc_tuner] Ready. Press 'e' for SLOW, 's' for FAST, 'p' to STOP.",
              flush=True)

    # ── Parameter helpers ─────────────────────────────────────────────────

    def _fetch_all_params(self):
        """Populate cache from a single `ros2 param dump` call."""
        try:
            result = subprocess.run(
                ['ros2', 'param', 'dump', self.target_node],
                capture_output=True, text=True, timeout=10.0,
            )
            if result.returncode == 0:
                for name in PARAM_RANGES:
                    # yaml line format:  "    name: value"
                    match = re.search(rf'^\s*{name}:\s*([-\d.]+)',
                                      result.stdout, re.MULTILINE)
                    if match:
                        self._param_cache[name] = float(match.group(1))
                self.get_logger().info(f'Param cache: {self._param_cache}')
            else:
                self.get_logger().warn(
                    f'param dump failed: {result.stderr.strip()}\n'
                    f'Using range midpoints. Is {self.target_node} running?'
                )
        except Exception as exc:
            self.get_logger().warn(f'param dump error: {exc}')

    def get_param(self, name: str) -> float:
        """Return cached param value, or range midpoint if not yet fetched."""
        if name in self._param_cache:
            return self._param_cache[name]
        lo, hi = PARAM_RANGES[name]
        return (lo + hi) / 2.0

    def clamp(self, name: str, value: float) -> float:
        lo, hi = PARAM_RANGES[name]
        return max(lo, min(float(value), hi))

    def parse_params(self, text: str) -> dict:
        match = re.search(r'new_mpc_params\s*=\s*(\{.*?\})', text, re.DOTALL)
        if not match:
            raise RuntimeError('No new_mpc_params found in LLM output')

        raw = ast.literal_eval(match.group(1))
        clean = {}
        for k, v in raw.items():
            if k in PARAM_RANGES:          # silently drop unknown params
                clean[k] = self.clamp(k, v)
            else:
                self.get_logger().warn(f'Ignoring unknown param from LLM: {k}')
        return clean

    def set_params(self, params: dict):
        # Enforce hard floors so the LLM can't zero out critical params
        floors = {
            'qn': 1.0, 'qalpha': 1.0, 'qddelta': 0.1,
            'a_max': 0.5,   # car must be able to accelerate
        }
        # v_ref and v_max must be negative (reverse driving)
        for k in ('v_ref', 'v_max'):
            if k in params and params[k] > -0.1:
                self.get_logger().warn(
                    f'LLM set {k}={params[k]}, enforcing negative ceiling -0.1'
                )
                params[k] = -0.1
        for k, floor in floors.items():
            if k in params and params[k] < floor:
                self.get_logger().warn(
                    f'LLM set {k}={params[k]}, enforcing floor {floor}'
                )
                params[k] = floor

        # ros2 param set accepts one param at a time, so set them sequentially.
        applied = {}
        errors  = []
        for name, value in params.items():
            cmd = ['ros2', 'param', 'set', self.target_node, name, str(float(value))]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10.0)
            if result.returncode != 0:
                errors.append(f'{name}: {result.stderr.strip()}')
                self.get_logger().error(f'FAILED {name} = {value} → {result.stderr.strip()}')
            else:
                applied[name] = value
                self.get_logger().info(f'Set {name} = {value}')

        self._param_cache.update(applied)
        self.get_logger().info(f'Applied {len(applied)}/{len(params)} params: {applied}')

        if errors:
            raise RuntimeError(
                f'Failed setting params: {errors}\n'
                f'target_node={self.target_node}'
            )

    # ── Prompt building ───────────────────────────────────────────────────

    def build_prompt(self, target_velocity: float) -> str:
        return PROMPT_TEMPLATE.format(
            current_speed=self.current_speed,
            target_velocity=target_velocity,
            speed_error=target_velocity - self.current_speed,
            v_max_target=target_velocity - 0.5,   # more negative = faster reverse cap
            **{name: self.get_param(name) for name in PARAM_RANGES},
        )

    # ── ROS callbacks ─────────────────────────────────────────────────────

    def odom_callback(self, msg):
        self.current_speed = float(msg.twist.twist.linear.x)

    def feedback_tick(self):
        with self._mode_lock:
            mode = self.mode

        if mode == MODE_IDLE:
            return
        if self._llm_busy:
            self.get_logger().debug('Skipping tick: previous LLM call still running')
            return

        target_v = (self.slow_target_velocity if mode == MODE_SLOW
                    else self.fast_target_velocity)
        label    = mode

        self._llm_busy = True
        try:
            self.get_logger().info(
                f'[{label}] Calling LLM (speed={self.current_speed:.2f}, '
                f'target={target_v:.2f})'
            )
            raw    = self.ask_llm(self.build_prompt(target_v))
            self.get_logger().debug(f'LLM raw: {raw}')
            params = self.parse_params(raw)
            self.get_logger().info(f'Parsed: {params}')
            self.set_params(params)
            self.get_logger().info('Update complete')
        except Exception as exc:
            self.get_logger().error(f'LLM update failed: {exc}')
        finally:
            self._llm_busy = False

    # ── STOP ──────────────────────────────────────────────────────────────

    def do_stop(self):
        with self._mode_lock:
            self.mode = MODE_IDLE
        msg = AckermannDriveStamped()
        msg.header.stamp        = self.get_clock().now().to_msg()
        msg.header.frame_id     = 'base_link'
        msg.drive.speed         = 0.0
        msg.drive.steering_angle = 0.0
        self.stop_pub.publish(msg)
        self.get_logger().info(
            f'[STOP] Zero command on {self.stop_topic}; loop suspended'
        )

    # ── Keyboard ──────────────────────────────────────────────────────────

    def _keyboard_loop(self):
        try:
            fd = sys.stdin.fileno()
            self._stdin_fd           = fd
            self._stdin_old_settings = termios.tcgetattr(fd)
            tty.setcbreak(fd)
        except Exception as exc:
            self.get_logger().warn(f'Keyboard disabled (not a tty): {exc}')
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
                        f'[KEY e] SLOW (target={self.slow_target_velocity} m/s)'
                    )
                elif ch == 's':
                    with self._mode_lock:
                        self.mode = MODE_FAST
                    self.get_logger().info(
                        f'[KEY s] FAST (target={self.fast_target_velocity} m/s)'
                    )
                elif ch == 'p':
                    self.get_logger().info('[KEY p] STOP')
                    self.do_stop()
                elif ch in ('\x03', '\x04'):
                    break
        finally:
            self._restore_stdin()

    def _restore_stdin(self):
        if self._stdin_fd is not None and self._stdin_old_settings is not None:
            try:
                termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN,
                                  self._stdin_old_settings)
            except Exception:
                pass

    # ── LLM ───────────────────────────────────────────────────────────────

    def ask_llm(self, prompt: str) -> str:
        response = requests.post(
            self.mpc_url,
            json={'prompt': prompt, 'n_predict': 256, 'temperature': 0.1},
            timeout=self.llm_timeout,
        )
        response.raise_for_status()
        return response.json().get('content', '')

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