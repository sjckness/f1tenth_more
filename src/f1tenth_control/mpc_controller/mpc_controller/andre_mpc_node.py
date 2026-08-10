import math
import numpy as np
from scipy.optimize import minimize

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

from f1tenth_params.param_defaults import get_odom_topic


# ---------------------------------------------------------------------------
# Circle MPC node
# Steering = geometric feedforward (pure pursuit to circle) +
#            MPC correction term (bad params → LLM has something to tune)
# Speed    = open-loop sine profile around CIRCLE_SPEED
# ---------------------------------------------------------------------------
CIRCLE_STEER = 0.1     # [rad] desired steady-state steering for the circle
CIRCLE_SPEED = 1.0     # [m/s] centre of the sine speed profile
SINE_AMP     = 0.0     # [m/s] sine amplitude (0 → constant speed)
SINE_PERIOD  = 4.0     # [s]


class AndreMPCNode(Node):
    def __init__(self):
        super().__init__('andre_mpc_controller')

        self.N  = 10
        self.dt = 0.1
        self.L  = 0.25

        # ── MPC correction params ─────────────────────────────────────────
        self.declare_parameter('qn',       50.0)   # high: tight radial tracking
        self.declare_parameter('qv',       50.0)   # high: tight radial tracking
        self.declare_parameter('qalpha',   30.0)   # high: stay tangent to circle
        self.declare_parameter('qddelta',   2.0)   # smooth steering corrections
        self.declare_parameter('alat_max', 10.0)   # lateral accel limit
        self.declare_parameter('a_min',    -3.0)
        self.declare_parameter('a_max',     3.0)
        self.declare_parameter('v_min',     -1.5)
        self.declare_parameter('v_max',     1.5)
        self.declare_parameter('v_ref',     CIRCLE_SPEED)
        self.declare_parameter('sine_amp',    SINE_AMP)
        self.declare_parameter('sine_period', SINE_PERIOD)
        self.declare_parameter('steer_sign',  1.0)

        self.max_steer  = 0.18
        self.max_dsteer = 0.05   # larger step so correction can act fast

        # Circle geometry — fixed from first odom
        self.R  = self.L / math.tan(CIRCLE_STEER)
        self.cx = None
        self.cy = None
        self.t0 = None

        self.state: np.ndarray | None = None
        self.prev_delta     = CIRCLE_STEER   # start on the circle steering
        self.last_solution  = np.zeros(2 * self.N)
        self.consec_failures = 0

        # Follows localization_source -- see MPC_corr.py's identical comment /
        # f1tenth_params' param_defaults.get_odom_topic().
        self.create_subscription(Odometry, get_odom_topic(), self.odom_callback, 10)
        self.pub   = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.update_params()
        self.get_logger().info(
            f'Circle MPC — R={self.R:.2f} m, '
            f'steer_ff={CIRCLE_STEER} rad, speed=[{CIRCLE_SPEED-SINE_AMP:.1f}, '
            f'{CIRCLE_SPEED+SINE_AMP:.1f}] m/s'
        )

    # ── Parameter refresh ─────────────────────────────────────────────────
    def update_params(self):
        self.qn          = float(self.get_parameter('qn').value)
        self.qalpha      = float(self.get_parameter('qalpha').value)
        self.qddelta     = float(self.get_parameter('qddelta').value)
        self.alat_max    = float(self.get_parameter('alat_max').value)
        self.a_min       = float(self.get_parameter('a_min').value)
        self.a_max       = float(self.get_parameter('a_max').value)
        self.v_min       = float(self.get_parameter('v_min').value)
        self.v_max       = float(self.get_parameter('v_max').value)
        self.v_ref       = float(self.get_parameter('v_ref').value)
        self.sine_amp    = float(self.get_parameter('sine_amp').value)
        self.sine_period = float(self.get_parameter('sine_period').value)
        self.steer_sign  = float(self.get_parameter('steer_sign').value)

    # ── Helpers ───────────────────────────────────────────────────────────
    def yaw_from_quat(self, q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def wrap_angle(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def sine_v_ref(self, t_offset_steps=0):
        t = (self.get_clock().now().nanoseconds * 1e-9 - self.t0) \
            + t_offset_steps * self.dt
        return self.v_ref + self.sine_amp * math.sin(
            2 * math.pi * t / self.sine_period
        )

    # ── Geometric feedforward steering ────────────────────────────────────
    def feedforward_delta(self, x, y, yaw):
        """Pure-pursuit steering to bring the car onto the circle.
        Returns the steering angle [rad] that points toward the nearest
        point on the circle, blended with the steady-state circle steer."""
        dist  = math.hypot(x - self.cx, y - self.cy)
        # radial error: positive = outside circle, negative = inside
        r_err = dist - self.R

        # Proportional correction: steer toward center if outside, away if inside
        # Gain chosen so 1 m error → ~0.05 rad correction
        correction = -0.05 * r_err

        ff = CIRCLE_STEER + correction
        return float(np.clip(ff, -self.max_steer, self.max_steer))

    # ── Odom callback ─────────────────────────────────────────────────────
    def odom_callback(self, msg):
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        yaw = self.yaw_from_quat(msg.pose.pose.orientation)
        v   = msg.twist.twist.linear.x   # keep sign for diagnostics

        if self.cx is None:
            self.cx = x - self.R * math.sin(yaw)
            self.cy = y + self.R * math.cos(yaw)
            self.t0 = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info(
                f'Circle center: ({self.cx:.2f}, {self.cy:.2f}), R={self.R:.2f} m'
            )

        self.state = np.array([x, y, yaw, abs(v), self.prev_delta], dtype=float)

    # ── Reference on circle ───────────────────────────────────────────────
    def get_reference(self, x, y, k_step):
        angle_now = math.atan2(y - self.cy, x - self.cx)
        lookahead = k_step * max(self.sine_v_ref(k_step), 0.1) * self.dt
        angle_ref = angle_now + lookahead / self.R   # CCW

        xr      = self.cx + self.R * math.cos(angle_ref)
        yr      = self.cy + self.R * math.sin(angle_ref)
        yaw_ref = math.atan2(math.cos(angle_ref), -math.sin(angle_ref))
        return xr, yr, yaw_ref

    # ── MPC correction cost ───────────────────────────────────────────────
    def rollout_cost(self, u_flat):
        """MPC optimises a small CORRECTION on top of the feedforward delta.
        u_flat = [ddelta_0, 0, ddelta_1, 0, ...] — accel slot unused (speed open-loop)."""
        x, y, yaw, v, delta = self.state.copy()
        v = max(v, self.v_min)

        cost = 0.0
        for k in range(self.N):
            ddelta = float(u_flat[2 * k])
            # feedforward for this rollout step
            ff     = self.feedforward_delta(x, y, yaw)
            delta  = float(np.clip(ff + ddelta, -self.max_steer, self.max_steer))

            x   += v * math.cos(yaw) * self.dt
            y   += v * math.sin(yaw) * self.dt
            yaw += v / self.L * math.tan(delta) * self.dt
            yaw  = self.wrap_angle(yaw)

            # Distance to circle — strong, smooth gradient everywhere
            radial_err = math.hypot(x - self.cx, y - self.cy) - self.R

            # Heading: should be tangent to circle
            angle_on_circle = math.atan2(y - self.cy, x - self.cx)
            tangent_yaw     = angle_on_circle + math.pi / 2
            heading_err     = self.wrap_angle(yaw - tangent_yaw)

            alat           = abs(v * v * math.tan(delta) / self.L)
            alat_violation = max(0.0, alat - self.alat_max)

            cost += self.qn      * radial_err   ** 2
            cost += self.qalpha  * heading_err  ** 2
            cost += self.qddelta * ddelta        ** 2
            cost += 100.0        * alat_violation ** 2

        return cost

    # ── Control loop ──────────────────────────────────────────────────────
    def control_loop(self):
        if self.state is None or self.cx is None:
            return

        self.update_params()
        state_snap = self.state.copy()
        x, y, yaw, v = state_snap[0], state_snap[1], state_snap[2], state_snap[3]

        # ── Feedforward steering ───────────────────────────────────────────
        ff_delta = self.feedforward_delta(x, y, yaw)

        # ── MPC correction ────────────────────────────────────────────────
        bounds = [(-self.max_dsteer, self.max_dsteer), (0.0, 0.0)] * self.N

        result = minimize(
            self.rollout_cost,
            self.last_solution,
            method='SLSQP',
            bounds=bounds,
            options={'maxiter': 50, 'ftol': 1e-2, 'disp': False},
        )

        if result.success:
            correction = float(result.x[0])
            self.last_solution[:-2] = result.x[2:]
            self.last_solution[-2:] = result.x[-2:]
            self.consec_failures = 0
        else:
            self.consec_failures += 1
            correction = 0.0
            if self.consec_failures >= 3:
                self.last_solution = np.zeros(2 * self.N)

        delta_cmd = float(np.clip(
            ff_delta + correction,
            -self.max_steer, self.max_steer
        ))

        # ── Open-loop speed toward sine profile ───────────────────────────
        target_v   = self.sine_v_ref()
        speed_step = math.copysign(
            min(self.a_max * self.dt, abs(target_v - v)),
            target_v - v
        )
        speed_cmd = float(np.clip(v + speed_step, self.v_min, self.v_max))

        self.prev_delta = delta_cmd

        # ── Publish ───────────────────────────────────────────────────────
        msg = AckermannDriveStamped()
        msg.header.stamp         = self.get_clock().now().to_msg()
        msg.drive.speed          = speed_cmd
        msg.drive.steering_angle = self.steer_sign * delta_cmd
        self.pub.publish(msg)

        # Tracking errors for logging / LLM tuner
        dist       = math.hypot(x - self.cx, y - self.cy)
        radial_err = dist - self.R
        angle_on_c = math.atan2(y - self.cy, x - self.cx)
        tangent_yaw = angle_on_c + math.pi / 2
        heading_err = math.degrees(self.wrap_angle(yaw - tangent_yaw))

        self.get_logger().info(
            f'x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f}° '
            f'v_ref={target_v:.2f} v={v:.2f} cmd_v={speed_cmd:.2f} '
            f'ff={ff_delta:.3f} corr={correction:.3f} delta={delta_cmd:.3f} '
            f'r_err={radial_err:.3f} h_err={heading_err:.1f}°'
        )


def main(args=None):
    rclpy.init(args=args)
    node = AndreMPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()