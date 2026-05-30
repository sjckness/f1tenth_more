import math
import numpy as np
from scipy.optimize import minimize

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


# ---------------------------------------------------------------------------
# Figure-8 sequence (repeating):
#   STRAIGHT  → travel 1.0 m
#   RIGHT     → rotate 180 ° (heading change > π)
#   STRAIGHT  → travel 1.0 m
#   LEFT      → rotate 180 ° (heading change > π)
#   STRAIGHT  → travel 1.0 m
#   LEFT      → rotate 180 ° (heading change > π)
#   STRAIGHT  → travel 1.0 m
#   RIGHT     → rotate 180 ° (heading change > π)
#   … then repeats
# ---------------------------------------------------------------------------
STRAIGHT_DIST = 1.0
TURN_ANGLE    = math.radians(170)   # 170° tolerance instead of 180°

STEP_SEQUENCE = [
    ('STRAIGHT', None),
    ('RIGHT',    None),
    ('STRAIGHT', None),
    ('LEFT',     None),
    ('STRAIGHT', None),
    ('LEFT',     None),
    ('STRAIGHT', None),
    ('RIGHT',    None),
]


class AndreMPCNode(Node):
    def __init__(self):
        super().__init__('andre_mpc_controller')

        self.N  = 10
        self.dt = 0.1
        self.L  = 0.25

        self.declare_parameter('qv',       10.0)
        self.declare_parameter('qn',       20.0)
        self.declare_parameter('qalpha',   30.0)
        self.declare_parameter('qac',       0.4)
        self.declare_parameter('qddelta',   2.0)
        self.declare_parameter('alat_max', 20.0)
        self.declare_parameter('a_min',   -10.0)
        self.declare_parameter('a_max',    10.0)
        self.declare_parameter('v_min',     0.0)
        self.declare_parameter('v_max',     1.0)
        self.declare_parameter('v_ref',     1.0)

        self.max_steer  = 0.18
        self.max_dsteer = 0.02

        self.state  = None
        self.x0     = None
        self.y0     = None
        self.yaw0   = None

        self.cx = None
        self.cy = None
        self.R  = None

        self.current_step  = 0
        self.circle_dir    = -1.0

        self.seg_start_x   = None
        self.seg_start_y   = None
        self.seg_start_yaw = None
        self.seg_yaw_accum = 0.0
        self.seg_prev_yaw  = None

        self.prev_delta     = 0.0
        self.prev_speed_cmd = 0.0
        self.last_solution  = np.zeros(2 * self.N)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pub   = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f'Figure-8 MPC node started — step 0: {STEP_SEQUENCE[0][0]}'
        )

    # ------------------------------------------------------------------
    def update_params(self):
        self.qv       = float(self.get_parameter('qv').value)
        self.qn       = float(self.get_parameter('qn').value)
        self.qalpha   = float(self.get_parameter('qalpha').value)
        self.qac      = float(self.get_parameter('qac').value)
        self.qddelta  = float(self.get_parameter('qddelta').value)
        self.alat_max = float(self.get_parameter('alat_max').value)
        self.a_min    = float(self.get_parameter('a_min').value)
        self.a_max    = float(self.get_parameter('a_max').value)
        self.v_min    = float(self.get_parameter('v_min').value)
        self.v_max    = float(self.get_parameter('v_max').value)
        self.v_ref    = float(self.get_parameter('v_ref').value)

    # ------------------------------------------------------------------
    def yaw_from_quat(self, q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def wrap_angle(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    # ------------------------------------------------------------------
    def _update_circle_center(self, x, y, yaw):
        self.R  = self.L / math.tan(self.max_steer)
        self.cx = x - self.circle_dir * self.R * math.sin(yaw)
        self.cy = y + self.circle_dir * self.R * math.cos(yaw)
        self.get_logger().info(
            f'Circle center fixed: ({self.cx:.2f}, {self.cy:.2f}), '
            f'R={self.R:.2f} m, dir={"LEFT" if self.circle_dir > 0 else "RIGHT"}'
        )

    # ------------------------------------------------------------------
    def _begin_step(self, x, y, yaw):
        step_name = STEP_SEQUENCE[self.current_step][0]

        if step_name == 'STRAIGHT':
            self.seg_start_x   = x
            self.seg_start_y   = y
            self.seg_yaw_accum = 0.0
            self.seg_prev_yaw  = None

        else:  # RIGHT or LEFT
            self.circle_dir = -1.0 if step_name == 'RIGHT' else 1.0
            self._update_circle_center(x, y, yaw)
            self.seg_yaw_accum = 0.0
            self.seg_prev_yaw  = yaw
            self.seg_start_x   = x
            self.seg_start_y   = y

        self.get_logger().info(f'→ Begin step {self.current_step}: {step_name}')

    # ------------------------------------------------------------------
    def _check_step_transition(self, x, y, yaw):
        step_name = STEP_SEQUENCE[self.current_step][0]

        if step_name == 'STRAIGHT':
            dist = math.hypot(x - self.seg_start_x, y - self.seg_start_y)
            if dist > STRAIGHT_DIST:
                self.get_logger().info(
                    f'STRAIGHT done: {dist:.3f} m > {STRAIGHT_DIST} m'
                )
                return True

        else:  # TURN
            if self.seg_prev_yaw is not None:
                dyaw = self.wrap_angle(yaw - self.seg_prev_yaw)
                self.seg_yaw_accum += abs(dyaw)   # just count total rotation regardless of sign

            self.seg_prev_yaw = yaw

            self.get_logger().info(
                f'[TURN {"R" if self.circle_dir < 0 else "L"}] '
                f'accum={math.degrees(self.seg_yaw_accum):.1f}° '
                f'yaw={math.degrees(yaw):.1f}°'
            )

            if self.seg_yaw_accum > TURN_ANGLE:
                self.get_logger().info(
                    f'{"RIGHT" if self.circle_dir < 0 else "LEFT"} turn done: '
                    f'{math.degrees(self.seg_yaw_accum):.1f}° accumulated'
                )
                return True

        return False

    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        yaw = self.yaw_from_quat(msg.pose.pose.orientation)
        v   = msg.twist.twist.linear.x

        if self.x0 is None:
            self.x0   = x
            self.y0   = y
            self.yaw0 = yaw
            self.get_logger().info(
                f'Initial pose: ({x:.2f}, {y:.2f}), yaw={math.degrees(yaw):.1f}°'
            )
            self._begin_step(x, y, yaw)

        self.state = np.array([x, y, yaw, v, self.prev_delta], dtype=float)

        if self._check_step_transition(x, y, yaw):
            self.current_step = (self.current_step + 1) % len(STEP_SEQUENCE)
            self._begin_step(x, y, yaw)

    # ------------------------------------------------------------------
    def get_circle_reference(self, x, y, k_step):
        step_name = STEP_SEQUENCE[self.current_step][0]

        if step_name == 'STRAIGHT':
            lookahead = k_step * max(self.v_ref, 0.1) * self.dt
            yaw_ref   = self.state[2]
            xr = x + lookahead * math.cos(yaw_ref)
            yr = y + lookahead * math.sin(yaw_ref)
            return xr, yr, yaw_ref

        # TURN — use fixed circle center
        angle_now   = math.atan2(y - self.cy, x - self.cx)
        lookahead   = k_step * max(self.v_ref, 0.1) * self.dt
        delta_angle = self.circle_dir * lookahead / self.R
        angle_ref   = angle_now + delta_angle

        xr = self.cx + self.R * math.cos(angle_ref)
        yr = self.cy + self.R * math.sin(angle_ref)

        yaw_ref = math.atan2(
             self.circle_dir * math.cos(angle_ref),
            -self.circle_dir * math.sin(angle_ref)
        )

        return xr, yr, yaw_ref

    # ------------------------------------------------------------------
    def rollout_cost(self, u_flat):
        x, y, yaw, v, delta = self.state.copy()

        cost = 0.0

        for k in range(self.N):
            ddelta = float(u_flat[2 * k])
            accel  = float(u_flat[2 * k + 1])

            delta = float(np.clip(delta + ddelta, -self.max_steer, self.max_steer))
            v     = float(np.clip(v + accel * self.dt, self.v_min, self.v_max))

            x   += v * math.cos(yaw) * self.dt
            y   += v * math.sin(yaw) * self.dt
            yaw += v / self.L * math.tan(delta) * self.dt
            yaw  = self.wrap_angle(yaw)

            xr, yr, yawr = self.get_circle_reference(x, y, k + 1)

            dx = x - xr
            dy = y - yr

            lateral_err  = -math.sin(yawr) * dx + math.cos(yawr) * dy
            heading_err  = self.wrap_angle(yaw - yawr)
            velocity_err = v - self.v_ref

            alat           = abs(v * v * math.tan(delta) / self.L)
            alat_violation = max(0.0, alat - self.alat_max)

            cost += self.qn      * lateral_err  * lateral_err
            cost += self.qalpha  * heading_err  * heading_err
            cost += self.qv      * velocity_err * velocity_err
            cost += self.qddelta * ddelta       * ddelta
            cost += self.qac     * accel        * accel
            cost += 100.0        * alat_violation * alat_violation

        return cost

    # ------------------------------------------------------------------
    def control_loop(self):
        if self.state is None or self.cx is None:
            return

        self.update_params()

        bounds = []
        for _ in range(self.N):
            bounds.append((-self.max_dsteer, self.max_dsteer))
            bounds.append((self.a_min, self.a_max))

        result = minimize(
            self.rollout_cost,
            self.last_solution,
            method='SLSQP',
            bounds=bounds,
            options={'maxiter': 25, 'ftol': 1e-2, 'disp': False},
        )

        if result.success:
            u = result.x
            self.last_solution[:-2] = u[2:]
            self.last_solution[-2:] = u[-2:]
        else:
            u = self.last_solution

        ddelta_cmd = float(u[0])
        accel_cmd  = float(u[1])

        delta_cmd = float(np.clip(
            self.prev_delta + ddelta_cmd,
            -self.max_steer,
            self.max_steer
        ))

        speed_cmd = float(np.clip(
            self.prev_speed_cmd + accel_cmd * self.dt,
            self.v_min,
            self.v_max
        ))

        self.prev_delta     = delta_cmd
        self.prev_speed_cmd = speed_cmd

        msg = AckermannDriveStamped()
        msg.header.stamp         = self.get_clock().now().to_msg()
        msg.drive.speed          = speed_cmd
        msg.drive.steering_angle = delta_cmd

        self.pub.publish(msg)

        step_name = STEP_SEQUENCE[self.current_step][0]
        self.get_logger().info(
            f'[{step_name}] '
            f'x={self.state[0]:.2f} y={self.state[1]:.2f} yaw={math.degrees(self.state[2]):.1f}° '
            f'v={self.state[3]:.2f} cmd_v={speed_cmd:.2f} '
            f'delta={delta_cmd:.3f} step={self.current_step}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = AndreMPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()