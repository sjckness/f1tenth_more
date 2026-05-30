import math
import numpy as np
from scipy.optimize import minimize

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class FrenetMPCNode(Node):
    def __init__(self):
        super().__init__('frenet_mpc_controller')

        self.N = 10
        self.dt = 0.1
        self.L = 0.25

        self.declare_parameter('qv', 10.0)
        self.declare_parameter('qn', 1.0)
        self.declare_parameter('qalpha', 30.0)
        self.declare_parameter('qac', 0.4)
        self.declare_parameter('qddelta', 2.0)
        self.declare_parameter('alat_max', 20.0)
        self.declare_parameter('a_min', -10.0)
        self.declare_parameter('a_max', 10.0)
        self.declare_parameter('v_min', 0.0)
        self.declare_parameter('v_max', 3.0)
        self.declare_parameter('v_ref', 0.45)

        self.max_steer = 0.18
        self.max_dsteer = 0.02

        self.state = None
        self.x0 = None
        self.y0 = None
        self.yaw0 = None

        self.prev_delta = 0.0
        self.prev_speed_cmd = 0.0
        self.last_solution = np.zeros(2 * self.N)

        self.path = None
        self.path_s = None
        self.path_total = 0.0

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('MPC triangle-smooth path controller started')

    def update_params(self):
        self.qv = float(self.get_parameter('qv').value)
        self.qn = float(self.get_parameter('qn').value)
        self.qalpha = float(self.get_parameter('qalpha').value)
        self.qac = float(self.get_parameter('qac').value)
        self.qddelta = float(self.get_parameter('qddelta').value)
        self.alat_max = float(self.get_parameter('alat_max').value)
        self.a_min = float(self.get_parameter('a_min').value)
        self.a_max = float(self.get_parameter('a_max').value)
        self.v_min = float(self.get_parameter('v_min').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.v_ref = float(self.get_parameter('v_ref').value)

    def yaw_from_quat(self, q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def wrap_angle(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.yaw_from_quat(msg.pose.pose.orientation)
        v = msg.twist.twist.linear.x

        if self.x0 is None:
            self.x0 = x
            self.y0 = y
            self.yaw0 = yaw
            self.build_path()
            self.get_logger().info('Initial pose locked and smooth triangle path built')

        self.state = np.array([x, y, yaw, v, self.prev_delta], dtype=float)

    def build_path(self):
        # Grande triangolo smussato locale, poi ruotato nella posa iniziale.
        # Dimensioni conservative: circa 4m x 3m.
        local_points = []

        corners = [
            np.array([0.0, 0.0]),
            np.array([8.0, 0.0]),
	    np.array([8.0, -4.0]),
            np.array([0.0, -4.0]),
        ]

        samples_per_edge = 80

        for i in range(3):
            p0 = corners[i]
            p1 = corners[(i + 1) % 3]

            for t in np.linspace(0.0, 1.0, samples_per_edge, endpoint=False):
                p = (1.0 - t) * p0 + t * p1
                local_points.append(p)

        pts = np.array(local_points)

        # smoothing circolare semplice: media mobile ripetuta
        for _ in range(8):
            pts = 0.25 * np.roll(pts, 1, axis=0) + 0.5 * pts + 0.25 * np.roll(pts, -1, axis=0)

        c = math.cos(self.yaw0)
        s = math.sin(self.yaw0)
        rot = np.array([[c, -s], [s, c]])

        world = []
        for p in pts:
            pw = np.array([self.x0, self.y0]) + rot @ p
            world.append(pw)

        self.path = np.array(world)

        ds = []
        total = 0.0
        for i in range(len(self.path)):
            p0 = self.path[i]
            p1 = self.path[(i + 1) % len(self.path)]
            total += float(np.linalg.norm(p1 - p0))
            ds.append(total)

        self.path_s = np.array(ds)
        self.path_total = float(total)

    def nearest_index(self, x, y):
        p = np.array([x, y])
        d = np.sum((self.path - p) ** 2, axis=1)
        return int(np.argmin(d))

    def reference_by_index_offset(self, idx, offset_m):
        s0 = self.path_s[idx]
        s_target = (s0 + offset_m) % self.path_total

        j = int(np.searchsorted(self.path_s, s_target))
        j = j % len(self.path)

        p = self.path[j]
        p_next = self.path[(j + 1) % len(self.path)]

        yaw_ref = math.atan2(
            p_next[1] - p[1],
            p_next[0] - p[0]
        )

        return float(p[0]), float(p[1]), yaw_ref

    def rollout_cost(self, u_flat):
        x, y, yaw, v, delta = self.state.copy()
        idx0 = self.nearest_index(x, y)

        cost = 0.0

        for k in range(self.N):
            ddelta = float(u_flat[2 * k])
            accel = float(u_flat[2 * k + 1])

            delta = float(np.clip(delta + ddelta, -self.max_steer, self.max_steer))
            v = float(np.clip(v + accel * self.dt, self.v_min, self.v_max))

            x += v * math.cos(yaw) * self.dt
            y += v * math.sin(yaw) * self.dt
            yaw += v / self.L * math.tan(delta) * self.dt
            yaw = self.wrap_angle(yaw)

            lookahead = (k + 1) * max(self.v_ref, 0.2) * self.dt
            xr, yr, yawr = self.reference_by_index_offset(idx0, lookahead)

            dx = x - xr
            dy = y - yr

            lateral_err = -math.sin(yawr) * dx + math.cos(yawr) * dy
            heading_err = self.wrap_angle(yaw - yawr)
            velocity_err = v - self.v_ref

            alat = abs(v * v * math.tan(delta) / self.L)
            alat_violation = max(0.0, alat - self.alat_max)

            cost += self.qn * lateral_err * lateral_err
            cost += self.qalpha * heading_err * heading_err
            cost += self.qv * velocity_err * velocity_err
            cost += self.qddelta * ddelta * ddelta
            cost += self.qac * accel * accel
            cost += 100.0 * alat_violation * alat_violation

        return cost

    def control_loop(self):
        if self.state is None or self.path is None:
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
        accel_cmd = float(u[1])

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

        self.prev_delta = delta_cmd
        self.prev_speed_cmd = speed_cmd

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = speed_cmd
        msg.drive.steering_angle = delta_cmd

        self.pub.publish(msg)

        self.get_logger().info(
            f'v={self.state[3]:.2f} cmd_v={speed_cmd:.2f} '
            f'v_ref={self.v_ref:.2f} delta={delta_cmd:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FrenetMPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
