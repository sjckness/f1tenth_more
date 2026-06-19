import math
import numpy as np
from scipy.optimize import minimize

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class KinematicMPCNode(Node):
    def __init__(self):
        super().__init__('kinematic_mpc_controller')

        self.N = 10
        self.dt = 0.1
        self.L = 0.25

        self.v_ref = 0.2
        self.max_steer = 0.35
        self.max_accel = 1.0

        self.state = None
        self.last_solution = np.zeros(2 * self.N)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('Kinematic MPC node started')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        v = msg.twist.twist.linear.x
        self.state = np.array([x, y, yaw, v], dtype=float)

    def reference(self, k):
        s = k * self.dt * self.v_ref

        if s < 2.0:
            x_ref = s
            y_ref = 0.0
            yaw_ref = 0.0
        else:
            r = 1.0
            theta = min((s - 2.0) / r, math.pi / 2.0)
            x_ref = 2.0 + r * math.sin(theta)
            y_ref = -r * (1.0 - math.cos(theta))
            yaw_ref = -theta

        return np.array([x_ref, y_ref, yaw_ref, self.v_ref])

    def rollout_cost(self, u_flat):
        x = self.state.copy()
        cost = 0.0

        for k in range(self.N):
            a = u_flat[2 * k]
            delta = u_flat[2 * k + 1]

            x_ref = self.reference(k)

            dx = x[0] - x_ref[0]
            dy = x[1] - x_ref[1]
            dyaw = math.atan2(math.sin(x[2] - x_ref[2]), math.cos(x[2] - x_ref[2]))
            dv = x[3] - x_ref[3]

            cost += 15.0 * (dx * dx + dy * dy)
            cost += 5.0 * dyaw * dyaw
            cost += 8.0 * dv * dv
            cost += 0.2 * a * a
            cost += 2.0 * delta * delta

            x[0] += x[3] * math.cos(x[2]) * self.dt
            x[1] += x[3] * math.sin(x[2]) * self.dt
            x[2] += x[3] / self.L * math.tan(delta) * self.dt
            x[3] += a * self.dt

        return cost

    def control_loop(self):
        if self.state is None:
            return

        bounds = []
        for _ in range(self.N):
            bounds.append((-self.max_accel, self.max_accel))
            bounds.append((-self.max_steer, self.max_steer))

        result = minimize(
            self.rollout_cost,
            self.last_solution,
            method='SLSQP',
            bounds=bounds,
            options={'maxiter': 30, 'ftol': 1e-2, 'disp': False},
        )

        if result.success:
            u = result.x
            self.last_solution[:-2] = u[2:]
            self.last_solution[-2:] = u[-2:]
        else:
            u = self.last_solution

        a_cmd = float(u[0])
        steer_cmd = 0.0

        speed_cmd = 0.25

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = speed_cmd
        msg.drive.steering_angle = steer_cmd

        self.pub.publish(msg)

        self.get_logger().info(
            f'v={self.state[3]:.2f} speed_cmd={speed_cmd:.2f} '
            f'steer={steer_cmd:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = KinematicMPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
