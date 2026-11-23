#!/usr/bin/env python3
"""fake_sim -- a kinematic stand-in for Isaac, to test the batch's plumbing without the work.

TEST ONLY. Publishes what the stack and car-hil wait for, in Isaac's names and frames
(sim/isaac/config/ros_interface.yaml): /clock (sim time = wall), /scan (ray-marched on the office
walls of map_geo.json at the lidar plane), /odom (wheel odometry from the start), /sensors/imu/raw,
/sim/ground_truth/odom (map_sim), /sim/ground_truth/start_pose (latched), and the /sim/reset
service; it drives a bicycle model (wheelbase 0.325 m) from /ackermann_drive. No camera: the
YOLO side sees nothing. It exits when its stdin closes, so `ssh ... docker exec -i` started by
hil_office_batch.py --sim-cmd dies with the local process.

    python3 fake_sim.py --x 16.464 --y -12.604 --yaw 180 --map map_geo.json
"""
import argparse
import math
import sys
import threading
import time

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from std_srvs.srv import Trigger

WHEELBASE = 0.325
LASER_X = 0.27
RES = 0.05
N_BEAMS = 1081
A_MIN, A_MAX, R_MAX = -2.356194, 2.356194, 30.0


def stamp(t):
    s = Time()
    s.sec = int(t)
    s.nanosec = int((t - int(t)) * 1e9)
    return s


class Grid:
    def __init__(self, geo):
        x0, y0, x1, y1 = geo['bounds']
        self.x0, self.y0 = x0, y0
        self.nx, self.ny = int((x1 - x0) / RES) + 1, int((y1 - y0) / RES) + 1
        self.occ = np.zeros((self.nx, self.ny), bool)
        for x, y, w, h in geo.get('wall', []):
            i0, j0 = int((x - x0) / RES), int((y - y0) / RES)
            i1, j1 = int(math.ceil((x + w - x0) / RES)), int(math.ceil((y + h - y0) / RES))
            self.occ[max(i0, 0):i1, max(j0, 0):j1] = True
        for o in geo.get('objects', []) or []:      # furniture boxes {lo, hi}
            if isinstance(o, dict) and o.get('lo') and o.get('hi'):
                (ax, ay), (bx, by) = o['lo'], o['hi']
                i0, j0 = int((ax - x0) / RES), int((ay - y0) / RES)
                i1, j1 = int(math.ceil((bx - x0) / RES)), int(math.ceil((by - y0) / RES))
                self.occ[max(i0, 0):i1, max(j0, 0):j1] = True
        self.angles = np.linspace(A_MIN, A_MAX, N_BEAMS)
        self.steps = np.arange(0.1, R_MAX, RES / 2)

    def scan(self, x, y, yaw):
        a = yaw + self.angles
        px = x + np.outer(np.cos(a), self.steps)
        py = y + np.outer(np.sin(a), self.steps)
        i = ((px - self.x0) / RES).astype(int)
        j = ((py - self.y0) / RES).astype(int)
        inside = (i >= 0) & (i < self.nx) & (j >= 0) & (j < self.ny)
        hit = np.zeros_like(inside)
        hit[inside] = self.occ[i[inside], j[inside]]
        first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
        r = np.where(first >= 0, self.steps[np.clip(first, 0, None)], np.inf)
        return r.astype(np.float32)


class FakeSim(Node):
    def __init__(self, a, grid):
        super().__init__('isaac_f1tenth_sim')
        self.start = (a.x, a.y, math.radians(a.yaw))
        self.grid = grid
        self.lock = threading.Lock()
        self.reset_state()
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.p_clock = self.create_publisher(Clock, '/clock', rel)
        self.p_scan = self.create_publisher(LaserScan, '/scan', QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
        self.p_odom = self.create_publisher(Odometry, '/odom', rel)
        self.p_gt = self.create_publisher(Odometry, '/sim/ground_truth/odom', rel)
        self.p_imu = self.create_publisher(Imu, '/sensors/imu/raw', rel)
        self.p_start = self.create_publisher(PoseStamped, '/sim/ground_truth/start_pose', latched)
        self.create_subscription(AckermannDriveStamped, '/ackermann_drive', self.cmd,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_service(Trigger, '/sim/reset', self.on_reset)
        self.t0 = time.monotonic()
        self.pub_start()

    def reset_state(self):
        with self.lock:
            self.x, self.y, self.yaw = self.start
            self.v = self.steer = self.cmd_v = self.cmd_steer = 0.0
            self.ox = self.oy = self.oyaw = 0.0

    def now(self):
        return 1.0 + time.monotonic() - self.t0

    def pub_start(self):
        m = PoseStamped()
        m.header.stamp = stamp(self.now())
        m.header.frame_id = 'map_sim'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = self.start[0], self.start[1], 0.02
        m.pose.orientation.z, m.pose.orientation.w = math.sin(self.start[2] / 2), math.cos(self.start[2] / 2)
        self.p_start.publish(m)

    def cmd(self, m):
        with self.lock:
            self.cmd_v = float(m.drive.speed)
            self.cmd_steer = max(-0.4, min(0.4, float(m.drive.steering_angle)))

    def on_reset(self, _req, resp):
        self.reset_state()
        resp.success, resp.message = True, 'fake_sim: car back at the start pose'
        return resp

    def step(self, dt):
        with self.lock:
            dv = self.cmd_v - self.v
            self.v += max(-2.0 * dt, min(2.0 * dt, dv))
            self.steer = self.cmd_steer
            w = self.v * math.tan(self.steer) / WHEELBASE
            # stop at walls (no penetration): look 0.35 m ahead
            nx = self.x + math.cos(self.yaw) * self.v * dt
            ny = self.y + math.sin(self.yaw) * self.v * dt
            if self.grid is not None:
                fx = nx + math.cos(self.yaw) * 0.35 * (1 if self.v >= 0 else -1)
                fy = ny + math.sin(self.yaw) * 0.35 * (1 if self.v >= 0 else -1)
                i, j = int((fx - self.grid.x0) / RES), int((fy - self.grid.y0) / RES)
                if 0 <= i < self.grid.nx and 0 <= j < self.grid.ny and self.grid.occ[i, j]:
                    self.v, nx, ny, w = 0.0, self.x, self.y, 0.0
            self.x, self.y = nx, ny
            self.yaw = math.atan2(math.sin(self.yaw + w * dt), math.cos(self.yaw + w * dt))
            self.oyaw += w * dt
            self.ox += math.cos(self.oyaw) * self.v * dt
            self.oy += math.sin(self.oyaw) * self.v * dt
            return self.x, self.y, self.yaw, self.v, w, self.ox, self.oy, self.oyaw

    def odom(self, pub, t, frame, x, y, yaw, v, w):
        m = Odometry()
        m.header.stamp = stamp(t)
        m.header.frame_id, m.child_frame_id = frame, 'base_link'
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m.twist.twist.linear.x, m.twist.twist.angular.z = v, w
        pc = [0.0] * 36
        pc[0] = pc[7] = 0.2
        pc[14] = pc[21] = pc[28] = 1e6
        pc[35] = 0.03
        tc = [0.0] * 36
        tc[0] = 0.001
        tc[7] = tc[14] = tc[21] = tc[28] = 1e6
        tc[35] = 0.01
        m.pose.covariance, m.twist.covariance = pc, tc
        pub.publish(m)

    def loop(self):
        dt, k, last = 0.01, 0, time.monotonic()
        while rclpy.ok():
            nxt = last + dt
            time.sleep(max(0.0, nxt - time.monotonic()))
            last = nxt
            t = self.now()
            c = Clock()
            c.clock = stamp(t)
            self.p_clock.publish(c)
            x, y, yaw, v, w, ox, oy, oyaw = self.step(dt)
            k += 1
            if k % 2 == 0:       # 50 Hz
                self.odom(self.p_gt, t, 'map_sim', x, y, yaw, v, w)
                self.odom(self.p_odom, t, 'odom', ox, oy, oyaw, v, w)
                im = Imu()
                im.header.stamp, im.header.frame_id = stamp(t), 'imu'
                im.orientation.z, im.orientation.w = math.sin(oyaw / 2), math.cos(oyaw / 2)
                im.angular_velocity.z = w
                im.linear_acceleration.z = 9.81
                im.angular_velocity_covariance = [1.7e-6, 0, 0, 0, 1.7e-6, 0, 0, 0, 1.7e-6]
                im.linear_acceleration_covariance = [1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3]
                im.orientation_covariance = [1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3]
                self.p_imu.publish(im)
            if k % 5 == 0:       # 20 Hz
                s = LaserScan()
                s.header.stamp, s.header.frame_id = stamp(t), 'laser'
                s.angle_min, s.angle_max = A_MIN, A_MAX
                s.angle_increment = (A_MAX - A_MIN) / (N_BEAMS - 1)
                s.scan_time, s.range_min, s.range_max = 0.05, 0.02, R_MAX
                if self.grid is not None:
                    lx, ly = x + LASER_X * math.cos(yaw), y + LASER_X * math.sin(yaw)
                    s.ranges = self.grid.scan(lx, ly, yaw).tolist()
                else:
                    s.ranges = [float('inf')] * N_BEAMS
                self.p_scan.publish(s)
            if k % 500 == 0:
                self.pub_start()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, required=True)
    ap.add_argument('--y', type=float, required=True)
    ap.add_argument('--z', type=float, default=0.02)
    ap.add_argument('--yaw', type=float, required=True, help='deg')
    ap.add_argument('--map', default='')
    a = ap.parse_args()
    grid = None
    if a.map:
        import json
        grid = Grid(json.load(open(a.map)))
    rclpy.init()
    node = FakeSim(a, grid)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    threading.Thread(target=node.loop, daemon=True).start()
    print(f'fake_sim: start ({a.x}, {a.y}, {a.yaw} deg), map {a.map or "none"}', flush=True)
    sys.stdin.read()         # until stdin closes (the local ssh / driver went away)
    print('fake_sim: stdin closed, exiting', flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
