#!/usr/bin/env python3
"""Scenario harness for the REAL mpc_corr node, no perception, no Gazebo.

Review item 6. Runs on linus next to a plain `ros2 run mpc_controller
mpc_corr` (tools/run_scenarios.sh starts both on an isolated domain):

  * plays the car: a kinematic bicycle (same wheelbase / lr as the MPC
    model) integrated at 100 Hz from /drive, with a first-order speed lag
    (tau 0.15 s, the measured stop latency), the 0.5 rad/s steering rate, and
    a breakaway speed (a command under 0.13 m/s does not move a car at rest,
    R011) -- published as /odometry/filtered;
  * plays perception: /perception/obstacles_2d in base_link at 8.2 Hz (the
    measured detection rate), only while the obstacle is within the
    scenario's appearance distance AND inside the camera's +-50.5 deg FOV
    (camera 0.12 m ahead of base_link). By default it publishes what the
    real pipeline does: the disc centred on the visible FRONT surface, r =
    half the width; --true-centre publishes the true centre instead;
  * plays the mission: a drive/straight move at 0.4 m/s (/mpc/goal_drive),
    ended by this node with /mpc/hold at 5 m of travel, by /mpc/blocked, or
    at the scenario timeout.

Measures, per scenario: outcome, time, the minimum gap between the car's
OUTLINE (swept_clearance_body_*) and the TRUE obstacle disc, every /drive
speed in (0, 0.4) m/s (must be none), and the pass sides /mpc/pass_sides
reported (must never flip).

    ros2 run mpc_controller campaign_scenario [--only NAME ...] [--out results.json]
"""

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

from f1tenth_description.vehicle_frames import base_front_x
from f1tenth_messages.msg import DriveCommand, Obstacle2D, Obstacle2DArray
from f1tenth_params.param_defaults import get_value

CAMERA_X_M = 0.12
HALF_FOV_RAD = math.radians(50.5)
DETECTION_PERIOD_S = 1.0 / 8.2
PLANT_DT = 0.01
SPEED_TAU_S = 0.15
STEER_RATE = 0.5
BREAKAWAY_MPS = 0.13
GOAL_M = 5.0


@dataclass
class Scenario:
    name: str
    r: float            # true radius [m] (half the width)
    y: float            # lateral offset [m], + left
    appear: float       # published once within this distance of base_link [m]
    x: float = 3.0      # distance ahead at start [m]
    timeout: float = 30.0


SCENARIOS = [
    Scenario(f'{w:.1f}m_{where}_{"late1.4" if ap < 9 else "early"}', w / 2.0, y, ap)
    for w in (0.1, 0.3, 0.5, 1.0)
    for where, y in (('centred', 0.0), ('off0.3', 0.3))
    for ap in (math.inf, 1.4)
]


def body_gap(x, y, yaw, ox, oy, r, front, rear, half_w):
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = ox - x, oy - y
    lon = dx * c + dy * s
    lat = -dx * s + dy * c
    qx = min(max(lon, rear), front)
    qy = min(max(lat, -half_w), half_w)
    return math.hypot(lon - qx, lat - qy) - r


class ScenarioNode(Node):

    def __init__(self, scenarios, true_centre):
        super().__init__('campaign_scenario')
        self.scenarios = list(scenarios)
        self.true_centre = true_centre
        self.L = float(get_value('mpc_wheelbase_m'))
        self.lr = 0.17
        self.front = base_front_x()
        self.rear = float(get_value('swept_clearance_body_rear_x_m'))
        self.half_w = float(get_value('swept_clearance_body_half_width_m'))
        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.obs_pub = self.create_publisher(Obstacle2DArray, '/perception/obstacles_2d', 10)
        self.goal_pub = self.create_publisher(DriveCommand, '/mpc/goal_drive', 10)
        self.hold_pub = self.create_publisher(Bool, '/mpc/hold', 10)
        self.create_subscription(AckermannDriveStamped, '/drive', self._on_drive, 10)
        self.create_subscription(String, '/mpc/blocked', self._on_blocked, 10)
        self.create_subscription(String, '/mpc/pass_sides', self._on_sides, 10)
        self.results = []
        self.cmd = (0.0, 0.0)
        self.index = -1
        self.create_timer(PLANT_DT, self._step)
        self._next_detection = 0.0
        self._start_next()

    # -- scenario lifecycle ------------------------------------------------
    def _start_next(self):
        self.index += 1
        if self.index >= len(self.scenarios):
            self.done = True
            return
        self.done = False
        self.sc = self.scenarios[self.index]
        self.state = [0.0, 0.0, 0.0, 0.0, 0.0]   # x, y, yaw, v, steer
        self.t0 = time.monotonic()
        self.seen = False
        self.res = {'scenario': self.sc.name, 'r': self.sc.r, 'y': self.sc.y,
                    'appear': None if math.isinf(self.sc.appear) else self.sc.appear,
                    'outcome': None, 't': None, 'min_body_gap': math.inf,
                    'crawl_cmds': 0, 'sides': [], 'blocked': None, 'steer_trace': []}
        self.cmd = (0.0, 0.0)
        self._settle_until = self.t0 + 1.0       # odometry first, then the goal
        self._goal_sent = False
        self._goal_ros_time = math.inf

    def _finish(self, outcome):
        self.res['outcome'] = outcome
        self.res['t'] = round(time.monotonic() - self.t0 - 1.0, 2)
        sides = [s for s in self.res['sides'] if s]
        self.res['side_flips'] = sum(1 for a, b in zip(sides, sides[1:]) if a != b)
        self.res['min_body_gap'] = round(self.res['min_body_gap'], 3)
        tr = self.res['steer_trace']
        self.res['max_abs_steer'] = round(max((abs(d) for _, d in tr), default=0.0), 3)
        rates = [abs(d1 - d0) / max(t1 - t0, 1e-3)
                 for (t0, d0), (t1, d1) in zip(tr, tr[1:])]
        self.res['max_steer_rate'] = round(max(rates, default=0.0), 3)
        # Steering reversals: sign changes of the command's increments above
        # 0.01 rad -- a measure of chatter after the ramp.
        incs = [d1 - d0 for (_, d0), (_, d1) in zip(tr, tr[1:]) if abs(d1 - d0) > 0.01]
        self.res['steer_reversals'] = sum(
            1 for a, b in zip(incs, incs[1:]) if (a > 0) != (b > 0))
        self.results.append(self.res)
        self.hold_pub.publish(Bool(data=True))
        self.get_logger().info(json.dumps(self.res))
        self._start_next()

    # -- callbacks -----------------------------------------------------------
    def _on_drive(self, msg):
        self.cmd = (float(msg.drive.speed), float(msg.drive.steering_angle))
        if not self.done and self._goal_sent:
            if 0.0 < self.cmd[0] < 0.4 - 1e-6:
                self.res['crawl_cmds'] += 1
            self.res['steer_trace'].append(
                (round(time.monotonic() - self.t0 - 1.0, 3), round(self.cmd[1], 4)))

    def _on_blocked(self, msg):
        if self.done or not self._goal_sent or self.res['outcome']:
            return
        try:
            record = json.loads(msg.data)
        except ValueError:
            return
        # mpc_corr repeats the PREVIOUS scenario's verdict until our new goal
        # reaches it: only a verdict stamped after this goal counts.
        if float(record.get('t_ros', 0.0)) <= self._goal_ros_time:
            return
        self.res['blocked'] = record.get('reason')
        self.res['blocked_detail'] = record.get('detail')
        self.res['blocked_t_ros'] = record.get('t_ros')
        self.res['goal_t_ros'] = self._goal_ros_time
        self._finish('BLOCKED')

    def _on_sides(self, msg):
        if self.done or not self._goal_sent:
            return
        try:
            for ob in json.loads(msg.data).get('obstacles', []):
                if ob.get('side'):
                    self.res['sides'].append(ob['side'])
        except ValueError:
            pass

    # -- the plant, perception and mission ---------------------------------
    def _step(self):
        if self.done:
            return
        now = time.monotonic()
        x, y, yaw, v, steer = self.state
        if now >= self._settle_until and not self._goal_sent:
            goal = DriveCommand(mode='straight', turn_sign=0.0, turn_mag_deg=0.0,
                                speed=0.4, d_safe=-1.0)
            self.goal_pub.publish(goal)
            self.hold_pub.publish(Bool(data=False))
            self._goal_ros_time = self.get_clock().now().nanoseconds * 1e-9
            self._goal_sent = True
        v_cmd, steer_cmd = self.cmd if self._goal_sent else (0.0, 0.0)
        if abs(v) < 1e-3 and 0.0 < v_cmd < BREAKAWAY_MPS:
            v_cmd = 0.0
        v += (v_cmd - v) * (PLANT_DT / SPEED_TAU_S)
        steer += max(-STEER_RATE * PLANT_DT, min(STEER_RATE * PLANT_DT, steer_cmd - steer))
        beta = math.atan(self.lr / self.L * math.tan(steer))
        x += v * math.cos(yaw + beta) * PLANT_DT
        y += v * math.sin(yaw + beta) * PLANT_DT
        yaw += v / self.lr * math.sin(beta) * PLANT_DT
        self.state = [x, y, yaw, v, steer]
        self._publish_odom(x, y, yaw, v)

        sc = self.sc
        gap = body_gap(x, y, yaw, sc.x, sc.y, sc.r, self.front, self.rear, self.half_w)
        if self._goal_sent:
            self.res['min_body_gap'] = min(self.res['min_body_gap'], gap)
        if now >= self._next_detection:
            self._next_detection = now + DETECTION_PERIOD_S
            self._publish_obstacles(x, y, yaw)

        if self._goal_sent:
            if x >= GOAL_M:
                self._finish('passed')
            elif now - self.t0 > sc.timeout:
                self._finish('timeout')

    def _publish_odom(self, x, y, yaw, v):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.twist.twist.linear.x = v
        self.odom_pub.publish(msg)

    def _publish_obstacles(self, x, y, yaw):
        sc = self.sc
        msg = Obstacle2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = sc.x - x, sc.y - y
        bx, by = c * dx + s * dy, -s * dx + c * dy          # true centre, base_link
        cam_dx, cam_dy = bx - CAMERA_X_M, by
        rng = math.hypot(bx, by)
        visible = (rng <= sc.appear and cam_dx > 0
                   and abs(math.atan2(cam_dy, cam_dx)) <= HALF_FOV_RAD)
        if visible:
            self.seen = True
            if self.true_centre:
                px, py = bx, by
            else:
                # The real pipeline: median mask depth = the visible FRONT
                # surface, bearing of the box centre; r = half the box width.
                dist = math.hypot(cam_dx, cam_dy)
                k = max(dist - sc.r, 0.05) / dist
                px, py = CAMERA_X_M + cam_dx * k, cam_dy * k
            msg.obstacles.append(Obstacle2D(x=float(px), y=float(py), r=float(sc.r)))
        self.obs_pub.publish(msg)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--only', nargs='*', default=None)
    ap.add_argument('--true-centre', action='store_true')
    ap.add_argument('--out', default='campaign_scenarios.json')
    args, ros_args = ap.parse_known_args(argv if argv is not None else sys.argv[1:])
    scenarios = [s for s in SCENARIOS if not args.only or s.name in args.only]
    rclpy.init(args=ros_args)
    node = ScenarioNode(scenarios, args.true_centre)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.hold_pub.publish(Bool(data=True))
        results = node.results
        node.destroy_node()
        rclpy.shutdown()
    with open(args.out, 'w') as fh:
        json.dump(results, fh, indent=2)
    print(f'\n{"scenario":28s} {"outcome":9s} {"t[s]":>5s} {"body gap":>8s} '
          f'{"crawl":>5s} {"flips":>5s} {"|d|max":>6s} {"dd/dt":>6s} {"rev":>4s}  blocked')
    for r in results:
        print(f'{r["scenario"]:28s} {r["outcome"]:9s} {r["t"]:5.1f} '
              f'{r["min_body_gap"]:+8.3f} {r["crawl_cmds"]:5d} {r["side_flips"]:5d} '
              f'{r["max_abs_steer"]:6.3f} {r["max_steer_rate"]:6.3f} '
              f'{r["steer_reversals"]:4d}  {r["blocked"] or ""}')
    print(f'wrote {args.out}')


if __name__ == '__main__':
    main()
