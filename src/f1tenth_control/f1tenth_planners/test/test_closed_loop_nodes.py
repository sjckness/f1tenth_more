"""Both planner-controller NODES in closed loop with a kinematic car, goal by goal.

The real node classes, constructed in-process: goals, odometry and obstacles
go in through the node's own callbacks, control_loop() is called once per
control period on a simulated clock, and what it publishes on /drive drives a
kinematic bicycle with a first-order speed lag. Nothing is spun and no
message crosses DDS, so the runs are deterministic and fast; the transport is
the one thing this does not cover (the launch and supervisor tests and a live
bringup do).

Each scenario is a mission move the behaviour tree sends today, with the
outcome the mission's stop conditions read: /mpc/goal_reached for distance and
pose moves, the heading for a turn (orientation_delta), /mpc/object_status's
stop latch for an object approach.
"""

import math
import os

from ackermann_msgs.msg import AckermannDriveStamped
from f1tenth_messages.msg import (
    DriveCommand, ObjectApproachStatus, ObjectGoal, Obstacle2D, Obstacle2DArray, TurnGoal)
from f1tenth_planners.corridor_rrt_node import CorridorRrtNode
from f1tenth_planners.geometry import world_to_robot
from f1tenth_planners.p2p_geometric_node import P2pGeometricNode
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32, String

WHEELBASE = 0.305
SPEED_TAU = 0.15


@pytest.fixture(scope='module', autouse=True)
def ros():
    os.environ.setdefault('ROS_DOMAIN_ID', '87')
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ.pop('ROS_DISCOVERY_SERVER', None)
    rclpy.init()
    yield
    rclpy.shutdown()


class Car:
    """A node under test plus the simulated car it drives."""

    def __init__(self, node_cls, x=0.0, y=0.0, yaw=0.0):
        self.node = node_cls(parameter_overrides=[Parameter('rrt_seed', value=11)]
                             if node_cls is CorridorRrtNode else [])
        self.t = 100.0
        self.node._now = lambda: self.t
        self.x, self.y, self.yaw, self.v = x, y, yaw, 0.0
        self.cmd = (0.0, 0.0)
        self.reached = 0
        self.status = None
        self.obstacles = []          # world (x, y, r)
        self.front_wall_x = None     # a wall across the x axis, for wall_turn
        self.min_clearance = math.inf
        self.trace = []
        self.node.drive_pub.publish = self._on_drive
        self.node.goal_reached_pub.publish = self._on_reached
        self.node.object_status_pub.publish = self._on_status
        self.odom()

    def _on_drive(self, msg: AckermannDriveStamped):
        self.cmd = (msg.drive.speed, msg.drive.steering_angle)

    def _on_reached(self, msg: Bool):
        self.reached += int(msg.data)

    def _on_status(self, msg: ObjectApproachStatus):
        self.status = msg

    def odom(self):
        msg = Odometry()
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.x, self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        msg.twist.twist.linear.x = self.v
        self.node._odom_cb(msg)

    def perceive(self):
        if self.obstacles:
            arr = Obstacle2DArray()
            for ox, oy, r in self.obstacles:
                xr, yr = world_to_robot((self.x, self.y, self.yaw), ox, oy)
                if xr > -0.5 and math.hypot(xr, yr) < 6.0:
                    arr.obstacles.append(Obstacle2D(x=float(xr), y=float(yr), r=float(r)))
            self.node._obstacles_cb(arr)
            for ox, oy, r in self.obstacles:
                self.min_clearance = min(self.min_clearance,
                                         math.hypot(self.x - ox, self.y - oy) - r)
        if self.front_wall_x is not None:
            # bumper distance to a wall x = front_wall_x, straight ahead only
            c = math.cos(self.yaw)
            d = (self.front_wall_x - self.x) / c - 0.33 if c > 0.2 else -1.0
            self.node._front_cb(Float32(data=float(d)))

    def step(self):
        dt = self.node.ts
        self.perceive()
        self.node.control_loop()
        speed, steer = self.cmd
        self.v += (speed - self.v) * (1.0 - math.exp(-dt / SPEED_TAU))
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WHEELBASE * math.tan(steer) * dt
        self.t += dt
        self.odom()
        self.trace.append((self.t, self.x, self.y, self.yaw, speed, steer))

    def run(self, seconds, until=None):
        for _ in range(int(seconds / self.node.ts)):
            self.step()
            if until is not None and until(self):
                return True
        return False

    def close(self):
        self.node.destroy_node()


MODES = [CorridorRrtNode, P2pGeometricNode]


@pytest.fixture(params=MODES, ids=['corridor_rrt', 'p2p_geometric'])
def car(request):
    c = Car(request.param)
    yield c
    c.close()


def _goal_pose(x, y):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.position.x, msg.pose.position.y = x, y
    msg.pose.orientation.w = 1.0
    return msg


class TestBothModes:

    def test_idle_publishes_zero(self, car):
        car.run(1.0)
        assert car.cmd == (0.0, 0.0) and car.reached == 0

    def test_goal_distance_reaches_and_reports(self, car):
        car.node._goal_distance_cb(Float32(data=2.0))
        assert car.run(15.0, until=lambda c: c.reached > 0)
        car.run(1.5)
        assert 1.8 <= car.x <= 2.2, car.x
        assert abs(car.y) < 0.1
        assert car.cmd[0] == 0.0 and abs(car.v) < 0.05
        assert max(t[4] for t in car.trace) <= 0.5 + 1e-9

    def test_goal_pose_off_axis(self, car):
        car.node._goal_pose_cb(_goal_pose(2.5, 1.0))
        assert car.run(20.0, until=lambda c: c.reached > 0)
        assert math.hypot(car.x - 2.5, car.y - 1.0) <= 0.4

    @pytest.mark.parametrize('delta', [90.0, -90.0])
    def test_turn_reaches_the_heading_the_mission_waits_for(self, car, delta):
        car.node._goal_turn_cb(TurnGoal(heading_delta_deg=delta, speed=0.5,
                                        steering='full_lock'))
        target = math.radians(delta)
        # orientation_delta's check, with a 10 deg margin
        assert car.run(12.0, until=lambda c: abs(c.yaw) >= abs(target) - math.radians(10))
        assert math.copysign(1.0, car.yaw) == math.copysign(1.0, target)

    def test_drive_straight_is_open_ended(self, car):
        car.y = 0.15
        car.odom()
        car.node._goal_drive_cb(DriveCommand(mode='straight', turn_sign=0.0, turn_mag_deg=0.0,
                                             speed=0.5, d_safe=-1.0))
        car.run(8.0)
        assert car.reached == 0
        assert car.x > 3.0
        assert abs(car.yaw) < 0.1
        assert car.cmd[0] > 0.0

    def test_wall_turn_turns_at_the_wall(self, car):
        car.front_wall_x = 4.0
        car.node._goal_drive_cb(DriveCommand(mode='wall_turn', turn_sign=1.0, turn_mag_deg=90.0,
                                             speed=0.5, d_safe=-1.0))
        assert car.run(15.0, until=lambda c: c.yaw > math.radians(70))
        assert car.x < 4.0 - 0.33, 'turned before reaching the wall'
        assert car.reached == 0

    def test_hold_stops_and_release_resumes(self, car):
        car.node._goal_distance_cb(Float32(data=3.0))
        car.run(2.0)
        x_hold = car.x
        car.node._hold_cb(Bool(data=True))
        car.run(1.0)
        assert car.cmd[0] == 0.0
        car.node._hold_cb(Bool(data=False))
        assert car.run(15.0, until=lambda c: c.reached > 0)
        assert car.x > x_hold + 0.5

    def test_stale_odometry_stops(self, car):
        car.node._goal_distance_cb(Float32(data=3.0))
        car.run(1.0)
        car.t += 1.0                 # no odometry for a second
        car.node.control_loop()
        assert car.cmd[0] == 0.0

    def test_object_approach_latches_at_the_standoff(self, car):
        target = (3.0, 0.6)
        car.obstacles = [(target[0], target[1], 0.2)]     # the target is on the list

        def refresh(c):
            msg = ObjectGoal()
            msg.header.frame_id = 'map'
            msg.move_id = 'obj-1'
            msg.target_class = 'person'
            msg.point.x, msg.point.y = target
            msg.standoff = 0.8
            msg.speed = 0.45
            c.node._goal_object_cb(msg)
            return c.status is not None and c.status.stop_latched and abs(c.v) < 0.05

        assert car.run(20.0, until=refresh)
        dist = math.hypot(car.x - target[0], car.y - target[1])
        assert 0.8 <= dist <= 0.8 + 0.4, dist
        assert car.status.move_id == 'obj-1'
        car.node._goal_object_end_cb(String(data='obj-1'))
        car.run(0.5)
        assert car.cmd[0] == 0.0


class TestCorridorRrt:

    def test_plans_around_an_obstacle_on_the_line(self):
        car = Car(CorridorRrtNode)
        try:
            car.obstacles = [(2.0, 0.05, 0.2)]
            car.node._goal_distance_cb(Float32(data=4.0))
            assert car.run(25.0, until=lambda c: c.reached > 0)
            assert car.min_clearance >= 0.2, car.min_clearance   # car radius
            assert car.x >= 3.7
        finally:
            car.close()


class TestP2pGeometric:

    def test_stops_short_of_an_obstacle_on_the_path(self):
        car = Car(P2pGeometricNode)
        try:
            car.obstacles = [(2.0, 0.0, 0.2)]
            car.node._goal_distance_cb(Float32(data=4.0))
            car.run(12.0)
            assert car.reached == 0
            assert car.cmd[0] == 0.0 and abs(car.v) < 0.05
            assert car.min_clearance >= 0.2, car.min_clearance
            # cleared: drives on
            car.obstacles = []
            car.node.obstacles_odom = []
            assert car.run(15.0, until=lambda c: c.reached > 0)
        finally:
            car.close()
