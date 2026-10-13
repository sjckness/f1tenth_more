#!/usr/bin/env python3
"""reset_manager: /reset_all puts odometry, pose, map and navigation state back at 0 0 0.

    ros2 service call /reset_all std_srvs/srv/Trigger

Runs as its own supervisor component (`reset_manager`, components.yaml) and
restarts only the components named below, through the supervisor's own
/restart_component. It never restarts the supervisor, and it refuses to touch
any component in `protected_components` (default: intelligence, i.e. the
llama-server whose first prompt is slow).

WHAT THIS STACK OFFERS, AND THEREFORE WHAT EACH STEP DOES
(ROS 2 Humble, real car only; the Gazebo sim cannot run on the Jetson.)

1. stop        Zero AckermannDriveStamped on /safety_stop (ackermann_mux
               priority 200, above everything) at `stop.rate_hz`, held for the
               WHOLE reset; /mission/abort_mission (which also latches
               /mpc/hold, so mpc_corr publishes zeros); Nav2 goal cancel when
               Nav2 is on. Then waits for a standstill on /odometry/filtered.
               There is no /cmd_vel in this stack.
2. wheel_odom  /odom comes from vesc_to_odom_node (vesc_ackermann, our fork),
               which has no reset service and is not ros2_control. The only
               way to zero it without code changes is to restart the
               `hardware` component (VESC driver + odometry; ~15 s including
               the battery pre-check). method `service` calls a std_srvs/
               Trigger instead, for when the node grows one.
3. ekf_local   robot_localization /set_pose on ekf_filter_node (world_frame
               odom, owns odom->base_link). odom0 is fused differentially, so
               the new pose is not pulled back by the old /odom values.
4. slam        slam_toolbox 2.6.10 has no reset service (no Reset.srv in
               Humble), so the `slam` component is restarted: a fresh pose
               graph whose map frame starts at the (now zero) odom pose. The
               same component holds the f1tenth_costmap nodes (semantic
               tracks, cached grid), which are reset with it.
   ekf_global  /ekf_global/set_pose (world_frame map, owns map->odom). Done
               AFTER slam, not with step 3: its pose0 is the ABSOLUTE
               /slam/pose, so resetting it while the old map is still
               publishing would be undone, and the new map's first pose would
               then be rejected against the stale estimate.
5. nav2        clear_entirely_* on both Nav2 costmaps -- only when Nav2 runs
               (enable_nav2, off by default; our own costmaps were reset in 4).
6. hooks       Restart other components holding odom/map-frame state
               (default: wall_distance, odom-frame glass tracks + odometer),
               then call any std_srvs/Trigger reset hooks listed.

Every service call has a timeout. The service runs on a MultiThreadedExecutor
with one ReentrantCallbackGroup, so the calls made from inside the /reset_all
callback are answered on other threads. A second /reset_all while one is
running is refused. A failure stops the sequence and names the step.
"""

import math
import threading
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_localization.srv import SetPose
from std_srvs.srv import Trigger

from f1tenth_messages.srv import RestartComponent


class StepError(Exception):
    """One step failed; the message is what /reset_all returns."""


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class _Latest:
    """Last message on one topic plus the monotonic time it arrived."""

    def __init__(self):
        self.cond = threading.Condition()
        self.msg = None
        self.stamp = None

    def put(self, msg):
        with self.cond:
            self.msg = msg
            self.stamp = time.monotonic()
            self.cond.notify_all()

    def wait(self, predicate, timeout, after=None):
        """First message (arrived after `after`) satisfying predicate, or None."""
        deadline = time.monotonic() + timeout
        with self.cond:
            while True:
                if (self.msg is not None and (after is None or self.stamp > after)
                        and predicate(self.msg)):
                    return self.msg
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self.cond.wait(remaining)


class ResetManager(Node):

    def __init__(self):
        super().__init__('reset_manager')
        self._group = ReentrantCallbackGroup()
        self._busy = threading.Lock()
        self._holding = False

        p = self._param
        self.service_name = p('service_name', '/reset_all')
        self.service_timeout = p('service_timeout_s', 5.0)
        self.restart_service = p('supervisor_restart_service', '/restart_component')
        self.restart_timeout = p('restart_timeout_s', 45.0)
        self.protected = set(p('protected_components', ['intelligence']))
        self.pos_var = p('covariance.position', 1.0e-4)
        self.yaw_var = p('covariance.yaw', 1.0e-4)

        self.stop_enabled = p('stop.enabled', True)
        self.stop_topic = p('stop.topic', '/safety_stop')
        self.stop_rate = p('stop.rate_hz', 20.0)
        self.abort_service = p('stop.abort_mission_service', '/mission/abort_mission')
        self.standstill_topic = p('stop.standstill_topic', '/odometry/filtered')
        self.standstill_speed = p('stop.standstill_speed_mps', 0.05)
        self.standstill_hold = p('stop.standstill_hold_s', 0.5)
        self.stop_timeout = p('stop.timeout_s', 5.0)

        self.odom_enabled = p('wheel_odom.enabled', True)
        self.odom_method = p('wheel_odom.method', 'restart_component')
        self.odom_component = p('wheel_odom.component', 'hardware')
        self.odom_service = p('wheel_odom.service', '')
        self.odom_topic = p('wheel_odom.topic', '/odom')
        self.odom_timeout = p('wheel_odom.timeout_s', 60.0)
        self.odom_tol = p('wheel_odom.tolerance_m', 0.05)

        self.ekf_local_enabled = p('ekf_local.enabled', True)
        self.ekf_local_service = p('ekf_local.service', '/set_pose')
        self.ekf_local_frame = p('ekf_local.frame_id', 'odom')
        self.ekf_local_topic = p('ekf_local.verify_topic', '/odometry/filtered')
        self.ekf_local_tol = p('ekf_local.tolerance_m', 0.05)

        self.slam_enabled = p('slam.enabled', True)
        self.slam_component = p('slam.component', 'slam')
        self.slam_ready_topic = p('slam.ready_topic', '/slam/pose')
        self.slam_timeout = p('slam.timeout_s', 60.0)

        self.ekf_global_enabled = p('ekf_global.enabled', True)
        self.ekf_global_service = p('ekf_global.service', '/ekf_global/set_pose')
        self.ekf_global_frame = p('ekf_global.frame_id', 'map')
        self.ekf_global_topic = p('ekf_global.verify_topic', '/ekf_global/odometry/filtered')
        self.ekf_global_tol = p('ekf_global.tolerance_m', 0.15)

        self.verify_timeout = p('verify_timeout_s', 5.0)
        self.yaw_tol = p('yaw_tolerance_rad', 0.05)

        self.nav2_enabled = p('nav2.enabled', False)
        self.nav2_actions = list(p('nav2.cancel_actions',
                                   ['/navigate_to_pose', '/navigate_through_poses']))
        self.nav2_clear = list(p('nav2.clear_costmap_services', [
            '/global_costmap/clear_entirely_global_costmap',
            '/local_costmap/clear_entirely_local_costmap']))

        self.hook_components = [c for c in p('hooks.restart_components', ['wall_distance']) if c]
        self.hook_services = [s for s in p('hooks.trigger_services', ['']) if s]

        named = {self.odom_component if self.odom_method == 'restart_component' else None,
                 self.slam_component, *self.hook_components} - {None}
        clash = sorted(named & self.protected)
        if clash:
            raise ValueError(f'reset_manager configured to restart protected '
                             f'component(s) {clash}: refusing to start')

        g = self._group
        self._stop_pub = self.create_publisher(AckermannDriveStamped, self.stop_topic, 10)
        self._restart_cli = self.create_client(
            RestartComponent, self.restart_service, callback_group=g)
        self._abort_cli = (self.create_client(Trigger, self.abort_service, callback_group=g)
                           if self.abort_service else None)
        self._ekf_local_cli = self.create_client(SetPose, self.ekf_local_service,
                                                 callback_group=g)
        self._ekf_global_cli = self.create_client(SetPose, self.ekf_global_service,
                                                  callback_group=g)
        self._odom_srv_cli = (self.create_client(Trigger, self.odom_service, callback_group=g)
                              if self.odom_service else None)
        self._hook_clis = {s: self.create_client(Trigger, s, callback_group=g)
                           for s in self.hook_services}
        self._cancel_clis = {}
        self._clear_clis = {}
        if self.nav2_enabled:
            from nav2_msgs.srv import ClearEntireCostmap
            self._cancel_clis = {a: self.create_client(
                CancelGoal, f'{a}/_action/cancel_goal', callback_group=g)
                for a in self.nav2_actions}
            self._clear_clis = {s: self.create_client(ClearEntireCostmap, s, callback_group=g)
                                for s in self.nav2_clear}
            self._clear_type = ClearEntireCostmap

        self._latest = {}
        for topic, msg_type in ((self.standstill_topic, Odometry),
                                (self.odom_topic, Odometry),
                                (self.ekf_local_topic, Odometry),
                                (self.ekf_global_topic, Odometry),
                                (self.slam_ready_topic, PoseWithCovarianceStamped)):
            if topic and topic not in self._latest:
                latest = _Latest()
                self._latest[topic] = latest
                self.create_subscription(msg_type, topic, latest.put,
                                         qos_profile_sensor_data, callback_group=g)

        self.create_timer(1.0 / max(self.stop_rate, 1.0), self._hold_tick, callback_group=g)
        self.create_service(Trigger, self.service_name, self._on_reset_all, callback_group=g)
        self.get_logger().info(
            f'[reset_all] ready on {self.service_name}: stop={self.stop_enabled} '
            f'wheel_odom={self.odom_enabled}({self.odom_method}:'
            f'{self.odom_component if self.odom_method == "restart_component" else self.odom_service}) '
            f'ekf_local={self.ekf_local_enabled} slam={self.slam_enabled}({self.slam_component}) '
            f'ekf_global={self.ekf_global_enabled} nav2={self.nav2_enabled} '
            f'hooks={self.hook_components + self.hook_services}; never restarts '
            f'{sorted(self.protected)}')

    def _param(self, name, default):
        return self.declare_parameter(name, default).value

    # -- plumbing ----------------------------------------------------------

    def _call(self, client, request, timeout, what):
        """Response of one service call, or StepError naming what timed out."""
        if not client.wait_for_service(timeout_sec=min(timeout, self.service_timeout)):
            raise StepError(f'{what}: service {client.srv_name} is not available')
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            client.remove_pending_request(future)
            raise StepError(f'{what}: no response from {client.srv_name} within {timeout:.0f} s')
        if future.exception() is not None:
            raise StepError(f'{what}: {client.srv_name} raised {future.exception()}')
        return future.result()

    def _restart(self, component, what):
        if component in self.protected:
            raise StepError(f'{what}: {component!r} is protected and is never restarted')
        request = RestartComponent.Request()
        request.component_name = component
        self.get_logger().info(f'[reset_all] {what}: restarting component {component!r}')
        response = self._call(self._restart_cli, request, self.restart_timeout, what)
        if not response.success:
            raise StepError(f'{what}: supervisor refused to restart {component!r}: '
                            f'{response.message}')
        return time.monotonic()

    def _set_pose(self, client, frame_id, what):
        request = SetPose.Request()
        request.pose.header.frame_id = frame_id
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        cov[0] = cov[7] = cov[14] = self.pos_var
        cov[21] = cov[28] = cov[35] = self.yaw_var
        request.pose.pose.covariance = cov
        self._call(client, request, self.service_timeout, what)
        return time.monotonic()

    def _near_zero(self, tol):
        def check(msg):
            pose = msg.pose.pose
            return (math.hypot(pose.position.x, pose.position.y) <= tol
                    and abs(_yaw(pose.orientation)) <= self.yaw_tol)
        return check

    def _wait_pose(self, topic, tol, timeout, after, what):
        msg = self._latest[topic].wait(self._near_zero(tol), timeout, after)
        if msg is None:
            last = self._latest[topic].msg
            seen = ('nothing received' if last is None else
                    f'last ({last.pose.pose.position.x:+.3f}, '
                    f'{last.pose.pose.position.y:+.3f}, yaw {_yaw(last.pose.pose.orientation):+.3f})')
            raise StepError(f'{what}: {topic} did not read 0 0 0 (+-{tol} m) within '
                            f'{timeout:.0f} s: {seen}')
        return msg

    def _hold_tick(self):
        if self._holding:
            self._publish_stop()

    def _publish_stop(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        self._stop_pub.publish(msg)     # speed 0, steering 0

    # -- the steps ---------------------------------------------------------

    def _step_stop(self):
        self._holding = True
        self._publish_stop()
        notes = [f'holding zero on {self.stop_topic}']
        if self._abort_cli is not None:
            result = self._call(self._abort_cli, Trigger.Request(), self.service_timeout,
                                'stop (abort mission)')
            # success=false is the benign "nothing to abort" answer
            notes.append(f'abort_mission: {result.message or result.success}')
        for action, client in self._cancel_clis.items():
            self._call(client, CancelGoal.Request(), self.service_timeout,
                       f'stop (cancel {action})')   # zero goal id + stamp = cancel all
            notes.append(f'cancelled {action}')

        # Standstill = every fresh sample at or below standstill_speed for
        # standstill_hold seconds in a row.
        latest = self._latest[self.standstill_topic]
        deadline = time.monotonic() + self.stop_timeout
        still_since, seen, speed = None, time.monotonic() - 0.5, None
        while True:
            msg = latest.wait(lambda m: True, 0.2, after=seen)
            now = time.monotonic()
            if msg is not None:
                seen = latest.stamp
                speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
                if speed <= self.standstill_speed:
                    still_since = still_since or now
                    if now - still_since >= self.standstill_hold:
                        notes.append(f'standstill ({speed:.3f} m/s)')
                        return '; '.join(notes)
                else:
                    still_since = None
            if now > deadline:
                what = ('no fresh message' if speed is None or now - seen > 0.5
                        else f'last speed {speed:.2f} m/s')
                raise StepError(f'stop: no standstill on {self.standstill_topic} within '
                                f'{self.stop_timeout:.0f} s ({what})')

    def _step_wheel_odom(self):
        if self.odom_method == 'service':
            if self._odom_srv_cli is None:
                raise StepError('wheel_odom: method is service but wheel_odom.service is empty')
            result = self._call(self._odom_srv_cli, Trigger.Request(), self.service_timeout,
                                'wheel_odom')
            if not result.success:
                raise StepError(f'wheel_odom: {self.odom_service} failed: {result.message}')
            since = time.monotonic()
        elif self.odom_method == 'restart_component':
            since = self._restart(self.odom_component, 'wheel_odom')
        else:
            raise StepError(f'wheel_odom: unknown method {self.odom_method!r}')
        msg = self._wait_pose(self.odom_topic, self.odom_tol, self.odom_timeout, since,
                              'wheel_odom')
        p = msg.pose.pose.position
        return f'{self.odom_topic} back at ({p.x:+.3f}, {p.y:+.3f})'

    def _step_ekf_local(self):
        since = self._set_pose(self._ekf_local_cli, self.ekf_local_frame, 'ekf_local')
        msg = self._wait_pose(self.ekf_local_topic, self.ekf_local_tol, self.verify_timeout,
                              since, 'ekf_local')
        p = msg.pose.pose.position
        return f'{self.ekf_local_topic} at ({p.x:+.3f}, {p.y:+.3f})'

    def _step_slam(self):
        since = self._restart(self.slam_component, 'slam')
        msg = self._latest[self.slam_ready_topic].wait(lambda m: True, self.slam_timeout, since)
        if msg is None:
            raise StepError(f'slam: no {self.slam_ready_topic} within {self.slam_timeout:.0f} s '
                            f'of restarting {self.slam_component!r}')
        p = msg.pose.pose.position
        return f'new map, first {self.slam_ready_topic} ({p.x:+.3f}, {p.y:+.3f})'

    def _step_ekf_global(self):
        since = self._set_pose(self._ekf_global_cli, self.ekf_global_frame, 'ekf_global')
        msg = self._wait_pose(self.ekf_global_topic, self.ekf_global_tol, self.verify_timeout,
                              since, 'ekf_global')
        p = msg.pose.pose.position
        return f'{self.ekf_global_topic} at ({p.x:+.3f}, {p.y:+.3f})'

    def _step_nav2(self):
        for service, client in self._clear_clis.items():
            self._call(client, self._clear_type.Request(), self.service_timeout,
                       f'nav2 ({service})')
        return f'cleared {len(self._clear_clis)} costmap(s)'

    def _step_hooks(self):
        done = []
        for component in self.hook_components:
            self._restart(component, 'hooks')
            done.append(f'restarted {component}')
        for service, client in self._hook_clis.items():
            result = self._call(client, Trigger.Request(), self.service_timeout, 'hooks')
            if not result.success:
                raise StepError(f'hooks: {service} failed: {result.message}')
            done.append(f'called {service}')
        return ', '.join(done) or 'nothing configured'

    # -- the service -------------------------------------------------------

    def _on_reset_all(self, request, response):
        if not self._busy.acquire(blocking=False):
            response.success = False
            response.message = 'a reset is already running; this call was refused'
            self.get_logger().warn(f'[reset_all] {response.message}')
            return response
        steps = [
            ('stop', self.stop_enabled, self._step_stop, 'disabled'),
            ('wheel_odom', self.odom_enabled, self._step_wheel_odom, 'disabled'),
            ('ekf_local', self.ekf_local_enabled, self._step_ekf_local, 'disabled'),
            ('slam', self.slam_enabled, self._step_slam, 'disabled'),
            ('ekf_global', self.ekf_global_enabled, self._step_ekf_global, 'disabled'),
            ('nav2', self.nav2_enabled, self._step_nav2,
             'Nav2 not running (enable_nav2 false); f1tenth_costmap was reset with slam'),
            ('hooks', True, self._step_hooks, ''),
        ]
        started = time.monotonic()
        report = []
        try:
            self.get_logger().info('[reset_all] requested')
            for index, (name, enabled, run, why_skipped) in enumerate(steps, 1):
                label = f'[reset_all] {index}/{len(steps)} {name}'
                if not enabled:
                    self.get_logger().info(f'{label}: skipped ({why_skipped})')
                    report.append(f'{name} skipped')
                    continue
                t0 = time.monotonic()
                self.get_logger().info(f'{label}: start')
                try:
                    detail = run()
                except StepError as exc:
                    raise StepError(f"step '{name}' failed after "
                                    f'{time.monotonic() - t0:.1f} s: {exc}') from None
                except Exception as exc:  # noqa: BLE001 - reported, never swallowed
                    raise StepError(f"step '{name}' failed: {type(exc).__name__}: {exc}") \
                        from None
                took = time.monotonic() - t0
                self.get_logger().info(f'{label}: done in {took:.1f} s -- {detail}')
                report.append(f'{name} {took:.1f}s')
            response.success = True
            response.message = (f'reset done in {time.monotonic() - started:.1f} s: '
                                + ', '.join(report))
            self.get_logger().info(f'[reset_all] {response.message}')
        except StepError as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f'[reset_all] FAILED: {response.message}')
        finally:
            self._holding = False
            self._busy.release()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ResetManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
