"""Generic launch-sequencing readiness gate.

Polls a std_srvs/Trigger-typed service by name until it returns success=true,
then exits 0; exits 1 on timeout. service_name is a required parameter -- this
node has no Nav2-specific logic, so it's reusable for any other Trigger-style
readiness service in this stack, not just nav2_lifecycle_manager's own
<manager_name>/is_active (which is what f1tenth_behavior/launch/
behavior_bringup.launch.py uses it for today, to wait for
lifecycle_manager_navigation to report all managed Nav2 nodes -- map_server,
controller_server, planner_server, behavior_server, bt_navigator -- ACTIVE
before starting behavior_executor_node/twist_to_ackermann_node).

Meant to be launch-event-gated: a RegisterEventHandler(OnProcessExit(...))
elsewhere inspects this node's exit code (event.returncode) to decide whether
to launch whatever depends on the service actually being ready -- same idiom
already used for vesc.launch.py's calibration/battery-check sequencing --
rather than a fixed-duration TimerAction guess.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class WaitForTriggerServiceNode(Node):

    def __init__(self):
        super().__init__('wait_for_trigger_service_node')

        self.service_name = str(self.declare_parameter('service_name', '').value)
        self.poll_interval_sec = float(
            self.declare_parameter('poll_interval_sec', 0.5).value)
        self.timeout_sec = float(self.declare_parameter('timeout_sec', 60.0).value)

        self.exit_code = None

        if not self.service_name:
            self.get_logger().error(
                'service_name parameter is required (empty) -- nothing to wait for.')
            self.exit_code = 1
            return

        self._client = self.create_client(Trigger, self.service_name)
        self._start_time = time.monotonic()
        self._pending_future = None

        self.get_logger().info(
            f'Waiting for "{self.service_name}" (std_srvs/Trigger) to report '
            f'success=true, timeout={self.timeout_sec:.1f}s.')

        self._timer = self.create_timer(self.poll_interval_sec, self._poll)

    def _poll(self):
        elapsed = time.monotonic() - self._start_time
        if elapsed >= self.timeout_sec:
            self._timer.cancel()
            self.get_logger().error(
                f'STARTUP ABORTED: "{self.service_name}" did not report success=true '
                f'within {self.timeout_sec:.1f}s.')
            self.exit_code = 1
            return

        if self._pending_future is not None:
            return  # previous call still outstanding, don't stack calls

        if not self._client.service_is_ready():
            return  # not advertised yet -- try again next poll

        self._pending_future = self._client.call_async(Trigger.Request())
        self._pending_future.add_done_callback(self._on_response)

    def _on_response(self, future):
        self._pending_future = None
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - keep polling on a transient RPC failure
            self.get_logger().warning(f'Trigger call to "{self.service_name}" failed: {exc}')
            return
        if response.success:
            self._timer.cancel()
            self.get_logger().info(
                f'"{self.service_name}" reports success=true -- proceeding.')
            self.exit_code = 0


def main():
    rclpy.init()
    node = WaitForTriggerServiceNode()
    try:
        # NOT rclpy.spin(node): a callback calling rclpy.shutdown() on itself while
        # running under the executor deadlocks (executor.shutdown() waits for the
        # very callback that's calling it to finish -- confirmed live, see the
        # f1tenth_behavior readiness-gate task). Callbacks only set self.exit_code;
        # shutdown happens here, in the main thread, once the loop notices it's set.
        while rclpy.ok() and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        exit_code = node.exit_code if node.exit_code is not None else 1
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
