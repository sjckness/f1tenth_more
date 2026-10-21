"""
link_manager: is the base reachable, on which path, and how well.

Every period_s it sends one heartbeat round over UDP to the base: to the
base's Tailscale address (tailscale_host) and to its hotspot address
(local_host), same sequence number; the base's heartbeat_responder echoes
them. heartbeat.py has the packet, the path choice (Tailscale first, the
local address as the fallback) and the OK / DEGRADED / LOST rules.

It publishes /comms/link_status (f1tenth_messages/LinkStatus) every period:
state, active path and address, RTT, loss, Wi-Fi RSSI, time since the last
reply, per-path detail, counters, and the reconnect attempts.

Reconnect: NetworkManager owns the Wi-Fi association. While the link is
LOST, link_manager may ask it to bring the hotspot profile up again
(`nmcli connection up`), on a timer, after some distance travelled, or on
entering a coverage cell not mapped yet; rate-limited, one at a time, in a
worker thread. reconnect.py has the triggers and the checks that keep it
from leaving eduroam for a hotspot that is not there.

It never touches the drive path, the e-stop or a mission. The gateway's
"no client for 2 s -> abort" is the gateway's own and does not read this.
"""

import math
import time

from f1tenth_comms.heartbeat import HeartbeatClient, LinkMonitor, STATE_LOST, STATE_NAMES
from f1tenth_comms.reconnect import NmcliReconnector, ReconnectDriver, ReconnectPolicy
from f1tenth_comms.tf_poll import make_tf
from f1tenth_comms.wifi import PROC_NET_WIRELESS, WifiReader
from f1tenth_messages.msg import LinkStatus
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformException

LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
NAN = float('nan')
MAX_SPEED_MPS = 5.0     # odom steps faster than this are jumps (a reset), not travel


def _f(v) -> float:
    return NAN if v is None else float(v)


def grid_cell(grid: OccupancyGrid, x: float, y: float):
    """Return ((ix, iy), mapped) for a map-frame point; ix, iy as the coverage grid counts them."""
    res = grid.info.resolution
    ix, iy = math.floor(x / res), math.floor(y / res)
    gx = ix - round(grid.info.origin.position.x / res)
    gy = iy - round(grid.info.origin.position.y / res)
    if not (0 <= gx < grid.info.width and 0 <= gy < grid.info.height):
        return (ix, iy), False
    return (ix, iy), grid.data[gy * grid.info.width + gx] != -1


class LinkManagerNode(Node):
    def __init__(self, **kwargs):
        super().__init__('link_manager', **kwargs)
        p = self._param
        self.period_s = p('period_s', 0.5)
        port = p('port', 47800)
        paths = [(name, host, port) for name, host in
                 (('tailscale', p('tailscale_host', '100.89.193.76')),
                  ('local', p('local_host', '10.42.50.1'))) if host]
        if not paths:
            raise ValueError('link_manager: tailscale_host and local_host are both empty')
        self.monitor = LinkMonitor(
            [n for n, _, _ in paths], start_t=time.monotonic(),
            timeout_s=p('timeout_s', 2.0), window_s=p('window_s', 10.0),
            degraded_loss_pct=p('degraded_loss_pct', 20.0),
            degraded_rtt_ms=p('degraded_rtt_ms', 250.0),
            degraded_silence_s=p('degraded_silence_s', 1.2),
            failback_s=p('failback_s', 3.0), ok_after_s=p('ok_after_s', 2.0))
        self.client = HeartbeatClient(paths, self.monitor,
                                      resolve_period_s=p('resolve_period_s', 30.0))
        self.wifi = WifiReader(interface=p('wifi_interface', ''),
                               proc_path=p('proc_net_wireless', PROC_NET_WIRELESS),
                               use_iw=p('use_iw_fallback', True))
        self.policy = ReconnectPolicy(
            retry_period_s=p('retry_period_s', 2.5),
            retry_backoff_max_s=p('retry_backoff_max_s', 0.0),
            retry_distance_m=p('retry_distance_m', 3.0),
            retry_on_new_cell=p('retry_on_new_cell', True),
            retry_min_interval_s=p('retry_min_interval_s', 2.0))
        self.reconnector = NmcliReconnector(
            interface=self.wifi.interface, connection=p('hotspot_connection', 'f1tenth-base'),
            ssid=p('hotspot_ssid', ''), wait_s=p('nmcli_wait_s', 15.0),
            skip_if_rssi_above_dbm=p('skip_if_rssi_above_dbm', -65.0),
            require_ssid_in_scan=p('require_ssid_in_scan', True),
            rescan=p('nmcli_rescan', 'auto'), nmcli=p('nmcli', 'nmcli'),
            dry_run=p('reconnect_dry_run', False))
        self.driver = ReconnectDriver(self.policy, self.reconnector.attempt,
                                      enabled=p('reconnect_enabled', True))
        self.map_frame = p('map_frame', 'map')
        self.odom_frame = p('odom_frame', 'odom')
        self.base_frame = p('base_frame', 'base_link')

        # TF only while LOST, the only time anything here needs a pose, and
        # polled (tf_poll.py): no callback per /tf message.
        self.tf_polling = p('tf_polling', True)
        self.tf = None
        self._grid = None           # latest /comms/coverage_rssi
        self._grid_prev = None      # as it was at the previous tick
        self.create_subscription(OccupancyGrid, p('coverage_topic', '/comms/coverage_rssi'),
                                 self._grid_cb, LATCHED)
        self.pub = self.create_publisher(LinkStatus, p('status_topic', '/comms/link_status'),
                                         10)
        self._last_odom = None
        self._last_cell = None
        self._last_tick = None
        self._last_report = None
        self._last_completed = 0
        self._last_result = None
        self.client.start()
        self.create_timer(self.period_s, self._tick)
        targets = ', '.join(
            f'{n} {a}' for n, a in zip(self.monitor.names, self.client.addresses()))
        mode = ('off' if not self.driver.enabled else
                'dry-run' if self.reconnector.dry_run else 'on')
        self.get_logger().info(
            f'link_manager: heartbeat every {self.period_s:g} s to {targets}; '
            f'Wi-Fi {self.wifi.interface or "(none found)"}; reconnect {mode} '
            f'(profile {self.reconnector.connection})')

    def _param(self, name, default):
        return self.declare_parameter(name, default).value

    def _grid_cb(self, msg: OccupancyGrid) -> None:
        self._grid = msg

    def _tf(self, on: bool) -> None:
        if on and self.tf is None:
            self.tf = make_tf(self, polled=self.tf_polling)
        elif not on and self.tf is not None:
            self.tf.close()
            self.tf = None

    def _xy(self, target: str):
        try:
            tf = self.tf.lookup(target, self.base_frame)
        except TransformException:
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    def _track_motion(self, now: float) -> None:
        """While LOST: distance travelled (odom) and entries into unmapped cells (map)."""
        xy = self._xy(self.odom_frame)
        if xy is not None and self._last_odom is not None and self._last_tick is not None:
            step = math.hypot(xy[0] - self._last_odom[0], xy[1] - self._last_odom[1])
            if step <= MAX_SPEED_MPS * max(self.period_s, now - self._last_tick):
                self.policy.add_distance(step)
        self._last_odom = xy
        # Judge the new cell on the grid as it was BEFORE this tick: the coverage
        # node may already have marked it (no link) since the car got there.
        grid = self._grid_prev
        xy = self._xy(self.map_frame)
        if grid is not None and xy is not None:
            cell, mapped = grid_cell(grid, *xy)
            if self._last_cell is not None and cell != self._last_cell and not mapped:
                self.policy.entered_new_cell()
            self._last_cell = cell
        else:
            self._last_cell = None

    def _tick(self) -> None:
        now = time.monotonic()
        view = self.client.evaluate(now)
        self.client.send_round(now)
        wifi = self.wifi.read()
        lost = view.state == STATE_LOST
        self.policy.update_link(now, lost)
        tracking = lost and self.driver.enabled
        self._tf(tracking)
        if tracking:
            self._track_motion(now)
        else:
            self._last_odom = self._last_cell = None
        self._grid_prev = self._grid
        self._last_tick = now
        started = self.driver.tick(now, wifi.rssi_dbm)
        if started:
            self.get_logger().debug(f'reconnect attempt {self.driver.attempts} ({started})')
        self._report_changes(view)
        self.pub.publish(self._status(view, wifi, now))

    def _report_changes(self, view) -> None:
        """Log state and path changes, and reconnect results when they change."""
        a = view.active
        if (view.state, a) != self._last_report:
            self._last_report = (view.state, a)
            where = (f'via {self.monitor.names[a]} {self.client.addresses()[a]}'
                     if a is not None else f'no reply for {view.since_last_reply_s:.1f} s')
            rtt = f', rtt {view.rtt_ms:.0f} ms' if view.rtt_ms is not None else ''
            text = f'link {STATE_NAMES[view.state]} {where}{rtt}'
            # One severity per call site: rclpy refuses to change it between calls.
            if view.state == STATE_LOST:
                self.get_logger().warn(text)
            else:
                self.get_logger().info(text)
        d = self.driver
        if d.completed != self._last_completed:
            self._last_completed = d.completed
            if d.last_result != self._last_result:
                self._last_result = d.last_result
                self.get_logger().info(
                    f'reconnect attempt {d.attempts} ({d.last_trigger}): {d.last_result}')

    def _status(self, view, wifi, now: float) -> LinkStatus:
        m = LinkStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.state = view.state
        m.state_name = view.state_name
        a = view.active
        m.path = self.monitor.names[a] if a is not None else ''
        m.address = self.client.addresses()[a] if a is not None else ''
        m.rtt_ms = _f(view.rtt_ms)
        m.rtt_mean_ms = _f(view.rtt_mean_ms)
        m.loss_pct = _f(view.loss_pct)
        m.since_last_reply_s = float(view.since_last_reply_s)
        m.window_s = float(self.monitor.window_s)
        m.wifi_associated = wifi.associated
        m.rssi_dbm = _f(wifi.rssi_dbm)
        m.wifi_interface = wifi.interface
        m.rounds_sent = view.rounds_sent & 0xFFFFFFFF
        m.rounds_answered = view.rounds_answered & 0xFFFFFFFF
        m.rounds_lost = view.rounds_lost & 0xFFFFFFFF
        m.path_names = [p.name for p in view.paths]
        m.path_addresses = self.client.addresses()
        m.path_up = [p.up for p in view.paths]
        m.path_rtt_ms = [_f(p.rtt_mean_ms) for p in view.paths]
        m.path_loss_pct = [_f(p.loss_pct) for p in view.paths]
        d = self.driver
        m.reconnect_attempts = d.attempts
        m.reconnect_in_flight = d.in_flight
        m.last_reconnect_trigger = d.last_trigger
        m.last_reconnect_result = d.last_result
        m.since_last_reconnect_s = (math.inf if d.last_attempt_t is None
                                    else float(now - d.last_attempt_t))
        return m

    def destroy_node(self):
        self.client.stop()
        self.driver.shutdown()
        self._tf(False)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LinkManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
