"""
wifi_coverage_node: a session map of Wi-Fi signal and link quality to the base.

Every sample_period_s (1 s) it takes one sample and bins it into the cell of
the map frame the car is in (coverage.py has the cell, the statistics and
the encoding):
- RSSI and association of the Wi-Fi interface (wifi.py: /proc/net/wireless,
  `iw` as a fallback; no root);
- RTT, heartbeat rounds answered/lost and LOST from the latest
  /comms/link_status. It never sends packets of its own;
- the pose from TF map_frame -> base_frame. With that transform missing, or
  older than tf_max_age_s, the sample is skipped (counted, not binned). TF
  is polled once per sample, not taken per message (tf_poll.py: a /tf
  callback per message would cost more than everything else here).

Outputs, latched (transient_local, depth 1), at most once per
publish_period_s (never faster than 1 Hz) and only when they changed:
- /comms/coverage_rssi, /comms/coverage_rtt: nav_msgs/OccupancyGrid in
  map_frame. -1 never visited / no data, 0..100 good..bad, -128 no link;
  the scaling is in coverage.py's docstring and the params below;
- /comms/coverage_cells: std_msgs/String, JSON, every cell's numbers (mean
  and min RSSI, RTT p95, loss %, samples, no-link samples, last update).

/comms/coverage_reset (std_srvs/Trigger) empties the grid. Nothing is saved:
the grid lives as long as the node.

Reports only. Nothing in the stack reads these topics to drive or stop.
"""

import json
import math
import time

from f1tenth_comms.coverage import CoverageGrid, LinkDeltas, Sample, Scale
from f1tenth_comms.tf_poll import make_tf
from f1tenth_comms.wifi import PROC_NET_WIRELESS, WifiReader
from f1tenth_messages.msg import LinkStatus
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformException

LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def finite_or_none(v):
    v = float(v)
    return v if math.isfinite(v) else None


class WifiCoverageNode(Node):
    def __init__(self, **kwargs):
        super().__init__('wifi_coverage_node', **kwargs)
        p = self._param
        self.map_frame = p('map_frame', 'map')
        self.base_frame = p('base_frame', 'base_link')
        self.sample_period_s = p('sample_period_s', 1.0)
        self.tf_max_age_s = p('tf_max_age_s', 1.0)
        self.link_status_timeout_s = p('link_status_timeout_s', 3.0)
        self.publish_period_s = max(1.0, p('publish_period_s', 1.0))
        self.cells_period_s = p('cells_publish_period_s', 5.0)
        self.rssi_z_m = p('rssi_z_m', 0.05)
        self.rtt_z_m = p('rtt_z_m', 0.10)
        self.scale = Scale(rssi_good_dbm=p('rssi_good_dbm', -40.0),
                           rssi_bad_dbm=p('rssi_bad_dbm', -90.0),
                           rtt_bad_ms=p('rtt_bad_ms', 200.0),
                           no_link_fraction=p('no_link_fraction', 0.5))
        self.grid = CoverageGrid(resolution=p('resolution_m', 1.0),
                                 rtt_keep=p('rtt_samples_per_cell', 200),
                                 max_extent_m=p('max_extent_m', 200.0))
        self.wifi = WifiReader(interface=p('wifi_interface', ''),
                               proc_path=p('proc_net_wireless', PROC_NET_WIRELESS),
                               use_iw=p('use_iw_fallback', True))

        self.tf = make_tf(self, polled=p('tf_polling', True))
        self.create_subscription(LinkStatus, p('link_status_topic', '/comms/link_status'),
                                 self._link_cb, 10)
        self.rssi_pub = self.create_publisher(
            OccupancyGrid, p('rssi_topic', '/comms/coverage_rssi'), LATCHED)
        self.rtt_pub = self.create_publisher(
            OccupancyGrid, p('rtt_topic', '/comms/coverage_rtt'), LATCHED)
        self.cells_pub = self.create_publisher(
            String, p('cells_topic', '/comms/coverage_cells'), LATCHED)
        self.create_service(Trigger, p('reset_service', '/comms/coverage_reset'), self._reset_cb)

        self._link = None
        self._link_rx = -math.inf          # monotonic time of the last link_status
        self._deltas = LinkDeltas()
        self.counts = {'samples': 0, 'tf_missing': 0, 'tf_stale': 0}
        self._last_key = {'rssi': None, 'rtt': None}
        self._grid_version = self._cells_version = -1
        self._last_pub_t = self._last_cells_t = -math.inf
        self.create_timer(self.sample_period_s, self._tick)
        self.get_logger().info(
            f'wifi_coverage_node: interface {self.wifi.interface or "(none found)"}, '
            f'cells {self.grid.resolution:g} m in {self.map_frame}, sample every '
            f'{self.sample_period_s:g} s')

    def _param(self, name, default):
        return self.declare_parameter(name, default).value

    # ---- inputs ----------------------------------------------------------------------

    def _link_cb(self, msg: LinkStatus) -> None:
        self._link = msg
        self._link_rx = time.monotonic()

    def _pose(self):
        """Return ((x, y), None), or (None, 'tf_missing' | 'tf_stale')."""
        try:
            tf = self.tf.lookup(self.map_frame, self.base_frame)
        except TransformException:
            return None, 'tf_missing'
        stamp = Time.from_msg(tf.header.stamp)
        # A zero stamp is a chain of static transforms: always current.
        if stamp.nanoseconds > 0:
            age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
            if age > self.tf_max_age_s:
                return None, 'tf_stale'
        t = tf.transform.translation
        return (t.x, t.y), None

    def _sample(self, xy, now_s: float) -> Sample:
        wifi = self.wifi.read()
        s = Sample(x=xy[0], y=xy[1], t=now_s, associated=wifi.associated,
                   rssi_dbm=wifi.rssi_dbm)
        link = self._link
        if link is None or time.monotonic() - self._link_rx > self.link_status_timeout_s:
            self._deltas.forget()
            return s
        s.link_known = True
        s.link_lost = link.state == LinkStatus.STATE_LOST
        s.rtt_ms = finite_or_none(link.rtt_ms)
        s.answered, s.lost = self._deltas.update(int(link.rounds_answered),
                                                 int(link.rounds_lost))
        return s

    def _tick(self) -> None:
        xy, why = self._pose()
        if xy is None:
            self.counts[why] += 1
            self.get_logger().warn(
                f'no sample: {self.map_frame} -> {self.base_frame} '
                f'{"missing" if why == "tf_missing" else "older than %g s" % self.tf_max_age_s}',
                throttle_duration_sec=30.0)
        else:
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if self.grid.add(self._sample(xy, now_s)):
                self.counts['samples'] += 1
        self._maybe_publish()

    # ---- outputs ---------------------------------------------------------------------

    def _grid_msg(self, img, z: float, stamp) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.info.map_load_time = stamp
        msg.info.resolution = float(img.resolution)
        msg.info.width = img.width
        msg.info.height = img.height
        msg.info.origin.position.x = float(img.origin_x)
        msg.info.origin.position.y = float(img.origin_y)
        msg.info.origin.position.z = float(z)
        msg.info.origin.orientation.w = 1.0
        msg.data = img.data
        return msg

    def _maybe_publish(self, force: bool = False) -> None:
        now = time.monotonic()
        version = self.grid.version
        # 5 % slack for timer jitter: the sample timer and this period are both 1 s.
        if not force and (now - self._last_pub_t < 0.95 * self.publish_period_s or
                          (version == self._grid_version and version == self._cells_version)):
            return
        stamp = self.get_clock().now().to_msg()
        changed = False
        for layer, pub, z in (('rssi', self.rssi_pub, self.rssi_z_m),
                              ('rtt', self.rtt_pub, self.rtt_z_m)):
            img = self.grid.encode(layer, self.scale)
            key = img.key()
            if force or key != self._last_key[layer]:
                pub.publish(self._grid_msg(img, z, stamp))
                self._last_key[layer] = key
                changed = True
        self._grid_version = version
        if changed:
            self._last_pub_t = now
        if force or (version != self._cells_version and
                     (changed or now - self._last_cells_t >= self.cells_period_s)):
            self.cells_pub.publish(String(data=self.cells_json()))
            self._cells_version = version
            self._last_cells_t = now
            self._last_pub_t = now

    def cells_json(self) -> str:
        s = self.scale
        return json.dumps({
            'frame_id': self.map_frame, 'resolution_m': self.grid.resolution,
            'stamp': round(self.get_clock().now().nanoseconds * 1e-9, 3),
            'interface': self.wifi.interface,
            'scale': {'rssi_good_dbm': s.rssi_good_dbm, 'rssi_bad_dbm': s.rssi_bad_dbm,
                      'rtt_bad_ms': s.rtt_bad_ms, 'no_link_fraction': s.no_link_fraction},
            'counts': dict(self.counts, rejected_extent=self.grid.rejected),
            'cells': self.grid.cells_report(self.scale),
        }, separators=(',', ':'))

    def _reset_cb(self, request, response):
        n = self.grid.reset()
        self._maybe_publish(force=True)
        response.success = True
        response.message = f'cleared {n} cells'
        self.get_logger().info(f'/comms/coverage_reset: {response.message}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WifiCoverageNode()
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
