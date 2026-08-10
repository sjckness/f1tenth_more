"""System-wide hardware/resource observer: CPU/RAM via psutil, Jetson GPU/EMC/temps
via jtop (jetson-stats).

Publishes f1tenth_messages/SystemStatus on /diagnostics/system_status at
publish_rate_hz (default 1.0 Hz -- a slow-changing diagnostic feed, not a control-loop
signal).

The jtop connection is opened once at startup and kept alive for the node's lifetime
(connection setup has non-trivial overhead -- see jetson-stats' own docs), not
reopened per publish cycle. If jtop can't connect (not installed, jtop.service down, or
the running user isn't in the `jtop` group -- the most common cause: `sudo usermod -aG
jtop $USER` then log out/in) this is logged loudly once at startup, never silently
faked -- cpu_temp_c falls back to reading /sys/devices/virtual/thermal/thermal_zone*/
temp directly, and gpu_percent/gpu_temp_c/emc_percent are published as 0.0 (not
available without jtop).
"""

import glob

import psutil

import rclpy
from rclpy.node import Node

from f1tenth_messages.msg import SystemStatus

try:
    from jtop import jtop, JtopException
    _JTOP_IMPORT_ERROR = None
except ImportError as exc:
    jtop = None
    JtopException = Exception
    _JTOP_IMPORT_ERROR = exc


class SystemObserverNode(Node):

    def __init__(self):
        super().__init__('system_observer_node')

        self.publish_rate_hz = float(self.declare_parameter('publish_rate_hz', 1.0).value)

        self._pub = self.create_publisher(SystemStatus, '/diagnostics/system_status', 10)

        # Prime psutil's non-blocking cpu_percent measurement: the first call after
        # import always returns a meaningless baseline (0.0), every call after this one
        # measures against the previous call instead of blocking for `interval` seconds.
        psutil.cpu_percent(interval=None)
        psutil.cpu_percent(interval=None, percpu=True)

        self._jetson = None
        self._setup_jtop()

        period = 1.0 / max(self.publish_rate_hz, 1e-3)
        self.create_timer(period, self._publish_tick)

        self.get_logger().info(
            f'[system_observer] Publishing SystemStatus on /diagnostics/system_status '
            f'at {self.publish_rate_hz:.2f} Hz (jtop: '
            f'{"connected" if self._jetson is not None else "UNAVAILABLE, see warning above"}).'
        )

    def _setup_jtop(self):
        if jtop is None:
            self.get_logger().error(
                f'[system_observer] jetson-stats not installed ({_JTOP_IMPORT_ERROR}). '
                f'gpu_percent/gpu_temp_c/emc_percent will be published as 0.0. Fix: '
                f'`sudo pip3 install jetson-stats`, then reboot (or restart jtop.service).'
            )
            return
        try:
            self._jetson = jtop()
            self._jetson.start()
        except JtopException as exc:
            self._jetson = None
            self.get_logger().error(
                f'[system_observer] jtop connection failed: {exc}. gpu_percent/'
                f'gpu_temp_c/emc_percent will be published as 0.0, cpu_temp_c falls back '
                f"to /sys/devices/virtual/thermal. Common cause: the running user isn't "
                f'in the `jtop` group -- `sudo usermod -aG jtop $USER`, then log out/in '
                f'(or reboot) so group membership takes effect, then restart this node.'
            )

    def destroy_node(self):
        if self._jetson is not None:
            try:
                self._jetson.close()
            except Exception:
                pass
        return super().destroy_node()

    # -- fallback CPU temp reading (no jtop) ---------------------------------

    def _read_thermal_zone_cpu_temp(self) -> float:
        """First thermal_zone whose `type` contains "cpu" (case-insensitive), else
        thermal_zone0. Zone naming/count varies by board/L4T revision -- this is only
        the no-jtop fallback; jtop's own `temperature` dict is preferred when available.
        """
        best_match = None
        first_zone = None
        for type_path in sorted(glob.glob('/sys/devices/virtual/thermal/thermal_zone*/type')):
            zone_dir = type_path.rsplit('/', 1)[0]
            try:
                with open(type_path) as f:
                    zone_type = f.read().strip()
                with open(zone_dir + '/temp') as f:
                    milli_c = int(f.read().strip())
            except (OSError, ValueError):
                continue
            temp_c = milli_c / 1000.0
            if first_zone is None:
                first_zone = temp_c
            if 'cpu' in zone_type.lower():
                best_match = temp_c
                break
        if best_match is not None:
            return best_match
        return first_zone if first_zone is not None else 0.0

    @staticmethod
    def _pick_temp(temps: dict, keyword: str) -> float:
        # jtop's temperature rail names vary by board/L4T revision (e.g. 'cpu', 'CPU',
        # 'tj') -- matched by substring rather than an exact expected key. A rail can
        # be present but offline (confirmed on this board: 'gpu' exists with
        # temp=-256, online=False -- jtop's documented sentinel for "no sensor"), so
        # online=False is treated the same as "not found": 0.0, not the sentinel.
        for name, data in temps.items():
            if keyword in name.lower() and data.get('online', False):
                return data.get('temp', 0.0)
        return 0.0

    # -- publish tick ---------------------------------------------------------

    def _publish_tick(self):
        msg = SystemStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.cpu_percent = float(psutil.cpu_percent(interval=None))
        msg.cpu_per_core = [float(p) for p in psutil.cpu_percent(interval=None, percpu=True)]

        vm = psutil.virtual_memory()
        msg.ram_used_mb = float(vm.used) / (1024.0 * 1024.0)
        msg.ram_total_mb = float(vm.total) / (1024.0 * 1024.0)

        if self._jetson is not None:
            try:
                temps = self._jetson.temperature
                msg.cpu_temp_c = float(self._pick_temp(temps, 'cpu'))
                msg.gpu_temp_c = float(self._pick_temp(temps, 'gpu'))

                gpus = self._jetson.gpu
                first_gpu = next(iter(gpus.values()), None)
                msg.gpu_percent = float(first_gpu['status']['load']) if first_gpu else 0.0

                memory = self._jetson.memory
                msg.emc_percent = float(memory['EMC']['val']) if 'EMC' in memory else 0.0
            except Exception as exc:
                self.get_logger().warn(f'[system_observer] jtop read failed this tick: {exc}')
                msg.cpu_temp_c = self._read_thermal_zone_cpu_temp()
        else:
            msg.cpu_temp_c = self._read_thermal_zone_cpu_temp()
            # gpu_percent, gpu_temp_c, emc_percent left at their msg default (0.0) --
            # not available without jtop, see _setup_jtop's logged reason.

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SystemObserverNode()
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
