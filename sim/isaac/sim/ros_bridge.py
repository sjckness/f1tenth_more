"""ROS 2 interface (rclpy from the bridge's internal Humble libs).

Publishers are driven from the simulation loop with **sim-time** stamps; the subscriber and the
reset service are serviced by ``spin_once`` between physics steps. Topic names, frame ids and QoS
come from ``ros_interface.yaml``. Python publishers (instead of OmniGraph helpers) are used so the
encoding (bgra8 like the real ZED), the depth conventions, noise and the 40 Hz scan rate can match
the real car exactly.
"""
from __future__ import annotations

import array
import logging
import math
from typing import Callable

import numpy as np

from sim.config import SimConfig
from sim.transforms import matrix_to_quat

log = logging.getLogger(__name__)


def _qos(spec: dict):
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=int(spec.get("depth", 10)),
        reliability=ReliabilityPolicy.BEST_EFFORT if spec.get("reliability") == "best_effort"
        else ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL if spec.get("durability") == "transient_local"
        else DurabilityPolicy.VOLATILE)


def _stamp(t: float):
    from builtin_interfaces.msg import Time

    sec = int(math.floor(t))
    return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1_000_000_000)


def _u8(buf: np.ndarray) -> array.array:
    a = array.array("B")
    a.frombytes(np.ascontiguousarray(buf).tobytes())
    return a


class RosBridge:
    def __init__(self, cfg: SimConfig, on_command: Callable[[float, float], None],
                 on_reset: Callable[[], tuple[bool, str]]) -> None:
        # The bridge's rosidl_generator_py logs a DEBUG traceback (No module named 'lark') on import: its code
        # generator needs lark, message classes at run time do not. Harmless; kept out of the log.
        logging.getLogger("rosidl_generator_py").setLevel(logging.WARNING)
        import rclpy
        from ackermann_msgs.msg import AckermannDriveStamped
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
        from std_srvs.srv import Trigger
        from tf2_msgs.msg import TFMessage

        self.cfg = cfg
        rc = cfg.ros
        if not rclpy.ok():
            try:   # keep SIGTERM/SIGINT for the sim's own clean shutdown (rclpy would kill the context first)
                from rclpy.signals import SignalHandlerOptions

                rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            except (ImportError, TypeError):
                rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node(rc["ros"]["node_name"])
        t, q = rc["topics"], rc["qos"]
        self.t, self.fr = t, rc["frames"]
        sensor_q = _qos(q["sensor"])
        self.pub = {
            "clock": self.node.create_publisher(Clock, t["clock"], _qos(q["clock"])),
            "scan": self.node.create_publisher(LaserScan, t["scan"], sensor_q),
            "odom": self.node.create_publisher(Odometry, t["odom"], _qos(q["odom"])),
            "gt_odom": self.node.create_publisher(Odometry, t["gt_odom"], _qos(q["odom"])),
            "gt_person": self.node.create_publisher(PoseStamped, t["gt_person"], _qos(q["odom"])),
            "imu": self.node.create_publisher(Imu, t["imu"], sensor_q),
            "left_image": self.node.create_publisher(Image, t["left_image"], sensor_q),
            "left_info": self.node.create_publisher(CameraInfo, t["left_info"], sensor_q),
            "depth": self.node.create_publisher(Image, t["depth"], sensor_q),
            "depth_info": self.node.create_publisher(CameraInfo, t["depth_info"], sensor_q),
            "camera_image": self.node.create_publisher(Image, t["camera_image"], sensor_q),
            "camera_info": self.node.create_publisher(CameraInfo, t["camera_info"], sensor_q),
            "tf": self.node.create_publisher(TFMessage, t["tf"], _qos(q["tf"])),
            "tf_static": self.node.create_publisher(TFMessage, t["tf_static"], _qos(q["tf_static"])),
        }
        st = cfg.sensors["stereo"]
        if st["right_camera"]:
            self.pub["right_image"] = self.node.create_publisher(Image, t["right_image"], sensor_q)
            self.pub["right_info"] = self.node.create_publisher(CameraInfo, t["right_info"], sensor_q)
        if st["semantic"]["enabled"]:
            self.pub["gt_semantic"] = self.node.create_publisher(Image, t["gt_semantic"], sensor_q)
        if st["depth_noise"]["enabled"] and st["depth_noise"]["publish_confidence"]:
            self.pub["confidence"] = self.node.create_publisher(Image, t["confidence"], sensor_q)
        self._on_command = on_command
        self._on_reset = on_reset
        if t.get("gt_front_range"):
            from sensor_msgs.msg import Range

            self.msgs_range = Range
            self.pub["gt_front_range"] = self.node.create_publisher(Range, t["gt_front_range"], _qos(q["odom"]))
        if t.get("gt_objects"):
            from vision_msgs.msg import Detection3DArray

            self.msgs_objects = Detection3DArray
            self.pub["gt_objects"] = self.node.create_publisher(Detection3DArray, t["gt_objects"],
                                                                _qos(q.get("objects", q["odom"])))
        if t.get("gt_start_pose"):   # latched: published once, late joiners (scripts/sim/restart.sh) get it
            self.pub["gt_start_pose"] = self.node.create_publisher(PoseStamped, t["gt_start_pose"],
                                                                   _qos(q.get("objects", q["odom"])))
        self.sub = self.node.create_subscription(AckermannDriveStamped, t["command"], self._cmd_cb, _qos(q["command"]))
        self.srv = self.node.create_service(Trigger, t["reset_service"], self._reset_cb)
        self.msgs = dict(Clock=Clock, LaserScan=LaserScan, Odometry=Odometry, PoseStamped=PoseStamped, Imu=Imu,
                         Image=Image, CameraInfo=CameraInfo, TFMessage=TFMessage)
        self.n_cmd = 0
        log.info("ROS node '%s' up: domain %s, command topic %s", rc["ros"]["node_name"],
                 rc["ros"]["domain_id"], t["command"])

    # ------------------------------------------------------------ callbacks
    def _cmd_cb(self, msg) -> None:
        self.n_cmd += 1
        self._on_command(float(msg.drive.speed), float(msg.drive.steering_angle))

    def _reset_cb(self, _req, resp):
        ok, message = self._on_reset()
        resp.success, resp.message = bool(ok), str(message)
        return resp

    def spin_once(self) -> None:
        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def shutdown(self) -> None:
        try:
            self.node.destroy_node()
            self.rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass

    # ------------------------------------------------------------ publishers
    def clock(self, t: float) -> None:
        m = self.msgs["Clock"]()
        m.clock = _stamp(t)
        self.pub["clock"].publish(m)

    def scan(self, t: float, ranges: np.ndarray) -> None:
        lc = self.cfg.sensors["lidar"]
        m = self.msgs["LaserScan"]()
        m.header.stamp = _stamp(t)
        m.header.frame_id = self.fr["laser"]
        m.angle_min, m.angle_max = float(lc["angle_min"]), float(lc["angle_max"])
        m.angle_increment = float(lc["angle_increment"])
        m.time_increment, m.scan_time = float(lc["time_increment"]), float(lc["scan_time"])
        m.range_min, m.range_max = float(lc["range_min"]), float(lc["range_max"])
        m.ranges = array.array("f", ranges.astype(np.float32).tobytes())
        self.pub["scan"].publish(m)

    def _transform(self, t: float, parent: str, child: str, xyz, q_wxyz):
        from geometry_msgs.msg import TransformStamped

        ts = TransformStamped()
        ts.header.stamp = _stamp(t)
        ts.header.frame_id, ts.child_frame_id = parent, child
        ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z = map(float, xyz)
        w, x, y, z = map(float, q_wxyz)
        ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z, ts.transform.rotation.w = x, y, z, w
        return ts

    def static_tf(self, t: float, tree) -> None:
        m = self.msgs["TFMessage"]()
        m.transforms = [self._transform(t, f.parent, f.child, f.tf[:3, 3], matrix_to_quat(f.tf[:3, :3])) for f in tree]
        self.pub["tf_static"].publish(m)

    def odom(self, t: float, key: str, frame: str, child: str, pos, q_wxyz, v_lin, v_ang, cov: dict | None,
             tf: bool = False) -> None:
        m = self.msgs["Odometry"]()
        m.header.stamp = _stamp(t)
        m.header.frame_id, m.child_frame_id = frame, child
        p = m.pose.pose.position
        p.x, p.y, p.z = map(float, pos)
        o = m.pose.pose.orientation
        o.w, o.x, o.y, o.z = map(float, q_wxyz)
        m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = map(float, v_lin)
        m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z = map(float, v_ang)
        if cov:
            pc = [0.0] * 36
            pc[0] = pc[7] = float(cov["pose_xy"])
            pc[14] = pc[21] = pc[28] = 1e6     # z, roll, pitch not observed
            pc[35] = float(cov["pose_yaw"])
            tc = [0.0] * 36
            tc[0] = float(cov["twist_vx"])
            tc[7] = tc[14] = tc[21] = tc[28] = 1e6
            tc[35] = float(cov["twist_wz"])
            m.pose.covariance, m.twist.covariance = pc, tc
        self.pub[key].publish(m)
        if tf:
            tm = self.msgs["TFMessage"]()
            tm.transforms = [self._transform(t, frame, child, pos, q_wxyz)]
            self.pub["tf"].publish(tm)

    def person(self, t: float, pos, yaw: float) -> None:
        m = self.msgs["PoseStamped"]()
        m.header.stamp = _stamp(t)
        m.header.frame_id = self.fr["map_sim"]
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, pos)
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.pub["gt_person"].publish(m)

    def start_pose(self, t: float, start: dict) -> None:
        """The car's start pose (scene ``car.start``: base_link in map_sim), where /sim/reset puts it back."""
        if "gt_start_pose" not in self.pub:
            return
        yaw = math.radians(float(start.get("yaw_deg", 0.0)))
        m = self.msgs["PoseStamped"]()
        m.header.stamp = _stamp(t)
        m.header.frame_id = self.fr["map_sim"]
        m.pose.position.x, m.pose.position.y = float(start["x"]), float(start["y"])
        m.pose.position.z = float(start.get("z", 0.0))
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.pub["gt_start_pose"].publish(m)

    def front_range(self, t: float, dist: float, max_range: float) -> None:
        """Ground-truth free distance ahead of the front bumper (PhysX raycast along the heading)."""
        if "gt_front_range" not in self.pub:
            return
        m = self.msgs_range()
        m.header.stamp = _stamp(t)
        m.header.frame_id = self.fr["base_link"]
        m.radiation_type = 1            # INFRARED: closest to a single ray
        m.field_of_view = 0.0
        m.min_range, m.max_range = 0.0, float(max_range)
        m.range = float(dist)
        self.pub["gt_front_range"].publish(m)

    def objects(self, t: float, objects: list[dict]) -> None:
        """Ground truth of the static scene objects: one Detection3D each (class_id, id, bbox pose + size)."""
        from vision_msgs.msg import Detection3D, ObjectHypothesisWithPose

        if "gt_objects" not in self.pub:
            return
        m = self.msgs_objects()
        m.header.stamp = _stamp(t)
        m.header.frame_id = self.fr["map_sim"]
        for o in objects:
            d = Detection3D()
            d.header = m.header
            d.id = str(o["id"])
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id, h.hypothesis.score = str(o["class"]), 1.0
            c = d.bbox.center
            c.position.x, c.position.y, c.position.z = map(float, o["center"])
            c.orientation.z, c.orientation.w = math.sin(o["yaw"] / 2), math.cos(o["yaw"] / 2)
            h.pose.pose = c
            d.results = [h]
            d.bbox.size.x, d.bbox.size.y, d.bbox.size.z = map(float, o["size"])
            m.detections.append(d)
        self.pub["gt_objects"].publish(m)

    def imu(self, t: float, gyro, accel, q_wxyz) -> None:
        m = self.msgs["Imu"]()
        m.header.stamp = _stamp(t)
        m.header.frame_id = self.fr["imu"]
        m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = map(float, q_wxyz)
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = map(float, gyro)
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = map(float, accel)
        n = self.cfg.sensors["imu"]["noise"]
        g2, a2 = float(n["gyro_stddev"]) ** 2, float(n["accel_stddev"]) ** 2
        m.angular_velocity_covariance = [g2, 0.0, 0.0, 0.0, g2, 0.0, 0.0, 0.0, g2]
        m.linear_acceleration_covariance = [a2, 0.0, 0.0, 0.0, a2, 0.0, 0.0, 0.0, a2]
        m.orientation_covariance = [1e-3, 0.0, 0.0, 0.0, 1e-3, 0.0, 0.0, 0.0, 1e-3]
        self.pub["imu"].publish(m)

    def camera_info(self, t: float, K: np.ndarray, frame: str, keys: list[str], tx: float = 0.0) -> None:
        st = self.cfg.sensors["stereo"]
        m = self.msgs["CameraInfo"]()
        m.header.stamp = _stamp(t)
        m.header.frame_id = frame
        m.width, m.height = int(st["width"]), int(st["height"])
        m.distortion_model = str(st["distortion_model"])
        m.d = [0.0] * (8 if m.distortion_model == "rational_polynomial" else 5)
        m.k = [float(v) for v in K.reshape(-1)]
        m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        m.p = [float(K[0, 0]), 0.0, float(K[0, 2]), float(tx), 0.0, float(K[1, 1]), float(K[1, 2]), 0.0,
               0.0, 0.0, 1.0, 0.0]
        for k in keys:
            self.pub[k].publish(m)

    def image(self, t: float, key_list: list[str], frame: str, data: np.ndarray, encoding: str) -> None:
        m = self.msgs["Image"]()
        m.header.stamp = _stamp(t)
        m.header.frame_id = frame
        m.height, m.width = int(data.shape[0]), int(data.shape[1])
        m.encoding = encoding
        m.is_bigendian = 0
        m.step = int(data.strides[0]) if data.flags["C_CONTIGUOUS"] else int(data.shape[1] * data.itemsize *
                                                                             (data.shape[2] if data.ndim == 3 else 1))
        m.data = _u8(data)
        for k in key_list:
            self.pub[k].publish(m)
