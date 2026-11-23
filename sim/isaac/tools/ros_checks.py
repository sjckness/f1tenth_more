"""ROS-side checks for the Isaac F1Tenth sim (run in a ROS 2 Humble environment, e.g. f1tenth_more:cpu).

    python3 ros_checks.py <check> --scene base_room --out /ws/output/runs/<run>

Checks: wait | rates | messages | geometry | drive | reset | rtf. Each writes <out>/<check>.json
(and .npz raw data for plots) and prints a PASS/FAIL summary. Only numpy + rclpy + yaml needed.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped

WS = os.environ.get("WS", "/ws")


def load_cfg(scene: str) -> dict:
    c = {}
    for name in ("car_spec", "sensors", "ros_interface"):
        with open(f"{WS}/config/{name}.yaml") as f:
            c[name] = yaml.safe_load(f)
    with open(f"{WS}/config/scenes/{scene}.yaml") as f:
        c["scene"] = yaml.safe_load(f)
    return c


def v(node):
    return node["value"] if isinstance(node, dict) and "value" in node else node


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def rel_qos() -> QoSProfile:
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE)


class Checker(Node):
    def __init__(self, cfg: dict) -> None:
        super().__init__("sim_checker")
        self.cfg = cfg
        self.t = cfg["ros_interface"]["topics"]
        self.sim_time = None
        self.create_subscription(Clock, self.t["clock"], self._on_clock, rel_qos())
        self.latest: dict = {}
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, self.t["command"], rel_qos())

    def _on_clock(self, m: Clock) -> None:
        self.sim_time = m.clock.sec + m.clock.nanosec * 1e-9

    def spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_sim(self, sim_seconds: float, cmd: tuple[float, float] | None = None, cmd_rate: float = 20.0) -> None:
        """Spin until sim time advanced by ``sim_seconds``; optionally publish a command at ``cmd_rate`` Hz."""
        while self.sim_time is None:
            rclpy.spin_once(self, timeout_sec=0.05)
        t0 = self.sim_time
        last = -1e9
        while self.sim_time - t0 < sim_seconds:
            if cmd is not None and self.sim_time - last >= 1.0 / cmd_rate:
                self.send(*cmd)
                last = self.sim_time
            rclpy.spin_once(self, timeout_sec=0.005)

    def send(self, speed: float, steer: float) -> None:
        m = AckermannDriveStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.drive.speed, m.drive.steering_angle = float(speed), float(steer)
        self.cmd_pub.publish(m)

    def grab(self, topic: str, typ, timeout: float = 10.0, qos=None):
        box = {}
        sub = self.create_subscription(typ, topic, lambda m: box.setdefault("m", m), qos or rel_qos())
        end = time.time() + timeout
        while "m" not in box and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.destroy_subscription(sub)
        return box.get("m")

    def reset(self) -> tuple[bool, str]:
        cli = self.create_client(Trigger, self.t["reset_service"])
        if not cli.wait_for_service(timeout_sec=10.0):
            return False, "service not available"
        fut = cli.call_async(Trigger.Request())
        end = time.time() + 10
        while not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.destroy_client(cli)
        if not fut.done():
            return False, "timeout"
        return fut.result().success, fut.result().message


def depth_array(m: Image) -> np.ndarray:
    return np.frombuffer(bytes(m.data), dtype=np.float32).reshape(m.height, m.width)


def image_array(m: Image) -> np.ndarray:
    ch = {"bgra8": 4, "rgba8": 4, "bgr8": 3, "rgb8": 3, "mono8": 1}[m.encoding]
    a = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.step)[:, : m.width * ch]
    return a.reshape(m.height, m.width, ch)


# --------------------------------------------------------------------------- checks
def check_wait(node: Checker, args) -> dict:
    end = time.time() + args.timeout
    while time.time() < end:
        topics = dict(node.get_topic_names_and_types())
        if node.t["clock"] in topics:
            node.spin_for(1.0)
            if node.sim_time is not None:
                return {"pass": True, "sim_time": node.sim_time, "waited_s": args.timeout - (end - time.time())}
        node.spin_for(2.0)
    return {"pass": False, "error": f"/clock not seen within {args.timeout} s"}


def expected_topics(cfg: dict) -> list[str]:
    t, st = cfg["ros_interface"]["topics"], cfg["sensors"]["stereo"]
    names = ["clock", "command", "scan", "odom", "imu", "left_image", "left_info", "depth", "depth_info",
             "camera_image", "camera_info", "gt_odom", "gt_person"]
    if st["right_camera"]:
        names += ["right_image", "right_info"]
    if st["semantic"]["enabled"]:
        names.append("gt_semantic")
    if cfg["ros_interface"]["publish_static_tf"]:
        names.append("tf_static")
    return [t[n] for n in names]


def check_rates(node: Checker, args) -> dict:
    cfg = node.cfg
    t, s = cfg["ros_interface"]["topics"], cfg["sensors"]
    targets = {t["clock"]: (Clock, 60.0), t["scan"]: (LaserScan, s["lidar"]["rate"]),
               t["left_image"]: (Image, s["stereo"]["rate"]), t["depth"]: (Image, s["stereo"]["rate"]),
               t["left_info"]: (CameraInfo, s["stereo"]["rate"]), t["camera_image"]: (Image, s["stereo"]["rate"]),
               t["odom"]: (Odometry, s["odometry"]["rate"]), t["imu"]: (Imu, s["imu"]["rate"]),
               t["gt_odom"]: (Odometry, s["ground_truth"]["rate"])}
    if s["stereo"]["right_camera"]:
        targets[t["right_image"]] = (Image, s["stereo"]["rate"])
    stamps: dict = {k: [] for k in targets}
    wall: dict = {k: [] for k in targets}
    subs = []
    for topic, (typ, _r) in targets.items():
        def cb(m, topic=topic):
            wall[topic].append(time.time())
            if hasattr(m, "header"):
                stamps[topic].append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            else:
                stamps[topic].append(m.clock.sec + m.clock.nanosec * 1e-9)
        subs.append(node.create_subscription(typ, topic, cb, rel_qos()))
    node.spin_for(3.0)          # discovery settles before the topic list is taken
    present = dict(node.get_topic_names_and_types())
    missing = [x for x in expected_topics(cfg) if x not in present]
    for k in targets:
        stamps[k].clear()
        wall[k].clear()
    node.spin_for(args.seconds)
    rows, ok = [], not missing
    for topic, (_typ, rate) in targets.items():
        st = np.array(stamps[topic])
        w = np.array(wall[topic])
        hz_sim = (len(st) - 1) / (st[-1] - st[0]) if len(st) > 2 and st[-1] > st[0] else 0.0
        hz_wall = (len(w) - 1) / (w[-1] - w[0]) if len(w) > 2 else 0.0
        jitter = float(np.std(np.diff(st))) if len(st) > 2 else None
        good = abs(hz_sim - rate) <= 0.1 * rate
        ok &= good
        rows.append({"topic": topic, "target_hz": rate, "hz_sim_stamps": round(hz_sim, 2),
                     "hz_wall": round(hz_wall, 2), "n": len(st), "stamp_jitter_s": jitter, "pass": bool(good)})
    for sb in subs:
        node.destroy_subscription(sb)
    return {"pass": bool(ok), "missing_topics": missing, "present_topics": sorted(present), "rates": rows,
            "window_s": args.seconds}


def check_messages(node: Checker, args) -> dict:
    cfg = node.cfg
    t, s, fr = cfg["ros_interface"]["topics"], cfg["sensors"], cfg["ros_interface"]["frames"]
    out, ok = {}, True
    info = node.grab(t["left_info"], CameraInfo)
    if info is None:
        return {"pass": False, "error": "no camera_info"}
    target = np.array([[s["stereo"]["fx"], 0, s["stereo"]["cx"]], [0, s["stereo"]["fy"], s["stereo"]["cy"]], [0, 0, 1]])
    k = np.array(info.k).reshape(3, 3)
    kerr = {n: abs(k[i, j] - target[i, j]) / abs(target[i, j]) for n, (i, j) in
            {"fx": (0, 0), "fy": (1, 1), "cx": (0, 2), "cy": (1, 2)}.items()}
    out["camera_info"] = {"frame_id": info.header.frame_id, "width": info.width, "height": info.height,
                          "K": list(info.k), "D": list(info.d), "distortion_model": info.distortion_model,
                          "K_rel_err": kerr, "pass": max(kerr.values()) < 0.01
                          and info.header.frame_id == fr["zed_left_optical"]}
    rinfo = node.grab(t["right_info"], CameraInfo) if s["stereo"]["right_camera"] else None
    if rinfo is not None:
        out["right_camera_info"] = {"frame_id": rinfo.header.frame_id, "P": list(rinfo.p),
                                    "baseline_from_P": -rinfo.p[3] / rinfo.p[0]}
    scan = node.grab(t["scan"], LaserScan)
    out["scan"] = {"frame_id": scan.header.frame_id, "angle_min": scan.angle_min, "angle_max": scan.angle_max,
                   "angle_increment": scan.angle_increment, "n_ranges": len(scan.ranges),
                   "range_min": scan.range_min, "range_max": scan.range_max, "scan_time": scan.scan_time,
                   "time_increment": scan.time_increment}
    exp_n = int(round((s["lidar"]["angle_max"] - s["lidar"]["angle_min"]) / s["lidar"]["angle_increment"])) + 1
    out["scan"]["pass"] = (scan.header.frame_id == fr["laser"] and len(scan.ranges) == exp_n and
                           abs(scan.angle_min - s["lidar"]["angle_min"]) < 1e-6 and
                           abs(scan.angle_max - s["lidar"]["angle_max"]) < 1e-6 and
                           abs(scan.angle_increment - s["lidar"]["angle_increment"]) < 1e-9)
    img = node.grab(t["left_image"], Image)
    out["left_image"] = {"encoding": img.encoding, "frame_id": img.header.frame_id, "w": img.width, "h": img.height,
                         "step": img.step, "pass": img.encoding == s["stereo"]["image_encoding"]}
    cam = node.grab(t["camera_image"], Image)
    out["camera_image"] = {"encoding": cam.encoding, "frame_id": cam.header.frame_id, "w": cam.width,
                           "h": cam.height, "pass": cam.encoding == s["stereo"]["image_encoding"]}
    dep = node.grab(t["depth"], Image)
    out["depth"] = {"encoding": dep.encoding, "frame_id": dep.header.frame_id, "w": dep.width, "h": dep.height,
                    "step": dep.step, "pass": dep.encoding == "32FC1" and dep.header.frame_id == fr["zed_left_optical"]}
    imu = node.grab(t["imu"], Imu)
    out["imu"] = {"frame_id": imu.header.frame_id, "acc": [imu.linear_acceleration.x, imu.linear_acceleration.y,
                                                         imu.linear_acceleration.z],
                  "gyro": [imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z]}
    od = node.grab(t["odom"], Odometry)
    out["odom"] = {"frame_id": od.header.frame_id, "child": od.child_frame_id}
    for kk in ("camera_info", "scan", "left_image", "camera_image", "depth"):
        ok &= bool(out[kk]["pass"])
    out["pass"] = bool(ok)
    return out


def camera_pose(cfg: dict) -> tuple[np.ndarray, np.ndarray]:
    """Left camera optical centre (world) and rotation world<-optical at the start pose."""
    cs, sc = cfg["car_spec"], cfg["scene"]
    lens = np.array(v(cs["sensors"]["zed"]["lens_center_xyz"]))
    pitch = v(cs["sensors"]["zed"]["pitch"])
    half_b = v(cs["sensors"]["zed"]["baseline"]) / 2
    yaw = math.radians(sc["car"]["start"]["yaw_deg"])
    rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    ry = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
    base = np.array([sc["car"]["start"]["x"], sc["car"]["start"]["y"], 0.0])
    left = base + rz @ (lens + ry @ np.array([0, half_b, 0]))
    r_body_opt = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
    return left, rz @ ry @ r_body_opt


def check_geometry(node: Checker, args) -> dict:
    cfg = node.cfg
    t, s, cs, sc = cfg["ros_interface"]["topics"], cfg["sensors"], cfg["car_spec"], cfg["scene"]
    ok_reset, _ = node.reset()
    node.wait_sim(1.5)
    res: dict = {"reset_ok": ok_reset}
    scan = node.grab(t["scan"], LaserScan)
    depth = node.grab(t["depth"], Image)
    left = node.grab(t["left_image"], Image)
    right = node.grab(t["right_image"], Image) if s["stereo"]["right_camera"] else None
    gt = node.grab(t["gt_odom"], Odometry)
    info = node.grab(t["left_info"], CameraInfo)
    present = dict(node.get_topic_names_and_types())
    sem = node.grab(t["gt_semantic"], Image, timeout=5) if t["gt_semantic"] in present else None
    ranges = np.array(scan.ranges, dtype=np.float32)
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
    i0 = int(np.argmin(np.abs(angles)))
    laser = np.array(v(cs["sensors"]["laser"]["xyz"]))
    x0, y0 = sc["car"]["start"]["x"], sc["car"]["start"]["y"]
    gp = gt.pose.pose.position
    res["gt_start"] = {"x": gp.x, "y": gp.y, "yaw": yaw_of(gt.pose.pose.orientation)}
    if "expected" in sc and "wall_ahead_range_from_base" in sc["expected"]:
        exp_scan = sc["expected"]["wall_ahead_range_from_base"] - laser[0]
        res["scan_ahead"] = {"measured": float(ranges[i0]), "expected": exp_scan, "source": "raycast in find_pose.py",
                             "err": float(ranges[i0] - exp_scan), "pass": abs(ranges[i0] - exp_scan) <= 0.05}
    if "expected" in sc and "wall_ahead_x" in sc["expected"]:
        exp_scan = sc["expected"]["wall_ahead_x"] - (x0 + laser[0])
        res["scan_ahead"] = {"measured": float(ranges[i0]), "expected": exp_scan,
                             "err": float(ranges[i0] - exp_scan), "pass": abs(ranges[i0] - exp_scan) <= 0.02}
        cam, r_wo = camera_pose(cfg)
        axis = r_wo[:, 2]
        d = depth_array(depth)
        cx, cy = info.k[2], info.k[5]
        u, vv = int(round(cx - 0.5)), int(round(cy - 0.5))
        dc = float(np.nanmedian(d[vv - 1:vv + 2, u - 1:u + 2]))
        # Z depth of the wall plane x = X along the optical axis from the camera centre
        exp_d = (sc["expected"]["wall_ahead_x"] - cam[0]) / axis[0]
        res["depth_center"] = {"measured": dc, "expected": float(exp_d), "err": float(dc - exp_d),
                               "pixel": [u, vv], "pass": abs(dc - exp_d) <= 0.02}
    # Person: expected bearing/distance in the scan and projection in the left image.
    if sc.get("person", {}).get("enabled"):
        pp = np.array(sc["person"]["position"], dtype=float)
        rad = sc["person"]["collider"]["radius"]
        yw = math.radians(sc["car"]["start"]["yaw_deg"])
        lxw = x0 + math.cos(yw) * laser[0] - math.sin(yw) * laser[1]
        lyw = y0 + math.sin(yw) * laser[0] + math.cos(yw) * laser[1]
        dx, dy = pp[0] - lxw, pp[1] - lyw
        brg = math.atan2(dy, dx) - math.radians(sc["car"]["start"]["yaw_deg"])
        brg = math.atan2(math.sin(brg), math.cos(brg))
        dist = math.hypot(dx, dy)
        win = np.abs(angles - brg) < math.radians(6)
        rmin = float(np.nanmin(ranges[win])) if win.any() else float("nan")
        amin = float(angles[win][int(np.nanargmin(ranges[win]))]) if win.any() else float("nan")
        res["scan_person"] = {"expected_bearing_deg": math.degrees(brg), "measured_bearing_deg": math.degrees(amin),
                              "expected_range_capsule": dist - rad, "measured_min_range": rmin,
                              "pass": abs(rmin - (dist - rad)) < 0.15 and abs(amin - brg) < math.radians(4)}
        cam, r_wo = camera_pose(cfg)
        pc = r_wo.T @ (np.array([pp[0], pp[1], 1.0]) - cam)
        u = info.k[0] * pc[0] / pc[2] + info.k[2]
        vv = info.k[4] * pc[1] / pc[2] + info.k[5]
        d = depth_array(depth)
        inside = 0 <= u < info.width and 0 <= vv < info.height and pc[2] > 0
        dp = float(np.nanmedian(d[max(0, int(vv) - 3):int(vv) + 4, max(0, int(u) - 3):int(u) + 4])) if inside else None
        res["image_person"] = {"projected_uv": [float(u), float(vv)], "expected_depth_z": float(pc[2]),
                               "depth_at_uv": dp, "pass": bool(inside and dp is not None and abs(dp - pc[2]) < 0.4)}
    os.makedirs(args.out, exist_ok=True)
    np.savez_compressed(f"{args.out}/frames.npz", left=image_array(left), depth=depth_array(depth),
                        right=image_array(right) if right is not None else np.zeros(1),
                        semantic=image_array(sem)[:, :, 0] if sem is not None else np.zeros(1),
                        ranges=ranges, angles=angles, K=np.array(info.k),
                        pose=np.array([gp.x, gp.y, yaw_of(gt.pose.pose.orientation)]))
    res["pass"] = all(vv.get("pass", True) for vv in res.values() if isinstance(vv, dict)) and ok_reset
    return res


def _track(node: Checker, topic: str, store: list):
    def cb(m: Odometry):
        p = m.pose.pose.position
        tw = m.twist.twist
        store.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, yaw_of(m.pose.pose.orientation),
                      tw.linear.x, tw.angular.z))
    return node.create_subscription(Odometry, topic, cb, rel_qos())


def _fit_circle(x: np.ndarray, y: np.ndarray) -> tuple[float, float, float]:
    a = np.c_[2 * x, 2 * y, np.ones_like(x)]
    b = x ** 2 + y ** 2
    (cx, cy, c), *_ = np.linalg.lstsq(a, b, rcond=None)
    return float(cx), float(cy), float(math.sqrt(c + cx ** 2 + cy ** 2))


def _odom_vs_gt(gt: np.ndarray, od: np.ndarray, t_end: float) -> dict:
    """Compare odom (starts at 0 after reset) with gt motion expressed in the start frame."""
    g = gt[gt[:, 0] <= t_end]
    o = od[od[:, 0] <= t_end]
    if len(g) < 2 or len(o) < 2:
        return {}
    x0, y0, th0 = g[0, 1:4]
    c, s = math.cos(-th0), math.sin(-th0)
    gx, gy = c * (g[-1, 1] - x0) - s * (g[-1, 2] - y0), s * (g[-1, 1] - x0) + c * (g[-1, 2] - y0)
    ox, oy = o[-1, 1] - o[0, 1], o[-1, 2] - o[0, 2]
    dyaw = math.atan2(math.sin(g[-1, 3] - th0), math.cos(g[-1, 3] - th0))
    return {"gt_rel": [gx, gy, dyaw], "odom_rel": [ox, oy, o[-1, 3] - o[0, 3]],
            "pos_err_m": math.hypot(ox - gx, oy - gy),
            "yaw_err_rad": math.atan2(math.sin(o[-1, 3] - o[0, 3] - (g[-1, 3] - th0)),
                                      math.cos(o[-1, 3] - o[0, 3] - (g[-1, 3] - th0)))}


def check_drive(node: Checker, args) -> dict:
    cfg = node.cfg
    t, cs = cfg["ros_interface"]["topics"], cfg["car_spec"]
    L = v(cs["geometry"]["wheelbase"])
    dmax = v(cs["limits"]["max_steering_left"])
    timeout = v(cs["limits"]["command_timeout"])
    res: dict = {}
    gt, od = [], []
    s1, s2 = _track(node, t["gt_odom"], gt), _track(node, t["odom"], od)

    def run(name: str, speed: float, steer: float, dur: float, then_stop: bool, settle: float = 1.5) -> dict:
        node.reset()
        node.wait_sim(1.0)
        gt.clear(), od.clear()
        node.wait_sim(0.2)
        t_start = node.sim_time
        node.wait_sim(dur, cmd=(speed, steer))
        t_cmd_end = node.sim_time
        if then_stop:
            node.wait_sim(settle, cmd=(0.0, steer))
        else:
            node.wait_sim(settle + timeout)
        g, o = np.array(gt), np.array(od)
        m = g[g[:, 0] >= t_start]
        disp_at_end = m[m[:, 0] <= t_cmd_end]
        d_end = math.hypot(disp_at_end[-1, 1] - m[0, 1], disp_at_end[-1, 2] - m[0, 2])
        after = m[m[:, 0] > t_cmd_end]
        moving = np.abs(after[:, 4]) > 0.02          # "stopped" = |v| below 2 cm/s, and stays there
        if len(after) == 0 or moving[-1]:
            stop_t = None
        elif not moving.any():
            stop_t = 0.0
        else:
            stop_t = float(after[np.where(moving)[0].max() + 1, 0] - t_cmd_end)
        np.save(f"{args.out}/drive_{name}_gt.npy", g)
        np.save(f"{args.out}/drive_{name}_odom.npy", o)
        return {"m": m, "g": g, "o": o, "t_start": t_start, "t_cmd_end": t_cmd_end, "disp_at_cmd_end": d_end,
                "stop_time_after_cmd_end": stop_t, "final_speed": float(after[-1, 4]) if len(after) else None,
                "odom_vs_gt": _odom_vs_gt(g, o, g[-1, 0])}

    os.makedirs(args.out, exist_ok=True)
    r = run("straight", 0.5, 0.0, 2.0, then_stop=True)
    res["straight"] = {"displacement_m": r["disp_at_cmd_end"], "expected": 1.0,
                       "stop_time_s": r["stop_time_after_cmd_end"], "odom_vs_gt": r["odom_vs_gt"],
                       "pass_disp": abs(r["disp_at_cmd_end"] - 1.0) <= 0.1,
                       "pass_stop": r["stop_time_after_cmd_end"] is not None and r["stop_time_after_cmd_end"] <= 0.5}
    r = run("turn_left", 0.5, dmax, 4.0, then_stop=True)
    m = r["m"]
    seg = m[(m[:, 0] >= r["t_start"] + 1.0) & (m[:, 0] <= r["t_cmd_end"])]
    cx, cy, rad = _fit_circle(seg[:, 1], seg[:, 2])
    exp_r = L / math.tan(dmax)
    yaw_change = float(np.unwrap(m[:, 3])[-1] - m[0, 3])
    res["turn_left"] = {"radius_m": rad, "expected_radius_m": exp_r, "rel_err": (rad - exp_r) / exp_r,
                        "yaw_change_rad": yaw_change, "turns_left": yaw_change > 0,
                        "odom_vs_gt": r["odom_vs_gt"], "pass": abs(rad - exp_r) / exp_r <= 0.10 and yaw_change > 0}
    dmin = v(cs["limits"]["max_steering_right"])
    r = run("turn_right", 0.5, -dmin, 2.5, then_stop=True)   # 2.5 s: a 4 s right turn hits the y=0 wall
    m = r["m"]
    seg = m[(m[:, 0] >= r["t_start"] + 0.8) & (m[:, 0] <= r["t_cmd_end"])]
    cx, cy, rad = _fit_circle(seg[:, 1], seg[:, 2])
    exp_r = L / math.tan(dmin)
    yaw_change = float(np.unwrap(m[:, 3])[-1] - m[0, 3])
    res["turn_right"] = {"radius_m": rad, "expected_radius_m": exp_r, "rel_err": (rad - exp_r) / exp_r,
                         "yaw_change_rad": yaw_change, "odom_vs_gt": r["odom_vs_gt"],
                         "pass": abs(rad - exp_r) / exp_r <= 0.10 and yaw_change < 0}
    r = run("timeout", 0.5, 0.0, 1.0, then_stop=False)
    res["timeout"] = {"command_timeout_s": timeout, "stop_time_after_last_cmd_s": r["stop_time_after_cmd_end"],
                      "final_speed": r["final_speed"],
                      "pass": r["stop_time_after_cmd_end"] is not None and r["final_speed"] is not None
                      and abs(r["final_speed"]) < 0.02 and r["stop_time_after_cmd_end"] >= timeout - 0.05
                      and r["stop_time_after_cmd_end"] <= timeout + 0.6}
    node.destroy_subscription(s1)
    node.destroy_subscription(s2)
    res["pass"] = bool(res["straight"]["pass_disp"] and res["straight"]["pass_stop"] and res["turn_left"]["pass"]
                       and res["turn_right"]["pass"] and res["timeout"]["pass"])
    return res


def check_reset(node: Checker, args) -> dict:
    cfg = node.cfg
    t, sc = cfg["ros_interface"]["topics"], cfg["scene"]
    node.reset()
    node.wait_sim(1.0)
    node.wait_sim(1.5, cmd=(0.8, 0.2))
    node.wait_sim(0.5, cmd=(0.0, 0.0))
    moved = node.grab(t["gt_odom"], Odometry).pose.pose.position
    t0 = time.time()
    ok, msg = node.reset()
    call_s = time.time() - t0
    node.wait_sim(1.0)
    g = node.grab(t["gt_odom"], Odometry)
    p = g.pose.pose.position
    ex, ey = sc["car"]["start"]["x"], sc["car"]["start"]["y"]
    err = math.hypot(p.x - ex, p.y - ey)
    yaw_err = yaw_of(g.pose.pose.orientation) - math.radians(sc["car"]["start"]["yaw_deg"])
    return {"service_ok": ok, "message": msg, "call_s": call_s, "moved_to": [moved.x, moved.y],
            "after_reset": [p.x, p.y], "pos_err_m": err, "yaw_err_rad": yaw_err,
            "speed_after": g.twist.twist.linear.x, "pass": bool(ok and err <= 0.01 and abs(yaw_err) < 0.01)}


def check_rtf(node: Checker, args) -> dict:
    node.spin_for(1.0)
    s0, w0 = node.sim_time, time.time()
    node.spin_for(args.seconds)
    s1, w1 = node.sim_time, time.time()
    rtf = (s1 - s0) / (w1 - w0)
    return {"sim_s": s1 - s0, "wall_s": w1 - w0, "rtf": rtf, "pass": rtf >= 0.5}


CHECKS = {"wait": check_wait, "rates": check_rates, "messages": check_messages, "geometry": check_geometry,
          "drive": check_drive, "reset": check_reset, "rtf": check_rtf}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("check", choices=list(CHECKS))
    ap.add_argument("--scene", default="base_room")
    ap.add_argument("--out", default=f"{WS}/output/runs/latest")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--timeout", type=float, default=900.0)
    args = ap.parse_args()
    rclpy.init()
    node = Checker(load_cfg(args.scene))
    try:
        res = CHECKS[args.check](node, args)
    except Exception as exc:  # noqa: BLE001
        import traceback
        res = {"pass": False, "error": repr(exc), "trace": traceback.format_exc()}
    os.makedirs(args.out, exist_ok=True)
    with open(f"{args.out}/{args.check}.json", "w") as f:
        json.dump(res, f, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    print(f"[{'PASS' if res.get('pass') else 'FAIL'}] {args.check}: " + json.dumps(
        {k: vv for k, vv in res.items() if k not in ("present_topics", "trace")}, default=str)[:3000])
    node.destroy_node()
    rclpy.shutdown()
    return 0 if res.get("pass") else 1


if __name__ == "__main__":
    sys.exit(main())
