"""Drive the sim car in circles for N sim seconds and count non-finite values on /odom, /sensors/imu/raw and
the ground truth (physics NaN check). Host side, plain rclpy (ROS 2 Jazzy).

    python3 tools/drive_nan_check.py [--seconds 300] [--speed 0.4] [--steer 0.3] [--out result.json]
The steering sign flips every 30 s (left and right circles, radius ~1 m at 0.3 rad).
"""
import argparse
import json
import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=300.0)
    ap.add_argument("--speed", type=float, default=0.4)
    ap.add_argument("--steer", type=float, default=0.3)
    ap.add_argument("--out", default="")
    ap.add_argument("--track", default="", help="CSV of the ground truth every 0.25 s sim time")
    a = ap.parse_args()
    rclpy.init()
    n = rclpy.create_node("drive_nan_check")
    st = {"t": None, "t0": None, "odom": 0, "odom_nan": 0, "imu": 0, "imu_nan": 0, "gt": 0, "gt_nan": 0,
          "path_m": 0.0, "last": None, "first_nan_t": None, "last_finite": None, "rows": [], "next_row": 0.0, "xmin": 1e9, "xmax": -1e9, "ymin": 1e9, "ymax": -1e9}

    def finite(*v):
        return all(math.isfinite(x) for x in v)

    def on_clock(m):
        st["t"] = m.clock.sec + m.clock.nanosec * 1e-9
        if st["t0"] is None:
            st["t0"] = st["t"]

    def on_odom(m):
        p, tw = m.pose.pose, m.twist.twist
        st["odom"] += 1
        st["odom_nan"] += not finite(p.position.x, p.position.y, p.orientation.z, p.orientation.w, tw.linear.x,
                                     tw.angular.z)

    def on_imu(m):
        st["imu"] += 1
        st["imu_nan"] += not finite(m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                                    m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z,
                                    m.orientation.w)

    def on_gt(m):
        p = m.pose.pose.position
        st["gt"] += 1
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t >= st["next_row"]:
            q = m.pose.pose.orientation
            st["rows"].append([round(t, 2), p.x, p.y, p.z, q.w, q.x, q.y, q.z, m.twist.twist.linear.x,
                               m.twist.twist.angular.z])
            st["next_row"] = t + 0.25
        if not finite(p.x, p.y, p.z):
            st["gt_nan"] += 1
            if st["first_nan_t"] is None:
                st["first_nan_t"] = round(t, 2)
            return
        st["last_finite"] = [round(t, 2), round(p.x, 3), round(p.y, 3), round(p.z, 3)]
        if st["last"]:
            st["path_m"] += math.hypot(p.x - st["last"][0], p.y - st["last"][1])
        st["last"] = (p.x, p.y)
        st["xmin"], st["xmax"] = min(st["xmin"], p.x), max(st["xmax"], p.x)
        st["ymin"], st["ymax"] = min(st["ymin"], p.y), max(st["ymax"], p.y)

    n.create_subscription(Clock, "/clock", on_clock, 10)
    n.create_subscription(Odometry, "/odom", on_odom, 50)
    n.create_subscription(Imu, "/sensors/imu/raw", on_imu, 50)
    n.create_subscription(Odometry, "/sim/ground_truth/odom", on_gt, 50)
    pub = n.create_publisher(AckermannDriveStamped, "/ackermann_drive", 1)
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.05)
        if st["t0"] is None:
            continue
        el = st["t"] - st["t0"]
        if el >= a.seconds:
            break
        cmd = AckermannDriveStamped()
        cmd.drive.speed = a.speed
        cmd.drive.steering_angle = a.steer if int(el // 30) % 2 == 0 else -a.steer
        pub.publish(cmd)
    pub.publish(AckermannDriveStamped())
    if a.track:
        with open(a.track, "w") as fh:
            fh.write("t,x,y,z,qw,qx,qy,qz,vx,wz\n")
            for r in st["rows"]:
                fh.write(",".join(f"{v:.4f}" if isinstance(v, float) else str(v) for v in r) + "\n")
    res = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in st.items()
           if k not in ("last", "rows", "next_row")}
    res["sim_seconds"] = round(st["t"] - st["t0"], 1) if st["t0"] is not None else 0
    print(json.dumps(res))
    if a.out:
        with open(a.out, "w") as fh:
            json.dump(res, fh, indent=1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
