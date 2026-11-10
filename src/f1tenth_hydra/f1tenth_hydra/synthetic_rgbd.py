"""A synthetic ZED 2i + Isaac GT feed for the Hydra smoke test (no Isaac needed).

Publishes exactly the topics of config/camera_zed2i.yaml that hydra.launch.py reads with
seg_source:=gt pose_source:=isaac_gt: RGB (bgra8), camera_info, depth (32FC1, -inf/+inf outside
0.3-10 m), GT semantics (mono8 Isaac class ids) and /sim/ground_truth/odom. The scene is a
ray-cast 8 x 6 x 2.5 m room with two boxes and a potted plant; the car drives a slow loop.
Stamps are wall time (run Hydra with use_sim_time:=false).

    ros2 run f1tenth_hydra hydra_synthetic_rgbd [--ros-args -p rate_hz:=5.0]
"""

import math

import numpy as np

from f1tenth_hydra.camera import base_T_left_optical, load_camera_contract

# Isaac class ids (base_room `semantics:`): unlabeled 0, floor 1, wall 2, potted plant 5, box 6
ROOM = (0.0, 8.0, 0.0, 6.0, 0.0, 2.5)                     # x0, x1, y0, y1, z0, z1
BOXES = [((1.0, 0.6, 0.0), (1.6, 1.2, 0.5), 6),          # (min, max, class id), all
         ((5.0, 5.2, 0.0), (5.6, 5.8, 0.8), 6),          # outside the loop (r 2 m about 4, 3)
         ((6.5, 1.0, 0.0), (6.9, 1.4, 1.0), 5)]
COLORS = {0: (200, 200, 200), 1: (90, 120, 90), 2: (180, 170, 150), 5: (40, 160, 40),
          6: (150, 90, 40)}


def render(contract, x, y, yaw):
    """Depth (m, float32), class ids (uint8) and BGRA for base_link at (x, y, yaw)."""
    cam = contract['camera']
    k = cam['intrinsics']
    w, h = int(cam['width']), int(cam['height'])
    u, v = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
    d_opt = np.stack([(u - k['cx']) / k['fx'], (v - k['cy']) / k['fy'], np.ones_like(u)], -1)
    t_bo = base_T_left_optical(contract)
    c, s = math.cos(yaw), math.sin(yaw)
    t_wb = np.array([[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, 0], [0, 0, 0, 1.0]])
    t_wo = t_wb @ t_bo
    d = d_opt @ t_wo[:3, :3].T                       # unnormalised: t along it == optical depth
    o = t_wo[:3, 3]
    with np.errstate(divide='ignore', invalid='ignore'):
        depth = np.full((h, w), np.inf)
        ids = np.zeros((h, w), np.uint8)
        # room interior: the nearest positive exit over the six planes
        for axis, (lo, hi) in enumerate(((ROOM[0], ROOM[1]), (ROOM[2], ROOM[3]),
                                         (ROOM[4], ROOM[5]))):
            for plane in (lo, hi):
                t = (plane - o[axis]) / d[..., axis]
                hit = (t > 1e-6) & (t < depth)
                depth = np.where(hit, t, depth)
                cls = (1 if plane == lo else 0) if axis == 2 else 2   # floor / ceiling / wall
                ids = np.where(hit, cls, ids)
        for bmin, bmax, cls in BOXES:
            tmin = (np.array(bmin) - o) / d
            tmax = (np.array(bmax) - o) / d
            t_near = np.nanmax(np.minimum(tmin, tmax), axis=-1)
            t_far = np.nanmin(np.maximum(tmin, tmax), axis=-1)
            hit = (t_near < t_far) & (t_near > 1e-6) & (t_near < depth)
            depth = np.where(hit, t_near, depth)
            ids = np.where(hit, cls, ids)
    dep = cam['depth']
    depth = depth.astype(np.float32)
    depth[depth < dep['min_m']] = -np.inf
    depth[depth > dep['max_m']] = np.inf
    rgb = np.zeros((h, w, 4), np.uint8)
    for cls, (r, g, b) in COLORS.items():
        rgb[ids == cls] = (b, g, r, 255)
    shade = np.clip(1.2 - np.nan_to_num(np.abs(depth), posinf=10.0) / 12.0, 0.3, 1.0)
    rgb[..., :3] = (rgb[..., :3] * shade[..., None]).astype(np.uint8)
    return depth, ids, rgb


def pose_at(t):
    """A slow loop inside the room: (x, y, yaw) at t seconds."""
    r, cx, cy, omega = 2.0, 4.0, 3.0, 0.12
    a = omega * t
    return cx + r * math.cos(a), cy + r * math.sin(a), a + math.pi / 2


def main(args=None):
    import rclpy
    from geometry_msgs.msg import Quaternion
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image

    rclpy.init(args=args)
    node = Node('hydra_synthetic_rgbd')
    contract = load_camera_contract()
    topics, cam = contract['topics'], contract['camera']
    rate = float(node.declare_parameter('rate_hz', 5.0).value)
    frame = contract['frames']['left_optical_v4']
    pubs = {k: node.create_publisher(Image, topics[k], 5) for k in ('rgb', 'depth', 'semantic_gt')}
    info_pubs = [node.create_publisher(CameraInfo, topics[k], 5) for k in ('rgb_info', 'depth_info')]
    odom_pub = node.create_publisher(Odometry, topics['gt_odom'], 10)
    k = cam['intrinsics']
    info = CameraInfo(width=int(cam['width']), height=int(cam['height']),
                      distortion_model=k['distortion_model'], d=list(k['D']),
                      k=[k['fx'], 0.0, k['cx'], 0.0, k['fy'], k['cy'], 0.0, 0.0, 1.0],
                      r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                      p=[k['fx'], 0.0, k['cx'], 0.0, 0.0, k['fy'], k['cy'], 0.0, 0.0, 0.0, 1.0, 0.0])
    t0 = node.get_clock().now()

    def image(arr, encoding, stamp):
        msg = Image(height=arr.shape[0], width=arr.shape[1], encoding=encoding,
                    is_bigendian=0, step=arr.strides[0], data=arr.tobytes())
        msg.header.stamp, msg.header.frame_id = stamp, frame
        return msg

    def odom():
        now = node.get_clock().now()
        x, y, yaw = pose_at((now - t0).nanoseconds * 1e-9)
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = now.to_msg(), 'map_sim', \
            'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation = Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2))
        odom_pub.publish(msg)

    def frame_cb():
        now = node.get_clock().now()
        x, y, yaw = pose_at((now - t0).nanoseconds * 1e-9)
        depth, ids, rgb = render(contract, x, y, yaw)
        stamp = now.to_msg()
        pubs['rgb'].publish(image(rgb, 'bgra8', stamp))
        pubs['depth'].publish(image(depth, '32FC1', stamp))
        pubs['semantic_gt'].publish(image(ids, 'mono8', stamp))
        for p in info_pubs:
            info.header.stamp, info.header.frame_id = stamp, frame
            p.publish(info)

    node.create_timer(0.02, odom)
    node.create_timer(1.0 / rate, frame_cb)
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
