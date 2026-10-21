"""
mission_overlay.py: the run-report overlay, drawn offline from a run's bag.

One frame from fixture data (an image, a mask, three LiDAR returns) through
the report's draw(); a run recorded the default way (the clean camera frame
and the 5 Hz depth, no recorded mask) into stills; and a run with no mask at
all, which must get no overlay and keep its recorded frames, through
mission_logger_node's own finalize step too. The bags use the sqlite3 schema
rosbag2 writes, with messages serialized by rclpy, so the hand-written CDR
decoders are checked against the real serializer.
"""

import array
import ast
import math
import shutil
import sqlite3
import struct
from types import SimpleNamespace

import cv2
from f1tenth_logger import mission_debug_video as mdv
from f1tenth_logger import mission_overlay as mo
from f1tenth_perception import scene_mask as sm
import numpy as np
import pytest

#: A 160 x 90 camera: the ZED's 640 x 360 intrinsics scaled by a quarter.
W, H = 160, 90
FX = FY = 262.4 * W / 640.0
CX, CY = W / 2.0, H / 2.0
#: base_link -> the camera's optical frame (no pitch) and -> the laser.
CAM_T = (0.24, 0.0, 0.20)
LASER_T = (0.30, 0.0, 0.19)
R_OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
OPTICAL_Q = (-0.5, 0.5, -0.5, 0.5)
#: A wall across the whole view at base_link x = WALL_X, on the floor z = 0.
WALL_X = 2.0
GREY = 128
#: Every recorded camera frame of _recorded_run, clean and debug alike.
GREY_JPEG = cv2.imencode('.jpg', np.full((H, W, 3), GREY, np.uint8))[1].tobytes()


def test_the_optical_quaternion_is_the_optical_rotation():
    assert np.allclose(mo.quat_to_matrix(*OPTICAL_Q), R_OPTICAL)


def _draw(labels, legend=False):
    img = np.full((H, W, 3), GREY, np.uint8)
    scan = {'ranges': np.array([1.0, 2.0, 3.0], np.float32), 'angle_min': -0.1,
            'angle_increment': 0.1, 'range_min': 0.05, 'range_max': 30.0}
    out = mo.draw(img, labels, scan, (np.eye(3), np.array(LASER_T)),
                  (R_OPTICAL, np.array(CAM_T)), (FX, FY, CX, CY), mo.Style(legend=legend))
    return img, out


def _mask():
    """Wall over floor at twice the image's resolution, an INVALID strip on the left."""
    labels = np.full((2 * H, 2 * W), sm.CLASS_WALL, np.uint8)
    labels[H:] = sm.CLASS_FLOOR
    labels[:, :20] = sm.CLASS_INVALID
    return labels


def test_one_frame_from_an_image_a_mask_and_three_lidar_points():
    img, out = _draw(_mask())
    assert out.shape == img.shape
    assert np.abs(out.astype(int) - img.astype(int)).sum() > 0
    # INVALID is not a report class: those pixels are the camera's, untouched.
    assert (out[H // 4, :8] == GREY).all()
    # Wall cyan and floor green, at alpha 0.3 over the grey.
    b, g, r = (int(c) for c in out[H // 4, W // 2])
    assert b - r > 50 and g - r > 50
    b, g, r = (int(c) for c in out[3 * H // 4, W // 2])
    assert g - r > 20 and g - b > 20
    # The middle return, 2 m straight ahead of the laser, lands just under the
    # principal point (the laser sits 1 cm under the lens) in its range colour.
    depth = LASER_T[0] + 2.0 - CAM_T[0]
    u, v = int(round(CX)), int(round(CY + FY * (CAM_T[2] - LASER_T[2]) / depth))
    assert tuple(out[v, u]) == tuple(sm.range_colours([2.0], mo.Style().range_max_m)[0])


def test_the_legend_is_a_small_top_left_corner_and_can_be_turned_off():
    img, without = _draw(_mask(), legend=False)
    _img, with_legend = _draw(_mask(), legend=True)
    changed = np.argwhere((with_legend != without).any(axis=2))
    assert len(changed)
    assert changed[:, 0].max() < 0.6 * H and changed[:, 1].max() < 0.5 * W


# ---- recorded runs -----------------------------------------------------------


@pytest.fixture
def ros():
    """Return rclpy's serializer and the message classes; skip where ROS is absent."""
    serialization = pytest.importorskip('rclpy.serialization')
    return (serialization.serialize_message, pytest.importorskip('sensor_msgs.msg'),
            pytest.importorskip('tf2_msgs.msg'), pytest.importorskip('geometry_msgs.msg'))


def _write_bag(run, messages):
    """Write a rosbag2 sqlite3 bag of (topic, type, receive_sec, cdr bytes)."""
    (run / 'bag').mkdir(parents=True)
    con = sqlite3.connect(run / 'bag' / 'bag_0.db3')
    con.execute('CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, '
                'type TEXT NOT NULL, serialization_format TEXT NOT NULL, '
                'offered_qos_profiles TEXT NOT NULL)')
    con.execute('CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, '
                'timestamp INTEGER NOT NULL, data BLOB NOT NULL)')
    ids = {}
    for topic, type_name, t, raw in messages:
        if topic not in ids:
            ids[topic] = len(ids) + 1
            con.execute("INSERT INTO topics VALUES (?, ?, ?, 'cdr', '')",
                        (ids[topic], topic, type_name))
        con.execute('INSERT INTO messages(topic_id, timestamp, data) VALUES (?, ?, ?)',
                    (ids[topic], int(round(t * 1e9)), raw))
    con.commit()
    con.close()


def _scene_depth():
    """Z-depth of the wall at WALL_X standing on the floor, as the camera sees it."""
    u = (np.arange(W) - CX) / FX
    v = (np.arange(H) - CY) / FY
    rays = np.stack(np.broadcast_arrays(u[None, :], v[:, None], np.ones((1, 1))), axis=-1)
    d = rays @ R_OPTICAL.T
    with np.errstate(divide='ignore', invalid='ignore'):
        wall = (WALL_X - CAM_T[0]) / d[..., 0]
        floor = -CAM_T[2] / d[..., 2]
    wall = np.where(wall > 0, wall, np.inf)
    floor = np.where(floor > 0, floor, np.inf)
    return np.minimum(wall, floor).astype(np.float32)


def _write_rosbag(run, messages):
    """Write the same through rosbag2_py, metadata.yaml included, for its own reader."""
    rosbag2_py = pytest.importorskip('rosbag2_py')
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(run / 'bag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    topics = set()
    for topic, type_name, t, raw in messages:
        if topic not in topics:
            topics.add(topic)
            meta = {'name': topic, 'type': type_name, 'serialization_format': 'cdr'}
            try:
                writer.create_topic(rosbag2_py.TopicMetadata(**meta))
            except TypeError:   # rosbag2 after Humble wants the topic id first
                writer.create_topic(rosbag2_py.TopicMetadata(id=len(topics), **meta))
        writer.write(topic, raw, int(round(t * 1e9)))
    del writer   # closing writes metadata.yaml (Humble's binding has no close())


def _recorded_run(tmp_path, ros, depth=True, frames=6, write=_write_bag):
    serialize, sensor, tf2, geo = ros

    def stamped(msg, t, frame):
        msg.header.stamp.sec = int(t)
        msg.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
        msg.header.frame_id = frame
        return msg

    tfm = tf2.TFMessage()
    for child, xyz, quat in (('cam_optical', CAM_T, OPTICAL_Q),
                             ('laser', LASER_T, (0.0, 0.0, 0.0, 1.0))):
        ts = geo.TransformStamped()
        ts.header.frame_id, ts.child_frame_id = 'base_link', child
        ts.transform.translation.x, ts.transform.translation.y, \
            ts.transform.translation.z = xyz
        ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z, \
            ts.transform.rotation.w = quat
        tfm.transforms.append(ts)
    info = stamped(sensor.CameraInfo(), 100.0, 'cam_optical')
    info.width, info.height = W, H
    info.distortion_model = 'plumb_bob'
    info.d = [0.0] * 5
    info.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
    messages = [('/tf_static', 'tf2_msgs/msg/TFMessage', 99.0, serialize(tfm)),
                ('/camera/camera_info', 'sensor_msgs/msg/CameraInfo', 100.0, serialize(info))]
    jpeg = array.array('B', GREY_JPEG)
    angles = -math.pi / 2 + np.arange(181) * math.pi / 180
    ranges = [(WALL_X - LASER_T[0]) / math.cos(a) if math.cos(a) > 0.2 else math.inf
              for a in angles]
    for i in range(frames):
        t = 100.0 + 0.2 * i
        for topic in ('/camera/debug_image/clean/compressed', '/camera/debug_image/compressed'):
            img = stamped(sensor.CompressedImage(), t, 'cam_optical')
            img.format, img.data = 'jpeg', jpeg
            messages.append((topic, 'sensor_msgs/msg/CompressedImage', t + 0.05, serialize(img)))
        scan = stamped(sensor.LaserScan(), t + 0.01, 'laser')
        scan.angle_min, scan.angle_max = float(angles[0]), float(angles[-1])
        scan.angle_increment = math.pi / 180
        scan.range_min, scan.range_max = 0.05, 30.0
        scan.ranges = [float(r) for r in ranges]
        messages.append(('/scan', 'sensor_msgs/msg/LaserScan', t + 0.02, serialize(scan)))
        if depth:
            d = stamped(sensor.Image(), t + 0.033, 'cam_optical')
            d.height, d.width, d.encoding, d.step = H, W, '32FC1', W * 4
            d.data = array.array('B', _scene_depth().tobytes())
            messages.append((mo.DEPTH_TOPIC, 'sensor_msgs/msg/Image', t + 0.1, serialize(d)))
    run = tmp_path / '2026-10-22T01-00-00_mission-overlay'
    write(run, messages)
    return run


def test_a_run_recorded_the_default_way_gets_overlay_stills_of_its_camera_frames(
        tmp_path, ros):
    run = _recorded_run(tmp_path, ros)
    paths, reason = mo.write_snapshots(run, tmp_path / 'snaps', limit=3,
                                       style=mo.Style(legend=False))
    assert reason is None
    assert [p.rsplit('/', 1)[1] for p in paths] == [
        f'2026-10-22T01-00-00_mission-overlay.{n:03d}.jpg' for n in range(3)]
    for path in paths:
        img = cv2.imread(path)
        assert img.shape == (H, W, 3)
        # The mask was rebuilt from the depth: the wall above, the floor below.
        b, g, r = (int(c) for c in img[20, 60])
        assert b - r > 40 and g - r > 40
        b, g, r = (int(c) for c in img[75, 60])
        assert g - r > 15 and g - b > 15


def test_a_run_without_a_mask_gets_no_overlay_stills(tmp_path, ros):
    run = _recorded_run(tmp_path, ros, depth=False)
    paths, reason = mo.write_snapshots(run, tmp_path / 'snaps')
    assert paths == []
    assert reason.startswith('no scene mask')
    assert not (tmp_path / 'snaps').exists()


@pytest.mark.skipif(shutil.which('ffmpeg') is None, reason='needs ffmpeg')
def test_the_video_of_a_run_without_a_mask_is_its_recorded_debug_frames(
        tmp_path, ros, capsys):
    run = _recorded_run(tmp_path, ros, depth=False)
    out = mdv.render_run(run, tmp_path / 'video')
    assert out is not None and out.stat().st_size > 0
    captured = capsys.readouterr()
    assert 'the recorded /camera/debug_image/compressed frames instead' in captured.err
    assert '6 frames' in captured.out


def _finalize_snapshots(tmp_path, ros, depth):
    """mission_logger_node's finalize step on a recorded run: (stills, log lines)."""
    node_module = pytest.importorskip('f1tenth_logger.mission_logger_node')
    run = _recorded_run(tmp_path, ros, depth=depth, write=_write_rosbag)
    log = []
    logger = SimpleNamespace(info=log.append, warn=log.append, error=log.append)
    node = SimpleNamespace(storage_id='sqlite3', get_logger=lambda: logger)
    node_module.MissionLoggerNode._write_snapshots(
        node, {'run_dir': str(run), 'bag_path': str(run / 'bag'), 'run_id': run.name})
    return sorted((run / 'snapshots').glob('*.jpg')), log


def test_the_logger_keeps_the_recorded_stills_for_a_run_without_a_mask(tmp_path, ros):
    # What every older run relies on: no overlay, then exactly the stills the
    # logger always wrote, the recorded debug JPEGs byte for byte.
    paths, log = _finalize_snapshots(tmp_path, ros, depth=False)
    assert len(paths) == 6
    assert all(p.read_bytes() == GREY_JPEG for p in paths)
    assert any(line.startswith('no overlay snapshots (no scene mask') for line in log)


def test_the_logger_writes_overlay_stills_for_a_run_with_a_mask(tmp_path, ros):
    paths, log = _finalize_snapshots(tmp_path, ros, depth=True)
    assert len(paths) == 6
    assert not any(p.read_bytes() == GREY_JPEG for p in paths)
    assert any(line.startswith('6 overlay snapshot(s)') for line in log)


def _decode_stamp(raw):
    if len(raw) != 8:
        raise ValueError('not a stamp')
    return struct.unpack('<d', raw)


def test_a_message_is_found_by_its_stamp_however_late_it_was_received(tmp_path):
    # 20 Hz for 20 s, each message just its stamp. The recorder falls 0.7 s
    # behind for 2 s and catches up (/scan of corridor10m_20261019T131815
    # arrived -0.002 to 0.679 s after its stamp); then its clock runs at twice
    # the stamps' rate (a sim at half real time). One message does not decode.
    stamps = [100.0 + 0.05 * i for i in range(400)]
    rows, last = [], 0.0
    for i, s in enumerate(stamps):
        recv = s + (0.7 if 100 <= i < 140 else 0.0) if i < 200 else 1000.0 + 2.0 * (s - 110.0)
        last = max(recv, last + 1e-4)
        rows.append(('/x', 'test/Stamp', last, b'bad' if i == 50 else struct.pack('<d', s)))
    _write_bag(tmp_path / 'run', rows)
    bag = mo.Bag([tmp_path / 'run' / 'bag' / 'bag_0.db3'])
    try:
        topic = mo.Topic(bag, '/x', _decode_stamp)
        found = [topic.nearest(s + 0.01, 0.02) for s in stamps]
        assert found == [None if i == 50 else i for i in range(len(stamps))]
        assert topic.nearest(stamps[0] - 0.5, 0.1) is None
        assert topic.nearest(stamps[-1] + 0.5, 0.1) is None
        # One lookup reads around the answer, not the run.
        fresh = mo.Topic(bag, '/x', _decode_stamp)
        assert fresh.nearest(stamps[120], 0.01) == 120
        assert len(fresh._stamps) < 40
    finally:
        bag.close()


def test_neighbouring_stills_that_pick_across_each_other_come_out_in_time_order():
    # 8 frames, 5 stills at frames 0, 2, 4, 5, 7. Frame 5's mask is the
    # nearest in time, then frame 4's: the still for frame 2 takes frame 5,
    # the one for frame 4 takes frame 4, before it in time.
    offsets = {5: 0.0, 4: 0.01}
    picked = mo.pick_stills(8, lambda i: offsets.get(i, 0.08), limit=5, search=3)
    assert picked == [0, 4, 5, 6, 7]


def test_a_run_without_a_params_snapshot_masks_with_stack_params_not_node_code_defaults():
    # The launch files hand front_clearance_node stack_params.yaml: a bag
    # with no snapshot is rebuilt with the traced self polygons the car used,
    # not the node's '' (the lidar_exclusion rectangle), and a key the run's
    # snapshot has wins over today's yaml.
    get_value = pytest.importorskip('f1tenth_params.param_defaults').get_value
    for key in mo.MASK_PARAMS:
        get_value(key)
    today = mo.RebuiltMask(None, None, None, None, None, {})
    assert today.polygons == (
        mo._polygons(get_value('front_clearance_scene_mask_self_polygons'))
        + mo._polygons(get_value('front_clearance_scene_mask_occluded_polygons')))
    assert today.floor_r_max == get_value('front_clearance_inferred_floor_r_max_m')
    run = mo.RebuiltMask(None, None, None, None, None, {
        'front_clearance_scene_mask_self_polygons': '',
        'front_clearance_inferred_floor_r_max_m': 2.5})
    assert run.polygons[0] == sm.rect_polygon(*(get_value('lidar_exclusion_' + k) for k in (
        'x_min', 'y_min', 'x_max', 'y_max')))
    assert run.floor_r_max == 2.5


def test_the_style_defaults_are_stack_params_mission_report_overlay_defaults():
    # A report host without f1tenth_params draws with Style(): it must draw
    # what the car draws. Change the yaml and Style together.
    param_defaults = pytest.importorskip('f1tenth_params.param_defaults')
    style = mo.Style()
    assert param_defaults.get_value('mission_report_overlay_alpha') == style.alpha == 0.3
    assert param_defaults.get_value('mission_report_overlay_point_px') == style.point_px == 2
    assert param_defaults.get_value('mission_report_overlay_legend') is style.legend is True


def test_module_imports_nothing_from_ros():
    # mission_debug_video's rule: the report host may have no ROS at all.
    tree = ast.parse(open(mo.__file__).read())
    roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            roots.update(a.name.split('.')[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            roots.add(node.module.split('.')[0])
    assert not roots & {'rclpy', 'rosbag2_py', 'rosidl_runtime_py', 'sensor_msgs',
                        'std_msgs', 'cv_bridge', 'tf2_ros'}
