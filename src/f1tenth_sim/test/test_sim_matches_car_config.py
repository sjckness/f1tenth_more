"""Sim defaults that copy a real-car config value must stay equal to it.

Each of these is a coupling (see the workspace CLAUDE.md): changing the car's
value in vesc.yaml / steering_calibration.yaml / mux.yaml means updating the
sim in the same commit, and this file is what notices when that is forgotten.
Reads the source trees directly, so it needs no built workspace or ROS.
"""
import ast
import math
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

from f1tenth_sim.kinematics import (
    servo_envelope,
    STEERING_MAX_RAD,
    STEERING_MIN_RAD,
)
import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
SRC = PKG.parent
VESC_YAML = SRC / 'f1tenth_bringup' / 'config' / 'vesc.yaml'
MUX_YAML = SRC / 'f1tenth_bringup' / 'config' / 'mux.yaml'
STEERING_YAML = (SRC / 'f1tenth_hardware' / 'f1tenth_hardware' / 'config'
                 / 'steering_calibration.yaml')
DESC = SRC / 'f1tenth_description'
URDF = DESC / 'urdf'
SIM_MOUNTS_YAML = DESC / 'config' / 'sim_sensor_mounts.yaml'
DESCRIPTION_LAUNCH = DESC / 'launch' / 'description.launch.py'
CAMERA_LAUNCH = SRC / 'f1tenth_perception' / 'launch' / 'camera.launch.py'
VESC_LAUNCH = SRC / 'f1tenth_hardware' / 'f1tenth_hardware' / 'launch' / 'vesc.launch.py'


def _load(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _ros_params(path, node):
    return _load(path)[node]['ros__parameters']


def _drive_bridge_defaults():
    """declare_parameter(name, literal) defaults, read without importing rclpy."""
    tree = ast.parse((PKG / 'f1tenth_sim' / 'drive_bridge.py').read_text())
    out = {}
    for call in ast.walk(tree):
        if (isinstance(call, ast.Call)
                and getattr(call.func, 'attr', None) == 'declare_parameter'
                and isinstance(call.args[0], ast.Constant)
                and isinstance(call.args[1], ast.Constant)):
            out[call.args[0].value] = call.args[1].value
    return out


@pytest.fixture(scope='module')
def controller():
    return _ros_params(PKG / 'config' / 'controllers.yaml',
                       'ackermann_steering_controller')


def test_steering_clamp_equals_servo_limits_of_steering_calibration_yaml():
    cal = _ros_params(STEERING_YAML, '/**')
    lo, hi = servo_envelope(
        cal['servo_min'], cal['servo_max'],
        cal['steering_angle_to_servo_offset'],
        cal['steering_angle_to_servo_gain_left'],
        cal['steering_angle_to_servo_gain_right'])
    assert STEERING_MIN_RAD == pytest.approx(lo, abs=5e-5)
    assert STEERING_MAX_RAD == pytest.approx(hi, abs=5e-5)


@pytest.mark.parametrize('key', [
    'gyro_variance_x', 'gyro_variance_y', 'gyro_variance_z',
    'accel_variance_x', 'accel_variance_y', 'accel_variance_z'])
def test_sim_imu_covariance_default_equals_vesc_yaml(key):
    assert _drive_bridge_defaults()[key] == _ros_params(VESC_YAML, '/**')[key]


def test_sim_imu_frame_id_is_empty_like_vesc_driver():
    assert _drive_bridge_defaults()['imu_frame_id'] == ''


def test_sim_odom_pose_covariance_equals_vesc_to_odom(controller):
    odom = _ros_params(VESC_YAML, 'vesc_to_odom_node')
    cov = controller['pose_covariance_diagonal']
    assert (cov[0], cov[1], cov[5]) == (
        odom['x_variance'], odom['y_variance'], odom['yaw_variance'])


def test_sim_odom_vx_covariance_equals_vesc_to_odom(controller):
    odom = _ros_params(VESC_YAML, 'vesc_to_odom_node')
    assert controller['twist_covariance_diagonal'][0] == odom['vx_variance']


def test_controller_reference_timeout_equals_mux_timeout(controller):
    timeouts = {t['timeout'] for t in _ros_params(MUX_YAML, 'ackermann_mux')['topics'].values()}
    assert timeouts == {controller['reference_timeout']}


def _xacro_properties(name):
    """Literal <xacro:property name=.. value=..> values of one xacro file."""
    text = (URDF / name).read_text()
    return dict(re.findall(r'<xacro:property\s+name="(\w+)"\s+value="([^"$]*)"', text))


def _launch_wheelbase(path):
    """The 'wheelbase' literal in a launch file's parameters dict."""
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.Dict):
            for k, v in zip(node.keys, node.values):
                if isinstance(k, ast.Constant) and k.value == 'wheelbase':
                    return v.value
    raise AssertionError('no wheelbase parameter in ' + str(path))


def _static_tf_args_all(path):
    """static_transform_publisher Node name -> list of each node's arg list.

    A name can map to several nodes (description.launch.py has one
    static_baselink_to_laser per sim/car branch); only args that are literal
    lists are collected (the sim branch builds its args from the yaml at
    runtime, so it has no literal list here -- the car branch does).
    """
    out = {}
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.Call) and getattr(node.func, 'id', None) == 'Node':
            kw = {k.arg: k.value for k in node.keywords}
            if (isinstance(kw.get('executable'), ast.Constant)
                    and kw['executable'].value == 'static_transform_publisher'
                    and isinstance(kw['arguments'], ast.List)
                    and all(isinstance(e, ast.Constant) for e in kw['arguments'].elts)):
                out.setdefault(kw['name'].value, []).append(
                    [e.value for e in kw['arguments'].elts])
    return out


@pytest.fixture(scope='module')
def urdf_dims():
    return {k: float(v) for k, v in _xacro_properties('roboracer.urdf.xacro').items()
            if k in ('wheelbase', 'track', 'wheel_radius', 'wheel_width')}


def test_urdf_uses_the_f1tenth_reference_dimensions(urdf_dims):
    # f1tenth_simulator racecar.xacro: wheelbase 0.3302, width 0.2032,
    # wheel_radius 0.0508, wheel_length 0.0381 (track = width + wheel_length).
    assert urdf_dims == {'wheelbase': 0.3302, 'track': 0.2413,
                         'wheel_radius': 0.0508, 'wheel_width': 0.0381}


def test_drive_bridge_launch_and_controller_share_the_urdf_wheelbase(controller, urdf_dims):
    launch = _launch_wheelbase(PKG / 'launch' / 'sim_bringup.launch.py')
    assert (_drive_bridge_defaults()['wheelbase'] == launch == controller['wheelbase']
            == urdf_dims['wheelbase'])


def test_controller_track_and_wheel_radius_equal_the_urdf(controller, urdf_dims):
    assert controller['traction_track_width'] == urdf_dims['track']
    assert controller['steering_track_width'] == urdf_dims['track']
    assert controller['traction_wheels_radius'] == urdf_dims['wheel_radius']


@pytest.fixture(scope='module')
def sim_mounts():
    return yaml.safe_load(SIM_MOUNTS_YAML.read_text())


def _urdf_joint_xyz(joint_name):
    """Expand roboracer.urdf.xacro (sim config) and return a joint's origin xyz."""
    urdf = subprocess.check_output(
        ['xacro', str(URDF / 'roboracer.urdf.xacro'),
         'use_sim:=true', 'enable_sensors:=true', 'enable_camera_mock:=true',
         f'pkg_share:={DESC}', 'control_config:='], text=True)
    for joint in ET.fromstring(urdf).iter('joint'):
        if joint.get('name') == joint_name:
            return [float(v) for v in joint.find('origin').get('xyz').split()]
    raise AssertionError(f'no joint {joint_name} in the expanded URDF')


def test_urdf_sim_mounts_equal_the_sim_sensor_mounts_yaml(sim_mounts):
    # The simulated LiDAR/ZED links (sensors.xacro) must render from exactly the
    # yaml pose -- sensors.xacro loads the same file, this proves it round-trips.
    assert _urdf_joint_xyz('laser_joint') == pytest.approx(sim_mounts['laser']['xyz'])
    # Camera mount is now a pan: base_link -> camera_pan_base (fixed, the pivot)
    # -> zed2_camera_link (revolute, zero translation). pan=0 identity requires
    # the pivot = the old mount and the revolute origin at 0.
    assert _urdf_joint_xyz('camera_pan_base_joint') == pytest.approx(
        sim_mounts['camera_pan_pivot']['xyz'])
    assert _urdf_joint_xyz('camera_pan_joint') == pytest.approx([0.0, 0.0, 0.0])
    # The new mounts, pinned so an accidental edit to either side is caught.
    assert sim_mounts['laser']['xyz'] == [0.40, 0.0, 0.155]
    assert sim_mounts['camera_pan_pivot']['xyz'] == [0.36, 0.0, 0.25]
    # pan=0 is bit-identical to the old fixed mount: pivot == old zed2_camera_link.
    assert sim_mounts['camera_pan_pivot']['xyz'] == sim_mounts['zed2_camera_link']['xyz']


@pytest.mark.parametrize('launch, node, xyz, child', [
    (DESCRIPTION_LAUNCH, 'static_baselink_to_laser', [0.12, 0.0, 0.20], 'laser'),
    (CAMERA_LAUNCH, 'static_baselink_to_zed2', [0.12, 0.0, 0.15], 'zed2_camera_link'),
])
def test_cars_static_sensor_tfs_are_unchanged(launch, node, xyz, child):
    # The real car's TFs must NOT move with the sim mounts. description.launch.py
    # now has two static_baselink_to_laser nodes (car + sim); this reads the one
    # with the car's literal translation. rpy must be 0 (level, front-facing).
    for args in _static_tf_args_all(launch)[node]:
        if [float(a) for a in args[:3]] == xyz:
            assert args[6:] == ['base_link', child]
            assert [float(a) for a in args[3:6]] == [0.0, 0.0, 0.0]
            return
    raise AssertionError(f'{node} with car translation {xyz} not found in {launch}')


def test_lidar_housing_is_outside_the_zed_vertical_fov(sim_mounts):
    # The simulated LiDAR must not obscure the camera ACROSS THE WHOLE PAN RANGE:
    # its housing (hokuyo.stl, 50x50x70 mm centred on `laser`) must stay below
    # the ZED left lens's lower vertical-FOV edge for every pan angle in
    # +-pan_limit. The pan is a yaw about z at the pivot, so the lens position
    # rotates with it; a point's elevation below the (horizontal) optical axis
    # must stay outside the half-VFOV. Camera level; FOV from sensors.xacro.
    props = _xacro_properties('sensors.xacro')
    w, h, hfov = (float(props['zed_width']), float(props['zed_height_px']),
                  float(props['zed_hfov']))
    half_vfov = math.atan(math.tan(hfov / 2) * h / w)       # ~34.3 deg
    pan_limit = float(props['pan_limit'])                   # +-15 deg

    # The pan pivot and the left lens offset from it (zed_macro geometry:
    # optical_offset_x -0.01, baseline/2 +0.06, mount->centre +0.015). At pan=0
    # the pivot == the old mount, so the lens is where it has always been.
    pivot = sim_mounts['camera_pan_pivot']['xyz']
    lens_off = (-0.01, 0.06, 0.015)
    laser = sim_mounts['laser']['xyz']
    half = (0.050 / 2, 0.050 / 2, 0.070 / 2)

    def _min_below_for_pan(theta):
        c, s = math.cos(theta), math.sin(theta)
        # lens position after yawing the offset about z at the pivot
        lx = pivot[0] + c * lens_off[0] - s * lens_off[1]
        ly = pivot[1] + s * lens_off[0] + c * lens_off[1]
        lz = pivot[2] + lens_off[2]
        angs = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    cx = laser[0] + sx * half[0]
                    cy = laser[1] + sy * half[1]
                    cz = laser[2] + sz * half[2]
                    # component along the panned optical axis (cos,sin,0)
                    fwd = (cx - lx) * c + (cy - ly) * s
                    if fwd <= 0:
                        continue                      # behind the lens -> not in frame
                    down = lz - cz                    # below the (horizontal) axis
                    angs.append(math.atan2(down, fwd))
        return min(angs) if angs else math.pi / 2

    # Sweep the whole range; worst case is pan=0 (yaw shrinks the forward
    # component -> larger below-angle), but the lens also shifts, so check it.
    sweep = [(-pan_limit + i * (2 * pan_limit) / 20) for i in range(21)]
    worst = min(_min_below_for_pan(t) for t in sweep)
    assert worst > half_vfov, (
        f'LiDAR housing only {math.degrees(worst):.1f} deg below the lens over '
        f'+-{math.degrees(pan_limit):.1f} deg pan, inside the '
        f'{math.degrees(half_vfov):.1f} deg half-VFOV')
    # Design intent: comfortably clear across the full pan range. The +-15 deg
    # sweep over all 8 housing corners tightens the worst case to ~43.4 deg (was
    # ~45 deg at pan=0, x/z corners only) -- still ~9 deg below the 34.3 deg edge.
    assert math.degrees(worst) >= 43.0


def test_sim_zed_is_a_zed2i_named_zed2_like_camera_launch():
    props = _xacro_properties('sensors.xacro')
    text = (URDF / 'sensors.xacro').read_text()
    # zed_macro.urdf.xacro, model zed2i (wrapper v5.4.1)
    assert (props['zed_baseline'], props['zed_bottom_slope'],
            props['zed_optical_offset_x']) == ('0.12', '0.0', '-0.01')
    assert 'camera_model\': \'zed2i\'' in CAMERA_LAUNCH.read_text()
    for frame in ('zed2_camera_link', 'zed2_camera_center', 'zed2_left_camera_frame',
                  'zed2_left_camera_frame_optical'):
        assert f'<link name="{frame}"' in text
    assert text.count('<gz_frame_id>zed2_left_camera_frame_optical</gz_frame_id>') == 2


def test_sim_camera_streams_land_on_the_topics_the_stack_reads():
    # The two heavy Image streams are bridged onto sim-PC-local staging topics
    # (compressed across the LAN, see the compression test below), NOT the
    # canonical raw names. camera_info is tiny and crosses fine, so it keeps the
    # canonical names.
    entries = _load(PKG / 'config' / 'ros_gz_bridge_camera.yaml')
    assert {e['ros_topic_name']: e['ros_type_name'] for e in entries} == {
        '/sim/camera/image_raw': 'sensor_msgs/msg/Image',
        '/camera/camera_info': 'sensor_msgs/msg/CameraInfo',
        '/sim/camera/depth_raw': 'sensor_msgs/msg/Image',
        '/zed2/zed_node/depth/camera_info': 'sensor_msgs/msg/CameraInfo',
    }
    # what detection.launch.py subscribes to for depth
    detection = (SRC / 'f1tenth_perception' / 'launch' / 'detection.launch.py').read_text()
    assert "'depth_topic': '/zed2/zed_node/depth/depth_registered'" in detection
    assert "'depth_info_topic': '/zed2/zed_node/depth/camera_info'" in detection
    # every gz topic is one the URDF sensors actually publish
    text = (URDF / 'sensors.xacro').read_text()
    for e in entries:
        assert e['gz_topic_name'] in text


def test_camera_compression_round_trips_to_the_canonical_raw_topics():
    """Check the compressed camera topics match end to end across the machines.

    The bridge's staging topics are compressed on linus (sim_bringup) and
    decompressed on the Thor (sim_camera_tf) back to the exact raw names
    yolo_detector_node / detection.launch.py read on the car; the compressed
    topic names in between must agree (output/sim_camera_compression.md).
    """
    bringup = (PKG / 'launch' / 'sim_bringup.launch.py').read_text()
    simtf = (SRC / 'f1tenth_perception' / 'launch'
             / 'sim_camera_tf.launch.py').read_text()

    # linus compressors: staging raw -> compressed on the LAN topics
    assert "('/in', '/sim/camera/image_raw')" in bringup
    assert "('/out/compressed', '/camera/image_raw/compressed')" in bringup
    assert "('/in', '/sim/camera/depth_raw')" in bringup
    assert ("'/zed2/zed_node/depth/depth_registered/compressedDepth'") in bringup

    # Thor decompressors: the same compressed topics -> canonical raw names
    assert "('/in/compressed', '/camera/image_raw/compressed')" in simtf
    assert "('/out', '/camera/image_raw')" in simtf
    assert ("'/zed2/zed_node/depth/depth_registered/compressedDepth'") in simtf
    assert "('/out', '/zed2/zed_node/depth/depth_registered')" in simtf

    # transports are passed as parameters (positional is silently ignored)
    for text in (bringup, simtf):
        assert "'in_transport'" in text and "'out_transport'" in text


def test_drive_bridge_listens_behind_ackermann_mux():
    assert _drive_bridge_defaults()['drive_topic'] == '/ackermann_drive'


def test_controller_does_not_broadcast_odom_tf(controller):
    assert controller['enable_odom_tf'] is False


@pytest.mark.parametrize('config', ['ros_gz_bridge.yaml', 'ros_gz_bridge_camera.yaml'])
def test_gz_bridge_never_publishes_tf(config):
    entries = _load(PKG / 'config' / config)
    assert {e['ros_topic_name'] for e in entries}.isdisjoint({'/tf', '/tf_static'})
    assert {e['direction'] for e in entries} == {'GZ_TO_ROS'}
    assert all(e['gz_type_name'].startswith('gz.msgs.') for e in entries)
