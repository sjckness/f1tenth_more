from glob import glob
import os

from setuptools import setup

package_name = 'f1tenth_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        # Vendored models (test room person / obstacles), one data_files
        # entry per directory so the tree is kept under share/models.
        *[('share/' + package_name + '/' + d, [f for f in glob(d + '/*') if os.path.isfile(f)])
          for d in sorted({os.path.dirname(f) for f in glob('models/**/*', recursive=True)
                           if os.path.isfile(f)})],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andreas',
    maintainer_email='andreas21steffens@gmail.com',
    description='Gazebo Harmonic simulation for the F1TENTH vehicle.',
    license='MIT',
    # Declares the 'test' extra colcon's ament_python test step looks for
    # before it will invoke pytest at all. Without it colcon falls back to
    # `setup.py test`, whose unittest discovery finds none of this package's
    # pytest-style tests and reports "Ran 0 tests ... OK" -- a green result
    # that ran nothing. Not tests_require: modern setuptools does not
    # recognize that argument (it warns and ignores it) and it never enabled
    # pytest either.
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # Stands in for ackermann_to_vesc / vesc_to_odom / vesc_driver:
            # /ackermann_drive -> controller reference, controller odometry
            # -> /odom, gz IMU -> /sensors/imu/raw (see drive_bridge.py).
            'drive_bridge = f1tenth_sim.drive_bridge:main',
            # Gazebo's 1 kHz clock (/sim/clock_raw) -> /clock at clock_rate,
            # so /clock does not flood the LAN (see clock_throttle.py).
            'clock_throttle = f1tenth_sim.clock_throttle:main',
            # /camera_pan/command -> the pan position controller; the gz joint
            # state -> /camera_pan/joint_state (see camera_pan_bridge.py).
            'camera_pan_bridge = f1tenth_sim.camera_pan_bridge:main',
            # /sim/ground_truth (car Odometry) -> /sim/ground_truth/car
            # (PoseStamped, like person/obstacle; see ground_truth_relay.py).
            'ground_truth_relay = f1tenth_sim.ground_truth_relay:main',
        ],
    },
)
