from glob import glob

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
        ('share/' + package_name + '/launch', glob('launch/*launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andreas',
    maintainer_email='andreas21steffens@gmail.com',
    description='Gazebo Fortress simulation for the F1TENTH vehicle.',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Adapts the real-stack /drive (AckermannDriveStamped) contract to
            # the ros2_control Ackermann controller, and relays its odometry to
            # /odom so ekf.yaml can be reused verbatim.
            'drive_bridge = f1tenth_sim.drive_bridge:main',
        ],
    },
)
