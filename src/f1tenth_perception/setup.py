import os
from glob import glob

from setuptools import setup

package_name = 'f1tenth_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andreas',
    maintainer_email='andreas21steffens@gmail.com',
    description='Centralized perception (ZED2 + Hokuyo + YOLO) for the F1TENTH car.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector_node = f1tenth_perception.yolo_detector_node:main',
            'detection_3d_node = f1tenth_perception.detection_3d_node:main',
        ],
    },
)
