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
        # YOLO weights (*.pt/*.onnx/*.engine) consumed by yolo_detector_node's
        # model_path param -- detection.launch.py resolves it against this
        # package's *installed* share dir (its own __file__ at runtime), so
        # without this entry colcon build never copies models/ into install/
        # and the node silently falls back to passthrough mode (empty
        # detections) even though the files are right there in src/.
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andreas',
    maintainer_email='andreas21steffens@gmail.com',
    description='Centralized perception (ZED2 + Hokuyo + YOLO) for the F1TENTH car.',
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
            'yolo_detector_node = f1tenth_perception.yolo_detector_node:main',
            'detection_3d_node = f1tenth_perception.detection_3d_node:main',
            'obstacle_projector_node = f1tenth_perception.obstacle_projector_node:main',
            'front_depth_monitor_node = f1tenth_perception.front_depth_monitor_node:main',
        ],
    },
)
