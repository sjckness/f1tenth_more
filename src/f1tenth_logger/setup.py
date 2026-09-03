import os
from glob import glob

from setuptools import setup

package_name = 'f1tenth_logger'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andreas',
    maintainer_email='andreas21steffens@gmail.com',
    description='Post-mission run logging and analysis for the F1TENTH stack: '
                'per-mission rosbag2 recording plus the replay/report tooling '
                'that reads those bags back.',
    license='MIT',
    # extras_require, NOT the tests_require this workspace's older setup.py
    # files carried: modern setuptools does not recognize tests_require (it
    # warns "Unknown distribution option" and ignores it), and colcon's
    # ament_python test step only invokes pytest for a package that declares a
    # 'test' extra. Without this it falls back to `setup.py test`, which runs
    # unittest discovery, finds nothing, and reports "Ran 0 tests ... OK" --
    # a green result that has tested nothing at all. Verified both ways on
    # this package. mpc_controller is the one package in this workspace that
    # already had it, which is why it is also the one whose tests actually run
    # under colcon test.
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # mission_logger_node is the name mission_logger.launch.py has
            # always invoked; mission_logger is the shorter alias the runs/
            # analyze_run CLI pair is named to match. Same main(), two names,
            # so moving the package did not break the launch file.
            'mission_logger_node = f1tenth_logger.mission_logger_node:main',
            'mission_logger = f1tenth_logger.mission_logger_node:main',
            # Was invoked as `python3 scripts/mission_replay_video.py`. That
            # path no longer exists, so it gets a console script rather than
            # silently losing its manual entry point.
            'mission_replay_video = f1tenth_logger.mission_replay_video:main',
        ],
    },
)
