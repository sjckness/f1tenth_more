import os
from glob import glob

from setuptools import setup

package_name = 'f1tenth_control'

setup(
    name=package_name,
    version='0.0.1',
    # Was a pure metapackage (packages=[]) until the avoidance pass. It now
    # owns ONE node, hard_stop_node: the last-resort LiDAR stop. It lives here
    # rather than in f1tenth_behavior or f1tenth_perception on purpose -- see
    # hard_stop.py's docstring for why it must not be a BT leaf, and
    # ackermann_mux.launch.py for why it launches beside the mux it feeds (if
    # the mux is down nothing can stop the car anyway, so sharing that
    # component's lifecycle is the honest coupling). mpc_controller still lives
    # in mpc_controller/ as its own ament_python package.
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
    description=('F1TENTH control: the drive-command mux wiring, the MPC launch '
                 'files, and the last-resort LiDAR stop (hard_stop_node).'),
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
            'hard_stop_node = f1tenth_control.hard_stop_node:main',
        ],
    },
)
