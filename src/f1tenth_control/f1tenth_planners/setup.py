from glob import glob
import os

from setuptools import setup

package_name = 'f1tenth_planners'

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
    description=('The two alternative planner-controllers behind planner_controller_mode: '
                 'corridor_rrt (the corridor with RRT* in the MPC place) and '
                 'p2p_geometric (Dubins point-to-point), both tracked by pure pursuit.'),
    license='MIT',
    # The 'test' extra is what makes colcon run pytest at all (CLAUDE.md).
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'corridor_rrt = f1tenth_planners.corridor_rrt_node:main',
            'p2p_geometric = f1tenth_planners.p2p_geometric_node:main',
        ],
    },
)
