import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'f1tenth_behavior'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
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
    description='py_trees-based reactive safety-stop + Nav2 waypoint navigation for the '
                'F1TENTH stack.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behavior_executor_node = f1tenth_behavior.behavior_executor_node:main',
            'twist_to_ackermann_node = f1tenth_behavior.twist_to_ackermann_node:main',
        ],
    },
)
