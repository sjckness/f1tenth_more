from setuptools import setup
import os
from glob import glob

package_name = 'f1tenth_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install everything under maps/ (yaml + png/pgm image data + README).
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fabiocar',
    maintainer_email='fabiocar@todo.todo',
    description='Static map server and Nav2 navigation stack (planner/controller/'
                 'behavior servers, bt_navigator, lifecycle manager) for the F1tenth '
                 'stack.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [],
    },
)
