from setuptools import setup
import os
from glob import glob

package_name = 'f1tenth_bringup'

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
    maintainer='Hongrui Zheng',
    maintainer_email='billyzheng.bz@gmail.com',
    description='Onboard drivers for vesc and sensors for F1TENTH vehicles.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stack_startup_sequence = f1tenth_bringup.stack_startup_sequence:main',
            'component_supervisor_node = f1tenth_bringup.component_supervisor_node:main',
        ],
    },
)
