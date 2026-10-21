from glob import glob
import os

from setuptools import setup

package_name = 'f1tenth_comms'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andreas',
    maintainer_email='andreas21steffens@gmail.com',
    description='The car side of the link to the base: Wi-Fi coverage map and heartbeat.',
    license='MIT',
    # The 'test' extra is what makes colcon run pytest (CLAUDE.md, Testing).
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'wifi_coverage_node = f1tenth_comms.coverage_node:main',
            'link_manager = f1tenth_comms.link_manager_node:main',
        ],
    },
)
