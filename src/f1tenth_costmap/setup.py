import os
from glob import glob

from setuptools import setup

package_name = 'f1tenth_costmap'

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
    description='Two-layer costmap (occupancy from slam_toolbox + semantic from '
                'detected objects) plus a combined PNG-image render for Foxglove.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'semantic_layer_node = f1tenth_costmap.semantic_layer_node:main',
            'costmap_renderer_node = f1tenth_costmap.costmap_renderer_node:main',
            'costmap_boundary_node = f1tenth_costmap.costmap_boundary_node:main',
        ],
    },
)
