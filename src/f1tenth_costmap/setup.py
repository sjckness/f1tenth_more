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
            'semantic_layer_node = f1tenth_costmap.semantic_layer_node:main',
            'costmap_renderer_node = f1tenth_costmap.costmap_renderer_node:main',
            'costmap_boundary_node = f1tenth_costmap.costmap_boundary_node:main',
        ],
    },
)
