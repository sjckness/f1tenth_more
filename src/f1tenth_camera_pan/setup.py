from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'f1tenth_camera_pan'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
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
    description='Motorized camera pan (aim law + controller + measured-angle TF).',
    license='MIT',
    # Without this extra, colcon runs `setup.py test` and executes 0 tests
    # (workspace CLAUDE.md). Keep it.
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'camera_pan_controller_node = f1tenth_camera_pan.camera_pan_controller_node:main',
            'camera_pan_tf_node = f1tenth_camera_pan.camera_pan_tf_node:main',
        ],
    },
)
