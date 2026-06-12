import os
from glob import glob

from setuptools import setup

package_name = 'sensors_stack_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Markdown docs (topics reference + tuning guide)
        ('share/' + package_name, glob('*.md')),
        # Python launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Parameter YAMLs
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # udev rules (installed by the user into /etc/udev/rules.d/, see file header)
        (os.path.join('share', package_name, 'udev'), glob('udev/*.rules')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andreas Steffens',
    maintainer_email='andreas21steffens@gmail.com',
    description='Bringup (launch/config/udev) for the F1TENTH sensor suite (ZED 2; LiDAR later).',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
