import os
from glob import glob

from setuptools import setup

package_name = 'f1tenth_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andreas',
    maintainer_email='andreas21steffens@gmail.com',
    description='Localization (EKF / state estimation) for the F1TENTH vehicle. '
                 'Owns robot_localization EKF bringup via launch/ekf_launch.py.',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
