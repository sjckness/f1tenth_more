import os
from glob import glob

from setuptools import setup

package_name = 'sensors_stack_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install URDF/XACRO so it is reachable via get_package_share_directory.
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf.xacro') + glob('urdf/*.urdf') + glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andreas Steffens',
    maintainer_email='andreas21steffens@gmail.com',
    description='URDF/XACRO sensor mounting frames for the F1TENTH car (ZED 2; LiDAR later).',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
