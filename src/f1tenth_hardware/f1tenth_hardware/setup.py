import os
from glob import glob

from setuptools import setup

package_name = 'f1tenth_hardware'

setup(
    name=package_name,
    version='0.0.1',
    # Metapackage: no Python modules. The VESC packages live in vesc/ and are
    # built as their own (ament_cmake / ament_python) packages. It does own
    # launch/vesc.launch.py, which wires those packages' nodes together.
    packages=[],
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
    description='Metapackage aggregating the F1TENTH hardware-driver packages (VESC stack).',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
