import os
from glob import glob

from setuptools import setup

package_name = 'f1tenth_params'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andreas',
    maintainer_email='andreas21steffens@gmail.com',
    description='Single shared source of launch-parameter defaults for the '
                 'f1tenth_more workspace.',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
