from glob import glob
import os

from setuptools import setup

package_name = 'f1tenth_ctl'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.env') + glob('config/*.yaml')),
        (os.path.join('share', package_name, 'bin'), ['bin/f1tenth_ctl']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andreas',
    maintainer_email='andreas21steffens@gmail.com',
    description='Process manager CLI for the car (stack, planner, logger, READY, e-stop).',
    license='MIT',
    # The 'test' extra is what makes colcon run pytest (CLAUDE.md, Testing).
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'f1tenth_ctl = f1tenth_ctl.cli:main',
        ],
    },
)
