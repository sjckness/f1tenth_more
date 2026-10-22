from glob import glob
import os

from setuptools import setup

package_name = 'f1tenth_telegram'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'systemd'), glob('systemd/*.service')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andreas',
    maintainer_email='andreas21steffens@gmail.com',
    description='Telegram control panel for the car, over ssh and f1tenth_ctl.',
    license='MIT',
    # The 'test' extra is what makes colcon run pytest (CLAUDE.md, Testing).
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'f1tenth_telegram = f1tenth_telegram.__main__:main',
        ],
    },
)
