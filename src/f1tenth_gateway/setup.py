from glob import glob
import os

from setuptools import setup

package_name = 'f1tenth_gateway'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'requirements.txt']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'systemd'), glob('systemd/*.service')),
        (os.path.join('share', package_name, 'bin'), ['bin/f1tenth_gateway']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andreas',
    maintainer_email='andreas21steffens@gmail.com',
    description='Gateway for the F1Tenth Lab Control PWA (REST + WebSocket + rclpy).',
    license='MIT',
    # The 'test' extra is what makes colcon run pytest (CLAUDE.md, Testing).
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'f1tenth_gateway = f1tenth_gateway.__main__:main',
        ],
    },
)
