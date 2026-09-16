import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'f1tenth_behavior'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'missions'), glob('missions/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andreas',
    maintainer_email='andreas21steffens@gmail.com',
    description='py_trees-based reactive safety-stop + Nav2 goal-pose navigation + scripted '
                'mission subtree for the F1TENTH stack.',
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
            'behavior_executor_node = f1tenth_behavior.behavior_executor_node:main',
            'twist_to_ackermann_node = f1tenth_behavior.twist_to_ackermann_node:main',
            'wait_for_trigger_service_node = '
            'f1tenth_behavior.wait_for_trigger_service_node:main',
        ],
    },
)
