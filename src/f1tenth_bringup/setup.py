from setuptools import setup
import os
from glob import glob

package_name = 'f1tenth_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # ensure_discovery_server.py -- launch-time dependency (invoked by
        # stack_bringup.launch.py/supervisor_bringup.launch.py's own
        # discovery_server ExecuteProcess action), NOT a top-level scripts/
        # standalone developer tool like check_ekf_update_rate.py/
        # check_cpu_pinning.py -- installed here specifically so it's
        # reliably resolvable via get_package_share_directory() regardless
        # of workspace location, the same mechanism every other launch-time
        # file dependency in this codebase already uses.
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hongrui Zheng',
    maintainer_email='billyzheng.bz@gmail.com',
    description='Onboard drivers for vesc and sensors for F1TENTH vehicles.',
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
            'stack_startup_sequence = f1tenth_bringup.stack_startup_sequence:main',
            'component_supervisor_node = f1tenth_bringup.component_supervisor_node:main',
        ],
    },
)
