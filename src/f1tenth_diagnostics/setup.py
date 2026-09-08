import os
from glob import glob

from setuptools import setup

package_name = 'f1tenth_diagnostics'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andreas',
    maintainer_email='andreas21steffens@gmail.com',
    description='Calibration and diagnostic tooling for the F1TENTH stack.',
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
            'gyro_bias_calibration_node = '
            'f1tenth_diagnostics.gyro_bias_calibration_node:main',
            'sensor_covariance_calibration_node = '
            'f1tenth_diagnostics.sensor_covariance_calibration_node:main',
            'slam_pose_covariance_calibration_node = '
            'f1tenth_diagnostics.slam_pose_covariance_calibration_node:main',
            'battery_voltage_check_node = '
            'f1tenth_diagnostics.battery_voltage_check_node:main',
            'system_observer_node = '
            'f1tenth_diagnostics.system_observer_node:main',
            'diagnostics_server_node = '
            'f1tenth_diagnostics.diagnostics_server_node:main',
            'ekf_cost_observer_node = '
            'f1tenth_diagnostics.ekf_cost_observer_node:main',
        ],
    },
)
