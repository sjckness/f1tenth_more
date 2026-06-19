from glob import glob

from setuptools import find_packages, setup

package_name = 'mpc_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install the track CSVs so track_file resolves under the share dir.
        ('share/' + package_name + '/tracks', glob('tracks/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fabiocar',
    maintainer_email='fabiocar@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
    'mpc_node = mpc_controller.mpc_node:main',
    'trajectory_mpc_node = mpc_controller.trajectory_mpc_node:main',
	'kinematic_mpc_node = mpc_controller.kinematic_mpc_node:main',
    'frenet_mpc_node = mpc_controller.frenet_mpc_node:main',    
    'andre_mpc_node = mpc_controller.andre_mpc_node:main',
    'andre_mpc_opt_node = mpc_controller.andre_mpc_opt_node:main',
    'track_mpc_controller = mpc_controller.track_mpc_opt_node:main',
    ],
   },
)
