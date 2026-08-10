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
            'andre_mpc_node = mpc_controller.andre_mpc_node:main',
            'mpc_corr = mpc_controller.MPC_corr:main',
        ],
    },
)
