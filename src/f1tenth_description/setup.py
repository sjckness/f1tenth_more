from glob import glob

from setuptools import setup

package_name = 'f1tenth_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Robot model + assets, shared by f1tenth_bringup (real) and f1tenth_sim.
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
        ('share/' + package_name + '/launch', glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andreas',
    maintainer_email='andreas21steffens@gmail.com',
    description='Robot description (URDF/xacro + meshes) for the F1TENTH vehicle.',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
