from setuptools import setup

package_name = 'sensors_stack'

# Pure metapackage: no Python modules and no build logic. It exists only to pull
# in the sub-packages (declared as <exec_depend> in package.xml), mirroring the
# role of the `vesc` metapackage. Built with ament_python per workspace policy.
setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andreas Steffens',
    maintainer_email='andreas21steffens@gmail.com',
    description='Metapackage for the F1TENTH sensor suite (ZED 2 camera; LiDAR later).',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
