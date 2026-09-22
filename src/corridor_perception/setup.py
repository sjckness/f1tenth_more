from setuptools import find_packages, setup

package_name = 'corridor_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='fabiocar',
    maintainer_email='fabio.carapellese@seamorphrobotics.com',
    description='Structural corridor perception from a 2D lidar (numpy only).',
    license='MIT',
    # Without this extra colcon never runs pytest here and reports
    # "Ran 0 tests ... OK" (see the workspace CLAUDE.md).
    extras_require={
        'test': [
            'pytest',
        ],
    },
)
