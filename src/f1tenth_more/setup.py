from setuptools import setup

package_name = 'f1tenth_more'

setup(
    name=package_name,
    version='0.0.1',
    # Metapackage: no Python modules, only aggregates other packages.
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andreas',
    maintainer_email='andreas21steffens@gmail.com',
    description='Top-level metapackage for the F1TENTH autonomous racing stack.',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
