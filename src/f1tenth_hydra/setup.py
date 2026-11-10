import os
from glob import glob

from setuptools import setup

package_name = 'f1tenth_hydra'


def tree(src):
    """data_files entries for every file under src, keeping the sub-directories."""
    out = {}
    for path in glob(os.path.join(src, '**', '*'), recursive=True):
        if os.path.isfile(path):
            out.setdefault(os.path.join('share', package_name, os.path.dirname(path)), []).append(path)
    return sorted(out.items())


setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ] + tree('config'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andreas',
    maintainer_email='andreas21steffens@gmail.com',
    description='Hydra 3D scene graph launch, configs and viewers for the F1TENTH stack.',
    license='MIT',
    # The 'test' extra is what makes colcon run pytest at all (CLAUDE.md).
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'hydra_synthetic_rgbd = f1tenth_hydra.synthetic_rgbd:main',
        ],
    },
)
