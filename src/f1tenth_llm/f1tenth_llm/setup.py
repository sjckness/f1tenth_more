from setuptools import setup

package_name = 'f1tenth_llm'

setup(
    name=package_name,
    version='0.0.1',
    # Metapackage: no Python modules. llm_mpc_tuner lives in llm_mpc_tuner/ and
    # is built as its own ament_python package.
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
    description='Metapackage aggregating the F1TENTH LLM-based packages (MPC tuner).',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
