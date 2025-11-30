import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'dynamic_obstacle_isaacsim_spawning'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='.', include=['isaacsim_scripts', 'isaacsim_scripts.*']),
    package_dir={'': '.'},
    data_files=[
        (f'share/{package_name}', ['package.xml']),
        (f'share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ali Jafari Fesharaki',
    maintainer_email='ali.jafari.fesh@gmail.com',
    description='Dynamic obstacle spawning for Isaac Sim with ROS2 Control integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_publisher = isaacsim_scripts.trajectory_publisher:main',
            'isaacsim_spawner = isaacsim_scripts.isaacsim_spawner:main',
        ],
    },
)
