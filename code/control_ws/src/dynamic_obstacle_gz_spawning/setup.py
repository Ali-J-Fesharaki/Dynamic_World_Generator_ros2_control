import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'dynamic_obstacle_gz_spawning'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(where='.', include=['multi_object_scripts', 'multi_object_scripts.*']),
    package_dir={'': '.'},
    data_files=[
        (f'share/{package_name}', ['package.xml']),
        (f'share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch/nav2_bringup'), glob('launch/nav2_bringup/*.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ali Jafari Fesharaki',
    maintainer_email='ali.jafari.fesh@gmail.com',
    description='Multi obstacle simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_publisher = multi_object_scripts.trajectory_publisher:main',
        ],
    },
)
