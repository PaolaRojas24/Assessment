import os
from glob import glob
from setuptools import setup

package_name = 'qcar_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xyg',
    maintainer_email='xyg122413@gmail.com',
    description='QCar control: command_mux + lane_follower bringup.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'command_mux = qcar_control.command_mux:main',
            'overtake_supervisor = qcar_control.overtake_supervisor:main',
            'obstacle_dodge = qcar_control.obstacle_dodge:main',
        ],
    },
)
