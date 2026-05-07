from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'line_follower_real'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xyg',
    maintainer_email='xyg122413@gmail.com',
    description='Line follower launch and config for the physical QCar',
    license='MIT',
    entry_points={
        'console_scripts': [
            'image_relay = line_follower_real.image_relay:main',
        ],
    },
)
