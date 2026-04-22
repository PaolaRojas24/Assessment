from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'control_helpers_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='dasc14@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'holonomic_joy_control = control_helpers_pkg.holonomic_joystick_control:main',
            'qcar2_joy_control = control_helpers_pkg.qcar2_joystick_controller:main',
            'lane_follower_q = control_helpers_pkg.lane_follower_q:main'
        ],
    },
)
