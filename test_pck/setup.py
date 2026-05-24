import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'test_pck'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xyg',
    maintainer_email='xyg122413@gmail.com',
    description='Test or calibration nodes',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_userCommand = test_pck.test_control:main',
            'lap_recorder = test_pck.lap_recorder:main',
        ],
    },
)
