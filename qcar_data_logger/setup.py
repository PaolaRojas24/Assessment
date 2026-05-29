from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'qcar_data_logger'

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
    maintainer='Desarrollador',
    maintainer_email='usuario@example.com',
    description='Data logger de odometría del QCar (/odom -> CSV)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_logger = qcar_data_logger.odom_logger:main',
        ],
    },
)
