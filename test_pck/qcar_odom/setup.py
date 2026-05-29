from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'qcar_odom'

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
    description='Odometria del QCar con IMU (ESP32 por serial) y EKF 2D',
    license='MIT',
    entry_points={
        'console_scripts': [
            'imu_publisher = qcar_odom.imu_publisher:main',
            'odom_kalman   = qcar_odom.odom_kalman:main',
        ],
    },
)
