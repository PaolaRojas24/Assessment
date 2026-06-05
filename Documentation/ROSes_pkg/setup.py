import os
from glob import glob
from setuptools import setup

package_name = 'ROSes_pkg'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xyg',
    maintainer_email='xyg122413@gmail.com',
    description='Lightweight line-follower CSI camera publisher for the QCar.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'csi_lf = ROSes_pkg.csinode_lf:main',
            'imu_external = ROSes_pkg.imu_external:main',
            'odom_kalman = ROSes_pkg.odom_kalman:main',
        ],
    },
)
