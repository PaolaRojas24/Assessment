import os
from glob import glob
from setuptools import setup

package_name = 'qcar_opencv_dashboard'

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
    description='OpenCV dashboard for the QCar.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dashboard = qcar_opencv_dashboard.dashboard:main',
        ],
    },
)
