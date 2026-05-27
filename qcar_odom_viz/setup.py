from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'qcar_odom_viz'

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
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Desarrollador',
    maintainer_email='usuario@example.com',
    description='Visualizacion en RViz de la odometria del QCar (/odom -> Path)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_path = qcar_odom_viz.odom_path:main',
        ],
    },
)
