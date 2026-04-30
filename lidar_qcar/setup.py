from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lidar_qcar'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Registro del paquete en ament
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Configuraciones (RViz, etc.)
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Desarrollador',
    maintainer_email='usuario@example.com',
    description='Nodo LiDAR para el QCar con visualización en RViz2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lidar_node = lidar_qcar.lidar_node:main',
        ],
    },
)
