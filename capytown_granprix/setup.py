import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'capytown_granprix'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='frayderMM',
    maintainer_email='fraydermezamorveli@gmail.com',
    description=(
        'Navegacion por seguimiento de pared derecha para el laberinto '
        'Gran Prix CapyTown (Yahboom ROSMASTER R2, ROS 2 Humble).'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_processor_node = '
            'capytown_granprix.lidar_processor_node:main',
            'wall_follower_node = '
            'capytown_granprix.wall_follower_node:main',
            'state_machine_node = '
            'capytown_granprix.state_machine_node:main',
            'stop_sign_detector_node = '
            'capytown_granprix.stop_sign_detector_node:main',
            'metrics_logger_node = '
            'capytown_granprix.metrics_logger_node:main',
            'dashboard_server_node = '
            'capytown_granprix.dashboard_server_node:main',
            'camera_publisher_node = '
            'capytown_granprix.camera_publisher_node:main',
        ],
    },
)
