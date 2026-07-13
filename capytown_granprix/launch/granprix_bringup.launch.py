"""Launch principal del Reto Final - Gran Prix CapyTown.

Lanza los 7 nodos del reto (camera_publisher, lidar_processor,
wall_follower, state_machine, metrics_logger, dashboard_server y,
opcionalmente, stop_sign_detector) con los parametros de
``config/granprix_params.yaml``. ``dashboard_server`` sirve una pagina
web local (ver dashboard_server_node.py) para ver en vivo el mapa
descubierto, la celda/heading actual, la camara y el LiDAR mientras el
robot navega -- puerto 8000 por defecto.

Este launch NO lanza el bringup del robot (driver LiDAR, driver de
motores, microROS): eso lo hace el paquete base del robot (ver
PROPIEDADES_ROBOT.md, ``capytown_esan bringup.launch.py`` /
``yahboomcar_bringup``) y debe correr antes, por separado. La camara
es la excepcion: este launch la levanta directamente con
``camera_publisher_node`` (driver propio, ver ese archivo), siempre
activa sin importar ``usar_camara`` -- ``usar_camara`` solo controla
si ademas se lanza ``stop_sign_detector_node`` (deteccion de PARE)
como consumidor de esas imagenes.

Argumentos:
    ronda        (1|2)        ronda de la competencia (ver DETALLE RETO 3.md)
    usar_camara  (true|false) activa el nodo de deteccion de PARE
    params_file  (ruta)       archivo de parametros a usar

Ejemplos:
    ros2 launch capytown_granprix granprix_bringup.launch.py
    ros2 launch capytown_granprix granprix_bringup.launch.py ronda:=2 usar_camara:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('capytown_granprix')
    default_params_file = os.path.join(pkg_share, 'config', 'granprix_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Archivo YAML de parametros del reto.',
    )
    ronda_arg = DeclareLaunchArgument(
        'ronda',
        default_value='1',
        description='1 = ronda de exploracion, 2 = ronda time attack.',
    )
    usar_camara_arg = DeclareLaunchArgument(
        'usar_camara',
        default_value='true',
        description='Activa el nodo de deteccion de PARE por camara.',
    )

    params_file = LaunchConfiguration('params_file')
    ronda = LaunchConfiguration('ronda')
    usar_camara = LaunchConfiguration('usar_camara')

    camera_publisher_node = Node(
        package='capytown_granprix',
        executable='camera_publisher_node',
        name='camera_publisher',
        output='screen',
        parameters=[params_file],
    )

    lidar_processor_node = Node(
        package='capytown_granprix',
        executable='lidar_processor_node',
        name='lidar_processor',
        output='screen',
        parameters=[params_file],
    )

    wall_follower_node = Node(
        package='capytown_granprix',
        executable='wall_follower_node',
        name='wall_follower',
        output='screen',
        parameters=[params_file],
    )

    state_machine_node = Node(
        package='capytown_granprix',
        executable='state_machine_node',
        name='state_machine',
        output='screen',
        parameters=[
            params_file,
            {
                'usar_camara': ParameterValue(usar_camara, value_type=bool),
                'ronda': ParameterValue(ronda, value_type=int),
            },
        ],
    )

    stop_sign_detector_node = Node(
        package='capytown_granprix',
        executable='stop_sign_detector_node',
        name='stop_sign_detector',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(usar_camara),
    )

    metrics_logger_node = Node(
        package='capytown_granprix',
        executable='metrics_logger_node',
        name='metrics_logger',
        output='screen',
        parameters=[params_file, {'ronda': ParameterValue(ronda, value_type=int)}],
    )

    dashboard_server_node = Node(
        package='capytown_granprix',
        executable='dashboard_server_node',
        name='dashboard_server',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        params_file_arg,
        ronda_arg,
        usar_camara_arg,
        camera_publisher_node,
        lidar_processor_node,
        wall_follower_node,
        state_machine_node,
        stop_sign_detector_node,
        metrics_logger_node,
        dashboard_server_node,
    ])
