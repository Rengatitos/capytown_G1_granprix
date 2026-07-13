#!/usr/bin/env python3
"""Version SOLO TEXTO de lidar_viz.py para los 4 sectores (frente,
derecha, izquierda, atras) a la vez -- util por SSH puro, sin
VNC/X11. No forma parte del paquete ROS2, no requiere colcon build.

Uso (dentro del contenedor), probando la calibracion actual:

    python3 lidar_zones_text.py
    python3 lidar_zones_text.py --ros-args -p front_offset_deg:=0.0 -p invert_left_right:=false

Procedimiento: con el robot QUIETO, toca cada lado FISICO real (frente,
atras, derecha, izquierda) contra una pared, uno a la vez, y anota que
sector de la terminal baja a ~0 en cada caso. El sector que baja a ~0
cuando tocas el FRENTE fisico deberia llamarse FRENTE en la terminal --
si se enciende otro (por ejemplo ATRAS o DERECHA), ese es el offset/
inversion real que hace falta, sin importar lo que diga el historial
del YAML.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import LaserScan

SECTORES = [
    ('FRENTE', -20.0, 20.0),
    ('DERECHA', -110.0, -70.0),
    ('IZQUIERDA', 70.0, 110.0),
    ('ATRAS', 160.0, -160.0),  # cruza +-180
]


class LidarZonesTextNode(Node):

    def __init__(self):
        super().__init__('lidar_zones_text')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('front_offset_deg', 0.0)
        self.declare_parameter('invert_left_right', False)
        self.declare_parameter('max_range_m', 2.5)

        self._front_offset_rad = math.radians(self.get_parameter('front_offset_deg').value)
        self._sign = -1 if self.get_parameter('invert_left_right').value else 1
        self._max_range = float(self.get_parameter('max_range_m').value)

        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._on_scan,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.get_logger().info(
            f'Ctrl+C para cerrar. offset={self.get_parameter("front_offset_deg").value} '
            f'invert={self.get_parameter("invert_left_right").value}. '
            'Toca cada lado fisico contra una pared, uno a la vez.'
        )

    def _zone_min(self, ranges, robot_angles, range_min, range_max, lo_deg, hi_deg):
        lo, hi = math.radians(lo_deg), math.radians(hi_deg)
        if lo <= hi:
            mask = (robot_angles >= lo) & (robot_angles <= hi)
        else:
            mask = (robot_angles >= lo) | (robot_angles <= hi)
        mask &= np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
        return float(np.min(ranges[mask])) if np.any(mask) else float('inf')

    def _on_scan(self, msg: LaserScan) -> None:
        ranges = np.asarray(msg.ranges, dtype=float)
        n = len(ranges)
        idx = np.arange(n, dtype=float)
        a = msg.angle_min + idx * msg.angle_increment
        a = np.mod(a + math.pi, 2 * math.pi) - math.pi
        robot_angles = self._sign * (a - self._front_offset_rad)
        robot_angles = np.mod(robot_angles + math.pi, 2 * math.pi) - math.pi

        range_max_use = min(msg.range_max, self._max_range)
        partes = []
        for nombre, lo_deg, hi_deg in SECTORES:
            d = self._zone_min(ranges, robot_angles, msg.range_min, range_max_use, lo_deg, hi_deg)
            texto = f'{d:.2f}m' if math.isfinite(d) else '---'
            partes.append(f'{nombre}={texto}')

        print('\r' + '  |  '.join(partes) + '          ', end='', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = LidarZonesTextNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
