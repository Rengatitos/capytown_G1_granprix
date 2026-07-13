#!/usr/bin/env python3
"""Version SOLO TEXTO de lidar_viz.py: imprime la distancia minima del
sector FRENTE en la terminal, sin ventana grafica (util por SSH puro,
sin VNC/X11). No forma parte del paquete ROS2, no requiere colcon build.

Uso (dentro del contenedor):

    python3 lidar_front_text.py
    python3 lidar_front_text.py --ros-args -p front_offset_deg:=0.0 -p invert_left_right:=false

Acerca el robot de frente a mano hacia una pared real hasta que el
parachoques la toque, y anota que numero muestra la terminal justo en
ese instante -- ese es tu umbral_colision_m minimo seguro real.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import LaserScan

FRENTE_LO_DEG = -20.0
FRENTE_HI_DEG = 20.0


class LidarFrontTextNode(Node):

    def __init__(self):
        super().__init__('lidar_front_text')
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
        self.get_logger().info('Ctrl+C para cerrar. Acerca el robot de frente a una pared...')

    def _on_scan(self, msg: LaserScan) -> None:
        ranges = np.asarray(msg.ranges, dtype=float)
        n = len(ranges)
        idx = np.arange(n, dtype=float)
        a = msg.angle_min + idx * msg.angle_increment
        a = np.mod(a + math.pi, 2 * math.pi) - math.pi
        robot_angles = self._sign * (a - self._front_offset_rad)
        robot_angles = np.mod(robot_angles + math.pi, 2 * math.pi) - math.pi

        lo, hi = math.radians(FRENTE_LO_DEG), math.radians(FRENTE_HI_DEG)
        mask = (robot_angles >= lo) & (robot_angles <= hi)
        mask &= np.isfinite(ranges) & (ranges >= msg.range_min)
        mask &= ranges <= min(msg.range_max, self._max_range)

        distancia = float(np.min(ranges[mask])) if np.any(mask) else float('inf')
        texto = f'{distancia:.3f} m' if math.isfinite(distancia) else '--- (sin lectura valida)'
        print(f'\rFRENTE: {texto}          ', end='', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = LidarFrontTextNode()
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
