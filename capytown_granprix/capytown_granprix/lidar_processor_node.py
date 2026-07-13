#!/usr/bin/env python3
"""Nodo de lectura LiDAR.

Se suscribe a ``/scan`` (sensor_msgs/LaserScan, LiDAR MS200) y publica
``/lidar_zones`` (capytown_interfaces/LidarZones) con:

- la distancia minima detectada en cada zona angular de interes
  (frente, derecha delantera S1, derecha lateral, derecha trasera S2,
  izquierda) -- metodo de 2 puntos, usado por ALINEAR;
- un ajuste de RECTA por regresion a todos los puntos del lado
  derecho (``right_line_*``) -- metodo principal usado por
  ``wall_follower_node``, mucho mas robusto al ruido que 2 puntos
  sueltos (validado en ``sim_local/`` antes de portarlo aqui).

Este nodo NO decide nada: solo traduce la nube de puntos 360 grados en
los numeros que los demas nodos (wall_follower, state_machine)
consumen. Mantenerlo asi facilita calibrar el LiDAR (offset de
montaje, inversion izquierda/derecha) sin tocar la logica de control.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import LaserScan

from capytown_interfaces.msg import LidarZones
from capytown_granprix.lidar_utils import (
    ZoneWindow,
    compute_robot_frame_angles,
    compute_zone_distance,
    fit_wall_line,
)


class LidarProcessorNode(Node):

    def __init__(self):
        super().__init__('lidar_processor')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('output_topic', '/lidar_zones')
        self.declare_parameter('front_offset_deg', 0.0)
        self.declare_parameter('invert_left_right', False)
        self.declare_parameter('max_range_use_m', 4.0)
        self.declare_parameter('front_window_deg', [-15.0, 15.0])
        self.declare_parameter('right_front_window_deg', [-75.0, -45.0])
        self.declare_parameter('right_window_deg', [-110.0, -70.0])
        self.declare_parameter('right_rear_window_deg', [-135.0, -105.0])
        self.declare_parameter('left_window_deg', [70.0, 110.0])
        self.declare_parameter('left_front_window_deg', [45.0, 75.0])
        self.declare_parameter('left_rear_window_deg', [105.0, 135.0])
        self.declare_parameter('right_side_window_deg', [-110.0, -70.0])
        self.declare_parameter('left_side_window_deg', [70.0, 110.0])
        self.declare_parameter('min_puntos_linea', 6)
        self.declare_parameter('outlier_max_iter', 3)
        self.declare_parameter('outlier_residuo_m', 0.03)

        self._scan_topic = self.get_parameter('scan_topic').value
        self._output_topic = self.get_parameter('output_topic').value
        self._front_offset_rad = self.get_parameter('front_offset_deg').value * 3.141592653589793 / 180.0
        self._sign = -1 if self.get_parameter('invert_left_right').value else 1
        self._max_range_use_m = float(self.get_parameter('max_range_use_m').value)

        self._windows = {
            'front': ZoneWindow(*self.get_parameter('front_window_deg').value),
            'right_front': ZoneWindow(*self.get_parameter('right_front_window_deg').value),
            'right': ZoneWindow(*self.get_parameter('right_window_deg').value),
            'right_rear': ZoneWindow(*self.get_parameter('right_rear_window_deg').value),
            'left': ZoneWindow(*self.get_parameter('left_window_deg').value),
            'left_front': ZoneWindow(*self.get_parameter('left_front_window_deg').value),
            'left_rear': ZoneWindow(*self.get_parameter('left_rear_window_deg').value),
        }
        self._right_side_window = ZoneWindow(*self.get_parameter('right_side_window_deg').value)
        self._left_side_window = ZoneWindow(*self.get_parameter('left_side_window_deg').value)
        self._min_puntos_linea = int(self.get_parameter('min_puntos_linea').value)
        self._outlier_max_iter = int(self.get_parameter('outlier_max_iter').value)
        self._outlier_residuo_m = float(self.get_parameter('outlier_residuo_m').value)

        self._pub = self.create_publisher(
            LidarZones, self._output_topic, QoSPresetProfiles.SENSOR_DATA.value
        )
        self._sub = self.create_subscription(
            LaserScan, self._scan_topic, self._on_scan, QoSPresetProfiles.SENSOR_DATA.value
        )

        self.get_logger().info(
            f'lidar_processor listo: {self._scan_topic} -> {self._output_topic} '
            f'(offset={self.get_parameter("front_offset_deg").value} deg, '
            f'invertido={self.get_parameter("invert_left_right").value})'
        )

    def _on_scan(self, msg: LaserScan) -> None:
        range_max_use = min(msg.range_max, self._max_range_use_m)
        ranges = np.asarray(msg.ranges, dtype=float)
        robot_angles = compute_robot_frame_angles(
            ranges, msg.angle_min, msg.angle_increment, self._front_offset_rad, self._sign
        )

        out = LidarZones()
        out.header = msg.header

        for name in ('front', 'right_front', 'right', 'right_rear', 'left', 'left_front', 'left_rear'):
            distancia, valido = compute_zone_distance(
                ranges, robot_angles, msg.range_min, range_max_use, self._windows[name]
            )
            setattr(out, name, distancia)
            setattr(out, f'{name}_valid', valido)

        angulo, distancia_linea, valido_linea = fit_wall_line(
            ranges, robot_angles, msg.range_min, range_max_use,
            self._right_side_window, self._min_puntos_linea,
            self._outlier_max_iter, self._outlier_residuo_m,
        )
        out.right_line_angle_rad = angulo
        out.right_line_distance_m = distancia_linea
        out.right_line_valid = valido_linea

        angulo_izq, distancia_linea_izq, valido_linea_izq = fit_wall_line(
            ranges, robot_angles, msg.range_min, range_max_use,
            self._left_side_window, self._min_puntos_linea,
            self._outlier_max_iter, self._outlier_residuo_m,
        )
        out.left_line_angle_rad = angulo_izq
        out.left_line_distance_m = distancia_linea_izq
        out.left_line_valid = valido_linea_izq

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
