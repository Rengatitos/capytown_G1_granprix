#!/usr/bin/env python3
"""Visualizador en vivo del LiDAR MS200 (diagnostico, no forma parte del
paquete ROS2 del reto). Dibuja con matplotlib los puntos crudos de
``/scan`` en el marco del ROBOT (frente arriba, izquierda a la
izquierda de la pantalla) y superpone 4 sectores angulares reducidos
(frente, derecha, izquierda, atras) con la distancia minima de cada
uno, para verificar visualmente si `front_offset_deg` / `invert_left_right`
estan bien calibrados antes de tocar el YAML del paquete principal.

Uso (dentro del contenedor, con el workspace *no* necesariamente
compilado -- este script no depende de capytown_granprix):

    python3 /root/yahboomcar_ws/src/reto-final/lidar_viz.py
    python3 lidar_viz.py --ros-args -p front_offset_deg:=0.0 -p invert_left_right:=false

Requiere entorno grafico (VNC, `-e DISPLAY=:0` al entrar al contenedor).
"""

import math

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import LaserScan

# Sectores reducidos para el diagnostico visual (mas angostos que los
# usados en produccion, solo para ver claramente los 4 lados).
SECTORES = [
    ('FRENTE', -20.0, 20.0, 'tab:green'),
    ('DERECHA', -110.0, -70.0, 'tab:red'),
    ('IZQUIERDA', 70.0, 110.0, 'tab:blue'),
    ('ATRAS', 160.0, -160.0, 'tab:orange'),  # cruza +-180
]


class LidarVizNode(Node):

    def __init__(self):
        super().__init__('lidar_viz')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('front_offset_deg', 0.0)
        self.declare_parameter('invert_left_right', False)
        self.declare_parameter('max_range_m', 2.5)

        self.front_offset_rad = math.radians(self.get_parameter('front_offset_deg').value)
        self.sign = -1 if self.get_parameter('invert_left_right').value else 1
        self.max_range = float(self.get_parameter('max_range_m').value)

        self.last_scan = None
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._on_scan,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

    def _on_scan(self, msg: LaserScan):
        self.last_scan = msg


def robot_frame_angles(scan: LaserScan, front_offset_rad: float, sign: int) -> np.ndarray:
    n = len(scan.ranges)
    idx = np.arange(n, dtype=float)
    a = scan.angle_min + idx * scan.angle_increment
    a = np.mod(a + math.pi, 2 * math.pi) - math.pi
    r = sign * (a - front_offset_rad)
    return np.mod(r + math.pi, 2 * math.pi) - math.pi


def zone_min_distance(ranges: np.ndarray, robot_angles: np.ndarray, range_min: float,
                       range_max: float, lo_deg: float, hi_deg: float) -> float:
    lo, hi = math.radians(lo_deg), math.radians(hi_deg)
    if lo <= hi:
        mask = (robot_angles >= lo) & (robot_angles <= hi)
    else:
        mask = (robot_angles >= lo) | (robot_angles <= hi)
    mask &= np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    return float(np.min(ranges[mask])) if np.any(mask) else float('inf')


def to_plot_xy(ranges: np.ndarray, robot_angles: np.ndarray):
    """Frente arriba, izquierda a la izquierda de la pantalla."""
    x = ranges * np.sin(-robot_angles)
    y = ranges * np.cos(robot_angles)
    return x, y


def main():
    rclpy.init()
    node = LidarVizNode()

    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7))

    print('Ctrl+C en esta terminal para cerrar.')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            scan = node.last_scan
            if scan is not None:
                ranges = np.asarray(scan.ranges, dtype=float)
                robot_angles = robot_frame_angles(scan, node.front_offset_rad, node.sign)
                x, y = to_plot_xy(ranges, robot_angles)

                ax.clear()
                ax.set_title('LiDAR MS200 - marco del robot (frente = arriba)')
                ax.scatter(x, y, s=4, c='black')
                ax.plot(0, 0, marker='s', markersize=12, color='dimgray')  # robot
                ax.annotate('FRENTE', (0, node.max_range * 0.95), ha='center')

                for nombre, lo_deg, hi_deg, color in SECTORES:
                    d = zone_min_distance(ranges, robot_angles, scan.range_min,
                                           min(scan.range_max, node.max_range), lo_deg, hi_deg)
                    for ang_deg in (lo_deg, hi_deg):
                        ang = math.radians(ang_deg)
                        bx = node.max_range * math.sin(-ang)
                        by = node.max_range * math.cos(ang)
                        ax.plot([0, bx], [0, by], color=color, linewidth=1, alpha=0.6)
                    mid = math.radians((lo_deg + hi_deg) / 2.0 if lo_deg <= hi_deg
                                        else (lo_deg + hi_deg + 360.0) / 2.0)
                    tx = node.max_range * 0.55 * math.sin(-mid)
                    ty = node.max_range * 0.55 * math.cos(mid)
                    etiqueta = f'{nombre}\n{d:.2f} m' if math.isfinite(d) else f'{nombre}\n---'
                    ax.text(tx, ty, etiqueta, color=color, ha='center', fontsize=9, weight='bold')

                ax.set_xlim(-node.max_range, node.max_range)
                ax.set_ylim(-node.max_range, node.max_range)
                ax.set_aspect('equal')
                ax.grid(True, linestyle=':', alpha=0.4)

            plt.pause(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
