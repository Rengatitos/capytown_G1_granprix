#!/usr/bin/env python3
"""Nodo de metricas: registra el desempeno de la corrida en CSV.

Escucha ``/robot_event`` (capytown_interfaces/RobotEvent) para contar
colisiones, senales PARE y callejones sin salida sin acoplarse a la
logica de navegacion, y ``/odom_raw`` para acumular la distancia
recorrida. Al recibir el evento META (o TIMEOUT, o al apagarse sin
haber llegado) escribe una fila en ``metricas_granprix.csv`` con el
formato de DETALLE RETO 3.md.

Nota sobre ``pare_falsos``: el robot no puede verificar por si mismo
si una deteccion fue realmente una senal PARE o un falso positivo
(eso requiere comparar contra la senal real de la pista). Este campo
queda en 0 por defecto y se documenta en el README para revision
manual comparando el video de la corrida contra los eventos
PARE_DETECTADO registrados.
"""

import csv
import math
import os

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from capytown_interfaces.msg import RobotEvent
from capytown_granprix import event_types as EV

_CSV_FIELDS = [
    'ronda', 'llego_meta', 'tiempo_s', 'long_ruta_cm', 'long_optima_cm',
    'eficiencia', 'colisiones', 'pare_reales', 'pare_detectados',
    'pare_respetados', 'pare_falsos', 'dead_ends_visitados',
]


class MetricsLoggerNode(Node):

    def __init__(self):
        super().__init__('metrics_logger')

        self.declare_parameter('event_topic', '/robot_event')
        self.declare_parameter('odom_topic', '/odom_raw')
        self.declare_parameter('csv_path', '~/capytown_resultados/metricas_granprix.csv')
        self.declare_parameter('ronda', 1)
        self.declare_parameter('pare_reales', 3)
        self.declare_parameter('long_optima_cm', 480.0)

        self._csv_path = os.path.expanduser(self.get_parameter('csv_path').value)
        self._ronda = int(self.get_parameter('ronda').value)
        self._pare_reales = int(self.get_parameter('pare_reales').value)
        self._long_optima_cm = float(self.get_parameter('long_optima_cm').value)

        self._start_time = self.get_clock().now()
        self._prev_xy = None
        self._long_ruta_cm = 0.0

        self._colisiones = 0
        self._pare_detectados = 0
        self._pare_respetados = 0
        self._pare_falsos = 0
        self._dead_ends = 0
        self._finalizado = False

        self.create_subscription(RobotEvent, self.get_parameter('event_topic').value, self._on_event, 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self._on_odom, 10)

        self.get_logger().info(
            f'metrics_logger listo: ronda={self._ronda}, csv={self._csv_path}'
        )

    def _on_odom(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._prev_xy is not None:
            dx = x - self._prev_xy[0]
            dy = y - self._prev_xy[1]
            self._long_ruta_cm += math.hypot(dx, dy) * 100.0
        self._prev_xy = (x, y)

    def _on_event(self, msg: RobotEvent) -> None:
        tipo = msg.tipo

        if tipo == EV.COLISION:
            self._colisiones += 1
        elif tipo == EV.PARE_DETECTADO:
            self._pare_detectados += 1
        elif tipo == EV.PARE_RESPETADO:
            self._pare_respetados += 1
        elif tipo == EV.PARE_FALSO:
            self._pare_falsos += 1
        elif tipo == EV.DEAD_END:
            self._dead_ends += 1
        elif tipo == EV.META:
            self._finalizar(llego_meta=True)
        elif tipo == EV.TIMEOUT:
            self._finalizar(llego_meta=False)

    def _finalizar(self, llego_meta: bool) -> None:
        if self._finalizado:
            return
        self._finalizado = True

        tiempo_s = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        eficiencia = (
            self._long_optima_cm / self._long_ruta_cm if self._long_ruta_cm > 0.0 else 0.0
        )

        fila = {
            'ronda': self._ronda,
            'llego_meta': 'Si' if llego_meta else 'No',
            'tiempo_s': round(tiempo_s, 1),
            'long_ruta_cm': round(self._long_ruta_cm, 1),
            'long_optima_cm': round(self._long_optima_cm, 1),
            'eficiencia': round(eficiencia, 3),
            'colisiones': self._colisiones,
            'pare_reales': self._pare_reales,
            'pare_detectados': self._pare_detectados,
            'pare_respetados': self._pare_respetados,
            'pare_falsos': self._pare_falsos,
            'dead_ends_visitados': self._dead_ends,
        }
        self._escribir_fila_csv(fila)
        self.get_logger().info(f'metricas registradas en {self._csv_path}: {fila}')

    def _escribir_fila_csv(self, fila: dict) -> None:
        os.makedirs(os.path.dirname(self._csv_path), exist_ok=True)
        existe = os.path.isfile(self._csv_path)
        with open(self._csv_path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=_CSV_FIELDS)
            if not existe:
                writer.writeheader()
            writer.writerow(fila)

    def finalizar_si_falta(self) -> None:
        """Se llama al apagar el nodo si nunca llego el evento META/TIMEOUT."""
        self._finalizar(llego_meta=False)


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalizar_si_falta()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
