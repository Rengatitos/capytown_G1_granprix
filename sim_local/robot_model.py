"""Modelo cinematico simple (uniciclo) para el robot simulado.

No modela la restriccion Ackermann (radio de giro minimo); es
suficiente para validar el algoritmo de seguimiento de pared en
tramos rectos, que es el foco de este simulador.
"""

import math
from dataclasses import dataclass


@dataclass
class Pose:
    x: float
    y: float
    theta: float

    def como_tupla(self):
        return (self.x, self.y, self.theta)


def integrar(pose: Pose, v: float, w: float, dt: float) -> Pose:
    x = pose.x + v * math.cos(pose.theta) * dt
    y = pose.y + v * math.sin(pose.theta) * dt
    theta = pose.theta + w * dt
    while theta > math.pi:
        theta -= 2.0 * math.pi
    while theta <= -math.pi:
        theta += 2.0 * math.pi
    return Pose(x, y, theta)
