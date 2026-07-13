"""Logica de giro en cruce (chasis Ackermann), replica exacta de
``_handle_girar`` / ``_compute_turn_target`` en
``state_machine_node.py``, para validar en el simulador local antes
de portar cualquier ajuste al nodo real.

Como el chasis Ackermann no puede rotar sobre su propio eje, el giro
se aproxima con un arco de avance lento (velocidad lineal baja +
angular maxima) que cierra el lazo contra el yaw real (odometria), no
contra un tiempo fijo.
"""

import math
from dataclasses import dataclass
from typing import Tuple


def normalizar_angulo(angulo: float) -> float:
    while angulo > math.pi:
        angulo -= 2.0 * math.pi
    while angulo <= -math.pi:
        angulo += 2.0 * math.pi
    return angulo


def diferencia_angular(objetivo: float, actual: float) -> float:
    return normalizar_angulo(objetivo - actual)


def calcular_objetivo_giro(yaw_actual: float, direccion: str) -> float:
    """direccion en {'DERECHA', 'IZQUIERDA', 'ATRAS'}."""
    if direccion == 'DERECHA':
        delta = -math.pi / 2.0
    elif direccion == 'IZQUIERDA':
        delta = math.pi / 2.0
    elif direccion == 'ATRAS':
        delta = math.pi
    else:
        delta = 0.0
    return normalizar_angulo(yaw_actual + delta)


@dataclass
class ParametrosGiro:
    velocidad_lineal_mps: float = 0.08
    velocidad_angular_radps: float = 0.5
    tolerancia_giro_deg: float = 4.0


def calcular_comando_giro(
    yaw_actual: float, yaw_objetivo: float, params: ParametrosGiro
) -> Tuple[float, float, bool]:
    """Retorna (linear_x, angular_z, terminado)."""
    error = diferencia_angular(yaw_objetivo, yaw_actual)
    tolerancia_rad = math.radians(params.tolerancia_giro_deg)

    if abs(error) <= tolerancia_rad:
        return 0.0, 0.0, True

    v = params.velocidad_lineal_mps
    w = params.velocidad_angular_radps if error > 0.0 else -params.velocidad_angular_radps
    return v, w, False
