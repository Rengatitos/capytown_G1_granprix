"""Logica de control de seguimiento de pared derecha por REGRESION DE
LINEA + Kp. Funciones puras (sin ROS2, sin estado de nodo) para poder
probarlas en el simulador local y despues portarlas tal cual a
``lidar_processor_node`` (el ajuste de linea) y ``wall_follower_node``
(el control Kp) del paquete real.
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class AjusteLinea:
    angulo_rad: float       # angulo de la pared respecto al frente del robot
    distancia_m: float      # distancia perpendicular del robot a la pared
    n_puntos: int


def ajustar_linea_pared(
    angulos_robot: np.ndarray,
    rangos: np.ndarray,
    ventana_lo_deg: float,
    ventana_hi_deg: float,
    range_min: float,
    range_max: float,
    min_puntos: int = 6,
    max_iter_outliers: int = 3,
    umbral_residuo_m: float = 0.03,
) -> Optional[AjusteLinea]:
    """Ajusta una recta y = m*x + b a los puntos del LiDAR dentro de la
    ventana angular dada (marco del robot: x=adelante, y=izquierda).

    Rechazo iterativo de outliers: si algunos puntos de la ventana
    pertenecen a OTRA superficie (ej. una pared perpendicular en una
    esquina, no la pared que se sigue), un solo ajuste de minimos
    cuadrados les da peso y sesga el angulo/distancia -- se probo con
    una esquina real en sim_local/ y da hasta 37 grados de error falso.
    Aqui se ajusta, se descartan los puntos con residuo grande (que
    probablemente no son la pared seguida) y se reajusta, unas pocas
    veces. La ventana angular por defecto (ver ``right_side_window_deg``
    en el YAML) tambien se angosto para el mismo problema: una ventana
    mas angosta alcanza menos hacia adelante/atras y por eso agarra la
    esquina mucho mas tarde (mas cerca de ella) que una ventana ancha.

    Retorna None si no hay suficientes puntos validos para un ajuste
    confiable (equivale a "sin pared derecha de referencia").
    """
    lo = math.radians(ventana_lo_deg)
    hi = math.radians(ventana_hi_deg)

    if lo <= hi:
        en_ventana = (angulos_robot >= lo) & (angulos_robot <= hi)
    else:
        en_ventana = (angulos_robot >= lo) | (angulos_robot <= hi)

    validos = en_ventana & np.isfinite(rangos) & (rangos >= range_min) & (rangos <= range_max)
    if int(np.sum(validos)) < min_puntos:
        return None

    a = angulos_robot[validos]
    r = rangos[validos]
    x = r * np.cos(a)
    y = r * np.sin(a)

    for _ in range(max_iter_outliers):
        m, b = np.polyfit(x, y, 1)
        residuos = np.abs(y - (m * x + b)) / math.sqrt(m * m + 1.0)
        inliers = residuos < umbral_residuo_m
        if bool(np.all(inliers)) or int(np.sum(inliers)) < min_puntos:
            break
        x, y = x[inliers], y[inliers]

    angulo = math.atan(m)
    distancia = abs(b) / math.sqrt(m * m + 1.0)

    return AjusteLinea(angulo_rad=angulo, distancia_m=distancia, n_puntos=len(x))


@dataclass
class ParametrosControl:
    distancia_objetivo_m: float = 0.07
    velocidad_lineal_mps: float = 0.15
    ganancia_angulo: float = 2.0       # Kp sobre radianes (escala distinta a la version de 2 puntos)
    ganancia_distancia: float = 2.0    # Kp sobre metros
    ganancia_heading: float = 2.0      # Kp de respaldo sin pared (radianes)
    angular_max_radps: float = 0.6
    umbral_muy_cerca_m: float = 0.06   # ver nota en calcular_comando
    lado: str = 'DERECHA'              # 'DERECHA' o 'IZQUIERDA' -- ver nota de signo en calcular_comando


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def calcular_comando(
    ajuste: Optional[AjusteLinea],
    yaw_actual: float,
    heading_objetivo: Optional[float],
    ultima_distancia_valida: Optional[float],
    params: ParametrosControl,
) -> Tuple[float, float, Optional[float], Optional[float]]:
    """Calcula (linear_x, angular_z, nuevo_heading_objetivo, nueva_ultima_distancia).

    Si hay pared, corrige angulo Y distancia SIMULTANEAMENTE (suma
    ponderada), no alternando entre uno u otro. Alternar (corregir solo
    angulo, luego solo distancia) crea un ciclo que no se amortigua:
    cada correccion de distancia induce un error de angulo (al girar
    cambia el heading), que dispara la correccion de angulo, que vuelve
    a inducir error de distancia -- oscilacion sostenida verificada en
    sim_local/ (±1.4 cm indefinidamente, nunca se asienta). La suma
    simultanea sí converge (std < 0.01 cm en la cola del recorrido).

    Si NO hay pared (``ajuste`` es None), el LiDAR no reporta puntos
    por debajo de su rango minimo (~3 cm en el MS200) -- un pasillo
    "genuinamente abierto" y una pared "demasiado cerca para medir" se
    ven IGUAL en el escaneo (sin puntos validos), asi que no se puede
    distinguir con la lectura actual sola. En cambio, se usa la
    ULTIMA distancia valida conocida como pista:

    - Si ``ultima_distancia_valida`` existe y es menor a
      ``umbral_muy_cerca_m``: lo mas probable es que el robot se
      acerco demasiado y perdio la pared por estar bajo el rango
      minimo. Girar activamente lejos de la pared derecha (angular
      maximo) hasta recuperar una lectura valida -- si en vez de esto
      se mantiene el rumbo (como en el caso "pasillo abierto"), el
      robot no corrige nada y, si el rumbo apunta un poco hacia
      afuera, se aleja sin control y nunca vuelve (bug real
      encontrado probando en el robot).
    - Si no, asumir pasillo genuinamente abierto: mantener rumbo con
      Kp de heading sobre el yaw, como antes.
    """
    signo_lado = 1.0 if params.lado == 'DERECHA' else -1.0

    if ajuste is None:
        muy_cerca = (
            ultima_distancia_valida is not None
            and ultima_distancia_valida < params.umbral_muy_cerca_m
        )
        if muy_cerca:
            return (
                params.velocidad_lineal_mps, signo_lado * params.angular_max_radps,
                None, ultima_distancia_valida,
            )

        if heading_objetivo is None:
            heading_objetivo = yaw_actual
        error_heading = _angle_diff(heading_objetivo, yaw_actual)
        correccion = params.ganancia_heading * error_heading
        angular = _clamp(correccion, -params.angular_max_radps, params.angular_max_radps)
        return params.velocidad_lineal_mps, angular, heading_objetivo, ultima_distancia_valida

    # Hay pared: se olvida el heading objetivo (se recaptura fresco la
    # proxima vez que se pierda la pared) y se actualiza la ultima
    # distancia valida conocida.
    heading_objetivo = None
    ultima_distancia_valida = ajuste.distancia_m

    # Geometria: si la pared (horizontal en el mundo) se ve en el marco
    # del robot con pendiente m, entonces angulo_rad = atan(m) =
    # -theta_mundo (para un robot casi paralelo). Para corregir
    # theta_mundo -> 0 se necesita w = -k*theta_mundo = +k*angulo_rad
    # (sin signo negativo). Con el signo cambiado el lazo es de
    # realimentacion POSITIVA y el robot diverge en menos de 1 s --
    # verificado en sim_local/ antes de portar esto al robot real. Este
    # termino NO cambia de signo entre lados (right/left son paralelas
    # entre si, rotan igual con el yaw del robot).
    #
    # El termino de DISTANCIA SI cambia de signo entre lados: "muy cerca
    # de la pared derecha" corrige alejandose (girar izquierda), "muy
    # cerca de la pared izquierda" corrige alejandose girando a la
    # DERECHA -- es un espejo, multiplicado por signo_lado.
    error_distancia = params.distancia_objetivo_m - ajuste.distancia_m
    correccion = (
        params.ganancia_angulo * ajuste.angulo_rad
        + signo_lado * params.ganancia_distancia * error_distancia
    )

    angular = _clamp(correccion, -params.angular_max_radps, params.angular_max_radps)
    return params.velocidad_lineal_mps, angular, heading_objetivo, ultima_distancia_valida


def _angle_diff(target: float, current: float) -> float:
    d = target - current
    while d > math.pi:
        d -= 2.0 * math.pi
    while d <= -math.pi:
        d += 2.0 * math.pi
    return d
