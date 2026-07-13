"""Entorno 2D simple (pasillo con paredes como segmentos) y un LiDAR
simulado por ray casting. Sin dependencia de ROS2 -- solo numpy.

Convencion: marco del ROBOT ya "calibrado" (x=adelante, y=izquierda,
angulo 0=frente, +90=izquierda), igual que el LiDAR real despues de
aplicar front_offset_deg/invert_left_right. El simulador no reproduce
el problema de calibracion del LiDAR (eso ya se resolvio en el robot
real); aqui el foco es validar el algoritmo de control.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class Segmento:
    a: Tuple[float, float]
    b: Tuple[float, float]


@dataclass
class Pasillo:
    """Coleccion de segmentos de pared (derecha, izquierda, lo que sea)."""

    segmentos: List[Segmento] = field(default_factory=list)

    def agregar(self, ax, ay, bx, by):
        self.segmentos.append(Segmento((ax, ay), (bx, by)))


def _interseccion_rayo_segmento(
    origen: np.ndarray, direccion: np.ndarray, seg: Segmento
) -> Optional[float]:
    """Distancia a lo largo del rayo hasta el segmento, o None si no cruza."""
    a = np.array(seg.a, dtype=float)
    b = np.array(seg.b, dtype=float)
    v1 = origen - a
    v2 = b - a
    v3 = np.array([-direccion[1], direccion[0]])

    denom = np.dot(v2, v3)
    if abs(denom) < 1e-9:
        return None

    t1 = (v2[0] * v1[1] - v2[1] * v1[0]) / denom  # producto cruz 2D / denom
    t2 = np.dot(v1, v3) / denom

    if t1 >= 0.0 and 0.0 <= t2 <= 1.0:
        return float(t1)
    return None


def escanear(
    pose: Tuple[float, float, float],
    pasillo: Pasillo,
    angle_min: float,
    angle_max: float,
    num_puntos: int,
    range_max: float,
    range_min: float = 0.03,
    ruido_std: float = 0.0,
    rng: Optional[np.random.Generator] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Simula un LiDAR 2D. Retorna (angulos_robot, rangos).

    ``pose`` = (x, y, theta) del robot en el marco MUNDO. Los angulos
    retornados ya estan en el marco del ROBOT (no del mundo).
    """
    x, y, theta = pose
    origen = np.array([x, y], dtype=float)
    angulos = np.linspace(angle_min, angle_max, num_puntos, endpoint=False)
    rangos = np.full(num_puntos, np.inf, dtype=float)

    if rng is None:
        rng = np.random.default_rng()

    for i, ang_robot in enumerate(angulos):
        ang_mundo = theta + ang_robot
        direccion = np.array([math.cos(ang_mundo), math.sin(ang_mundo)])

        mejor = range_max
        for seg in pasillo.segmentos:
            t = _interseccion_rayo_segmento(origen, direccion, seg)
            if t is not None and range_min <= t < mejor:
                mejor = t

        if mejor < range_max:
            if ruido_std > 0.0:
                mejor = max(range_min, mejor + rng.normal(0.0, ruido_std))
            rangos[i] = mejor
        else:
            rangos[i] = float('inf')

    return angulos, rangos


def pasillo_recto_con_quiebre(
    largo_m: float = 6.0,
    ancho_m: float = 0.60,
    offset_y0: float = 0.0,
    quiebre_x: float = 3.0,
    quiebre_delta_y: float = -0.04,
    gap_x: Optional[float] = None,
    gap_ancho: float = 0.15,
) -> Pasillo:
    """Pasillo de prueba: pared derecha con un leve quiebre de angulo a
    mitad de camino (para probar que el ajuste de linea detecta un
    angulo distinto de cero), pared izquierda recta, y opcionalmente
    un hueco en la pared derecha (para probar el fallback sin pared).
    """
    p = Pasillo()

    y_der_0 = offset_y0 - ancho_m / 2.0
    y_der_1 = y_der_0 + quiebre_delta_y

    if gap_x is None:
        p.agregar(0.0, y_der_0, quiebre_x, y_der_0)
        p.agregar(quiebre_x, y_der_0, largo_m, y_der_1)
    else:
        gap_fin = gap_x + gap_ancho
        p.agregar(0.0, y_der_0, gap_x, y_der_0)
        p.agregar(gap_fin, y_der_0, quiebre_x, y_der_0)
        p.agregar(quiebre_x, y_der_0, largo_m, y_der_1)

    y_izq = offset_y0 + ancho_m / 2.0
    p.agregar(0.0, y_izq, largo_m, y_izq)

    return p


def pasillo_esquina_giro_derecha(
    celda_m: float = 0.60,
    ancho_m: float = 0.60,
    x_inicio: float = 0.0,
    largo_pared_izq: Optional[float] = None,
    celdas_cierre: float = 2.0,
) -> Pasillo:
    """Corredor recto de una celda (robot avanza en +x) cuya pared
    derecha "dobla la esquina" y continua como pared derecha de un
    SEGUNDO corredor perpendicular hacia -y (equivalente a un giro a
    la derecha real en el laberinto). Pared izquierda solo en el
    primer tramo (para no interferir con la deteccion de "izquierda
    libre" en el cruce).

    Incluye un cierre lejano (pared perpendicular a ``celdas_cierre``
    celdas del cruce) para que "frente libre"/"izquierda libre" den
    lecturas validas y finitas en vez de infinito -- en el laberinto
    real siempre hay una pared dentro del alcance del LiDAR (maze
    acotado); un pasillo de prueba totalmente abierto no es realista.
    """
    p = Pasillo()

    y_der_0 = -ancho_m / 2.0
    x_esquina = x_inicio + celda_m

    # Pared derecha del primer corredor + su continuacion doblando la
    # esquina (una sola pared en L, como en el laberinto real).
    p.agregar(x_inicio, y_der_0, x_esquina, y_der_0)
    p.agregar(x_esquina, y_der_0, x_esquina, y_der_0 - celda_m)

    # Pared izquierda del primer corredor (para que "izquierda" no se
    # vea libre por error mientras aun esta en el tramo recto).
    largo_izq = largo_pared_izq if largo_pared_izq is not None else celda_m
    y_izq = ancho_m / 2.0
    p.agregar(x_inicio, y_izq, x_inicio + largo_izq, y_izq)

    # Cierre lejano: pared perpendicular delante del cruce, simulando
    # la siguiente pared del laberinto real (no vacio infinito).
    if celdas_cierre > 0:
        x_lejos = x_esquina + celdas_cierre * celda_m
        p.agregar(x_lejos, y_izq, x_lejos, y_der_0 - celda_m)

    return p


def pasillo_esquina_concava_derecha(
    celda_m: float = 0.60,
    ancho_m: float = 0.60,
    x_inicio: float = 0.0,
    retranqueo_m: Optional[float] = None,
    largo_pared_izq: Optional[float] = None,
    celdas_cierre: float = 2.0,
) -> Pasillo:
    """Corredor recto de una celda (robot avanza en +x) que gira a la
    derecha hacia una esquina CONCAVA/interior: la pared derecha del
    segundo corredor NO continua desde el mismo punto donde termina la
    primera (eso seria una esquina convexa/exterior, ver
    ``pasillo_esquina_giro_derecha``, donde la pared se "proyecta"
    hacia el robot). Aqui la pared del segundo corredor esta
    RETRANQUEADA hacia adentro: el espacio se ABRE al girar, como la
    esquina interior de una habitacion en L, dejando mas margen para
    el giro.

    Incluye el mismo cierre lejano que ``pasillo_esquina_giro_derecha``
    (ver docstring alli) para evitar lecturas invalidas por "infinito".
    """
    p = Pasillo()

    y_der_0 = -ancho_m / 2.0
    x_esquina = x_inicio + celda_m
    # Por defecto, el hueco mide una celda completa (60cm): asi coincide
    # con la resolucion real del laberinto en grilla, donde toda abertura
    # es de al menos una celda. Un hueco mas angosto que el paso entre
    # chequeos de DECIDIR (~celda-margen_avance) puede saltarse por
    # completo sin ser muestreado -- ver hallazgo de calibracion.
    retranqueo = retranqueo_m if retranqueo_m is not None else celda_m

    # Pared derecha del primer corredor (NO dobla la esquina, termina limpia).
    p.agregar(x_inicio, y_der_0, x_esquina, y_der_0)

    # Pared derecha del segundo corredor, retranqueada MAS ALLA de la
    # esquina (se aleja del robot en vez de proyectarse hacia el), asi
    # se abre un hueco en L justo donde el robot decide (x_esquina a
    # x_pared2) en vez de un hueco "detras" que el DECIDIR nunca ve.
    x_pared2 = x_esquina + retranqueo
    p.agregar(x_pared2, y_der_0, x_pared2, y_der_0 - celda_m)

    # Pared izquierda del primer corredor.
    largo_izq = largo_pared_izq if largo_pared_izq is not None else celda_m
    y_izq = ancho_m / 2.0
    p.agregar(x_inicio, y_izq, x_inicio + largo_izq, y_izq)

    # Cierre lejano (ver docstring de pasillo_esquina_giro_derecha).
    if celdas_cierre > 0:
        x_lejos = x_esquina + celdas_cierre * celda_m
        p.agregar(x_lejos, y_izq, x_lejos, y_der_0 - celda_m)

        # Cierre SUR del hueco concavo: sin esto, un rayo lateral hacia
        # la abertura no encuentra ninguna pared dentro del alcance del
        # LiDAR y la lectura vuelve "invalida" (igual que "sin pared" en
        # DECIDIR), lo cual NO representa el laberinto real (acotado a
        # 3.6x2.4m, siempre hay una pared dentro de rango). Se cierra a
        # la misma distancia que celdas_cierre, simulando la pared del
        # laberinto un par de celdas mas alla de la abertura.
        y_sur = y_der_0 - celdas_cierre * celda_m
        p.agregar(x_esquina, y_sur, x_lejos, y_sur)

    return p


def pasillo_frente_bloqueado_gira_izquierda(
    celda_m: float = 0.60,
    ancho_m: float = 0.60,
    x_inicio: float = 0.0,
    celdas_cierre: float = 2.0,
    largo_aproximacion_m: Optional[float] = None,
) -> Pasillo:
    """Corredor recto (robot avanza en +x, pared derecha en -y) que
    termina en un FONDO CIEGO: una pared frontal cierra todo el ancho
    de la celda (igual que V01 en el laberinto real, entre C4 y D4).
    Ni derecha (perimetral, sin salida) ni frente tienen paso -- solo
    la izquierda esta abierta (como C4 -> C3), asi que el robot debe
    frenar cerca de la pared frontal y girar 90 grados a la IZQUIERDA.

    Sin pared izquierda cerca de la esquina (la salida real esta
    abierta ahi); se agrega solo un cierre lejano hacia el norte para
    que "izquierda libre" de una lectura valida y finita en vez de
    infinito (mismo motivo que en ``pasillo_esquina_concava_derecha``).

    ``largo_aproximacion_m`` separa el largo del tramo recto (cuanto
    corre el robot antes de llegar al fondo) del ancho de la celda
    (``celda_m``, que sigue siendo 0.60 = una celda del laberinto real).
    Por defecto son iguales (una sola celda), pero un tramo mas largo
    (varias celdas) le da al control mas tiempo para converger a la
    distancia objetivo antes de la decision, mas realista que un solo
    tramo corto.
    """
    p = Pasillo()

    y_der_0 = -ancho_m / 2.0
    y_izq_0 = ancho_m / 2.0
    largo = largo_aproximacion_m if largo_aproximacion_m is not None else celda_m
    x_frente = x_inicio + largo

    # Pared derecha (la que el robot sigue).
    p.agregar(x_inicio, y_der_0, x_frente, y_der_0)

    # Pared frontal: cierra TODO el ancho de la celda (fondo ciego).
    p.agregar(x_frente, y_der_0, x_frente, y_izq_0)

    # Cierre lejano hacia la izquierda/norte (limite del laberinto
    # real un par de celdas mas alla de la salida abierta).
    if celdas_cierre > 0:
        y_lejos = y_izq_0 + celdas_cierre * celda_m
        p.agregar(x_inicio, y_lejos, x_frente, y_lejos)

    return p


def pasillo_laberinto_completo() -> Pasillo:
    """Laberinto completo Gran Prix CapyTown, coordenadas EXACTAS de
    DETALLE_PISTA.md (centimetros -> metros, /100). Origen (0,0) en la
    esquina superior izquierda, X crece a la derecha (0-360cm), Y
    crece hacia ABAJO (0-240cm) -- igual convencion que el plano
    oficial. Inicio en A4 (centro aprox. x=0.30 y=2.10), meta en F1.
    """
    segmentos_cm = [
        # Paredes perimetrales (P01-P18)
        (0, 0, 60, 0), (60, 0, 120, 0), (120, 0, 180, 0),
        (180, 0, 240, 0), (240, 0, 300, 0), (300, 0, 360, 0),
        (0, 240, 60, 240), (60, 240, 120, 240), (120, 240, 180, 240),
        (180, 240, 240, 240), (240, 240, 300, 240), (300, 240, 360, 240),
        (0, 0, 0, 60), (0, 60, 0, 120), (0, 120, 0, 180),
        (360, 60, 360, 120), (360, 120, 360, 180), (360, 180, 360, 240),
        # Paredes internas verticales (V01-V06)
        (180, 180, 180, 240), (60, 120, 60, 180), (240, 120, 240, 180),
        (180, 60, 180, 120), (300, 60, 300, 120), (120, 0, 120, 60),
        # Paredes internas horizontales (H01-H06)
        (60, 60, 120, 60), (120, 120, 180, 120), (180, 120, 240, 120),
        (240, 60, 300, 60), (240, 180, 300, 180), (300, 180, 360, 180),
        # Obstaculos parciales (S01-S02, 30 cm)
        (120, 180, 120, 210), (270, 150, 300, 150),
    ]
    p = Pasillo()
    for x1, y1, x2, y2 in segmentos_cm:
        p.agregar(x1 / 100.0, y1 / 100.0, x2 / 100.0, y2 / 100.0)
    return p
