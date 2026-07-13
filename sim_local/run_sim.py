#!/usr/bin/env python3
"""Simulador local (sin ROS2) para probar el seguimiento de pared
derecha por REGRESION DE LINEA + Kp antes de tocar el robot real.

Uso:
    python run_sim.py
    python run_sim.py --ganancia-angulo 3 --ganancia-distancia 3
    python run_sim.py --gap            # prueba el fallback sin pared derecha
    python run_sim.py --ruido 0.01     # agrega ruido al LiDAR simulado

Controles: cierra la ventana o Ctrl+C en la terminal para detener.
"""

import argparse
import math

import matplotlib

try:
    matplotlib.use('TkAgg')
except Exception:  # noqa: BLE001 - si TkAgg no esta disponible, usar el backend por defecto
    pass

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

from environment import escanear, pasillo_recto_con_quiebre
from robot_model import Pose, integrar
from wall_follow_control import ParametrosControl, ajustar_linea_pared, calcular_comando

VENTANA_DERECHA_DEG = (-110.0, -70.0)
DT = 0.05  # s (20 Hz, igual que control_rate_hz del robot real)
NUM_PUNTOS_SCAN = 452  # ~0.8 grados de resolucion, como el MS200
RANGE_MAX = 4.0
RANGE_MIN = 0.03


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ganancia-angulo', type=float, default=20.0)
    p.add_argument('--ganancia-distancia', type=float, default=2.0)
    p.add_argument('--ganancia-heading', type=float, default=2.0)
    p.add_argument('--velocidad', type=float, default=0.15)
    p.add_argument('--distancia-objetivo', type=float, default=0.07)
    p.add_argument('--angular-max', type=float, default=0.6)
    p.add_argument('--gap', action='store_true', help='agrega un hueco en la pared derecha')
    p.add_argument('--ruido', type=float, default=0.0, help='desviacion estandar de ruido del LiDAR (m)')
    p.add_argument('--largo', type=float, default=6.0)
    p.add_argument('--ancho', type=float, default=0.60)
    p.add_argument('--largo-robot', type=float, default=0.24,
                    help='largo real del robot en metros (PROPIEDADES_ROBOT.md)')
    p.add_argument('--ancho-robot', type=float, default=0.16,
                    help='ancho real del robot en metros (PROPIEDADES_ROBOT.md)')
    return p.parse_args()


def main():
    args = parse_args()

    params = ParametrosControl(
        distancia_objetivo_m=args.distancia_objetivo,
        velocidad_lineal_mps=args.velocidad,
        ganancia_angulo=args.ganancia_angulo,
        ganancia_distancia=args.ganancia_distancia,
        ganancia_heading=args.ganancia_heading,
        angular_max_radps=args.angular_max,
    )

    pasillo = pasillo_recto_con_quiebre(
        largo_m=args.largo,
        ancho_m=args.ancho,
        quiebre_x=args.largo * 0.5,
        quiebre_delta_y=-0.04,
        gap_x=(args.largo * 0.4) if args.gap else None,
        gap_ancho=0.15,
    )

    pose = Pose(x=0.15, y=0.0, theta=0.0)
    heading_objetivo = None
    ultima_distancia_valida = None
    rng = np.random.default_rng(0)

    trayectoria_x, trayectoria_y = [pose.x], [pose.y]

    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 5))

    print('Cierra la ventana o Ctrl+C para detener.')
    try:
        paso = 0
        while pose.x < args.largo - 0.3:
            angulos, rangos = escanear(
                pose.como_tupla(), pasillo,
                angle_min=-math.pi, angle_max=math.pi, num_puntos=NUM_PUNTOS_SCAN,
                range_max=RANGE_MAX, range_min=RANGE_MIN,
                ruido_std=args.ruido, rng=rng,
            )

            ajuste = ajustar_linea_pared(
                angulos, rangos, *VENTANA_DERECHA_DEG,
                range_min=RANGE_MIN, range_max=RANGE_MAX, min_puntos=6,
            )

            v, w, heading_objetivo, ultima_distancia_valida = calcular_comando(
                ajuste, pose.theta, heading_objetivo, ultima_distancia_valida, params
            )
            pose = integrar(pose, v, w, DT)
            trayectoria_x.append(pose.x)
            trayectoria_y.append(pose.y)
            paso += 1

            if paso % 2 == 0:
                _dibujar(ax, pose, pasillo, angulos, rangos, ajuste, v, w,
                         trayectoria_x, trayectoria_y, args)
                plt.pause(0.001)

            if ajuste is not None:
                margen_real_m = ajuste.distancia_m - args.ancho_robot / 2.0
                if margen_real_m < 0.0:
                    print(
                        f'AVISO paso={paso}: margen real negativo '
                        f'({margen_real_m * 100:.1f} cm) -- la carroceria tocaria la pared.'
                    )

            if paso > 4000:
                print('Limite de pasos alcanzado, deteniendo.')
                break
    except KeyboardInterrupt:
        pass

    print(f'Fin: x={pose.x:.2f} m, y={pose.y:.2f} m, pasos={paso}')
    plt.ioff()
    plt.show()


def _dibujar_robot(ax, pose, largo, ancho):
    """Dibuja el robot a escala real, centrado en ``pose`` (se asume que
    el LiDAR esta en el centro lateral del robot -- mismo supuesto que
    usa el calculo de margen real)."""
    hl, hw = largo / 2.0, ancho / 2.0
    esquinas_local = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])
    c, s = math.cos(pose.theta), math.sin(pose.theta)
    rot = np.array([[c, -s], [s, c]])
    esquinas = esquinas_local @ rot.T + np.array([pose.x, pose.y])
    ax.add_patch(Polygon(esquinas, closed=True, facecolor='dimgray',
                          edgecolor='black', alpha=0.85, zorder=5))

    frente_local = np.array([hl, 0.0])
    frente = frente_local @ rot.T + np.array([pose.x, pose.y])
    ax.plot([pose.x, frente[0]], [pose.y, frente[1]], color='gold', linewidth=2, zorder=6)


def _dibujar(ax, pose, pasillo, angulos, rangos, ajuste, v, w, tx, ty, args):
    ax.clear()

    for seg in pasillo.segmentos:
        ax.plot([seg.a[0], seg.b[0]], [seg.a[1], seg.b[1]], color='dimgray', linewidth=2)

    ax.plot(tx, ty, color='tab:blue', linewidth=1, alpha=0.6)

    _dibujar_robot(ax, pose, args.largo_robot, args.ancho_robot)

    lo, hi = math.radians(VENTANA_DERECHA_DEG[0]), math.radians(VENTANA_DERECHA_DEG[1])
    en_ventana = (angulos >= lo) & (angulos <= hi) & np.isfinite(rangos)
    if np.any(en_ventana):
        a = angulos[en_ventana]
        r = rangos[en_ventana]
        px = pose.x + r * np.cos(pose.theta + a)
        py = pose.y + r * np.sin(pose.theta + a)
        ax.scatter(px, py, s=8, color='tab:red', label='LiDAR (ventana derecha)')

    if ajuste is not None:
        m = math.tan(ajuste.angulo_rad)
        b_local = -ajuste.distancia_m * math.sqrt(m * m + 1.0)
        x_local = np.array([-0.1, 0.6])
        y_local = m * x_local + b_local
        wx = pose.x + x_local * math.cos(pose.theta) - y_local * math.sin(pose.theta)
        wy = pose.y + x_local * math.sin(pose.theta) + y_local * math.cos(pose.theta)
        ax.plot(wx, wy, color='tab:green', linewidth=2, label='recta ajustada')

    info = (
        f'x={pose.x:.2f} m  v={v:.2f} m/s  w={w:+.2f} rad/s\n'
    )
    color_titulo = 'black'
    if ajuste is not None:
        margen_real_cm = (ajuste.distancia_m - args.ancho_robot / 2.0) * 100.0
        info += (
            f'angulo_pared={math.degrees(ajuste.angulo_rad):+.1f} deg  '
            f'dist_pared(sensor)={ajuste.distancia_m * 100:.1f} cm  n={ajuste.n_puntos}\n'
            f'margen real hasta carroceria={margen_real_cm:+.1f} cm'
        )
        if margen_real_cm < 0.0:
            info += '  *** CHOQUE ***'
            color_titulo = 'red'
    else:
        info += 'sin pared derecha valida (fallback heading)'

    ax.set_title(info, fontsize=10, color=color_titulo)
    ax.set_xlim(pose.x - 1.0, pose.x + 1.5)
    ax.set_ylim(-args.ancho, args.ancho)
    ax.set_aspect('equal')
    ax.grid(True, linestyle=':', alpha=0.4)
    ax.legend(loc='upper right', fontsize=8)


if __name__ == '__main__':
    main()
