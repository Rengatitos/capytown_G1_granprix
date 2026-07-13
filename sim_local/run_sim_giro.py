#!/usr/bin/env python3
"""Simulador local (sin ROS2) de un CRUCE con giro a la derecha: pared
derecha que dobla la esquina, replicando ``modo_simplificado`` de
``state_machine_node.py`` (AVANZAR_PARALELO -> DECIDIR -> GIRAR ->
AVANZAR_PARALELO), para ver el giro completo antes de probarlo en el
robot real.

Uso:
    python run_sim_giro.py
    python run_sim_giro.py --umbral-lado-libre 0.30
    python run_sim_giro.py --margen-avance -0.15   # avanza mas alla de la esquina antes de decidir

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

from environment import (
    escanear,
    pasillo_esquina_giro_derecha,
    pasillo_esquina_concava_derecha,
    pasillo_frente_bloqueado_gira_izquierda,
)
from robot_model import Pose, integrar
from wall_follow_control import ParametrosControl, ajustar_linea_pared, calcular_comando
from turn_control import ParametrosGiro, calcular_comando_giro, calcular_objetivo_giro

DT = 0.05
NUM_PUNTOS_SCAN = 452
RANGE_MAX = 4.0
RANGE_MIN = 0.03

VENT_LINEA = (-110.0, -70.0)
VENT_FRONT = (-15.0, 15.0)
VENT_LEFT = (70.0, 110.0)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--celda', type=float, default=0.60)
    p.add_argument('--ancho', type=float, default=0.60)
    p.add_argument('--distancia-objetivo', type=float, default=0.12)
    p.add_argument('--ganancia-angulo', type=float, default=2.0)
    p.add_argument('--ganancia-distancia', type=float, default=2.0)
    p.add_argument('--ganancia-heading', type=float, default=2.0)
    p.add_argument('--angular-max', type=float, default=0.6)
    p.add_argument('--velocidad', type=float, default=0.15)
    p.add_argument('--v-giro-lineal', type=float, default=0.08)
    p.add_argument('--v-giro-angular', type=float, default=0.5)
    p.add_argument('--tolerancia-giro-deg', type=float, default=4.0)
    p.add_argument('--umbral-frente-pared', type=float, default=0.25)
    p.add_argument('--umbral-frente-libre', type=float, default=0.35)
    p.add_argument('--umbral-lado-libre', type=float, default=0.40)
    p.add_argument('--margen-avance', type=float, default=0.15,
                    help='cuanto antes de la celda completa se detiene a decidir (m)')
    p.add_argument('--ventana-decision', type=float, nargs=2, default=[-100.0, -80.0],
                    help='ventana angular (grados) para el chequeo de "derecha libre" en el cruce')
    p.add_argument('--largo-robot', type=float, default=0.24)
    p.add_argument('--ancho-robot', type=float, default=0.16)
    p.add_argument('--tipo-esquina', choices=['convexa', 'concava', 'frente_izquierda'],
                    default='convexa',
                    help='convexa: la pared se proyecta hacia el robot al girar (exterior). '
                         'concava: el espacio se abre al girar (interior). '
                         'frente_izquierda: fondo ciego (como C4->D4 real), gira a la IZQUIERDA.')
    p.add_argument('--retranqueo', type=float, default=None,
                    help='solo para --tipo-esquina concava: cuanto se retranquea la pared del '
                         'segundo tramo (m, por defecto medio ancho de pasillo)')
    p.add_argument('--largo-aproximacion', type=float, default=None,
                    help='solo para --tipo-esquina frente_izquierda: largo del tramo recto antes '
                         'del fondo ciego (m, por defecto = --celda, o sea una sola celda)')
    return p.parse_args()


def zona_min(angulos, rangos, ventana_deg, range_min=RANGE_MIN, range_max=RANGE_MAX):
    lo, hi = math.radians(ventana_deg[0]), math.radians(ventana_deg[1])
    if lo <= hi:
        mask = (angulos >= lo) & (angulos <= hi)
    else:
        mask = (angulos >= lo) | (angulos <= hi)
    mask &= np.isfinite(rangos) & (rangos >= range_min) & (rangos <= range_max)
    if not np.any(mask):
        return float('inf'), False
    return float(np.min(rangos[mask])), True


def main():
    args = parse_args()

    params_wf = ParametrosControl(
        distancia_objetivo_m=args.distancia_objetivo,
        velocidad_lineal_mps=args.velocidad,
        ganancia_angulo=args.ganancia_angulo,
        ganancia_distancia=args.ganancia_distancia,
        ganancia_heading=args.ganancia_heading,
        angular_max_radps=args.angular_max,
    )
    params_giro = ParametrosGiro(
        velocidad_lineal_mps=args.v_giro_lineal,
        velocidad_angular_radps=args.v_giro_angular,
        tolerancia_giro_deg=args.tolerancia_giro_deg,
    )

    if args.tipo_esquina == 'concava':
        pasillo = pasillo_esquina_concava_derecha(
            celda_m=args.celda, ancho_m=args.ancho, retranqueo_m=args.retranqueo
        )
    elif args.tipo_esquina == 'frente_izquierda':
        pasillo = pasillo_frente_bloqueado_gira_izquierda(
            celda_m=args.celda, ancho_m=args.ancho, largo_aproximacion_m=args.largo_aproximacion
        )
    else:
        pasillo = pasillo_esquina_giro_derecha(celda_m=args.celda, ancho_m=args.ancho)

    pose = Pose(x=0.15, y=0.0, theta=0.0)
    heading_objetivo = None
    ultima_distancia_valida = None
    cell_start = (pose.x, pose.y)
    estado = 'AVANZAR_PARALELO'
    decision_actual = None
    giro_objetivo = None
    ultima_decision_info = ''

    trayectoria_x, trayectoria_y = [pose.x], [pose.y]
    rng = np.random.default_rng(0)

    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7))

    print('Cierra la ventana o Ctrl+C para detener.')
    try:
        paso = 0
        while paso < 3000:
            angulos, rangos = escanear(
                pose.como_tupla(), pasillo,
                angle_min=-math.pi, angle_max=math.pi, num_puntos=NUM_PUNTOS_SCAN,
                range_max=RANGE_MAX, range_min=RANGE_MIN, ruido_std=0.0, rng=rng,
            )

            if estado == 'AVANZAR_PARALELO':
                ajuste = ajustar_linea_pared(angulos, rangos, *VENT_LINEA,
                                              range_min=RANGE_MIN, range_max=RANGE_MAX, min_puntos=6)
                v, w, heading_objetivo, ultima_distancia_valida = calcular_comando(
                    ajuste, pose.theta, heading_objetivo, ultima_distancia_valida, params_wf
                )
                pose = integrar(pose, v, w, DT)

                avance = math.hypot(pose.x - cell_start[0], pose.y - cell_start[1])
                front_d, front_v = zona_min(angulos, rangos, VENT_FRONT)
                frente_cerca = front_v and front_d < args.umbral_frente_pared

                if avance >= (args.celda - args.margen_avance) or frente_cerca:
                    right_d, right_v = zona_min(angulos, rangos, tuple(args.ventana_decision))
                    left_d, left_v = zona_min(angulos, rangos, VENT_LEFT)
                    derecha_libre = right_v and right_d > args.umbral_lado_libre
                    frente_libre = front_v and front_d > args.umbral_frente_libre
                    izquierda_libre = left_v and left_d > args.umbral_lado_libre

                    if derecha_libre:
                        decision_actual = 'DERECHA'
                    elif frente_libre:
                        decision_actual = 'NINGUNO'
                    elif izquierda_libre:
                        decision_actual = 'IZQUIERDA'
                    else:
                        decision_actual = 'ATRAS'

                    ultima_decision_info = (
                        f'DECIDIR: der={derecha_libre}({right_d*100:.0f}cm) '
                        f'frente={frente_libre}({front_d*100:.0f}cm) '
                        f'izq={izquierda_libre}({left_d*100:.0f}cm) -> {decision_actual}'
                    )
                    print(f'[paso {paso}] {ultima_decision_info}')

                    if decision_actual == 'NINGUNO':
                        cell_start = (pose.x, pose.y)
                    else:
                        giro_objetivo = calcular_objetivo_giro(pose.theta, decision_actual)
                        estado = 'GIRAR'

            elif estado == 'GIRAR':
                v, w, terminado = calcular_comando_giro(pose.theta, giro_objetivo, params_giro)
                pose = integrar(pose, v, w, DT)
                ajuste = None
                if terminado:
                    print(f'[paso {paso}] GIRO TERMINADO theta={math.degrees(pose.theta):+.1f} deg')
                    cell_start = (pose.x, pose.y)
                    heading_objetivo = None
                    ultima_distancia_valida = None
                    estado = 'AVANZAR_PARALELO'

            trayectoria_x.append(pose.x)
            trayectoria_y.append(pose.y)
            paso += 1

            if paso % 2 == 0:
                _dibujar(ax, pose, pasillo, angulos, rangos, ajuste, estado,
                         ultima_decision_info, trayectoria_x, trayectoria_y, args)
                plt.pause(0.001)
    except KeyboardInterrupt:
        pass

    print(f'Fin: paso={paso} estado={estado} x={pose.x:.2f} y={pose.y:.2f} theta={math.degrees(pose.theta):+.1f}')
    plt.ioff()
    plt.show()


def _dibujar_robot(ax, pose, largo, ancho):
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


def _dibujar(ax, pose, pasillo, angulos, rangos, ajuste, estado, decision_info, tx, ty, args):
    ax.clear()

    for seg in pasillo.segmentos:
        ax.plot([seg.a[0], seg.b[0]], [seg.a[1], seg.b[1]], color='dimgray', linewidth=3)

    ax.plot(tx, ty, color='tab:blue', linewidth=1, alpha=0.6)
    _dibujar_robot(ax, pose, args.largo_robot, args.ancho_robot)

    lo, hi = math.radians(VENT_LINEA[0]), math.radians(VENT_LINEA[1])
    en_ventana = (angulos >= lo) & (angulos <= hi) & np.isfinite(rangos)
    if np.any(en_ventana):
        a = angulos[en_ventana]
        r = rangos[en_ventana]
        px = pose.x + r * np.cos(pose.theta + a)
        py = pose.y + r * np.sin(pose.theta + a)
        ax.scatter(px, py, s=8, color='tab:red', zorder=4)

    color_estado = {'AVANZAR_PARALELO': 'black', 'GIRAR': 'purple'}.get(estado, 'black')
    info = f'estado={estado}\n{decision_info}'
    ax.set_title(info, fontsize=9, color=color_estado)

    if args.tipo_esquina == 'frente_izquierda':
        largo = args.largo_aproximacion if args.largo_aproximacion is not None else args.celda
        ax.set_xlim(-0.3, largo + 0.6)
        ax.set_ylim(-0.6, 1.4)
    else:
        ax.set_xlim(-0.6, 1.4)
        ax.set_ylim(-1.4, 0.6)
    ax.set_aspect('equal')
    ax.grid(True, linestyle=':', alpha=0.4)


if __name__ == '__main__':
    main()
