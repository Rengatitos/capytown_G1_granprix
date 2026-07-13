#!/usr/bin/env python3
"""Simulador local (sin ROS2) del LABERINTO COMPLETO Gran Prix
CapyTown, con coordenadas exactas de DETALLE_PISTA.md. El robot
arranca en A4 (entrada) y navega de forma REACTIVA (misma logica que
``modo_simplificado`` de ``state_machine_node.py``: AVANZAR_PARALELO
-> DECIDIR -> GIRAR -> AVANZAR_PARALELO), sin conocer el mapa de
antemano -- por eso no necesariamente sigue la "ruta optima" del
plano, sino la que resulte de seguir la pared del lado elegido
(``--lado DERECHA|IZQUIERDA``, prioridad derecha->frente->izquierda->
atras o izquierda->frente->derecha->atras segun corresponda).

Uso:
    python run_sim_laberinto.py
    python run_sim_laberinto.py --lado IZQUIERDA
    python run_sim_laberinto.py --umbral-lado-libre 0.30
    python run_sim_laberinto.py --margen-avance -0.15
    python run_sim_laberinto.py --sin-graficos --lado IZQUIERDA   # validacion rapida sin ventana

Controles: cierra la ventana o Ctrl+C en la terminal para detener.
"""

import argparse
import math

import matplotlib

try:
    matplotlib.use('TkAgg')
except Exception:  # noqa: BLE001
    pass

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

from environment import escanear, pasillo_laberinto_completo
from robot_model import Pose, integrar
from wall_follow_control import ParametrosControl, ajustar_linea_pared, calcular_comando
from turn_control import ParametrosGiro, calcular_comando_giro, calcular_objetivo_giro

DT = 0.05
NUM_PUNTOS_SCAN = 452
RANGE_MAX = 4.0
RANGE_MIN = 0.03

VENT_LINEA_DER = (-110.0, -70.0)
VENT_LINEA_IZQ = (70.0, 110.0)
VENT_FRONT = (-15.0, 15.0)
VENT_RIGHT = (-110.0, -70.0)
VENT_LEFT = (70.0, 110.0)

# Centro de A4 (entrada) en cm -> m, mirando hacia el "norte" (Y
# decreciente, hacia A3/A2/A1) para iniciar el recorrido.
INICIO_X = 0.30
INICIO_Y = 2.10
INICIO_THETA = -math.pi / 2.0


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--lado', choices=['DERECHA', 'IZQUIERDA'], default='DERECHA')
    p.add_argument('--sin-graficos', action='store_true',
                    help='corre sin ventana grafica, solo imprime resultados (para validacion automatica)')
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
    p.add_argument('--celda', type=float, default=0.60)
    p.add_argument('--margen-avance', type=float, default=0.05)
    p.add_argument('--ventana-decision', type=float, nargs=2, default=[-100.0, -80.0])
    p.add_argument('--largo-robot', type=float, default=0.24)
    p.add_argument('--ancho-robot', type=float, default=0.16)
    p.add_argument('--max-pasos', type=int, default=20000)
    p.add_argument('--dibujar-cada', type=int, default=3)
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
        lado=args.lado,
    )
    params_giro = ParametrosGiro(
        velocidad_lineal_mps=args.v_giro_lineal,
        velocidad_angular_radps=args.v_giro_angular,
        tolerancia_giro_deg=args.tolerancia_giro_deg,
    )
    vent_linea = VENT_LINEA_DER if args.lado == 'DERECHA' else VENT_LINEA_IZQ

    pasillo = pasillo_laberinto_completo()

    pose = Pose(x=INICIO_X, y=INICIO_Y, theta=INICIO_THETA)
    heading_objetivo = None
    ultima_distancia_valida = None
    cell_start = (pose.x, pose.y)
    estado = 'AVANZAR_PARALELO'
    decision_actual = None
    giro_objetivo = None
    ultima_decision_info = ''
    num_celdas = 0
    num_giros = 0

    trayectoria_x, trayectoria_y = [pose.x], [pose.y]
    rng = np.random.default_rng(0)
    celdas_visitadas = set()

    fig = ax = None
    if not args.sin_graficos:
        plt.ion()
        fig, ax = plt.subplots(figsize=(9, 7))

    print(f'lado={args.lado}. Cierra la ventana o Ctrl+C para detener.')
    meta_alcanzada = False
    try:
        paso = 0
        while paso < args.max_pasos:
            angulos, rangos = escanear(
                pose.como_tupla(), pasillo,
                angle_min=-math.pi, angle_max=math.pi, num_puntos=NUM_PUNTOS_SCAN,
                range_max=RANGE_MAX, range_min=RANGE_MIN, ruido_std=0.0, rng=rng,
            )
            celdas_visitadas.add((int(pose.x // args.celda), int(pose.y // args.celda)))

            if estado == 'AVANZAR_PARALELO':
                ajuste = ajustar_linea_pared(angulos, rangos, *vent_linea,
                                              range_min=RANGE_MIN, range_max=RANGE_MAX, min_puntos=6)
                v, w, heading_objetivo, ultima_distancia_valida = calcular_comando(
                    ajuste, pose.theta, heading_objetivo, ultima_distancia_valida, params_wf
                )
                pose = integrar(pose, v, w, DT)

                avance = math.hypot(pose.x - cell_start[0], pose.y - cell_start[1])
                front_d, front_v = zona_min(angulos, rangos, VENT_FRONT)
                frente_cerca = front_v and front_d < args.umbral_frente_pared

                if avance >= (args.celda - args.margen_avance) or frente_cerca:
                    num_celdas += 1
                    right_d, right_v = zona_min(angulos, rangos, tuple(args.ventana_decision))
                    left_d, left_v = zona_min(angulos, rangos, VENT_LEFT)
                    derecha_libre = right_v and right_d > args.umbral_lado_libre
                    frente_libre = front_v and front_d > args.umbral_frente_libre
                    izquierda_libre = left_v and left_d > args.umbral_lado_libre

                    if args.lado == 'IZQUIERDA':
                        orden = (('IZQUIERDA', izquierda_libre), ('NINGUNO', frente_libre),
                                 ('DERECHA', derecha_libre))
                    else:
                        orden = (('DERECHA', derecha_libre), ('NINGUNO', frente_libre),
                                 ('IZQUIERDA', izquierda_libre))
                    decision_actual = next((d for d, libre in orden if libre), 'ATRAS')

                    ultima_decision_info = (
                        f'celda #{num_celdas}  der={derecha_libre}({right_d*100:.0f}) '
                        f'frente={frente_libre}({front_d*100:.0f}) izq={izquierda_libre}({left_d*100:.0f}) '
                        f'-> {decision_actual}'
                    )
                    print(f'[paso {paso}] x={pose.x*100:.0f}cm y={pose.y*100:.0f}cm '
                          f'theta={math.degrees(pose.theta):+.0f} | {ultima_decision_info}')

                    if decision_actual == 'NINGUNO':
                        cell_start = (pose.x, pose.y)
                    else:
                        num_giros += 1
                        giro_objetivo = calcular_objetivo_giro(pose.theta, decision_actual)
                        estado = 'GIRAR'

            elif estado == 'GIRAR':
                v, w, terminado = calcular_comando_giro(pose.theta, giro_objetivo, params_giro)
                pose = integrar(pose, v, w, DT)
                ajuste = None
                if terminado:
                    cell_start = (pose.x, pose.y)
                    heading_objetivo = None
                    ultima_distancia_valida = None
                    estado = 'AVANZAR_PARALELO'

            trayectoria_x.append(pose.x)
            trayectoria_y.append(pose.y)
            paso += 1

            # Meta aproximada: centro de F1 (330, 30 cm).
            dist_meta = math.hypot(pose.x - 3.30, pose.y - 0.30)
            if dist_meta < 0.15:
                print(f'\n*** META ALCANZADA en paso {paso} ***')
                meta_alcanzada = True
                break

            if not args.sin_graficos and paso % args.dibujar_cada == 0:
                _dibujar(ax, pose, pasillo, angulos, rangos, ajuste, estado,
                         ultima_decision_info, num_celdas, num_giros,
                         trayectoria_x, trayectoria_y, args)
                plt.pause(0.001)
    except KeyboardInterrupt:
        pass

    print(f'\nFin: paso={paso} estado={estado} celdas_avanzadas={num_celdas} giros={num_giros} '
          f'x={pose.x*100:.0f}cm y={pose.y*100:.0f}cm theta={math.degrees(pose.theta):+.0f} '
          f'meta={meta_alcanzada} celdas_unicas_visitadas={len(celdas_visitadas)}/24')

    if not args.sin_graficos:
        plt.ioff()
        plt.show()


def _dibujar_robot(ax, pose, largo, ancho):
    hl, hw = largo / 2.0, ancho / 2.0
    esquinas_local = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])
    c, s = math.cos(pose.theta), math.sin(pose.theta)
    rot = np.array([[c, -s], [s, c]])
    esquinas = esquinas_local @ rot.T + np.array([pose.x, pose.y])
    ax.add_patch(Polygon(esquinas, closed=True, facecolor='dimgray',
                          edgecolor='black', alpha=0.9, zorder=5))
    frente_local = np.array([hl, 0.0])
    frente = frente_local @ rot.T + np.array([pose.x, pose.y])
    ax.plot([pose.x, frente[0]], [pose.y, frente[1]], color='gold', linewidth=2, zorder=6)


def _dibujar_grid(ax):
    for col in range(7):
        ax.axvline(col * 0.6, color='lightgray', linewidth=0.5, zorder=0)
    for row in range(5):
        ax.axhline(row * 0.6, color='lightgray', linewidth=0.5, zorder=0)
    letras = 'ABCDEF'
    for c in range(6):
        for r in range(4):
            ax.text((c + 0.5) * 0.6, (r + 0.5) * 0.6, f'{letras[c]}{r+1}',
                     ha='center', va='center', fontsize=8, color='lightgray', zorder=0)


def _dibujar(ax, pose, pasillo, angulos, rangos, ajuste, estado, decision_info,
             num_celdas, num_giros, tx, ty, args):
    ax.clear()

    _dibujar_grid(ax)

    for seg in pasillo.segmentos:
        ax.plot([seg.a[0], seg.b[0]], [seg.a[1], seg.b[1]], color='saddlebrown', linewidth=3)

    ax.plot(tx, ty, color='tab:blue', linewidth=1, alpha=0.6, zorder=2)
    _dibujar_robot(ax, pose, args.largo_robot, args.ancho_robot)

    vent_linea = VENT_LINEA_DER if args.lado == 'DERECHA' else VENT_LINEA_IZQ
    lo, hi = math.radians(vent_linea[0]), math.radians(vent_linea[1])
    en_ventana = (angulos >= lo) & (angulos <= hi) & np.isfinite(rangos)
    if np.any(en_ventana):
        a = angulos[en_ventana]
        r = rangos[en_ventana]
        px = pose.x + r * np.cos(pose.theta + a)
        py = pose.y + r * np.sin(pose.theta + a)
        ax.scatter(px, py, s=6, color='tab:red', zorder=4)

    ax.plot(0.30, 2.10, marker='*', markersize=14, color='tab:green', zorder=7)
    ax.plot(3.30, 0.30, marker='*', markersize=14, color='tab:orange', zorder=7)

    color_estado = {'AVANZAR_PARALELO': 'black', 'GIRAR': 'purple'}.get(estado, 'black')
    info = f'estado={estado}  celdas={num_celdas}  giros={num_giros}\n{decision_info}'
    ax.set_title(info, fontsize=9, color=color_estado)

    ax.set_xlim(-0.3, 3.9)
    ax.set_ylim(2.7, -0.3)  # invertido: Y crece hacia abajo, igual que el plano
    ax.set_aspect('equal')


if __name__ == '__main__':
    main()
