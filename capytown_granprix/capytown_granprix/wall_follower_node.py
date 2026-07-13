#!/usr/bin/env python3
"""Nodo de control de seguimiento de pared (AVANZAR_PARALELO).

Se suscribe a ``/lidar_zones`` y publica una velocidad SUGERIDA en
``/wall_follow/cmd_vel_suggestion`` (geometry_msgs/Twist). Este nodo
NO escribe directamente en ``/cmd_vel``: el nodo de decision
(state_machine_node) es el unico que actua sobre el robot, y solo
reenvia esta sugerencia mientras el estado sea AVANZAR_PARALELO. Esto
evita que dos nodos publiquen comandos de movimiento en simultaneo.

Parametro ``lado_seguimiento`` (DERECHA por defecto, o IZQUIERDA):
elige que lado del LiDAR usar (``right_line_*``/``left_line_*`` de
``lidar_processor_node``). Debe coincidir con el mismo parametro en
``state_machine_node`` (prioridad de giro en DECIDIR y referencia de
ALINEAR) -- ver README, seccion "Seguimiento de pared izquierda".

Logica de control -- REGRESION DE LINEA + Kp (ajuste por minimos
cuadrados a todos los puntos del lado elegido, no solo 2 como el
metodo original de S1/S2). Se corrigen el ANGULO de la recta
(paralelismo) y la DISTANCIA perpendicular hacia
``distancia_objetivo_m`` SIMULTANEAMENTE (suma ponderada), no
alternando entre uno u otro -- alternar crea un ciclo que no se
amortigua (cada correccion de distancia induce un error de angulo al
girar, que dispara la correccion de angulo, que vuelve a inducir
error de distancia). Validado en ``sim_local/`` (simulador local sin
ROS2): alternando oscilaba ±1.4 cm indefinidamente; sumando converge
con std < 0.01 cm.

Convencion de signos de angular.z (REP-103): positivo = giro hacia la
izquierda (antihorario), negativo = giro hacia la derecha (horario).

Nota de signo entre lados (importante si se toca esto): el termino de
ANGULO (paralelismo) usa el MISMO signo para ambos lados -- rotar el
robot un angulo theta rota por igual, en el marco del robot, tanto la
pared derecha como la izquierda (son paralelas entre si en el mundo),
asi que ``left_line_angle_rad`` ~ ``right_line_angle_rad`` ~ -theta
para el mismo theta. El termino de DISTANCIA en cambio SI cambia de
signo entre lados: "demasiado cerca de la pared derecha" corrige
girando hacia la izquierda (alejarse), pero "demasiado cerca de la
pared izquierda" corrige girando hacia la DERECHA (alejarse) -- es un
espejo. Verificado en ``sim_local/`` con un pasillo espejado antes de
portarlo al robot real.

Cuando no hay pared de referencia (del lado elegido) hay dos casos,
indistinguibles en la lectura actual del LiDAR (el MS200 no reporta
puntos por debajo de su rango minimo, ~3 cm -- un pasillo abierto y
una pared demasiado cerca para medir se ven IGUAL: sin puntos validos):

- **Pasillo genuinamente abierto:** mantener rumbo con Kp de heading
  sobre el yaw de ``/odom_raw``, en vez de simplemente anular la
  correccion -- evita que un sesgo mecanico del chasis lo desvie
  lentamente sin que nada lo corrija.
- **Pared demasiado cerca para medir:** se detecta con la ULTIMA
  distancia valida conocida (``_ultima_distancia_valida``) -- si era
  menor a ``umbral_muy_cerca_m`` justo antes de perder la lectura, es
  mucho mas probable que el robot se haya acercado demasiado que que
  el pasillo se haya abierto de repente. En ese caso se gira
  activamente lejos de la pared (angular maximo, con el signo del
  lado elegido) en vez de mantener rumbo -- si no, el robot no
  corrige nada y, si el rumbo apunta un poco hacia afuera, se aleja
  sin control y nunca vuelve (bug real encontrado y corregido
  probando en el robot, validado en ``sim_local/`` con un empujon
  manual gradual antes de portarlo aqui).

Modo de prueba (``publicar_directo_en_cmd_vel``): para calibrar SOLO el
seguimiento recto, sin que ``state_machine_node`` interrumpa con fases
de celda/cruce/giro, este nodo puede publicar la misma velocidad
directo en ``/cmd_vel`` ademas de la sugerencia normal. Util para
correr unicamente ``lidar_processor_node`` + ``wall_follower_node`` en
un pasillo largo. NO usar este modo junto con ``state_machine_node``
corriendo (dos nodos escribirian en ``/cmd_vel`` a la vez).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from capytown_interfaces.msg import LidarZones
from capytown_granprix.geometry_utils import angle_diff, clamp, yaw_from_quaternion


class WallFollowerNode(Node):

    def __init__(self):
        super().__init__('wall_follower')

        self.declare_parameter('lidar_zones_topic', '/lidar_zones')
        self.declare_parameter('odom_topic', '/odom_raw')
        self.declare_parameter('output_topic', '/wall_follow/cmd_vel_suggestion')
        self.declare_parameter('lado_seguimiento', 'DERECHA')
        self.declare_parameter('distancia_objetivo_m', 0.12)
        self.declare_parameter('velocidad_lineal_mps', 0.15)
        self.declare_parameter('ganancia_angulo', 2.0)
        self.declare_parameter('ganancia_distancia', 2.0)
        self.declare_parameter('ganancia_heading', 2.0)
        self.declare_parameter('angular_max_radps', 0.6)
        self.declare_parameter('umbral_muy_cerca_m', 0.06)
        self.declare_parameter('frente_minimo_seguro_m', 0.15)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('publicar_directo_en_cmd_vel', False)

        self._zones_topic = self.get_parameter('lidar_zones_topic').value
        self._odom_topic = self.get_parameter('odom_topic').value
        self._output_topic = self.get_parameter('output_topic').value
        self._lado = str(self.get_parameter('lado_seguimiento').value).strip().upper()
        if self._lado not in ('DERECHA', 'IZQUIERDA'):
            raise ValueError(f"lado_seguimiento invalido: {self._lado!r} (usar DERECHA o IZQUIERDA)")
        # Signo del termino de DISTANCIA (ver nota de modulo): +1 para
        # DERECHA, -1 para IZQUIERDA -- el termino de ANGULO NO cambia
        # de signo entre lados.
        self._signo_lado = 1.0 if self._lado == 'DERECHA' else -1.0
        self._distancia_objetivo = float(self.get_parameter('distancia_objetivo_m').value)
        self._v_base = float(self.get_parameter('velocidad_lineal_mps').value)
        self._k_angulo = float(self.get_parameter('ganancia_angulo').value)
        self._k_distancia = float(self.get_parameter('ganancia_distancia').value)
        self._k_heading = float(self.get_parameter('ganancia_heading').value)
        self._angular_max = float(self.get_parameter('angular_max_radps').value)
        self._umbral_muy_cerca = float(self.get_parameter('umbral_muy_cerca_m').value)
        self._frente_minimo = float(self.get_parameter('frente_minimo_seguro_m').value)
        self._publicar_directo = bool(self.get_parameter('publicar_directo_en_cmd_vel').value)

        self._yaw = 0.0
        self._heading_objetivo = None
        self._ultima_distancia_valida = None

        self._pub = self.create_publisher(Twist, self._output_topic, 10)
        self._cmd_vel_pub = None
        if self._publicar_directo:
            cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
            self._cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
            self.get_logger().warn(
                f'MODO DE PRUEBA activo: publicando directo en {cmd_vel_topic}. '
                'No correr junto con state_machine_node.'
            )

        self._sub = self.create_subscription(
            LidarZones, self._zones_topic, self._on_zones, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)

        self.get_logger().info(
            f'wall_follower listo: lado={self._lado}, objetivo={self._distancia_objetivo * 100:.1f} cm, '
            f'v_base={self._v_base:.2f} m/s'
        )

    def _publish(self, cmd: Twist) -> None:
        self._pub.publish(cmd)
        if self._cmd_vel_pub is not None:
            self._cmd_vel_pub.publish(cmd)

    def _on_odom(self, msg: Odometry) -> None:
        self._yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def _on_zones(self, msg: LidarZones) -> None:
        cmd = Twist()

        if msg.front_valid and msg.front < self._frente_minimo:
            # Seguridad redundante: si hay pared muy cerca al frente,
            # no avanzar aunque el estado siga siendo AVANZAR_PARALELO
            # (el nodo de decision debera reaccionar en su propio ciclo).
            self._publish(cmd)
            return

        if self._lado == 'DERECHA':
            linea_valida = msg.right_line_valid
            linea_angulo = msg.right_line_angle_rad
            linea_distancia = msg.right_line_distance_m
        else:
            linea_valida = msg.left_line_valid
            linea_angulo = msg.left_line_angle_rad
            linea_distancia = msg.left_line_distance_m

        if not linea_valida:
            muy_cerca = (
                self._ultima_distancia_valida is not None
                and self._ultima_distancia_valida < self._umbral_muy_cerca
            )
            if muy_cerca:
                # Probablemente el robot se acerco demasiado a la pared
                # del lado elegido (mas cerca que el rango minimo del
                # LiDAR) en vez de que el pasillo se haya abierto de
                # repente. Girar activamente lejos hasta recuperar una
                # lectura valida (signo del lado: alejarse de la pared
                # derecha es +angular_max, de la izquierda es
                # -angular_max) -- mantener rumbo aqui (como en el caso
                # "pasillo abierto") dejaria al robot sin corregir nada.
                self._heading_objetivo = None
                cmd.linear.x = self._v_base
                cmd.angular.z = self._signo_lado * self._angular_max
                self._publish(cmd)
                return

            # Sin referencia confiable de pared del lado elegido (pasillo
            # abierto): mantener el rumbo con un Kp de heading sobre el
            # yaw de odometria, en vez de simplemente anular la
            # correccion (eso dejaba que un sesgo mecanico del chasis
            # desviara el robot sin que nada lo corrigiera).
            if self._heading_objetivo is None:
                self._heading_objetivo = self._yaw
            error_heading = angle_diff(self._heading_objetivo, self._yaw)
            correccion = self._k_heading * error_heading
            cmd.linear.x = self._v_base
            cmd.angular.z = clamp(correccion, -self._angular_max, self._angular_max)
            self._publish(cmd)
            return

        # Hay pared valida del lado elegido: al recuperarla, olvidar el
        # heading objetivo anterior para que la proxima vez que se
        # pierda la pared se capture un rumbo fresco (no uno
        # desactualizado), y guardar esta distancia como la ultima
        # valida conocida.
        self._heading_objetivo = None
        self._ultima_distancia_valida = linea_distancia

        # Correccion de angulo y distancia SUMADAS (no alternadas -- ver
        # nota del modulo). Geometria del termino de angulo (verificada
        # en sim_local/ antes de portarla aqui, MISMO signo para ambos
        # lados): si la pared es una recta horizontal en el mundo y el
        # robot tiene yaw theta respecto a ella, el angulo que se ve EN
        # EL MARCO DEL ROBOT es linea_angulo = atan(pendiente_local) =
        # -theta (para right_line_* y left_line_* por igual, son
        # paralelas entre si). Para corregir theta -> 0 se necesita
        # angular.z = -k*theta = +k*linea_angulo (SIN signo negativo).
        # Con el signo cambiado, el lazo es de realimentacion POSITIVA y
        # el robot diverge en menos de 1 s.
        #
        # El termino de DISTANCIA SI cambia de signo entre lados (ver
        # nota de modulo) -- multiplicado por self._signo_lado (+1
        # derecha, -1 izquierda).
        error_distancia = self._distancia_objetivo - linea_distancia
        correccion = (
            self._k_angulo * linea_angulo
            + self._signo_lado * self._k_distancia * error_distancia
        )

        cmd.linear.x = self._v_base
        cmd.angular.z = clamp(correccion, -self._angular_max, self._angular_max)
        self._publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
