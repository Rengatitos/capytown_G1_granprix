#!/usr/bin/env python3
"""Nodo de decision de intersecciones y maquina de estados principal.

Es el UNICO nodo que escribe en ``/cmd_vel``: centraliza toda decision
de movimiento para evitar que dos publicadores manden comandos
contradictorios al mismo tiempo. Mientras el estado es
AVANZAR_PARALELO, reenvia la sugerencia de ``wall_follower_node``
(``/wall_follow/cmd_vel_suggestion``); en el resto de estados calcula
sus propios comandos (girar detenido-lento, alinear, detener).

Maquina de estados (ver logica_pared_derecha_robot.md y
DETALLE RETO 3.md):

    INICIAR -> AVANZAR_PARALELO -> DETECTAR_CRUCE -> BUSCAR_PARE
    -> DECIDIR -> PAUSA_GIRO -> GIRAR -> ALINEAR -> VERIFICAR_META
    -> (META o vuelve a AVANZAR_PARALELO)

    PAUSA_GIRO (fuera de la lista original del documento de referencia)
    es una espera fija de ``tiempo_pausa_antes_girar_s`` con el robot
    detenido entre "ya decidi" y "empiezo a girar", para que el giro se
    vea como un movimiento separado del avance.

Se agrega un estado adicional ``DETENIDO`` (fuera de la lista pedida)
solo como red de seguridad ante un limite de celdas recorridas sin
llegar a la meta (evita loops infinitos por fallas de sensor); no
reemplaza ni altera el flujo principal solicitado.

Nota sobre giros con chasis Ackermann: un vehiculo con direccion
Ackermann no puede rotar sobre su propio eje (radio de giro cero). El
estado GIRAR aproxima el "giro detenido" del documento de referencia
con un arco de avance lento y radio de giro pequeno (velocidad lineal
baja + angular maxima), usando el yaw de ``/odom_raw`` como
referencia de cierre en vez de tiempo fijo. Se probo dos veces cerrar
el lazo con el giroscopio del IMU (``/imu``) en vez de odometria --
revertido ambas veces: la primera, el topico publicaba solo ceros (el
agente micro-ROS no estaba corriendo); la segunda, con el agente ya
activo y con calibracion de sesgo (bias) del giroscopio antes de cada
giro, el angulo integrado igual salio muy lejos del real en pista
(error de +250 grados en un giro de 180), asi que se dejo con
odometria de forma definitiva. Ademas de la tolerancia angular, GIRAR
tiene un limite duro de tiempo (``tiempo_max_girar_s``) por si la
odometria tampoco cierra el error a tiempo (falla de sensor, deriva
severa, etc.) -- evita que el robot quede girando sin parar. Esto se
debe calibrar en pista (ver README).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from capytown_interfaces.msg import LidarZones, RobotEvent
from capytown_granprix import event_types as EV
from capytown_granprix.geometry_utils import angle_diff, normalize_angle, yaw_from_quaternion
from capytown_granprix.grid_map import GridTracker, cell_from_name, neighbor_cell, turn_left, turn_right
from capytown_granprix.maze_map import MazeMap

# Color RGB por tipo de marcador en RViz (visualization_msgs/Marker).
_MARKER_COLORS = {
    'CRUCE': (0.2, 0.4, 1.0),
    'PARE_DETECTADO': (1.0, 0.1, 0.1),
    'PARE_RESPETADO': (0.1, 0.8, 0.1),
}


class StateMachineNode(Node):

    def __init__(self):
        super().__init__('state_machine')
        self._declare_parameters()
        self._read_parameters()

        self._grid = GridTracker.from_cell_name(self._celda_inicio, self._heading_inicial)

        # Mapa de conectividad: se construye SIEMPRE durante la corrida
        # (registra cada interseccion confirmada por LiDAR), y en Ronda 2
        # se usa para calcular con BFS la ruta mas corta CONOCIDA en vez
        # de repetir la exploracion reactiva -- ver README, seccion
        # "Memoria de ruta (Ronda 2)". Requiere que el layout de paredes
        # NO cambie entre rondas (usar_mapa_ronda2: false si tu evaluacion
        # si las reconfigura).
        self._maze_map = MazeMap(num_columns=self._grid.num_columns, num_rows=self._grid.num_rows)
        self._plan = None
        self._plan_index = 0
        if self._ronda == 2 and self._usar_mapa_ronda2:
            self._cargar_plan_ronda2()

        # Si True, el PROXIMO retroceso-antes-de-girar se omite (solo
        # para el giro forzado de INICIO, ver _handle_iniciar) -- se
        # resetea a False despues de usarse, asi que los giros
        # normales de DECIDIR si retroceden.
        self._omitir_retroceso_giro = False

        # Estado de la maquina
        self._state = 'INICIAR'
        self._terminado = False

        # Datos de sensores (ultimo valor recibido)
        self._zones = None
        self._zones_ready = False
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._yaw = 0.0
        self._odom_ready = False
        self._pare_activo = False
        self._wall_follow_cmd = Twist()

        # Variables de trabajo por estado
        self._cell_start_xy = (0.0, 0.0)
        self._num_celdas = 0
        self._cruce_muestras = None
        self._derecha_libre = False
        self._frente_libre = False
        self._izquierda_libre = False
        self._buscar_pare_start = None
        self._pare_hold_start = None
        self._celdas_pare_respetadas = set()
        self._decision_actual = 'NINGUNO'
        self._alinear_start = None
        self._pausa_giro_start = None

        self._giro_objetivo = 0.0
        # Red de seguridad: limite de tiempo duro para GIRAR, ademas de
        # la tolerancia angular -- si la odometria no cierra el error a
        # tiempo (falla de sensor, deriva severa), evita que el robot
        # quede girando sin parar. Igual que ya tiene ALINEAR con
        # tiempo_max_alinear_s.
        self._girar_start = None

        self._esperando_obstaculo = False
        self._espera_obstaculo_inicio = None

        self._STATE_HANDLERS = {
            'INICIAR': self._handle_iniciar,
            'AVANZAR_PARALELO': self._handle_avanzar_paralelo,
            'DETECTAR_CRUCE': self._handle_detectar_cruce,
            'BUSCAR_PARE': self._handle_buscar_pare,
            'DECIDIR': self._handle_decidir,
            'PAUSA_GIRO': self._handle_pausa_giro,
            'GIRAR': self._handle_girar,
            'ALINEAR': self._handle_alinear,
            'VERIFICAR_META': self._handle_verificar_meta,
            'META': self._handle_meta,
            'DETENIDO': self._handle_detenido,
        }

        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._event_pub = self.create_publisher(RobotEvent, self._event_topic, 10)
        self._state_pub = self.create_publisher(String, self._robot_state_topic, 10)
        self._cell_pub = self.create_publisher(String, self._cell_topic, 10)
        self._path_pub = self.create_publisher(Path, self._path_topic, 10)
        self._markers_pub = self.create_publisher(MarkerArray, self._markers_topic, 10)

        # Trayectoria (RViz): acumula poses de /odom_raw, decimada por
        # distancia para no publicar un Path gigante a la frecuencia
        # cruda del odometro.
        self._path_msg = Path()
        self._path_last_xy = None

        # Marcadores (RViz): un marcador persistente por cada interseccion
        # detectada y cada evento de PARE, en la posicion del robot en ese
        # instante (odom corregido).
        self._markers = MarkerArray()
        self._marker_id = 0

        self.create_subscription(
            LidarZones, self._lidar_zones_topic, self._on_zones, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self.create_subscription(Bool, self._pare_topic, self._on_pare, 10)
        self.create_subscription(Twist, self._wall_follow_topic, self._on_wall_follow, 10)

        self.create_timer(1.0 / self._control_rate_hz, self._on_timer)

        self.get_logger().info(
            f'state_machine listo: inicio={self._celda_inicio} meta={self._celda_meta} '
            f'heading_inicial={self._heading_inicial}'
        )

    # ------------------------------------------------------------------
    # Parametros
    # ------------------------------------------------------------------
    def _declare_parameters(self):
        defaults = {
            'lidar_zones_topic': '/lidar_zones',
            'odom_topic': '/odom_raw',
            'cmd_vel_topic': '/cmd_vel',
            'wall_follow_topic': '/wall_follow/cmd_vel_suggestion',
            'pare_topic': '/pare_detectado',
            'event_topic': '/robot_event',
            'robot_state_topic': '/robot_state',
            'cell_topic': '/robot_cell',
            'path_topic': '/trayectoria',
            'markers_topic': '/marcadores_granprix',
            'path_min_dist_m': 0.02,
            'usar_camara': True,
            'control_rate_hz': 20.0,
            # Lado de la pared que se sigue: DERECHA (por defecto) o
            # IZQUIERDA. Cambia la prioridad de giro en DECIDIR (el lado
            # elegido se evalua primero) y la referencia de ALINEAR
            # (right_front/right_rear vs left_front/left_rear). Debe
            # coincidir con el mismo parametro en wall_follower.
            'lado_seguimiento': 'DERECHA',
            # Modo de prueba: si es true, se saltan DETECTAR_CRUCE y
            # BUSCAR_PARE -- decide con una lectura unica (sin
            # confirmar con varias muestras). ALINEAR SI corre (no se
            # salta): es el paso que corrige el giro contra la pared
            # real via LiDAR en vez de confiar solo en el angulo
            # objetivo fijo + odometria. Util para calibrar el giro de
            # forma aislada, con el feedback de alineacion incluido.
            'modo_simplificado': False,
            'umbral_frente_pared_m': 0.25,
            'umbral_frente_libre_m': 0.35,
            'umbral_lado_libre_m': 0.40,
            # Regla general de seguridad (siempre activa, en cualquier
            # estado): objeto al frente mas cerca que esto -> detenerse
            # de inmediato, esperar y volver a preguntar si esta libre.
            'umbral_colision_m': 0.10,
            # Igual que umbral_colision_m pero para el lado que se
            # sigue de cerca (lado_seguimiento) -- mas ajustado porque
            # el objetivo normal de seguimiento (distancia_objetivo_m)
            # ya deja al robot a ~12 cm de esa pared todo el tiempo; si
            # este umbral fuera igual de holgado que el del frente,
            # dispararia constantemente durante el seguimiento normal.
            'umbral_colision_lateral_m': 0.05,
            'tiempo_espera_obstaculo_s': 2.0,
            'distancia_celda_m': 0.60,
            'margen_avance_m': 0.05,
            'muestras_confirmacion': 5,
            'consenso_minimo': 4,
            'velocidad_giro_lineal_mps': 0.08,
            'velocidad_giro_angular_radps': 0.5,
            'tolerancia_giro_deg': 4.0,
            # Angulo objetivo de giro para DERECHA (ATRAS siempre es 180,
            # no usa este valor; IZQUIERDA usa angulo_giro_izquierda_deg
            # abajo, no este). 90 es el giro "real" de una esquina en
            # grilla; un poco mas (ej. 95) compensa que el arco Ackermann
            # suele quedar corto del objetivo.
            'angulo_giro_deg': 90.0,
            # Angulo objetivo de giro IZQUIERDA en DECIDIR (pedido
            # explicitamente mayor que angulo_giro_deg: en pista el giro
            # izquierdo reactivo quedaba corto del objetivo real con el
            # mismo valor que DERECHA). NO afecta el giro inicial forzado
            # de INICIO (ver angulo_giro_inicial_deg).
            'angulo_giro_izquierda_deg': 120.0,
            # Angulo fijo del giro inicial forzado en INICIO (pedido
            # explicitamente como 90 grados, independiente de
            # angulo_giro_izquierda_deg -- no cambia si se recalibra el
            # giro reactivo normal).
            'angulo_giro_inicial_deg': 90.0,
            # Pausa fija (segundos) con el robot detenido entre DECIDIR
            # (ya sabe que va a girar) y el inicio del arco de GIRAR --
            # pedido para que el giro sea un movimiento claramente
            # separado del avance, no una transicion instantanea.
            'tiempo_pausa_antes_girar_s': 1.0,
            # Retroceso corto AL INICIO de esa pausa (incluido en
            # tiempo_pausa_antes_girar_s, no se suma aparte) para ganar
            # margen antes de pivotar -- si el arco de GIRAR arranca
            # pegado a la pared/esquina, el sensor frontal no ve el
            # riesgo de terminar muy cerca del siguiente tramo.
            'tiempo_retroceso_giro_s': 0.4,
            'velocidad_retroceso_giro_mps': 0.05,
            # Limite duro para GIRAR: si el sensor de giro (IMU) no
            # completa el angulo objetivo en este tiempo (falla de
            # sensor, topico vacio, etc.), se fuerza a terminar el giro
            # igual -- evita que el robot quede girando sin parar.
            'tiempo_max_girar_s': 6.0,
            'tolerancia_alineacion_m': 0.02,
            'tiempo_max_alinear_s': 4.0,
            'velocidad_alineacion_lineal_mps': 0.06,
            'velocidad_alineacion_angular_radps': 0.3,
            'tiempo_pare_s': 3.0,
            'tiempo_espera_camara_s': 0.5,
            'celda_inicio': 'A4',
            'celda_meta': 'F1',
            'heading_inicial': 'NORTE',
            'max_celdas_recorridas': 60,
            # Ronda 1 = exploracion (siempre reactiva, va por todos
            # lados). Ronda 2 = time attack: si usar_mapa_ronda2 es true,
            # intenta cargar el mapa guardado al final de la Ronda 1 y
            # seguir la ruta mas corta CONOCIDA via BFS; si el layout de
            # paredes cambia entre rondas en tu evaluacion, poner esto en
            # false (Ronda 2 vuelve a ser 100% reactiva, igual que la 1).
            'ronda': 1,
            'usar_mapa_ronda2': True,
            'mapa_path': '~/capytown_resultados/mapa_granprix.json',
            # Factores de correccion de escala del odometro (calibrados en
            # pista: avance real 76 cm / odometro 78.3 cm y giro real 90 /
            # odometro 90.92). Dejar en 1.0 si se recalibra desde cero.
            'factor_dist_odom': 0.9474,
            'factor_ang_odom': 0.9899,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self):
        g = lambda name: self.get_parameter(name).value  # noqa: E731

        self._lidar_zones_topic = g('lidar_zones_topic')
        self._odom_topic = g('odom_topic')
        self._cmd_vel_topic = g('cmd_vel_topic')
        self._wall_follow_topic = g('wall_follow_topic')
        self._pare_topic = g('pare_topic')
        self._event_topic = g('event_topic')
        self._robot_state_topic = g('robot_state_topic')
        self._cell_topic = g('cell_topic')
        self._path_topic = g('path_topic')
        self._markers_topic = g('markers_topic')
        self._path_min_dist = float(g('path_min_dist_m'))

        self._usar_camara = bool(g('usar_camara'))
        self._control_rate_hz = float(g('control_rate_hz'))
        self._lado = str(g('lado_seguimiento')).strip().upper()
        if self._lado not in ('DERECHA', 'IZQUIERDA'):
            raise ValueError(f"lado_seguimiento invalido: {self._lado!r} (usar DERECHA o IZQUIERDA)")
        self._modo_simplificado = bool(g('modo_simplificado'))

        self._umbral_frente_pared = float(g('umbral_frente_pared_m'))
        self._umbral_frente_libre = float(g('umbral_frente_libre_m'))
        self._umbral_lado_libre = float(g('umbral_lado_libre_m'))
        self._umbral_colision = float(g('umbral_colision_m'))
        self._umbral_colision_lateral = float(g('umbral_colision_lateral_m'))
        self._distancia_celda = float(g('distancia_celda_m'))
        self._margen_avance = float(g('margen_avance_m'))

        self._muestras_confirmacion = int(g('muestras_confirmacion'))
        self._consenso_minimo = int(g('consenso_minimo'))

        self._v_giro_lineal = float(g('velocidad_giro_lineal_mps'))
        self._v_giro_angular = float(g('velocidad_giro_angular_radps'))
        self._tolerancia_giro_rad = math.radians(float(g('tolerancia_giro_deg')))
        self._angulo_giro_rad = math.radians(float(g('angulo_giro_deg')))
        self._angulo_giro_izquierda_rad = math.radians(float(g('angulo_giro_izquierda_deg')))
        self._angulo_giro_inicial_rad = math.radians(float(g('angulo_giro_inicial_deg')))
        self._tiempo_pausa_antes_girar = float(g('tiempo_pausa_antes_girar_s'))
        self._tiempo_retroceso_giro = float(g('tiempo_retroceso_giro_s'))
        self._v_retroceso_giro = float(g('velocidad_retroceso_giro_mps'))
        self._tiempo_max_girar = float(g('tiempo_max_girar_s'))

        self._tolerancia_alineacion = float(g('tolerancia_alineacion_m'))
        self._tiempo_max_alinear = float(g('tiempo_max_alinear_s'))
        self._v_alinear_lineal = float(g('velocidad_alineacion_lineal_mps'))
        self._v_alinear_angular = float(g('velocidad_alineacion_angular_radps'))

        self._tiempo_pare = float(g('tiempo_pare_s'))
        self._tiempo_espera_camara = float(g('tiempo_espera_camara_s'))

        self._tiempo_espera_obstaculo = float(g('tiempo_espera_obstaculo_s'))

        self._celda_inicio = str(g('celda_inicio'))
        self._celda_meta = str(g('celda_meta'))
        self._heading_inicial = str(g('heading_inicial'))
        self._max_celdas = int(g('max_celdas_recorridas'))

        self._ronda = int(g('ronda'))
        self._usar_mapa_ronda2 = bool(g('usar_mapa_ronda2'))
        self._mapa_path = str(g('mapa_path'))

        self._factor_dist_odom = float(g('factor_dist_odom'))
        self._factor_ang_odom = float(g('factor_ang_odom'))

    # ------------------------------------------------------------------
    # Callbacks de suscripcion
    # ------------------------------------------------------------------
    def _on_zones(self, msg: LidarZones):
        self._zones = msg
        self._zones_ready = True

    def _on_odom(self, msg: Odometry):
        # Correccion de escala del odometro (medida en pista, ver README):
        # el ROSMASTER R2 sobreestima tanto distancia como angulo girado,
        # de forma consistente, por lo que se corrige con un factor fijo.
        self._odom_x = msg.pose.pose.position.x * self._factor_dist_odom
        self._odom_y = msg.pose.pose.position.y * self._factor_dist_odom
        self._yaw = yaw_from_quaternion(msg.pose.pose.orientation) * self._factor_ang_odom
        self._odom_ready = True
        self._update_path(msg.header)

    def _on_pare(self, msg: Bool):
        self._pare_activo = bool(msg.data)

    def _on_wall_follow(self, msg: Twist):
        self._wall_follow_cmd = msg

    # ------------------------------------------------------------------
    # Ciclo de control principal
    # ------------------------------------------------------------------
    def _on_timer(self):
        self._cell_pub.publish(String(data=f'{self._grid.cell}|{self._grid.heading}'))

        if not (self._odom_ready and self._zones_ready):
            return

        if self._state == 'AVANZAR_PARALELO' and self._handle_obstaculo_frente():
            return

        self._STATE_HANDLERS[self._state]()

    def _handle_obstaculo_frente(self) -> bool:
        """Regla de seguridad SOLO para AVANZAR_PARALELO (avance recto).

        Si hay un objeto al frente mas cerca que ``umbral_colision_m``,
        detiene el robot de inmediato, espera ``tiempo_espera_obstaculo_s``
        y vuelve a comprobar si ya esta libre; si sigue bloqueado,
        reinicia la espera (queda preguntando en bucle hasta que se
        libere). Retorna True si este ciclo ya publico un comando (el
        llamador debe omitir el despacho normal de estados).

        IMPORTANTE: no se aplica en el resto de estados (a proposito).
        En una interseccion o callejon sin salida real de un pasillo de
        60 cm, la pared al frente esta normalmente a menos de
        ``umbral_colision_m`` mientras el robot esta parado decidiendo
        o girando -- eso es esperado, no un choque. Si esta regla
        corriera en TODOS los estados (como antes), el robot quedaba
        atrapado esperando indefinidamente a que esa pared "se libere"
        sin poder llegar nunca a GIRAR (que es lo que en realidad lo
        aleja de ella) -- bug real encontrado en pista: el robot se
        quedaba parado sin razon aparente en ciertas intersecciones.
        """
        if self._terminado:
            return False

        z = self._zones
        frente_bloqueado = z.front_valid and z.front < self._umbral_colision

        # Ademas del frente, vigilar el lado que se sigue de cerca
        # (lado_seguimiento, objetivo 12 cm) con un umbral MAS
        # ajustado que umbral_colision_m -- si solo se chequeara el
        # frente, un obstaculo angosto que sobresale a un costado (ej.
        # una "media pared" suelta, ver _filtrar_fuera_de_rejilla) nunca
        # dispara esta regla y el robot lo puede rozar sin que se
        # registre ningun evento COLISION (encontrado en pista real:
        # choque fisico contra un panel lateral, 0 colisiones en las
        # metricas). El lado opuesto no se vigila -- no se sigue de
        # cerca, no deberia estar nunca a esta distancia salvo ruido.
        if self._lado == 'IZQUIERDA':
            lado_valid, lado_dist = z.left_valid, z.left
        else:
            lado_valid, lado_dist = z.right_valid, z.right
        lado_bloqueado = lado_valid and lado_dist < self._umbral_colision_lateral

        bloqueado = frente_bloqueado or lado_bloqueado

        if self._esperando_obstaculo:
            if bloqueado:
                self._publish_twist(Twist())
                elapsed = (
                    self.get_clock().now() - self._espera_obstaculo_inicio
                ).nanoseconds / 1e9
                if elapsed >= self._tiempo_espera_obstaculo:
                    # Se cumplio la espera y sigue bloqueado: volver a
                    # preguntar en el proximo ciclo tras otra espera igual.
                    self._espera_obstaculo_inicio = self.get_clock().now()
                return True
            self._esperando_obstaculo = False
            return False

        if bloqueado:
            self._publish_twist(Twist())
            if frente_bloqueado:
                detalle = f'frente a {z.front:.2f} m'
            else:
                detalle = f'lado {self._lado.lower()} a {lado_dist:.2f} m'
            self._publish_event(
                EV.COLISION, f'obstaculo ({detalle}) cerca de {self._grid.cell}'
            )
            self._esperando_obstaculo = True
            self._espera_obstaculo_inicio = self.get_clock().now()
            return True

        return False

    def _filtrar_fuera_de_rejilla(self, derecha_libre: bool, frente_libre: bool, izquierda_libre: bool):
        """Descarta como False cualquier direccion que el LiDAR reporte
        libre pero que llevaria a una celda fuera de la rejilla 6x4 real
        del laberinto (columna A es el borde izquierdo, F el derecho,
        fila 1 el borde superior, fila 4 el inferior -- no hay celdas
        mas alla, salvo las aberturas de INICIO/META que no se evaluan
        aqui). Una lectura de "libre" hacia afuera de la rejilla es
        necesariamente un falso positivo del LiDAR (esquina, ruido, o
        un nicho de una media pared/chican), nunca un camino real --
        bug real encontrado en pista: giraba hacia "fuera" del
        laberinto, la celda se recortaba al borde sin avisar, y la
        posicion estimada quedaba desincronizada de la real para el
        resto de la corrida."""
        col, row, heading = self._grid.col, self._grid.row, self._grid.heading
        cols, rows = self._grid.num_columns, self._grid.num_rows

        if derecha_libre and neighbor_cell(col, row, turn_right(heading), cols, rows) is None:
            derecha_libre = False
        if frente_libre and neighbor_cell(col, row, heading, cols, rows) is None:
            frente_libre = False
        if izquierda_libre and neighbor_cell(col, row, turn_left(heading), cols, rows) is None:
            izquierda_libre = False

        return derecha_libre, frente_libre, izquierda_libre

    # ------------------------------------------------------------------
    # Estados
    # ------------------------------------------------------------------
    def _handle_iniciar(self):
        self._publish_event(
            EV.INICIO, f'inicio en {self._grid.cell}, heading {self._grid.heading}'
        )
        # Primer movimiento forzado: gira 90 grados a la izquierda antes
        # de empezar la exploracion reactiva normal, pedido
        # explicitamente -- no depende de lo que lea el LiDAR en este
        # primer instante (se salta BUSCAR_PARE tambien, no es una
        # interseccion real todavia). Reutiliza el mismo flujo de un
        # giro decidido normal (PAUSA_GIRO -> GIRAR -> ALINEAR ->
        # VERIFICAR_META -> AVANZAR_PARALELO).
        self._decision_actual = 'IZQUIERDA'
        self._giro_objetivo = self._compute_turn_target(
            self._yaw, 'IZQUIERDA', angulo_rad=self._angulo_giro_inicial_rad
        )
        self._publish_event(EV.GIRO, f'IZQUIERDA (inicial forzado) desde {self._grid.cell}')
        self._publish_twist(Twist())
        self._pausa_giro_start = self.get_clock().now()
        # Sin retroceso aqui (a diferencia de un giro normal en
        # DECIDIR): en INICIO el robot lo coloca el equipo a mano, no
        # sabemos cuanto espacio libre tiene detras -- a diferencia de
        # un giro durante la exploracion, donde llego ahi avanzando por
        # un pasillo con espacio conocido. Encontrado en pista: el
        # retroceso ciego lo dejo atascado justo en este primer giro.
        self._omitir_retroceso_giro = True
        self._set_state('PAUSA_GIRO')

    def _begin_avanzar_paralelo(self):
        self._cell_start_xy = (self._odom_x, self._odom_y)

    def _handle_avanzar_paralelo(self):
        dx = self._odom_x - self._cell_start_xy[0]
        dy = self._odom_y - self._cell_start_xy[1]
        avance = math.hypot(dx, dy)

        z = self._zones
        frente_cerca = z.front_valid and z.front < self._umbral_frente_pared

        if avance >= (self._distancia_celda - self._margen_avance) or frente_cerca:
            self._publish_twist(Twist())
            self._num_celdas += 1
            fuera_de_rango = self._grid.advance_cell()
            self._publish_event(
                EV.CELDA_AVANZADA, f'celda {self._grid.cell} (#{self._num_celdas})'
            )
            if fuera_de_rango:
                self._publish_event(
                    EV.DERIVA_SOSPECHOSA,
                    f'el avance calculado caia fuera de la rejilla 6x4, se recorto a '
                    f'{self._grid.cell} -- la celda estimada probablemente ya no coincide '
                    'con la posicion fisica real (revisar calibracion de odometria/giro)',
                )

            if self._num_celdas > self._max_celdas:
                self._publish_event(
                    EV.TIMEOUT, 'limite de celdas recorridas alcanzado sin llegar a la meta'
                )
                self._guardar_mapa()
                self._terminado = True
                self._set_state('DETENIDO')
                return

            if self._modo_simplificado:
                # Decidir con una sola lectura, sin confirmar con varias
                # muestras ni pasar por BUSCAR_PARE.
                self._derecha_libre = bool(z.right_valid and z.right > self._umbral_lado_libre)
                self._frente_libre = bool(z.front_valid and z.front > self._umbral_frente_libre)
                self._izquierda_libre = bool(z.left_valid and z.left > self._umbral_lado_libre)
                self._derecha_libre, self._frente_libre, self._izquierda_libre = (
                    self._filtrar_fuera_de_rejilla(
                        self._derecha_libre, self._frente_libre, self._izquierda_libre
                    )
                )
                self._maze_map.record(
                    self._grid.col, self._grid.row, self._grid.heading,
                    self._derecha_libre, self._frente_libre, self._izquierda_libre,
                )
                self._set_state('DECIDIR')
            else:
                self._set_state('DETECTAR_CRUCE')
            return

        self._publish_twist(self._wall_follow_cmd)

    def _handle_detectar_cruce(self):
        self._publish_twist(Twist())

        if self._cruce_muestras is None:
            self._cruce_muestras = {'right': [], 'front': [], 'left': []}

        z = self._zones
        self._cruce_muestras['right'].append(
            bool(z.right_valid and z.right > self._umbral_lado_libre)
        )
        self._cruce_muestras['front'].append(
            bool(z.front_valid and z.front > self._umbral_frente_libre)
        )
        self._cruce_muestras['left'].append(
            bool(z.left_valid and z.left > self._umbral_lado_libre)
        )

        if len(self._cruce_muestras['right']) < self._muestras_confirmacion:
            return

        def consenso(muestras):
            return sum(muestras) >= self._consenso_minimo

        self._derecha_libre = consenso(self._cruce_muestras['right'])
        self._frente_libre = consenso(self._cruce_muestras['front'])
        self._izquierda_libre = consenso(self._cruce_muestras['left'])
        self._cruce_muestras = None

        self._derecha_libre, self._frente_libre, self._izquierda_libre = (
            self._filtrar_fuera_de_rejilla(
                self._derecha_libre, self._frente_libre, self._izquierda_libre
            )
        )

        self._maze_map.record(
            self._grid.col, self._grid.row, self._grid.heading,
            self._derecha_libre, self._frente_libre, self._izquierda_libre,
        )

        self._publish_event(
            EV.CRUCE,
            f'derecha={self._derecha_libre} frente={self._frente_libre} '
            f'izquierda={self._izquierda_libre}',
        )
        self._publish_marker('CRUCE', f'cruce {self._grid.cell}')

        self._buscar_pare_start = self.get_clock().now()
        self._pare_hold_start = None
        self._set_state('BUSCAR_PARE')

    def _handle_buscar_pare(self):
        self._publish_twist(Twist())

        if not self._usar_camara:
            self._set_state('DECIDIR')
            return

        cell = self._grid.cell

        # Si ya se inicio el conteo de los 3 s, completarlo sin importar
        # parpadeos momentaneos de la deteccion (evita abortar el PARE
        # a mitad de camino si la camara pierde el color rojo un frame).
        if self._pare_hold_start is not None:
            elapsed = (self.get_clock().now() - self._pare_hold_start).nanoseconds / 1e9
            if elapsed >= self._tiempo_pare:
                self._celdas_pare_respetadas.add(cell)
                self._publish_event(EV.PARE_RESPETADO, f'PARE respetado en {cell}')
                self._publish_marker('PARE_RESPETADO', f'PARE ok {cell}')
                self._set_state('DECIDIR')
            return

        if self._pare_activo and cell not in self._celdas_pare_respetadas:
            self._publish_event(EV.PARE_DETECTADO, f'senal PARE detectada en {cell}')
            self._publish_marker('PARE_DETECTADO', f'PARE {cell}')
            self._pare_hold_start = self.get_clock().now()
            return

        elapsed_settle = (self.get_clock().now() - self._buscar_pare_start).nanoseconds / 1e9
        if elapsed_settle >= self._tiempo_espera_camara:
            self._set_state('DECIDIR')

    def _cargar_plan_ronda2(self):
        """Carga el mapa guardado al final de la Ronda 1 y calcula con
        BFS la ruta mas corta CONOCIDA de inicio a meta. Si falla
        cualquier paso (no hay archivo, o el mapa no llega a la meta),
        deja ``self._plan`` en None -- la Ronda 2 sigue siendo 100%
        reactiva, como si esta funcionalidad no existiera."""
        try:
            mapa_previo = MazeMap.load(
                self._mapa_path, self._grid.num_columns, self._grid.num_rows
            )
        except (OSError, ValueError) as exc:
            self.get_logger().warn(
                f'no se pudo cargar el mapa de la Ronda 1 ({exc}) -- '
                'Ronda 2 usara la logica reactiva normal.'
            )
            return

        goal_col, goal_row = cell_from_name(self._celda_meta)
        plan = mapa_previo.shortest_headings((self._grid.col, self._grid.row), (goal_col, goal_row))
        if plan is None:
            self.get_logger().warn(
                'el mapa cargado no tiene una ruta conocida de inicio a meta -- '
                'Ronda 2 usara la logica reactiva normal.'
            )
            return

        self._plan = plan
        self.get_logger().info(f'plan de ruta cargado de la Ronda 1: {len(plan)} tramos hasta la meta.')

    def _direccion_planificada(self):
        """Si hay un plan de ruta (Ronda 2) vigente, retorna la direccion
        del siguiente tramo SOLO si coincide con lo que el LiDAR acaba de
        confirmar libre en esta interseccion. Si no hay plan, ya se
        agoto, o no coincide (pared nueva o error de mapa), descarta el
        plan para el resto de la corrida y retorna None -- de ahi en
        adelante sigue con la logica reactiva normal, igual que Ronda 1."""
        if self._plan is None or self._plan_index >= len(self._plan):
            return None

        heading_planeado = self._plan[self._plan_index]
        heading_actual = self._grid.heading
        if heading_planeado == heading_actual:
            direction = 'NINGUNO'
        elif heading_planeado == turn_right(heading_actual):
            direction = 'DERECHA'
        elif heading_planeado == turn_left(heading_actual):
            direction = 'IZQUIERDA'
        else:
            direction = 'ATRAS'

        libre = {
            'DERECHA': self._derecha_libre,
            'NINGUNO': self._frente_libre,
            'IZQUIERDA': self._izquierda_libre,
            'ATRAS': not (self._derecha_libre or self._frente_libre or self._izquierda_libre),
        }[direction]

        if not libre:
            self.get_logger().warn(
                f'plan de ruta no coincide con el LiDAR en {self._grid.cell} '
                '-- sigo con logica reactiva desde aqui.'
            )
            self._plan = None
            return None

        self._plan_index += 1
        return direction

    def _guardar_mapa(self):
        if self._ronda != 1 or not self._usar_mapa_ronda2:
            return
        try:
            self._maze_map.save(self._mapa_path)
            self.get_logger().info(f'mapa de la Ronda 1 guardado en {self._mapa_path}')
        except OSError as exc:
            self.get_logger().warn(f'no se pudo guardar el mapa: {exc}')

    def _handle_decidir(self):
        direction = self._direccion_planificada()
        if direction is None:
            # Regla de la mano: el lado que se sigue (lado_seguimiento)
            # se evalua SIEMPRE primero -- pura wall-following reactiva,
            # sin memoria de celdas visitadas. Este laberinto no tiene
            # bolsillos cerrados (confirmado contra el trazado oficial
            # de la pista): seguir siempre la izquierda alcanza para
            # recorrerlo entero sin quedar en loop, y una memoria de
            # visitadas solo agregaba retrocesos innecesarios.
            if self._lado == 'IZQUIERDA':
                orden = (
                    ('IZQUIERDA', self._izquierda_libre),
                    ('NINGUNO', self._frente_libre),
                    ('DERECHA', self._derecha_libre),
                )
            else:
                orden = (
                    ('DERECHA', self._derecha_libre),
                    ('NINGUNO', self._frente_libre),
                    ('IZQUIERDA', self._izquierda_libre),
                )

            direction = next((d for d, libre in orden if libre), None)
            if direction is None:
                direction = 'ATRAS'
                self._publish_event(EV.DEAD_END, f'callejon sin salida en {self._grid.cell}')

        self._decision_actual = direction

        if direction == 'NINGUNO':
            if self._modo_simplificado:
                self._begin_avanzar_paralelo()
                self._set_state('AVANZAR_PARALELO')
            else:
                self._alinear_start = None
                self._set_state('ALINEAR')
            return

        self._giro_objetivo = self._compute_turn_target(self._yaw, direction)
        self._publish_event(EV.GIRO, f'{direction} desde {self._grid.cell}')
        self._publish_twist(Twist())
        self._pausa_giro_start = self.get_clock().now()
        self._set_state('PAUSA_GIRO')

    def _handle_pausa_giro(self):
        """Primero retrocede un poco (``tiempo_retroceso_giro_s``) para
        ganar espacio antes de pivotar -- si el arco de GIRAR arranca
        pegado a la pared/esquina, puede terminar demasiado cerca del
        siguiente tramo (encontrado en pista: colisiones justo al
        retomar AVANZAR_PARALELO despues de cada giro). Despues del
        retroceso, se queda detenido el resto de
        ``tiempo_pausa_antes_girar_s`` -- separa visiblemente "termine
        de avanzar" de "empiezo a girar" en vez de una transicion
        instantanea."""
        elapsed = (self.get_clock().now() - self._pausa_giro_start).nanoseconds / 1e9
        tiempo_retroceso = 0.0 if self._omitir_retroceso_giro else self._tiempo_retroceso_giro

        if elapsed < tiempo_retroceso:
            cmd = Twist()
            cmd.linear.x = -self._v_retroceso_giro
            self._publish_twist(cmd)
            return

        self._publish_twist(Twist())
        if elapsed >= self._tiempo_pausa_antes_girar:
            self._omitir_retroceso_giro = False
            self._girar_start = self.get_clock().now()
            self._set_state('GIRAR')

    def _compute_turn_target(self, yaw: float, direction: str, angulo_rad: float = None) -> float:
        if direction == 'DERECHA':
            delta = -(angulo_rad if angulo_rad is not None else self._angulo_giro_rad)
        elif direction == 'IZQUIERDA':
            delta = angulo_rad if angulo_rad is not None else self._angulo_giro_izquierda_rad
        elif direction == 'ATRAS':
            delta = math.pi
        else:
            delta = 0.0
        return normalize_angle(yaw + delta)

    def _handle_girar(self):
        error = angle_diff(self._giro_objetivo, self._yaw)

        elapsed = (self.get_clock().now() - self._girar_start).nanoseconds / 1e9
        if elapsed >= self._tiempo_max_girar:
            self._publish_event(
                EV.DERIVA_SOSPECHOSA,
                f'GIRAR supero tiempo_max_girar_s ({self._tiempo_max_girar:.1f}s) sin '
                f'completar el angulo objetivo (error={math.degrees(error):.1f} deg) -- '
                'se fuerza a terminar el giro; revisar calibracion de odometria/giro.',
            )
            self._publish_twist(Twist())
            self._grid.apply_turn(self._decision_actual)
            self._alinear_start = None
            self._set_state('ALINEAR')
            return

        if abs(error) <= self._tolerancia_giro_rad:
            self._publish_twist(Twist())
            self._grid.apply_turn(self._decision_actual)
            # ALINEAR corre siempre, incluso en modo_simplificado: GIRAR
            # por si solo solo cierra el lazo contra el yaw de odometria
            # (un angulo objetivo fijo, con la deriva propia del
            # odometro pese al factor de correccion). ALINEAR corrige
            # ese resultado con el LiDAR real (right_front/right_rear)
            # despues del giro -- es el feedback real, no un angulo fijo.
            self._alinear_start = None
            self._set_state('ALINEAR')
            return

        # Chasis Ackermann: no puede rotar en el sitio. Se aproxima el
        # giro con avance lento + direccion maxima, cerrando el lazo
        # con el yaw de la odometria (no con tiempo fijo).
        cmd = Twist()
        cmd.linear.x = self._v_giro_lineal
        cmd.angular.z = self._v_giro_angular if error > 0.0 else -self._v_giro_angular
        self._publish_twist(cmd)

    def _handle_alinear(self):
        if self._alinear_start is None:
            self._alinear_start = self.get_clock().now()

        z = self._zones
        if self._lado == 'DERECHA':
            front_valido, rear_valido = z.right_front_valid, z.right_rear_valid
            front, rear = z.right_front, z.right_rear
        else:
            front_valido, rear_valido = z.left_front_valid, z.left_rear_valid
            front, rear = z.left_front, z.left_rear

        if not (front_valido and rear_valido):
            # Sin pared de referencia del lado elegido (p.ej. abertura
            # tras el giro): el yaw de GIRAR ya dejo al robot orientado
            # al cardinal correcto, se continua sin correccion adicional.
            self._alinear_start = None
            self._set_state('VERIFICAR_META')
            return

        error_angulo = front - rear
        elapsed = (self.get_clock().now() - self._alinear_start).nanoseconds / 1e9

        if abs(error_angulo) <= self._tolerancia_alineacion or elapsed >= self._tiempo_max_alinear:
            self._publish_twist(Twist())
            self._alinear_start = None
            self._set_state('VERIFICAR_META')
            return

        # Signo de la correccion en espejo entre lados (misma logica que
        # el termino de distancia en wall_follower_node): corregir hacia
        # la pared derecha gira distinto que corregir hacia la izquierda.
        cmd = Twist()
        cmd.linear.x = self._v_alinear_lineal
        if self._lado == 'DERECHA':
            cmd.angular.z = -self._v_alinear_angular if error_angulo > 0.0 else self._v_alinear_angular
        else:
            cmd.angular.z = self._v_alinear_angular if error_angulo > 0.0 else -self._v_alinear_angular
        self._publish_twist(cmd)

    def _handle_verificar_meta(self):
        if self._grid.cell == self._celda_meta:
            self._publish_twist(Twist())
            self._publish_event(EV.META, f'meta alcanzada en {self._grid.cell}')
            self._guardar_mapa()
            self._terminado = True
            self._set_state('META')
            return

        self._begin_avanzar_paralelo()
        self._set_state('AVANZAR_PARALELO')

    def _handle_meta(self):
        self._publish_twist(Twist())

    def _handle_detenido(self):
        self._publish_twist(Twist())

    # ------------------------------------------------------------------
    # Utilidades de publicacion
    # ------------------------------------------------------------------
    def _publish_twist(self, cmd: Twist):
        self._cmd_pub.publish(cmd)

    def _publish_event(self, tipo: str, detalle: str):
        evt = RobotEvent()
        evt.header.stamp = self.get_clock().now().to_msg()
        evt.tipo = tipo
        evt.detalle = detalle
        self._event_pub.publish(evt)
        self.get_logger().info(f'[{tipo}] {detalle}')

    def _update_path(self, odom_header) -> None:
        """Acumula la trayectoria (RViz) decimada por distancia minima."""
        if self._path_last_xy is not None:
            dx = self._odom_x - self._path_last_xy[0]
            dy = self._odom_y - self._path_last_xy[1]
            if math.hypot(dx, dy) < self._path_min_dist:
                return

        if not self._path_msg.poses:
            self._path_msg.header.frame_id = odom_header.frame_id or 'odom'

        self._path_last_xy = (self._odom_x, self._odom_y)

        pose = PoseStamped()
        pose.header.frame_id = self._path_msg.header.frame_id
        pose.header.stamp = odom_header.stamp
        pose.pose.position.x = self._odom_x
        pose.pose.position.y = self._odom_y
        pose.pose.orientation.z = math.sin(self._yaw / 2.0)
        pose.pose.orientation.w = math.cos(self._yaw / 2.0)
        self._path_msg.poses.append(pose)
        self._path_msg.header.stamp = odom_header.stamp
        self._path_pub.publish(self._path_msg)

    def _publish_marker(self, kind: str, texto: str) -> None:
        """Agrega un marcador persistente (RViz) en la posicion actual del
        robot: ``kind`` es 'CRUCE', 'PARE_DETECTADO' o 'PARE_RESPETADO'
        (ver ``_MARKER_COLORS``)."""
        marker = Marker()
        marker.header.frame_id = self._path_msg.header.frame_id or 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = kind
        marker.id = self._marker_id
        self._marker_id += 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self._odom_x
        marker.pose.position.y = self._odom_y
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.08
        r, g, b = _MARKER_COLORS.get(kind, (1.0, 1.0, 1.0))
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.9
        marker.text = texto

        self._markers.markers.append(marker)
        self._markers_pub.publish(self._markers)

    def _set_state(self, new_state: str):
        if new_state != self._state:
            self.get_logger().info(f'estado: {self._state} -> {new_state}')
            self._state = new_state
        self._state_pub.publish(String(data=self._state))


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
