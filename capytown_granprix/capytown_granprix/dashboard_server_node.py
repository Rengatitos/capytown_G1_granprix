#!/usr/bin/env python3
"""Nodo de diagnostico: sirve una pagina web local con el mapa del
laberinto (lo que el robot ya descubrio), la celda/heading actual, la
imagen de la camara en vivo, Y lo que ve el LiDAR en vivo (equivalente
en el navegador a ``lidar_viz.py``, pero sin necesitar VNC/X11) -- sin
RViz, sin rosbridge.

No forma parte de la corrida de competencia: es una herramienta de
depuracion aparte, se lanza en su propia terminal junto al resto del
paquete. La pagina usa sondeo (polling, fetch cada medio segundo) para
actualizarse, servido todo desde un http.server plano -- funciona con
un navegador comun apuntando a ``http://<ip-de-la-Pi>:<puerto>/``.

Uso:
    ros2 run capytown_granprix dashboard_server_node
    ros2 run capytown_granprix dashboard_server_node --ros-args -p puerto:=8000

NO usa ``cv_bridge``: en este robot, instanciar ``CvBridge()`` produce
un segmentation fault (desajuste entre la version con la que esta
compilado cv_bridge y el python3-opencv instalado -- confirmado
aislando el import, ver stop_sign_detector_node.py que tiene la misma
correccion). La conversion de imagen se hace a mano con numpy
(``image_codec.py``).
"""

import json
import math
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

from capytown_granprix.image_codec import imgmsg_to_bgr

_PAGINA_HTML = """<!doctype html>
<html lang="es">
<head>
<meta charset="utf-8">
<title>CapyTown Gran Prix - Dashboard</title>
<style>
  body { font-family: system-ui, sans-serif; background: #1b1e24; color: #e8e8e8; margin: 0; padding: 16px; }
  h1 { font-size: 1.1rem; margin: 0 0 12px; color: #9fd3ff; }
  .fila { display: flex; gap: 24px; flex-wrap: wrap; align-items: flex-start; }
  .panel { background: #262a33; border-radius: 8px; padding: 12px; }
  #estado { font-size: 1.05rem; margin-bottom: 8px; }
  #estado b { color: #7be08a; }
  canvas { background: #10131a; border-radius: 4px; display: block; }
  img#camara { max-width: 420px; border-radius: 4px; display: block; background: #10131a; }
  .aviso { color: #f0a860; font-size: 0.9rem; }
</style>
</head>
<body>
<h1>CapyTown Gran Prix &mdash; Dashboard en vivo</h1>
<div id="estado">Conectando...</div>
<div class="fila">
  <div class="panel">
    <canvas id="mapa" width="480" height="320"></canvas>
  </div>
  <div class="panel">
    <img id="camara" alt="camara" />
    <div class="aviso" id="avisoCamara"></div>
  </div>
  <div class="panel">
    <canvas id="lidar" width="420" height="420"></canvas>
    <div class="aviso" id="avisoLidar"></div>
  </div>
</div>
<script>
const COLS = 6, ROWS = 4, CELL = 80;
const canvas = document.getElementById('mapa');
const ctx = canvas.getContext('2d');
const HEAD_DELTA = { NORTE: [0, -1], ESTE: [1, 0], SUR: [0, 1], OESTE: [-1, 0] };

const lidarCanvas = document.getElementById('lidar');
const lidarCtx = lidarCanvas.getContext('2d');
const LIDAR_RANGO_M = 2.5;  // debe coincidir con lidar_max_range_m del nodo
const LIDAR_ESCALA = (lidarCanvas.width / 2 - 10) / LIDAR_RANGO_M;  // px por metro

function dibujarLidar(puntos) {
  const cx = lidarCanvas.width / 2, cy = lidarCanvas.height / 2;
  lidarCtx.clearRect(0, 0, lidarCanvas.width, lidarCanvas.height);

  // circulos de referencia (cada 0.5 m)
  lidarCtx.strokeStyle = '#2a2f3a';
  lidarCtx.fillStyle = '#4d5568';
  lidarCtx.font = '10px system-ui';
  for (let r = 0.5; r <= LIDAR_RANGO_M; r += 0.5) {
    lidarCtx.beginPath();
    lidarCtx.arc(cx, cy, r * LIDAR_ESCALA, 0, Math.PI * 2);
    lidarCtx.stroke();
    lidarCtx.fillText(r.toFixed(1) + 'm', cx + 4, cy - r * LIDAR_ESCALA);
  }

  // robot (centro, frente hacia arriba)
  lidarCtx.fillStyle = '#dfe3ea';
  lidarCtx.fillRect(cx - 8, cy - 12, 16, 24);
  lidarCtx.fillStyle = '#9fd3ff';
  lidarCtx.font = 'bold 11px system-ui';
  lidarCtx.fillText('FRENTE', cx - 20, cy - 16);

  // puntos del scan: x=adelante, y=izquierda (marco del robot) ->
  // pantalla: adelante = arriba (-y en canvas), izquierda = izquierda (-x en canvas)
  lidarCtx.fillStyle = '#ff5a5a';
  puntos.forEach(([x, y]) => {
    const px = cx - y * LIDAR_ESCALA;
    const py = cy - x * LIDAR_ESCALA;
    lidarCtx.fillRect(px - 1, py - 1, 2, 2);
  });
}

function celdaAColRow(cell) {
  const col = cell.charCodeAt(0) - 65;
  const row = parseInt(cell.slice(1), 10) - 1;
  return [col, row];
}

function dibujarMapa(estado) {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // grilla
  ctx.strokeStyle = '#3a4150';
  ctx.lineWidth = 1;
  for (let c = 0; c <= COLS; c++) {
    ctx.beginPath();
    ctx.moveTo(c * CELL, 0);
    ctx.lineTo(c * CELL, ROWS * CELL);
    ctx.stroke();
  }
  for (let r = 0; r <= ROWS; r++) {
    ctx.beginPath();
    ctx.moveTo(0, r * CELL);
    ctx.lineTo(COLS * CELL, r * CELL);
    ctx.stroke();
  }

  // etiquetas de celda
  ctx.fillStyle = '#4d5568';
  ctx.font = '11px system-ui';
  for (let c = 0; c < COLS; c++) {
    for (let r = 0; r < ROWS; r++) {
      const nombre = String.fromCharCode(65 + c) + (r + 1);
      ctx.fillText(nombre, c * CELL + 4, r * CELL + 14);
    }
  }

  // conexiones ya descubiertas (mapa.json guardado por state_machine)
  ctx.strokeStyle = '#3f7d4a';
  ctx.lineWidth = 4;
  ctx.lineCap = 'round';
  (estado.mapa || []).forEach(([[c1, r1], [c2, r2]]) => {
    const x1 = c1 * CELL + CELL / 2, y1 = r1 * CELL + CELL / 2;
    const x2 = c2 * CELL + CELL / 2, y2 = r2 * CELL + CELL / 2;
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();
  });

  // INICIO / META
  ctx.fillStyle = 'rgba(120,180,255,0.25)';
  ctx.fillRect(0, 3 * CELL, CELL, CELL);
  ctx.fillStyle = 'rgba(120,255,150,0.25)';
  ctx.fillRect(5 * CELL, 0, CELL, CELL);

  // robot actual
  if (estado.cell) {
    const [col, row] = celdaAColRow(estado.cell);
    const cx = col * CELL + CELL / 2, cy = row * CELL + CELL / 2;
    ctx.fillStyle = '#ff5a5a';
    ctx.beginPath();
    ctx.arc(cx, cy, 10, 0, Math.PI * 2);
    ctx.fill();

    const delta = HEAD_DELTA[estado.heading];
    if (delta) {
      ctx.strokeStyle = '#ff5a5a';
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(cx + delta[0] * 22, cy + delta[1] * 22);
      ctx.stroke();
    }
  }
}

async function actualizar() {
  try {
    const resp = await fetch('/estado.json', { cache: 'no-store' });
    const estado = await resp.json();
    document.getElementById('estado').innerHTML =
      `Celda: <b>${estado.cell || '?'}</b> &nbsp; Heading: <b>${estado.heading || '?'}</b> `
      + `&nbsp; Estado FSM: <b>${estado.robot_state || '?'}</b>`;
    dibujarMapa(estado);
  } catch (e) {
    document.getElementById('estado').textContent = 'Sin conexion al robot...';
  }
  document.getElementById('camara').src = '/camara.jpg?t=' + Date.now();

  try {
    const respLidar = await fetch('/lidar.json', { cache: 'no-store' });
    const datosLidar = await respLidar.json();
    if (datosLidar.puntos && datosLidar.puntos.length) {
      dibujarLidar(datosLidar.puntos);
      document.getElementById('avisoLidar').textContent =
        `${datosLidar.puntos.length} puntos`;
    } else {
      document.getElementById('avisoLidar').textContent = 'Sin datos de /scan todavia...';
    }
  } catch (e) {
    document.getElementById('avisoLidar').textContent = 'Sin conexion al LiDAR...';
  }
}

document.getElementById('camara').addEventListener('error', () => {
  document.getElementById('avisoCamara').textContent =
    'Sin imagen de camara (usar_camara:=false o aun no llega el primer frame).';
});
document.getElementById('camara').addEventListener('load', () => {
  document.getElementById('avisoCamara').textContent = '';
});

setInterval(actualizar, 500);
actualizar();
</script>
</body>
</html>
"""


class DashboardServerNode(Node):

    def __init__(self):
        super().__init__('dashboard_server')

        self.declare_parameter('puerto', 8000)
        self.declare_parameter('cell_topic', '/robot_cell')
        self.declare_parameter('robot_state_topic', '/robot_state')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('mapa_path', '~/capytown_resultados/mapa_granprix.json')
        # Mismos valores que lidar_processor (granprix_params.yaml) para
        # que el sector "FRENTE" del dashboard coincida con lo que usa
        # el robot de verdad -- si se recalibra el LiDAR, actualizar
        # aca tambien.
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('front_offset_deg', 0.0)
        self.declare_parameter('invert_left_right', False)
        self.declare_parameter('lidar_max_range_m', 2.5)
        self.declare_parameter('lidar_max_puntos', 500)

        self._puerto = int(self.get_parameter('puerto').value)
        self._mapa_path = os.path.expanduser(str(self.get_parameter('mapa_path').value))
        self._front_offset_rad = math.radians(float(self.get_parameter('front_offset_deg').value))
        self._sign = -1 if bool(self.get_parameter('invert_left_right').value) else 1
        self._lidar_max_range = float(self.get_parameter('lidar_max_range_m').value)
        self._lidar_max_puntos = int(self.get_parameter('lidar_max_puntos').value)

        self._lock = threading.Lock()
        self._cell = None
        self._heading = None
        self._robot_state = None
        self._ultimo_jpg = None
        self._ultimos_puntos_lidar = []

        self.create_subscription(
            String, self.get_parameter('cell_topic').value, self._on_cell, 10
        )
        self.create_subscription(
            String, self.get_parameter('robot_state_topic').value, self._on_robot_state, 10
        )
        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self._on_image,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._on_scan,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        self._iniciar_servidor()
        self.get_logger().info(f'dashboard listo en http://0.0.0.0:{self._puerto}/')

    def _on_cell(self, msg: String) -> None:
        cell, _, heading = msg.data.partition('|')
        with self._lock:
            self._cell = cell
            self._heading = heading

    def _on_robot_state(self, msg: String) -> None:
        with self._lock:
            self._robot_state = msg.data

    def _on_image(self, msg: Image) -> None:
        try:
            frame = imgmsg_to_bgr(msg)
        except ValueError:
            return
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            with self._lock:
                self._ultimo_jpg = buf.tobytes()

    def _on_scan(self, msg: LaserScan) -> None:
        """Convierte el scan crudo a puntos (x, y) en el marco del robot
        (x=adelante, y=izquierda), aplicando la misma calibracion de
        montaje (front_offset_deg/invert_left_right) que
        lidar_processor_node -- para que se vea igual que lo que usa el
        robot de verdad, no el marco crudo del sensor."""
        ranges = np.asarray(msg.ranges, dtype=float)
        idx = np.arange(len(ranges), dtype=float)
        a = msg.angle_min + idx * msg.angle_increment
        a = np.mod(a + math.pi, 2 * math.pi) - math.pi
        robot_angles = self._sign * (a - self._front_offset_rad)
        robot_angles = np.mod(robot_angles + math.pi, 2 * math.pi) - math.pi

        range_max = min(msg.range_max, self._lidar_max_range)
        mask = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= range_max)
        r = ranges[mask]
        ang = robot_angles[mask]
        x = r * np.cos(ang)
        y = r * np.sin(ang)

        puntos = list(zip(x.tolist(), y.tolist()))
        if len(puntos) > self._lidar_max_puntos:
            paso = len(puntos) // self._lidar_max_puntos
            puntos = puntos[::paso]

        with self._lock:
            self._ultimos_puntos_lidar = puntos

    def _leer_mapa(self):
        try:
            with open(self._mapa_path, encoding='utf-8') as f:
                return json.load(f)
        except (OSError, ValueError):
            return []

    def _iniciar_servidor(self) -> None:
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):  # noqa: A002 - silencia el log de acceso
                pass

            def do_GET(self):
                if self.path in ('/', '') or self.path.startswith('/?'):
                    self._responder(200, 'text/html; charset=utf-8', _PAGINA_HTML.encode('utf-8'))
                elif self.path.startswith('/estado.json'):
                    with node._lock:
                        data = {
                            'cell': node._cell,
                            'heading': node._heading,
                            'robot_state': node._robot_state,
                        }
                    data['mapa'] = node._leer_mapa()
                    self._responder(
                        200, 'application/json', json.dumps(data).encode('utf-8'), sin_cache=True
                    )
                elif self.path.startswith('/camara.jpg'):
                    with node._lock:
                        jpg = node._ultimo_jpg
                    if jpg is None:
                        self.send_response(204)
                        self.end_headers()
                        return
                    self._responder(200, 'image/jpeg', jpg, sin_cache=True)
                elif self.path.startswith('/lidar.json'):
                    with node._lock:
                        puntos = node._ultimos_puntos_lidar
                    body = json.dumps({'puntos': puntos}).encode('utf-8')
                    self._responder(200, 'application/json', body, sin_cache=True)
                else:
                    self.send_response(404)
                    self.end_headers()

            def _responder(self, codigo, tipo, cuerpo, sin_cache=False):
                self.send_response(codigo)
                self.send_header('Content-Type', tipo)
                self.send_header('Content-Length', str(len(cuerpo)))
                if sin_cache:
                    self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(cuerpo)

        servidor = ThreadingHTTPServer(('0.0.0.0', self._puerto), Handler)
        hilo = threading.Thread(target=servidor.serve_forever, daemon=True)
        hilo.start()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
