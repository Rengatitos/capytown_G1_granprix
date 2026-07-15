#!/usr/bin/env python3
"""Nodo opcional de camara: deteccion de senales de PARE y META.

Segmenta color en HSV (rojo para PARE, verde para META), filtra por
area, relacion de aspecto y SOLIDEZ (area del blob / area de su
convex hull -- descarta ruido disperso o reflejos alargados que no
tienen forma de cartel), y exige confirmacion durante varios frames
consecutivos antes de avisar una deteccion (y varios frames sin
deteccion antes de retirarla) para reducir falsos positivos. Adaptado
de una rama mas madura del mismo proyecto (mismo autor/mantenedor).

El PARE ademas exige que su centro caiga en la BANDA CENTRAL
horizontal del frame (banda_central_frac): un cartel visto de refilon
a un costado (que el robot no esta mirando de frente) no cuenta -- la
META no exige esto, porque puede aparecer mas al costado segun por
donde entra el robot al cartel final.

Publica DOS booleanos continuos:
    ``/pare_detectado``  -- true mientras el cartel PARE (rojo) esta
                             confirmado en el campo de vision. La
                             logica de "detenerse 3 segundos" vive en
                             ``state_machine_node`` (estado
                             BUSCAR_PARE), no aqui.
    ``/meta_detectado``  -- true mientras el cartel META (verde) esta
                             confirmado. OPCIONAL: nuestro reto define
                             META como llegar a la celda F1 por
                             dead-reckoning (ver VERIFICAR_META en
                             state_machine_node.py), no un cartel
                             fisico -- este topico solo tiene efecto
                             si el layout real de la pista SI tiene un
                             cartel META verde y se activa el uso
                             correspondiente en state_machine_node.

Si el robot no tiene camara disponible, simplemente no se lanza este
nodo (ver parametro ``usar_camara`` en state_machine y el launch
file); la navegacion por LiDAR sigue funcionando sin PARE/META.

NO usa ``cv_bridge``: en este robot, instanciar ``CvBridge()`` produce
un segmentation fault (desajuste entre la version con la que esta
compilado cv_bridge y el python3-opencv instalado -- confirmado
aislando el import). La conversion sensor_msgs/Image <-> array de
OpenCV se hace a mano con numpy en su lugar (``image_codec.py``),
evitando cv_bridge por completo.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from capytown_granprix.image_codec import bgr_to_imgmsg, imgmsg_to_bgr


class _DetectorColor:
    """Hysteresis + deteccion de blob de un color (PARE o META).

    Cada instancia lleva su propio contador de confirmacion/perdida,
    independiente del otro color -- PARE y META pueden estar
    confirmados (o no) al mismo tiempo sin interferirse.
    """

    def __init__(self, rango1_min, rango1_max, rango2_min, rango2_max,
                 area_min, area_max, aspecto_min, aspecto_max, solidez_min,
                 frames_confirmacion, frames_perdida,
                 exigir_centro=False, banda_central_frac=0.20):
        self.rango1_min = rango1_min
        self.rango1_max = rango1_max
        self.rango2_min = rango2_min  # None si el color no cruza el 0 del hue
        self.rango2_max = rango2_max
        self.area_min = area_min
        self.area_max = area_max
        self.aspecto_min = aspecto_min
        self.aspecto_max = aspecto_max
        self.solidez_min = solidez_min
        self.frames_confirmacion = frames_confirmacion
        self.frames_perdida = frames_perdida
        self.exigir_centro = exigir_centro
        self.banda_central_frac = banda_central_frac

        self.consec_detect = 0
        self.consec_lost = 0
        self.confirmado = False

    def _mascara(self, hsv):
        mask = cv2.inRange(hsv, self.rango1_min, self.rango1_max)
        if self.rango2_min is not None:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, self.rango2_min, self.rango2_max))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _quitar_componentes_pequenos(self, mask):
        """Descarta de entrada las regiones conectadas mas chicas que
        area_min -- evita que motas de ruido dispersas por el frame
        lleguen siquiera a formar un contorno a evaluar."""
        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        limpia = np.zeros_like(mask)
        for etiqueta in range(1, n):
            if stats[etiqueta, cv2.CC_STAT_AREA] >= self.area_min:
                limpia[labels == etiqueta] = 255
        return limpia

    def _mejor_blob(self, mask, y_offset, ancho_frame):
        cx_frame = ancho_frame / 2.0
        tol_px = self.banda_central_frac * ancho_frame
        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        mejor = None
        for cnt in contornos:
            area = cv2.contourArea(cnt)
            if area < self.area_min or area > self.area_max:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            if h == 0:
                continue
            if self.exigir_centro and abs((x + w / 2.0) - cx_frame) > tol_px:
                continue

            aspecto = w / float(h)
            if not (self.aspecto_min <= aspecto <= self.aspecto_max):
                continue

            hull = cv2.convexHull(cnt)
            area_hull = cv2.contourArea(hull)
            if area_hull < 1e-3 or area / area_hull < self.solidez_min:
                continue

            if mejor is None or area > mejor[0]:
                mejor = (area, (x, y + y_offset, w, h))

        return mejor  # None o (area, bbox)

    def procesar(self, hsv, y_offset, ancho_frame):
        mask = self._mascara(hsv)
        mask = self._quitar_componentes_pequenos(mask)
        mejor = self._mejor_blob(mask, y_offset, ancho_frame)

        if mejor is not None:
            self.consec_detect += 1
            self.consec_lost = 0
        else:
            self.consec_lost += 1
            self.consec_detect = 0

        if not self.confirmado and self.consec_detect >= self.frames_confirmacion:
            self.confirmado = True
        elif self.confirmado and self.consec_lost >= self.frames_perdida:
            self.confirmado = False

        bbox = mejor[1] if mejor is not None else None
        return self.confirmado, bbox


class StopSignDetectorNode(Node):

    def __init__(self):
        super().__init__('stop_sign_detector')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/pare_detectado')
        self.declare_parameter('meta_output_topic', '/meta_detectado')
        self.declare_parameter('debug_image_topic', '/pare_detectado/debug_image')
        self.declare_parameter('publicar_debug', False)

        # --- Rojo (PARE): el hue del rojo cruza 0/179, dos rangos ---
        self.declare_parameter('rango1_min', [0, 120, 70])
        self.declare_parameter('rango1_max', [10, 255, 255])
        self.declare_parameter('rango2_min', [170, 120, 70])
        self.declare_parameter('rango2_max', [180, 255, 255])
        self.declare_parameter('area_minima_px', 800.0)
        self.declare_parameter('area_maxima_px', 60000.0)
        self.declare_parameter('relacion_aspecto_min', 0.6)
        self.declare_parameter('relacion_aspecto_max', 1.4)
        self.declare_parameter('solidez_minima', 0.75)
        self.declare_parameter('banda_central_frac', 0.20)
        self.declare_parameter('frames_confirmacion', 3)
        self.declare_parameter('frames_perdida', 5)

        # --- Verde (META, opcional): un solo rango de hue, no cruza el 0 ---
        self.declare_parameter('usar_meta_camara', False)
        self.declare_parameter('meta_rango_min', [35, 40, 60])
        self.declare_parameter('meta_rango_max', [95, 255, 255])
        self.declare_parameter('meta_area_minima_px', 600.0)
        self.declare_parameter('meta_area_maxima_px', 150000.0)
        self.declare_parameter('meta_aspecto_min', 0.3)
        self.declare_parameter('meta_aspecto_max', 3.0)
        self.declare_parameter('meta_solidez_minima', 0.35)
        self.declare_parameter('meta_frames_confirmacion', 3)
        self.declare_parameter('meta_frames_perdida', 5)

        self.declare_parameter('roi_y_min_frac', 0.0)
        self.declare_parameter('roi_y_max_frac', 1.0)

        gp = lambda n: self.get_parameter(n).value  # noqa: E731

        self._image_topic = gp('image_topic')
        self._output_topic = gp('output_topic')
        self._meta_output_topic = gp('meta_output_topic')
        self._debug_topic = gp('debug_image_topic')
        self._publicar_debug = bool(gp('publicar_debug'))
        self._usar_meta_camara = bool(gp('usar_meta_camara'))
        self._roi_y_min_frac = float(gp('roi_y_min_frac'))
        self._roi_y_max_frac = float(gp('roi_y_max_frac'))

        self._pare = _DetectorColor(
            rango1_min=np.array(gp('rango1_min'), dtype=np.uint8),
            rango1_max=np.array(gp('rango1_max'), dtype=np.uint8),
            rango2_min=np.array(gp('rango2_min'), dtype=np.uint8),
            rango2_max=np.array(gp('rango2_max'), dtype=np.uint8),
            area_min=float(gp('area_minima_px')),
            area_max=float(gp('area_maxima_px')),
            aspecto_min=float(gp('relacion_aspecto_min')),
            aspecto_max=float(gp('relacion_aspecto_max')),
            solidez_min=float(gp('solidez_minima')),
            frames_confirmacion=int(gp('frames_confirmacion')),
            frames_perdida=int(gp('frames_perdida')),
            exigir_centro=True,
            banda_central_frac=float(gp('banda_central_frac')),
        )
        # El detector de META se instancia siempre (no cuesta nada en
        # reposo), pero solo se procesa cada frame si usar_meta_camara
        # es true -- ver _on_image.
        self._meta = _DetectorColor(
            rango1_min=np.array(gp('meta_rango_min'), dtype=np.uint8),
            rango1_max=np.array(gp('meta_rango_max'), dtype=np.uint8),
            rango2_min=None,
            rango2_max=None,
            area_min=float(gp('meta_area_minima_px')),
            area_max=float(gp('meta_area_maxima_px')),
            aspecto_min=float(gp('meta_aspecto_min')),
            aspecto_max=float(gp('meta_aspecto_max')),
            solidez_min=float(gp('meta_solidez_minima')),
            frames_confirmacion=int(gp('meta_frames_confirmacion')),
            frames_perdida=int(gp('meta_frames_perdida')),
            exigir_centro=False,
        )

        self._pub_pare = self.create_publisher(Bool, self._output_topic, 10)
        self._pub_meta = (
            self.create_publisher(Bool, self._meta_output_topic, 10)
            if self._usar_meta_camara else None
        )
        self._debug_pub = (
            self.create_publisher(Image, self._debug_topic, 10) if self._publicar_debug else None
        )
        self.create_subscription(
            Image, self._image_topic, self._on_image, QoSPresetProfiles.SENSOR_DATA.value
        )

        self.get_logger().info(
            f'stop_sign_detector listo: {self._image_topic} -> {self._output_topic} (PARE)'
            + (f' + {self._meta_output_topic} (META)' if self._usar_meta_camara else '')
        )

    def _on_image(self, msg: Image) -> None:
        try:
            frame = imgmsg_to_bgr(msg)
        except (ValueError, RuntimeError) as exc:
            self.get_logger().warn(f'no se pudo convertir la imagen: {exc}')
            return

        height, width = frame.shape[:2]
        y0 = int(height * self._roi_y_min_frac)
        y1 = int(height * self._roi_y_max_frac)
        roi = frame[y0:y1, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        conf_pare, bbox_pare = self._pare.procesar(hsv, y0, width)
        self._pub_pare.publish(Bool(data=conf_pare))

        conf_meta, bbox_meta = (False, None)
        if self._usar_meta_camara:
            conf_meta, bbox_meta = self._meta.procesar(hsv, y0, width)
            self._pub_meta.publish(Bool(data=conf_meta))

        if self._debug_pub is not None:
            self._publicar_debug_img(frame, bbox_pare, conf_pare, bbox_meta, conf_meta)

    def _publicar_debug_img(self, frame, bbox_pare, conf_pare, bbox_meta, conf_meta) -> None:
        debug_frame = frame.copy()
        for bbox, confirmado, color_pendiente, etiqueta in (
            (bbox_pare, conf_pare, (0, 165, 255), 'PARE'),
            (bbox_meta, conf_meta, (0, 165, 255), 'META'),
        ):
            if bbox is None:
                continue
            x, y, w, h = bbox
            color = (0, 255, 0) if confirmado else color_pendiente
            cv2.rectangle(debug_frame, (x, y), (x + w, y + h), color, 2)
            cv2.putText(debug_frame, etiqueta, (x, max(15, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        self._debug_pub.publish(bgr_to_imgmsg(debug_frame))


def main(args=None):
    rclpy.init(args=args)
    node = StopSignDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
