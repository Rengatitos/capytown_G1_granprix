#!/usr/bin/env python3
"""Nodo opcional de camara: deteccion de senales de PARE.

Segmenta color rojo en HSV, filtra por area y relacion de aspecto, y
exige confirmacion durante varios frames consecutivos antes de avisar
una deteccion (y varios frames sin deteccion antes de retirarla) para
reducir falsos positivos por reflejos o ruido.

Publica un booleano continuo en ``/pare_detectado``: true mientras la
senal esta confirmada en el campo de vision. La logica de "detenerse
3 segundos" vive en ``state_machine_node`` (estado BUSCAR_PARE), no
aqui: este nodo solo reporta lo que ve la camara.

Si el robot no tiene camara disponible, simplemente no se lanza este
nodo (ver parametro ``usar_camara`` en state_machine y el launch
file); la navegacion por LiDAR sigue funcionando sin PARE.

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


class StopSignDetectorNode(Node):

    def __init__(self):
        super().__init__('stop_sign_detector')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/pare_detectado')
        self.declare_parameter('debug_image_topic', '/pare_detectado/debug_image')
        self.declare_parameter('publicar_debug', False)
        self.declare_parameter('rango1_min', [0, 120, 70])
        self.declare_parameter('rango1_max', [10, 255, 255])
        self.declare_parameter('rango2_min', [170, 120, 70])
        self.declare_parameter('rango2_max', [180, 255, 255])
        self.declare_parameter('area_minima_px', 800.0)
        self.declare_parameter('area_maxima_px', 60000.0)
        self.declare_parameter('relacion_aspecto_min', 0.6)
        self.declare_parameter('relacion_aspecto_max', 1.4)
        self.declare_parameter('frames_confirmacion', 3)
        self.declare_parameter('frames_perdida', 5)
        self.declare_parameter('roi_y_min_frac', 0.0)
        self.declare_parameter('roi_y_max_frac', 1.0)

        self._image_topic = self.get_parameter('image_topic').value
        self._output_topic = self.get_parameter('output_topic').value
        self._debug_topic = self.get_parameter('debug_image_topic').value
        self._publicar_debug = bool(self.get_parameter('publicar_debug').value)

        self._rango1_min = np.array(self.get_parameter('rango1_min').value, dtype=np.uint8)
        self._rango1_max = np.array(self.get_parameter('rango1_max').value, dtype=np.uint8)
        self._rango2_min = np.array(self.get_parameter('rango2_min').value, dtype=np.uint8)
        self._rango2_max = np.array(self.get_parameter('rango2_max').value, dtype=np.uint8)

        self._area_minima = float(self.get_parameter('area_minima_px').value)
        self._area_maxima = float(self.get_parameter('area_maxima_px').value)
        self._aspecto_min = float(self.get_parameter('relacion_aspecto_min').value)
        self._aspecto_max = float(self.get_parameter('relacion_aspecto_max').value)

        self._frames_confirmacion = int(self.get_parameter('frames_confirmacion').value)
        self._frames_perdida = int(self.get_parameter('frames_perdida').value)

        self._roi_y_min_frac = float(self.get_parameter('roi_y_min_frac').value)
        self._roi_y_max_frac = float(self.get_parameter('roi_y_max_frac').value)

        self._consec_detect = 0
        self._consec_lost = 0
        self._confirmado = False

        self._pub = self.create_publisher(Bool, self._output_topic, 10)
        self._debug_pub = (
            self.create_publisher(Image, self._debug_topic, 10) if self._publicar_debug else None
        )
        self.create_subscription(
            Image, self._image_topic, self._on_image, QoSPresetProfiles.SENSOR_DATA.value
        )

        self.get_logger().info(f'stop_sign_detector listo: {self._image_topic} -> {self._output_topic}')

    def _on_image(self, msg: Image) -> None:
        try:
            frame = imgmsg_to_bgr(msg)
        except (ValueError, RuntimeError) as exc:
            self.get_logger().warn(f'no se pudo convertir la imagen: {exc}')
            return

        detectado_este_frame, bbox = self._detectar_pare(frame)

        if detectado_este_frame:
            self._consec_detect += 1
            self._consec_lost = 0
        else:
            self._consec_lost += 1
            self._consec_detect = 0

        if not self._confirmado and self._consec_detect >= self._frames_confirmacion:
            self._confirmado = True
        elif self._confirmado and self._consec_lost >= self._frames_perdida:
            self._confirmado = False

        self._pub.publish(Bool(data=self._confirmado))

        if self._debug_pub is not None:
            self._publicar_debug_img(frame, bbox)

    def _detectar_pare(self, frame):
        height = frame.shape[0]
        y0 = int(height * self._roi_y_min_frac)
        y1 = int(height * self._roi_y_max_frac)
        roi = frame[y0:y1, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self._rango1_min, self._rango1_max)
        mask2 = cv2.inRange(hsv, self._rango2_min, self._rango2_max)
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        mejor_area = 0.0
        mejor_bbox = None

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self._area_minima or area > self._area_maxima:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if h == 0:
                continue
            relacion_aspecto = w / float(h)
            if not (self._aspecto_min <= relacion_aspecto <= self._aspecto_max):
                continue

            if area > mejor_area:
                mejor_area = area
                mejor_bbox = (x, y + y0, w, h)

        return (mejor_bbox is not None), mejor_bbox

    def _publicar_debug_img(self, frame, bbox) -> None:
        debug_frame = frame.copy()
        if bbox is not None:
            x, y, w, h = bbox
            color = (0, 255, 0) if self._confirmado else (0, 165, 255)
            cv2.rectangle(debug_frame, (x, y), (x + w, y + h), color, 2)
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
