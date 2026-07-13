#!/usr/bin/env python3
"""Driver minimo de camara: abre el dispositivo de video local y
publica frames en ``sensor_msgs/Image`` (bgr8) a un ritmo fijo.

Necesario porque el script de demo de Yahboom para esta camara
(``yahboomcar_astra/colorHSV.py``) no sirve como driver de fondo: usa
``cv_bridge`` (que hace segmentation fault en este robot, ver
``image_codec.py``) y ademas requiere una ventana grafica
(``cv.imshow``/``cv.waitKey``), lo que lo hace inutilizable por SSH
sin servidor X. Este nodo solo abre la camara y publica -- sin
cv_bridge, sin GUI -- para que la imagen este siempre disponible
(``stop_sign_detector_node``, ``dashboard_server_node``) sin depender
de lanzar aparte ningun script de demo.

Publica en ``image_topic`` (por defecto ``/camera/image_raw``, el
mismo topic que ya esperan el resto de nodos de este paquete).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image

import cv2

from capytown_granprix.image_codec import bgr_to_imgmsg


class CameraPublisherNode(Node):

    def __init__(self):
        super().__init__('camera_publisher')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15.0)

        self._image_topic = str(self.get_parameter('image_topic').value)
        camera_index = int(self.get_parameter('camera_index').value)
        self._width = int(self.get_parameter('width').value)
        self._height = int(self.get_parameter('height').value)
        fps = float(self.get_parameter('fps').value)

        self._pub = self.create_publisher(
            Image, self._image_topic, QoSPresetProfiles.SENSOR_DATA.value
        )

        self._capture = cv2.VideoCapture(camera_index)
        if not self._capture.isOpened():
            self.get_logger().error(
                f'no se pudo abrir la camara (indice {camera_index}) -- '
                f'{self._image_topic} no publicara nada.'
            )
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)

        self.create_timer(1.0 / fps, self._on_timer)
        self.get_logger().info(
            f'camera_publisher listo: indice={camera_index} -> {self._image_topic} '
            f'({self._width}x{self._height} @ {fps} fps)'
        )

    def _on_timer(self) -> None:
        ok, frame = self._capture.read()
        if not ok or frame is None:
            return
        if frame.shape[1] != self._width or frame.shape[0] != self._height:
            frame = cv2.resize(frame, (self._width, self._height))
        self._pub.publish(bgr_to_imgmsg(frame))

    def destroy_node(self) -> None:
        self._capture.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
