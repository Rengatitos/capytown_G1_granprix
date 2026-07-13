#!/usr/bin/env python3
"""Conversion sensor_msgs/Image <-> array BGR de OpenCV sin cv_bridge.

En este robot, instanciar ``CvBridge()`` produce un segmentation fault
(desajuste entre la version con la que esta compilado cv_bridge y el
python3-opencv instalado -- confirmado aislando el import). Todo nodo
de este paquete que necesite convertir imagenes usa estas funciones en
su lugar. Compartido por camera_publisher_node, stop_sign_detector_node
y dashboard_server_node.
"""

import cv2
import numpy as np
from sensor_msgs.msg import Image


def imgmsg_to_bgr(msg: Image) -> np.ndarray:
    """Convierte sensor_msgs/Image a un array BGR de OpenCV. Soporta
    los encodings mas comunes de camaras USB/CSI."""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding == 'bgr8':
        return buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(
            msg.height, msg.width, 3
        )
    if msg.encoding == 'rgb8':
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(
            msg.height, msg.width, 3
        )
        return img[:, :, ::-1]
    if msg.encoding == 'mono8':
        img = buf.reshape(msg.height, msg.step)[:, :msg.width]
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if msg.encoding in ('bgra8', 'rgba8'):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 4].reshape(
            msg.height, msg.width, 4
        )
        if msg.encoding == 'rgba8':
            img = img[:, :, [2, 1, 0, 3]]
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    raise ValueError(f'encoding de imagen no soportado: {msg.encoding!r}')


def bgr_to_imgmsg(frame: np.ndarray) -> Image:
    """Construye un sensor_msgs/Image (bgr8) a partir de un array BGR
    de OpenCV."""
    msg = Image()
    msg.height, msg.width = frame.shape[0], frame.shape[1]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(frame).tobytes()
    return msg
