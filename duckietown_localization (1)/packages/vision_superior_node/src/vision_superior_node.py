#!/usr/bin/env python3
import rospy
import os
import cv2
import numpy as np
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from dt_apriltags import Detector
import pyzxing  # ZXing wrapper

class VisionSuperiorNode(DTROS):
    def __init__(self, node_name):
        super(VisionSuperiorNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.node_name = node_name
        self.veh_name = os.environ.get('VEHICLE_NAME', 'eustaquio')

        # Detectores
        self.detector = Detector()
        self.zxing_reader = pyzxing.BarCodeReader()

        # Subscripción a imagen
        img_topic = f"/{self.veh_name}/camera_node/image/compressed"
        self.sub_img = rospy.Subscriber(img_topic, CompressedImage, self.cb_img, queue_size=1)

        # Publicadores
        self.pub_apriltag = rospy.Publisher("/vision_superior/apriltags_detectados", String, queue_size=1)
        self.pub_qr = rospy.Publisher("/vision_superior/qr_detectado", String, queue_size=1)

        rospy.loginfo("✅ Nodo vision_superior_node iniciado con ZXing y AprilTags")

def cb_img(self, msg):
    try:
        np_arr = np.frombuffer(msg.data, np.uint8)
        color_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        h, w = color_img.shape[:2]

        # ROI más amplio (70% superior)
        roi = color_img[:int(h * 0.7), :]

        # CLAHE (mejor contraste)
        lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        cl = clahe.apply(l)
        lab_clahe = cv2.merge((cl, a, b))
        enhanced = cv2.cvtColor(lab_clahe, cv2.COLOR_LAB2BGR)

        # Convertir a escala de grises (sin binarizar)
        gray = cv2.cvtColor(enhanced, cv2.COLOR_BGR2GRAY)

        # === ZXing ===
        tmp_path = "/tmp/qr_frame.jpg"
        cv2.imwrite(tmp_path, gray)  # GUARDAR EN GRIS
        result = self.zxing_reader.decode(tmp_path)

        if result and isinstance(result, list) and "parsed" in result[0]:
            qr_data = result[0]['parsed']
            rospy.loginfo(f"📷 QR detectado (ZXing): {qr_data}")
            self.pub_qr.publish(String(data=qr_data))
        else:
            rospy.loginfo("🔍 No se detectó QR con ZXing")

        # === AprilTags ===
        tags = self.detector.detect(gray)
        if tags:
            tag_id = tags[0].tag_id
            rospy.loginfo(f"🏷️ AprilTag detectado: ID {tag_id}")
            self.pub_apriltag.publish(String(data=str(tag_id)))
        else:
            rospy.loginfo("🔍 No se detectaron AprilTags")

    except Exception as e:
        rospy.logwarn(f"❌ Error procesando imagen en vision_superior_node: {e}")

if __name__ == '__main__':
    node = VisionSuperiorNode(node_name='vision_superior_node')
    rospy.spin()
