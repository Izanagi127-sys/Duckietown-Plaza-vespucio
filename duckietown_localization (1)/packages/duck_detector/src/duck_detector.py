#!/usr/bin/env python3
import cv2
import os 
import rospy
import numpy as np
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

class duckDetector(DTROS):
    def __init__(self, node_name):
        super(duckDetector, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
        self.node_name = node_name
        self.rgb_img = None
        self.run = True

        # Subscripción a imagen superior de vision_superior_node
        self.img_sub = rospy.Subscriber("/vision_superior/image_top/compressed", CompressedImage, self.cb_img, queue_size=1)

        # Publicador de imagen procesada
        self.pub = rospy.Publisher("/" + os.environ['VEHICLE_NAME'] + '/duckdetector/compressed', CompressedImage, queue_size=1)

        rospy.loginfo("✅ duckDetector conectado a vision_superior_node")

    def cb_img(self, msg):
        self.pub_img = rospy.Publisher("/vision_superior/image_top/compressed", CompressedImage, queue_size=1)
        msg_img = CompressedImage()
        msg_img.header.stamp = rospy.Time.now()
        msg_img.format = "jpeg"
        msg_img.data = cv2.imencode('.jpg', top_half)[1].tobytes()
        self.pub_img.publish(msg_img)

        try:
            arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            self.rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        except Exception as e:
            rospy.logwarn(f"[duckDetector] Error procesando imagen: {e}")

    def duckdetector(self):
        if self.rgb_img is None:
            return
        
        hsv = cv2.cvtColor(self.rgb_img, cv2.COLOR_RGB2HSV)
        hsv_lower = np.array([0, 83, 86])
        hsv_upper = np.array([12, 132, 223])

        mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
        kernel = np.ones((5, 5), np.uint8)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        filtered = [cnt for cnt in contours if cv2.contourArea(cnt) > 200]
        num_duckies = len(filtered)

        if num_duckies == 0:
            return

        rospy.loginfo(f"🦆 Patitos detectados: {num_duckies}")

        # Publicar imagen con detección
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', self.rgb_img)[1]).tobytes()
        self.pub.publish(msg)

if __name__ == '__main__':
    node = duckDetector(node_name='duckDetector')
    rate = rospy.Rate(2)
    while not rospy.is_shutdown() and node.run:
        node.duckdetector()
        rate.sleep()
