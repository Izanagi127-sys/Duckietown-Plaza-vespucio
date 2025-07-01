#!/usr/bin/env python3
import cv2
import os
import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import Twist2DStamped
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from duckietown.dtros import DTROS, NodeType

class lane_follow_node(DTROS):
    def __init__(self, node_name):
        super(lane_follow_node, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
        self.node_name = node_name
        self.pub_img = None
        self.run = True

        # LAB color thresholds (calibrar según condiciones reales)
        self.yellow_lower = np.array([190, 120, 140])
        self.yellow_upper = np.array([255, 150, 180])
        self.red_lower = np.array([50, 160, 150])
        self.red_upper = np.array([255, 200, 200])
        self.blue_lower = np.array([20, 140, 130])
        self.blue_upper = np.array([255, 170, 170])

        self.speed = 0.15
        self.omega = 0
        self.size_ratio = 0.8
        self.PID = [1.5, 0, 0]

        self.estado = "inicio_recto"
        self.start_time = rospy.Time.now()
        self.pose_inicial = None
        self.pose_actual = None
        self.retorno_activado = False
        self.retorno_completado = False
        self.tiempo_stop = None
        self.ignorar_rojo_hasta = rospy.Time.now()

        rospy.Subscriber("/qr_reader/retorno_pedido", Bool, self.qr_callback)
        rospy.Subscriber("/fused_localization_node/transform", TransformStamped, self.pose_callback)

        img_topic = f"/{os.environ['VEHICLE_NAME']}/camera_node/image/compressed"
        self.img_sub = rospy.Subscriber(img_topic, CompressedImage, self.cb_img, queue_size=1)

        self.img_publisher = rospy.Publisher('/masked_image/compressed', CompressedImage)
        twist_topic = f"/{os.environ['VEHICLE_NAME']}/car_cmd_switch_node/cmd"
        self.twist_publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        self.pub_retorno_completado = rospy.Publisher("/lane_follow/retorno_completado", Bool, queue_size=1)

        rospy.on_shutdown(self.on_shutdown)

    def cb_img(self, msg):
        data_arr = np.frombuffer(msg.data, np.uint8)
        col_img = cv2.imdecode(data_arr, cv2.IMREAD_COLOR)
        h, w = col_img.shape[:2]
        roi = col_img[int(h * 0.7):, :]

        # Enmascarar lado derecho (reflejo)
        mask_ignore = np.zeros(roi.shape[:2], dtype=np.uint8)
        mask_ignore[:, -80:] = 255
        lab = cv2.cvtColor(roi, cv2.COLOR_BGR2Lab)

        yellow_mask = cv2.inRange(lab, self.yellow_lower, self.yellow_upper)
        yellow_mask = cv2.bitwise_and(yellow_mask, cv2.bitwise_not(mask_ignore))
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        red_mask = cv2.inRange(lab, self.red_lower, self.red_upper)
        red_mask = cv2.bitwise_and(red_mask, cv2.bitwise_not(mask_ignore))
        red_detected = cv2.countNonZero(red_mask) > 800

        if red_detected and self.estado != "pausa_temporal" and rospy.Time.now() > self.ignorar_rojo_hasta:
            rospy.loginfo("🟥 Línea roja detectada. Pausando.")
            self.estado = "pausa_temporal"
            self.tiempo_stop = rospy.Time.now()
            self.ignorar_rojo_hasta = rospy.Time.now() + rospy.Duration(4)

        blue_mask = cv2.inRange(lab, self.blue_lower, self.blue_upper)
        blue_detected = cv2.countNonZero(blue_mask) > 800 and self.retorno_activado and not self.retorno_completado

        if blue_detected and self.pose_inicial is not None and self.pose_actual is not None:
            dist = np.linalg.norm(self.pose_actual - self.pose_inicial)
            if dist < 0.15:
                rospy.loginfo("🔵 Punto de retorno alcanzado. Detenido.")
                self.estado = "detenido"
                self.retorno_activado = False
                self.retorno_completado = True
                self.pub_retorno_completado.publish(Bool(data=True))

        if len(contours) == 0:
            self.pub_img = roi
            return

        largest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest)
        conts = [largest]

        image = cv2.drawContours(roi.copy(), conts, -1, (0, 255, 0), 3)
        image = cv2.line(image, (x, y + h // 2), (x + int((self.size_ratio * (y + h))), y + h), (0, 255, 0), 2)

        for i in range(len(image)):
            image[i][len(image[i]) // 2] = [255, 0, 0]

        image = cv2.bitwise_or(image, cv2.cvtColor(red_mask, cv2.COLOR_GRAY2BGR))
        image = cv2.bitwise_or(image, cv2.cvtColor(blue_mask, cv2.COLOR_GRAY2BGR))

        self.pub_img = image

        if rospy.Time.now() - self.start_time > rospy.Duration(6) and self.estado == "inicio_recto":
            self.estado = "autonomo"
            rospy.loginfo("🚗 Transición a modo autónomo.")

        if self.estado in ["autonomo", "retorno"]:
            self.pid(x, y + h // 2, len(image[0]) // 2)

    def img_pub(self):
        if self.pub_img is not None:
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', self.pub_img)[1]).tobytes()
            self.img_publisher.publish(msg)

    def twist_pub(self):
        msg = Twist2DStamped()
        if self.estado == "pausa_temporal":
            if rospy.Time.now() - self.tiempo_stop < rospy.Duration(2):
                msg.v = 0.0
                msg.omega = 0.0
            else:
                rospy.loginfo("🟢 Reanudando tras pausa.")
                self.estado = "autonomo"
                msg.v = self.speed
                msg.omega = self.omega
        elif self.estado == "detenido":
            msg.v = 0.0
            msg.omega = 0.0
        elif self.estado == "inicio_recto":
            msg.v = self.speed
            msg.omega = 0.0
        else:
            msg.v = self.speed
            msg.omega = self.omega
        self.twist_publisher.publish(msg)

    def pid(self, x, y, goal):
        scale = 0.02
        diff = ((x + int((self.size_ratio * y))) - goal) * scale
        self.omega = -self.PID[0] * diff
        rospy.loginfo(f"[{self.estado.upper()}] Diff: {diff:.3f} | Omega: {self.omega:.3f}")

    def qr_callback(self, msg):
        if msg.data and not self.retorno_completado:
            rospy.loginfo("🔁 Señal de retorno activada.")
            self.retorno_activado = True
            self.estado = "retorno"
            if self.pose_actual is not None:
                self.pose_inicial = self.pose_actual.copy()
                rospy.loginfo(f"📍 Pose inicial registrada: {self.pose_inicial}")

    def pose_callback(self, msg):
        pos = msg.transform.translation
        self.pose_actual = np.array([pos.x, pos.y])

    def on_shutdown(self):
        rospy.loginfo("🛑 Nodo apagado. Deteniendo el Duckiebot.")
        msg = Twist2DStamped()
        msg.v = 0.0
        msg.omega = 0.0
        self.twist_publisher.publish(msg)

if __name__ == '__main__':
    node = lane_follow_node(node_name='custom_lane_follow')
    rate = rospy.Rate(2)
    while not rospy.is_shutdown() and node.run:
        node.img_pub()
        node.twist_pub()
        rate.sleep()
