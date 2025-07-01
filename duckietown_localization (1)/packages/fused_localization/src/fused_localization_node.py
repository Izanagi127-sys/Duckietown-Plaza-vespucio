#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 20 11:08:11 2020
@author: yueshan (modificado)
"""

import numpy as np
import rospy
import tf
import tf2_ros
from duckietown.dtros import DTROS, NodeType
from geometry_msgs.msg import TransformStamped
from encoder_localization.srv import FrameCalibration, FrameCalibrationRequest

class FusedLocalizationNode(DTROS):

    def __init__(self, node_name):
        super(FusedLocalizationNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        self.first_detected_at = False
        self.detected_at = False
        self.mode = 0  # 0: encoder, 1: april tag
        self.T_EF = np.eye(4)
        self.last_time = rospy.Time()
        self.threshold = 1  # segundos

        self.br = tf2_ros.TransformBroadcaster()

        self.pub_fused_localization = rospy.Publisher('fused_localization_node/transform', TransformStamped, queue_size=1)

        self.sub_encoder_localization = rospy.Subscriber(
            'encoder_localization_node/transform', TransformStamped, self.cb_encoder)
        self.sub_at_localization = rospy.Subscriber(
            'at_localization_node/transform', TransformStamped, self.cb_at)

        rospy.loginfo("FusedLocalizationNode inicializado.")

    def cb_at(self, transform_at):
        if self.mode == 0:
            self.mode = 1

        self.last_time = transform_at.header.stamp
        at_matrix = self.msg_to_matrix(transform_at)
        al, be, ga = tf.transformations.euler_from_matrix(at_matrix, axes='szxy')

        fused_matrix = tf.transformations.euler_matrix(al, 0, 0, 'szxy')
        fused_matrix[0:2, 3] = at_matrix[0:2, 3]
        self.T_MF = fused_matrix

        transform_fused = self.matrix_to_msg(fused_matrix, 'map', 'fused_baselink', transform_at.header.stamp)

        if not self.first_detected_at:
            self.first_detected_at = True
            rospy.wait_for_service('encoder_localization_node/frame_calibration')
            frame_calibration = rospy.ServiceProxy('encoder_localization_node/frame_calibration', FrameCalibration)
            req = FrameCalibrationRequest(transform_fused)
            frame_calibration(req)

        try:
            self.br.sendTransform(transform_fused)
        except rospy.exceptions.ROSException as e:
            rospy.logwarn(f"No se pudo publicar transform (AT): {e}")
        self.pub_fused_localization.publish(transform_fused)

    def cb_encoder(self, transform_encoder):
        duration = rospy.Duration.from_sec(self.threshold)
        if (rospy.get_rostime() - self.last_time) > duration:
            T_ME = self.msg_to_matrix(transform_encoder)
            if self.mode == 1:
                self.T_EF = np.linalg.inv(T_ME).dot(self.T_MF)
                self.mode = 0
            else:
                self.T_MF = T_ME.dot(self.T_EF)
                transform_fused = self.matrix_to_msg(self.T_MF, 'map', 'fused_baselink', transform_encoder.header.stamp)
                try:
                    self.br.sendTransform(transform_fused)
                except rospy.exceptions.ROSException as e:
                    rospy.logwarn(f"No se pudo publicar transform (encoder): {e}")
                self.pub_fused_localization.publish(transform_fused)

    def matrix_to_msg(self, T, frame_id, child_frame_id, time=rospy.Time()):
        transform = TransformStamped()
        transform.header.stamp = time
        transform.header.frame_id = frame_id
        transform.child_frame_id = child_frame_id

        transform.transform.translation.x = T[0, 3]
        transform.transform.translation.y = T[1, 3]
        transform.transform.translation.z = T[2, 3]

        quaternion = tf.transformations.quaternion_from_matrix(T)

        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]

        return transform

    def msg_to_matrix(self, msg):
        matrix = np.eye(4)
        matrix[0, 3] = msg.transform.translation.x
        matrix[1, 3] = msg.transform.translation.y
        matrix[2, 3] = msg.transform.translation.z

        qx = msg.transform.rotation.x
        qy = msg.transform.rotation.y
        qz = msg.transform.rotation.z
        qw = msg.transform.rotation.w

        R = tf.transformations.quaternion_matrix([qx, qy, qz, qw])
        matrix[0:3, 0:3] = R[0:3, 0:3]

        return matrix

if __name__ == '__main__':
    node = FusedLocalizationNode(node_name='fused_localization_node')
    rospy.on_shutdown(lambda: rospy.loginfo("Apagando fused_localization_node"))
    rospy.spin()
