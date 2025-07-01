#!/usr/bin/env python3
import rospy
import os
import threading
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped, LEDPattern
from std_msgs.msg import ColorRGBA, String

class ApriltagNode(DTROS):
    def __init__(self, node_name):
        super(ApriltagNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.node_name = node_name
        self.veh_name = os.environ['VEHICLE_NAME']
        self.estado = False

        self.dict_apriltag = {
            "Pare": [26, 31, 32, 33],
            "Cruce_I": [61, 10],
            "Cruce_D": [57, 9],
            "Cruce_T": [65, 11]
        }
        self.stops = self.dict_apriltag["Pare"]
        self.detectado = None

        # Publishers
        self.cmd_pub = rospy.Publisher(f"/{self.veh_name}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)
        self.led_pub = rospy.Publisher(f"/{self.veh_name}/led_emitter_node/led_pattern", LEDPattern, queue_size=1)
        self.conduc_pub = rospy.Publisher(f"/{self.veh_name}/car_node/conduccion/estado", String, queue_size=1)

        # Subscriber al nodo de visión superior
        rospy.Subscriber("/vision_superior/apriltags_detectados", String, self.apriltag_callback)

        rospy.loginfo("✅ Nodo apriltag_node conectado a vision_superior_node")

    def apriltag_callback(self, msg):
        try:
            tag_id = int(msg.data)
            if tag_id == self.detectado:
                return  # Ya procesado
            self.detectado = tag_id

            rospy.loginfo(f"🔵 AprilTag recibido desde vision_superior_node: {tag_id}")

            if tag_id in self.stops:
                rospy.loginfo("🛑 Señal de Pare detectada")
                self.set_pausa(True)
                threading.Timer(3, self.set_pausa, [False]).start()
                threading.Timer(0.2, self.mover, [0.0, 0.0]).start()
            elif tag_id in self.dict_apriltag["Cruce_I"]:
                rospy.loginfo("↩️ Cruce a la Izquierda detectado")
                # Aquí puedes agregar lógica para doblar o indicar maniobra
            elif tag_id in self.dict_apriltag["Cruce_D"]:
                rospy.loginfo("↪️ Cruce a la Derecha detectado")
            elif tag_id in self.dict_apriltag["Cruce_T"]:
                rospy.loginfo("🔀 Cruce en T detectado")

        except Exception as e:
            rospy.logwarn(f"❌ Error en apriltag_callback: {e}")

    def mover(self, v, omega):
        if not self.estado:
            msg = Twist2DStamped()
            msg.v = v
            msg.omega = omega
            self.cmd_pub.publish(msg)

    def set_pausa(self, estado):
        msg = String()
        msg.data = f"{not estado}"
        self.conduc_pub.publish(msg)
        self.estado = estado

if __name__ == '__main__':
    node = ApriltagNode(node_name='apriltag_node')
    rospy.spin()
