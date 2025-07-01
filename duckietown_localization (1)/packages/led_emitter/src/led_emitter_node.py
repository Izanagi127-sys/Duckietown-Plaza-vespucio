#!/usr/bin/env python3
import rospy
import json
from std_msgs.msg import String
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA

class LEDEmitterNode(DTROS):
    def __init__(self, node_name):
        super(LEDEmitterNode, self).__init__(node_name=node_name, node_type=NodeType.DRIVER)
        self.node_name = node_name
        self.veh_name = rospy.get_namespace().strip("/")
        self.led_pub = rospy.Publisher(f"/{self.veh_name}/led_emitter_node/led_pattern", LEDPattern, queue_size=1)

        # Colores disponibles
        self.colores = {
            "WHITE": [1.0, 1.0, 1.0],
            "BLUE": [0.0, 0.0, 1.0],
            "GREEN": [0.0, 1.0, 0.0],
            "ORANGE": [1.0, 0.647, 0.0],
            "RED": [1.0, 0.0, 0.0],
            "BLACK": [0.0, 0.0, 0.0],
            "YELLOW": [1.0, 1.0, 0.0],
        }

        # Suscripciones a mensajes del nodo central
        rospy.Subscriber("/vision_superior/apriltags_detectados", String, self.callback_apriltag)
        rospy.Subscriber("/vision_superior/qr_detectado", String, self.callback_qr)

        rospy.loginfo("✅ LED Emitter Node escuchando a vision_superior_node")

    def change_color(self, color_name):
        color = self.colores.get(color_name.upper(), [1.0, 1.0, 1.0])
        pattern = LEDPattern()
        for _ in range(5):
            rgba = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
            pattern.rgb_vals.append(rgba)
        self.led_pub.publish(pattern)

    def apagar_leds(self):
        self.change_color("BLACK")

    def callback_apriltag(self, msg):
        try:
            tag_id = int(msg.data)
            rospy.loginfo(f"💡 AprilTag detectado: {tag_id}")

            if tag_id == 31:
                self.change_color("RED")
            elif tag_id == 32:
                self.change_color("GREEN")
            elif tag_id == 33:
                self.change_color("BLUE")
            else:
                self.change_color("WHITE")
        except Exception as e:
            rospy.logwarn(f"❌ Error en callback_apriltag: {e}")

    def callback_qr(self, msg):
        try:
            data = json.loads(msg.data)
            accion = data.get("accion", "")
            tipo_luz = data.get("tipo_luz", "")
            repeticiones = int(data.get("repeticiones", 1))

            if tipo_luz == "frontal":
                color = "WHITE"
            elif tipo_luz == "trasera":
                color = "RED"
            elif tipo_luz == "ambas":
                color = "YELLOW"
            else:
                color = "WHITE"

            if accion == "encender_luces":
                rospy.loginfo(f"💡 Encendiendo luces {color} x{repeticiones}")
                for _ in range(repeticiones):
                    self.change_color(color)
                    rospy.sleep(0.5)
            elif accion == "parpadear_luces":
                rospy.loginfo(f"⚡ Parpadeando luces {color} x{repeticiones}")
                for _ in range(repeticiones):
                    self.change_color(color)
                    rospy.sleep(0.5)
                    self.apagar_leds()
                    rospy.sleep(0.5)
            else:
                rospy.logwarn("⚠️ Acción desconocida en QR.")

        except json.JSONDecodeError:
            rospy.logwarn("⚠️ QR recibido no es JSON válido.")
        except Exception as e:
            rospy.logerr(f"❌ Error en callback_qr: {e}")

if __name__ == '__main__':
    node = LEDEmitterNode(node_name='led_emitter_node')
    rospy.spin()
