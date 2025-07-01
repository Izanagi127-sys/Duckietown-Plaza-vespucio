#!/usr/bin/env python3
import rospy
import json
from std_msgs.msg import String, Bool
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA

class QRReaderNode(DTROS):
    def __init__(self, node_name):
        super(QRReaderNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")
        self.qr_procesados = set()

        self.desafio_activo = rospy.get_param("~desafio_activo", 2)
        self.orden_esperado = [1, 2, 3] if self.desafio_activo == 2 else None
        self.indice_actual = 0

        self.led_pub = rospy.Publisher(f"/{self.veh_name}/led_emitter_node/led_pattern", LEDPattern, queue_size=1)
        self.retorno_pub = rospy.Publisher("/qr_reader/retorno_pedido", Bool, queue_size=1)

        rospy.Subscriber("/vision_superior/qr_detectado", String, self.callback_qr)

        self.colores = {
            "WHITE": [1.0, 1.0, 1.0],
            "RED": [1.0, 0.0, 0.0],
            "YELLOW": [1.0, 1.0, 0.0],
            "BLACK": [0.0, 0.0, 0.0],
            "GREEN": [0.0, 1.0, 0.0]
        }

        rospy.loginfo(f"✅ QRReaderNode iniciado (Desafío {self.desafio_activo})")

    def callback_qr(self, msg):
        data = msg.data
        if data in self.qr_procesados:
            return
        self.qr_procesados.add(data)

        rospy.loginfo(f"🟩 QR recibido: {data}")

        try:
            qr_json = json.loads(data)
            qr_id = int(qr_json.get("id", -1))

            if not self.es_qr_valido(qr_id):
                rospy.loginfo(f"🔕 QR ignorado (ID {qr_id} no válido para desafío {self.desafio_activo})")
                return

            if self.orden_esperado:
                if qr_id != self.orden_esperado[self.indice_actual]:
                    rospy.logwarn(f"⚠️ Se esperaba ID {self.orden_esperado[self.indice_actual]}, recibido {qr_id}")
                    return
                rospy.loginfo(f"🟢 Orden correcto: ID {qr_id}")
                self.indice_actual += 1

            rospy.loginfo(f"📦 Contenido QR:\n{json.dumps(qr_json, indent=2)}")
            self.feedback_visual_exito()
            self.ejecutar_accion(qr_json)

        except json.JSONDecodeError:
            rospy.logwarn("❌ QR no es JSON válido.")
        except Exception as e:
            rospy.logerr(f"🚨 Error procesando QR: {e}")

    def feedback_visual_exito(self):
        self.publicar_leds(self.colores["GREEN"])
        rospy.sleep(0.3)
        self.publicar_leds(self.colores["BLACK"])

    def es_qr_valido(self, qr_id):
        if self.desafio_activo == 1:
            return qr_id == 1
        elif self.desafio_activo == 2:
            return qr_id in [1, 2, 3]
        elif self.desafio_activo == 3:
            return qr_id in [1, 2, 3, 4]
        return False

    def ejecutar_accion(self, qr_json):
        accion = qr_json.get("accion", "")
        tipo_luz = qr_json.get("tipo_luz", "frontal")
        repeticiones = int(qr_json.get("repeticiones", 1))
        tiempo_detencion = float(qr_json.get("tiempo_detencion", 0))
        retornar = qr_json.get("retornar_punto_partida", False)

        color = self.obtener_color(tipo_luz)

        if accion == "encender_luces":
            rospy.loginfo("💡 Encendiendo luces")
            for _ in range(repeticiones):
                self.publicar_leds(color)
                rospy.sleep(0.5)
                self.publicar_leds(self.colores["BLACK"])
                rospy.sleep(0.2)
        elif accion == "parpadear_luces":
            rospy.loginfo("⚡ Parpadeando luces")
            for _ in range(repeticiones):
                self.publicar_leds(color)
                rospy.sleep(0.5)
                self.publicar_leds(self.colores["BLACK"])
                rospy.sleep(0.5)
        else:
            rospy.logwarn("⚠️ Acción no reconocida.")

        if tiempo_detencion > 0:
            rospy.loginfo(f"⏸️ Detención por {tiempo_detencion:.1f} segundos...")
            rospy.sleep(tiempo_detencion)

        if retornar:
            rospy.loginfo("🔁 Retorno solicitado")
            self.retorno_pub.publish(Bool(data=True))

    def obtener_color(self, tipo):
        return {
            "frontal": self.colores["WHITE"],
            "trasera": self.colores["RED"],
            "ambas": self.colores["YELLOW"]
        }.get(tipo, self.colores["WHITE"])

    def publicar_leds(self, color):
        pattern = LEDPattern()
        for _ in range(5):
            pattern.rgb_vals.append(ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0))
        self.led_pub.publish(pattern)

if __name__ == '__main__':
    rospy.init_node('qr_reader_node')
    node = QRReaderNode(node_name='qr_reader_node')
    rospy.spin()