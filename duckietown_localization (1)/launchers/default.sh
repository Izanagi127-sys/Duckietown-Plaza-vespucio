#!/bin/bash

source /environment.sh

# Inicializar archivo de lanzamiento
dt-launchfile-init

# ----------------------------------------------------------------------------
# 🧠 Nodo centralizado de percepción (una sola lectura de cámara)
dt-exec rosrun vision_superior_node vision_superior_node.py  &

# ----------------------------------------------------------------------------
# 🛰️ Nodos de localización
dt-exec roslaunch encoder_localization encoder_localization_node.launch veh:="$VEHICLE_NAME" &
dt-exec roslaunch at_localization at_localization_node.launch veh:="$VEHICLE_NAME" &

# ----------------------------------------------------------------------------
# 🧭 Nodo de seguimiento de pista (lane following)
dt-exec rosrun lane_follow lane_follow.py veh:="$VEHICLE_NAME" &
# ----------------------------------------------------------------------------
# 🏷️ Nodo de AprilTags (debe modificarse para usar /vision_superior/image_gray)
dt-exec roslaunch apriltag apriltag_node.launch veh:="$VEHICLE_NAME" &

# ----------------------------------------------------------------------------
# 💡 Nodo de control de LEDs
dt-exec roslaunch led_emitter led_emitter_node.launch veh:="$VEHICLE_NAME" &

# ----------------------------------------------------------------------------
# 🦆 Nodo de detección de patitos (puede conectarse a /vision_superior/image_color)
dt-exec roslaunch duck_detector duck_detector.launch veh:="$VEHICLE_NAME" &

# ----------------------------------------------------------------------------
# 🧠 Nodo principal bloqueante (espera final de ejecución)
dt-exec roslaunch fused_localization fused_localization_node.launch veh:="$VEHICLE_NAME"

# ----------------------------------------------------------------------------
# Esperar a que los procesos terminen
dt-launchfile-join
