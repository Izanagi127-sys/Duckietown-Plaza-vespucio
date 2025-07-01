# Duckietown-Plaza-vespucio
# Duckietown DB21J Localization Repository

Este repositorio contiene código ROS para el manejo de distintos módulos de percepción y control en el Duckiebot DB21J equipado con una Jetson Nano 4GB. Estos módulos permiten al Duckiebot navegar autónomamente detectando señales visuales, siguiendo carriles, detectando obstáculos y respondiendo a códigos QR y AprilTags.

## Contenido del Repositorio

### Nodos Principales

#### 1. **Apriltag Node** (`apriltag_node.py`)

**Función**: Detección de AprilTags para navegación y ejecución de maniobras específicas (detenciones, giros).

* **Tópicos ROS**:

  * `/VEHICLE_NAME/camera_node/image/compressed`: Imágenes de la cámara.
  * `/VEHICLE_NAME/april_tag/compressed`: Publica imágenes procesadas.
  * `/VEHICLE_NAME/led_emitter_node/led_pattern`: Controla LEDs según detecciones.

#### 2. **QR Reader Node** (`qr_reader_nodo.py`) ⚠️

**Función**: Lee códigos QR, interpreta instrucciones JSON contenidas en los códigos y ejecuta acciones como encendido de luces y pausas.

⚠️ **Nota:** Actualmente, este módulo no está funcionando correctamente. Se recomienda verificar y depurar el código antes de su uso operativo.

* **Tópicos ROS**:

  * `/VEHICLE_NAME/camera_node/image/compressed`: Imágenes de la cámara.
  * `/VEHICLE_NAME/qrreader/compressed`: Publica imágenes procesadas.
  * `/VEHICLE_NAME/led_emitter_node/led_pattern`: Control visual mediante LEDs.

#### 3. **Lane Follow Node** (`lane_follow.py`)

**Función**: Seguimiento automático de carriles usando detección de colores mediante máscaras HSV y control PID.

* **Tópicos ROS**:

  * `/VEHICLE_NAME/camera_node/image/compressed`: Imágenes de la cámara.
  * `/VEHICLE_NAME/masked_image/compressed`: Imágenes con máscaras aplicadas para depuración.
  * `/VEHICLE_NAME/car_cmd_switch_node/cmd`: Comandos de velocidad y dirección.

#### 4. **Duck Detector Node** (`duckdetector.py`)

**Función**: Detección visual de patitos (objetos amarillos) mediante máscaras HSV.

* **Tópicos ROS**:

  * `/VEHICLE_NAME/camera_node/image/compressed`: Imágenes de la cámara.
  * `/VEHICLE_NAME/duckdetector/compressed`: Publica imágenes procesadas mostrando detecciones.

#### 5. **Distance Node** (`distancia.py`)

**Función**: Detección de proximidad usando sensores ToF para evitar colisiones.

* **Tópicos ROS**:

  * `/VEHICLE_NAME/front_center_tof_driver_node/range`: Detección de distancia frontal.
  * `/VEHICLE_NAME/car_cmd_switch_node/cmd`: Comandos para detener el robot.

#### 6. **LED Emitter Node** (`led_emitter_node.py`)

**Función**: Gestión avanzada del patrón visual de los LEDs del Duckiebot para señalizar estados y acciones.

* **Tópicos y Servicios ROS**:

  * Servicio `/HOSTNAME/led_emitter_node/set_custom_pattern`: Establece patrones LED personalizados.
  * Servicio `/HOSTNAME/led_emitter_node/set_pattern`: Cambia a patrones predefinidos.


¡Gracias por contribuir al proyecto Duckietown!
