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

#### 3. **Lane Follow Node con Odometría** (`lane_follow.py`)

**Función**: Seguimiento automático de carriles usando detección de colores mediante máscaras HSV, control PID y odometría para estimar la posición y orientación del Duckiebot.

* **Funcionamiento con Odometría**: El módulo utiliza sensores de rotación en las ruedas (encoders) para calcular la posición relativa del robot. Esta información se combina con la detección visual del carril para corregir y mejorar la precisión del seguimiento.

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


```

## Estructura de Configuración

Para facilitar el manejo y activación de nodos, se recomienda centralizar su gestión en el script `default.sh`. Ejemplo:

```bash
# Lanzamiento automático de nodos principales
roslaunch apriltag apriltag_node.launch veh:="$VEHICLE_NAME" &
rosrun lane_follow lane_follow.py &
roslaunch duck_detector duck_detector.launch veh:="$VEHICLE_NAME" &
```



comando claves 

dts init_sd_card --hostname miguelito --type duckiebot --configuration db21j --wifi WIFI

// ambiente virtual instalcion duckie shell
python3 -m venv ~/my_virtual_env
source ~/my_virtual_env/bin/activate
pip install --upgrade pip
pip install duckietown-shell

which dts
nano ~/.bashrc
---- agregar al final: export PATH=~/.local/bin:${PATH}

pip install setuptools


dts version

dts init_sd_card --hostname duckieMont  --type duckiebot --configuration DB21J --wifi 'Casa Pardo:18642160'


https://hub.docker.com/

github:
git --version
sudo apt update
sudo apt install git

ssh-keygen -t ed25519 -C "tu_correo@ejemplo.com"
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
cat ~/.ssh/id_ed25519.pub

https://github.com/settings/keys

si todo esta bien:  ssh -T git@github.com
daria un msj de bienvenida

finalmente configurar:
git config --global user.name "tu_nombre_de_usuario"
git config --global user.email "tu_correo@ejemplo.com"





dts fleet discover
dts duckiebot calibrate_intrinsics [your_duckiebot_hostname]
/data/config/calibrations/camera_intrinsic/[your_duckiebot_hostname].yaml
dts duckiebot calibrate_extrinsics [your_duckiebot_hostname]
/data/config/calibrations/camera_extrinsic/[your_duckiebot_hostname].yaml









dts duckiebot keyboard_control ![DUCKIEBOT_NAME]
dts start_gui_tools [hostname]

si va hacia la izquierda se le disminuye el trim, si va hacia la derecha el trim debe aumentar


rosparam get /autito/kinematics_node/trim 


rosparam set /[hostname]/kinematics_node/trim [trim_value]




finalmente guardar todo esto con:
rosservice call /[hostname]/kinematics_node/save_calibration
/data/config/calibrations/kinematics/[hostname].yaml










demos: LF
dts duckiebot demo --demo_name lane_following --duckiebot_name zombieduck --package_name duckietown_demos

dts code workbench -b nombre-Duckie
dts code workbench 


To build:
```dts devel build -H ROBOT_NAME -f``` or ```dts devel build -f```

To run:
``````dts devel run -R ROBOT_NAME```
ssh duckie@Pato quackquack
sudo apt update && sudo apt install curl -y


sudo apt-key del F42ED6FBAB17C654 && \
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
dts devel build -H eustaquio && dts devel run -H eustaquio




#Corregir errores
RUN sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

# install apt dependencies


   <arg name="veh" doc="Name of vehicle." default="$(env VEHICLE_NAME)"/>
   <arg name="pkg_name" value="lane_follow"/>
   <arg name="node_name" default="lane_follow_node"/>
   <group ns="$(arg veh)">


ROAD_MASK = [(20, 45, 25), (35, 255, 255)]
STOP_LINE_MASK = [(0, 128, 161), (10, 225, 225)]

dt-gui-tools# rqt_image_view
ts devel build -H miguelito -f --no-cache

Ideas para el futuro
buscar como habilitar los nucleos cuda de la jetson nano para reducir carga de la cpu y mejorar rendimiento general utilizando opencv con cuda

