# Team: Guns-n-ROSes - Reporte de Estatus - Semana 2

## 1. Hito de la Semana 

* Objetivo: Detección de obstáculos con el sensor LiDAR dentro de una zona de seguridad configurable en entorno fisico </br>
Estado: 🟢 Completado

* Objetivo: Frenado automático del QCar al detectar un obstáculo en la zona de seguridad. </br>
Estado:  🟢 Completado 

* Objetivo: Seguimiento de carril. </br>
Estado:  🟡 En proceso 

## 2. Logros Técnicos

* Se amplió el paquete de ROS 2 `lidar_qcar` para incorporar lógica de detección de obstáculos y control reactivo del QCar. Los nuevos componentes son:
  * `control_node.py`: Nodo que se suscribe al tópico <em>/qcar/obstacle_detected</em> y publica comandos de velocidad en <em>/qcar/user_command</em>. Cuando se detecta un obstáculo, envía velocidad cero deteniendo el robot; cuando la zona queda despejada, retoma la marcha hacia adelante.
  * `lidar_control.launch.py`: Launcher que levanta en conjunto el `lidar_node`, el `control_node`, el `robot_state_publisher`, el `joint_state_publisher`, un transformador estático TF y RViz2.


* Se añadió lógica de zona de seguridad rectangular al lidar_node.py. La detección evalúa cada rayo del LiDAR con la siguiente geometría:
  * <b>Zona activa</b>: si un punto válido cae dentro de este rectángulo centrado en el robot, se publica True en <em>/qcar/obstacle_detected</em>.
  * <b>Dead zone</b>: región interior alrededor del robot que se ignora, evitando falsas detecciones causadas por partes del propio chasis.


* La zona de seguridad se visualiza en tiempo real en RViz2 mediante un marcador tipo CUBE publicado en <em>/qcar/safety_zone</em>: el rectángulo cambia de color verde (libre) a rojo (obstáculo detectado) y la dead zone se muestra en azul.

## 3. Obstáculos y Bloqueos
* La alta latencia de la cámara del QCar dificulta el desarrollo de un seguidor de carril en tiempo real. El retardo en la recepción de los fotogramas impide una respuesta de control suficientemente rápida, por lo que este objetivo se mantiene bloqueado hasta encontrar una estrategia que mitigue el problema de latencia.

## 4. Plan para la Siguiente Semana

* Retomar el desarrollo del seguidor de carril una vez mitigada la latencia.
* Empezar a implementar la lógica de rebasar objetos con el Lidar.

## 5. Evidencias

En la Figura 1, se puede observar la zona activa (rectángulo verde) y la dead zone (rectángulo azul).

<p align="center">
  <img src="https://github.com/PaolaRojas24/Assessment/blob/main/Documentation/Images/lidar_green.png"/>
</p>
<p align="center"><em><b>Figura 2.</b> Visualización de zona activa y dead zobe en Rviz</em></p>

En la Figura 2, se puede observar como el rectángulo se vuelve rojo cuando el lidar detecta un objeto dentro de la zona activa.

<p align="center">
  <img src="https://github.com/PaolaRojas24/Assessment/blob/main/Documentation/Images/lidar_red.png"/>
</p>
<p align="center"><em><b>Figura 2.</b> Visualización de obstáculos en Rviz</em></p>

### Enlaces

* [control_node.py](https://github.com/PaolaRojas24/Assessment/blob/dd36cc6/lidar_qcar/lidar_qcar/control_node.py) (commit `dd36cc6`)
* [lidar_control.launch.py](https://github.com/PaolaRojas24/Assessment/blob/dd36cc6/lidar_qcar/launch/lidar_control.launch.py) (commit `dd36cc6`)
