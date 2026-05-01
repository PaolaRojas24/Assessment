# Team: Guns-n-ROSes - Reporte de Estatus - Semana 1

## 1. Hito de la Semana 

* Objetivo: Visualización de los datos del sensor lidar del qcar en rviz </br>
Estado: 🟢 Completado

* Objetivo: Prueba para conocer como maniobra el qcar. </br>
Estado:  🟢 Completado 

* Objetivo: Seguimiento de carril. </br>
Estado:  🟡 En proceso 

## 2. Logros Técnicos

* Crear un paquete de ROS 2 titulado `lidar_qcar` que permite visualizar en rviz los valores enviados por el sensor lidar del qcar. Esta compuesto por:
  * `lidar_visualizer.launch.py`: Launcher que permite lanzar el nodo del lidar y el nodo de rviz
  * `lidar_view.rviz`: Archivo que configura el rviz para una mejor visualización.
  * `idar_node.py`: Nuestro nodo principal, se subscribe a el tópico de <em>/qcar/scan</em> para resivir los datos del Lidar del qcar


* Crear un paquete de ROS 2 titulado `vector3_teleop` que permite conocer como maniobra el qcar. Esta compuesto por:
  * `vector3_publisher.py`: Nodo principal que se subscribe al tópico de <em>/qcar/user_command</em> para programar el movimiento del qcar en forma de onda senoidal.

## 3. Obstáculos y Bloqueos
* Debido a una diferencia de versiones de Gazebo, la simulación proporcionada no funciona correctamente. La simulación original está preparada para funcionar con Gazebo Harmonic, mientras que la versión que utilizan dos integrantes del equipo es Garden. Como se puede observar en la Figura 1, tanto el rviz como el gazebo se visualizan correctamente, el problema es que no se están enviando los topicos corresponcientes.

<p align="center">
  <img src="https://github.com/PaolaRojas24/Assessment/blob/main/Documentation/Images/gazebo_garden.png"/>
</p>
<p align="center"><em>Figura 1. Gazebo Graden </em></p>


## 4. Plan para la Siguiente Semana

* Seguir trabajando con el robot qcar en físico.
* Trabajar con la cámara del qcar para obtener los primeros resultados
* Programar Lidar y Cámara para detección de objetos.

## 5. Evidencias

En la Figura 2, se puede observar los datos que envía el Lidar, en este caso correspondiendo a las paredes de la pista.

<p align="center">
  <img src="https://github.com/PaolaRojas24/Assessment/blob/main/Documentation/Images/rviz_lidar.png"/>
</p>
<p align="center"><em>Figura 2. Visualización del Lidar en rviz</em></p>
