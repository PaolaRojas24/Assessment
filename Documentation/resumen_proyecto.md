
**Resumen Ejecutivo: Proyecto de Vehículo Autónomo para el Torneo Mexicano de Robótica 2026**

**Equipo:** [Nombre del Equipo/Laboratorio]
**Institución:** [Nombre de la Institución]
**Fecha:** 1 de octubre de 2025

**1. Introducción**

El manejo autónomo representa una de las fronteras tecnológicas más relevantes y de mayor crecimiento en la actualidad, fusionando disciplinas como la inteligencia artificial, la robótica, la visión por computadora y la teoría de control. Este documento describe el proyecto para el diseño, desarrollo e implementación de un vehículo autónomo a escala 1:10, con el fin de competir en la categoría *Automodel Car* del prestigioso Torneo Mexicano de Robótica (TMR), que se llevará a cabo en mayo de 2026. Este proyecto no solo constituye un reto de ingeniería de alto nivel, sino también una plataforma excepcional para el desarrollo de competencias técnicas y profesionales avanzadas en estudiantes de licenciatura.

**2. Objetivo General**

Diseñar, construir e implementar un vehículo autónomo a escala 1:10 que sea capaz de navegar de manera segura y eficiente en un entorno desconocido, superando exitosamente todas las pruebas de la competencia. Esto incluye el seguimiento de carril, la detección y reacción a señales de tránsito, la evasión de obstáculos fijos y móviles, y la ejecución de maniobras de precisión como el estacionamiento en reversa. El objetivo es posicionar a nuestro equipo e institución como un referente en robótica móvil a nivel nacional.

**3. Sistemas a Desarrollar e Implementar**

El vehículo funcionará con una arquitectura de software modular basada en **ROS 2 (Robot Operating System 2)**, centralizada en una unidad de cómputo de alto rendimiento **NVIDIA Jetson Orin Nano**. La inteligencia del sistema se distribuirá en cinco subsistemas interconectados que operarán en tiempo real:

* **1. Percepción:** Responsable de "ver" y entender el entorno. Utilizará una **cámara estéreo ZED 2** para la detección de líneas de carril y señales de tránsito, y un sensor **LiDAR de 360°** para la detección precisa de obstáculos.
* **2. Estimación y Localización:** Su función es responder a la pregunta "¿Dónde estoy?". Fusionará los datos de **encoders magnéticos** en las llantas y una **Unidad de Medición Inercial (IMU)** para estimar con alta precisión la posición y orientación del vehículo en todo momento.
* **3. Planeación de Trayectorias:** Actuará como el "cerebro" para la toma de decisiones. Generará las rutas óptimas y seguras, desde seguir una línea hasta complejas maniobras de evasión y estacionamiento, basándose en la información de los sistemas de percepción y localización.
* **4. Control:** Será el encargado de traducir las decisiones del planificador en acciones físicas. Implementará algoritmos de control (ej. PID) para manipular con precisión el ángulo de la dirección y la velocidad del motor, asegurando que el vehículo siga la trayectoria deseada.
* **5. Sistema Embebido y Gestión de Energía:** Constituye la base del vehículo, asegurando la integridad del hardware, la distribución de energía desde una **batería LiPo de 11.1V** y la comunicación de bajo nivel entre la Jetson, los sensores y los actuadores.

**4. Impacto en la Formación de los Estudiantes Participantes**

La participación en este proyecto ofrece un valor formativo integral que va más allá del salón de clases.

* **Corto Plazo (Durante el desarrollo):** Los estudiantes aplicarán conocimientos teóricos de cálculo, álgebra, programación y electrónica en un problema real y tangible. Adquirirán competencias técnicas en tecnologías de alta demanda como Linux, Python/C++, ROS 2, OpenCV y plataformas de IA. Desarrollarán habilidades interpersonales cruciales como el trabajo en equipo, la gestión de proyectos y la resolución de problemas complejos bajo presión.

* **Mediano Plazo (Al egresar):** Este proyecto se convertirá en un diferenciador clave en su currículum, demostrando experiencia práctica en el ciclo completo de desarrollo de un producto tecnológico. Serán profesionales altamente competitivos para integrarse a industrias de vanguardia como la automotriz, la robótica, la automatización industrial y el desarrollo de software de alto nivel.

* **Largo Plazo (Carrera profesional):** Fomentará una mentalidad de innovación y una pasión por la tecnología que puede impulsar la creación de nuevas empresas (startups), la continuación de estudios de posgrado de alto impacto o el asumir roles de liderazgo en equipos de desarrollo tecnológico. Los participantes no solo aprenderán a resolver un problema, sino que aprenderán a "aprender", adaptándose a los rápidos cambios tecnológicos que definirán las próximas décadas.
