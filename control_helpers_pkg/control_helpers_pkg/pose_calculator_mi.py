#!/usr/bin/env python3
"""
Nodo simple que integra el modelo de bicicleta para obtener solo la orientación
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Vector3Stamped, PoseStamped
from std_msgs.msg import Float64
import math

class QCarOrientation(Node):
    def __init__(self):
        super().__init__('qcar_orientation')
        
        # Parámetros
        self.declare_parameter('wheelbase', 0.256)  # m
        self.declare_parameter('max_steering_angle', 0.3)  # rad
        self.declare_parameter('velocity_scale', 1)  # Factor de escala para velocity
        self.declare_parameter('velocity_threshold', 0.000000000001)  # m/s - ignorar valores menores
        
        self.L = self.get_parameter('wheelbase').value
        self.max_steering = self.get_parameter('max_steering_angle').value
        self.velocity_scale = self.get_parameter('velocity_scale').value
        self.velocity_threshold = self.get_parameter('velocity_threshold').value
        
        # Estado
        self.theta = 0.0  # Orientación en radianes
        self.velocity = 0
        self.steering = 0.0
        self.direction = 1
        self.alpha = 0.0
        # Tiempo
        self.last_time = self.get_clock().now()
        
        # Subscribers
        self.velocity_sub = self.create_subscription(
            Vector3Stamped,
            '/qcar/velocity',
            self.velocity_callback,
            10
        )
        
        self.command_sub = self.create_subscription(
            Vector3Stamped,
            '/qcar/user_command',
            self.command_callback,
            10
        )
        
        # Publisher - solo la orientación
        self.theta_pub = self.create_publisher(
            Float64,
            '/qcar/orientation',
            10
        )
        
        # Timer para integrar
        self.timer = self.create_timer(0.02, self.integrate)  # 50 Hz
        
        self.get_logger().info('Nodo de orientación QCAR iniciado')
        self.get_logger().info(f'Wheelbase: {self.L} m')
        self.get_logger().info(f'Max steering: {math.degrees(self.max_steering):.1f}°')
        self.get_logger().info(f'Velocity scale: {self.velocity_scale}x')
        self.get_logger().info(f'Velocity threshold: {self.velocity_threshold} m/s')
        
    def velocity_callback(self, msg):
        # Obtener velocidad y aplicar escala
        # raw_velocity = msg.vector.x 
        # raw_velocity = math.cos(self.steering) / (msg.vector.x + 1e-6)
        #float(np.cos(self.command[1]) / linearVelocity)
        raw_velocity = math.sqrt(msg.vector.x**2 + msg.vector.y**2)

        self.alpha = math.atan2(msg.vector.y, msg.vector.x) 
        self.direction = -1 if msg.vector.x < 0. else 1
        # Aplicar factor de escala
        scaled_velocity = raw_velocity #* self.velocity_scale
        
        # Aplicar threshold (ignorar ruido)
        if abs(scaled_velocity) < self.velocity_threshold:
            self.velocity = 0
        else:
            self.velocity = scaled_velocity
        
    def command_callback(self, msg):
        # Obtener steering del comando
        self.steering = msg.vector.y
        
    def integrate(self):
        """Integrar el modelo de bicicleta para obtener orientación"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        dt = max(0.001, min(dt, 0.1))
        
        # Modelo de bicicleta: dtheta/dt = (v / L) * tan(delta)
        # if abs(self.steering) < 0.001:
        #     omega = 0.0
        # else:
        omega = (self.velocity / self.L) * math.sin(self.steering)

        # self.get_logger().info(f"ω = {omega:.6f} rad/s")

        # Integrar
        self.theta += (omega * dt) 
        # self.theta += omega *0.02
        # Normalizar entre -pi y pi
        # self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        # self.theta = math.atan(math.sin(self.theta)/ math.cos(self.theta))
        # Publicar
        msg = Float64()
        msg.data = self.theta
        self.theta_pub.publish(msg)
        
        # Log cada 1 segundo
        if int(current_time.nanoseconds / 1e9) % 1 == 0 and dt < 0.1:
            self.get_logger().info(
                f'θ = {self.theta:.3f}rad | Degree = {math.degrees(self.theta):+7.2f}° | '
                f'v = {self.velocity:.10f} m/s | δ = {math.degrees(self.steering):+6.2f}°'
            )
        
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = QCarOrientation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f'\nOrientación final: {node.theta:.3f} rad = {math.degrees(node.theta):+.2f}°')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()