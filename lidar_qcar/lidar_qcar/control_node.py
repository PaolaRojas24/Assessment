#!/usr/bin/env python3

"""
-- Command for simulated qcar --

ros2 run lidar_qcar straight_publisher --ros-args -p is_sim:=true

"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool


class StraightPublisher(Node):

    def __init__(self):
        super().__init__('straight_publisher')

        self.declare_parameter('is_sim', False)
        is_sim = self.get_parameter('is_sim').get_parameter_value().bool_value

        topic = '/qcar_sim/user_command' if is_sim else '/qcar/user_command'
        self.get_logger().info(f'Publishing to: {topic}')

        self.pub = self.create_publisher(Vector3Stamped, topic, 10)

        # ── Suscripción al flag de obstáculo ─────────────────────────────────
        self._obstacle_detected = False
        self.create_subscription(
            Bool,
            '/qcar/obstacle_detected',
            self._obstacle_callback,
            10
        )

        # 10 Hz
        self.timer = self.create_timer(0.1, self.publish_msg)

    # ── Callback del flag ─────────────────────────────────────────────────────
    def _obstacle_callback(self, msg: Bool):
        if msg.data and not self._obstacle_detected:
            self.get_logger().warn('⛔ Obstáculo detectado — deteniendo el QCar')
        elif not msg.data and self._obstacle_detected:
            self.get_logger().info('✓ Zona despejada — reanudando marcha')
        self._obstacle_detected = msg.data

    def publish_msg(self):

        msg = Vector3Stamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'command_input'

        if self._obstacle_detected:
            # Parada completa
            msg.vector.x = 0.0
            msg.vector.y = 0.0
        else:
            # Forward velocity
            msg.vector.x = 0.05
            # No steering
            msg.vector.y = 0.0

        # Unused
        msg.vector.z = 0.0

        self.pub.publish(msg)

        self.get_logger().info(
            f'x={msg.vector.x:.3f}  y={msg.vector.y:.3f}'
        )


def main():

    rclpy.init()

    node = StraightPublisher()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
