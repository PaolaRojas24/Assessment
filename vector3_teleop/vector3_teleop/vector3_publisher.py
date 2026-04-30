
""" 
-- Command for simulated qcar --

ros2 run vector3_teleop publisher --ros-args -p is_sim:=true

"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
import math

class Vector3Publisher(Node):
    def __init__(self):
        super().__init__('vector3_publisher')

        self.declare_parameter('is_sim', False)
        is_sim = self.get_parameter('is_sim').get_parameter_value().bool_value

        topic = '/qcar_sim/user_command' if is_sim else '/qcar/user_command'
        self.get_logger().info(f'Publishing to: {topic}')

        self.pub = self.create_publisher(Vector3Stamped, topic, 10)
        self.timer = self.create_timer(0.1, self.publish_msg)  # 10 Hz
        self.t = 0.0

    def publish_msg(self):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'command_input'
        msg.vector.x = 0.45
        msg.vector.y = 0.3 * math.sin(self.t)
        msg.vector.z = 0.0

        self.pub.publish(msg)
        self.get_logger().info(f'x={msg.vector.x:.2f}  y={msg.vector.y:.4f}')
        self.t += 0.1

def main():
    rclpy.init()
    node = Vector3Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()