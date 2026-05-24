import time
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Vector3Stamped
import math

class test_control(Node):
    def __init__(self):
        super().__init__('test_control')

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
        msg.vector.x = 0.045
        msg.vector.y = 0.3 * math.sin(self.t)
        msg.vector.z = 0.0

        self.pub.publish(msg)
        self.get_logger().info(f'x={msg.vector.x:.3f}  y={msg.vector.y:.4f}')
        self.t += 0.1

    def publish_stop(self):
        msg = Vector3Stamped()
        msg.header.frame_id = 'command_input'
        msg.vector.x = 0.0
        msg.vector.y = 0.0
        msg.vector.z = 0.0
        for _ in range(10):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            time.sleep(0.02)
        self.get_logger().info('shutdown: published zero velocity/direction')

def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = test_control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            try:
                node.publish_stop()
            except Exception as e:
                node.get_logger().error(f'failed to publish stop: {e}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()