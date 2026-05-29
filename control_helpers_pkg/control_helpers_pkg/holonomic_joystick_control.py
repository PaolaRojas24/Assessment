import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class JoyControl(Node):
    def __init__(self):
        super().__init__('holonomic_joystick_controller')


        self.declare_parameter('publish_topic', '/cmd_vel')
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('max_linear_vel', 2.5)
        
        # 2. Get the parameter value
        publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.max_angular_vel_ = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.max_linear_vel_ = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.linear_vel_ = 0.0
        self.angular_vel_ = 0.0

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscriber_ = self.create_subscription(Joy, '/joy', self.key_listener_callback, qos_profile)
        self.twist_publisher_ = self.create_publisher(Twist, publish_topic, qos_profile)

        self.timer_ = self.create_timer(0.01, self.timer_callback)

        self.get_logger().info(f'Node successfully started Publishing to topic: {publish_topic}.\nLeft stick controls angular velocity, RT controls linear velocity, and button 0 the linear velocity direction')

    def key_listener_callback(self, msg):
        self.angular_vel_ = msg.axes[0] * self.max_angular_vel_
        self.linear_vel_ = abs((1. - msg.axes[5])) * self.max_linear_vel_

        if msg.buttons[0]:
            self.linear_vel_ *= -1

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.linear_vel_
        msg.angular.z = self.angular_vel_

        self.twist_publisher_.publish(msg)

    
def main(args=None):
    rclpy.init(args=args)
    joystick_controller = JoyControl()
    rclpy.spin(joystick_controller)
    joystick_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()