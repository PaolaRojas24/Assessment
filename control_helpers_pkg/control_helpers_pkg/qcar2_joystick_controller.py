import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from qcar2_interfaces.msg import MotorCommands
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class Qcar2JoyControl(Node):
    def __init__(self):
        super().__init__('qcar2_joystick_controller')


        self.declare_parameter('publish_topic', '/qcar2_motor_speed_cmd')
        self.declare_parameter('max_steering_angle', 0.5236)
        self.declare_parameter('max_linear_vel', 4.458883)
        
        # 2. Get the parameter value
        publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.max_steering_angle_ = self.get_parameter('max_steering_angle').get_parameter_value().double_value
        self.max_linear_vel_ = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.linear_vel_ = 0.0
        self.steering_angle_= 0.0

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscriber_ = self.create_subscription(Joy, '/joy', self.key_listener_callback, qos_profile)
        self.twist_publisher_ = self.create_publisher(MotorCommands, publish_topic, qos_profile)

        self.timer_ = self.create_timer(0.01, self.timer_callback)

        self.get_logger().info(f'Node successfully started Publishing to topic: {publish_topic}.\nLeft stick controls angular velocity, RT(R2) controls linear velocity, and button A(X) the linear velocity direction\n\
                               For reduced speed mode (1/10th) keep pressed LB(L1) button.')

    def key_listener_callback(self, msg):
        self.steering_angle_= msg.axes[0] * self.max_steering_angle_
        if msg.buttons[4]:
            self.linear_vel_ = abs((1. - msg.axes[5])) * self.max_linear_vel_/10
        else:
            self.linear_vel_ = abs((1. - msg.axes[5])) * self.max_linear_vel_

        if msg.buttons[0]:
            self.linear_vel_ *= -1

    def timer_callback(self):
        msg = MotorCommands()
        msg.motor_names = ["steering_angle", "motor_throttle"]
        msg.values = [self.steering_angle_, self.linear_vel_]

        self.twist_publisher_.publish(msg)

    
def main(args=None):
    rclpy.init(args=args)
    joystick_controller = Qcar2JoyControl()
    rclpy.spin(joystick_controller)
    joystick_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()