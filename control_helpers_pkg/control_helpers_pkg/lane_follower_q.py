import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from qcar2_interfaces.msg import MotorCommands
from geometry_msgs.msg import Vector3Stamped
import math

class LaneFollowerQ(Node):
    def __init__(self):
        super().__init__('lane_follower_q')

        self.declare_parameter('platform', 'qcar2')
        self.declare_parameter('target_point_topic', '/lane_target_point_m')
        self.declare_parameter('cmd_topic', '/qcar2_motor_speed_cmd')
        self.declare_parameter('wheelbase', 0.256)
        self.declare_parameter('lookahead_min', 0.16)
        self.declare_parameter('lookahead_max', 0.34)
        self.declare_parameter('lookahead_base', 0.20)
        self.declare_parameter('lookahead_speed_gain', 0.80)
        self.declare_parameter('max_steering_angle', 0.5)
        self.declare_parameter('curve_threshold', 0.2)
        self.declare_parameter('speed_straight', 0.15)
        self.declare_parameter('speed_curve', 0.0775)
        self.declare_parameter('steering_sign', -1.0)

        target_topic = self.get_parameter('target_point_topic').value
        cmd_topic = self.get_parameter('cmd_topic').value

        self.platform = self.get_parameter('platform').value
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.lookahead_min = float(self.get_parameter('lookahead_min').value)
        self.lookahead_max = float(self.get_parameter('lookahead_max').value)
        self.lookahead_base = float(self.get_parameter('lookahead_base').value)
        self.lookahead_speed_gain = float(self.get_parameter('lookahead_speed_gain').value)
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)
        self.curve_threshold = float(self.get_parameter('curve_threshold').value)
        self.speed_straight = float(self.get_parameter('speed_straight').value)
        self.speed_curve = float(self.get_parameter('speed_curve').value)
        self.steering_sign = float(self.get_parameter('steering_sign').value)

        self.last_speed_command = 0.0

        self.subscription = self.create_subscription(
            Float32MultiArray, target_topic, self.target_callback, 10)
        
        if self.platform == 'qcar':
            self.cmd_pub = self.create_publisher(
                Vector3Stamped, cmd_topic, 10)
        elif self.platform == 'qcar2':
            self.cmd_pub = self.create_publisher(
                MotorCommands, cmd_topic, 10)

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def publish_stop(self):
        if self.platform == 'qcar2':
            cmd = MotorCommands()
            cmd.motor_names = ["steering_angle", "motor_throttle"]
            cmd.values = [0.0, 0.0]
            self.cmd_pub.publish(cmd)
            self.last_speed_command = 0.0
        elif self.platform == 'qcar':
            cmd = Vector3Stamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.vector.x = 0.0  # throttle
            cmd.vector.y = 0.0  # steering
            self.cmd_pub.publish(cmd)
            self.last_speed_command = 0.0

    def target_callback(self, msg):
        data = msg.data
        if len(data) < 2:
            self.publish_stop()
            return

        target_x = float(data[0])
        target_y = float(data[1])

        if target_y <= 0.0:
            self.publish_stop()
            return

        # Raw geometric distance from rear axle to the selected target point.
        lookahead_raw = math.hypot(target_x, target_y)
        lookahead_speed = self.clamp(
            self.lookahead_base + self.lookahead_speed_gain * abs(self.last_speed_command),
            self.lookahead_min,
            self.lookahead_max,
        )
        lookahead = self.clamp(min(lookahead_raw, lookahead_speed), self.lookahead_min, self.lookahead_max)

        # Pure Pursuit using rear-axle frame: x lateral, y forward.
        steering_angle = math.atan2(2.0 * self.wheelbase * target_x, lookahead * lookahead)
        steering_angle = self.clamp(steering_angle, -self.max_steering_angle, self.max_steering_angle)

        if abs(steering_angle) > self.curve_threshold:
            speed = self.speed_curve
        else:
            speed = self.speed_straight

        if self.platform == 'qcar':
            cmd = Vector3Stamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.vector.x = speed  # throttle
            cmd.vector.y = self.steering_sign * steering_angle  # steering
            self.cmd_pub.publish(cmd)
            self.last_speed_command = speed
        elif self.platform == 'qcar2':
            cmd = MotorCommands()
            cmd.motor_names = ["steering_angle", "motor_throttle"]
            cmd.values = [self.steering_sign * steering_angle, speed]
            self.cmd_pub.publish(cmd)

            self.last_speed_command = speed

def main(args=None):
    rclpy.init(args=args)
    lane_follower_q = LaneFollowerQ()
    rclpy.spin(lane_follower_q)
    lane_follower_q.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()