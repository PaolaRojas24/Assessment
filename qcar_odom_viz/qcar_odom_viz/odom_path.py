#!/usr/bin/env python3
"""
Suscribe /odom y:
  - Publica nav_msgs/Path con la traza acumulada (/odom_path)
  - Publica el TF odom → base_link (necesario para RViz)
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class OdomPath(Node):
    def __init__(self):
        super().__init__('odom_path')

        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('min_dist', 0.02)
        self.declare_parameter('max_points', 5000)

        self.frame_id       = self.get_parameter('frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value
        self.min_dist       = self.get_parameter('min_dist').get_parameter_value().double_value
        self.max_points     = self.get_parameter('max_points').get_parameter_value().integer_value

        self.path = Path()
        self.path.header.frame_id = self.frame_id
        self.last_xy = None

        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.pub_path = self.create_publisher(Path, '/odom_path', 10)

        self.get_logger().info(
            f'odom_path listo — suscrito a /odom\n'
            f'  TF: {self.frame_id} → {self.child_frame_id}\n'
            f'  publica /odom_path'
        )

    def _odom_cb(self, msg: Odometry):
        # --- TF odom → base_link ---
        t = TransformStamped()
        t.header.stamp    = msg.header.stamp
        t.header.frame_id = self.frame_id
        t.child_frame_id  = self.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation      = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)

        # --- Path acumulado ---
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_xy is not None:
            dx = x - self.last_xy[0]
            dy = y - self.last_xy[1]
            if (dx * dx + dy * dy) ** 0.5 < self.min_dist:
                return

        self.last_xy = (x, y)

        pose = PoseStamped()
        pose.header.stamp    = msg.header.stamp
        pose.header.frame_id = self.frame_id
        pose.pose            = msg.pose.pose
        self.path.poses.append(pose)

        if len(self.path.poses) > self.max_points:
            self.path.poses = self.path.poses[-self.max_points:]

        self.path.header.stamp = msg.header.stamp
        self.pub_path.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
