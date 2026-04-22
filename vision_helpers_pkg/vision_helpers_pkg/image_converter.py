import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy, QoSDurabilityPolicy

class ImageConverter(Node):
    def __init__(self):
        super().__init__('image_converter')

        # 1. Declare the parameter with a default value
        self.declare_parameter('subscribe_topic', '/qcar/csi_front')
        self.declare_parameter('publish_topic', '/qcar/decompressed/csi_front')
        # 2. Get the parameter value
        sub_topic_name = self.get_parameter('subscribe_topic').get_parameter_value().string_value
        pub_topic_name = self.get_parameter("publish_topic").get_parameter_value().string_value
        self.get_logger().info(f'Subscribing to CompressedImage topic: {sub_topic_name}, and publishing to Image topic: {pub_topic_name}')

        self.qcar_qos_profile = QoSProfile(
				reliability   = QoSReliabilityPolicy.BEST_EFFORT,
				history 	  = QoSHistoryPolicy.KEEP_LAST,
				durability    = QoSDurabilityPolicy.VOLATILE,
				depth 		  = 10)
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            CompressedImage,
            sub_topic_name,
            self.compressed_image_callback,
            self.qcar_qos_profile
        )
        self.publisher = self.create_publisher(Image, pub_topic_name, self.qcar_qos_profile)

    def compressed_image_callback(self, msg):
        try:
            # Convert CompressedImage to OpenCV image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            # Convert OpenCV image to Image message
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            # rotated_image = cv2.rotate(cv_image, cv2.ROTATE_180)
            # image_msg = self.bridge.cv2_to_imgmsg(rotated_image, "bgr8")
            self.publisher.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

def main(args=None):
    rclpy.init(args=args)
    image_converter = ImageConverter()
    rclpy.spin(image_converter)
    image_converter.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()