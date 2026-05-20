"""
Low-latency CompressedImage -> Image decoder for the QCar line follower.

Patched copy of vision_helpers_pkg/image_converter.py kept inside
line_follower_real so that movilidad_inteligente stays untouched.

Differences vs. the original:
  - Subscribe / publish queue depth = 1 instead of 10. The camera runs at
    ~15 Hz with our patched csinode_lf, and the detector callback can be
    slower than that, so any queue >1 just buffers stale frames.
  - Decode straight with cv2.imdecode and build the Image message manually,
    avoiding the cv_bridge two-step (decode -> cv2 -> re-encode as Image).

QoS stays BEST_EFFORT + VOLATILE: the QCar publisher offers RELIABLE +
TRANSIENT_LOCAL, which is compatible with a weaker subscriber.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
import numpy as np
import cv2


class ImageConverterFast(Node):
    def __init__(self):
        super().__init__('image_converter_fast')

        self.declare_parameter('subscribe_topic', '/qcar/csi_front')
        self.declare_parameter('publish_topic', '/qcar/decompressed/csi_front')

        sub_topic = self.get_parameter('subscribe_topic').value
        pub_topic = self.get_parameter('publish_topic').value

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        self.subscription = self.create_subscription(
            CompressedImage, sub_topic, self._cb, qos
        )
        self.publisher = self.create_publisher(Image, pub_topic, qos)

        self.get_logger().info(
            f'image_converter_fast: {sub_topic} -> {pub_topic} '
            f'(BEST_EFFORT, depth=1)'
        )

    def _cb(self, msg: CompressedImage):
        try:
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if img is None:
                return

            out = Image()
            out.header = msg.header
            out.height = img.shape[0]
            out.width = img.shape[1]
            out.encoding = 'bgr8'
            out.is_bigendian = 0
            out.step = img.shape[1] * 3
            out.data = img.tobytes()
            self.publisher.publish(out)
        except Exception as e:
            self.get_logger().error(f'decode error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageConverterFast()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
