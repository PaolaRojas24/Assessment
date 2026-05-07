import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


class ImageRelay(Node):
    """
    Bridges BEST_EFFORT (img_converter) → RELIABLE (lane_detector) and throttles
    the frame rate so the detector never builds a stale-frame backlog.

    Why this is needed:
      - img_converter publishes BEST_EFFORT; lane_detector subscribes RELIABLE.
        DDS drops every message when policies mismatch.
      - The camera runs ~30 fps but lane_detector (cv2.imshow + BEV warp) is slower.
        With queue depth=10, up to 10 stale frames pile up → ~300-600 ms lag.
      - Subscribing with depth=1 here keeps only the newest frame in the relay buffer.
      - Publishing at throttle_hz (default 15) limits the input rate to the detector,
        so it always processes a recent frame instead of catching up with old ones.
    """

    def __init__(self):
        super().__init__('image_relay')

        self.declare_parameter('input_topic',  '/qcar/decompressed/csi_front')
        self.declare_parameter('output_topic', '/qcar/reliable/csi_front')
        self.declare_parameter('throttle_hz',  15.0)

        input_topic  = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        throttle_hz  = float(self.get_parameter('throttle_hz').value)

        # depth=1: drop every frame except the latest so we never send stale data
        qos_be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        self._latest: Image | None = None

        # Subscriber just stores the latest frame; timer decides when to publish
        self.sub = self.create_subscription(Image, input_topic, self._store_cb, qos_be)
        # depth=1 on publisher side too: don't buffer unsent frames
        self.pub = self.create_publisher(Image, output_topic, 1)

        self._timer = self.create_timer(1.0 / throttle_hz, self._publish_latest)

        self.get_logger().info(
            f'ImageRelay: {input_topic} → {output_topic}  '
            f'(BEST_EFFORT → RELIABLE, {throttle_hz:.0f} Hz)'
        )

    def _store_cb(self, msg: Image):
        self._latest = msg

    def _publish_latest(self):
        if self._latest is not None:
            self.pub.publish(self._latest)
            self._latest = None


def main(args=None):
    rclpy.init(args=args)
    node = ImageRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
