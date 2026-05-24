"""
QCar lap recorder.

Subscribes to a camera topic (real QCar CompressedImage by default) and
writes the stream to ~/Desktop/qcar_laps_video/lap_NNN.mp4. The lap
number is auto-incremented from whatever is already on disk so
consecutive runs do not overwrite previous recordings.

Ctrl-C cleanly finalises the video file before exit.

Parameters:
  topic        camera topic to subscribe to (default /qcar/csi_front)
  compressed   True if the topic is sensor_msgs/CompressedImage,
               False if it is sensor_msgs/Image
  output_dir   directory to save laps in
  fps          frame rate written into the video container
  codec        FOURCC codec, e.g. mp4v, XVID, MJPG
  extension    file extension matching the codec (e.g. .mp4, .avi)
"""

import os
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from rclpy.signals import SignalHandlerOptions

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

import cv2


DEFAULT_OUTPUT_DIR = os.path.expanduser('~/Desktop/qcar_laps_video')


class LapRecorder(Node):
    def __init__(self):
        super().__init__('lap_recorder')

        self.declare_parameter('topic', '/qcar/csi_front')
        self.declare_parameter('compressed', True)
        self.declare_parameter('output_dir', DEFAULT_OUTPUT_DIR)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('codec', 'mp4v')
        self.declare_parameter('extension', '.mp4')

        topic = self.get_parameter('topic').get_parameter_value().string_value
        self._compressed = self.get_parameter('compressed').get_parameter_value().bool_value
        self._out_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self._fps = float(self.get_parameter('fps').get_parameter_value().double_value)
        self._codec = self.get_parameter('codec').get_parameter_value().string_value
        self._ext = self.get_parameter('extension').get_parameter_value().string_value
        if not self._ext.startswith('.'):
            self._ext = '.' + self._ext

        os.makedirs(self._out_dir, exist_ok=True)
        self._out_path = self._next_lap_path()

        # Match the QoS the real QCar camera publishes with (best-effort).
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.bridge = CvBridge()
        if self._compressed:
            self.create_subscription(CompressedImage, topic, self._on_compressed, qos)
        else:
            self.create_subscription(Image, topic, self._on_raw, qos)

        self._writer = None
        self._frame_count = 0

        self.get_logger().info(
            'lap_recorder ready\n'
            f'  topic     : {topic} '
            f'({"CompressedImage" if self._compressed else "Image"})\n'
            f'  output    : {self._out_path}\n'
            f'  fps/codec : {self._fps:.0f} / {self._codec}\n'
        )

    def _next_lap_path(self):
        pattern = re.compile(r'^lap_(\d+)' + re.escape(self._ext) + r'$')
        max_n = 0
        for name in os.listdir(self._out_dir):
            m = pattern.match(name)
            if m:
                n = int(m.group(1))
                if n > max_n:
                    max_n = n
        return os.path.join(self._out_dir, f'lap_{max_n + 1:03d}{self._ext}')

    def _ensure_writer(self, frame):
        if self._writer is not None:
            return
        h, w = frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*self._codec)
        writer = cv2.VideoWriter(self._out_path, fourcc, self._fps, (w, h))
        if not writer.isOpened():
            self.get_logger().error(
                f'failed to open VideoWriter for {self._out_path} '
                f'(codec={self._codec}, size={w}x{h}, fps={self._fps})'
            )
            return
        self._writer = writer
        self.get_logger().info(f'recording to {self._out_path} ({w}x{h})')

    def _on_compressed(self, msg: CompressedImage):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge decode failed: {e}')
            return
        self._write_frame(frame)

    def _on_raw(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge decode failed: {e}')
            return
        self._write_frame(frame)

    def _write_frame(self, frame):
        self._ensure_writer(frame)
        if self._writer is None:
            return
        self._writer.write(frame)
        self._frame_count += 1
        if self._frame_count % 60 == 0:
            self.get_logger().info(f'frames written: {self._frame_count}')

    def close(self):
        if self._writer is not None:
            self._writer.release()
            self._writer = None
            self.get_logger().info(
                f'saved {self._frame_count} frames to {self._out_path}'
            )


def main():
    # Disable rclpy's default SIGINT handler so we can release the
    # VideoWriter (and write the mp4 trailer) before the process exits.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = LapRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
