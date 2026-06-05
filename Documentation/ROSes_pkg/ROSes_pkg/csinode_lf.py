#!/usr/bin/env python3
"""
Line-follower CSI publisher for the QCar.

Captures from the front CSI camera at a known-good native mode
(default 820x410 @ 30 Hz), then downscales in software to the
target resolution before JPEG-encoding and publishing. This is
required because the Jetson/Quanser CSI pipeline only accepts a
limited set of (resolution, frame-rate) combinations; asking the
sensor directly for 320x240 fails with
"The video format is not supported".

Bandwidth wins still apply -- the wire payload is the downscaled
JPEG, not the captured frame.

QoS profile matches the stock csinode (RELIABLE + TRANSIENT_LOCAL
+ depth=1) so the `ros2 topic hz` discovery keepalive still works.

Run on the QCar:
    ros2 run ROSes_pkg csi_lf
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy, QoSDurabilityPolicy

import numpy as np
import cv2

from pal.utilities.vision import Camera2D
from sensor_msgs.msg import CompressedImage


class CSINodeLF(Node):
    def __init__(self):
        super().__init__('csi_node_lf')

        # --- Parameters -----------------------------------------------------
        # Capture-side: must be a mode the Quanser CSI pipeline supports.
        # The stock launcher uses 820x410 @ 120 Hz -- the GStreamer pipeline
        # underneath Camera2D rejects every other combination tried so far
        # (e.g. 820x410 @ 30 fails with "format not supported"). The 120 Hz
        # capture rate is sensor-side only; we still publish at publish_freq.
        self.declare_parameter('capture_width', 820)
        self.declare_parameter('capture_height', 410)
        self.declare_parameter('capture_freq', 120)

        # Output-side: published JPEG size after software downscale.
        # 320x240 cuts on-the-wire bytes ~7x vs. capture native.
        self.declare_parameter('output_width', 320)
        self.declare_parameter('output_height', 240)

        # Publish rate. Set <= capture_freq. If lower, we drop frames in
        # software to avoid backlogging the sensor pull.
        self.declare_parameter('publish_freq', 15.0)

        self.declare_parameter('jpeg_quality', 60)
        self.declare_parameter('topic', '/qcar/csi_front')
        # cameraId is wiring-dependent and DIFFERS BETWEEN CARS.
        #   Stock csinode (most cars): csi_front -> id "3"
        #   csinode_redpatch (red car): csi_front -> id "2"
        # Default here matches the red car (the launcher we ship is
        # qcar_red_lf.launch.py). Override for other colours.
        self.declare_parameter('camera_id', '2')

        cap_w = int(self.get_parameter('capture_width').value)
        cap_h = int(self.get_parameter('capture_height').value)
        cap_f = int(self.get_parameter('capture_freq').value)
        out_w = int(self.get_parameter('output_width').value)
        out_h = int(self.get_parameter('output_height').value)
        pub_f = float(self.get_parameter('publish_freq').value)
        jpeg_q = int(self.get_parameter('jpeg_quality').value)
        topic = str(self.get_parameter('topic').value)
        cam_id = str(self.get_parameter('camera_id').value)

        self._capture_size = (cap_w, cap_h)
        self._output_size = (out_w, out_h)
        self._needs_resize = (out_w, out_h) != (cap_w, cap_h)
        self._encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_q]

        # --- ROS plumbing ---------------------------------------------------
        self.qcar_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.publisher = self.create_publisher(
            CompressedImage, topic, self.qcar_qos_profile
        )

        # --- Camera ---------------------------------------------------------
        try:
            self.camera = Camera2D(
                cameraId=cam_id,
                frameRate=cap_f,
                frameWidth=cap_w,
                frameHeight=cap_h,
            )
        except Exception as e:
            self.get_logger().error(
                'Camera2D init failed for camera_id={} {}x{} @ {} Hz: {}'.format(
                    cam_id, cap_w, cap_h, cap_f, e
                )
            )
            raise

        # Quanser's wrapper silently fails to set `self.capture` when the
        # requested mode is unsupported. Catch that here with a clearer error
        # than the eventual AttributeError on read().
        if not hasattr(self.camera, 'capture'):
            raise RuntimeError(
                "Camera2D opened but has no 'capture' attribute -- the "
                "Quanser pipeline rejected capture={}x{} @ {} Hz on cam id {}. "
                "Pick a native mode (e.g. 820x410 @ 30) and downscale via "
                "output_width/output_height.".format(cap_w, cap_h, cap_f, cam_id)
            )

        # Publish timer. If pub_f < cap_f the sensor will queue frames
        # internally; that's fine because we only ever pull the latest.
        self.timer = self.create_timer(1.0 / pub_f, self._tick)

        self.get_logger().info(
            'csi_node_lf: capture {}x{} @ {} Hz -> publish {}x{} @ {} Hz '
            'on {} (JPEG q={})'.format(
                cap_w, cap_h, cap_f, out_w, out_h, pub_f, topic, jpeg_q
            )
        )

    def _tick(self):
        try:
            self.camera.read()
        except Exception as e:
            self.get_logger().warning('camera read failed: {}'.format(e))
            return

        frame = self.camera.imageData
        if frame is None or getattr(frame, 'size', 0) == 0:
            return

        if self._needs_resize:
            frame = cv2.resize(
                frame, self._output_size, interpolation=cv2.INTER_AREA
            )

        ok, buf = cv2.imencode('.jpg', frame, self._encode_params)
        if not ok:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'cam_img_input4'
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        self.publisher.publish(msg)

    def stop_csi(self):
        self.get_logger().info('stopping front CSI...')
        try:
            self.camera.terminate()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CSINodeLF()
    except Exception as e:
        print('csi_lf failed to start: {}'.format(e))
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_csi()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
