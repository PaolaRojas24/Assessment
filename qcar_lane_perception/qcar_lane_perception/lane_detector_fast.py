"""
Low-latency lane detector for the QCar line follower.

Patched copy of vision_helpers_pkg/lane_detector.py kept inside
line_follower_real so that movilidad_inteligente stays untouched.

Differences vs. the original:
  - Subscribes BEST_EFFORT + VOLATILE + depth=1 directly to the decompressed
    topic, so we don't need image_relay in this pipeline. DDS drops stale
    frames automatically when the callback can't keep up.
  - cv2.imshow / waitKey is gated by `enable_display` (default False). On
    the headless deploy these two calls were each costing ~10-20 ms per
    frame, dominating callback time.
  - Publishes a small annotated image (default 320x240) at a low rate on a
    separate BEST_EFFORT topic for rqt_image_view, so the visualizer never
    competes with the control loop.
  - Camera intrinsics are rebuilt from the actual frame size at runtime.
    All pixel-domain params in the YAML must be expressed at the same
    resolution the QCar is publishing (e.g. 320x240 with csinode_lf).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import numpy as np
import cv2


class LaneDetectorFast(Node):
    def __init__(self):
        super().__init__('lane_detector_fast')

        self.declare_parameter('subscribe_topic', '/qcar/decompressed/csi_front')
        self.declare_parameter('target_point_topic', '/lane_target_point_m')
        self.declare_parameter('enable_display', False)
        self.declare_parameter('overlay_topic', '/qcar/line_follower/overlay')
        self.declare_parameter('bev_topic', '/qcar/line_follower/bev')
        self.declare_parameter('mask_topic', '/qcar/line_follower/mask')
        self.declare_parameter('overlay_publish_hz', 5.0)
        self.declare_parameter('overlay_width', 320)
        self.declare_parameter('overlay_height', 240)

        self.declare_parameter('hfov_deg', 160.0)
        self.declare_parameter('vfov_deg', 120.0)

        self.declare_parameter('bev_pixels_per_meter', 500.0)
        self.declare_parameter('bev_dst_points_m', [
            0.0, 0.0,
            0.0, 0.103908,
            0.434, 0.103908,
            0.434, 0.0,
        ])
        self.declare_parameter('roi_polygon_points_px', [
            0.0, 240.0,
            105.0, 140.0,
            210.0, 140.0,
            320.0, 240.0,
        ])
        self.declare_parameter('camera_to_rear_axle_forward_m', 0.323)
        self.declare_parameter('camera_to_rear_axle_lateral_m', 0.0)
        self.declare_parameter('lane_half_width_px', 19.0)
        self.declare_parameter('lane_half_width_ema_alpha', 0.2)
        self.declare_parameter('lane_lateral_bias', 0.0)

        self.declare_parameter('hough_threshold', 25)
        self.declare_parameter('hough_min_line_length', 20)
        self.declare_parameter('hough_max_line_gap', 12)

        topic_name = self.get_parameter('subscribe_topic').value
        target_topic = self.get_parameter('target_point_topic').value
        overlay_topic = self.get_parameter('overlay_topic').value
        bev_topic = self.get_parameter('bev_topic').value
        mask_topic = self.get_parameter('mask_topic').value

        self.enable_display = bool(self.get_parameter('enable_display').value)
        self.overlay_period_frames = max(
            1,
            int(round(15.0 / float(self.get_parameter('overlay_publish_hz').value)))
        )
        self.overlay_size = (
            int(self.get_parameter('overlay_width').value),
            int(self.get_parameter('overlay_height').value),
        )

        self.hfov = float(self.get_parameter('hfov_deg').value)
        self.vfov = float(self.get_parameter('vfov_deg').value)
        self.K = None
        self._last_frame_shape = None

        self.bev_pixels_per_meter = float(self.get_parameter('bev_pixels_per_meter').value)

        self.bev_dst_points_m = np.array(
            self.get_parameter('bev_dst_points_m').value, dtype=np.float32
        ).reshape((4, 2))
        self.roi_polygon_points_px = np.array(
            self.get_parameter('roi_polygon_points_px').value, dtype=np.float32
        ).reshape((4, 2))

        self.camera_to_rear_axle_forward_m = float(self.get_parameter('camera_to_rear_axle_forward_m').value)
        self.camera_to_rear_axle_lateral_m = float(self.get_parameter('camera_to_rear_axle_lateral_m').value)
        self.lane_half_width_px = float(self.get_parameter('lane_half_width_px').value)
        self.lane_half_width_ema_alpha = float(self.get_parameter('lane_half_width_ema_alpha').value)
        self.dynamic_lane_half_width_px = self.lane_half_width_px
        self.lane_lateral_bias = float(np.clip(self.get_parameter('lane_lateral_bias').value, -0.9, 0.9))

        self.hough_threshold = int(self.get_parameter('hough_threshold').value)
        self.hough_min_line_length = int(self.get_parameter('hough_min_line_length').value)
        self.hough_max_line_gap = int(self.get_parameter('hough_max_line_gap').value)

        self.last_detected_lane = 1
        self.last_target_pixel = None
        self.max_waiting_cycles = 500
        self.waiting_cycles = 0
        self._frame_count = 0

        bev_width_m = float(np.max(self.bev_dst_points_m[:, 0]) - np.min(self.bev_dst_points_m[:, 0]))
        bev_height_m = float(np.max(self.bev_dst_points_m[:, 1]) - np.min(self.bev_dst_points_m[:, 1]))
        self.bev_size = (
            max(1, int(np.ceil(bev_width_m * self.bev_pixels_per_meter))),
            max(1, int(np.ceil(bev_height_m * self.bev_pixels_per_meter))),
        )

        self.gray_lower = np.array([30, 160, 0], dtype=np.uint8)
        self.gray_upper = np.array([180, 200, 40], dtype=np.uint8)
        self.yellow_lower = np.array([15, 30, 115], dtype=np.uint8)
        self.yellow_upper = np.array([35, 204, 255], dtype=np.uint8)

        qos_in = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        qos_overlay = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        self.subscription = self.create_subscription(
            Image, topic_name, self.listener_callback, qos_in
        )
        self.lane_pub = self.create_publisher(Float32MultiArray, '/lane_lines', 10)
        self.target_point_pub = self.create_publisher(Float32MultiArray, target_topic, 10)
        self.overlay_pub = self.create_publisher(Image, overlay_topic, qos_overlay)
        self.bev_pub = self.create_publisher(Image, bev_topic, qos_overlay)
        self.mask_pub = self.create_publisher(Image, mask_topic, qos_overlay)

        self.get_logger().info(
            f'lane_detector_fast: sub={topic_name} (BE,d=1)  '
            f'overlay={overlay_topic} / bev={bev_topic} / mask={mask_topic} '
            f'(BE,d=1, every {self.overlay_period_frames} frames)  '
            f'display={self.enable_display}'
        )

    def _build_intrinsics(self, w, h):
        fx = (w / 2.0) / np.tan(np.radians(self.hfov / 2.0))
        fy = (h / 2.0) / np.tan(np.radians(self.vfov / 2.0))
        self.K = np.array([[fx, 0.0, w / 2.0],
                           [0.0, fy, h / 2.0],
                           [0.0, 0.0, 1.0]])
        self._last_frame_shape = (h, w)

    def lane_average(self, image, lines):
        left_fits = []
        right_fits = []

        if lines is None:
            return [None, None, None]

        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x1 == x2:
                continue
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope, intersect = parameters[0], parameters[1]
            if slope < 0:
                left_fits.append((slope, intersect))
            else:
                right_fits.append((slope, intersect))

        left_avg = np.average(left_fits, axis=0) if left_fits else None
        right_avg = np.average(right_fits, axis=0) if right_fits else None
        left_line = self.point_generator(image, left_avg)
        right_line = self.point_generator(image, right_avg)

        center_line = None
        if left_line is not None and right_line is not None:
            center_line = [
                int((left_line[0] + right_line[0]) / 2),
                left_line[1],
                int((left_line[2] + right_line[2]) / 2),
                left_line[3],
            ]
        return [left_line, right_line, center_line]

    def point_generator(self, image, fit):
        if fit is None:
            return None
        m, b = fit
        # Reject nearly-horizontal fits. A real lane stripe seen from a
        # forward-facing camera projects with |slope| well above 0; when |m|
        # drops near zero the extrapolation x = (y - b) / m explodes
        # (saw -1.77e13 in the field), which then crashes cv2.line / circle
        # because the integer parameter no longer fits in a C int. Drop the
        # fit instead; downstream code already handles "no line found".
        if not np.isfinite(m) or abs(m) < 0.1:
            return None
        y1 = image.shape[0]
        y2 = int(y1 * 0.56)
        x1 = (y1 - b) / m
        x2 = (y2 - b) / m
        # Defensive reject: even with |m| > 0.1, b can be huge after polyfit
        # of noisy points. Anything more than 3x the image width away is
        # garbage for our purposes.
        img_w = image.shape[1]
        if (not np.isfinite(x1) or not np.isfinite(x2)
                or abs(x1) > 3 * img_w or abs(x2) > 3 * img_w):
            return None
        return [int(x1), int(y1), int(x2), int(y2)]

    def color_segment(self, hls, lower_range, upper_range):
        mask_in_range = cv2.inRange(hls, lower_range, upper_range)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        return cv2.morphologyEx(mask_in_range, cv2.MORPH_DILATE, kernel)

    def get_homography_matrix(self, polygon):
        src_points = polygon[0].astype(np.float32)
        dst_points_px = self.bev_dst_points_m * self.bev_pixels_per_meter
        return cv2.getPerspectiveTransform(src_points, dst_points_px)

    def update_lane_half_width(self, left_line, right_line):
        if left_line is None or right_line is None:
            return
        measured_half_width_px = 0.5 * abs(float(right_line[2] - left_line[2]))
        if measured_half_width_px <= 1.0:
            return
        alpha = float(np.clip(self.lane_half_width_ema_alpha, 0.0, 1.0))
        self.dynamic_lane_half_width_px = (
            alpha * measured_half_width_px
            + (1.0 - alpha) * self.dynamic_lane_half_width_px
        )

    def select_target_pixel(self, left_line, right_line, center_line):
        bias = self.lane_lateral_bias * self.last_detected_lane
        half_w = self.dynamic_lane_half_width_px

        if self.waiting_cycles > self.max_waiting_cycles:
            return None

        if center_line is not None:
            self.last_detected_lane = 1
            self.last_target_pixel = [int(center_line[2] - bias * half_w), center_line[3]]
            self.waiting_cycles = 0
        elif left_line is not None:
            self.last_detected_lane = 0.5
            self.last_target_pixel = [int(left_line[2] + half_w * (1.0 - bias)), left_line[3]]
            self.waiting_cycles = 0
        elif right_line is not None:
            self.last_detected_lane = -0.75
            self.last_target_pixel = [int(right_line[2] - half_w * (1.0 + bias)), right_line[3]]
            self.waiting_cycles = 0
        else:
            self.waiting_cycles += 1

        return self.last_target_pixel

    def target_to_rear_axle_m(self, target_pixel, homography_matrix):
        target_pixel_np = np.array(
            [[[float(target_pixel[0]), float(target_pixel[1])]]],
            dtype=np.float32,
        )
        target_bev_px = cv2.perspectiveTransform(target_pixel_np, homography_matrix)[0][0]
        target_bev_m = target_bev_px / self.bev_pixels_per_meter

        left_x = 0.5 * (self.bev_dst_points_m[0][0] + self.bev_dst_points_m[1][0])
        right_x = 0.5 * (self.bev_dst_points_m[2][0] + self.bev_dst_points_m[3][0])
        center_x = 0.5 * (left_x + right_x)

        near_y = 0.5 * (self.bev_dst_points_m[0][1] + self.bev_dst_points_m[3][1])
        far_y = 0.5 * (self.bev_dst_points_m[1][1] + self.bev_dst_points_m[2][1])
        max_visible_forward = abs(far_y - near_y)

        lateral_camera = float(target_bev_m[0] - center_x)
        forward_camera = float(np.clip(abs(target_bev_m[1] - near_y), 0.0, max_visible_forward))

        target_x_rear = lateral_camera + self.camera_to_rear_axle_lateral_m
        target_y_rear = forward_camera + self.camera_to_rear_axle_forward_m
        return target_x_rear, target_y_rear

    def _decode(self, data: Image):
        # Manual decode for bgr8: skip cv_bridge.
        if data.encoding != 'bgr8':
            # Fallback: shape inference still works for rgb8, mono8 won't.
            arr = np.frombuffer(data.data, dtype=np.uint8)
            if data.encoding == 'rgb8':
                return cv2.cvtColor(
                    arr.reshape((data.height, data.width, 3)),
                    cv2.COLOR_RGB2BGR,
                )
            return arr.reshape((data.height, data.width, -1))
        return np.frombuffer(data.data, dtype=np.uint8).reshape(
            (data.height, data.width, 3)
        )

    @staticmethod
    def _safe_pt(x, y):
        """Clamp drawing coords to a range that fits comfortably in C int.
        cv2 drawing functions happily accept out-of-image coordinates and
        clip what they actually rasterize, but they crash if the integer
        won't fit in a C int. 1e6 is far outside any image and well inside
        the 2^31 limit."""
        return (int(np.clip(x, -1_000_000, 1_000_000)),
                int(np.clip(y, -1_000_000, 1_000_000)))

    def _publish_image(self, publisher, image, resize_to=None):
        # Publish unconditionally. We previously gated on
        # `get_subscription_count() == 0`, but a brief DDS discovery hiccup
        # would make that read 0 transiently and visually freeze the
        # downstream viewer until the next reconnect. The cost of an
        # unwatched publish at 5 Hz is negligible.
        try:
            if resize_to is not None and (image.shape[1], image.shape[0]) != resize_to:
                image = cv2.resize(image, resize_to, interpolation=cv2.INTER_AREA)
            out = Image()
            out.header.stamp = self.get_clock().now().to_msg()
            out.height = image.shape[0]
            out.width = image.shape[1]
            out.encoding = 'bgr8'
            out.is_bigendian = 0
            out.step = image.shape[1] * 3
            out.data = image.tobytes()
            publisher.publish(out)
        except Exception as e:
            self.get_logger().warning(f'image publish failed: {e}')

    def listener_callback(self, data):
        try:
            current_frame = self._decode(data)
            if current_frame is None or current_frame.size == 0:
                return

            h, w = current_frame.shape[:2]
            if self._last_frame_shape != (h, w):
                self._build_intrinsics(w, h)

            src = current_frame
            poligon = self.roi_polygon_points_px.astype(np.int32).reshape((1, 4, 2))
            h_matrix = self.get_homography_matrix(poligon)
            hls = cv2.cvtColor(src, cv2.COLOR_BGR2HLS)
            white_mask = self.color_segment(hls, self.gray_lower, self.gray_upper)
            yellow_mask = self.color_segment(hls, self.yellow_lower, self.yellow_upper)

            masked_colors = cv2.bitwise_or(white_mask, yellow_mask)
            roi_mask = np.zeros_like(masked_colors)
            cv2.fillPoly(roi_mask, poligon, 255)
            masked_edges = cv2.bitwise_and(masked_colors, roi_mask)

            lines = cv2.HoughLinesP(
                masked_edges, 1, np.pi / 180,
                threshold=self.hough_threshold,
                minLineLength=self.hough_min_line_length,
                maxLineGap=self.hough_max_line_gap,
            )

            left_line, right_line, center_line = self.lane_average(src, lines)
            self.update_lane_half_width(left_line, right_line)
            target_pixel = self.select_target_pixel(left_line, right_line, center_line)

            target_msg = Float32MultiArray()
            if target_pixel is not None:
                target_x_rear, target_y_rear = self.target_to_rear_axle_m(target_pixel, h_matrix)
                target_msg.data = [float(target_x_rear), float(target_y_rear)]
            else:
                target_msg.data = [-1.0, -1.0]
            self.target_point_pub.publish(target_msg)

            msg_lines = Float32MultiArray()
            left_data = [float(x) for x in left_line] if left_line is not None else [-1.0] * 4
            right_data = [float(x) for x in right_line] if right_line is not None else [-1.0] * 4
            msg_lines.data = left_data + right_data
            self.lane_pub.publish(msg_lines)

            self._frame_count += 1
            need_overlay = (self._frame_count % self.overlay_period_frames) == 0
            if not (self.enable_display or need_overlay):
                return

            line_image = src.copy()
            if left_line is not None:
                cv2.line(line_image,
                         self._safe_pt(left_line[0], left_line[1]),
                         self._safe_pt(left_line[2], left_line[3]),
                         (255, 0, 0), 3)
            if right_line is not None:
                cv2.line(line_image,
                         self._safe_pt(right_line[0], right_line[1]),
                         self._safe_pt(right_line[2], right_line[3]),
                         (0, 0, 255), 3)
            if center_line is not None:
                cv2.line(line_image,
                         self._safe_pt(center_line[0], center_line[1]),
                         self._safe_pt(center_line[2], center_line[3]),
                         (0, 255, 0), 3)
            if target_pixel is not None:
                cv2.circle(line_image,
                           self._safe_pt(target_pixel[0], target_pixel[1]),
                           4, (0, 255, 255), -1)
                tx, ty = target_msg.data[0], target_msg.data[1]
                cv2.putText(
                    line_image, f"x={tx:.2f} y={ty:.2f}",
                    (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (0, 255, 255), 1,
                )
            cv2.polylines(line_image, poligon, isClosed=True,
                          color=(255, 255, 0), thickness=1)

            bev_image = None
            if need_overlay or self.enable_display:
                bev_image = cv2.warpPerspective(line_image, h_matrix, self.bev_size)
                # Flip vertically so the car is at the bottom and the road
                # goes up the image. The bev_dst_points_m mapping puts the
                # "near" edge at y=0 in pixel space (which is the TOP of the
                # cv2 image), which reads upside down to a human viewer.
                # Geometry math uses the metric output, not this rendered
                # image, so flipping only affects display.
                bev_image = cv2.flip(bev_image, 0)

            if need_overlay:
                # Color-coded mask: white pixels stay white, yellow pixels
                # stay yellow, everything else black. Useful for tuning
                # the HLS thresholds.
                color_mask = np.zeros((h, w, 3), dtype=np.uint8)
                color_mask[white_mask > 0] = (255, 255, 255)
                color_mask[yellow_mask > 0] = (0, 255, 255)  # BGR yellow
                # Draw the ROI outline on the mask too, so the user can
                # see what slice of pixels actually feeds HoughLinesP.
                cv2.polylines(color_mask, poligon, isClosed=True,
                              color=(255, 255, 0), thickness=1)

                self._publish_image(
                    self.overlay_pub, line_image, resize_to=self.overlay_size
                )
                if bev_image is not None:
                    self._publish_image(self.bev_pub, bev_image)
                # Resize the mask to overlay_size too. Native 820x410x3 = ~1 MB
                # per frame; at 15 Hz that's ~15 MB/s of local DDS traffic on
                # one topic, which is enough to backpressure the publisher and
                # stall the whole listener_callback. Downstream consumers
                # (dashboard, hz keepalive) only need it at display resolution.
                self._publish_image(
                    self.mask_pub, color_mask, resize_to=self.overlay_size
                )

            if self.enable_display:
                cv2.imshow("Detections", line_image)
                if bev_image is not None:
                    cv2.imshow("BEV Detections", bev_image)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'processing error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorFast()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if node.enable_display:
            cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
