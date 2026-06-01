"""
Lane detector for the QCar line follower — BEV-first detection.

Pipeline:
  1. Preprocess (WB + CLAHE) on perspective frame
  2. HLS colour segmentation → binary mask (perspective)
  3. Morphological cleaning
  4. Apply ROI, then warpPerspective → BEV binary mask
  5. Optional Canny on BEV grayscale AND-ed with BEV mask
  6. HoughLinesP in BEV space (lines are straight/near-straight)
  7. Classify lines by X position (left half / right half of BEV)
  8. Fit x = f(y) per lane (robust for near-vertical lines)
  9. Kalman smoothing of target pixel (BEV coords)
 10. Direct BEV-pixel → metric conversion (no perspectiveTransform on target)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import numpy as np
import cv2


class _TargetKalman:
    """Constant-velocity Kalman filter on the (u, v) target pixel."""

    def __init__(self):
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.transitionMatrix = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ], dtype=np.float32)
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=np.float32)
        self.set_noise(3.0, 10.0)
        self.reset()

    def set_noise(self, q, r):
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * float(max(1e-3, q))
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * float(max(1e-3, r))

    def reset(self):
        self.initialized = False
        self.predict_count = 0
        self.kf.statePost = np.zeros((4, 1), dtype=np.float32)
        self.kf.errorCovPost = np.eye(4, dtype=np.float32) * 100.0

    def step(self, measurement, gate_px=0, max_predict=30):
        if not self.initialized:
            if measurement is None:
                return None, False
            u, v = measurement
            self.kf.statePost = np.array(
                [[float(u)], [float(v)], [0.0], [0.0]], dtype=np.float32)
            self.initialized = True
            self.predict_count = 0
            return (int(u), int(v)), True

        pred = self.kf.predict()
        pu, pv = float(pred[0]), float(pred[1])

        if measurement is None:
            self.predict_count += 1
            if self.predict_count > max_predict:
                self.initialized = False
                return None, False
            return (int(pu), int(pv)), False

        u, v = measurement
        if gate_px > 0:
            dist = float(np.hypot(u - pu, v - pv))
            if dist > gate_px:
                self.predict_count += 1
                if self.predict_count > max_predict:
                    self.initialized = False
                    return None, False
                return (int(pu), int(pv)), False

        meas = np.array([[float(u)], [float(v)]], dtype=np.float32)
        corr = self.kf.correct(meas)
        self.predict_count = 0
        return (int(corr[0]), int(corr[1])), True


class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')

        # ---- Topics ----
        subscribe_topic = '/qcar/decompressed/csi_front'
        target_topic    = '/lane_target_point_m'
        overlay_topic   = '/qcar/line_follower/overlay'
        bev_topic       = '/qcar/line_follower/bev'
        mask_topic      = '/qcar/line_follower/mask'

        self.enable_display = False
        overlay_publish_hz  = 15.0
        self.overlay_size   = (320, 240)
        self.overlay_period_frames = max(1, int(round(15.0 / overlay_publish_hz)))

        # ---- Camera intrinsics ----
        self.hfov = 160.0
        self.vfov = 120.0
        self.K = None
        self._last_frame_shape = None

        # ---- BEV geometry (calibrated for 820x410 source) ----
        # Physical area: 0.434 m wide × 0.104 m deep → 217×52 px at 500 px/m.
        # Near edge (close to car) = y=0 in BEV image.
        # Far edge (ahead of car)  = y=52 in BEV image.
        self.bev_pixels_per_meter = 500.0
        self.bev_dst_points_m = np.array([
            0.0,   0.0,
            0.0,   0.103908,
            0.434, 0.103908,
            0.434, 0.0,
        ], dtype=np.float32).reshape((4, 2))

        # ROI trapezoid in the perspective image (bottom-left, top-left,
        # top-right, bottom-right). Maps onto the BEV destination above.
        self.roi_polygon_points_px = np.array([
            0.0,   410.0,
            20.0,  239.0,
            780.0, 239.0,
            820.0, 410.0,
        ], dtype=np.float32).reshape((4, 2))

        # ---- Physical car geometry ----
        self.camera_to_rear_axle_forward_m = 0.323
        self.camera_to_rear_axle_lateral_m = 0.0

        # ---- Lane tracking state ----
        self.lane_half_width_px         = 100.0  # 20 cm at 500 px/m — adjust to half the actual lane width
        self.lane_half_width_ema_alpha   = 0.2
        self.dynamic_lane_half_width_px  = self.lane_half_width_px
        # Bias = 0: follow the detected line exactly.
        # Tune this if the robot should run left or right of the yellow line.
        self.lane_lateral_bias           = 0.0

        # ---- Hough — tuned for BEV (217×52 px) ----
        self.hough_threshold       = 10
        self.hough_min_line_length = 8
        self.hough_max_line_gap    = 5

        # ---- Preprocess ----
        self.wb_enable  = True
        self.clahe_clip = 2.0
        self.clahe_tile = 8

        # ---- HLS colour thresholds ----
        # White: tighter than before to avoid walls — only bright floor markings.
        # Increase gray_lower[1] (lightness) if walls still appear.
        self.gray_lower   = np.array([0,   190, 0],  dtype=np.uint8)
        self.gray_upper   = np.array([179, 255, 40],  dtype=np.uint8)
        self.yellow_lower = np.array([4,    19, 50],  dtype=np.uint8)
        self.yellow_upper = np.array([35,  225, 255], dtype=np.uint8)

        # ---- Mask morphology ----
        self.mask_median_k = 5
        self.mask_open_k   = 3
        self.mask_close_k  = 5

        # ---- Canny (applied on BEV grayscale) ----
        self.canny_low  = 14
        self.canny_high = 75

        # ---- Kalman ----
        self.kalman_enable      = True
        self.kalman_q           = 3.0
        self.kalman_r           = 10.0
        self.kalman_gate_px     = 80
        self.kalman_max_predict = 30
        self.kf_target = _TargetKalman()
        self.kf_target.set_noise(self.kalman_q, self.kalman_r)

        # ---- Target-selection state ----
        self.last_detected_lane = 1
        self.last_target_pixel  = None
        self.max_waiting_cycles = 500
        self.waiting_cycles     = 0
        self._frame_count       = 0

        bev_w_m = float(self.bev_dst_points_m[:, 0].max() -
                        self.bev_dst_points_m[:, 0].min())
        bev_h_m = float(self.bev_dst_points_m[:, 1].max() -
                        self.bev_dst_points_m[:, 1].min())
        self.bev_size = (
            max(1, int(np.ceil(bev_w_m * self.bev_pixels_per_meter))),
            max(1, int(np.ceil(bev_h_m * self.bev_pixels_per_meter))),
        )

        qos_in = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        qos_out = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        self.subscription     = self.create_subscription(
            Image, subscribe_topic, self.listener_callback, qos_in)
        self.lane_pub         = self.create_publisher(
            Float32MultiArray, '/lane_lines', 10)
        self.target_point_pub = self.create_publisher(
            Float32MultiArray, target_topic, 10)
        self.overlay_pub      = self.create_publisher(Image, overlay_topic, qos_out)
        self.bev_pub          = self.create_publisher(Image, bev_topic,     qos_out)
        self.mask_pub         = self.create_publisher(Image, mask_topic,    qos_out)

        self.get_logger().info(
            f'lane_detector (BEV-first)\n'
            f'  sub     : {subscribe_topic}\n'
            f'  bev_size: {self.bev_size[0]}×{self.bev_size[1]} px '
            f'({bev_w_m:.2f}m × {bev_h_m:.2f}m @ {self.bev_pixels_per_meter:.0f} px/m)\n'
            f'  display : {self.enable_display}'
        )

    # ── helpers ────────────────────────────────────────────────────────────

    def _build_intrinsics(self, w, h):
        fx = (w / 2.0) / np.tan(np.radians(self.hfov / 2.0))
        fy = (h / 2.0) / np.tan(np.radians(self.vfov / 2.0))
        self.K = np.array([[fx, 0.0, w / 2.0],
                           [0.0, fy, h / 2.0],
                           [0.0, 0.0, 1.0]])
        self._last_frame_shape = (h, w)

    def _preprocess(self, frame):
        img = frame
        if self.wb_enable:
            f = img.astype(np.float32)
            means = f.reshape(-1, 3).mean(axis=0)
            avg = float(means.mean())
            if avg > 1.0:
                gains = np.clip(avg / np.maximum(means, 1.0), 0.5, 2.0)
                f *= gains
            img = np.clip(f, 0, 255).astype(np.uint8)
        if self.clahe_clip > 0.05:
            lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
            clahe = cv2.createCLAHE(clipLimit=self.clahe_clip,
                                    tileGridSize=(self.clahe_tile, self.clahe_tile))
            lab[:, :, 0] = clahe.apply(lab[:, :, 0])
            img = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        return img

    def _clean_mask(self, mask):
        k = self.mask_median_k
        if k >= 3:
            if k % 2 == 0:
                k += 1
            mask = cv2.medianBlur(mask, k)
        if self.mask_open_k >= 2:
            kk = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (self.mask_open_k, self.mask_open_k))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kk)
        if self.mask_close_k >= 2:
            kk = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (self.mask_close_k, self.mask_close_k))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kk)
        return mask

    def color_segment(self, hls, lower, upper):
        mask = cv2.inRange(hls, lower, upper)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        return cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

    def get_homography_matrix(self, polygon):
        src = polygon[0].astype(np.float32)
        dst = self.bev_dst_points_m * self.bev_pixels_per_meter
        return cv2.getPerspectiveTransform(src, dst)

    # ── BEV-aware lane fitting ──────────────────────────────────────────────

    def lane_average_bev(self, bev_w, bev_h, lines):
        """Classify Hough lines by X position and fit x = f(y) for BEV.

        Line format: [x_far, y_far, x_target, y_target]
          - (x_far, y_far)       = endpoint at far edge of BEV (y = bev_h-1)
          - (x_target, y_target) = lookahead point at 60 % BEV depth
        """
        left_pts  = []
        right_pts = []
        mid_x = bev_w / 2.0

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if abs(y2 - y1) < 5:          # reject near-horizontal
                    continue
                avg_x = (x1 + x2) / 2.0
                if avg_x < mid_x:
                    left_pts.extend([(y1, x1), (y2, x2)])
                else:
                    right_pts.extend([(y1, x1), (y2, x2)])

        def fit(pts):
            if len(pts) < 2:
                return None
            ys, xs = zip(*pts)
            try:
                a, b = np.polyfit(ys, xs, 1)   # x = a·y + b
            except Exception:
                return None
            y_far    = bev_h - 1
            y_target = int(bev_h * 0.7)
            lim = 2 * bev_w
            x_far    = int(np.clip(a * y_far    + b, -lim, lim))
            x_target = int(np.clip(a * y_target + b, -lim, lim))
            return [x_far, y_far, x_target, y_target]

        left_line  = fit(left_pts)
        right_line = fit(right_pts)
        center_line = None
        if left_line is not None and right_line is not None:
            center_line = [
                int((left_line[0]  + right_line[0])  / 2), left_line[1],
                int((left_line[2]  + right_line[2])  / 2), left_line[3],
            ]
        return [left_line, right_line, center_line]

    def update_lane_half_width(self, left_line, right_line):
        if left_line is None or right_line is None:
            return
        half_w = 0.5 * abs(float(right_line[2] - left_line[2]))
        if half_w <= 1.0:
            return
        a = float(np.clip(self.lane_half_width_ema_alpha, 0.0, 1.0))
        self.dynamic_lane_half_width_px = (
            a * half_w + (1.0 - a) * self.dynamic_lane_half_width_px)

    def select_target_pixel(self, left_line, right_line, center_line):
        bias   = self.lane_lateral_bias * self.last_detected_lane
        half_w = self.dynamic_lane_half_width_px

        if self.waiting_cycles > self.max_waiting_cycles:
            return None

        if center_line is not None:
            self.last_detected_lane = 1
            self.last_target_pixel  = [
                int(center_line[2] - bias * half_w), center_line[3]]
            self.waiting_cycles = 0
        elif left_line is not None:
            self.last_detected_lane = 0.5
            self.last_target_pixel  = [
                int(left_line[2] + half_w * (1.0 - bias)), left_line[3]]
            self.waiting_cycles = 0
        elif right_line is not None:
            self.last_detected_lane = -0.75
            self.last_target_pixel  = [
                int(right_line[2] - half_w * (1.0 + bias)), right_line[3]]
            self.waiting_cycles = 0
        else:
            self.waiting_cycles += 1

        return self.last_target_pixel

    def _bev_pixel_to_m(self, target_pixel):
        """Convert BEV pixel (u, v) → metric rear-axle coordinates (no perspectiveTransform)."""
        u, v = float(target_pixel[0]), float(target_pixel[1])
        x_m = u / self.bev_pixels_per_meter
        y_m = v / self.bev_pixels_per_meter

        bev_xs   = self.bev_dst_points_m[:, 0]
        bev_ys   = self.bev_dst_points_m[:, 1]
        center_x = float((bev_xs.min() + bev_xs.max()) / 2.0)
        max_y    = float(bev_ys.max())

        lateral = x_m - center_x
        forward = float(np.clip(y_m, 0.0, max_y))
        return (lateral + self.camera_to_rear_axle_lateral_m,
                forward + self.camera_to_rear_axle_forward_m)

    # ── image helpers ──────────────────────────────────────────────────────

    def _decode(self, data: Image):
        if data.encoding != 'bgr8':
            arr = np.frombuffer(data.data, dtype=np.uint8)
            if data.encoding == 'rgb8':
                return cv2.cvtColor(
                    arr.reshape((data.height, data.width, 3)), cv2.COLOR_RGB2BGR)
            return arr.reshape((data.height, data.width, -1))
        return np.frombuffer(data.data, dtype=np.uint8).reshape(
            (data.height, data.width, 3))

    def _publish_image(self, publisher, image, resize_to=None):
        try:
            if resize_to is not None and (image.shape[1], image.shape[0]) != resize_to:
                image = cv2.resize(image, resize_to, interpolation=cv2.INTER_AREA)
            out = Image()
            out.header.stamp = self.get_clock().now().to_msg()
            out.height       = image.shape[0]
            out.width        = image.shape[1]
            out.encoding     = 'bgr8'
            out.is_bigendian = 0
            out.step         = image.shape[1] * 3
            out.data         = image.tobytes()
            publisher.publish(out)
        except Exception as e:
            self.get_logger().warning(f'image publish failed: {e}')

    @staticmethod
    def _safe_pt(x, y):
        return (int(np.clip(x, -1_000_000, 1_000_000)),
                int(np.clip(y, -1_000_000, 1_000_000)))

    # ── main callback ──────────────────────────────────────────────────────

    def listener_callback(self, data):
        try:
            src = self._decode(data)
            if src is None or src.size == 0:
                return

            h, w = src.shape[:2]
            if self._last_frame_shape != (h, w):
                self._build_intrinsics(w, h)

            polygon  = self.roi_polygon_points_px.astype(np.int32).reshape((1, 4, 2))
            h_matrix = self.get_homography_matrix(polygon)

            # 1. Preprocess on full-resolution perspective frame
            pp = self._preprocess(src)

            # 2. Warp preprocessed BGR image to BEV (before segmentation).
            #    Warping the color image gives much better quality than warping
            #    a binary mask: no fragmentation at the far edge.
            bev_color = cv2.warpPerspective(pp, h_matrix, self.bev_size)

            # 3. Colour segmentation in BEV space
            hls_bev     = cv2.cvtColor(bev_color, cv2.COLOR_BGR2HLS)
            white_mask  = self.color_segment(hls_bev, self.gray_lower,   self.gray_upper)
            yellow_mask = self.color_segment(hls_bev, self.yellow_lower, self.yellow_upper)
            bev_mask    = self._clean_mask(cv2.bitwise_or(white_mask, yellow_mask))

            # Keep perspective masks for the colour overlay visualization
            hls_persp        = cv2.cvtColor(pp, cv2.COLOR_BGR2HLS)
            white_mask_persp  = self.color_segment(hls_persp, self.gray_lower,   self.gray_upper)
            yellow_mask_persp = self.color_segment(hls_persp, self.yellow_lower, self.yellow_upper)

            # 4. Optional Canny on BEV grayscale AND-ed with BEV mask
            if self.canny_low > 0 and self.canny_high > self.canny_low:
                bev_gray  = cv2.cvtColor(bev_color, cv2.COLOR_BGR2GRAY)
                bev_gray  = cv2.GaussianBlur(bev_gray, (5, 5), 1.4)
                bev_edges = cv2.Canny(bev_gray, self.canny_low, self.canny_high)
                hough_in  = cv2.bitwise_and(bev_edges, bev_mask)
            else:
                hough_in = bev_mask

            # 6. Hough in BEV space
            bev_h, bev_w = hough_in.shape[:2]
            lines = cv2.HoughLinesP(
                hough_in, 1, np.pi / 180,
                threshold=self.hough_threshold,
                minLineLength=self.hough_min_line_length,
                maxLineGap=self.hough_max_line_gap,
            )

            # 7. Lane fitting (BEV-aware: classify by X, fit x = f(y))
            left_line, right_line, center_line = self.lane_average_bev(bev_w, bev_h, lines)
            self.update_lane_half_width(left_line, right_line)
            raw_target = self.select_target_pixel(left_line, right_line, center_line)

            # 8. Kalman smoothing (coordinates in BEV pixels)
            target_vel = None
            if self.kalman_enable:
                meas     = tuple(raw_target) if raw_target is not None else None
                filtered, _ = self.kf_target.step(
                    meas,
                    gate_px=self.kalman_gate_px,
                    max_predict=self.kalman_max_predict,
                )
                target_px = list(filtered) if filtered is not None else None
                if target_px is not None and self.kf_target.initialized:
                    state    = self.kf_target.kf.statePost
                    target_vel = (float(state[2, 0]), float(state[3, 0]))
            else:
                target_px = raw_target

            # 9. BEV pixel → metric (direct, no perspectiveTransform)
            target_msg = Float32MultiArray()
            if target_px is not None:
                tx, ty = self._bev_pixel_to_m(target_px)
                target_msg.data = [float(tx), float(ty)]
            else:
                target_msg.data = [-1.0, -1.0]
            self.target_point_pub.publish(target_msg)

            # 10. Lane lines message
            msg_lines       = Float32MultiArray()
            left_data  = [float(x) for x in left_line]  if left_line  else [-1.0] * 4
            right_data = [float(x) for x in right_line] if right_line else [-1.0] * 4
            msg_lines.data = left_data + right_data
            self.lane_pub.publish(msg_lines)

            # 11. Visualization
            self._frame_count += 1
            need_overlay = (self._frame_count % self.overlay_period_frames) == 0
            if not (self.enable_display or need_overlay):
                return

            # BEV overlay: draw detected lines and target
            bev_bgr = cv2.cvtColor(bev_mask, cv2.COLOR_GRAY2BGR)
            if left_line is not None:
                cv2.line(bev_bgr,
                         self._safe_pt(left_line[0],  left_line[1]),
                         self._safe_pt(left_line[2],  left_line[3]),
                         (255, 0, 0), 2)
            if right_line is not None:
                cv2.line(bev_bgr,
                         self._safe_pt(right_line[0], right_line[1]),
                         self._safe_pt(right_line[2], right_line[3]),
                         (0, 0, 255), 2)
            if center_line is not None:
                cv2.line(bev_bgr,
                         self._safe_pt(center_line[0], center_line[1]),
                         self._safe_pt(center_line[2], center_line[3]),
                         (0, 255, 0), 2)
            if target_px is not None:
                cv2.circle(bev_bgr, self._safe_pt(target_px[0], target_px[1]),
                           5, (0, 255, 255), -1)
                cv2.putText(bev_bgr,
                            f"x={target_msg.data[0]:.2f} y={target_msg.data[1]:.2f}",
                            (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                if target_vel is not None:
                    du, dv = target_vel
                    if np.hypot(du, dv) > 0.5:
                        ax = target_px[0] + du * 8.0
                        ay = target_px[1] + dv * 8.0
                        cv2.arrowedLine(bev_bgr,
                                        self._safe_pt(target_px[0], target_px[1]),
                                        self._safe_pt(ax, ay),
                                        (0, 200, 255), 2, tipLength=0.3)

            # Perspective overlay: original frame + ROI polygon
            line_image = src.copy()
            cv2.polylines(line_image, polygon, isClosed=True,
                          color=(255, 255, 0), thickness=1)

            if need_overlay:
                # Flip BEV for human readability (near at bottom, far at top)
                bev_display = cv2.flip(bev_bgr, 0)

                color_mask = np.zeros((h, w, 3), dtype=np.uint8)
                color_mask[white_mask_persp  > 0] = (255, 255, 255)
                color_mask[yellow_mask_persp > 0] = (0, 255, 255)
                cv2.polylines(color_mask, polygon, isClosed=True,
                              color=(255, 255, 0), thickness=1)

                self._publish_image(self.overlay_pub, line_image,  resize_to=self.overlay_size)
                self._publish_image(self.bev_pub,     bev_display)
                self._publish_image(self.mask_pub,    color_mask,  resize_to=self.overlay_size)

            if self.enable_display:
                cv2.imshow('Original + ROI', line_image)
                cv2.imshow('BEV detections', cv2.flip(bev_bgr, 0))
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'processing error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
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
