"""
Lane detector for the QCar line follower.

QoS:
  - subscribe BEST_EFFORT + VOLATILE + depth=1 directly to the decompressed
    topic, so DDS drops stale frames when the callback can't keep up.
  - overlay/bev/mask publishers are BEST_EFFORT + depth=1 too, so the
    visualizer never competes with the control loop.

cv2.imshow is gated by the `enable_display` flag (default False). On the
headless deploy each imshow+waitKey costs ~10-20 ms per frame.
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
    """Constant-velocity Kalman filter on the (u, v) target pixel.

    State = [u, v, du, dv]^T, measurement = [u, v]^T. Smooths frame-to-frame
    jitter and, when the lane is briefly lost, coasts on the last velocity
    estimate for up to `max_predict` frames instead of freezing or jumping.
    """

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
        """Run one predict (+ optional correct) step.

        measurement: (u, v) or None.
        Returns ((u, v) filtered, used_measurement_bool), or (None, False)
        if the track is uninitialized or has been lost too long.
        """
        if not self.initialized:
            if measurement is None:
                return None, False
            u, v = measurement
            self.kf.statePost = np.array([[float(u)], [float(v)], [0.0], [0.0]],
                                         dtype=np.float32)
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

        # ---- Visualization ----
        # enable_display opens cv2.imshow windows locally. Leave False when
        # driving; use rqt_image_view on /qcar/line_follower/overlay instead.
        self.enable_display = False
        # Overlay publish rate target. Camera runs at ~15 Hz,
        # one published overlay per processed frame (keeps DDS pipe warm).
        overlay_publish_hz  = 15.0
        self.overlay_size   = (320, 240)
        self.overlay_period_frames = max(1, int(round(15.0 / overlay_publish_hz)))

        # ---- Camera intrinsics (rebuilt at runtime from frame size) ----
        self.hfov = 160.0
        self.vfov = 120.0
        self.K = None
        self._last_frame_shape = None

        # ---- BEV / ROI geometry (tuned for 820x410, csinode_lf native) ----
        self.bev_pixels_per_meter = 500.0
        self.bev_dst_points_m = np.array([
            0.0, 0.0,
            0.0, 0.103908,
            0.434, 0.103908,
            0.434, 0.0,
        ], dtype=np.float32).reshape((4, 2))
        # ROI trapezoid: bottom-left, top-left, top-right, bottom-right.
        # Wider top edge than the previous yaml so the perception sees more
        # of the lane horizon (tuned in line_perception_offline.py).
        self.roi_polygon_points_px = np.array([
            0.0,   410.0,
            20.0,  290.0, #old value 239
            780.0, 290.0, #old value 239
            820.0, 410.0,
        ], dtype=np.float32).reshape((4, 2))

        # ---- Physical car geometry ----ros2 launch ROSes_pkg qcar_red_lf.launch.py nodes:='qcar,rgbd,csi_lf,imu_external,odom_kalman,lidar_qos,command'
        self.camera_to_rear_axle_forward_m = 0.323
        self.camera_to_rear_axle_lateral_m = 0.0

        # ---- Lane width tracking ----
        self.lane_half_width_px = 50.0
        self.lane_half_width_ema_alpha = 0.2
        self.dynamic_lane_half_width_px = self.lane_half_width_px
        self.lane_lateral_bias = float(np.clip(0.25, -0.9, 0.9))
        # Yellow dominates direction: the white line only sets the lateral
        # offset (half-lane width). Reject white measurements that imply a
        # sudden width change beyond this ratio -- e.g. a fork where the white
        # peels off right while the yellow goes straight/left -- so the offset
        # stays bounded and the yellow keeps steering. 1.0 = freeze on first
        # estimate, higher = more permissive.
        self.lane_width_max_ratio = 1.5

        # ---- Hough (tuned at 820x410) ----
        self.hough_threshold       = 50
        self.hough_min_line_length = 50
        self.hough_max_line_gap    = 25

        # ---- Preprocess: gray-world WB + CLAHE on L of LAB ----
        self.wb_enable  = True
        self.clahe_clip = 0 #20 / 10.0
        self.clahe_tile = 8

        # ---- HLS color thresholds (offline-tuned) ----
        self.gray_lower   = np.array([0,   111, 0],  dtype=np.uint8)
        self.gray_upper   = np.array([179, 230, 65], dtype=np.uint8)
        self.yellow_lower = np.array([4,   19,  50], dtype=np.uint8) # old 4 19 50
        self.yellow_upper = np.array([35,  225, 255], dtype=np.uint8) # old 35 225 255

        # ---- Yellow-left guard ----
        # Keep the stable slope-based left/right split, but never let a YELLOW
        # segment land on the right: the yellow line is always on the robot's
        # left, so a yellow segment is forced into the left group. Yellow is
        # sampled only inside the ROI, so the yellowish damaged camera center
        # (which sits above the ROI) can't trigger it.
        self.yellow_left_guard  = True
        self.yellow_sample_frac = 0.30   # min fraction of a segment on yellow
        # Yellow wins the overlap: the yellow line is bordered by white and the
        # white mask bleeds onto it. Remove yellow (grown by this many px) from
        # the white mask so white keeps only the genuine right line. 0 disables.
        self.yellow_priority_px = 7
        # Master switch: strict color sides (left=yellow, right=white) with the
        # target ANCHORED on the always-solid yellow line, so the car no longer
        # depends on the dashed white line and auto-recovers from the opposite
        # lane. False falls back to the slope split + yellow_left_guard.
        self.color_side_classify = True

        # ---- Mask morphology (median + open + close) ----
        self.mask_median_k = 5 # 5
        self.mask_open_k   = 3 # 3
        self.mask_close_k  = 5 # 5

        # ---- Canny on grayscale, AND-ed with cleaned mask + ROI ----
        self.canny_low  = 14
        self.canny_high = 75

        # ---- Kalman filter on target pixel ----
        self.kalman_enable      = True
        self.kalman_q           = 5
        self.kalman_r           = 10
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

        bev_width_m  = float(np.max(self.bev_dst_points_m[:, 0]) - np.min(self.bev_dst_points_m[:, 0]))
        bev_height_m = float(np.max(self.bev_dst_points_m[:, 1]) - np.min(self.bev_dst_points_m[:, 1]))
        self.bev_size = (
            max(1, int(np.ceil(bev_width_m  * self.bev_pixels_per_meter))),
            max(1, int(np.ceil(bev_height_m * self.bev_pixels_per_meter))),
        )

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

        self.subscription      = self.create_subscription(Image, subscribe_topic, self.listener_callback, qos_in)
        self.lane_pub          = self.create_publisher(Float32MultiArray, '/lane_lines', 10)
        self.target_point_pub  = self.create_publisher(Float32MultiArray, target_topic, 10)
        self.overlay_pub       = self.create_publisher(Image, overlay_topic, qos_overlay)
        self.bev_pub           = self.create_publisher(Image, bev_topic, qos_overlay)
        self.mask_pub          = self.create_publisher(Image, mask_topic, qos_overlay)

        self.get_logger().info(
            f'lane_detector: sub={subscribe_topic} (BE,d=1)  '
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

    def _segment_is_yellow(self, x1, y1, x2, y2, yellow_in_roi):
        """True if enough points sampled along the segment fall on yellow.

        yellow_in_roi must already be AND-ed with the ROI so the yellowish
        camera center (outside the ROI) cannot mislabel a line.
        """
        h, w = yellow_in_roi.shape[:2]
        n = max(2, int(np.hypot(x2 - x1, y2 - y1)))
        xs = np.clip(np.linspace(x1, x2, n).astype(np.int32), 0, w - 1)
        ys = np.clip(np.linspace(y1, y2, n).astype(np.int32), 0, h - 1)
        return float(np.mean(yellow_in_roi[ys, xs] > 0)) >= self.yellow_sample_frac

    def lane_average(self, image, lines, yellow_in_roi=None):
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
            # Yellow-left guard: a yellow segment can never be the right line,
            # so force it into the left group regardless of slope. White
            # segments keep the original, stable slope-based split.
            if (yellow_in_roi is not None
                    and self._segment_is_yellow(x1, y1, x2, y2, yellow_in_roi)):
                left_fits.append((slope, intersect))
            elif slope < 0:
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
        # Reject nearly-horizontal fits: x = (y - b) / m explodes when |m|
        # approaches 0, which overflows the C int that cv2 drawing takes.
        if not np.isfinite(m) or abs(m) < 0.1:
            return None
        y1 = image.shape[0]
        # 0.72 keeps the line's far endpoint near the top of the ROI (y~295)
        # instead of extrapolating to y~230, so the target point isn't aimed
        # far above the detected region (was 0.56 -> turned into curves early).
        y2 = int(y1 * 0.72)
        x1 = (y1 - b) / m
        x2 = (y2 - b) / m
        img_w = image.shape[1]
        if (not np.isfinite(x1) or not np.isfinite(x2)
                or abs(x1) > 3 * img_w or abs(x2) > 3 * img_w):
            return None
        return [int(x1), int(y1), int(x2), int(y2)]

    def color_segment(self, hls, lower_range, upper_range):
        mask_in_range = cv2.inRange(hls, lower_range, upper_range)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        return cv2.morphologyEx(mask_in_range, cv2.MORPH_DILATE, kernel)

    def _preprocess(self, frame):
        """Neutralize warm camera tint (gray-world WB) and equalize lightness (CLAHE)."""
        img = frame
        if self.wb_enable:
            f = img.astype(np.float32)
            means = f.reshape(-1, 3).mean(axis=0)
            avg = float(means.mean())
            if avg > 1.0:
                gains = avg / np.maximum(means, 1.0)
                gains = np.clip(gains, 0.5, 2.0)
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
        """median -> opening -> closing on a binary mask. Each step is optional."""
        k = self.mask_median_k
        if k >= 3:
            if k % 2 == 0:
                k += 1
            mask = cv2.medianBlur(mask, k)
        if self.mask_open_k >= 2:
            kk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                           (self.mask_open_k, self.mask_open_k))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kk)
        if self.mask_close_k >= 2:
            kk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                           (self.mask_close_k, self.mask_close_k))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kk)
        return mask

    def get_homography_matrix(self, polygon):
        src_points = polygon[0].astype(np.float32)
        dst_points_px = self.bev_dst_points_m * self.bev_pixels_per_meter
        return cv2.getPerspectiveTransform(src_points, dst_points_px)

    def update_lane_half_width(self, left_line, right_line):
        if left_line is None or right_line is None:
            return
        # Only learn the half-lane width in the normal configuration (left line
        # genuinely to the left of the right one). When inverted -- e.g. yellow
        # seen on the right in the opposite lane -- this would measure across
        # both lanes, so keep the last good value instead.
        if right_line[2] <= left_line[2]:
            return
        measured_half_width_px = 0.5 * abs(float(right_line[2] - left_line[2]))
        if measured_half_width_px <= 1.0:
            return
        # Reject diverging-white outliers: if the new width is far from the
        # running estimate (a fork), ignore it so the white can't inflate the
        # offset and pull the car off the yellow's direction.
        ref = self.dynamic_lane_half_width_px
        ratio = float(max(1.0, self.lane_width_max_ratio))
        if ref > 1.0 and not (ref / ratio <= measured_half_width_px <= ref * ratio):
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

    def _fit_line_robust(self, image, points):
        """Robust line fit through all segment endpoints (Huber norm).

        cv2.fitLine handles near-vertical lane lines (where averaging polyfit
        slopes blows up) and Huber down-weights stray/outlier segments.
        Returns [x_bottom, y_bottom, x_top, y_top] or None.
        """
        if len(points) < 2:
            return None
        pts = np.asarray(points, dtype=np.float32)
        vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_HUBER, 0, 0.01, 0.01).flatten()
        # Reject near-horizontal fits (|dy/dx| < 0.1), like point_generator did.
        if abs(vy) < 0.1 * abs(vx) or abs(vy) < 1e-6:
            return None
        y_bottom = image.shape[0]
        y_top = int(y_bottom * 0.72)
        x_bottom = x0 + (y_bottom - y0) * vx / vy
        x_top = x0 + (y_top - y0) * vx / vy
        img_w = image.shape[1]
        if (not np.isfinite(x_bottom) or not np.isfinite(x_top)
                or abs(x_bottom) > 3 * img_w or abs(x_top) > 3 * img_w):
            return None
        return [int(x_bottom), int(y_bottom), int(x_top), int(y_top)]

    def _line_yellowness(self, line, yellow_in_roi):
        """Fraction of points sampled along a line that fall on yellow."""
        h, w = yellow_in_roi.shape[:2]
        x1, y1, x2, y2 = line
        n = max(2, int(np.hypot(x2 - x1, y2 - y1)))
        xs = np.clip(np.linspace(x1, x2, n).astype(np.int32), 0, w - 1)
        ys = np.clip(np.linspace(y1, y2, n).astype(np.int32), 0, h - 1)
        return float(np.mean(yellow_in_roi[ys, xs] > 0))

    def lane_by_color(self, image, lines, yellow_in_roi):
        """Build lines from the COMBINED mask, then label each by majority
        colour.

        Segments are grouped left/right by slope and fitted whole, so a white
        reflection in the middle of the yellow line doesn't split it. Each
        fitted line is then sampled: the more-yellow one (if yellow enough)
        becomes the yellow/left reference, the other the white/right line.
        """
        if lines is None:
            return [None, None, None]

        left_pts, right_pts = [], []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x1 == x2:
                continue
            slope = (y2 - y1) / float(x2 - x1)
            if slope < 0:
                left_pts.extend(((x1, y1), (x2, y2)))
            else:
                right_pts.extend(((x1, y1), (x2, y2)))
        line_a = self._fit_line_robust(image, left_pts)
        line_b = self._fit_line_robust(image, right_pts)

        # Label by majority colour: the most-yellow line (above threshold) wins
        # the yellow slot, the other becomes white. If neither is yellow enough,
        # the detected line(s) are treated as white.
        a_f = self._line_yellowness(line_a, yellow_in_roi) if line_a is not None else -1.0
        b_f = self._line_yellowness(line_b, yellow_in_roi) if line_b is not None else -1.0
        yellow_line = white_line = None
        if a_f >= b_f and a_f >= self.yellow_sample_frac:
            yellow_line, white_line = line_a, line_b
        elif b_f > a_f and b_f >= self.yellow_sample_frac:
            yellow_line, white_line = line_b, line_a
        else:
            white_line = line_a if line_a is not None else line_b

        center_line = None
        if yellow_line is not None and white_line is not None:
            center_line = [
                int((yellow_line[0] + white_line[0]) / 2),
                yellow_line[1],
                int((yellow_line[2] + white_line[2]) / 2),
                yellow_line[3],
            ]
        return [yellow_line, white_line, center_line]

    def select_target_anchored_yellow(self, yellow_line, white_line):
        """Target anchored on the always-solid yellow line.

        Places the target ~half a lane to the RIGHT of the yellow line, i.e.
        the yellow stays on the car's left. If the yellow ends up on the right
        (opposite lane), the same offset puts the target further right and
        steers the car back into the correct lane. White is only a fallback
        for frames where the (dashed) white line is all that's visible.
        """
        bias = self.lane_lateral_bias   # >0 hugs slightly toward the yellow
        half_w = self.dynamic_lane_half_width_px

        if self.waiting_cycles > self.max_waiting_cycles:
            return None

        if yellow_line is not None:
            self.last_target_pixel = [int(yellow_line[2] + half_w * (1.0 - bias)),
                                      yellow_line[3]]
            self.waiting_cycles = 0
        elif white_line is not None:
            self.last_target_pixel = [int(white_line[2] - half_w * (1.0 + bias)),
                                      white_line[3]]
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
        """Clamp drawing coords so they fit in a C int (cv2 crashes otherwise)."""
        return (int(np.clip(x, -1_000_000, 1_000_000)),
                int(np.clip(y, -1_000_000, 1_000_000)))

    def _publish_image(self, publisher, image, resize_to=None):
        # Publish unconditionally: gating on get_subscription_count() == 0
        # produced visual freezes during transient DDS discovery hiccups.
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

            # 1. Preprocess: WB + CLAHE to fight the warm camera tint.
            pp = self._preprocess(src)

            # 2. HLS segmentation on the preprocessed frame.
            hls = cv2.cvtColor(pp, cv2.COLOR_BGR2HLS)
            white_mask = self.color_segment(hls, self.gray_lower, self.gray_upper)
            yellow_mask = self.color_segment(hls, self.yellow_lower, self.yellow_upper)

            # 3. Combined mask (yellow OR white), cleaned + ROI. Joined for all
            #    line generation so a white reflection in the middle of the
            #    yellow line doesn't split it -- the line is built whole and
            #    labelled by majority colour afterwards (lane_by_color).
            masked_colors = self._clean_mask(cv2.bitwise_or(white_mask, yellow_mask))
            roi_mask = np.zeros_like(masked_colors)
            cv2.fillPoly(roi_mask, poligon, 255)
            masked_in_roi = cv2.bitwise_and(masked_colors, roi_mask)

            # 4. Hough on the EDGES of the combined MASK (binary), not a
            #    grayscale Canny: the mask boundary is luminance-independent, so
            #    low-contrast yellow survives instead of being gated out.
            hough_input = cv2.Canny(masked_in_roi, 50, 150)

            lines = cv2.HoughLinesP(
                hough_input, 1, np.pi / 180,
                threshold=self.hough_threshold,
                minLineLength=self.hough_min_line_length,
                maxLineGap=self.hough_max_line_gap,
            )

            if self.color_side_classify:
                # Strict color sides + target anchored on the solid yellow line.
                yellow_in_roi = cv2.bitwise_and(yellow_mask, roi_mask)
                left_line, right_line, center_line = self.lane_by_color(
                    src, lines, yellow_in_roi)
                self.update_lane_half_width(left_line, right_line)
                raw_target_pixel = self.select_target_anchored_yellow(
                    left_line, right_line)
            else:
                yellow_in_roi = (cv2.bitwise_and(yellow_mask, roi_mask)
                                 if self.yellow_left_guard else None)
                left_line, right_line, center_line = self.lane_average(
                    src, lines, yellow_in_roi)
                self.update_lane_half_width(left_line, right_line)
                raw_target_pixel = self.select_target_pixel(
                    left_line, right_line, center_line)

            # 6. Kalman smoothing + missing-measurement extrapolation.
            target_velocity_px = None  # (du, dv) en px/frame del estado KF
            if self.kalman_enable:
                meas = (tuple(raw_target_pixel)
                        if raw_target_pixel is not None else None)
                filtered, _ = self.kf_target.step(
                    meas,
                    gate_px=self.kalman_gate_px,
                    max_predict=self.kalman_max_predict,
                )
                target_pixel = (list(filtered) if filtered is not None else None)
                if target_pixel is not None and self.kf_target.initialized:
                    state = self.kf_target.kf.statePost
                    target_velocity_px = (float(state[2, 0]), float(state[3, 0]))
            else:
                target_pixel = raw_target_pixel

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
                # Flecha de heading del Kalman: direccion de (du, dv) del
                # estado KF. Escalado para visibilidad (px/frame suele ser
                # del orden de 1-5, asi que x10 da una flecha legible).
                if target_velocity_px is not None:
                    du, dv = target_velocity_px
                    speed = float(np.hypot(du, dv))
                    if speed > 0.5:  # umbral para no dibujar ruido
                        arrow_scale = 10.0
                        ax = target_pixel[0] + du * arrow_scale
                        ay = target_pixel[1] + dv * arrow_scale
                        cv2.arrowedLine(
                            line_image,
                            self._safe_pt(target_pixel[0], target_pixel[1]),
                            self._safe_pt(ax, ay),
                            (0, 200, 255), 2, tipLength=0.3,
                        )
                        heading_deg = float(np.degrees(np.arctan2(-dv, du)))
                        cv2.putText(
                            line_image, f"hdg={heading_deg:+.1f}deg",
                            (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                            (0, 200, 255), 1,
                        )
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
                # Flip vertically: bev_dst_points_m puts the "near" edge at
                # y=0 (top of the cv2 image), which reads upside down to a
                # human. Geometry math uses the metric output, not this view.
                bev_image = cv2.flip(bev_image, 0)

            if need_overlay:
                color_mask = np.zeros((h, w, 3), dtype=np.uint8)
                color_mask[white_mask > 0] = (255, 255, 255)
                color_mask[yellow_mask > 0] = (0, 255, 255)
                cv2.polylines(color_mask, poligon, isClosed=True,
                              color=(255, 255, 0), thickness=1)

                self._publish_image(self.overlay_pub, line_image, resize_to=self.overlay_size)
                if bev_image is not None:
                    self._publish_image(self.bev_pub, bev_image)
                # Resize mask to overlay_size: native 820x410x3 ~= 1 MB/frame;
                # at 15 Hz that's ~15 MB/s of DDS traffic on a single topic,
                # enough to backpressure the publisher and stall the listener.
                self._publish_image(self.mask_pub, color_mask, resize_to=self.overlay_size)

            if self.enable_display:
                cv2.imshow("Detections", line_image)
                if bev_image is not None:
                    cv2.imshow("BEV Detections", bev_image)
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
