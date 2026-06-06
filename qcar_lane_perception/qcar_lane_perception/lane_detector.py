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
            20.0,  410.0,
            60.0,  270.0,
            620.0, 270.0,
            670.0, 410.0,
        ], dtype=np.float32).reshape((4, 2))

        # ---- Physical car geometry ----
        self.camera_to_rear_axle_forward_m = 0.323
        self.camera_to_rear_axle_lateral_m = 0.0

        # ---- Lane tracking state ----
        # True: fit all Hough points as one line and follow it directly —
        # ignores left/right classification and half_width offset.
        # False: original left/right split with half_width offset to lane center.
        self.follow_line_directly        = True
        # Fixed pixel offset from the detected line (follow_line_directly only).
        # Positive = right of line, negative = left. 500 px/m → 25 px = 5 cm.
        self.line_offset_px              = 10    # compensación mínima offset BEV (~1cm)
        self.lane_half_width_px         = 70.0
        self.lane_half_width_ema_alpha   = 0.2
        self.dynamic_lane_half_width_px  = self.lane_half_width_px
        self.lane_lateral_bias           = 0.0


        # ---- Hough — tuned for BEV (217×52 px) ----
        self.hough_threshold       = 10
        self.hough_min_line_length = 8
        self.hough_max_line_gap    = 8
        self.hough_x_max_px        = 120  # ignora detecciones a la derecha de este px

        # ---- Preprocess ----
        self.wb_enable  = True
        self.wb_p       = 3.0
        self.clahe_clip = 2.0
        self.clahe_tile = 8

        # ---- HLS colour thresholds ----
        # White completely disabled — walls pass the white threshold and create
        # false left-lane detections that cause oscillation.
        self.gray_lower   = np.array([0,   111, 0],  dtype=np.uint8)
        self.gray_upper   = np.array([179, 230, 65], dtype=np.uint8)
        self.yellow_lower = np.array([4,   19,  50], dtype=np.uint8)
        self.yellow_upper = np.array([35,  225, 255], dtype=np.uint8)

        # ---- Mask morphology ----
        # open_k=3 preserves thin lines in the small BEV (5×5 erases them).
        self.mask_median_k = 5
        self.mask_open_k   = 3
        self.mask_close_k  = 5
        # White blobs larger than this area (px²) are treated as walls and removed.
        self.white_max_blob_area = 230

        # ---- Canny (applied on BEV grayscale) ----
        self.canny_low  = 14
        self.canny_high = 75

        # ---- Target-selection state ----
        self.last_detected_lane = 1
        self.last_target_pixel  = None
        self.max_waiting_cycles = 45   # ~3s @ 15Hz antes de publicar stop
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

        # Cache polygon and homography — never change between frames
        self._polygon  = self.roi_polygon_points_px.astype(np.int32).reshape((1, 4, 2))
        self._h_matrix = self.get_homography_matrix(self._polygon)

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
            p = self.wb_p
            pixels = f.reshape(-1, 3)
            # Shades of Gray: per-channel Minkowski-p norm
            est = np.power(
                np.power(pixels, p).mean(axis=0) + 1e-6,
                1.0 / p
            )
            avg = float(est.mean())
            if avg > 1.0:
                gains = np.clip(avg / np.maximum(est, 1e-6), 0.5, 2.0)
                f *= gains
            img = np.clip(f, 0, 255).astype(np.uint8)
        if self.clahe_clip > 0.05:
            lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
            clahe = cv2.createCLAHE(clipLimit=self.clahe_clip,
                                    tileGridSize=(self.clahe_tile, self.clahe_tile))
            lab[:, :, 0] = clahe.apply(lab[:, :, 0])
            img = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        return img

    def _remove_large_blobs(self, mask, max_area):
        """Zero-out connected components larger than max_area pixels."""
        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        out = mask.copy()
        for i in range(1, n):  # skip background (i=0)
            if stats[i, cv2.CC_STAT_AREA] > max_area:
                out[labels == i] = 0
        return out

    def _clean_mask(self, mask):
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
        def fit(pts):
            if len(pts) < 2:
                return None
            ys, xs = zip(*pts)
            try:
                a, b = np.polyfit(ys, xs, 1)   # x = a·y + b
            except Exception:
                return None
            y_far    = bev_h - 1
            y_target = int(bev_h * 0.5)
            lim = 2 * bev_w
            x_far    = int(np.clip(a * y_far    + b, -lim, lim))
            x_target = int(np.clip(a * y_target + b, -lim, lim))
            return [x_far, y_far, x_target, y_target]

        # follow_line_directly: fit ALL points as one line, ignore left/right.
        if self.follow_line_directly:
            all_pts = []
            if lines is not None:
                x_max = self.hough_x_max_px
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    if abs(y2 - y1) < 5:
                        continue
                    if x1 > x_max or x2 > x_max:
                        continue
                    all_pts.extend([(y1, x1), (y2, x2)])
            single = fit(all_pts)
            return [None, None, single]

        left_pts  = []
        right_pts = []
        mid_x = bev_w * 0.50

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if abs(y2 - y1) < 5:
                    continue
                # Classify by near-end x (min y = closest to car).
                # avg_x misclassifies curved lines whose far end crosses the midpoint.
                near_x = x1 if y1 <= y2 else x2
                if near_x < mid_x:
                    left_pts.extend([(y1, x1), (y2, x2)])
                else:
                    right_pts.extend([(y1, x1), (y2, x2)])

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
        raw_width = float(right_line[2] - left_line[2])
        half_w = abs(raw_width) * 0.5
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
            offset = self.line_offset_px if self.follow_line_directly else int(bias * half_w)
            self.last_target_pixel  = [
                int(center_line[2] + offset), center_line[3]]
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

            polygon  = self._polygon
            h_matrix = self._h_matrix

            # 1. Warp raw frame to BEV (217×52 px — 30× smaller than full frame)
            bev_color = cv2.warpPerspective(src, h_matrix, self.bev_size)

            # 2. WB + CLAHE on small BEV (much faster than full frame)
            bev_color = self._preprocess(bev_color)

            # 3. Colour segmentation — solo línea amarilla
            hls_bev     = cv2.cvtColor(bev_color, cv2.COLOR_BGR2HLS)
            yellow_mask = self.color_segment(hls_bev, self.yellow_lower, self.yellow_upper)
            bev_mask    = self._clean_mask(yellow_mask)

            # 4. Canny + AND con máscara amarilla
            bev_gray  = cv2.cvtColor(bev_color, cv2.COLOR_BGR2GRAY)
            bev_gray  = cv2.GaussianBlur(bev_gray, (3, 3), 1.0)
            bev_edges = cv2.Canny(bev_gray, self.canny_low, self.canny_high)
            hough_in  = cv2.bitwise_and(bev_edges, bev_mask)

            # 5. Hough in BEV space
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

            target_px = raw_target

            # 8. BEV pixel → metric (direct, no perspectiveTransform)
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

            # Perspective overlay: original frame + ROI polygon
            line_image = src.copy()
            cv2.polylines(line_image, polygon, isClosed=True,
                          color=(255, 255, 0), thickness=1)

            if need_overlay:
                # Flip BEV for human readability (near at bottom, far at top)
                bev_display = cv2.flip(bev_bgr, 0)

                # Mask: BEV con línea amarilla detectada (cyan)
                color_mask = cv2.cvtColor(bev_mask, cv2.COLOR_GRAY2BGR)
                color_mask[bev_mask > 0] = (0, 255, 255)

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
