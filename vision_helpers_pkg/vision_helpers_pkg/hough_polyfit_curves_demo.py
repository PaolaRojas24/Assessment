import warnings

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class HoughPolyfitCurvesDemo(Node):
    def __init__(self):
        super().__init__('hough_polyfit_curves_demo')

        self.declare_parameter('subscribe_topic', '/qcar_sim/csi_front/image_raw')
        self.declare_parameter('poly_degrees', [2, 3, 4])
        self.declare_parameter('roi_polygon_points_px', [
            0.0, 460.0,
            240.0, 280.0,
            390.0, 280.0,
            640.0, 460.0,
        ])

        topic_name = self.get_parameter('subscribe_topic').value
        self.degrees = [int(d) for d in self.get_parameter('poly_degrees').value]

        roi_flat = self.get_parameter('roi_polygon_points_px').value
        self.roi_polygon_points_px = np.array(roi_flat, dtype=np.float32).reshape((4, 2))

        # White lane color range in HLS
        self.gray_lower = np.array([30, 160, 0], dtype=np.uint8)
        self.gray_upper = np.array([180, 200, 40], dtype=np.uint8)

        # Yellow lane color range in HLS
        self.yellow_lower = np.array([15, 30, 115], dtype=np.uint8)
        self.yellow_upper = np.array([35, 204, 255], dtype=np.uint8)

        self.br = CvBridge()

        self.create_subscription(
            Image,
            topic_name,
            self.listener_callback,
            10,
        )

        self.get_logger().info(
            'Started hough_polyfit_curves_demo. This demo shows how degree>=2 polynomials behave after Hough points.'
        )

    def color_segment(self, hls, lower_range, upper_range):
        mask_in_range = cv2.inRange(hls, lower_range, upper_range)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        return cv2.morphologyEx(mask_in_range, cv2.MORPH_DILATE, kernel)

    def split_hough_points(self, lines):
        left_pts = []
        right_pts = []

        if lines is None:
            return left_pts, right_pts

        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x1 == x2:
                continue

            slope = (y2 - y1) / (x2 - x1)

            # Same convention as your lane detector: negative slope = left, positive slope = right.
            if slope < 0:
                left_pts.append((x1, y1))
                left_pts.append((x2, y2))
            else:
                right_pts.append((x1, y1))
                right_pts.append((x2, y2))

        return left_pts, right_pts

    def fit_poly_x_of_y(self, points, degree):
        # For lane geometry in image space, fitting x=f(y) is more stable than y=f(x).
        if len(points) < degree + 1:
            return None, f'Not enough points ({len(points)}) for degree {degree}'

        pts = np.array(points, dtype=np.float32)
        y = pts[:, 1]
        x = pts[:, 0]

        try:
            with warnings.catch_warnings(record=True) as w:
                warnings.simplefilter('always', np.RankWarning)
                coeffs = np.polyfit(y, x, degree)
                if len(w) > 0:
                    return coeffs, f'RankWarning on degree {degree} fit (likely unstable)'
            return coeffs, None
        except Exception as exc:
            return None, f'Fit failed: {exc}'

    def draw_poly_curve(self, canvas, coeffs, color, y_min, y_max):
        ys = np.linspace(y_min, y_max, num=120)
        xs = np.polyval(coeffs, ys)

        points = []
        h, w = canvas.shape[:2]
        for x, y in zip(xs, ys):
            xi = int(round(x))
            yi = int(round(y))
            if 0 <= xi < w and 0 <= yi < h:
                points.append((xi, yi))

        if len(points) >= 2:
            cv2.polylines(canvas, [np.array(points, dtype=np.int32)], False, color, 2)

    def listener_callback(self, data):
        frame = self.br.imgmsg_to_cv2(data, 'bgr8')
        if frame is None:
            return

        try:
            poly = self.roi_polygon_points_px.astype(np.int32).reshape((1, 4, 2))

            hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
            white_mask = self.color_segment(hls, self.gray_lower, self.gray_upper)
            yellow_mask = self.color_segment(hls, self.yellow_lower, self.yellow_upper)
            lane_mask = cv2.bitwise_or(white_mask, yellow_mask)

            roi_mask = np.zeros_like(lane_mask)
            cv2.fillPoly(roi_mask, poly, 255)
            masked = cv2.bitwise_and(lane_mask, roi_mask)

            lines = cv2.HoughLinesP(
                masked,
                1,
                np.pi / 180,
                threshold=50,
                minLineLength=100,
                maxLineGap=50,
            )

            left_pts, right_pts = self.split_hough_points(lines)

            base = frame.copy()
            cv2.polylines(base, poly, isClosed=True, color=(255, 255, 0), thickness=2)

            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(base, (x1, y1), (x2, y2), (180, 180, 180), 1)

            cv2.putText(
                base,
                f'Hough points | left: {len(left_pts)} right: {len(right_pts)}',
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
            )

            cv2.imshow('Hough Segments', base)

            # Degree-specific overlays to visualize what happens as degree increases.
            y_min = int(np.min(poly[:, :, 1]))
            y_max = int(np.max(poly[:, :, 1]))

            for degree in self.degrees:
                canvas = base.copy()

                left_coeffs, left_msg = self.fit_poly_x_of_y(left_pts, degree)
                right_coeffs, right_msg = self.fit_poly_x_of_y(right_pts, degree)

                if left_coeffs is not None:
                    self.draw_poly_curve(canvas, left_coeffs, (255, 0, 0), y_min, y_max)
                if right_coeffs is not None:
                    self.draw_poly_curve(canvas, right_coeffs, (0, 0, 255), y_min, y_max)

                status = f'deg={degree}'
                if left_msg:
                    status += f' | L: {left_msg}'
                if right_msg:
                    status += f' | R: {right_msg}'

                cv2.putText(
                    canvas,
                    status[:120],
                    (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (0, 255, 0),
                    1,
                )

                cv2.imshow(f'Polyfit Degree {degree}', canvas)

            cv2.waitKey(1)

        except Exception as exc:
            self.get_logger().error(f'Error in demo processing: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = HoughPolyfitCurvesDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
