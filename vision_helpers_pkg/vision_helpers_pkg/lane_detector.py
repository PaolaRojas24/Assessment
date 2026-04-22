import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        # 1. Declare the parameter with a default value
        self.declare_parameter('subscribe_topic', '/camera/csi_image')
        self.declare_parameter('target_point_topic', '/lane_target_point_m')
        self.declare_parameter('bev_pixels_per_meter', 1000.0)
        self.declare_parameter('bev_dst_points_m', [
            0.0, 0.0,
            0.0, 0.103908,
            0.309683, 0.103908,
            0.309683, 0.0,
        ])
        self.declare_parameter('roi_polygon_points_px', [
            0.0, 460.0,
            240.0, 280.0,
            390.0, 280.0,
            640.0, 460.0,
        ])
        self.declare_parameter('camera_to_rear_axle_forward_m', 0.323)
        self.declare_parameter('camera_to_rear_axle_lateral_m', 0.0)
        self.declare_parameter('lane_half_width_px', 50.0)
        self.declare_parameter('lane_half_width_ema_alpha', 0.2)
        # Lateral bias: 0.0 = lane center, positive = toward left line (max ~0.9)
        self.declare_parameter('lane_lateral_bias', 0.0)
        
        # 2. Get the parameter value
        topic_name = self.get_parameter('subscribe_topic').get_parameter_value().string_value
        target_point_topic = self.get_parameter('target_point_topic').get_parameter_value().string_value

        self.group = ReentrantCallbackGroup()

        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.listener_callback,
            10,
            callback_group=self.group)
        
        self.lane_pub = self.create_publisher(
            Float32MultiArray, '/lane_lines', 10)
        self.target_point_pub = self.create_publisher(
            Float32MultiArray, target_point_topic, 10)
        
        self.br = CvBridge()

        hfov = 160 # Horizontal Field of View in degrees
        vfov = 120 # Vertical Field of View in degrees
        img_width = 640 # Image width in pixels
        img_height = 480 # Image height in pixels
        fx = (img_width / 2) / np.tan(np.radians(hfov / 2)) # Focal length in pixels
        fy = (img_height / 2) / np.tan(np.radians(vfov / 2)) # Focal length in pixels
        self.K = np.array([[fx, 0, img_width / 2],
                           [0, fy, img_height / 2],
                           [0, 0, 1]]) # Camera intrinsic matrix
        self.x_m_per_pixel = 2 * np.tan(np.radians(hfov / 2)) / img_width # Meters per pixel in x direction
        self.y_m_per_pixel = 2 * np.tan(np.radians(vfov / 2)) / img_height # Meters per pixel in y direction
        # <k1>-0.264598808</k1>
        # <k2>0.0156281135</k2>
        # <k3>0.0822019378</k3>
        # <p1>0.0000652954</p1>
        # <p2>0.0053984313</p2>
        self.distCoefs = np.array([-0.264598808, 0.0156281135, 0.0000652954, 0.0053984313, 0.0822019378]) # Distortion coefficients
        self.newK, roi = cv2.getOptimalNewCameraMatrix(self.K, self.distCoefs, (img_width, img_height), 1, (img_width, img_height))
        

        self.bev_pixels_per_meter = float(self.get_parameter('bev_pixels_per_meter').value)

        bev_dst_points_flat = self.get_parameter('bev_dst_points_m').value
        self.bev_dst_points_m = np.array(bev_dst_points_flat, dtype=np.float32).reshape((4, 2))

        roi_polygon_points_flat = self.get_parameter('roi_polygon_points_px').value
        self.roi_polygon_points_px = np.array(roi_polygon_points_flat, dtype=np.float32).reshape((4, 2))

        self.camera_to_rear_axle_forward_m = float(self.get_parameter('camera_to_rear_axle_forward_m').value)
        self.camera_to_rear_axle_lateral_m = float(self.get_parameter('camera_to_rear_axle_lateral_m').value)
        self.lane_half_width_px = float(self.get_parameter('lane_half_width_px').value)
        self.lane_half_width_ema_alpha = float(self.get_parameter('lane_half_width_ema_alpha').value)
        self.dynamic_lane_half_width_px = self.lane_half_width_px
        self.lane_lateral_bias = float(np.clip(self.get_parameter('lane_lateral_bias').value, -0.9, 0.9))

        bev_width_m = float(np.max(self.bev_dst_points_m[:, 0]) - np.min(self.bev_dst_points_m[:, 0]))
        bev_height_m = float(np.max(self.bev_dst_points_m[:, 1]) - np.min(self.bev_dst_points_m[:, 1]))
        self.bev_size = (
            max(1, int(np.ceil(bev_width_m * self.bev_pixels_per_meter))),
            max(1, int(np.ceil(bev_height_m * self.bev_pixels_per_meter))),
        )
        # White lane color range in HLS
        self.gray_lower = np.array([30, 160, 0], dtype=np.uint8)
        self.gray_upper = np.array([180, 200, 40], dtype=np.uint8)

        # Yellow lane color range in HLS
        self.yellow_lower = np.array([15, 30, 115], dtype=np.uint8)
        self.yellow_upper = np.array([35, 204, 255], dtype=np.uint8)
        self.get_logger().info(f'Lane Detector Node has been started. ROI: {self.K}')

    def lane_average(self, image, lines):
        left_fits = []
        right_fits = []
        
        if lines is None:
            return [None, None, None]

        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x1 == x2: continue # Evitar división por cero
            
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope, intersect = parameters[0], parameters[1]
            
            # negative slope = left lane (en coordenadas de imagen)
            # positive slope = right lane
            if slope < 0:
                left_fits.append((slope, intersect))
            else:
                right_fits.append((slope, intersect))
                
        # Average
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
        if fit is None: return None
        m, b = fit
        y1 = image.shape[0] # Fondo de la imagen
        y2 = int(y1 * 0.56)   # Look-ahead (56% de la imagen)
        x1 = int((y1 - b) / m)
        x2 = int((y2 - b) / m)
        return [x1, y1, x2, y2]
    
    def color_segment(self, hls, lower_range, upper_range):
        mask_in_range = cv2.inRange(hls, lower_range, upper_range)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask_dilated = cv2.morphologyEx(mask_in_range, cv2.MORPH_DILATE, kernel)

        return mask_dilated

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
            alpha * measured_half_width_px +
            (1.0 - alpha) * self.dynamic_lane_half_width_px
        )

    def select_target_pixel(self, left_line, right_line, center_line):
        bias = self.lane_lateral_bias
        half_w = self.dynamic_lane_half_width_px

        if center_line is not None:
            # center_line top point: shift left by bias fraction of half-width
            return [int(center_line[2] - bias * half_w), center_line[3]]

        if left_line is not None:
            # target = left_line + half_w * (1 - bias)
            # bias=0 -> center, bias=0.5 -> halfway to left line
            return [int(left_line[2] + half_w * (1.0 - bias)), left_line[3]]

        if right_line is not None:
            # target = right_line - half_w * (1 + bias)
            return [int(right_line[2] - half_w * (1.0 + bias)), right_line[3]]

        return None

    def target_to_rear_axle_m(self, target_pixel, homography_matrix):
        target_pixel_np = np.array([[[float(target_pixel[0]), float(target_pixel[1])]]], dtype=np.float32)
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

    def bird_eye_view(self, image, h_matrix):
        bev = cv2.warpPerspective(image, h_matrix, self.bev_size)
        return bev
    
    def listener_callback(self, data):
        # self.get_logger().info('Receiving video frame') # Uncomment for debugging
        current_frame = self.br.imgmsg_to_cv2(data, "bgr8")
        # undistorted_frame = cv2.undistort(current_frame, self.K, self.distCoefs, None, self.newK)
        # cv2.imshow("Undistorted", undistorted_frame)
        src = current_frame.copy()
        if current_frame is not None:
            try: 
                # self.get_logger().info('Receiving video frame')
                # --- Image Processing Logic ---
                poligon = self.roi_polygon_points_px.astype(np.int32).reshape((1, 4, 2))
                h_matrix = self.get_homography_matrix(poligon)
                hls = cv2.cvtColor(src, cv2.COLOR_BGR2HLS)
                # blur = cv2.GaussianBlur(gray, (5, 5), 0)
                # edges = cv2.Canny(blur, 50, 150)
                white_mask = self.color_segment(hls, self.gray_lower, self.gray_upper)
                yellow_mask = self.color_segment(hls, self.yellow_lower, self.yellow_upper)

                # Find min and max values and their locations
                # min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(edges)

                # print(f"Minimum Value: {min_val} at {min_loc}")
                # print(f"Maximum Value: {max_val} at {max_loc}")
                # ROI
                masked_colors = cv2.bitwise_or(white_mask, yellow_mask)
                roi_mask = np.zeros_like(masked_colors)
                cv2.fillPoly(roi_mask, poligon, 255)
                masked_edges = cv2.bitwise_and(masked_colors, roi_mask)
                # Hough
                lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, threshold=50, 
                        minLineLength=100, maxLineGap=50)
                
                # Extrapolation
                left_line, right_line, center_line = self.lane_average(src, lines)
                line_image = np.copy(src)
                # if lines is not None:
                #     for line in lines:
                #         x1, y1, x2, y2 = line[0]
                #         cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
                if left_line is not None:
                    # Dibujar línea AZUL (BGR) para izquierda
                    cv2.line(line_image, (left_line[0], left_line[1]), (left_line[2], left_line[3]), (255, 0, 0), 5)
                    
                if right_line is not None:
                    # Dibujar línea ROJA (BGR) para derecha
                    cv2.line(line_image, (right_line[0], right_line[1]), (right_line[2], right_line[3]), (0, 0, 255), 5)

                if center_line is not None:
                    # Dibujar línea VERDE (BGR) para centro
                    cv2.line(line_image, (center_line[0], center_line[1]), (center_line[2], center_line[3]), (0, 255, 0), 5)

                self.update_lane_half_width(left_line, right_line)

                target_pixel = self.select_target_pixel(left_line, right_line, center_line)
                target_msg = Float32MultiArray()
                if target_pixel is not None:
                    target_x_rear, target_y_rear = self.target_to_rear_axle_m(target_pixel, h_matrix)
                    target_msg.data = [float(target_x_rear), float(target_y_rear)]
                    cv2.circle(line_image, (target_pixel[0], target_pixel[1]), 5, (0, 255, 255), -1)
                    cv2.putText(
                        line_image,
                        f"target[m] x={target_x_rear:.3f}, y={target_y_rear:.3f}",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 255),
                        1,
                    )
                    cv2.putText(
                        line_image,
                        f"lane_half_width_px={self.dynamic_lane_half_width_px:.1f}",
                        (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 255),
                        1,
                    )
                else:
                    target_msg.data = [-1.0, -1.0]

                self.target_point_pub.publish(target_msg)

                cv2.polylines(line_image, poligon, isClosed=True, color=(255, 255, 0), thickness=2)
                bev_line_image = self.bird_eye_view(line_image, h_matrix)
                msg_lines = Float32MultiArray()
                # Convertimos a float y usamos -1.0 si es None
                left_data = [float(x) for x in left_line] if left_line is not None else [-1.0]*4
                right_data = [float(x) for x in right_line] if right_line is not None else [-1.0]*4
                msg_lines.data = left_data + right_data
                self.lane_pub.publish(msg_lines)

                # Display the processed frame
                cv2.imshow("Detections", line_image)
                cv2.imshow("BEV Detections", bev_line_image)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().error(f'Error processing video frame {e}')
                return

def main(args=None):
    rclpy.init(args=args)
    lane_detector = LaneDetector()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(lane_detector)

    try:
        rclpy.spin(lane_detector)
    except KeyboardInterrupt:
        pass
    finally:
        lane_detector.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()