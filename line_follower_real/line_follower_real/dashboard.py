"""
QCar dashboard: one OpenCV window with everything you'd otherwise
need five or six separate viewers for.

Layout (3 columns x 2 rows, all equal cells, default 1080x540):

  +----------+-----------+----------+
  |   Raw    | Detection |   BEV    |
  +----------+-----------+----------+
  |   Mask   |  Command  |  LiDAR   |
  +----------+-----------+----------+

Topics consumed:
  /qcar/decompressed/csi_front  sensor_msgs/Image          raw camera
  /qcar/line_follower/overlay   sensor_msgs/Image          annotated
  /qcar/line_follower/bev       sensor_msgs/Image          BEV
  /qcar/line_follower/mask      sensor_msgs/Image          color-coded mask
  /lane_target_point_m          std_msgs/Float32MultiArray target xy
  /qcar/user_command            geometry_msgs/Vector3Stamped  cmd
                                (x=throttle, y=steering -- platform=qcar)
  /qcar/scan                    sensor_msgs/LaserScan      RPLidar A2

Run:
  ros2 run line_follower_real dashboard
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy, QoSDurabilityPolicy

import numpy as np
import cv2

from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3Stamped


def _be_qos():
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        durability=QoSDurabilityPolicy.VOLATILE,
        depth=1,
    )


class Dashboard(Node):
    def __init__(self):
        super().__init__('line_follower_dashboard')

        self.declare_parameter('raw_topic', '/qcar/decompressed/csi_front')
        self.declare_parameter('overlay_topic', '/qcar/line_follower/overlay')
        self.declare_parameter('bev_topic', '/qcar/line_follower/bev')
        self.declare_parameter('mask_topic', '/qcar/line_follower/mask')
        self.declare_parameter('target_topic', '/lane_target_point_m')
        self.declare_parameter('cmd_topic', '/qcar/user_command')
        self.declare_parameter('scan_topic', '/qcar/scan')
        self.declare_parameter('lidar_range_m', 2.0)
        # Yaw correction (degrees) applied to the LiDAR scan before plotting,
        # so the car's forward direction points UP the panel. The RPLidar on
        # the QCar is mounted with its 0° at the side of the car, not at the
        # front -- without this offset, "front" would appear on the panel's
        # horizontal axis. Flip to -90.0 if the rotation goes the wrong way.
        self.declare_parameter('lidar_yaw_offset_deg', 90.0)
        self.declare_parameter('panel_width', 360)
        self.declare_parameter('panel_height', 270)
        self.declare_parameter('render_hz', 15.0)
        self.declare_parameter('window_title', 'QCar dashboard')

        self._panel_w = int(self.get_parameter('panel_width').value)
        self._panel_h = int(self.get_parameter('panel_height').value)
        self._title = str(self.get_parameter('window_title').value)
        self._lidar_range_m = float(self.get_parameter('lidar_range_m').value)
        yaw = np.radians(float(self.get_parameter('lidar_yaw_offset_deg').value))
        self._lidar_cos_yaw = float(np.cos(yaw))
        self._lidar_sin_yaw = float(np.sin(yaw))

        be = _be_qos()
        self.create_subscription(
            Image, self.get_parameter('raw_topic').value,
            self._on_raw, be,
        )
        self.create_subscription(
            Image, self.get_parameter('overlay_topic').value,
            self._on_overlay, be,
        )
        self.create_subscription(
            Image, self.get_parameter('bev_topic').value,
            self._on_bev, be,
        )
        self.create_subscription(
            Image, self.get_parameter('mask_topic').value,
            self._on_mask, be,
        )
        self.create_subscription(
            Float32MultiArray, self.get_parameter('target_topic').value,
            self._on_target, 10,
        )
        self.create_subscription(
            Vector3Stamped, self.get_parameter('cmd_topic').value,
            self._on_cmd, 10,
        )
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self._on_scan, be,
        )

        self._raw = None
        self._overlay = None
        self._bev = None
        self._mask = None
        self._target = (None, None)
        self._cmd = {'throttle': 0.0, 'steering': 0.0, 'stamp_ns': 0}
        # _scan_xy: precomputed (x, y) points in lidar frame (forward=+x, left=+y).
        self._scan_xy = None

        cv2.namedWindow(self._title, cv2.WINDOW_AUTOSIZE)

        period = 1.0 / float(self.get_parameter('render_hz').value)
        self._render_timer = self.create_timer(period, self._render)

        self.get_logger().info(
            f'dashboard: panel {self._panel_w}x{self._panel_h}, '
            f'render @ {1.0/period:.1f} Hz'
        )

    @staticmethod
    def _decode(msg: Image):
        if msg.encoding == 'bgr8':
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(
                (msg.height, msg.width, 3)
            )
        if msg.encoding == 'rgb8':
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                (msg.height, msg.width, 3)
            )
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        if msg.encoding == 'mono8':
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                (msg.height, msg.width)
            )
            return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
        return None

    def _on_raw(self, msg):
        img = self._decode(msg)
        if img is not None:
            self._raw = img

    def _on_overlay(self, msg):
        img = self._decode(msg)
        if img is not None:
            self._overlay = img

    def _on_bev(self, msg):
        img = self._decode(msg)
        if img is not None:
            self._bev = img

    def _on_mask(self, msg):
        img = self._decode(msg)
        if img is not None:
            self._mask = img

    def _on_target(self, msg):
        d = msg.data
        if len(d) >= 2 and d[0] != -1.0 and d[1] != -1.0:
            self._target = (float(d[0]), float(d[1]))
        else:
            self._target = (None, None)

    def _on_cmd(self, msg):
        self._cmd = {
            'throttle': float(msg.vector.x),
            'steering': float(msg.vector.y),
            'stamp_ns': msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
        }

    def _on_scan(self, msg: LaserScan):
        # Vectorized polar -> Cartesian. LaserScan ranges are in meters, angles
        # in radians from angle_min, growing by angle_increment.
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        if ranges.size == 0:
            self._scan_xy = None
            return
        n = ranges.size
        angles = (msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment)
        rmax = float(msg.range_max) if msg.range_max > 0 else 30.0
        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < rmax)
        if not np.any(valid):
            self._scan_xy = None
            return
        r = ranges[valid]
        a = angles[valid]
        # Forward = +x, left = +y (ROS REP-103).
        x = r * np.cos(a)
        y = r * np.sin(a)
        self._scan_xy = np.stack([x, y], axis=1)

    def _fit_panel(self, img, label, width=None, height=None):
        """Letterbox `img` into a (width x height) tile and stamp a label."""
        tw = width if width is not None else self._panel_w
        th = height if height is not None else self._panel_h
        tile = np.zeros((th, tw, 3), dtype=np.uint8)
        if img is not None and img.size > 0:
            h, w = img.shape[:2]
            scale = min(tw / w, th / h)
            new_w = max(1, int(w * scale))
            new_h = max(1, int(h * scale))
            resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)
            x0 = (tw - new_w) // 2
            y0 = (th - new_h) // 2
            tile[y0:y0 + new_h, x0:x0 + new_w] = resized
        else:
            cv2.putText(tile, 'waiting...', (10, th // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 80, 80), 1)
        cv2.putText(tile, label, (6, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                    lineType=cv2.LINE_AA)
        return tile

    def _draw_steering_arc(self, panel, steering_rad, throttle, max_steer=0.5):
        """Draws a top-down steering indicator on the right side of the status panel."""
        cx = panel.shape[1] - 70
        cy = panel.shape[0] - 50
        radius = 38
        # base circle
        cv2.circle(panel, (cx, cy), radius, (60, 60, 60), 1)
        # steering needle (left positive == counter-clockwise on screen)
        clipped = float(np.clip(steering_rad, -max_steer, max_steer))
        # Map [-max_steer, +max_steer] -> [-90deg, +90deg], 0 = forward (up).
        angle = -np.pi / 2 + (clipped / max_steer) * (np.pi / 2)
        ex = int(cx + radius * np.cos(angle))
        ey = int(cy + radius * np.sin(angle))
        color = (0, 200, 255) if abs(clipped) > 1e-3 else (0, 200, 0)
        cv2.line(panel, (cx, cy), (ex, ey), color, 2)
        # throttle bar (vertical) on the left of the dial
        bar_x = cx - radius - 18
        bar_top = cy - radius
        bar_bottom = cy + radius
        cv2.rectangle(panel, (bar_x, bar_top), (bar_x + 8, bar_bottom),
                      (60, 60, 60), 1)
        # forward = green up, reverse = red down
        tt = float(np.clip(throttle, -0.3, 0.3)) / 0.3
        mid = (bar_top + bar_bottom) // 2
        if tt >= 0:
            top = int(mid - tt * (mid - bar_top))
            cv2.rectangle(panel, (bar_x + 1, top), (bar_x + 7, mid),
                          (0, 200, 0), -1)
        else:
            bot = int(mid - tt * (bar_bottom - mid))
            cv2.rectangle(panel, (bar_x + 1, mid), (bar_x + 7, bot),
                          (0, 0, 220), -1)

    def _build_lidar_panel(self):
        """Top-down LiDAR view: car at panel CENTER so the user can see all around.
        Forward (after yaw correction) points UP the panel."""
        panel = np.zeros((self._panel_h, self._panel_w, 3), dtype=np.uint8)
        cv2.putText(panel, f'LiDAR /qcar/scan  (R={self._lidar_range_m:.1f} m)',
                    (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1, lineType=cv2.LINE_AA)

        # Plot center: middle of the panel (full 360° visible around the car).
        cx = self._panel_w // 2
        cy = self._panel_h // 2

        # Scale so lidar_range_m fits inside half the smaller panel dimension,
        # minus a margin so range labels don't get clipped.
        margin_px = 18
        half_min = min(self._panel_w, self._panel_h) // 2 - margin_px
        scale = max(1.0, half_min) / self._lidar_range_m

        # Range circles at 1 m intervals up to lidar_range_m.
        for r in range(1, int(np.ceil(self._lidar_range_m)) + 1):
            radius = int(r * scale)
            if radius < 4:
                continue
            cv2.circle(panel, (cx, cy), radius, (40, 40, 40), 1)
            cv2.putText(panel, f'{r}m', (cx + radius - 22, cy - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, (70, 70, 70), 1,
                        lineType=cv2.LINE_AA)

        # Axes through the car: forward (up) and lateral (across).
        axis_len = int(self._lidar_range_m * scale)
        cv2.line(panel, (cx, cy - axis_len), (cx, cy + axis_len), (60, 60, 60), 1)
        cv2.line(panel, (cx - axis_len, cy), (cx + axis_len, cy), (60, 60, 60), 1)

        # Car marker: small triangle pointing up = forward.
        car = np.array([[cx, cy - 6], [cx - 5, cy + 4], [cx + 5, cy + 4]], np.int32)
        cv2.fillPoly(panel, [car], (0, 200, 0))

        # Scan points (with yaw correction so the car's forward axis = up).
        if self._scan_xy is not None and self._scan_xy.size:
            xs = self._scan_xy[:, 0]
            ys = self._scan_xy[:, 1]
            c, s = self._lidar_cos_yaw, self._lidar_sin_yaw
            # Rotate (xs, ys) by -yaw so the lidar's "forward" lines up with
            # the panel's +y_up. This compensates for the lidar's mounting
            # angle on the QCar.
            xs_rot = c * xs + s * ys
            ys_rot = -s * xs + c * ys

            # Screen mapping: +x_rot (forward) -> up; +y_rot (left) -> left.
            px = (cx - ys_rot * scale).astype(np.int32)
            py = (cy - xs_rot * scale).astype(np.int32)
            inside = ((px >= 1) & (px < self._panel_w - 1)
                      & (py >= 1) & (py < self._panel_h - 1))
            px = px[inside]
            py = py[inside]
            if px.size:
                # Color by distance: close = red, far = cyan.
                dist = np.hypot(xs[inside], ys[inside])
                t = np.clip(dist / max(self._lidar_range_m, 1e-3), 0.0, 1.0)
                b = (t * 255).astype(np.uint8)
                g = (t * 200 + 40).astype(np.uint8)
                r = ((1.0 - t) * 255).astype(np.uint8)
                panel[py, px] = np.stack([b, g, r], axis=1)
                # Pad each point to a 3x3 dot for visibility.
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        py2 = py + dy
                        px2 = px + dx
                        m = ((px2 >= 0) & (px2 < self._panel_w)
                             & (py2 >= 0) & (py2 < self._panel_h))
                        panel[py2[m], px2[m]] = np.stack([b[m], g[m], r[m]], axis=1)

        return panel

    def _build_status_panel(self, width=None, height=None):
        tw = width if width is not None else self._panel_w
        th = height if height is not None else self._panel_h
        panel = np.zeros((th, tw, 3), dtype=np.uint8)
        cv2.putText(panel, 'QCar command', (6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1,
                    lineType=cv2.LINE_AA)

        throttle = self._cmd['throttle']
        steering = self._cmd['steering']

        if throttle > 0.005:
            motion = 'FORWARD'
            mcol = (0, 220, 0)
        elif throttle < -0.005:
            motion = 'REVERSE'
            mcol = (0, 0, 220)
        else:
            motion = 'STOP'
            mcol = (160, 160, 160)

        if steering > 0.02:
            steer_word = 'LEFT'
            scol = (0, 200, 255)
        elif steering < -0.02:
            steer_word = 'RIGHT'
            scol = (0, 200, 255)
        else:
            steer_word = 'STRAIGHT'
            scol = (160, 160, 160)

        lines = [
            ('throttle', f'{throttle:+.3f}', mcol),
            ('steering', f'{steering:+.3f} rad ({np.degrees(steering):+.1f} deg)', scol),
            ('motion',   motion, mcol),
            ('wheels',   steer_word, scol),
        ]
        if self._target[0] is not None:
            lines.append(('target xy',
                          f'x={self._target[0]:+.3f}  y={self._target[1]:+.3f}',
                          (0, 200, 255)))
        else:
            lines.append(('target xy', 'no lane', (120, 120, 120)))

        y = 42
        for label, value, color in lines:
            cv2.putText(panel, f'{label}:', (6, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1,
                        lineType=cv2.LINE_AA)
            cv2.putText(panel, value, (95, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1,
                        lineType=cv2.LINE_AA)
            y += 22

        self._draw_steering_arc(panel, steering, throttle)
        return panel

    def _render(self):
        # 3 x 2 layout, all cells equal width:
        #   raw    | detections | BEV
        #   mask   | status     | lidar
        top_l = self._fit_panel(self._raw,     'raw /qcar/decompressed/csi_front')
        top_m = self._fit_panel(self._overlay, 'detections /qcar/line_follower/overlay')
        top_r = self._fit_panel(self._bev,     'BEV /qcar/line_follower/bev')

        bot_l = self._fit_panel(self._mask,    'mask /qcar/line_follower/mask')
        bot_m = self._build_status_panel()
        bot_r = self._build_lidar_panel()

        top = cv2.hconcat([top_l, top_m, top_r])
        bottom = cv2.hconcat([bot_l, bot_m, bot_r])
        canvas = cv2.vconcat([top, bottom])

        cv2.imshow(self._title, canvas)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):  # q or ESC
            self.get_logger().info('quit key pressed -- shutting down dashboard')
            raise KeyboardInterrupt()


def main(args=None):
    rclpy.init(args=args)
    node = Dashboard()
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
