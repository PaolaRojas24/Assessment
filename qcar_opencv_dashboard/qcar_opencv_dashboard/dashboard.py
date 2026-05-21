"""
guns-n-ROSes QCar dashboard.

Layout (title bar + 3 columns x 3 rows of equal cells):

  +-------------------------------------------+
  |        guns-n-ROSes      [SAFE STOP]      |   title bar
  +-----------+-----------+-------------------+
  | Processed |  Command  |       LiDAR       |
  +-----------+-----------+-------------------+
  |ColorSelect|    IMU    |     Csi_front     |
  +-----------+-----------+-------------------+
  |    BEV    |  Battery  |    Lidar det      |
  +-----------+-----------+-------------------+

Click the [SAFE STOP] button in the title bar (or press SPACE while the
window has focus) to toggle the safety-mux's safe-stop flag. While
engaged, safety_mux forces (throttle=0, steering=0) on /qcar/user_command
regardless of what the lane follower publishes.

Topics consumed:
  /qcar/decompressed/csi_front  Image                Csi_front tile (raw)
  /qcar/line_follower/overlay   Image                Processed (detections)
  /qcar/line_follower/bev       Image                BEV
  /qcar/line_follower/mask      Image                ColorSelect
  /lane_target_point_m          Float32MultiArray    target xy (status text)
  /qcar/user_command            Vector3Stamped       throttle/steering
  /qcar/scan                    LaserScan            LiDAR + Lidar detection
  /qcar/stateBattery            BatteryState         Battery
  /qcar/obstacle_detected       Bool                 Lidar detection status

Topics published:
  /qcar/safe_stop_active        Bool                 dashboard -> safety_mux

Run:
  ros2 run line_follower_real dashboard
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy, QoSDurabilityPolicy

import numpy as np
import cv2

from sensor_msgs.msg import Image, LaserScan, BatteryState
from std_msgs.msg import Float32MultiArray, Bool
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
        self.declare_parameter('battery_topic', '/qcar/stateBattery')
        self.declare_parameter('scan_topic', '/qcar/scan')
        self.declare_parameter('obstacle_topic', '/qcar/obstacle_detected')
        self.declare_parameter('safe_stop_pub_topic', '/qcar/safe_stop_active')
        # Safety-zone box dimensions, defaulted to lidar_node.py's constants.
        self.declare_parameter('obstacle_zone_depth_m', 0.4)
        self.declare_parameter('obstacle_zone_width_m', 1.2)
        self.declare_parameter('dead_zone_depth_m', 0.2)
        self.declare_parameter('dead_zone_width_m', 0.36)
        self.declare_parameter('lidar_range_m', 2.0)
        # Yaw correction (degrees) applied to the LiDAR scan before plotting,
        # so the car's forward direction points UP the panel. The RPLidar on
        # the QCar is mounted with its 0° at the side of the car, not at the
        # front -- without this offset, "front" would appear on the panel's
        # horizontal axis. Flip to -90.0 if the rotation goes the wrong way.
        self.declare_parameter('lidar_yaw_offset_deg', -90.0)
        self.declare_parameter('panel_width', 360)
        self.declare_parameter('panel_height', 270)
        self.declare_parameter('title_bar_height', 56)
        self.declare_parameter('team_name', 'guns-n-ROSes')
        self.declare_parameter('render_hz', 15.0)
        self.declare_parameter('window_title', 'guns-n-ROSes -- QCar dashboard')

        self._panel_w = int(self.get_parameter('panel_width').value)
        self._panel_h = int(self.get_parameter('panel_height').value)
        self._title = str(self.get_parameter('window_title').value)
        self._team_name = str(self.get_parameter('team_name').value)
        self._title_h = int(self.get_parameter('title_bar_height').value)
        self._lidar_range_m = float(self.get_parameter('lidar_range_m').value)
        yaw = np.radians(float(self.get_parameter('lidar_yaw_offset_deg').value))
        self._lidar_cos_yaw = float(np.cos(yaw))
        self._lidar_sin_yaw = float(np.sin(yaw))

        # Safety-zone rectangle dimensions (meters).
        self._obs_zone = (
            float(self.get_parameter('obstacle_zone_depth_m').value),
            float(self.get_parameter('obstacle_zone_width_m').value),
        )
        self._dead_zone = (
            float(self.get_parameter('dead_zone_depth_m').value),
            float(self.get_parameter('dead_zone_width_m').value),
        )
        # Safe-stop state. Click the title-bar button or press SPACE to toggle.
        self._safe_stop_active = False
        self._safe_stop_button_bbox = None  # populated each render

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
        self.create_subscription(
            BatteryState, self.get_parameter('battery_topic').value,
            self._on_battery, 10,
        )
        self.create_subscription(
            Bool, self.get_parameter('obstacle_topic').value,
            self._on_obstacle, 10,
        )

        # Publisher for safe-stop toggle. The safety_mux node subscribes here.
        self.safe_stop_pub = self.create_publisher(
            Bool, self.get_parameter('safe_stop_pub_topic').value, 10
        )

        self._raw = None
        self._overlay = None
        self._bev = None
        self._mask = None
        self._target = (None, None)
        self._cmd = {'throttle': 0.0, 'steering': 0.0, 'stamp_ns': 0}
        # Battery: only voltage is populated by the QCar driver; everything
        # else in BatteryState comes through as zero/empty.
        self._battery = {'voltage': None, 'stamp_ns': 0}
        # _scan_xy: precomputed (x, y) points in lidar frame (forward=+x, left=+y).
        self._scan_xy = None
        # Lidar obstacle flag from lidar_qcar's lidar_node.
        self._obstacle = False

        cv2.namedWindow(self._title, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self._title, self._on_mouse)

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

    def _on_battery(self, msg: BatteryState):
        self._battery = {
            'voltage': float(msg.voltage),
            'stamp_ns': msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
        }

    def _on_obstacle(self, msg: Bool):
        self._obstacle = bool(msg.data)

    def _set_safe_stop(self, active: bool):
        if self._safe_stop_active == active:
            return
        self._safe_stop_active = active
        self.get_logger().warn(
            f'SAFE STOP {"ENGAGED" if active else "RELEASED"} (from dashboard)'
        )
        out = Bool()
        out.data = active
        self.safe_stop_pub.publish(out)

    def _on_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        bbox = self._safe_stop_button_bbox
        if bbox is None:
            return
        x0, y0, x1, y1 = bbox
        if x0 <= x <= x1 and y0 <= y <= y1:
            self._set_safe_stop(not self._safe_stop_active)

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

    def _build_battery_panel(self):
        """Battery voltage tile. The QCar BatteryState only populates the
        `voltage` field -- everything else is zero/empty, so we don't try
        to display fake-zero current/percentage."""
        panel = np.zeros((self._panel_h, self._panel_w, 3), dtype=np.uint8)
        cv2.putText(panel, 'Battery /qcar/stateBattery', (6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
                    lineType=cv2.LINE_AA)

        v = self._battery['voltage']
        if v is None:
            cv2.putText(panel, 'no reading', (10, self._panel_h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 120, 120), 1,
                        lineType=cv2.LINE_AA)
            return panel

        # QCar pack is a 3S LiPo: 9.0 V empty, 11.1 V nominal, 12.6 V full.
        V_EMPTY, V_NOMINAL, V_FULL = 9.0, 11.1, 12.6
        if v >= 11.5:
            color = (0, 220, 0)
            label = 'GOOD'
        elif v >= 10.5:
            color = (0, 220, 255)
            label = 'OK'
        elif v >= V_EMPTY:
            color = (0, 0, 220)
            label = 'LOW -- CHARGE'
        else:
            color = (0, 0, 220)
            label = 'CRITICAL'

        # Large voltage readout.
        text = f'{v:.2f} V'
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.2, 2)
        tx = (self._panel_w - tw) // 2
        ty = 80
        cv2.putText(panel, text, (tx, ty),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 2,
                    lineType=cv2.LINE_AA)

        # Status word.
        (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.putText(panel, label, ((self._panel_w - lw) // 2, ty + 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1,
                    lineType=cv2.LINE_AA)

        # Horizontal voltage bar from V_EMPTY to V_FULL.
        bar_top = ty + 50
        bar_h = 14
        bar_left = 20
        bar_right = self._panel_w - 20
        bar_w = bar_right - bar_left
        cv2.rectangle(panel, (bar_left, bar_top),
                      (bar_right, bar_top + bar_h),
                      (60, 60, 60), 1)
        frac = float(np.clip((v - V_EMPTY) / (V_FULL - V_EMPTY), 0.0, 1.0))
        fill_right = bar_left + int(bar_w * frac)
        cv2.rectangle(panel, (bar_left + 1, bar_top + 1),
                      (fill_right, bar_top + bar_h - 1),
                      color, -1)
        # Nominal-voltage tick.
        nom_x = bar_left + int(bar_w * (V_NOMINAL - V_EMPTY) / (V_FULL - V_EMPTY))
        cv2.line(panel, (nom_x, bar_top - 3),
                 (nom_x, bar_top + bar_h + 3),
                 (200, 200, 200), 1)

        cv2.putText(panel, f'{V_EMPTY:.1f}', (bar_left - 4, bar_top + bar_h + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (160, 160, 160), 1,
                    lineType=cv2.LINE_AA)
        cv2.putText(panel, f'{V_FULL:.1f}', (bar_right - 22, bar_top + bar_h + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (160, 160, 160), 1,
                    lineType=cv2.LINE_AA)
        cv2.putText(panel, f'{V_NOMINAL:.1f}', (nom_x - 10, bar_top - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, (180, 180, 180), 1,
                    lineType=cv2.LINE_AA)
        return panel

    def _build_lidar_detection_panel(self):
        """Top-down LiDAR with the safety rectangles overlaid + a big
        OBSTACLE / CLEAR status indicator."""
        panel = np.zeros((self._panel_h, self._panel_w, 3), dtype=np.uint8)
        cv2.putText(panel, 'Lidar detection /qcar/obstacle_detected',
                    (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (255, 255, 255), 1, lineType=cv2.LINE_AA)

        cx = self._panel_w // 2
        cy = self._panel_h // 2

        # Scale so the obstacle zone fits comfortably with room around it.
        obs_depth, obs_width = self._obs_zone
        max_extent_m = max(obs_depth, obs_width) * 1.2
        half_min = min(self._panel_w, self._panel_h) // 2 - 24
        scale = max(1.0, half_min) / max(max_extent_m, 1e-3)

        # Apply same yaw correction as the LiDAR panel.
        c, s = self._lidar_cos_yaw, self._lidar_sin_yaw

        # Draw scan points dimmed (so the zones pop).
        if self._scan_xy is not None and self._scan_xy.size:
            xs = self._scan_xy[:, 0]
            ys = self._scan_xy[:, 1]
            xs_rot = c * xs + s * ys
            ys_rot = -s * xs + c * ys
            px = (cx - ys_rot * scale).astype(np.int32)
            py = (cy - xs_rot * scale).astype(np.int32)
            inside = ((px >= 1) & (px < self._panel_w - 1)
                      & (py >= 1) & (py < self._panel_h - 1))
            px = px[inside]
            py = py[inside]
            if px.size:
                panel[py, px] = (90, 90, 90)

        # Draw safety zones as filled translucent rectangles centred on the car.
        # The zones are defined in the lidar frame; we rotate the corners
        # the same way as the scan points so they line up after yaw correction.
        def zone_corners(depth, width):
            # ±half_depth in lidar +x, ±half_width in lidar +y.
            hd, hw = depth / 2.0, width / 2.0
            pts = np.array([[ hd,  hw], [ hd, -hw],
                            [-hd, -hw], [-hd,  hw]], dtype=np.float32)
            # Rotate.
            x = pts[:, 0]
            y = pts[:, 1]
            xr = c * x + s * y
            yr = -s * x + c * y
            sx = (cx - yr * scale).astype(np.int32)
            sy = (cy - xr * scale).astype(np.int32)
            return np.stack([sx, sy], axis=1).reshape((-1, 1, 2))

        obstacle_color = (0, 0, 220) if self._obstacle else (0, 200, 0)
        # Translucent fill by drawing on overlay then blending.
        overlay = panel.copy()
        cv2.fillPoly(overlay, [zone_corners(*self._obs_zone)], obstacle_color)
        cv2.fillPoly(overlay, [zone_corners(*self._dead_zone)], (200, 100, 0))
        cv2.addWeighted(overlay, 0.30, panel, 0.70, 0, panel)
        # Outlines.
        cv2.polylines(panel, [zone_corners(*self._obs_zone)], True,
                      obstacle_color, 1)
        cv2.polylines(panel, [zone_corners(*self._dead_zone)], True,
                      (220, 140, 0), 1)

        # Car triangle at centre.
        tri = np.array([[cx, cy - 6], [cx - 5, cy + 4], [cx + 5, cy + 4]], np.int32)
        cv2.fillPoly(panel, [tri], (0, 200, 0))

        # Big status word at the bottom.
        if self._obstacle:
            text = 'OBSTACLE'
            color = (0, 0, 230)
        else:
            text = 'CLEAR'
            color = (0, 220, 0)
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
        cv2.putText(panel, text,
                    ((self._panel_w - tw) // 2, self._panel_h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2,
                    lineType=cv2.LINE_AA)
        return panel

    def _build_placeholder_panel(self, label, message, sub=None):
        panel = np.zeros((self._panel_h, self._panel_w, 3), dtype=np.uint8)
        cv2.putText(panel, label, (6, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
                    lineType=cv2.LINE_AA)
        (mw, mh), _ = cv2.getTextSize(message, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
        cv2.putText(panel, message,
                    ((self._panel_w - mw) // 2, self._panel_h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (140, 140, 140), 2,
                    lineType=cv2.LINE_AA)
        if sub:
            (sw, sh), _ = cv2.getTextSize(sub, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
            cv2.putText(panel, sub,
                        ((self._panel_w - sw) // 2, self._panel_h // 2 + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1,
                        lineType=cv2.LINE_AA)
        return panel

    def _build_title_bar(self, width):
        """Title bar with team name and a clickable SAFE STOP button.
        Returns the rendered bar; also updates self._safe_stop_button_bbox
        so the mouse-callback knows where the button is."""
        bar = np.full((self._title_h, width, 3), 25, dtype=np.uint8)

        # Team name on the left.
        cv2.putText(bar, self._team_name, (16, 38),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.95, (240, 240, 240), 2,
                    lineType=cv2.LINE_AA)

        # Safe-stop button on the right.
        btn_w, btn_h = 220, self._title_h - 14
        btn_x1 = width - 16
        btn_x0 = btn_x1 - btn_w
        btn_y0 = (self._title_h - btn_h) // 2
        btn_y1 = btn_y0 + btn_h
        self._safe_stop_button_bbox = (btn_x0, btn_y0, btn_x1, btn_y1)

        if self._safe_stop_active:
            fill = (0, 0, 200)
            border = (0, 0, 255)
            label = 'SAFE STOP: ON'
        else:
            fill = (50, 50, 50)
            border = (140, 140, 140)
            label = 'SAFE STOP: off  (click / SPACE)'

        cv2.rectangle(bar, (btn_x0, btn_y0), (btn_x1, btn_y1), fill, -1)
        cv2.rectangle(bar, (btn_x0, btn_y0), (btn_x1, btn_y1), border, 2)

        (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        text_x = btn_x0 + (btn_w - lw) // 2
        text_y = btn_y0 + (btn_h + lh) // 2 - 2
        cv2.putText(bar, label, (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (240, 240, 240), 1,
                    lineType=cv2.LINE_AA)
        return bar

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
        # 3 columns x 3 rows, all equal cells.
        # Row 1:  Processed   | Command      | LiDAR
        # Row 2:  ColorSelect | IMU          | Csi_front
        # Row 3:  BEV         | Battery      | Lidar detection
        r1c1 = self._fit_panel(self._overlay, 'Processed /qcar/line_follower/overlay')
        r1c2 = self._build_status_panel()
        r1c3 = self._build_lidar_panel()

        r2c1 = self._fit_panel(self._mask, 'ColorSelect /qcar/line_follower/mask')
        r2c2 = self._build_placeholder_panel(
            'IMU', 'IMU + encoders', 'PCB / ESP32 firmware pending'
        )
        r2c3 = self._fit_panel(self._raw, 'Csi_front /qcar/decompressed/csi_front')

        r3c1 = self._fit_panel(self._bev, 'BEV /qcar/line_follower/bev')
        r3c2 = self._build_battery_panel()
        r3c3 = self._build_lidar_detection_panel()

        row1 = cv2.hconcat([r1c1, r1c2, r1c3])
        row2 = cv2.hconcat([r2c1, r2c2, r2c3])
        row3 = cv2.hconcat([r3c1, r3c2, r3c3])
        body = cv2.vconcat([row1, row2, row3])

        title = self._build_title_bar(body.shape[1])
        canvas = cv2.vconcat([title, body])

        cv2.imshow(self._title, canvas)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):  # q or ESC
            self.get_logger().info('quit key pressed -- shutting down dashboard')
            raise KeyboardInterrupt()
        if key == ord(' '):
            self._set_safe_stop(not self._safe_stop_active)


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
