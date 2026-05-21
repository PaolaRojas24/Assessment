"""
guns-n-ROSes -- PyQt5 dashboard for the QCar.

Same feature set as the OpenCV dashboard in line_follower_real:
title bar + safe-stop toggle + 4 x 3 grid of tiles.

Architecture:
  - rclpy spin runs in a background thread (RosNode).
  - A separate QObject (Signals) holds the pyqtSignal definitions.
  - Each ROS callback in RosNode emits the corresponding signal.
  - The Qt main window connects signals to widget slots.
  - Qt's auto-queued connections deliver them on the GUI thread, so widgets
    can be updated safely without locks.

Layout:

  +-----------------------------------------+
  |  guns-n-ROSes              [SAFE STOP]  |
  +-----------+-----------+-----------------+
  | Processed |  Command  |      LiDAR      |
  +-----------+-----------+-----------------+
  |ColorSelect|    IMU    |     Csi_front   |
  +-----------+-----------+-----------------+
  |    BEV    |  Battery  |    Lidar det    |
  +-----------+-----------+-----------------+

Topics consumed:
  /qcar/decompressed/csi_front  sensor_msgs/Image          Csi_front tile
  /qcar/line_follower/overlay   sensor_msgs/Image          Processed
  /qcar/line_follower/bev       sensor_msgs/Image          BEV
  /qcar/line_follower/mask      sensor_msgs/Image          ColorSelect
  /lane_target_point_m          std_msgs/Float32MultiArray target xy
  /qcar/user_command            geometry_msgs/Vector3Stamped throttle / steering
  /qcar/scan                    sensor_msgs/LaserScan      LiDAR + Lidar detection
  /qcar/stateBattery            sensor_msgs/BatteryState   Battery
  /qcar/obstacle_detected       std_msgs/Bool              Lidar detection

Topics published:
  /qcar/safe_stop_active        std_msgs/Bool              dashboard -> safety_mux

Run:
  ros2 launch qcar_pyqt_dashboard qcar_pyqt_dashboard.launch.py
or
  ros2 run qcar_pyqt_dashboard dashboard
"""

import sys
import threading

import numpy as np
import cv2  # only for JPEG decode + depth colormap

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSHistoryPolicy, QoSDurabilityPolicy)

from PyQt5.QtCore import Qt, QObject, pyqtSignal, QPointF, QRectF
from PyQt5.QtGui import (QImage, QPixmap, QPainter, QColor, QPen,
                         QBrush, QFont, QPolygonF)
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QLabel,
                             QPushButton, QGridLayout, QHBoxLayout,
                             QVBoxLayout, QFrame, QProgressBar, QSizePolicy)

from sensor_msgs.msg import Image, LaserScan, BatteryState
from std_msgs.msg import Bool, Float32MultiArray
from geometry_msgs.msg import Vector3Stamped


# ──────────────────────────────────────────────────────────────────────────────
# ROS bridge: subscribes, emits Qt signals.
# ──────────────────────────────────────────────────────────────────────────────

def _be_qos():
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        durability=QoSDurabilityPolicy.VOLATILE,
        depth=1,
    )


class Signals(QObject):
    """QObject that owns the pyqtSignals -- lives on the GUI thread."""
    rawImage = pyqtSignal(object)
    overlayImage = pyqtSignal(object)
    bevImage = pyqtSignal(object)
    maskImage = pyqtSignal(object)
    scanXY = pyqtSignal(object)
    battery = pyqtSignal(float)
    command = pyqtSignal(float, float)  # throttle, steering
    target = pyqtSignal(object)          # (x, y) or (None, None)
    obstacle = pyqtSignal(bool)


class RosNode(Node):
    """rclpy Node. Declares params, subscribes, emits Qt signals."""

    def __init__(self, signals: Signals):
        super().__init__('qcar_pyqt_dashboard')
        self.signals = signals

        # Topics
        self.declare_parameter('raw_topic',     '/qcar/decompressed/csi_front')
        self.declare_parameter('overlay_topic', '/qcar/line_follower/overlay')
        self.declare_parameter('bev_topic',     '/qcar/line_follower/bev')
        self.declare_parameter('mask_topic',    '/qcar/line_follower/mask')
        self.declare_parameter('scan_topic',     '/qcar/scan')
        self.declare_parameter('battery_topic',  '/qcar/stateBattery')
        self.declare_parameter('cmd_topic',      '/qcar/user_command')
        self.declare_parameter('target_topic',   '/lane_target_point_m')
        self.declare_parameter('obstacle_topic', '/qcar/obstacle_detected')
        self.declare_parameter('safe_stop_pub_topic', '/qcar/safe_stop_active')

        # Tunables published to the GUI
        self.declare_parameter('lidar_range_m',     2.0)
        self.declare_parameter('lidar_yaw_offset_deg', -90.0)
        self.declare_parameter('obstacle_zone_depth_m', 0.4)
        self.declare_parameter('obstacle_zone_width_m', 1.2)
        self.declare_parameter('dead_zone_depth_m',     0.2)
        self.declare_parameter('dead_zone_width_m',     0.36)
        self.declare_parameter('team_name',     'guns-n-ROSes')

        self.lidar_range_m      = float(self.get_parameter('lidar_range_m').value)
        self.lidar_yaw_offset_deg = float(
            self.get_parameter('lidar_yaw_offset_deg').value)
        self.obs_zone = (float(self.get_parameter('obstacle_zone_depth_m').value),
                         float(self.get_parameter('obstacle_zone_width_m').value))
        self.dead_zone = (float(self.get_parameter('dead_zone_depth_m').value),
                          float(self.get_parameter('dead_zone_width_m').value))
        self.team_name = str(self.get_parameter('team_name').value)

        be = _be_qos()
        self.create_subscription(Image, self.get_parameter('raw_topic').value,
                                 self._on_raw, be)
        self.create_subscription(Image, self.get_parameter('overlay_topic').value,
                                 self._on_overlay, be)
        self.create_subscription(Image, self.get_parameter('bev_topic').value,
                                 self._on_bev, be)
        self.create_subscription(Image, self.get_parameter('mask_topic').value,
                                 self._on_mask, be)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 self._on_scan, be)
        self.create_subscription(BatteryState,
                                 self.get_parameter('battery_topic').value,
                                 self._on_battery, 10)
        self.create_subscription(Vector3Stamped,
                                 self.get_parameter('cmd_topic').value,
                                 self._on_command, 10)
        self.create_subscription(Float32MultiArray,
                                 self.get_parameter('target_topic').value,
                                 self._on_target, 10)
        self.create_subscription(Bool,
                                 self.get_parameter('obstacle_topic').value,
                                 self._on_obstacle, 10)

        self.safe_stop_pub = self.create_publisher(
            Bool, self.get_parameter('safe_stop_pub_topic').value, 10
        )
        self.get_logger().info('qcar_pyqt_dashboard ROS bridge ready')

    # ── publisher ──────────────────────────────────────────────────────────
    def publish_safe_stop(self, active: bool):
        msg = Bool()
        msg.data = bool(active)
        self.safe_stop_pub.publish(msg)
        self.get_logger().warn(
            f'SAFE STOP {"ENGAGED" if active else "RELEASED"} (PyQt5 dashboard)'
        )

    # ── decoders ───────────────────────────────────────────────────────────
    @staticmethod
    def _decode_raw_image(msg: Image):
        if msg.encoding == 'bgr8':
            return np.frombuffer(msg.data, np.uint8).reshape(
                (msg.height, msg.width, 3)
            )
        if msg.encoding == 'rgb8':
            arr = np.frombuffer(msg.data, np.uint8).reshape(
                (msg.height, msg.width, 3)
            )
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        return None

    # ── callbacks ──────────────────────────────────────────────────────────
    def _on_raw(self, msg):
        img = self._decode_raw_image(msg)
        if img is not None: self.signals.rawImage.emit(img)

    def _on_overlay(self, msg):
        img = self._decode_raw_image(msg)
        if img is not None: self.signals.overlayImage.emit(img)

    def _on_bev(self, msg):
        img = self._decode_raw_image(msg)
        if img is not None: self.signals.bevImage.emit(img)

    def _on_mask(self, msg):
        img = self._decode_raw_image(msg)
        if img is not None: self.signals.maskImage.emit(img)

    def _on_scan(self, msg: LaserScan):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        if ranges.size == 0: return
        n = ranges.size
        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment
        rmax = float(msg.range_max) if msg.range_max > 0 else 30.0
        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < rmax)
        if not np.any(valid): return
        r = ranges[valid]; a = angles[valid]
        xy = np.stack([r * np.cos(a), r * np.sin(a)], axis=1)
        self.signals.scanXY.emit(xy)

    def _on_battery(self, msg):
        self.signals.battery.emit(float(msg.voltage))

    def _on_command(self, msg):
        self.signals.command.emit(float(msg.vector.x), float(msg.vector.y))

    def _on_target(self, msg):
        d = msg.data
        if len(d) >= 2 and d[0] != -1.0 and d[1] != -1.0:
            self.signals.target.emit((float(d[0]), float(d[1])))
        else:
            self.signals.target.emit((None, None))

    def _on_obstacle(self, msg):
        self.signals.obstacle.emit(bool(msg.data))


# ──────────────────────────────────────────────────────────────────────────────
# Widgets
# ──────────────────────────────────────────────────────────────────────────────

def _numpy_bgr_to_qpixmap(arr, target_w=None, target_h=None):
    """Wrap a NumPy BGR array in a QPixmap (letterboxed to target if given)."""
    h, w = arr.shape[:2]
    qimg = QImage(arr.data, w, h, w * 3, QImage.Format_BGR888).copy()
    pix = QPixmap.fromImage(qimg)
    if target_w and target_h:
        pix = pix.scaled(target_w, target_h,
                         Qt.KeepAspectRatio, Qt.SmoothTransformation)
    return pix


# ─── Image panel: header + letterboxed image ─────────────────────────────────
class ImagePanel(QFrame):
    def __init__(self, title):
        super().__init__()
        self.setFrameShape(QFrame.NoFrame)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(2)

        self._title = QLabel(title)
        self._title.setStyleSheet('color: white; font-size: 8.5pt;')
        layout.addWidget(self._title)

        self._image = QLabel('waiting...')
        self._image.setAlignment(Qt.AlignCenter)
        self._image.setStyleSheet('color: #707070; background-color: #050505;')
        self._image.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._image.setMinimumSize(180, 120)
        layout.addWidget(self._image, 1)

        self.setStyleSheet('ImagePanel { background-color: #1a1a1a; '
                           'border: 1px solid #303030; border-radius: 4px; }')

    def set_image(self, arr):
        if arr is None or arr.size == 0: return
        self._image.setPixmap(_numpy_bgr_to_qpixmap(
            arr, max(1, self._image.width()), max(1, self._image.height())))


# ─── Placeholder panel (IMU, ???) ────────────────────────────────────────────
class PlaceholderPanel(QFrame):
    def __init__(self, title, big_text, sub_text=''):
        super().__init__()
        self.setFrameShape(QFrame.NoFrame)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(4)

        t = QLabel(title)
        t.setStyleSheet('color: white; font-size: 9pt;')
        layout.addWidget(t)
        layout.addStretch()

        big = QLabel(big_text)
        big.setAlignment(Qt.AlignCenter)
        big.setStyleSheet('color: #888888; font-size: 14pt;')
        layout.addWidget(big)

        if sub_text:
            sub = QLabel(sub_text)
            sub.setAlignment(Qt.AlignCenter)
            sub.setStyleSheet('color: #666666; font-size: 9pt;')
            layout.addWidget(sub)

        layout.addStretch()
        self.setStyleSheet('PlaceholderPanel { background-color: #1a1a1a; '
                           'border: 1px dashed #404040; border-radius: 4px; }')


# ─── Steering dial + throttle bar ────────────────────────────────────────────
class CommandIndicator(QWidget):
    """Compact widget: vertical throttle bar on the left, steering arc on the right."""

    def __init__(self):
        super().__init__()
        self._throttle = 0.0
        self._steering = 0.0
        self._max_steer = 0.5      # rad, matches lane_follower_q max
        self._max_throttle = 0.3
        self.setMinimumSize(180, 90)

    def set_command(self, throttle, steering):
        self._throttle = throttle
        self._steering = steering
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QColor(20, 20, 20))

        # Steering arc on the right.
        dial_r = min(h // 2 - 6, 40)
        dial_cx = w - dial_r - 12
        dial_cy = h // 2
        p.setPen(QPen(QColor(80, 80, 80), 1))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(dial_cx - dial_r, dial_cy - dial_r, 2 * dial_r, 2 * dial_r)

        clipped = float(np.clip(self._steering, -self._max_steer, self._max_steer))
        angle = -np.pi / 2 + (clipped / self._max_steer) * (np.pi / 2)
        ex = int(dial_cx + dial_r * np.cos(angle))
        ey = int(dial_cy + dial_r * np.sin(angle))
        needle_color = QColor(255, 200, 0) if abs(clipped) > 1e-3 else QColor(0, 200, 0)
        p.setPen(QPen(needle_color, 2))
        p.drawLine(dial_cx, dial_cy, ex, ey)

        # Throttle bar -- vertical, center=0, top=+max_throttle, bottom=-max_throttle.
        bar_x = dial_cx - dial_r - 28
        bar_w = 12
        bar_top = dial_cy - dial_r
        bar_bottom = dial_cy + dial_r
        bar_mid = (bar_top + bar_bottom) // 2
        p.setPen(QPen(QColor(80, 80, 80), 1))
        p.drawRect(bar_x, bar_top, bar_w, bar_bottom - bar_top)
        p.drawLine(bar_x - 2, bar_mid, bar_x + bar_w + 2, bar_mid)

        tt = float(np.clip(self._throttle, -self._max_throttle, self._max_throttle))
        tt /= self._max_throttle
        if tt >= 0:
            top = int(bar_mid - tt * (bar_mid - bar_top))
            fill = QRectF(bar_x + 1, top, bar_w - 2, bar_mid - top)
            p.fillRect(fill, QColor(0, 200, 0))
        else:
            bot = int(bar_mid - tt * (bar_bottom - bar_mid))
            fill = QRectF(bar_x + 1, bar_mid, bar_w - 2, bot - bar_mid)
            p.fillRect(fill, QColor(220, 0, 0))


# ─── Status panel: throttle, steering, motion, wheels, target + indicator ────
class StatusPanel(QFrame):
    def __init__(self):
        super().__init__()
        self.setFrameShape(QFrame.NoFrame)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(4)

        title = QLabel('QCar command')
        title.setStyleSheet('color: white; font-size: 11pt; font-weight: bold;')
        outer.addWidget(title)

        # Text rows
        self._throttle = QLabel('throttle:  --')
        self._steering = QLabel('steering:  --')
        self._motion   = QLabel('motion:    --')
        self._wheels   = QLabel('wheels:    --')
        self._target   = QLabel('target xy: --')
        for lbl in (self._throttle, self._steering,
                    self._motion, self._wheels, self._target):
            lbl.setStyleSheet('color: #cccccc; font-size: 10pt; '
                              'font-family: monospace;')
            outer.addWidget(lbl)

        # Visual indicator at the bottom
        self._indicator = CommandIndicator()
        outer.addWidget(self._indicator, 1)

        self.setStyleSheet('StatusPanel { background-color: #1a1a1a; '
                           'border: 1px solid #303030; border-radius: 4px; }')

    def set_command(self, throttle, steering):
        self._throttle.setText(f'throttle:  {throttle:+.3f}')
        self._steering.setText(
            f'steering:  {steering:+.3f} rad ({np.degrees(steering):+.1f}°)'
        )
        if throttle > 0.005:
            motion, mcol = 'FORWARD', '#00cc00'
        elif throttle < -0.005:
            motion, mcol = 'REVERSE', '#ff5050'
        else:
            motion, mcol = 'STOP', '#888888'
        self._motion.setText(f'motion:    {motion}')
        self._motion.setStyleSheet(
            f'color: {mcol}; font-size: 10pt; font-family: monospace;')

        if steering > 0.02:
            w, wcol = 'LEFT', '#00aaff'
        elif steering < -0.02:
            w, wcol = 'RIGHT', '#00aaff'
        else:
            w, wcol = 'STRAIGHT', '#888888'
        self._wheels.setText(f'wheels:    {w}')
        self._wheels.setStyleSheet(
            f'color: {wcol}; font-size: 10pt; font-family: monospace;')

        self._indicator.set_command(throttle, steering)

    def set_target(self, target):
        x, y = target
        if x is None:
            self._target.setText('target xy: no lane')
            self._target.setStyleSheet('color: #888888; font-size: 10pt; '
                                       'font-family: monospace;')
        else:
            self._target.setText(f'target xy: x={x:+.3f} y={y:+.3f}')
            self._target.setStyleSheet('color: #00cccc; font-size: 10pt; '
                                       'font-family: monospace;')


# ─── Battery panel ───────────────────────────────────────────────────────────
class BatteryPanel(QFrame):
    def __init__(self):
        super().__init__()
        self.setFrameShape(QFrame.NoFrame)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        title = QLabel('Battery')
        title.setStyleSheet('color: white; font-size: 11pt; font-weight: bold;')
        layout.addWidget(title)

        self._voltage = QLabel('-- V')
        self._voltage.setAlignment(Qt.AlignCenter)
        self._voltage.setStyleSheet('color: #aaaaaa; font-size: 24pt; '
                                    'font-weight: bold;')
        layout.addWidget(self._voltage)

        self._bar = QProgressBar()
        self._bar.setRange(0, 100); self._bar.setValue(0)
        self._bar.setTextVisible(False); self._bar.setFixedHeight(14)
        layout.addWidget(self._bar)

        self._status = QLabel('no reading')
        self._status.setAlignment(Qt.AlignCenter)
        self._status.setStyleSheet('color: #aaaaaa; font-size: 9pt;')
        layout.addWidget(self._status)
        layout.addStretch()

        self.setStyleSheet('BatteryPanel { background-color: #1a1a1a; '
                           'border: 1px solid #303030; border-radius: 4px; }')

    def set_voltage(self, v):
        V_EMPTY, V_FULL = 9.0, 12.6
        self._voltage.setText(f'{v:.2f} V')
        pct = int(np.clip((v - V_EMPTY) / (V_FULL - V_EMPTY), 0.0, 1.0) * 100)
        self._bar.setValue(pct)
        if v >= 11.5:
            color, status = '#00cc00', 'GOOD'
        elif v >= 10.5:
            color, status = '#cccc00', 'OK'
        else:
            color, status = '#ff4040', 'LOW -- CHARGE'
        self._voltage.setStyleSheet(
            f'color: {color}; font-size: 24pt; font-weight: bold;')
        self._status.setText(status)
        self._status.setStyleSheet(f'color: {color}; font-size: 9pt;')
        # All fragments need to be f-strings so that '{' and '}' escape rules
        # are consistent across the whole concatenation. Mixing a plain string
        # with an f-string used to leave a stray '}' at the end and produce
        # "Could not parse stylesheet" in the log.
        self._bar.setStyleSheet(
            f'QProgressBar {{ background-color: #303030; '
            f'border: 1px solid #505050; border-radius: 3px; }} '
            f'QProgressBar::chunk {{ background-color: {color}; '
            f'border-radius: 2px; }}'
        )


# ─── LiDAR panel ─────────────────────────────────────────────────────────────
class LidarPanel(QFrame):
    def __init__(self, title='LiDAR /qcar/scan', range_m=2.0, yaw_deg=-90.0):
        super().__init__()
        self.setFrameShape(QFrame.NoFrame)
        self.setMinimumSize(220, 220)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._title = title
        self._scan = None
        self._range_m = range_m
        yaw = np.radians(yaw_deg)
        self._cos_yaw = float(np.cos(yaw))
        self._sin_yaw = float(np.sin(yaw))
        self.setStyleSheet('LidarPanel { background-color: #0a0a0a; '
                           'border: 1px solid #303030; border-radius: 4px; }')

    def set_scan(self, xy):
        self._scan = xy
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        w, h = self.width(), self.height()
        cx, cy = w // 2, h // 2
        scale = (min(w, h) // 2 - 22) / max(self._range_m, 1e-3)

        p.setPen(QColor(220, 220, 220))
        p.setFont(QFont('sans-serif', 8))
        p.drawText(8, 14, f'{self._title}  (R={self._range_m:.1f} m)')

        p.setPen(QPen(QColor(55, 55, 55), 1))
        for r in range(1, int(np.ceil(self._range_m)) + 1):
            radius = int(r * scale)
            if radius < 4: continue
            p.drawEllipse(cx - radius, cy - radius, 2 * radius, 2 * radius)
        ext = int(self._range_m * scale)
        p.drawLine(cx, cy - ext, cx, cy + ext)
        p.drawLine(cx - ext, cy, cx + ext, cy)

        p.setBrush(QBrush(QColor(0, 200, 0))); p.setPen(Qt.NoPen)
        p.drawPolygon(QPolygonF([QPointF(cx, cy - 6),
                                 QPointF(cx - 5, cy + 4),
                                 QPointF(cx + 5, cy + 4)]))

        if self._scan is not None and self._scan.size:
            xs = self._scan[:, 0]; ys = self._scan[:, 1]
            c, s = self._cos_yaw, self._sin_yaw
            xs_rot = c * xs + s * ys; ys_rot = -s * xs + c * ys
            px = (cx - ys_rot * scale).astype(np.int32)
            py = (cy - xs_rot * scale).astype(np.int32)
            inside = (px >= 0) & (px < w) & (py >= 0) & (py < h)
            px = px[inside]; py = py[inside]
            dist = np.hypot(xs[inside], ys[inside])
            t = np.clip(dist / max(self._range_m, 1e-3), 0.0, 1.0)
            p.setPen(Qt.NoPen)
            for i in range(px.size):
                ti = t[i]
                p.setBrush(QBrush(QColor(int(ti * 255),
                                         int(ti * 200 + 40),
                                         int((1 - ti) * 255))))
                p.drawRect(px[i] - 1, py[i] - 1, 3, 3)


# ─── Lidar detection (scan + safety zones + OBSTACLE/CLEAR) ──────────────────
class LidarDetectionPanel(QFrame):
    def __init__(self, obs_zone, dead_zone, yaw_deg=-90.0):
        super().__init__()
        self.setFrameShape(QFrame.NoFrame)
        self.setMinimumSize(220, 220)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._obs = obs_zone
        self._dead = dead_zone
        self._scan = None
        self._obstacle = False
        yaw = np.radians(yaw_deg)
        self._cos_yaw = float(np.cos(yaw))
        self._sin_yaw = float(np.sin(yaw))
        self.setStyleSheet('LidarDetectionPanel { background-color: #0a0a0a; '
                           'border: 1px solid #303030; border-radius: 4px; }')

    def set_scan(self, xy):
        self._scan = xy
        self.update()

    def set_obstacle(self, obstacle):
        self._obstacle = obstacle
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        w, h = self.width(), self.height()
        cx, cy = w // 2, h // 2

        obs_depth, obs_width = self._obs
        max_extent = max(obs_depth, obs_width) * 1.2
        scale = (min(w, h) // 2 - 24) / max(max_extent, 1e-3)

        c, s = self._cos_yaw, self._sin_yaw

        # Title
        p.setPen(QColor(220, 220, 220))
        p.setFont(QFont('sans-serif', 8))
        p.drawText(8, 14, 'Lidar detection /qcar/obstacle_detected')

        # Dim scan points
        if self._scan is not None and self._scan.size:
            xs = self._scan[:, 0]; ys = self._scan[:, 1]
            xs_rot = c * xs + s * ys; ys_rot = -s * xs + c * ys
            px = (cx - ys_rot * scale).astype(np.int32)
            py = (cy - xs_rot * scale).astype(np.int32)
            inside = (px >= 0) & (px < w) & (py >= 0) & (py < h)
            px = px[inside]; py = py[inside]
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(QColor(90, 90, 90)))
            for i in range(px.size):
                p.drawRect(px[i] - 1, py[i] - 1, 2, 2)

        # Safety zones (rotated rectangles).
        def rect_poly(depth, width):
            hd, hw = depth / 2.0, width / 2.0
            pts = np.array([[ hd,  hw], [ hd, -hw],
                            [-hd, -hw], [-hd,  hw]], dtype=np.float32)
            x = pts[:, 0]; y = pts[:, 1]
            xr = c * x + s * y; yr = -s * x + c * y
            sx = (cx - yr * scale).astype(np.int32)
            sy = (cy - xr * scale).astype(np.int32)
            return QPolygonF([QPointF(float(sx[i]), float(sy[i]))
                              for i in range(4)])

        obstacle_color = QColor(220, 0, 0) if self._obstacle else QColor(0, 200, 0)
        deadzone_color = QColor(220, 140, 0)

        # Translucent fills.
        fill_obs = QColor(obstacle_color); fill_obs.setAlpha(75)
        fill_dead = QColor(deadzone_color); fill_dead.setAlpha(75)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(fill_obs)); p.drawPolygon(rect_poly(*self._obs))
        p.setBrush(QBrush(fill_dead)); p.drawPolygon(rect_poly(*self._dead))

        # Outlines.
        p.setPen(QPen(obstacle_color, 1)); p.setBrush(Qt.NoBrush)
        p.drawPolygon(rect_poly(*self._obs))
        p.setPen(QPen(deadzone_color, 1))
        p.drawPolygon(rect_poly(*self._dead))

        # Car triangle.
        p.setPen(Qt.NoPen); p.setBrush(QBrush(QColor(0, 200, 0)))
        p.drawPolygon(QPolygonF([QPointF(cx, cy - 6),
                                 QPointF(cx - 5, cy + 4),
                                 QPointF(cx + 5, cy + 4)]))

        # Big OBSTACLE / CLEAR.
        if self._obstacle:
            text, color = 'OBSTACLE', QColor(230, 0, 0)
        else:
            text, color = 'CLEAR', QColor(0, 220, 0)
        font = QFont('sans-serif', 14, QFont.Bold)
        p.setFont(font); p.setPen(color)
        metrics = p.fontMetrics()
        tw = metrics.horizontalAdvance(text)
        p.drawText((w - tw) // 2, h - 12, text)


# ─── Safe-stop button ────────────────────────────────────────────────────────
class SafeStopButton(QPushButton):
    def __init__(self):
        super().__init__('SAFE STOP: off')
        self.setCheckable(True)
        self.setMinimumHeight(54)
        self.setMinimumWidth(260)
        self.toggled.connect(self._restyle)
        self._restyle()

    def _restyle(self):
        if self.isChecked():
            self.setText('SAFE STOP: ENGAGED')
            self.setStyleSheet(
                'QPushButton {'
                '  background-color: #c40000; color: white;'
                '  font-size: 16pt; font-weight: bold;'
                '  border-radius: 8px; border: 2px solid #ff5050;'
                '} QPushButton:hover { background-color: #e60000; }'
            )
        else:
            self.setText('SAFE STOP: off')
            self.setStyleSheet(
                'QPushButton {'
                '  background-color: #303030; color: #dddddd;'
                '  font-size: 13pt; border-radius: 8px;'
                '  border: 2px solid #606060;'
                '} QPushButton:hover {'
                '  background-color: #404040; border-color: #909090;'
                '}'
            )


# ──────────────────────────────────────────────────────────────────────────────
# Main window
# ──────────────────────────────────────────────────────────────────────────────

class MainWindow(QMainWindow):
    def __init__(self, signals: Signals, ros: RosNode):
        super().__init__()
        self.signals = signals
        self.ros = ros

        self.setWindowTitle(f'{ros.team_name} -- QCar dashboard (PyQt5)')
        self.setStyleSheet('QMainWindow { background-color: #121212; }')
        self.resize(1180, 880)

        central = QWidget()
        self.setCentralWidget(central)
        outer = QVBoxLayout(central)
        outer.setContentsMargins(10, 10, 10, 10)
        outer.setSpacing(10)

        # ── Title bar ──
        title_bar = QHBoxLayout()
        team = QLabel(ros.team_name)
        team.setStyleSheet('color: white; font-size: 22pt; font-weight: bold;')
        title_bar.addWidget(team)

        subtitle = QLabel('  QCar dashboard / PyQt5')
        subtitle.setStyleSheet('color: #888888; font-size: 11pt;')
        title_bar.addWidget(subtitle)
        title_bar.addStretch()

        self.safe_stop_btn = SafeStopButton()
        self.safe_stop_btn.toggled.connect(self._on_safe_stop)
        title_bar.addWidget(self.safe_stop_btn)
        outer.addLayout(title_bar)

        # ── 3 x 3 grid ──
        grid = QGridLayout()
        grid.setSpacing(8)

        self.processed_p = ImagePanel('Processed /qcar/line_follower/overlay')
        self.status_p    = StatusPanel()
        self.lidar_p     = LidarPanel(range_m=ros.lidar_range_m,
                                      yaw_deg=ros.lidar_yaw_offset_deg)

        self.mask_p      = ImagePanel('ColorSelect /qcar/line_follower/mask')
        self.imu_p       = PlaceholderPanel('IMU', 'IMU + encoders',
                                            'PCB / ESP32 firmware pending')
        self.csi_p       = ImagePanel('Csi_front /qcar/decompressed/csi_front')

        self.bev_p       = ImagePanel('BEV /qcar/line_follower/bev')
        self.battery_p   = BatteryPanel()
        self.lidar_det_p = LidarDetectionPanel(ros.obs_zone, ros.dead_zone,
                                               yaw_deg=ros.lidar_yaw_offset_deg)

        grid.addWidget(self.processed_p, 0, 0)
        grid.addWidget(self.status_p,    0, 1)
        grid.addWidget(self.lidar_p,     0, 2)

        grid.addWidget(self.mask_p,      1, 0)
        grid.addWidget(self.imu_p,       1, 1)
        grid.addWidget(self.csi_p,       1, 2)

        grid.addWidget(self.bev_p,       2, 0)
        grid.addWidget(self.battery_p,   2, 1)
        grid.addWidget(self.lidar_det_p, 2, 2)

        outer.addLayout(grid, 1)

        # ── Signal wiring ──
        signals.rawImage.connect(self.csi_p.set_image)
        signals.overlayImage.connect(self.processed_p.set_image)
        signals.bevImage.connect(self.bev_p.set_image)
        signals.maskImage.connect(self.mask_p.set_image)
        signals.scanXY.connect(self.lidar_p.set_scan)
        signals.scanXY.connect(self.lidar_det_p.set_scan)
        signals.battery.connect(self.battery_p.set_voltage)
        signals.command.connect(self.status_p.set_command)
        signals.target.connect(self.status_p.set_target)
        signals.obstacle.connect(self.lidar_det_p.set_obstacle)

    def _on_safe_stop(self, checked):
        self.ros.publish_safe_stop(checked)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Space:
            self.safe_stop_btn.toggle()
        elif event.key() in (Qt.Key_Q, Qt.Key_Escape):
            self.close()
        else:
            super().keyPressEvent(event)


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    app = QApplication(sys.argv)

    signals = Signals()
    ros = RosNode(signals)

    spin_thread = threading.Thread(target=rclpy.spin, args=(ros,), daemon=True)
    spin_thread.start()

    window = MainWindow(signals, ros)
    window.show()

    exit_code = app.exec_()
    ros.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
