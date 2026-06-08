#!/usr/bin/env python3
"""
obstacle_dodge.py — Esquive REACTIVO por lidar (sin IMU, sin yaw)
=================================================================

El IMU se congela => NO se usa nada de IMU/odom/yaw. Todo es lidar + cámara.

Idea (reactivo, cada frame):
  1. Cono enfrente: si hay objeto a <= trigger_dist del MORRO, el dodge toma
     el control (source=overtake).
  2. RODEAR: cada frame el lidar ve el objeto; el coche DIRIGE hacia el hueco
     al lado elegido, apuntando un punto justo MÁS ALLÁ del borde del objeto
     (+ holgura). Dirección proporcional al rumbo de ese punto. No integra
     nada: reacciona al objeto que ve AHORA.
  3. Cuando el objeto ya no está enfrente (lo pasó), SUELTA el control
     (source=lane) y el SEGUIDOR DE LÍNEA (cámara) recupera el carril.

Sensores: SOLO lidar (rodear) + cámara vía el lane follower (recuperar).
Salida (por el mux): /overtake/raw_cmd (x=vel, y=dir) + /qcar/control_source.
"""

import math
import statistics
from collections import deque
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSHistoryPolicy, QoSDurabilityPolicy)
from geometry_msgs.msg import Vector3Stamped, Point, TransformStamped
from std_msgs.msg import Bool, String
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import StaticTransformBroadcaster


class S(Enum):
    IDLE  = auto()   # seguidor de línea; vigila el cono
    ROUND = auto()   # rodea el objeto (reactivo por lidar)
    DONE  = auto()   # suelta al seguidor (cámara recupera)


class ObstacleDodge(Node):

    def __init__(self):
        super().__init__('obstacle_dodge')

        # Geometría del coche (del dead zone del lidar).
        self.declare_parameter('d_lf',     0.18)    # lidar -> morro
        self.declare_parameter('car_half', 0.10)    # medio ancho del coche

        # Detección (cono).
        self.declare_parameter('cone_half_deg', 23.0)
        self.declare_parameter('reach',         0.60)   # alcance del cono (m, desde lidar)
        self.declare_parameter('trigger_dist',  0.48)   # m DESDE EL MORRO para disparar
        self.declare_parameter('min_pts',       2)
        self.declare_parameter('confirm_cycles', 3)
        self.declare_parameter('clear_cycles',   4)     # ciclos sin objeto = pasado

        # Rodeo (reactivo).
        self.declare_parameter('pass_side',    1)       # +1 izq, -1 der
        self.declare_parameter('margin',       0.06)    # holgura extra al borde del objeto
        self.declare_parameter('kp_steer',     1.2)     # ganancia dir = kp * rumbo_al_hueco
        self.declare_parameter('max_steer',    0.30)    # tope físico de la dirección
        self.declare_parameter('steering_sign', -1.0)   # igual que el lane follower
        self.declare_parameter('v_dodge',      0.075)
        self.declare_parameter('maneuver_timeout', 12.0)

        # Lidar / frame.
        self.declare_parameter('front_angle_deg', -90.0)
        self.declare_parameter('scan_frame', 'lidar_corrected')
        self.declare_parameter('scan_filter_size', 3)   # mediana por haz (anti-fantasma)
        self.declare_parameter('rate_hz',    15.0)
        self.declare_parameter('publish_markers', True)

        g = lambda n: self.get_parameter(n).value
        self.d_lf       = float(g('d_lf'))
        self.car_half   = float(g('car_half'))
        self.cone_half  = math.radians(float(g('cone_half_deg')))
        self.reach      = float(g('reach'))
        self.trig_dist  = float(g('trigger_dist'))
        self.min_pts    = int(g('min_pts'))
        self.confirm_n  = int(g('confirm_cycles'))
        self.clear_n    = int(g('clear_cycles'))
        self.pass_side  = int(g('pass_side'))
        self.margin     = float(g('margin'))
        self.kp_steer   = float(g('kp_steer'))
        self.max_steer  = float(g('max_steer'))
        self.steer_sign = float(g('steering_sign'))
        self.v_dodge    = float(g('v_dodge'))
        self.man_to     = float(g('maneuver_timeout'))
        self._front     = math.radians(float(g('front_angle_deg')))
        self._cf, self._sf = math.cos(self._front), math.sin(self._front)
        self._scan_frame = str(g('scan_frame'))
        self._filt_n    = max(1, int(g('scan_filter_size')))
        self._pub_mk    = bool(g('publish_markers'))
        rate_hz         = float(g('rate_hz'))
        self._mk_child  = 'dodge_fwd'

        # Estado.
        self._state = S.IDLE
        self._scan = None
        self._scan_buf = deque(maxlen=self._filt_n)
        self._count = 0
        self._clear = 0
        self._t_start = None
        self._aim = None            # (fwd, lat) punto al que apunta (para viz)
        self._last_steer = 0.0

        qos_be = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                            history=QoSHistoryPolicy.KEEP_LAST,
                            durability=QoSDurabilityPolicy.VOLATILE, depth=10)

        self._pub_cmd     = self.create_publisher(Vector3Stamped, '/overtake/raw_cmd', 10)
        self._pub_source  = self.create_publisher(String, '/qcar/control_source', 10)
        self._pub_markers = self.create_publisher(MarkerArray, '/overtake/markers', 10)

        self.create_subscription(LaserScan, '/qcar/scan_republished', self._on_scan, qos_be)

        self._tf = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self._scan_frame
        tf.child_frame_id = self._mk_child
        tf.transform.rotation.z = math.sin(self._front / 2.0)
        tf.transform.rotation.w = math.cos(self._front / 2.0)
        self._tf.sendTransform(tf)

        self.create_timer(1.0 / rate_hz, self._loop)
        self.get_logger().info(
            f'obstacle_dodge (REACTIVO lidar) | cono ±{math.degrees(self.cone_half):.0f}° '
            f'dispara a {self.trig_dist} m del morro | rodea por '
            f'{"IZQ" if self.pass_side > 0 else "DER"} | v={self.v_dodge}')

    # ── Callback ─────────────────────────────────────────────────────────────
    def _on_scan(self, msg):
        self._scan = msg
        r = list(msg.ranges)
        if self._scan_buf and len(self._scan_buf[-1]) != len(r):
            self._scan_buf.clear()
        self._scan_buf.append(r)

    # ── Lidar ────────────────────────────────────────────────────────────────
    def _filtered_ranges(self):
        """Mediana por haz sobre los scans recientes (descarta fantasmas)."""
        buf = self._scan_buf
        if not buf:
            return None
        if len(buf) == 1:
            return buf[0]
        n = len(buf[-1])
        out = []
        for i in range(n):
            vals = [(b[i] if math.isfinite(b[i]) else math.inf)
                    for b in buf if len(b) == n]
            out.append(statistics.median(vals))
        return out

    def _points(self):
        s = self._scan
        ranges = self._filtered_ranges()
        if s is None or ranges is None:
            return []
        pts, ang = [], s.angle_min
        for r in ranges:
            if math.isfinite(r) and s.range_min <= r <= s.range_max:
                xr, yr = r * math.cos(ang), r * math.sin(ang)
                fwd = xr * self._cf + yr * self._sf
                lat = -xr * self._sf + yr * self._cf
                pts.append((fwd, lat))
            ang += s.angle_increment
        return pts

    def _cone(self, pts):
        """Objeto en el cono frontal. (present, dist_morro, sel)."""
        sel = [(f, l) for (f, l) in pts
               if self.d_lf < f <= self.d_lf + self.reach
               and abs(math.atan2(l, f)) <= self.cone_half]
        if len(sel) < self.min_pts:
            return False, None, sel
        d_morro = min(f for f, _ in sel) - self.d_lf
        return True, d_morro, sel

    def _round_target(self, sel):
        """Punto al que apuntar: justo MÁS ALLÁ del borde del objeto del lado
        de paso, + holgura. Devuelve (fwd, lat)."""
        clear = self.car_half + self.margin
        if self.pass_side > 0:                 # izquierda: borde más a la izq (lat máx)
            edge = max(l for _, l in sel)
            aim_lat = edge + clear
        else:                                  # derecha: borde más a la der (lat mín)
            edge = min(l for _, l in sel)
            aim_lat = edge - clear
        aim_fwd = max(self.d_lf + 0.05, min(f for f, _ in sel))
        return aim_fwd, aim_lat

    # ── Salidas ──────────────────────────────────────────────────────────────
    def _src(self, s):
        m = String(); m.data = s; self._pub_source.publish(m)

    def _cmd(self, v, steer):
        m = Vector3Stamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'obstacle_dodge'
        m.vector.x, m.vector.y, m.vector.z = float(v), float(steer), 0.0
        self._last_steer = float(steer)
        self._pub_cmd.publish(m)

    def _go(self, ns):
        if ns != self._state:
            self.get_logger().info(f'[{self._state.name}] -> [{ns.name}]')
            self._state = ns

    # ── Loop ─────────────────────────────────────────────────────────────────
    def _loop(self):
        now = self.get_clock().now()
        in_maneuver = self._state == S.ROUND
        if in_maneuver and self._t_start is not None and \
                (now - self._t_start).nanoseconds * 1e-9 > self.man_to:
            self.get_logger().warn('TIMEOUT -> suelto al seguidor')
            self._release(); return

        pts = self._points()
        if self._pub_mk:
            self._markers(pts)

        if self._state == S.IDLE:
            self._src('lane')
            self._aim = None
            present, d, _ = self._cone(pts)
            hit = present and d is not None and d <= self.trig_dist
            self._count = self._count + 1 if hit else 0
            if self._count >= self.confirm_n:
                self._t_start = now
                self._clear = 0
                self._src('overtake')
                self.get_logger().info(
                    f'objeto a {d:.2f} m -> RODEAR por '
                    f'{"IZQ" if self.pass_side > 0 else "DER"} (reactivo lidar)')
                self._go(S.ROUND)

        elif self._state == S.ROUND:
            present, d, sel = self._cone(pts)
            if not present:
                # objeto ya no está enfrente -> ¿lo pasamos?
                self._clear += 1
                self._cmd(self.v_dodge, 0.0)
                if self._clear >= self.clear_n:
                    self.get_logger().info('objeto pasado -> suelto al seguidor (cámara recupera)')
                    self._release()
                return
            self._clear = 0
            aim_fwd, aim_lat = self._round_target(sel)
            self._aim = (aim_fwd, aim_lat)
            bearing = math.atan2(aim_lat, aim_fwd)
            steer = self.steer_sign * max(-self.max_steer,
                                          min(self.max_steer, self.kp_steer * bearing))
            self._cmd(self.v_dodge, steer)
            self._log(d, aim_lat, bearing, steer)

        elif self._state == S.DONE:
            self._src('lane')
            self._count = 0
            self._t_start = None
            self._go(S.IDLE)

    def _release(self):
        self._src('lane')
        self._count = 0
        self._clear = 0
        self._t_start = None
        self._aim = None
        self._go(S.IDLE)

    def _log(self, d, aim_lat, bearing, steer):
        n = self.get_clock().now()
        if getattr(self, '_last_log', None) is not None and \
                (n - self._last_log).nanoseconds < 4e8:
            return
        self._last_log = n
        self.get_logger().info(
            f'[ROUND] obj={d:.2f}m aim_lat={aim_lat:+.2f} '
            f'rumbo={math.degrees(bearing):+.0f}° steer={steer:+.2f}')

    # ── Markers ──────────────────────────────────────────────────────────────
    def _markers(self, pts):
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()

        # Cono.
        r = self.d_lf + self.reach
        edge = r * math.tan(self.cone_half)
        cone = [(self.d_lf, 0.0), (r, +edge), (r, -edge), (self.d_lf, 0.0)]
        m = Marker()
        m.header.frame_id = self._mk_child; m.header.stamp = stamp
        m.ns = 'cone'; m.id = 0; m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.scale.x = 0.012; m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = (1.0, 0.9, 0.1, 0.95)
        m.points = [Point(x=float(x), y=float(y), z=0.0) for (x, y) in cone]
        arr.markers.append(m)

        # Objeto.
        present, d, sel = self._cone(pts)
        mp = Marker()
        mp.header.frame_id = self._mk_child; mp.header.stamp = stamp
        mp.ns = 'obj'; mp.id = 1; mp.type = Marker.SPHERE_LIST
        mp.action = Marker.ADD if present else Marker.DELETE
        mp.scale.x = mp.scale.y = mp.scale.z = 0.03
        mp.color.r, mp.color.g, mp.color.b, mp.color.a = (1.0, 0.1, 0.1, 0.9)
        mp.points = [Point(x=float(f), y=float(l), z=0.0) for (f, l) in sel]
        mp.pose.orientation.w = 1.0
        arr.markers.append(mp)

        # Punto al que apunta (hueco) — esfera verde.
        ma = Marker()
        ma.header.frame_id = self._mk_child; ma.header.stamp = stamp
        ma.ns = 'aim'; ma.id = 2; ma.type = Marker.SPHERE
        ma.action = Marker.ADD if self._aim is not None else Marker.DELETE
        if self._aim is not None:
            ma.pose.position.x = float(self._aim[0])
            ma.pose.position.y = float(self._aim[1])
        ma.pose.position.z = 0.05; ma.pose.orientation.w = 1.0
        ma.scale.x = ma.scale.y = ma.scale.z = 0.07
        ma.color.r, ma.color.g, ma.color.b, ma.color.a = (0.1, 1.0, 0.1, 0.9)
        arr.markers.append(ma)

        # Estado.
        mt = Marker()
        mt.header.frame_id = self._mk_child; mt.header.stamp = stamp
        mt.ns = 'state'; mt.id = 3; mt.type = Marker.TEXT_VIEW_FACING; mt.action = Marker.ADD
        mt.pose.position.x = r + 0.1; mt.pose.position.z = 0.15; mt.pose.orientation.w = 1.0
        mt.scale.z = 0.08
        mt.color.r, mt.color.g, mt.color.b, mt.color.a = (1.0, 1.0, 1.0, 0.9)
        mt.text = self._state.name + (f' obj={d:.2f}' if present else '')
        arr.markers.append(mt)

        self._pub_markers.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDodge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
