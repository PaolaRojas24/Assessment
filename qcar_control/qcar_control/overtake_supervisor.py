#!/usr/bin/env python3
"""
overtake_supervisor.py — Supervisor de rebase (Etapa 2: solo rectas)
====================================================================

FSM que decide cuándo rebasar un obstáculo del carril y ejecuta la maniobra
de desplazamiento lateral. NO toca la seguridad: la parada de emergencia la
sigue haciendo command_mux a partir de /qcar/obstacle_detected.

Percepción por ZONAS en capas (todo en frame del robot, frente=+x tras
corregir el montaje rotado del lidar con front_angle_deg):
  Z0 Emergencia : caja del lidar_node. Solo debug visual (no se actúa).
  Z1 Lejana     : corredor frontal largo. Objeto lejos -> "ralentizar"
                  (por ahora SOLO debug/log, no frena).
  Z2 Cercana    : corredor frontal corto. Objeto aquí -> decidir maniobra.
  Z3 Lado obj   : centroide del objeto -> ¿izquierda o derecha?
  Z4 Pasos      : corredores izq/der (resto del mapa) -> qué lado está libre.

Decisión de lado: paso por el lado LIBRE, preferentemente el OPUESTO a donde
está el objeto. Si ninguno libre (o curva) -> HOLD.

Reparto de sensores:
  - lidar : detectar, localizar el objeto, elegir lado, "al costado/atrás".
  - odom  : yaw (fiable, IMU) para el rumbo; offset n (aprox) para la forma.
  - cámara: la vuelta al carril (REACQUIRE).
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
from std_msgs.msg import Bool, String, Float32MultiArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import StaticTransformBroadcaster


class State(Enum):
    LANE_FOLLOW = auto()
    CONFIRM     = auto()
    SHIFT_OUT   = auto()
    SHIFT_DONE  = auto()   # test: se detiene tras salir, para inspección
    ALONGSIDE   = auto()
    SHIFT_BACK  = auto()
    REACQUIRE   = auto()
    HOLD        = auto()


class OvertakeSupervisor(Node):

    def __init__(self):
        super().__init__('overtake_supervisor')

        # ── Geometría de pista / coche (Etapa 0) ──────────────────────────────
        self.declare_parameter('lane_width',    0.25)
        self.declare_parameter('n_paso',        0.25)
        self.declare_parameter('corner_radius', 1.0)    # (Etapa 3)

        # ── Zonas de detección (m, en frente del robot) ───────────────────────
        # La maniobra se DECIDE en [near_x, far_x], que debe quedar MÁS ALLÁ de
        # la zona de emergencia del lidar (~0.32 m) para que dé tiempo a
        # correrse antes de que el obstáculo entre en la caja de emergencia.
        # Geometría del coche derivada del dead zone del lidar (comprobada ±1cm):
        # dead zone 0.36×0.20 -> lidar al centro, morro a 0.18 m, medio ancho 0.10.
        self.declare_parameter('d_lf',          0.18)   # lidar -> parachoques delantero
        self.declare_parameter('car_half',      0.10)   # medio ancho del coche
        # Zonas de detección (m, frente del robot). La maniobra se DECIDE en
        # [near_x, far_x]; near_x = emerg_end(0.32) + d_clear(0.45, δm=0.4).
        self.declare_parameter('dead_x',        0.20)   # inicio detección = morro + margen
        self.declare_parameter('near_x',        0.77)   # inicio zona de decisión
        self.declare_parameter('far_x',         1.07)   # fin zona de decisión
        self.declare_parameter('min_obj_pts',   2)
        self.declare_parameter('side_check_len', 0.90)  # largo corredor de paso (cubre el obstáculo)
        self.declare_parameter('beside_len',    0.30)   # ventana |x| para "al costado"

        # ── Montaje del lidar ─────────────────────────────────────────────────
        # front_angle_deg: dirección del FRENTE del robot en el frame del scan
        # (mismo valor que FRONT_ANGLE_DEG de lidar_node; el RPLidar va a -90).
        self.declare_parameter('front_angle_deg', -90.0)
        self.declare_parameter('scan_frame',      'lidar_corrected')

        # Filtro temporal anti-fantasma: mediana por haz sobre M scans.
        self.declare_parameter('scan_filter_size', 3)

        # ── Zona de emergencia (solo debug visual; igualar a lidar_node) ──────
        self.declare_parameter('emerg_depth', 0.32)   # OBS_DEPTH de lidar_node
        self.declare_parameter('emerg_width', 0.20)   # OBS_WIDTH_NEAR de lidar_node

        # ── Ciclos de confirmación ────────────────────────────────────────────
        self.declare_parameter('confirm_cycles',   5)
        self.declare_parameter('clear_cycles',     5)
        self.declare_parameter('reacquire_frames', 5)

        # ── Recta/curva ───────────────────────────────────────────────────────
        self.declare_parameter('curve_omega', 0.12)
        self.declare_parameter('check_curve', False)  # veto de curva por ω desactivado (IMU ruidoso)

        # ── Control lateral / actuación ───────────────────────────────────────
        self.declare_parameter('k_n',           2.0)
        self.declare_parameter('k_theta',       1.0)
        self.declare_parameter('steering_sign', -1.0)  # igual que el lane follower
        self.declare_parameter('yaw_sign',       1.0)  # -1 si Δθ va al revés
        self.declare_parameter('eps_n',          0.04)
        self.declare_parameter('v_overtake',     0.10)
        self.declare_parameter('ramp_time',      1.5)
        self.declare_parameter('maneuver_max_steer', 0.40)  # δm de maniobra (equilibrio)
        # SHIFT_OUT por YAW (sin distancia): gira turn_steer hacia el lado hasta
        # theta_out grados, luego endereza (yaw->0) para quedar paralelo.
        self.declare_parameter('turn_steer',     0.40)   # dirección durante el giro
        self.declare_parameter('theta_out_deg',  35.0)   # cuánto girar al salir
        self.declare_parameter('straighten_eps_deg', 5.0)  # tolerancia para "paralelo"
        self.declare_parameter('maneuver_timeout',   8.0)   # s, aborta si no completa

        self.declare_parameter('rate_hz', 15.0)

        # Modo seguro: ejecuta lógica y registra, pero NO conmuta a overtake.
        self.declare_parameter('dry_run', True)
        self.declare_parameter('publish_markers', True)

        g = lambda n: self.get_parameter(n).value
        self.lane_w     = float(g('lane_width'))
        self.n_paso     = float(g('n_paso'))
        self.R          = float(g('corner_radius'))
        self.dead_x     = float(g('dead_x'))
        self.near_x     = float(g('near_x'))
        self.far_x      = float(g('far_x'))
        self.d_lf       = float(g('d_lf'))
        self.car_half   = float(g('car_half'))
        self.min_pts    = int(g('min_obj_pts'))
        self.side_len   = float(g('side_check_len'))
        self.beside_len = float(g('beside_len'))
        self._front     = math.radians(float(g('front_angle_deg')))
        self._cf        = math.cos(self._front)
        self._sf        = math.sin(self._front)
        self._scan_frame = str(g('scan_frame'))
        self._filt_n    = max(1, int(g('scan_filter_size')))
        self.emerg_d    = float(g('emerg_depth'))
        self.emerg_w    = float(g('emerg_width'))
        self.confirm_n  = int(g('confirm_cycles'))
        self.clear_n    = int(g('clear_cycles'))
        self.reacq_n    = int(g('reacquire_frames'))
        self.curve_om   = float(g('curve_omega'))
        self._check_curve = bool(g('check_curve'))
        self.k_n        = float(g('k_n'))
        self.k_th       = float(g('k_theta'))
        self.steer_sign = float(g('steering_sign'))
        self._yaw_sign  = float(g('yaw_sign'))
        self.eps_n      = float(g('eps_n'))
        self.v_reb      = float(g('v_overtake'))
        self.ramp_time  = float(g('ramp_time'))
        self.man_steer  = float(g('maneuver_max_steer'))
        self.turn_steer = float(g('turn_steer'))
        self.theta_out  = math.radians(float(g('theta_out_deg')))
        self.straighten_eps = math.radians(float(g('straighten_eps_deg')))
        self.man_to     = float(g('maneuver_timeout'))
        rate_hz         = float(g('rate_hz'))
        self._dry_run   = bool(g('dry_run'))
        self._pub_mk    = bool(g('publish_markers'))
        self._mk_child  = 'overtake_fwd'

        # ── Estado interno ────────────────────────────────────────────────────
        self._state = State.LANE_FOLLOW
        self._scan = None
        self._scan_buf = deque(maxlen=self._filt_n)
        self._odom = None
        self._target_valid = False
        self._emergency = False

        self._obstacle_count = 0
        self._clear_count = 0
        self._reacq_count = 0
        self._pass_side = 0
        self._snap = None
        self._maneuver_start = None
        self._yaw0 = 0.0            # yaw al entrar a SHIFT_OUT
        self._shift_phase = 'out'   # 'out' (girar) / 'straighten' (enderezar)
        self._last_steer = 0.0
        self._dry_last_log = None
        self._man_last_log = None

        # ── QoS ───────────────────────────────────────────────────────────────
        qos_be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )

        # ── Pub / Sub ─────────────────────────────────────────────────────────
        self._pub_cmd     = self.create_publisher(Vector3Stamped, '/overtake/raw_cmd', 10)
        self._pub_source  = self.create_publisher(String, '/qcar/control_source', 10)
        self._pub_markers = self.create_publisher(MarkerArray, '/overtake/markers', 10)

        self.create_subscription(LaserScan, '/qcar/scan_republished', self._on_scan, qos_be)
        self.create_subscription(Odometry,  '/odom',                  self._on_odom, 10)
        self.create_subscription(Float32MultiArray, '/lane_target_point_m',
                                 self._on_target, 10)
        self.create_subscription(Bool, '/qcar/obstacle_detected', self._on_emergency, 10)

        # TF estático lidar_corrected -> overtake_fwd (frente=+x) para que los
        # markers se superpongan al scan crudo en RViz.
        self._tf_static = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self._scan_frame
        tf.child_frame_id = self._mk_child
        tf.transform.rotation.z = math.sin(self._front / 2.0)
        tf.transform.rotation.w = math.cos(self._front / 2.0)
        self._tf_static.sendTransform(tf)

        self.create_timer(1.0 / rate_hz, self._loop)

        mode = 'DRY_RUN (no toca la conducción)' if self._dry_run else 'ARMADO'
        self.get_logger().info(
            'overtake_supervisor listo (Etapa 2, zonas en capas)\n'
            f'  zonas: dead={self.dead_x} near={self.near_x} far={self.far_x}\n'
            f'  lane_w={self.lane_w} n_paso={self.n_paso} front={math.degrees(self._front):.0f}°\n'
            f'  modo: {mode}'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _on_scan(self, msg):
        self._scan = msg
        r = list(msg.ranges)
        if self._scan_buf and len(self._scan_buf[-1]) != len(r):
            self._scan_buf.clear()
        self._scan_buf.append(r)

    def _on_odom(self, msg):      self._odom = msg
    def _on_emergency(self, msg): self._emergency = bool(msg.data)

    def _on_target(self, msg):
        d = msg.data
        self._target_valid = (len(d) >= 2 and float(d[1]) > 0.0)

    # ── Lidar: filtro + proyección ──────────────────────────────────────────────
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
        """Puntos del scan proyectados al frame del robot: x=frente, y=izquierda.

        Corrige el montaje rotado del lidar con front_angle:
        fwd = x·cosF + y·sinF ; lat = −x·sinF + y·cosF.
        """
        s = self._scan
        ranges = self._filtered_ranges()
        if s is None or ranges is None:
            return []
        pts = []
        ang = s.angle_min
        for r in ranges:
            if math.isfinite(r) and s.range_min <= r <= s.range_max:
                xr = r * math.cos(ang)
                yr = r * math.sin(ang)
                fwd = xr * self._cf + yr * self._sf
                lat = -xr * self._sf + yr * self._cf
                pts.append((fwd, lat))
            ang += s.angle_increment
        return pts

    # ── Zonas ───────────────────────────────────────────────────────────────────
    def _detect(self, x_lo, x_hi, pts):
        """Detecta objeto en el corredor frontal [x_lo, x_hi].

        Devuelve (present, d, w, c_lat, c_fwd, sel_points).
        """
        half = self.lane_w / 2.0
        sel = [(f, l) for (f, l) in pts if x_lo < f <= x_hi and abs(l) <= half]
        if len(sel) < self.min_pts:
            return False, None, None, None, None, []
        d = min(f for f, _ in sel)
        ls = [l for _, l in sel]
        w = max(ls) - min(ls)
        c_lat = sum(ls) / len(sel)
        c_fwd = sum(f for f, _ in sel) / len(sel)
        return True, d, w, c_lat, c_fwd, sel

    def _side_free(self, side, pts):
        """¿Está libre la HUELLA DESTINO del lado `side` (+1 izq, -1 der)?

        Es la banda donde quedará el coche tras correrse n_paso: centro en
        n_paso, medio ancho car_half, a lo largo desde el morro (d_lf).
        """
        y_lo = self.n_paso - self.car_half
        y_hi = self.n_paso + self.car_half
        for (f, l) in pts:
            if self.d_lf <= f <= self.d_lf + self.side_len and y_lo <= l * side <= y_hi:
                return False
        return True

    def _choose_side(self, pts, object_side):
        """Lado LIBRE, preferentemente el OPUESTO a donde está el objeto.

        object_side: +1 objeto a la izq, -1 a la der, 0 centrado.
        Devuelve +1/-1 (lado de paso) o 0 si ninguno libre.
        """
        free_l = self._side_free(+1, pts)
        free_r = self._side_free(-1, pts)
        prefer = -object_side if object_side != 0 else +1   # opuesto al objeto
        if prefer == +1 and free_l:
            return +1
        if prefer == -1 and free_r:
            return -1
        if free_l:
            return +1
        if free_r:
            return -1
        return 0

    def _obstacle_beside(self, pts):
        """¿Hay objeto al costado (del lado del obstáculo) cerca de x≈0?"""
        obj_side = -self._pass_side
        y_max = self.lane_w / 2.0 + self.n_paso
        for (f, l) in pts:
            if abs(f) <= self.beside_len and (l * obj_side) > 0 and abs(l) <= y_max:
                return True
        return False

    # ── Odom ─────────────────────────────────────────────────────────────────────
    def _yaw(self, odom):
        q = odom.pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _snapshot(self):
        o = self._odom
        self._snap = (o.pose.pose.position.x, o.pose.pose.position.y, self._yaw(o))

    def _offset_n(self):
        if self._snap is None or self._odom is None:
            return 0.0
        x0, y0, th0 = self._snap
        dx = self._odom.pose.pose.position.x - x0
        dy = self._odom.pose.pose.position.y - y0
        return -dx * math.sin(th0) + dy * math.cos(th0)

    def _dtheta(self):
        if self._snap is None or self._odom is None:
            return 0.0
        return self._wrap(self._yaw(self._odom) - self._snap[2])

    @staticmethod
    def _wrap(a):
        return math.atan2(math.sin(a), math.cos(a))

    def _omega(self):
        return self._odom.twist.twist.angular.z if self._odom else 0.0

    def _is_curve(self):
        if not self._check_curve or self._odom is None:
            return False
        return abs(self._omega()) > self.curve_om

    # ── Control lateral ─────────────────────────────────────────────────────────
    def _lateral_cmd(self, n_target):
        """Ley P sobre el offset, amortiguada por el rumbo (yaw_sign permite
        invertir el amortiguamiento si Δθ va al revés en este coche)."""
        e_n = n_target - self._offset_n()
        delta = self.k_n * e_n - self.k_th * self._yaw_sign * self._dtheta()
        delta = max(min(delta, self.man_steer), -self.man_steer)
        self._last_steer = self.steer_sign * delta
        return self._last_steer

    # ── Salidas ─────────────────────────────────────────────────────────────────
    def _set_source(self, src):
        m = String(); m.data = src
        self._pub_source.publish(m)

    def _publish_cmd(self, v, steer):
        m = Vector3Stamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'overtake_supervisor'
        m.vector.x = float(v)
        m.vector.y = float(steer)
        m.vector.z = 0.0
        self._pub_cmd.publish(m)

    def _go(self, new_state):
        if new_state != self._state:
            self.get_logger().info(f'[{self._state.name}] -> [{new_state.name}]')
            self._state = new_state

    # ── Logs (throttled) ──────────────────────────────────────────────────────────
    def _throttle(self, attr, period_s=2.0):
        now = self.get_clock().now()
        last = getattr(self, attr)
        if last is not None and (now - last).nanoseconds < period_s * 1e9:
            return False
        setattr(self, attr, now)
        return True

    def _dry_report(self, curve, object_side, side, pts):
        if not self._throttle('_dry_last_log', 2.0):
            return
        _p, d, w, _cl, _cf, _s = self._detect(self.dead_x, self.far_x, pts)
        free_l = self._side_free(+1, pts)
        free_r = self._side_free(-1, pts)
        obj = {1: 'IZQ', -1: 'DER', 0: 'centro'}[object_side]
        if curve:
            verdict = 'CURVA -> HOLD'
        elif side == 0:
            verdict = 'sin lado libre -> HOLD'
        else:
            verdict = f'paso por {"IZQ" if side > 0 else "DER"}'
        ds = f'{d:.2f}' if d is not None else '--'
        ws = f'{w:.2f}' if w is not None else '--'
        self.get_logger().info(
            f'[DRY] obj@{obj} d={ds} w={ws} | libre izq={free_l} der={free_r} '
            f'| curva={curve} | {verdict}'
        )

    def _scan_diag(self, pts):
        """Diagnóstico SIEMPRE-visible en LANE_FOLLOW: ¿ve el objeto y dónde?

        Reporta el punto más cercano del corredor del carril (rango amplio) y
        cuántos puntos caen en la zona de decisión [near_x, far_x].
        """
        if not self._throttle('_dry_last_log', 1.5):
            return
        half = self.lane_w / 2.0
        fwd = [(f, l) for (f, l) in pts if 0.10 < f <= 1.6 and abs(l) <= half]
        if not fwd:
            self.get_logger().info(
                '[scan] sin puntos en el corredor del carril (¿proyección/ángulo?)')
            return
        d = min(f for f, _ in fwd)
        lat = next(l for f, l in fwd if f == d)
        n_dec = sum(1 for f, _ in fwd if self.near_x <= f <= self.far_x)
        if self.near_x <= d <= self.far_x:
            zona = 'DECISIÓN'
        elif d < self.near_x:
            zona = 'demasiado cerca (tierra de nadie/emergencia)'
        else:
            zona = 'más lejos que far_x'
        self.get_logger().info(
            f'[scan] +cercano d={d:.2f} lat={lat:+.2f} ({len(fwd)} pts en carril) '
            f'| en decisión[{self.near_x:.2f},{self.far_x:.2f}]={n_dec} '
            f'(min {self.min_pts}) | {zona}'
        )

    def _shift_log(self, prog, steer):
        if not self._throttle('_man_last_log', 0.4):
            return
        self.get_logger().info(
            f'[SHIFT_OUT/{self._shift_phase}] giro={math.degrees(prog):+.0f}° '
            f'(meta {math.degrees(self.theta_out):.0f}°) steer={steer:+.3f}')

    def _man_log(self, pts):
        if not self._throttle('_man_last_log', 0.4):
            return
        self.get_logger().info(
            f'[{self._state.name}] n={self._offset_n():+.3f} '
            f'dθ={math.degrees(self._dtheta()):+.1f}° steer={self._last_steer:+.3f} '
            f'beside={self._obstacle_beside(pts)}'
        )

    # ── Loop principal (FSM) ─────────────────────────────────────────────────────
    def _loop(self):
        now = self.get_clock().now()
        maneuver = self._state in (State.SHIFT_OUT, State.ALONGSIDE,
                                   State.SHIFT_BACK, State.REACQUIRE)

        # Timeout de maniobra: nunca girar indefinidamente.
        if maneuver and self._maneuver_start is not None:
            if (now - self._maneuver_start).nanoseconds * 1e-9 > self.man_to:
                self.get_logger().warn('TIMEOUT de maniobra -> ABORT a lane')
                self._abort()
                return

        # ABORT por emergencia en plena maniobra.
        if self._emergency and maneuver:
            self.get_logger().warn('emergencia durante maniobra -> ABORT a lane')
            self._abort()
            return

        pts = self._points()
        if self._pub_mk:
            self._publish_overlay(pts)

        if self._state == State.LANE_FOLLOW:
            self._set_source('lane')
            if self._dry_run:
                self._scan_diag(pts)
            # Disparo en la zona de decisión [near_x, far_x], MÁS ALLÁ de la
            # zona de emergencia, para arrancar el rebase con espacio de sobra.
            decide = self._detect(self.near_x, self.far_x, pts)
            self._obstacle_count = self._obstacle_count + 1 if decide[0] else 0
            if self._obstacle_count >= self.confirm_n:
                self._go(State.CONFIRM)

        elif self._state == State.CONFIRM:
            curve = self._is_curve()
            det = self._detect(self.near_x, self.far_x, pts)
            c_lat = det[3]
            object_side = 0 if c_lat is None else (1 if c_lat > 0 else -1)
            side = 0 if curve else self._choose_side(pts, object_side)

            if self._dry_run:
                self._dry_report(curve, object_side, side, pts)
                self._go(State.LANE_FOLLOW)
                return

            if curve:
                self.get_logger().info(
                    f'obstáculo en CURVA (ω={self._omega():+.3f} > {self.curve_om}) '
                    f'-> HOLD (Etapa 3 pendiente)')
                self._go(State.HOLD); return
            if side == 0:
                self.get_logger().info('sin lado libre -> HOLD')
                self._go(State.HOLD); return
            self._pass_side = side
            self._maneuver_start = now
            self._yaw0 = self._yaw(self._odom) if self._odom else 0.0
            self._shift_phase = 'out'
            self.get_logger().info(
                f'rebase por {"IZQ" if side > 0 else "DER"} (objeto '
                f'{"IZQ" if object_side > 0 else "DER" if object_side < 0 else "centro"}) '
                f'yaw0={math.degrees(self._yaw0):.0f}°')
            self._set_source('overtake')
            self._go(State.SHIFT_OUT)

        elif self._state == State.SHIFT_OUT:
            # Por YAW (sin distancia): gira hacia el lado libre hasta theta_out,
            # luego endereza (yaw->0) para quedar paralelo. prog>0 = girado
            # hacia pass_side (yaw_sign permite invertir si el yaw va al revés).
            yaw = self._yaw(self._odom) if self._odom else self._yaw0
            prog = self._pass_side * self._yaw_sign * self._wrap(yaw - self._yaw0)
            if self._shift_phase == 'out':
                steer = self.steer_sign * self._pass_side * self.turn_steer
                if prog >= self.theta_out:
                    self._shift_phase = 'straighten'
                    self.get_logger().info(
                        f'SHIFT_OUT: girado {math.degrees(prog):.0f}° -> enderezando')
            else:  # straighten -> volver a paralelo (prog -> 0)
                steer = -self.steer_sign * self._pass_side * self.turn_steer
                if prog <= self.straighten_eps:
                    self.get_logger().info('SHIFT_OUT completo (paralelo) -> detenido para inspección')
                    self._publish_cmd(0.0, 0.0)
                    self._go(State.SHIFT_DONE)
                    return
            self._publish_cmd(self.v_reb, steer)
            self._shift_log(prog, steer)

        elif self._state == State.SHIFT_DONE:
            # Test: detenido tras salir. Se reinicia al quitar el obstáculo.
            self._publish_cmd(0.0, 0.0)
            if not self._detect(self.near_x, self.far_x, pts)[0]:
                self.get_logger().info('obstáculo retirado -> vuelvo a LANE_FOLLOW')
                self._set_source('lane')
                self._reset_counters()
                self._go(State.LANE_FOLLOW)

        elif self._state == State.ALONGSIDE:
            n_target = self._pass_side * self.n_paso
            self._publish_cmd(self.v_reb, self._lateral_cmd(n_target))
            self._man_log(pts)
            self._clear_count = 0 if self._obstacle_beside(pts) else self._clear_count + 1
            if self._clear_count >= self.clear_n:
                self._go(State.SHIFT_BACK)

        elif self._state == State.SHIFT_BACK:
            self._publish_cmd(self.v_reb, self._lateral_cmd(0.0))
            self._man_log(pts)
            if abs(self._offset_n()) < self.eps_n or self._target_valid:
                self._reacq_count = 0
                self._go(State.REACQUIRE)

        elif self._state == State.REACQUIRE:
            self._publish_cmd(self.v_reb, self._lateral_cmd(0.0))
            self._reacq_count = self._reacq_count + 1 if self._target_valid else 0
            if self._reacq_count >= self.reacq_n:
                self._set_source('lane')
                self._reset_counters()
                self._go(State.LANE_FOLLOW)

        elif self._state == State.HOLD:
            self._set_source('lane')
            if not self._detect(self.near_x, self.far_x, pts)[0]:
                self._reset_counters()
                self._go(State.LANE_FOLLOW)

    def _abort(self):
        self._set_source('lane')
        self._reset_counters()
        self._go(State.LANE_FOLLOW)

    def _reset_counters(self):
        self._obstacle_count = 0
        self._clear_count = 0
        self._reacq_count = 0
        self._pass_side = 0
        self._snap = None
        self._maneuver_start = None

    # ── Visualización RViz (zonas) ───────────────────────────────────────────────
    def _box(self, stamp, mid, ns, cx, cy, sx, sy, rgba):
        m = Marker()
        m.header.frame_id = self._mk_child
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = float(cx)
        m.pose.position.y = float(cy)
        m.pose.position.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = float(sx)
        m.scale.y = float(sy)
        m.scale.z = 0.02
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        return m

    def _publish_overlay(self, pts):
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()

        # Z0 emergencia (debug) — rojo.
        arr.markers.append(self._box(
            stamp, 0, 'z0_emergency', cx=self.emerg_d / 2.0, cy=0.0,
            sx=self.emerg_d, sy=self.emerg_w, rgba=(0.9, 0.1, 0.1, 0.15)))
        # Z1 lejana — amarillo.
        arr.markers.append(self._box(
            stamp, 1, 'z1_far', cx=(self.near_x + self.far_x) / 2.0, cy=0.0,
            sx=self.far_x - self.near_x, sy=self.lane_w, rgba=(1.0, 0.9, 0.1, 0.15)))
        # Z2 cercana — naranja.
        arr.markers.append(self._box(
            stamp, 2, 'z2_near', cx=(self.dead_x + self.near_x) / 2.0, cy=0.0,
            sx=self.near_x - self.dead_x, sy=self.lane_w, rgba=(1.0, 0.5, 0.1, 0.25)))
        # Z4 corredores de paso (huella destino) — verde libre / rojo bloqueado.
        free_l = self._side_free(+1, pts)
        free_r = self._side_free(-1, pts)
        cx_side = self.d_lf + self.side_len / 2.0
        sy_side = 2.0 * self.car_half
        arr.markers.append(self._box(
            stamp, 3, 'z4_left', cx=cx_side, cy=+self.n_paso,
            sx=self.side_len, sy=sy_side,
            rgba=(0.1, 0.8, 0.1, 0.25) if free_l else (0.9, 0.1, 0.1, 0.25)))
        arr.markers.append(self._box(
            stamp, 4, 'z4_right', cx=cx_side, cy=-self.n_paso,
            sx=self.side_len, sy=sy_side,
            rgba=(0.1, 0.8, 0.1, 0.25) if free_r else (0.9, 0.1, 0.1, 0.25)))

        # Puntos del objeto (Z1+Z2) — esferas rojas.
        present, d, w, c_lat, c_fwd, sel = self._detect(self.dead_x, self.far_x, pts)
        mp = Marker()
        mp.header.frame_id = self._mk_child
        mp.header.stamp = stamp
        mp.ns = 'obstacle_pts'
        mp.id = 5
        mp.type = Marker.SPHERE_LIST
        mp.action = Marker.ADD if present else Marker.DELETE
        mp.scale.x = mp.scale.y = mp.scale.z = 0.03
        mp.color.r, mp.color.g, mp.color.b, mp.color.a = (1.0, 0.1, 0.1, 0.9)
        mp.points = [Point(x=float(f), y=float(l), z=0.0) for (f, l) in sel]
        mp.pose.orientation.w = 1.0
        arr.markers.append(mp)

        # Z3 centroide del objeto — esfera magenta.
        mc = Marker()
        mc.header.frame_id = self._mk_child
        mc.header.stamp = stamp
        mc.ns = 'z3_centroid'
        mc.id = 6
        mc.type = Marker.SPHERE
        mc.action = Marker.ADD if present else Marker.DELETE
        if present:
            mc.pose.position.x = float(c_fwd)
            mc.pose.position.y = float(c_lat)
        mc.pose.position.z = 0.05
        mc.pose.orientation.w = 1.0
        mc.scale.x = mc.scale.y = mc.scale.z = 0.07
        mc.color.r, mc.color.g, mc.color.b, mc.color.a = (1.0, 0.0, 1.0, 0.9)
        arr.markers.append(mc)

        # Texto.
        mt = Marker()
        mt.header.frame_id = self._mk_child
        mt.header.stamp = stamp
        mt.ns = 'txt'
        mt.id = 7
        mt.type = Marker.TEXT_VIEW_FACING
        mt.action = Marker.ADD
        mt.pose.position.x = self.far_x
        mt.pose.position.y = 0.0
        mt.pose.position.z = 0.15
        mt.pose.orientation.w = 1.0
        mt.scale.z = 0.07
        mt.color.r, mt.color.g, mt.color.b, mt.color.a = (1.0, 1.0, 1.0, 0.9)
        if present:
            objs = 'IZQ' if c_lat > 0 else 'DER'
            mt.text = f'{self._state.name} obj@{objs} d={d:.2f}'
        else:
            mt.text = self._state.name
        arr.markers.append(mt)

        self._pub_markers.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = OvertakeSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
