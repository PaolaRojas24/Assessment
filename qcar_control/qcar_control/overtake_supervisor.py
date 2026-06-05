#!/usr/bin/env python3
"""
overtake_supervisor.py — Supervisor de rebase (Etapa 2: solo rectas)
====================================================================

FSM que decide cuándo rebasar un obstáculo del carril y ejecuta la
maniobra de desplazamiento lateral. NO toca la seguridad: la parada de
emergencia la sigue haciendo command_mux a partir de /qcar/obstacle_detected.

Arquitectura (acordada en diseño):
  - odom (NO preciso) -> solo la FORMA de la trayectoria (offset n aprox) y
    clasificar recta/curva. Nunca decide algo crítico.
  - lidar -> TODAS las decisiones del obstáculo: detectarlo, elegir lado,
    "ya estoy a su costado", "ya quedó atrás".
  - cámara (/lane_target_point_m) -> la vuelta al carril (REACQUIRE).

Salida:
  - /qcar/control_source  (String 'lane'|'overtake')  conmuta el mux.
  - /overtake/raw_cmd     (Vector3Stamped x=vel, y=dirección) la maniobra.

Convención del lidar: angulo 0 = frente (+x adelante), y = r·sin(θ),
y>0 = IZQUIERDA del robot. pass_side: +1 izquierda, -1 derecha.

Estados:
  LANE_FOLLOW  el lane follower conduce; vigilo obstáculo en mi carril.
  CONFIRM      obstáculo persiste; clasifico recta/curva, elijo lado, fijo n_paso.
  SHIFT_OUT    me corro al carril de paso (vel reb).
  ALONGSIDE    avanzo a la par del obstáculo hasta dejarlo atrás.
  SHIFT_BACK   regreso hacia mi carril.
  REACQUIRE    la cámara confirma el carril; rampa de velocidad y vuelvo a lane.
  HOLD         no se puede rebasar (curva o sin lado libre): sigo en carril,
               la emergencia del mux para si me acerco demasiado.
  (ABORT = emergencia en plena maniobra -> source=lane, suelto el control.)

ESTE ARCHIVO ES UN ESQUELETO: la lógica está completa pero los umbrales
geométricos (marcados con TODO TUNE) se calibran en el QCar.
"""

import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSHistoryPolicy, QoSDurabilityPolicy)
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool, String, Float32MultiArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class State(Enum):
    LANE_FOLLOW = auto()
    CONFIRM     = auto()
    SHIFT_OUT   = auto()
    ALONGSIDE   = auto()
    SHIFT_BACK  = auto()
    REACQUIRE   = auto()
    HOLD        = auto()


class OvertakeSupervisor(Node):

    def __init__(self):
        super().__init__('overtake_supervisor')

        # ── Parámetros ────────────────────────────────────────────────────────
        # Geometría de pista / coche (medidos en Etapa 0).
        self.declare_parameter('lane_width',   0.25)   # m
        self.declare_parameter('n_paso',       0.25)   # m, offset lateral de paso
        self.declare_parameter('corner_radius', 1.0)   # m (para Etapa 3)

        # Detección lidar.
        self.declare_parameter('look_ahead',   0.80)   # m, corredor frontal
        self.declare_parameter('dead_x',       0.10)   # m, zona muerta frontal
        self.declare_parameter('min_obj_pts',  4)      # pts para confirmar objeto
        self.declare_parameter('side_check_len', 0.60) # m, largo del corredor lateral
        self.declare_parameter('side_window_x', 0.20)  # m, ventana |x| para "al costado"

        # Tiempos/ciclos de confirmación.
        self.declare_parameter('confirm_cycles', 5)
        self.declare_parameter('clear_cycles',   5)
        self.declare_parameter('reacquire_frames', 5)

        # Clasificación recta/curva.
        self.declare_parameter('curve_omega',  0.12)   # rad/s |ω| de /odom

        # Control lateral.
        self.declare_parameter('k_n',          2.0)    # ganancia offset
        self.declare_parameter('k_theta',      1.0)    # amortiguamiento rumbo
        self.declare_parameter('max_steer',    0.50)   # rad
        self.declare_parameter('steering_sign', -1.0)  # igual que el lane follower
        self.declare_parameter('eps_n',        0.04)   # m, tolerancia de offset

        # Velocidad / rampa.
        self.declare_parameter('v_overtake',   0.10)   # m/s (el mux igual lo topa)
        self.declare_parameter('ramp_time',    1.5)    # s, REACQUIRE -> normal

        self.declare_parameter('rate_hz',      15.0)

        # Modo seguro de validación: ejecuta la lógica y registra las
        # decisiones, pero NUNCA conmuta a overtake ni publica comandos.
        # El coche sigue 100% bajo el seguidor de línea. Default: activado.
        self.declare_parameter('dry_run',      True)

        g = lambda n: self.get_parameter(n).value
        self.lane_w     = float(g('lane_width'))
        self.n_paso     = float(g('n_paso'))
        self.R          = float(g('corner_radius'))
        self.look_ahead = float(g('look_ahead'))
        self.dead_x     = float(g('dead_x'))
        self.min_pts    = int(g('min_obj_pts'))
        self.side_len   = float(g('side_check_len'))
        self.side_win_x = float(g('side_window_x'))
        self.confirm_n  = int(g('confirm_cycles'))
        self.clear_n    = int(g('clear_cycles'))
        self.reacq_n    = int(g('reacquire_frames'))
        self.curve_om   = float(g('curve_omega'))
        self.k_n        = float(g('k_n'))
        self.k_th       = float(g('k_theta'))
        self.max_steer  = float(g('max_steer'))
        self.steer_sign = float(g('steering_sign'))
        self.eps_n      = float(g('eps_n'))
        self.v_reb      = float(g('v_overtake'))
        self.ramp_time  = float(g('ramp_time'))
        rate_hz         = float(g('rate_hz'))
        self._dry_run   = bool(g('dry_run'))

        # ── Estado interno ────────────────────────────────────────────────────
        self._state = State.LANE_FOLLOW
        self._scan = None            # último LaserScan
        self._odom = None            # último Odometry
        self._target_valid = False   # cámara: target de carril válido?
        self._emergency = False      # /qcar/obstacle_detected

        self._obstacle_count = 0     # ciclos seguidos con obstáculo en carril
        self._clear_count = 0        # ciclos seguidos con costado libre
        self._reacq_count = 0        # frames seguidos con carril válido
        self._pass_side = 0          # +1 izq, -1 der, 0 sin elegir
        self._snap = None            # (X0, Y0, θ0) al entrar a la maniobra
        self._ramp_start = None
        self._dry_last_log = None    # throttle del log en dry_run

        # ── QoS ───────────────────────────────────────────────────────────────
        qos_be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )

        # ── Pub / Sub ─────────────────────────────────────────────────────────
        self._pub_cmd    = self.create_publisher(Vector3Stamped, '/overtake/raw_cmd', 10)
        self._pub_source = self.create_publisher(String, '/qcar/control_source', 10)

        self.create_subscription(LaserScan, '/qcar/scan_republished', self._on_scan, qos_be)
        self.create_subscription(Odometry,  '/odom',                  self._on_odom, 10)
        self.create_subscription(Float32MultiArray, '/lane_target_point_m',
                                 self._on_target, 10)
        self.create_subscription(Bool, '/qcar/obstacle_detected', self._on_emergency, 10)

        self.create_timer(1.0 / rate_hz, self._loop)

        mode = 'DRY_RUN (no toca la conducción)' if self._dry_run else 'ARMADO (puede rebasar)'
        self.get_logger().info(
            'overtake_supervisor listo (Etapa 2: solo rectas)\n'
            f'  lane_w={self.lane_w}  n_paso={self.n_paso}  v_reb={self.v_reb}\n'
            f'  disparo AUTOMÁTICO, HOLD pasivo\n'
            f'  modo: {mode}'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _on_scan(self, msg):     self._scan = msg
    def _on_odom(self, msg):     self._odom = msg
    def _on_emergency(self, msg): self._emergency = bool(msg.data)

    def _on_target(self, msg):
        # lane_detector publica [-1,-1] cuando no hay carril; >0 en y = válido.
        d = msg.data
        self._target_valid = (len(d) >= 2 and float(d[1]) > 0.0)

    # ── Helpers de lidar ───────────────────────────────────────────────────────
    def _points(self):
        """Devuelve lista de (x, y) válidos del último scan en frame del lidar."""
        s = self._scan
        if s is None:
            return []
        pts = []
        ang = s.angle_min
        for r in s.ranges:
            if math.isfinite(r) and s.range_min <= r <= s.range_max:
                pts.append((r * math.cos(ang), r * math.sin(ang)))
            ang += s.angle_increment
        return pts

    def _obstacle_ahead(self, pts):
        """(present, d_obj, w_obj) para el corredor de mi carril al frente."""
        half = self.lane_w / 2.0
        sel = [(x, y) for (x, y) in pts
               if self.dead_x < x <= self.look_ahead and abs(y) <= half]
        if len(sel) < self.min_pts:
            return False, None, None
        d_obj = min(x for x, _ in sel)
        ys = [y for _, y in sel]
        w_obj = max(ys) - min(ys)
        return True, d_obj, w_obj

    def _side_free(self, side, pts):
        """¿Está libre el corredor de paso del lado `side` (+1 izq, -1 der)?"""
        half = self.lane_w / 2.0
        y_lo = half                       # borde interno del corredor de paso
        y_hi = half + self.n_paso         # borde externo
        for (x, y) in pts:
            if 0.0 < x <= self.side_len:
                ys = y * side              # proyecto al lado consultado
                if y_lo <= ys <= y_hi:
                    return False           # hay algo en el carril de paso
        return True

    def _choose_side(self, pts):
        """Prefiere izquierda; si no, derecha; 0 si ninguno libre (gate universal)."""
        if self._side_free(+1, pts):
            return +1
        if self._side_free(-1, pts):
            return -1
        return 0

    def _obstacle_beside(self, pts):
        """¿Hay un objeto al costado (del lado del obstáculo) cerca de x≈0?"""
        # Tras salir hacia pass_side, el obstáculo queda en -pass_side.
        obj_side = -self._pass_side
        y_max = self.lane_w / 2.0 + self.n_paso
        for (x, y) in pts:
            if abs(x) <= self.side_win_x and (y * obj_side) > 0 and abs(y) <= y_max:
                return True
        return False

    # ── Helpers de odom ─────────────────────────────────────────────────────────
    def _yaw(self, odom):
        q = odom.pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _snapshot(self):
        o = self._odom
        self._snap = (o.pose.pose.position.x, o.pose.pose.position.y, self._yaw(o))

    def _offset_n(self):
        """Offset lateral (aprox) respecto al marco snapshot. + = izquierda."""
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

    def _is_curve(self):
        if self._odom is None:
            return False
        return abs(self._odom.twist.twist.angular.z) > self.curve_om

    # ── Control lateral ─────────────────────────────────────────────────────────
    def _lateral_cmd(self, n_target):
        """Ley P sobre el offset, amortiguada por el rumbo. δ>0 = izquierda."""
        e_n = n_target - self._offset_n()
        delta = self.k_n * e_n - self.k_th * self._dtheta()
        delta = max(min(delta, self.max_steer), -self.max_steer)
        return self.steer_sign * delta

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

    def _dry_report(self, curve, side, pts):
        """Log throttled (~1/2s) de la decisión en dry_run, para calibrar."""
        now = self.get_clock().now()
        if (self._dry_last_log is not None and
                (now - self._dry_last_log).nanoseconds < 2e9):
            return
        self._dry_last_log = now
        _p, d, w = self._obstacle_ahead(pts)
        free_l = self._side_free(+1, pts)
        free_r = self._side_free(-1, pts)
        if curve:
            verdict = 'CURVA -> HOLD (no rebasa)'
        elif side == 0:
            verdict = 'sin lado libre -> HOLD (no rebasa)'
        else:
            verdict = f'recta + lado {"IZQ" if side > 0 else "DER"} -> rebasaría'
        self.get_logger().info(
            f'[DRY] obst d={d:.2f}m ancho={w:.2f}m | '
            f'libre izq={free_l} der={free_r} | curva={curve} | {verdict}'
        )

    # ── Loop principal (FSM) ─────────────────────────────────────────────────────
    def _loop(self):
        # ABORT transversal: emergencia en plena maniobra -> devolver a lane.
        maneuver = self._state in (State.SHIFT_OUT, State.ALONGSIDE,
                                   State.SHIFT_BACK, State.REACQUIRE)
        if self._emergency and maneuver:
            self.get_logger().warn('emergencia durante maniobra -> ABORT a lane')
            self._set_source('lane')
            self._reset_counters()
            self._go(State.LANE_FOLLOW)
            return

        pts = self._points()

        if self._state == State.LANE_FOLLOW:
            self._set_source('lane')
            present, _d, _w = self._obstacle_ahead(pts)
            self._obstacle_count = self._obstacle_count + 1 if present else 0
            if self._obstacle_count >= self.confirm_n:
                self._go(State.CONFIRM)

        elif self._state == State.CONFIRM:
            # Decisión (gate universal): curva o sin lado libre -> HOLD.
            curve = self._is_curve()
            side = 0 if curve else self._choose_side(pts)

            if self._dry_run:
                # Solo valido percepción/decisión: registro y vuelvo a LANE_FOLLOW
                # sin tocar la conducción (el seguidor de línea sigue mandando).
                self._dry_report(curve, side, pts)
                self._go(State.LANE_FOLLOW)
                return

            if curve:
                self.get_logger().info('obstáculo en CURVA -> HOLD (Etapa 3 pendiente)')
                self._go(State.HOLD); return
            if side == 0:
                self.get_logger().info('sin lado libre -> HOLD')
                self._go(State.HOLD); return
            self._pass_side = side
            self._snapshot()
            self.get_logger().info(
                f'rebase por {"IZQ" if side > 0 else "DER"}  n_paso={self.n_paso}')
            self._set_source('overtake')
            self._go(State.SHIFT_OUT)

        elif self._state == State.SHIFT_OUT:
            n_target = self._pass_side * self.n_paso
            self._publish_cmd(self.v_reb, self._lateral_cmd(n_target))
            reached = abs(self._offset_n() - n_target) < self.eps_n
            if reached and self._obstacle_beside(pts):
                self._go(State.ALONGSIDE)

        elif self._state == State.ALONGSIDE:
            n_target = self._pass_side * self.n_paso
            self._publish_cmd(self.v_reb, self._lateral_cmd(n_target))
            if not self._obstacle_beside(pts):
                self._clear_count += 1
            else:
                self._clear_count = 0
            if self._clear_count >= self.clear_n:   # obstáculo quedó atrás
                self._go(State.SHIFT_BACK)

        elif self._state == State.SHIFT_BACK:
            self._publish_cmd(self.v_reb, self._lateral_cmd(0.0))
            back = abs(self._offset_n()) < self.eps_n
            if back or self._target_valid:          # cámara manda en la vuelta
                self._reacq_count = 0
                self._ramp_start = self.get_clock().now()
                self._go(State.REACQUIRE)

        elif self._state == State.REACQUIRE:
            # Espero carril válido y estable; rampa de velocidad recto.
            self._publish_cmd(self.v_reb, self._lateral_cmd(0.0))
            self._reacq_count = self._reacq_count + 1 if self._target_valid else 0
            if self._reacq_count >= self.reacq_n:
                self._set_source('lane')            # el lane follower retoma
                self._reset_counters()
                self._go(State.LANE_FOLLOW)

        elif self._state == State.HOLD:
            # Pasivo: sigo en carril. La emergencia del mux para si me acerco.
            self._set_source('lane')
            present, _d, _w = self._obstacle_ahead(pts)
            if not present:
                self._reset_counters()
                self._go(State.LANE_FOLLOW)

    def _reset_counters(self):
        self._obstacle_count = 0
        self._clear_count = 0
        self._reacq_count = 0
        self._pass_side = 0
        self._snap = None


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
