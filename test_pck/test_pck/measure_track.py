"""
Medidor de geometria de pista — Etapa 0 del rebase.

Se suscribe a /odom mientras das UNA vuelta limpia siguiendo el carril.
Al cortar con Ctrl+C:
  - segmenta los tramos de curva por |yaw-rate| (twist.angular.z),
  - ajusta un circulo (Kasa) a cada esquina -> radio R,
  - reporta R por esquina y la media,
  - guarda la trayectoria en <output_dir>/track_odom.csv (+ PNG si hay matplotlib).

El radio R medido aqui es la constante de pista para el rebase en curva
(arco concentrico R +/- n_paso). El ancho de carril se mide aparte con
cinta metrica (es una pista fisica).

Parametros:
  odom_topic     topic de odometria (default /odom)
  output_dir     carpeta de salida (default ~/Desktop/qcar_track)
  corner_omega   umbral |yaw-rate| rad/s para considerar curva (default 0.05)
  min_points     minimo de puntos por esquina para ajustar (default 8)

Uso:
  ros2 run test_pck measure_track
  # conduce una vuelta completa siguiendo el carril, luego Ctrl+C
"""

import os
import csv
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry


DEFAULT_OUTPUT_DIR = os.path.expanduser('~/Desktop/qcar_track')


def _solve3(a, b):
    """Resuelve un sistema 3x3 por eliminacion gaussiana. None si singular."""
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-12:
            return None
        m[col], m[piv] = m[piv], m[col]
        pv = m[col][col]
        m[col] = [v / pv for v in m[col]]
        for r in range(3):
            if r != col:
                f = m[r][col]
                m[r] = [v - f * mc for v, mc in zip(m[r], m[col])]
    return [m[0][3], m[1][3], m[2][3]]


def fit_circle_kasa(xs, ys):
    """Ajuste algebraico de circulo (Kasa). Devuelve (cx, cy, R) o None."""
    n = len(xs)
    sx = sum(xs); sy = sum(ys)
    sxx = sum(x * x for x in xs); syy = sum(y * y for y in ys)
    sxy = sum(x * y for x, y in zip(xs, ys))
    sxz = sum(x * (x * x + y * y) for x, y in zip(xs, ys))
    syz = sum(y * (x * x + y * y) for x, y in zip(xs, ys))
    sz = sum(x * x + y * y for x, y in zip(xs, ys))
    sol = _solve3(
        [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, float(n)]],
        [sxz, syz, sz],
    )
    if sol is None:
        return None
    A, B, C = sol
    cx, cy = A / 2.0, B / 2.0
    r2 = C + cx * cx + cy * cy
    if r2 <= 0:
        return None
    return cx, cy, math.sqrt(r2)


class TrackMeasurer(Node):
    def __init__(self):
        super().__init__('measure_track')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('output_dir', DEFAULT_OUTPUT_DIR)
        self.declare_parameter('corner_omega', 0.05)
        self.declare_parameter('min_points', 8)

        topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self._out_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self._corner_omega = float(self.get_parameter('corner_omega').value)
        self._min_pts = int(self.get_parameter('min_points').value)

        os.makedirs(self._out_dir, exist_ok=True)
        self.pts = []   # (t, x, y, omega)

        self.create_subscription(Odometry, topic, self._cb, 50)
        self.get_logger().info(
            f'measure_track listo — sub {topic}\n'
            f'  Conduce UNA vuelta siguiendo el carril y luego Ctrl+C\n'
            f'  salida: {self._out_dir}'
        )

    def _cb(self, msg: Odometry):
        self.pts.append((
            time.monotonic(),
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.twist.twist.angular.z,
            msg.twist.twist.linear.x,
        ))

    def analyze(self):
        if len(self.pts) < 20:
            self.get_logger().warn(
                f'Solo {len(self.pts)} puntos; conduce una vuelta completa.')
            return

        csv_path = os.path.join(self._out_dir, 'track_odom.csv')
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'omega', 'v'])
            w.writerows(self.pts)

        # --- Diagnostico de giroscopio (deriva de yaw) ---
        # Con el carro QUIETO: media de omega = bias residual del gyro_z
        #   -> cargalo en imu_external como gyro_bias_z.
        # En una vuelta cerrada: el heading integrado deberia ser ~+/-360 deg
        #   por vuelta; un exceso revela bias.
        omegas = [p[3] for p in self.pts]
        mean_om = sum(omegas) / len(omegas)
        var_om = sum((o - mean_om) ** 2 for o in omegas) / len(omegas)
        std_om = math.sqrt(var_om)
        heading = 0.0
        for a, b in zip(self.pts[:-1], self.pts[1:]):
            dt = b[0] - a[0]
            if 0.0 < dt < 0.5:
                heading += b[3] * dt
        dur = self.pts[-1][0] - self.pts[0][0]
        print('\n===== Diagnostico giroscopio =====')
        print(f'  duracion: {dur:.1f} s   omega media: {mean_om:+.5f} rad/s   '
              f'sigma: {std_om:.5f}')
        print(f'  heading integrado: {math.degrees(heading):+.1f} deg '
              f'(una vuelta cerrada ~ +/-360)')
        print('  Si el carro estaba QUIETO -> omega media = bias a corregir '
              'en imu_external (gyro_bias_z).')

        # --- R instantaneo = v/omega (independiente del umbral/segmentacion) ---
        # En curva, |v/omega| converge al radio de la esquina; ignoramos los
        # puntos casi-rectos (omega pequeno) donde el radio tiende a infinito.
        r_inst = sorted(
            abs(p[4]) / abs(p[3])
            for p in self.pts
            if abs(p[3]) > self._corner_omega and abs(p[4]) > 1e-3
        )
        if r_inst:
            n = len(r_inst)
            med = r_inst[n // 2]
            q1 = r_inst[n // 4]
            q3 = r_inst[(3 * n) // 4]
            print(f'  R instantaneo (v/omega): mediana {med:.3f} m  '
                  f'[IQR {q1:.3f}-{q3:.3f}, {n} pts]  <-- robusto al umbral')

        # Segmentar en tramos consecutivos de curva (|omega| sobre umbral)
        corners, cur = [], []
        for _t, x, y, om, _v in self.pts:
            if abs(om) > self._corner_omega:
                cur.append((x, y))
            else:
                if len(cur) >= self._min_pts:
                    corners.append(cur)
                cur = []
        if len(cur) >= self._min_pts:
            corners.append(cur)

        print('\n===== Geometria de pista (de /odom) =====')
        print(f'puntos totales: {len(self.pts)}   esquinas detectadas: {len(corners)}')
        radii = []
        for i, c in enumerate(corners):
            res = fit_circle_kasa([p[0] for p in c], [p[1] for p in c])
            if res is None:
                print(f'  esquina {i+1}: ajuste fallido ({len(c)} pts)')
                continue
            cx, cy, R = res
            radii.append(R)
            print(f'  esquina {i+1}: R = {R:.3f} m  '
                  f'(centro {cx:+.2f},{cy:+.2f}, {len(c)} pts)')
        if radii:
            mean = sum(radii) / len(radii)
            spread = max(radii) - min(radii)
            print(f'\n  >>> R medio = {mean:.3f} m   (dispersion {spread:.3f} m)')
            print('      Constante de pista para el arco concentrico del rebase.')
        else:
            print('  No se ajusto ninguna esquina. Baja corner_omega y reintenta.')
        print(f'  Trayectoria: {csv_path}')

        self._maybe_plot()

    def _maybe_plot(self):
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except Exception:
            return
        xs = [p[1] for p in self.pts]; ys = [p[2] for p in self.pts]
        cm = ['r' if abs(p[3]) > self._corner_omega else 'b' for p in self.pts]
        plt.figure(figsize=(6, 6))
        plt.scatter(xs, ys, c=cm, s=4)
        plt.axis('equal')
        plt.title('odom path (rojo=curva, azul=recta)')
        png_path = os.path.join(self._out_dir, 'track_path.png')
        plt.savefig(png_path, dpi=120)
        print(f'  Grafica: {png_path}')


def main():
    # Desactivar el SIGINT de rclpy para correr analyze() al cortar con Ctrl+C.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TrackMeasurer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.analyze()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
