#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped, Vector3Stamped
import serial
import json
import math


class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        self.declare_parameter('port', '/dev/ttyUSB0')

        # Bias inicial (rad/s) — punto de partida; se recalibra en arranque.
        self.declare_parameter('gyro_bias_x', 0.0)
        self.declare_parameter('gyro_bias_y', 0.0)
        self.declare_parameter('gyro_bias_z', 0.042945)

        # --- Eliminación de la deriva estacionaria del giroscopio ---
        # 1) Auto-calibración al arranque: promedia N muestras con el carro
        #    quieto y fija el bias real de este encendido (el bias MEMS cambia
        #    con la temperatura, por eso un valor fijo no basta). 0 = desactivar.
        self.declare_parameter('gyro_calib_samples', 500)      # ~5 s a 100 Hz

        # 2) ZUPT (Zero-velocity UPdaTe): cuando el ENCODER dice que el carro
        #    está parado, la velocidad angular real es 0 → se fuerza gz=0.
        #    Detectar el reposo con el encoder (no con |gz|) evita el parpadeo:
        #    el bias residual puede ser del tamaño del umbral del giroscopio y
        #    haría oscilar la detección. velocity_topic = fuente del encoder.
        self.declare_parameter('velocity_topic', '/qcar/velocity')
        self.declare_parameter('zupt_vel_thresh', 0.03)        # m/s

        # 3) Estimación continua de bias: mientras está en reposo, el bias se
        #    ajusta hacia la lectura cruda, absorbiendo la deriva térmica para
        #    que al arrancar el movimiento el bias esté fresco.
        self.declare_parameter('gyro_bias_adapt', True)
        self.declare_parameter('gyro_bias_adapt_alpha', 0.01)  # 0=sin ajuste
        # Umbral de reposo por |gz| (solo respaldo si no hay encoder).
        self.declare_parameter('gyro_stationary_thresh', 0.03) # rad/s

        # Corrección de inclinación física del IMU (soldadura de pines).
        # pitch_offset_deg: rotación alrededor de Y  (tilt adelante/atrás)
        # roll_offset_deg : rotación alrededor de X  (tilt lateral)
        # Estimado de calibración: ax_static=1.7393, az_static=10.1389
        #   → pitch ≈ atan2(1.7393, 10.1389) ≈ 9.7°
        self.declare_parameter('pitch_offset_deg', 9.7)
        self.declare_parameter('roll_offset_deg',  0.0)

        # Filtro pasa-bajas exponencial (EMA) para el giroscopio.
        # alpha=0.0 → sin filtro (dato crudo)
        # alpha=0.9 → muy suavizado, más latencia
        self.declare_parameter('gyro_lpf_alpha', 0.5)

        port = self.get_parameter('port').get_parameter_value().string_value
        # Bias mutable (lista) porque se recalibra y se adapta en caliente.
        self._bias = [
            self.get_parameter('gyro_bias_x').get_parameter_value().double_value,
            self.get_parameter('gyro_bias_y').get_parameter_value().double_value,
            self.get_parameter('gyro_bias_z').get_parameter_value().double_value,
        ]
        self._alpha = self.get_parameter('gyro_lpf_alpha').get_parameter_value().double_value

        # Parámetros anti-deriva estacionaria
        self._calib_remaining = self.get_parameter('gyro_calib_samples').get_parameter_value().integer_value
        self._bias_adapt      = self.get_parameter('gyro_bias_adapt').get_parameter_value().bool_value
        self._stat_thresh     = self.get_parameter('gyro_stationary_thresh').get_parameter_value().double_value
        self._adapt_alpha     = self.get_parameter('gyro_bias_adapt_alpha').get_parameter_value().double_value
        self._zupt_thresh     = self.get_parameter('zupt_vel_thresh').get_parameter_value().double_value

        # Acumuladores de la auto-calibración de arranque
        self._calib_sum = [0.0, 0.0, 0.0]
        self._calib_n   = 0

        # Estado del encoder para el ZUPT
        self._v          = 0.0
        self._v_time_ns  = 0
        self._have_vel   = False

        # Matriz de rotación para corregir el tilt físico del IMU.
        # Se aplica a acelerómetro y giroscopio.
        pitch = math.radians(self.get_parameter('pitch_offset_deg').get_parameter_value().double_value)
        roll  = math.radians(self.get_parameter('roll_offset_deg').get_parameter_value().double_value)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll),  math.sin(roll)
        # R = Ry(-pitch) @ Rx(-roll) = transpuesta de Rx(roll)@Ry(pitch)
        # Transforma del frame inclinado del IMU al frame del robot.
        self._R = [
            [ cp,   sp*sr, -sp*cr],
            [ 0.0,  cr,     sr   ],
            [ sp,  -cp*sr,  cp*cr],
        ]

        # Estado del filtro EMA
        self._gx = self._gy = self._gz = 0.0
        self._filter_init = False

        # Covarianzas estimadas de la calibración:
        #   giroscopio → std medido = 0.087 rad/s → var = 0.087² ≈ 0.00757
        #   acelerómetro → varianza pequeña estimada
        _gv = 0.087 ** 2
        _av = 0.01
        self._gyro_cov  = [_gv, 0.0, 0.0,  0.0, _gv, 0.0,  0.0, 0.0, _gv]
        self._accel_cov = [_av, 0.0, 0.0,  0.0, _av, 0.0,  0.0, 0.0, _av]

        try:
            self.ser = serial.Serial(port, 115200, timeout=1)
            self.get_logger().info(f'Conectado a {port}')
        except Exception as e:
            self.get_logger().error(f'Error abriendo {port}: {e}')
            raise

        self.pub_imu   = self.create_publisher(Imu, '/imu/data', 10)
        self.pub_accel = self.create_publisher(TwistStamped, '/imu/accel_raw', 10)

        # Encoder para el ZUPT (reposo independiente del giroscopio).
        vel_topic = self.get_parameter('velocity_topic').get_parameter_value().string_value
        self.create_subscription(Vector3Stamped, vel_topic, self._vel_cb, 10)

        self.log_counter = 0
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz

        self.get_logger().info(
            f'imu_publisher listo\n'
            f'  bias_z      = {self._bias[2]:+.6f} rad/s\n'
            f'  alpha       = {self._alpha:.2f}\n'
            f'  pitch_offset= {self.get_parameter("pitch_offset_deg").value:.2f}°\n'
            f'  roll_offset = {self.get_parameter("roll_offset_deg").value:.2f}°'
        )

    def _rotate(self, x: float, y: float, z: float):
        """Aplica la rotación de corrección de tilt al vector (x, y, z)."""
        R = self._R
        xr = R[0][0]*x + R[0][1]*y + R[0][2]*z
        yr = R[1][0]*x + R[1][1]*y + R[1][2]*z
        zr = R[2][0]*x + R[2][1]*y + R[2][2]*z
        return xr, yr, zr

    def _vel_cb(self, msg: Vector3Stamped):
        """Velocidad lineal del encoder, usada por el ZUPT."""
        self._v = msg.vector.x
        self._v_time_ns = self.get_clock().now().nanoseconds
        self._have_vel = True

    def _apply_filter(self, gx_raw: float, gy_raw: float, gz_raw: float):
        """Procesa el giroscopio para eliminar la deriva estacionaria.

        Etapas: (1) auto-calibración de arranque, (2) resta de bias,
        (3) filtro EMA, (4) adaptación de bias + banda muerta en reposo.
        """
        raw = (gx_raw, gy_raw, gz_raw)

        # (1) Auto-calibración: con el carro quieto al arrancar, promedia las
        #     primeras N muestras para fijar el bias real de este encendido.
        #     Mientras tanto, la salida es 0 (no integrar nada aún).
        if self._calib_remaining > 0:
            for i in range(3):
                self._calib_sum[i] += raw[i]
            self._calib_n += 1
            self._calib_remaining -= 1
            if self._calib_remaining == 0:
                self._bias = [s / self._calib_n for s in self._calib_sum]
                self.get_logger().info(
                    f'Auto-calibración de giroscopio completa '
                    f'(bias_z = {self._bias[2]:+.6f} rad/s)')
            return 0.0, 0.0, 0.0

        # (2) Resta de bias
        gx = gx_raw - self._bias[0]
        gy = gy_raw - self._bias[1]
        gz = gz_raw - self._bias[2]

        # (3) Filtro EMA
        if not self._filter_init:
            self._gx, self._gy, self._gz = gx, gy, gz
            self._filter_init = True
        else:
            a = self._alpha
            self._gx = a * self._gx + (1.0 - a) * gx
            self._gy = a * self._gy + (1.0 - a) * gy
            self._gz = a * self._gz + (1.0 - a) * gz

        gx, gy, gz = self._gx, self._gy, self._gz

        # (4) Detección de reposo. Preferimos el ENCODER porque es independiente
        #     del bias del giroscopio; el criterio por |gz| es frágil cuando el
        #     bias residual es del tamaño del umbral (causa el parpadeo). Si no
        #     hay dato de encoder fresco, caemos a ese criterio como respaldo.
        now_ns = self.get_clock().now().nanoseconds
        vel_fresh = self._have_vel and (now_ns - self._v_time_ns) < 500_000_000
        if vel_fresh:
            stationary = abs(self._v) < self._zupt_thresh
        else:
            stationary = (abs(gx) < self._stat_thresh and
                          abs(gy) < self._stat_thresh and
                          abs(gz) < self._stat_thresh)

        if stationary:
            # Re-aprende el bias mientras está quieto (absorbe deriva térmica).
            if self._bias_adapt:
                b = self._adapt_alpha
                for i in range(3):
                    self._bias[i] = (1.0 - b) * self._bias[i] + b * raw[i]
            # ZUPT: en reposo la velocidad angular real es 0 → salida exacta 0.
            return 0.0, 0.0, 0.0

        return gx, gy, gz

    def timer_callback(self):
        try:
            if self.ser.in_waiting == 0:
                return

            line = self.ser.readline().decode('utf-8').strip()
            if not line or 'ax' not in line:
                return

            data = json.loads(line)
            if 'ax' not in data:
                return

            stamp = self.get_clock().now().to_msg()

            # Giroscopio: bias + filtro + corrección de tilt
            gx, gy, gz = self._apply_filter(data['gx'], data['gy'], data['gz'])
            gx, gy, gz = self._rotate(gx, gy, gz)

            # Acelerómetro: corrección de tilt
            ax, ay, az = self._rotate(data['ax'], data['ay'], data['az'])

            # --- Imu estándar ---
            imu = Imu()
            imu.header.stamp    = stamp
            imu.header.frame_id = 'imu_link'

            imu.linear_acceleration.x = ax
            imu.linear_acceleration.y = ay
            imu.linear_acceleration.z = az

            imu.angular_velocity.x = gx
            imu.angular_velocity.y = gy
            imu.angular_velocity.z = gz

            roll  = math.radians(data['roll'])
            pitch = math.radians(data['pitch'])
            yaw   = math.radians(data['yaw'])
            cy, sy = math.cos(yaw * 0.5),  math.sin(yaw * 0.5)
            cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
            cr, sr = math.cos(roll * 0.5),  math.sin(roll * 0.5)
            imu.orientation.w =  cr * cp * cy + sr * sp * sy
            imu.orientation.x =  sr * cp * cy - cr * sp * sy
            imu.orientation.y =  cr * sp * cy + sr * cp * sy
            imu.orientation.z =  cr * cp * sy - sr * sp * cy

            imu.angular_velocity_covariance    = self._gyro_cov
            imu.linear_acceleration_covariance = self._accel_cov

            self.pub_imu.publish(imu)

            # --- TwistStamped auxiliar (datos crudos para debug) ---
            tw = TwistStamped()
            tw.header.stamp    = stamp
            tw.header.frame_id = 'imu_link'
            tw.twist.linear.x  = data['ax']
            tw.twist.linear.y  = data['ay']
            tw.twist.linear.z  = data['az']
            tw.twist.angular.x = roll
            tw.twist.angular.y = pitch
            tw.twist.angular.z = yaw
            self.pub_accel.publish(tw)

            self.log_counter += 1
            if self.log_counter % 100 == 0:
                self.get_logger().info(
                    f"ax={data['ax']:6.3f} ay={data['ay']:6.3f} az={data['az']:6.3f} | "
                    f"gz_raw={data['gz']:+.4f} gz_corr={gz:+.4f} | "
                    f"yaw={data['yaw']:6.1f}°")

        except json.JSONDecodeError:
            pass
        except KeyError as e:
            self.get_logger().warn(f'Clave faltante en JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error serial: {e}')

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()