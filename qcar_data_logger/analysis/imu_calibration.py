#!/usr/bin/env python3
"""
Analiza el CSV generado por odom_logger para calibrar el IMU del QCar.

Uso:
    python3 imu_calibration.py ~/qcar_circle_log.csv

Qué calcula:
  - Bias del giroscopio Z  : offset aditivo entre wz_imu y wz_enc
  - Factor de escala Z     : cuánto le falta / sobra al IMU
  - Bias del acelerómetro  : desviación de ax, ay en movimiento uniforme
  - Gráficas comparativas
"""
import sys
import os
import math
import pandas as pd
import matplotlib.pyplot as plt


# ── carga ──────────────────────────────────────────────────────────────────────

def load(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    required = {'wz_enc', 'wz_imu', 'vx_enc', 'ax', 'ay'}
    missing = required - set(df.columns)
    if missing:
        print(f'[ERROR] Faltan columnas: {missing}')
        sys.exit(1)
    return df


# ── cálculos de calibración ────────────────────────────────────────────────────

def calibrate(df: pd.DataFrame):
    # Filtrar filas donde el carro se mueve (descartar paradas)
    moving = df[df['vx_enc'].abs() > 0.01].copy()

    if moving.empty:
        print('[WARN] No se detectó movimiento (vx_enc > 0.01). Usando todos los datos.')
        moving = df.copy()

    # ── Giroscopio Z ──────────────────────────────────────────────────────────
    error = moving['wz_imu'] - moving['wz_enc']
    bias_z   = error.mean()
    std_z    = error.std()

    # Factor de escala: pendiente de wz_imu vs wz_enc (solo si hay variación)
    if moving['wz_enc'].std() > 1e-4:
        scale_z = (moving['wz_imu'] * moving['wz_enc']).sum() / (moving['wz_enc'] ** 2).sum()
    else:
        scale_z = float('nan')

    # ── Acelerómetro ─────────────────────────────────────────────────────────
    # En movimiento circular uniforme: ax ≈ 0, ay ≈ vx²/r  (centrípeta)
    # bias_ax = mean(ax) cuando vx es constante y la trayectoria es recta
    # Como no tenemos tramo recto, reportamos estadísticas generales
    ax_mean = moving['ax'].mean()
    ay_mean = moving['ay'].mean()
    az_mean = moving['az'].mean() if 'az' in moving else float('nan')

    # Aceleración centrípeta esperada: ay_expected = wz_enc * vx_enc
    moving['ay_expected'] = moving['wz_enc'] * moving['vx_enc']
    ay_bias = (moving['ay'] - moving['ay_expected']).mean()

    return {
        'bias_gyro_z_rad_s'  : bias_z,
        'std_gyro_z_rad_s'   : std_z,
        'bias_gyro_z_deg_s'  : math.degrees(bias_z),
        'scale_factor_gyro_z': scale_z,
        'ax_mean_m_s2'       : ax_mean,
        'ay_bias_m_s2'       : ay_bias,
        'az_mean_m_s2'       : az_mean,
        'n_samples'          : len(moving),
    }, moving


# ── reporte ────────────────────────────────────────────────────────────────────

def report(stats: dict, runs):
    sep = '─' * 52
    print(f'\n{sep}')
    print('  RESULTADOS DE CALIBRACIÓN IMU  —  QCar')
    print(sep)
    print(f'  Muestras analizadas : {stats["n_samples"]}')
    print()
    print('  GIROSCOPIO Z (yaw rate)')
    print(f'    Bias        : {stats["bias_gyro_z_rad_s"]:+.6f} rad/s  '
          f'({stats["bias_gyro_z_deg_s"]:+.4f} °/s)')
    print(f'    Desv. std   : {stats["std_gyro_z_rad_s"]:.6f} rad/s')
    if not math.isnan(stats['scale_factor_gyro_z']):
        print(f'    Factor escala: {stats["scale_factor_gyro_z"]:.6f}')
    print()
    print('  ACELERÓMETRO')
    print(f'    ax medio    : {stats["ax_mean_m_s2"]:+.4f} m/s²')
    print(f'    ay bias     : {stats["ay_bias_m_s2"]:+.4f} m/s²  '
          f'(ay_medido - ay_centrípeta_esperada)')
    print(f'    az medio    : {stats["az_mean_m_s2"]:+.4f} m/s²')
    print()
    print('  CORRECCIÓN A APLICAR EN EL NODO DE ODOMETRÍA:')
    print(f'    wz_corr = wz_imu - ({stats["bias_gyro_z_rad_s"]:+.6f})')
    print(sep)

    # Por corrida
    if 'run' in runs.columns:
        print('\n  BIAS POR CORRIDA:')
        for run, g in runs.groupby('run'):
            b = (g['wz_imu'] - g['wz_enc']).mean()
            print(f'    {run:<10} bias_z = {b:+.6f} rad/s  ({math.degrees(b):+.4f} °/s)')
        print()


# ── gráficas ───────────────────────────────────────────────────────────────────

def plot(df: pd.DataFrame, moving: pd.DataFrame):
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=False)
    fig.suptitle('Calibración IMU — QCar', fontsize=13, fontweight='bold')

    runs = df['run'].unique() if 'run' in df.columns else ['all']
    colors = plt.cm.tab10.colors

    # ── Panel 1: wz_imu vs wz_enc por corrida ──
    ax1 = axes[0]
    for i, (run, g) in enumerate(df.groupby('run') if 'run' in df.columns
                                  else [('all', df)]):
        t = g['timestamp_s'] - g['timestamp_s'].iloc[0]
        c = colors[i % len(colors)]
        ax1.plot(t, g['wz_enc'], color=c, lw=1.2, label=f'{run} enc')
        ax1.plot(t, g['wz_imu'], color=c, lw=1.2, ls='--', alpha=0.8, label=f'{run} imu')
    ax1.set_ylabel('yaw rate (rad/s)')
    ax1.set_title('wz_enc (sólido) vs wz_imu (punteado)')
    ax1.legend(fontsize=7, ncol=4)
    ax1.grid(True, alpha=0.3)

    # ── Panel 2: error wz_imu - wz_enc ──
    ax2 = axes[1]
    for i, (run, g) in enumerate(moving.groupby('run') if 'run' in moving.columns
                                  else [('all', moving)]):
        t = g['timestamp_s'] - g['timestamp_s'].iloc[0]
        err = g['wz_imu'] - g['wz_enc']
        ax2.plot(t, err, color=colors[i % len(colors)], lw=1, label=run)
    bias = (moving['wz_imu'] - moving['wz_enc']).mean()
    ax2.axhline(bias, color='red', lw=1.5, ls='--', label=f'bias={bias:+.5f} rad/s')
    ax2.set_ylabel('error (rad/s)')
    ax2.set_title('Error giroscopio Z: wz_imu − wz_enc')
    ax2.legend(fontsize=7)
    ax2.grid(True, alpha=0.3)

    # ── Panel 3: trayectoria XY ──
    ax3 = axes[2]
    for i, (run, g) in enumerate(df.groupby('run') if 'run' in df.columns
                                  else [('all', df)]):
        ax3.plot(g['x_m'], g['y_m'], color=colors[i % len(colors)],
                 lw=1.2, label=run)
        ax3.plot(g['x_m'].iloc[0], g['y_m'].iloc[0], 'o',
                 color=colors[i % len(colors)], ms=5)
    ax3.set_xlabel('x (m)')
    ax3.set_ylabel('y (m)')
    ax3.set_title('Trayectoria XY (encoder)')
    ax3.set_aspect('equal')
    ax3.legend(fontsize=7)
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()

    out = os.path.splitext(sys.argv[1])[0] + '_calibration.png'
    plt.savefig(out, dpi=150)
    print(f'  Gráfica guardada: {out}')
    plt.show()


# ── main ───────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print('Uso: python3 imu_calibration.py <ruta_del_csv>')
        sys.exit(1)

    path = sys.argv[1]
    if not os.path.exists(path):
        print(f'[ERROR] No existe: {path}')
        sys.exit(1)

    df      = load(path)
    stats, moving = calibrate(df)
    report(stats, moving)
    plot(df, moving)


if __name__ == '__main__':
    main()
