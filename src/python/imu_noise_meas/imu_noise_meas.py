"""IMUノイズ測定ノード（dora経由）

静止状態のIMUデータを収集し、加速度・ジャイロの標準偏差を計算する。
EKFのプロセスノイズQ / 観測ノイズR の設定に使用。

使い方:
  1. dataflow に imu_noise_meas ノードを追加（imu_data を受信）
  2. ロボットを静止させた状態で dora start
  3. 指定サンプル数を収集したら自動終了し結果を表示
"""

import struct
import sys
import numpy as np
from dora import Node

# ImuData: timestamp, ax, ay, az, gx, gy, gz, q0, q1, q2, q3, roll, pitch, yaw
IMU_DATA_FMT = "<14d"
IMU_DATA_SIZE = struct.calcsize(IMU_DATA_FMT)  # 112

NUM_SAMPLES = 1000  # 3ms tick × 1000 = 3秒分


def main():
    node = Node("imu_noise_meas")

    samples_ax = []
    samples_ay = []
    samples_az = []
    samples_gx = []
    samples_gy = []
    samples_gz = []
    samples_pitch = []
    samples_roll = []

    print(f"IMUノイズ測定: {NUM_SAMPLES} サンプル収集中...")
    print("ロボットを静止させてください。")

    for event in node:
        if event["type"] == "INPUT" and event["id"] == "imu_data":
            raw = bytes(event["value"].to_pylist())
            if len(raw) < IMU_DATA_SIZE:
                continue

            d = struct.unpack(IMU_DATA_FMT, raw[:IMU_DATA_SIZE])
            # d: timestamp, ax, ay, az, gx, gy, gz, q0-q3, roll, pitch, yaw
            samples_ax.append(d[1])
            samples_ay.append(d[2])
            samples_az.append(d[3])
            samples_gx.append(d[4])
            samples_gy.append(d[5])
            samples_gz.append(d[6])
            samples_roll.append(d[11])
            samples_pitch.append(d[12])

            n = len(samples_ax)
            if n % 100 == 0:
                print(f"  {n}/{NUM_SAMPLES}")

            if n >= NUM_SAMPLES:
                break

    # 統計計算
    ax = np.array(samples_ax)
    ay = np.array(samples_ay)
    az = np.array(samples_az)
    gx = np.array(samples_gx)
    gy = np.array(samples_gy)
    gz = np.array(samples_gz)
    pitch = np.array(samples_pitch)
    roll = np.array(samples_roll)

    print(f"\n=== IMUノイズ測定結果 ({NUM_SAMPLES} samples) ===")
    print(f"{'':>12} {'mean':>10} {'std':>10} {'min':>10} {'max':>10}")
    print("-" * 55)
    for name, data in [("ax", ax), ("ay", ay), ("az", az),
                        ("gx", gx), ("gy", gy), ("gz", gz),
                        ("pitch", pitch), ("roll", roll)]:
        print(f"{name:>12} {np.mean(data):>+10.4f} {np.std(data):>10.6f} "
              f"{np.min(data):>+10.4f} {np.max(data):>+10.4f}")

    # EKF パラメータ推奨値
    dt = 0.003
    sigma_a = np.sqrt(np.std(ax)**2 + np.std(az)**2)  # 前方加速度のノイズ
    sigma_v_obs = np.std(gy) * 0.2484  # pitch_rate × L のノイズ寄与

    print(f"\n=== EKF パラメータ推奨値 ===")
    print(f"  σ_accel (ax,az合成): {sigma_a:.6f} [m/s²]")
    print(f"  Q_position ≈ (σ_a × dt²/2)² = {(sigma_a * dt**2 / 2)**2:.2e}")
    print(f"  Q_velocity ≈ (σ_a × dt)²    = {(sigma_a * dt)**2:.2e}")
    print(f"  R_velocity ≈ σ_v_obs²        = {sigma_v_obs**2:.2e}")


if __name__ == "__main__":
    main()
