#pragma once
// IMU 取り付け回転の数学 (header-only)。
//
// 取り付け回転 R_ri = 「IMU 座標系のベクトルをロボット座標系へ写す回転」を
// RPY (deg, Z-Y-X 順: R = Rz(yaw)·Ry(pitch)·Rx(roll)) で指定し、
//   - ベクトル (accel/gyro):      v_robot = R_ri · v_imu
//   - 姿勢 quaternion:            q_world_robot = q_world_imu ⊗ q_mount⁻¹
//     (q_mount = R_ri に対応する quaternion。IMU がロボットに対して q_mount
//      だけ回して取り付けられているとき、world から見たロボットの姿勢は
//      IMU の姿勢から取り付け分を戻したもの)
// を計算する。取り付けがロボット座標系と一致なら恒等 (rpy = 0,0,0)。
//
// quaternion は (w, x, y, z) 順 (Madgwick 出力と同じ)。

#include <array>
#include <cmath>

namespace imu_mount {

struct Quat {
  double w, x, y, z;
};

// RPY [deg] (Z-Y-X: yaw→pitch→roll の順に適用) → quaternion
inline Quat RpyDegToQuat(double roll_deg, double pitch_deg, double yaw_deg) {
  constexpr double kD2R = M_PI / 180.0;
  const double r = roll_deg * kD2R * 0.5;
  const double p = pitch_deg * kD2R * 0.5;
  const double y = yaw_deg * kD2R * 0.5;
  const double cr = std::cos(r), sr = std::sin(r);
  const double cp = std::cos(p), sp = std::sin(p);
  const double cy = std::cos(y), sy = std::sin(y);
  return {
      cr * cp * cy + sr * sp * sy,
      sr * cp * cy - cr * sp * sy,
      cr * sp * cy + sr * cp * sy,
      cr * cp * sy - sr * sp * cy,
  };
}

inline Quat Conjugate(const Quat& q) { return {q.w, -q.x, -q.y, -q.z}; }

// Hamilton 積 a ⊗ b
inline Quat Multiply(const Quat& a, const Quat& b) {
  return {
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

// v_out = q で回したベクトル (q ⊗ v ⊗ q⁻¹)
inline std::array<double, 3> RotateVec(const Quat& q,
                                       const std::array<double, 3>& v) {
  // 展開形 (行列化と等価、一時 quaternion を作らない)
  const double tx = 2.0 * (q.y * v[2] - q.z * v[1]);
  const double ty = 2.0 * (q.z * v[0] - q.x * v[2]);
  const double tz = 2.0 * (q.x * v[1] - q.y * v[0]);
  return {
      v[0] + q.w * tx + (q.y * tz - q.z * ty),
      v[1] + q.w * ty + (q.z * tx - q.x * tz),
      v[2] + q.w * tz + (q.x * ty - q.y * tx),
  };
}

// quaternion → オイラー角 (Z-Y-X, rad)。取り付け補正はここではしない (純変換)。
inline void QuatToEuler(const Quat& q, double& roll, double& pitch, double& yaw) {
  const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
  const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  if (std::abs(sinp) >= 1.0) {
    pitch = std::copysign(M_PI / 2.0, sinp);  // ジンバルロック時
  } else {
    pitch = std::asin(sinp);
  }

  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace imu_mount
