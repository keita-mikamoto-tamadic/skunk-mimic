#include "posture_ik.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <pinocchio/parsers/mjcf.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

namespace {

bool ResolveJoint(const pinocchio::Model& model, const std::string& name,
                  pinocchio::JointIndex& out, std::string* err) {
    if (!model.existJointName(name)) {
        if (err) *err = "joint '" + name + "' not found in MJCF model";
        return false;
    }
    out = model.getJointId(name);
    return true;
}

// 1 自由度関節の位置リミット。MJCF に range が無い関節は Pinocchio が ±∞ や巨大値を
// 入れてくるので、そのまま「無制限」として扱う
void JointLimits(const pinocchio::Model& model, pinocchio::JointIndex j, double& lo, double& hi) {
    const auto idx = model.idx_qs[j];
    lo = model.lowerPositionLimit[idx];
    hi = model.upperPositionLimit[idx];
    if (!std::isfinite(lo) || std::abs(lo) > 1e6) lo = -std::numeric_limits<double>::infinity();
    if (!std::isfinite(hi) || std::abs(hi) > 1e6) hi = std::numeric_limits<double>::infinity();
}

}  // namespace

bool PostureIk::Load(const Config& cfg, std::string* err) {
    loaded_ = false;
    try {
        model_ = pinocchio::Model();
        pinocchio::mjcf::buildModel(cfg.mjcf_path, pinocchio::JointModelFreeFlyer(), model_);
    } catch (const std::exception& e) {
        if (err) *err = std::string("failed to parse MJCF '") + cfg.mjcf_path + "': " + e.what();
        return false;
    }
    if (!ResolveJoint(model_, cfg.hip_r, j_hip_r_, err)) return false;
    if (!ResolveJoint(model_, cfg.knee_r, j_knee_r_, err)) return false;
    if (!ResolveJoint(model_, cfg.hip_l, j_hip_l_, err)) return false;
    if (!ResolveJoint(model_, cfg.knee_l, j_knee_l_, err)) return false;
    if (!ResolveJoint(model_, cfg.wheel_r, j_wheel_r_, err)) return false;
    if (!ResolveJoint(model_, cfg.wheel_l, j_wheel_l_, err)) return false;
    for (auto j : {j_hip_r_, j_knee_r_, j_hip_l_, j_knee_l_}) {
        if (model_.nqs[j] != 1 || model_.nvs[j] != 1) {
            if (err) *err = "joint '" + model_.names[j] + "' is not a 1-DoF joint";
            return false;
        }
    }
    // 左右でリミットが違う場合は厳しい方
    double lo_r, hi_r, lo_l, hi_l;
    JointLimits(model_, j_hip_r_, lo_r, hi_r);
    JointLimits(model_, j_hip_l_, lo_l, hi_l);
    hip_min_ = std::max(lo_r, lo_l); hip_max_ = std::min(hi_r, hi_l);
    JointLimits(model_, j_knee_r_, lo_r, hi_r);
    JointLimits(model_, j_knee_l_, lo_l, hi_l);
    knee_min_ = std::max(lo_r, lo_l); knee_max_ = std::min(hi_r, hi_l);

    data_ = pinocchio::Data(model_);
    q_ = pinocchio::neutral(model_);
    total_mass_ = pinocchio::computeTotalMass(model_);
    J_com_.setZero(3, model_.nv);
    J_wr_.setZero(6, model_.nv);
    J_wl_.setZero(6, model_.nv);
    loaded_ = true;
    return true;
}

void PostureIk::SetQ(double hip, double knee, double pitch) {
    // free flyer: 並進 0、回転 = +y 軸まわり pitch (Pinocchio の quat は x,y,z,w)
    q_.setZero();
    q_[0] = q_[1] = q_[2] = 0.0;
    q_[3] = 0.0;
    q_[4] = std::sin(pitch / 2.0);
    q_[5] = 0.0;
    q_[6] = std::cos(pitch / 2.0);
    q_[model_.idx_qs[j_hip_r_]] = hip;
    q_[model_.idx_qs[j_knee_r_]] = knee;
    q_[model_.idx_qs[j_hip_l_]] = hip;
    q_[model_.idx_qs[j_knee_l_]] = knee;
    // ホイール角は車軸位置に影響しないので 0 のまま
}

void PostureIk::Evaluate(Eigen::Vector2d& com_rel, Eigen::Matrix2d& J) {
    // 重心ヤコビアン (内部で順運動学 + 重心も計算する)。LOCAL_WORLD_ALIGNED 相当 (world 基準)
    pinocchio::jacobianCenterOfMass(model_, data_, q_, /*computeSubtreeComs=*/false);
    J_com_ = data_.Jcom;
    // 関節ヤコビアン (jacobianCenterOfMass が computeJointJacobians 相当を済ませている)
    J_wr_.setZero();
    J_wl_.setZero();
    pinocchio::getJointJacobian(model_, data_, j_wheel_r_, pinocchio::LOCAL_WORLD_ALIGNED, J_wr_);
    pinocchio::getJointJacobian(model_, data_, j_wheel_l_, pinocchio::LOCAL_WORLD_ALIGNED, J_wl_);

    const Eigen::Vector3d com = data_.com[0];
    const Eigen::Vector3d axle = 0.5 * (data_.oMi[j_wheel_r_].translation() + data_.oMi[j_wheel_l_].translation());
    com_rel << com.x() - axle.x(),   // [0] = 水平オフセット
               com.z() - axle.z();   // [1] = 高さ

    // d(com - axle)/dq の x,z 行、hip/knee 列 (左右を足して対称拘束)
    const auto vh_r = model_.idx_vs[j_hip_r_], vk_r = model_.idx_vs[j_knee_r_];
    const auto vh_l = model_.idx_vs[j_hip_l_], vk_l = model_.idx_vs[j_knee_l_];
    auto rel = [&](int row, int col) {
        return J_com_(row, col) - 0.5 * (J_wr_(row, col) + J_wl_(row, col));
    };
    J(0, 0) = rel(0, vh_r) + rel(0, vh_l);   // d x / d hip
    J(0, 1) = rel(0, vk_r) + rel(0, vk_l);   // d x / d knee
    J(1, 0) = rel(2, vh_r) + rel(2, vh_l);   // d z / d hip
    J(1, 1) = rel(2, vk_r) + rel(2, vk_l);   // d z / d knee
}

Eigen::Vector2d PostureIk::ComRelAxle(double hip, double knee, double pitch) {
    SetQ(hip, knee, pitch);
    pinocchio::centerOfMass(model_, data_, q_, /*computeSubtreeComs=*/false);
    const Eigen::Vector3d com = data_.com[0];
    const Eigen::Vector3d axle = 0.5 * (data_.oMi[j_wheel_r_].translation() + data_.oMi[j_wheel_l_].translation());
    return {com.x() - axle.x(),    // [0] = 水平オフセット
            com.z() - axle.z()};   // [1] = 高さ
}

double PostureIk::Step(double pitch, double h_target, double& hip, double& knee, double max_step) {
    SetQ(hip, knee, pitch);
    Eigen::Vector2d com_rel;
    Eigen::Matrix2d J;
    Evaluate(com_rel, J);
    const Eigen::Vector2d r(com_rel[0], com_rel[1] - h_target);

    // 2x2 を減衰付きで解く (特異姿勢近くで暴れないように Levenberg-Marquardt 風)
    constexpr double kDamping = 1e-6;
    const Eigen::Matrix2d JtJ = J.transpose() * J + kDamping * Eigen::Matrix2d::Identity();
    Eigen::Vector2d dq = -JtJ.ldlt().solve(J.transpose() * r);
    const double n = dq.norm();
    if (n > max_step) dq *= max_step / n;

    hip = std::clamp(hip + dq[0], hip_min_, hip_max_);
    knee = std::clamp(knee + dq[1], knee_min_, knee_max_);
    return r.norm();
}
