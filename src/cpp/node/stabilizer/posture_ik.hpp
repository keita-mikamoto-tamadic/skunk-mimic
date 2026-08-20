#pragma once
// posture_ik — 倒立点を変えずに重心高さを変える脚 IK (Pinocchio)。
//
// 問題設定: ベースのピッチを目標倒立角 θ (angle_pid の kTargetPitch) に固定したとき、
//   (1) 全体重心が車軸 (左右ホイール関節原点の中点) の真上にある   ← 倒立点が変わらない
//   (2) 全体重心の車軸からの高さ = h (指令値)
// を満たす hip / knee 角 (左右対称の 2 未知数) を求める。未知数 2 / 拘束 2 で一意。
// 「ベースを真上に動かす」のではなく「重心を真上に動かす」のがポイント — 脚リンクの
// 質量 (全体の約半分) が膝の曲げ伸ばしで前後に動くので、ベースだけ上げると倒立点がずれる。
//
// モデルは robot_config の model_mjcf (sim/<model>.xml。mjmodel_converter が生成し
// com_comp.py の重心補正も入っている) を Pinocchio の MJCF パーサで読む。MuJoCo と
// 同じファイルなので質量パラメータの二重管理が無い。
//
// 毎制御周期 (3 ms) に Newton 1 反復ずつ進める前提 (h はスティックでゆっくりしか
// 動かないので warm start で十分収束する)。ヤコビアンは Pinocchio の重心ヤコビアンと
// ホイール関節のヤコビアンから作る (2x2、左右の列を足して対称拘束)。
//
// ピッチの符号は mujoco_backend.quat_to_euler / imu_node の ImuData.pitch と同じ
// (ベースを +y 軸まわりに回した角)。

#include <string>
#include <Eigen/Core>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

class PostureIk {
public:
    struct Config {
        std::string mjcf_path;                 // 解決済みのファイルパス
        std::string hip_r, knee_r, hip_l, knee_l;   // MJCF joint 名 (= robot_config の軸名)
        std::string wheel_r, wheel_l;               // 同上 (関節原点 = 車軸)
    };

    // モデルを読み込んで関節を解決する。失敗時 false + err
    bool Load(const Config& cfg, std::string* err);
    bool loaded() const { return loaded_; }

    // 脚角 (左右同じ) とピッチ θ での、全体重心の車軸からのオフセット [m]。
    // [0] = 水平 (world x。0 なら倒立点)、[1] = 高さ (world z)。
    // 2 成分ベクトルなので .x()/.y() ではなく [0]/[1] でアクセスする
    // (.y() が空間の y (ロボット左右) と紛らわしいため)
    Eigen::Vector2d ComRelAxle(double hip, double knee, double pitch);

    // Newton 1 反復: 残差 r = ComRelAxle - [0, h_target] を減らす方向に hip/knee を更新し、
    // 更新前の残差ノルム [m] を返す。1 反復のステップ幅は max_step [rad] でクランプ、
    // 結果はモデルの関節リミットでクランプする
    double Step(double pitch, double h_target, double& hip, double& knee, double max_step = 0.05);

    // モデルから読んだ関節リミット (無ければ ±inf)
    double hip_min() const { return hip_min_; }
    double hip_max() const { return hip_max_; }
    double knee_min() const { return knee_min_; }
    double knee_max() const { return knee_max_; }
    double total_mass() const { return total_mass_; }

private:
    void SetQ(double hip, double knee, double pitch);
    // 現在の q_ で順運動学 + 重心 + ヤコビアンを計算し、残差と 2x2 ヤコビアンを返す
    void Evaluate(Eigen::Vector2d& com_rel, Eigen::Matrix2d& J);

    bool loaded_ = false;
    pinocchio::Model model_;
    pinocchio::Data data_;
    Eigen::VectorXd q_;
    pinocchio::JointIndex j_hip_r_ = 0, j_knee_r_ = 0, j_hip_l_ = 0, j_knee_l_ = 0;
    pinocchio::JointIndex j_wheel_r_ = 0, j_wheel_l_ = 0;
    double hip_min_ = -1e9, hip_max_ = 1e9, knee_min_ = -1e9, knee_max_ = 1e9;
    double total_mass_ = 0.0;
    Eigen::MatrixXd J_com_, J_wr_, J_wl_;   // 作業用 (3xnv, 6xnv, 6xnv)
};
