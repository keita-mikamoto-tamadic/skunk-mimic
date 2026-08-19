# posture_ik — Pinocchio による重心高さ IK

`stabilizer` の姿勢変更機能。倒立バランス制御 (angle_pid) を回したまま、**倒立点を変えずに**
hip / knee を動かして全体重心の高さを変える。右スティック上下 (`DriveCommand.height`) が入力。

実装: [posture_ik.hpp](posture_ik.hpp) / [posture_ik.cpp](posture_ik.cpp)、呼び出し側は
[angle_pid_controller.cpp](angle_pid_controller.cpp)。

---

## 1. 問題設定

車輪倒立ロボットの静的な倒立点は「全体重心が車軸 (接地点) の真上に来るベースのピッチ角」で決まる。
いま angle_pid は固定の `kTargetPitch` (実測 0.165 rad) を倒立点として使っている。

脚 (hip, knee) を動かすと全体重心が動くので、何も考えずに動かすと倒立点がずれ、
外側ループ (速度 PI) が新しい倒立点を探し直す間、ロボットが前後に流れる。
これを避けるために、次の 2 拘束を満たす hip / knee を毎周期求める:

| # | 拘束 | 意味 |
|---|---|---|
| (1) | ピッチ = `kTargetPitch` のとき、全体重心の**水平位置** = 車軸の水平位置 | 倒立点が変わらない |
| (2) | 全体重心の車軸からの**高さ** = 指令値 h | 高さが指令に追従する |

未知数は hip と knee の 2 つ (左右は対称に同じ値)。拘束 2 本なので一意に決まる。

記号:
- θ … ベースのピッチ (ImuData.pitch と同じ符号: ベースを +y 軸まわりに回した角)
- q_hip, q_knee … 脚角 (左右同じ)
- c(q) … 全体重心 (world)、a(q) … 車軸中心 (左右ホイール関節原点の中点、world)
- 残差 r(q) = [ c_x − a_x, (c_z − a_z) − h ]ᵀ  … これを 0 にする

## 2. 解法 — Newton 法を 1 周期 1 反復

h はスティックで秒オーダーでしか変わらない (既定 `kMaxComHeightVel` = 0.03 m/s) ので、
毎制御周期 (3 ms) に Newton を 1 反復ずつ進めれば十分追従する (warm start)。
検証では 0.03 m/s のランプに対し残差 0.09 mm (= 1 tick 分の遅れ) で追従した。

```
J = ∂r/∂[q_hip, q_knee]   (2x2)
dq = −(JᵀJ + λI)⁻¹ Jᵀ r    (λ = 1e-6 の減衰付き最小二乗。特異姿勢で暴れないため)
|dq| > max_step なら縮める (既定 0.02 rad/反復)
q ← clamp(q + dq, 関節リミット)
```

J は Pinocchio が出す重心ヤコビアンと車軸 (ホイール関節) のヤコビアンの差で作る:

```
J_rel = J_com − ½ (J_wheel_R + J_wheel_L)        (3 x nv, world 座標)
J(0, 0) = J_rel[x, v_hipR]  + J_rel[x, v_hipL]    ← 左右の列を足す = 対称拘束
J(0, 1) = J_rel[x, v_kneeR] + J_rel[x, v_kneeL]
J(1, 0) = J_rel[z, v_hipR]  + J_rel[z, v_hipL]
J(1, 1) = J_rel[z, v_kneeR] + J_rel[z, v_kneeL]
```

ホイール角は車軸位置に影響しないので q の該当成分は 0 のままでよい。

## 3. Pinocchio

### モデル読み込み — MJCF を直接パース

```cpp
#include <pinocchio/parsers/mjcf.hpp>
pinocchio::Model model;
pinocchio::mjcf::buildModel(path, pinocchio::JointModelFreeFlyer(), model);
pinocchio::Data data(model);
```

- `sim/mimic_v2_5.xml` (scripts/mjmodel_converter が生成) をそのまま読む。
- ルートは `JointModelFreeFlyer` を明示する (MJCF の `<joint type="free"/>` があっても引数で指定)。
- MJCF の `fullinertia` / `<default class>` / `<keyframe>` (→ `model.referenceConfigurations["standing"]`) は
  読める。`<actuator>` / `<sensor>` は無視される。
- 注意: MJCF で `<body name="base_link" pos="0 0 0.5">` にしてあると、Pinocchio はこの `pos` を
  free flyer 関節の**固定オフセット**として扱う (MuJoCo は qpos0 の初期値扱い)。world 座標の絶対値は
  0.5 m ずれるが、本 IK は重心と車軸の**差**しか使わないので影響しない。

### 関節の解決

**MJCF の駆動 joint 名 = robot_config の軸名** (`hip_pitch_r`, `knee_r`, `wheel_r`, …) という約束で、
mjmodel_converter がそう生成する (model.json の `body.axis`)。なので余計な対応表は持たず、
robot_config の `axes[i].name` でそのまま引く:

```cpp
model.existJointName(name); auto j = model.getJointId(name);
model.idx_qs[j]   // q ベクトル内の先頭インデックス (1 自由度関節なら 1 要素)
model.idx_vs[j]   // v (速度/ヤコビアン列) のインデックス
model.nqs[j], model.nvs[j]   // 1 自由度か確認
```

### 配置ベクトル q のレイアウト

```
q = [ x y z | qx qy qz qw | joint1 joint2 ... ]      (nq = 7 + 関節数)
v = [ vx vy vz | wx wy wz | ... ]                    (nv = 6 + 関節数)
```

- **Pinocchio の四元数は (x, y, z, w)** の順。MuJoCo は (w, x, y, z)。
- ピッチ θ (= +y 軸まわり) のベース姿勢: `q[3]=0, q[4]=sin(θ/2), q[5]=0, q[6]=cos(θ/2)`。
  これは mujoco_backend.quat_to_euler / imu_node が報告する `ImuData.pitch` と同じ回転。
- 並進 q[0..2] は 0 でよい (相対量しか使わない)。

### 重心と車軸

```cpp
#include <pinocchio/algorithm/center-of-mass.hpp>
pinocchio::centerOfMass(model, data, q, /*computeSubtreeComs=*/false);
data.com[0]                       // 全体重心 (world) — 順運動学も内部で回る
data.oMi[j_wheel].translation()   // ホイール関節原点 (= 車軸) の world 位置
pinocchio::computeTotalMass(model)
```

`data.oMi[j]` は関節 j のフレーム (その関節の回転中心) の world 配置。MJCF の body 原点 =
関節原点という前提 (mjmodel_converter の約束) なので、ホイール関節の原点がそのまま車軸中心。

### ヤコビアン

```cpp
#include <pinocchio/algorithm/jacobian.hpp>
pinocchio::jacobianCenterOfMass(model, data, q, false);   // data.Jcom (3 x nv, world)
// ↑ 内部で順運動学・重心・全関節ヤコビアン (data.J) も計算する
Eigen::MatrixXd Jw(6, model.nv); Jw.setZero();
pinocchio::getJointJacobian(model, data, j_wheel, pinocchio::LOCAL_WORLD_ALIGNED, Jw);
// 上 3 行が並進、world 軸向き (LOCAL_WORLD_ALIGNED) — Jcom と同じ座標系で引き算できる
```

- `getJointJacobian` は `computeJointJacobians` 済みの data が前提。`jacobianCenterOfMass` が
  それを兼ねるので、順序は「jacobianCenterOfMass → getJointJacobian」。
- 出力行列は呼び出し前に `setZero()` しておく (書き込まれない列が残るため)。
- 参照フレーム: `LOCAL` (関節フレーム基準) ではなく **`LOCAL_WORLD_ALIGNED`** (原点は関節、向きは world)
  を使う。重心ヤコビアンが world 基準なので揃える。

### 関節リミット

```cpp
model.lowerPositionLimit[idx_q], model.upperPositionLimit[idx_q]
```

MJCF に `range` の無い関節 (knee) は ±∞ や巨大値が入るので、`|limit| > 1e6` は「無制限」として扱う。
hip は model.json の range (0.6〜1.6 rad) がそのまま来る。

### RT 周期で使うときの注意

- `Model` / `Data` は起動時に 1 回作り、使い回す (Data のアロケーションは構築時のみ)。
- 作業用の `Eigen::MatrixXd` (Jcom, Jw) もメンバに持って `setZero()` で再利用する。
- 計測: 1 反復 (Step) ≈ 45 µs (x86 PC)。3 ms 周期に対して十分小さい。

## 4. angle_pid への組み込み

- コンストラクタ: robot_config に `model_mjcf` があり、hip_pitch_*/knee_*/wheel_* の軸があれば
  `PostureIk::Load` (joint 名 = 軸名) → 有効。初期姿勢 (`initial_position`) での重心高さ h0 を計算し、指令範囲を
  `[h0, h0 + kMaxComRise]` にする (v2_5 は 0.08 m で hip が可動域下限 0.6 rad に当たる)。
- `SetDriveCommand`: `height` (正規化 −1..1) × `kMaxComHeightVel` [m/s] をレートとして保持。
- `Compute` (毎 tick): h_cmd を積分・クランプ → `Step(kTargetPitch, h_cmd, hip, knee)` 1 回 →
  hip/knee の POSITION 指令 (左右同値)。外側ループの `angle_offset` は過渡補正なので IK の基準には
  使わない (幾何は `kTargetPitch` で固定)。
- `Reset` (RUN 開始): READY で initial_position に来ているので h_cmd = h0、脚角 = 初期値に戻す。
- 無効時 (model 未指定 / 関節が引けない) は従来どおり hip/knee = `initial_position` 固定。
  起動ログに `posture IK: enabled/disabled (...)` が出る。

設定 (robot_config/mimic_v2_5.json) は 1 行だけ:
```json
"model_mjcf": "sim/mimic_v2_5.xml",
```
MJCF 側が robot_config に合わせる (joint 名 = 軸名) ので、軸ごとの追加パラメータは無い。
mujoco_backend も同じ理由で joint 名を持たず、actuator (名前 = 軸名) の transmission から joint を引く。

## 6. ビルド

`find_package(pinocchio)`。
pip の `pin` (cmeel ビルド) は `~/.local/lib/python3.*/site-packages/cmeel.prefix`に C++ ライブラリ・ヘッダ・CMake config を置くので、stabilizer の CMakeLists がそこを `CMAKE_PREFIX_PATH` に
足す (バイナリに RUNPATH が入る)。Jetson も `pip install pin` で同じ。robotpkg / ソースビルドなら通常の
探索で見つかる。別の場所なら `-DCMAKE_PREFIX_PATH=<prefix>`。
