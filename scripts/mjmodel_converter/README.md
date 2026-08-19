# mjmodel_converter

Fusion 360 の物理プロパティ CSV + 骨格定義 JSON から MuJoCo モデル (`sim/<model>.xml`) を生成する。
`sim/mimic_v2.xml` (手作業で URDF から変換したもの) と同じ構造の MJCF を、CAD の値から再生成できるようにしたもの。

```bash
cd scripts && uv sync
uv run mjmodel_converter/mjmodel_converter.py                       # sim/fusion_param/mimic_v2_5 → sim/mimic_v2_5.xml
uv run mjmodel_converter/mjmodel_converter.py sim/fusion_param/<robot> [-o sim/<name>.xml] [--no-validate]
```

生成後に MuJoCo で読み込んで body 数・質量・keyframe を表示する (`--no-validate` で省略)。
シミュレーションで使うには `dataflow_mimic_sim.yaml` (実機 `dataflow_mimic.yaml` のシム版) を起動し、`mujoco_backend` を `MUJOCO_MODEL=scene.xml` 付きで立ち上げる (手順は yaml 冒頭コメント)。`sim/scene.xml` は**手書き**で、床 (無限グリッド)・光源・カメラを持ち `mimic_v2_5.xml` を `<include>` する。生成側 (ロボット XML) は床・光源・カメラを一切持たない (単体で読むと床が無い)。

## 入力

1 ロボット = 1 ディレクトリ (`sim/fusion_param/<robot>/`)。

| ファイル | 役割 |
|---|---|
| `<part>.csv` | Fusion 360 の「物理」タブ書き出し (タブ区切り)。質量・重心・**重心まわり慣性テンソル**を使う。`Mass`, `Center of Mass`, `Moment of Inertia at Center of Mass (g mm^2)` を読む。`at Origin` ブロックは検証にだけ使う |
| `model.json` | CSV に無い情報: body の親子関係・関節位置 (`pos`) / 軸 / 可動域 / armature / damping・メッシュ名・衝突形状・IMU site・センサ・moteus 利得。各 body の `"part"` が `<part>.csv` を指す |
| `robot_config/*.json` | `model.json` の `"robot_config"` が指す。**軸名と順序** (actuator 名・順番)、`initial_position` (keyframe `standing`)、`torque_limit` (forcerange) の正本。二重管理しない。**駆動軸の joint 名も軸名そのもの**になる (`body.axis`; stabilizer の posture IK / mujoco_backend はそれで引く) |

### 単位・座標系の前提

- CSV は g / mm / g·mm² → kg / m / kg·m² に変換する (他の単位表記も `MASS_UNIT` 等に追加すれば読める)。
- `Center of Mass` は Fusion の**部品原点**基準。MuJoCo の body 原点 = 関節の回転中心なので、**Fusion 側で部品原点を関節中心に置いておく**こと (v2 以来の前提。`wheel` の CoM が車軸上に乗っているかで確認できる)。
- 慣性テンソルの非対角成分 (Ixy, Ixz, Iyz) は Fusion の値を**そのまま** `fullinertia` に渡す。Fusion の出力はテンソル成分そのもの (−∫xy dm) で、スクリプトが `at Origin` ブロックと平行軸の定理で突き合わせて検証する。規約が違えば `parallel-axis mismatch` 警告が出る (mimic_v2_5 の CSV では全部一致)。
- 関節位置 (`pos`) は CSV に含まれない。`model.json` に書く。mimic_v2_5 は関節オフセット・メッシュ・衝突形状が v2 と同一で、変わったのは質量・重心・慣性 (CSV 側) のみ。

### `model.json` の主なキー

```jsonc
{
  "model": "mimic_v2_5",                        // 出力 sim/<model>.xml と <mujoco model=...>
  "robot_config": "robot_config/mimic_v2_5.json",
  "meshdir": "meshes/", "timestep": 0.001,
  "bodies": [                                   // 親が先に来る順で並べる (qpos の並びもこの順)
    {"name": "base_link", "part": "MIM2_5-A000", "mesh": "base_link", "parent": null,
     "pos": [0,0,0.5], "joint": {"type": "free"}, "collision": {"type": "mesh"},
     "sites": [{"name": "imu", "size": "0.01", "rgba": "1 0 0 1", "type": "box"}]},
    {"name": "wheel_R", "part": "MIM2_5-A300", "mesh": "wheel_R", "parent": "lower_link_R",
     "pos": [0,-0.04985,-0.18],
     "joint": {"axis": [0,1,0], "armature": 0.008361, "damping": 0.047421},   // 名前は axis から
     "axis": "wheel_r",                         // robot_config の軸名 = joint 名 = actuator 名
     "collision": {"type": "cylinder", "radius": 0.07795, "half_length": 0.024361, "euler": [1.5708,0,0]}}
  ],
  "contact_exclude": [["upper_link_L","wheel_L"], ["upper_link_R","wheel_R"]],
  "sensors": [{"tag": "gyro", "name": "gyro", "site": "imu"}, ...],   // tag = MJCF 要素名、残りは属性
  "actuators": {
    "moteus_gains_per_rev": {"hip_pitch_r": {"kp": 600, "kd": 6.0}, "wheel_r": {"kp": 140, "kd": 0.5}, ...},
    "initial_mode": {"hip_pitch_r": "position", "wheel_r": "velocity", ...}
  }
}
```

`_` で始まるキーはコメント扱い。`collision` は `mesh` / `cylinder` / `sphere` / `null` (無し)。
`cylinder` 衝突を持つ body は接地体 (ホイール) とみなし、keyframe `standing` の base 高さを
「その姿勢で円筒の最下点が z=0」になるよう順運動学で決める (`base_height_margin` で上乗せ可)。

## 倒立点の補正 (`com_comp.py`)

CAD の質量特性だけだと、配線・バッテリー・搭載物など CAD に無い質量のぶん倒立点 (全体重心が車軸の真上に
来る base pitch) が実機とずれる。実機で平衡する pitch を実測し (立位 = angle_pid の `kTargetPitch`、必要なら
脚を伸ばした姿勢でも)、それを正として body の重心を body 座標 **x 方向にだけ**動かして合わせる。
**同定できる未知数の数 = 実測姿勢の数**: 立位だけなら `base_link` の x、立位 + 上げ姿勢なら `base_link` と
`upper_link` (左右同値) の x。base だけの補正は姿勢を変えると倒立点がずれる (立位 0.165 → 最高点 0.181 rad) ので
脚にも振る。

```bash
cd scripts
uv run mjmodel_converter/com_comp.py 0.165 --at 0.08:0.181     # 立位 0.165 rad、重心 +0.08 m の姿勢で 0.181 rad
uv run mjmodel_converter/com_comp.py 0.165 --pose 0.6056,-1.1979:0.181   # 姿勢を関節角で指定 (stabilizer の posture ログ)
uv run mjmodel_converter/com_comp.py 0.165                     # 立位だけ → base_link のみ
uv run mjmodel_converter/com_comp.py ... --bodies base_link,lower_link    # 補正する body 群を変える (prefix は左右に展開)
uv run mjmodel_converter/com_comp.py ... --dry-run             # 計算だけ
uv run mjmodel_converter/com_comp.py --reset                   # 全 com_offset を外して CAD の値に戻す
```

- `model.json` の該当 body に `"com_offset": [dx, 0, 0]` と根拠コメント `"_com_offset"` (実測姿勢・再実行コマンド) が
  書かれる (他の行・整形は触らない)。converter はこれを CSV の重心に足すだけ。
- `--at RISE:PITCH` は「実測時にロボットが使っていたモデル (= いまの model.json)」の IK で関節角に換算する。
  補正後に同じ `--at` を再実行すると別の姿勢になるので、**再実行は `_com_offset` に記録された `--pose` 形式**で。
- converter は毎回 `balance pitch at standing pose` を表示するので、CAD 更新後に補正量が不自然に膨らんでいないか分かる。
- IMU 取り付け角や関節原点のズレもこの補正に吸収される。
- pitch の符号は `mujoco_backend.quat_to_euler` (= stabilizer が見る `ImuData.pitch`) と同じで、`kTargetPitch` をそのまま渡せる。
- mimic_v2_5 (2026-08-20): CAD 倒立点 立位 +0.1416 / +0.08 m 姿勢 +0.1521 rad → 実測 +0.165 / +0.181 rad。
  `base_link` x **−8.64 mm**、`upper_link_R/L` x **−9.08 mm** で両姿勢とも一致。

## 出力の互換性

`mujoco_backend` / `stabilizer` が参照する名前をそのまま出す: joint 名と actuator 名 (どちらも = robot_config 軸名)、
keyframe `standing`、センサ `gyro` / `accel` / `linvel` / `imu_pos` / `imu_quat`。
actuator は `general` で、`mujoco_backend` が `biasprm` から `base_kp = -biasprm[1]`, `base_kv = -biasprm[2]` を読んで
実行時に POSITION / VELOCITY / TORQUE / OFF を切り替える。`initial_mode: velocity` (ホイール) は v2 と同様 `base_kp = 0` になる。

## sysid 版 (`mimic_v2_sysid.xml` 相当) について

未対応。free joint 無し + jointpos/jointvel センサの変種が要るときは、生成した XML をもとに作るか、
`model.json` に `"joint": {"type": "free"}` を外した variant を書く (センサは `sensors` に `jointpos` / `jointvel` を列挙)。
